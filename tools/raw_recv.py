#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MIC16_ADC123_RAW 上位机：接收 16 路原始直通并落盘。

帧格式（38 字节，小端）：
    [0]      0xAA
    [1]      0x55
    [2..3]   seq   u16
    [4..35]  16 x u16 原始码值 0..4095
    [36..37] CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF)，覆盖 [2..35]

用法：
    python raw_recv.py COM5
    python raw_recv.py COM5 --save raw.bin
    python raw_recv.py COM5 --save raw.bin --csv raw.csv --seconds 30
    python raw_recv.py COM5 --plot

依赖：
    pip install pyserial
    pip install pyqtgraph          # 只有 --plot 需要
"""

import argparse
import collections
import struct
import sys
import time
import wave

FRAME_LEN = 38
SYNC = b"\xAA\x55"
CH = 16
VREF_MV = 3300.0
ADC_FULL = 4096.0
HIST = 2000          # 绘图窗口：2000 帧 = 1 s @ 2000 fps


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class Parser:
    """字节流 -> 帧。找到 AA55 且 CRC 对就收，天然能从任意位置重新同步。"""

    def __init__(self):
        self.buf = bytearray()
        self.good = 0
        self.crc_err = 0
        self.skipped = 0
        self.last_seq = None
        self.lost = 0

    def feed(self, data):
        self.buf += data
        out = []

        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # 末尾的 0xAA 可能是下一帧的开头，留着
                keep = 1 if self.buf[-1:] == b"\xAA" else 0
                self.skipped += max(0, len(self.buf) - keep)
                self.buf = self.buf[-keep:] if keep else bytearray()
                break

            if i > 0:
                self.skipped += i
                self.buf = self.buf[i:]

            if len(self.buf) < FRAME_LEN:
                break

            frame = bytes(self.buf[:FRAME_LEN])
            crc_rx = struct.unpack_from("<H", frame, 36)[0]

            if crc16_ccitt(frame[2:36]) != crc_rx:
                self.crc_err += 1
                self.skipped += 1
                self.buf = self.buf[1:]      # 假帧头，只跳 1 字节
                continue

            seq = struct.unpack_from("<H", frame, 2)[0]
            samples = struct.unpack_from("<16H", frame, 4)

            if self.last_seq is not None:
                gap = (seq - self.last_seq - 1) & 0xFFFF
                if 0 < gap < 1000:
                    self.lost += gap
            self.last_seq = seq
            self.good += 1

            out.append((seq, samples))
            self.buf = self.buf[FRAME_LEN:]

        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="COM5 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--save", metavar="BIN", help="每帧 32 字节的原始负载落盘")
    ap.add_argument("--csv", metavar="CSV", help="同时写 CSV（seq + 16 列）")
    ap.add_argument("--seconds", type=float, default=0, help="采多久退出，0=一直采")
    ap.add_argument("--plot", action="store_true", help="实时画 16 路（需要 pyqtgraph）")
    ap.add_argument("--text", action="store_true", help="在终端里实时滚动打印 16 路码值（10 Hz）")
    ap.add_argument("--every", type=int, default=0,
                    help="配合 --text：每 N 帧打印一行（0=按 10Hz；100=约 180 行/秒）")
    ap.add_argument("--mv", action="store_true", help="CSV 里换算成 mV")
    ap.add_argument("--fs", type=int, default=18000,
                    help="采样率，只用于 WAV 头（默认 18000，要和板子 mic.h 的 MIC_FS_HZ 一致）")
    ap.add_argument("--start", action="store_true",
                    help="先向板子发 c 启动数据流，再开始接收")
    ap.add_argument("--png", metavar="PNG", help="把采集到的波形画成图片（16 路叠图）")
    ap.add_argument("--png-ms", type=float, default=20, help="PNG 显示多少毫秒（默认 20）")
    ap.add_argument("--wav", metavar="WAV", help="把数据存成 WAV（可直接播放）")
    ap.add_argument("--wav-ch", type=int, default=0,
                    help="存单通道 WAV 的通道号 1..16；0=16 声道")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("需要 pyserial：pip install pyserial")

    if args.start:
        try:
            sc = serial.Serial(args.port, 115200, timeout=0.3)
            time.sleep(0.3)
            sc.write(b"c")
            time.sleep(0.4)
            sc.close()
            print("已向板子发送 c（启动数据流）")
            time.sleep(0.5)
        except Exception as e:
            print("发送 c 失败:", e)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print("")
        print("打不开串口 %s：%s" % (args.port, e))
        print("  1) COM 号对不对？设备管理器 -> 端口 (COM 和 LPT)")
        print("  2) 是不是被串口助手占用了？先关掉它")
        print("  3) 板子插好了吗？有没有出现 'USB 串行设备 (COMx)'")
        sys.exit(2)
    ser.reset_input_buffer()

    fb = open(args.save, "wb") if args.save else None
    fc = open(args.csv, "w", encoding="utf-8") if args.csv else None
    if fc:
        unit = "mV" if args.mv else "code"
        fc.write("seq," + ",".join("ch%02d_%s" % (i + 1, unit) for i in range(CH)) + "\n")

    wf = None
    if args.wav:
        wf = wave.open(args.wav, "wb")
        wf.setnchannels(1 if args.wav_ch else CH)
        wf.setsampwidth(2)
        wf.setframerate(args.fs)

    hist = [collections.deque(maxlen=HIST) for _ in range(CH)]
    app = None
    curves = None
    win = None

    if args.plot:
        try:
            import pyqtgraph as pg
            from pyqtgraph.Qt import QtWidgets
        except ImportError:
            sys.exit("--plot 需要：pip install pyqtgraph")

        app = QtWidgets.QApplication([])
        win = pg.GraphicsLayoutWidget(title="MIC16 stream")
        plot = win.addPlot()
        plot.setLabel("bottom", "frame (1 s window)")
        plot.setLabel("left", "ADC code")
        plot.showGrid(x=True, y=True, alpha=0.3)
        colors = [pg.intColor(i, hues=CH) for i in range(CH)]
        curves = [plot.plot(pen=pg.mkPen(colors[i])) for i in range(CH)]
        win.show()

    if args.text:
        print("  time      " + " ".join("ch%02d" % (i+1) for i in range(CH)))

    wave_buf = []
    max_wave = int(args.fs * args.png_ms / 1000.0) if args.png else 0
    if args.png and max_wave < 64:
        max_wave = 64

    p = Parser()
    t0 = time.time()
    t_txt = t0
    t_plot = t0
    t_text = t0
    n_win = 0
    last = [0] * CH

    try:
        while True:
            chunk = ser.read(4096)
            now = time.time()

            if chunk:
                for seq, s in p.feed(chunk):
                    if fb:
                        fb.write(struct.pack("<16H", *s))
                    if wf:
                        if args.wav_ch:
                            v = (s[args.wav_ch - 1] - 2048) * 16
                            v = -32768 if v < -32768 else (32767 if v > 32767 else v)
                            wf.writeframesraw(struct.pack("<h", v))
                        else:
                            vals = []
                            for x in s:
                                v = (x - 2048) * 16
                                vals.append(-32768 if v < -32768 else (32767 if v > 32767 else v))
                            wf.writeframesraw(struct.pack("<16h", *vals))
                    if fc:
                        if args.mv:
                            vals = ["%.1f" % (v * VREF_MV / ADC_FULL) for v in s]
                        else:
                            vals = [str(v) for v in s]
                        fc.write(str(seq) + "," + ",".join(vals) + "\n")
                    for i in range(CH):
                        hist[i].append(s[i])
                    last = s
                    n_win += 1
                    if args.png and len(wave_buf) < max_wave:
                        wave_buf.append(s)
                    if args.text and args.every > 0 and (n_win % args.every == 0):
                        sys.stdout.write("%8.3fs  %s\n" % (now - t0, " ".join("%4d" % v for v in s)))
                        sys.stdout.flush()

            if args.text and not args.every and now - t_text >= 0.1:
                sys.stdout.write("%8.2fs  %s\n" % (now - t0, " ".join("%4d" % v for v in last)))
                sys.stdout.flush()
                t_text = now

            if (not args.text) and (now - t_txt >= 1.0):
                fps = n_win / (now - t_txt)
                sys.stdout.write(
                    "\rgood=%-8d %7.1f fps  lost=%-6d crc=%-4d skip=%-7d | "
                    "ch01..08: %s   "
                    % (p.good, fps, p.lost, p.crc_err, p.skipped,
                       " ".join("%4d" % last[i] for i in range(8)))
                )
                sys.stdout.flush()
                t_txt = now
                n_win = 0

            if curves is not None and now - t_plot >= 0.05:
                n = len(hist[0])
                if n:
                    x = list(range(n))
                    for i in range(CH):
                        curves[i].setData(x, list(hist[i]))
                app.processEvents()
                t_plot = now

            if args.seconds and now - t0 > args.seconds:
                break

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if fb:
            fb.close()
        if fc:
            fc.close()
        if wf:
            wf.close()
        if win is not None:
            win.close()

        dur = time.time() - t0
        print(
            "\n\n共 %d 帧 / %.1f s = %.1f fps   丢帧 %d   假帧头 %d   跳过字节 %d"
            % (p.good, dur, p.good / max(dur, 1e-9), p.lost, p.crc_err, p.skipped)
        )
        if args.save:
            print("原始负载已写入 %s（每帧 32 字节，共 %d 字节）"
                  % (args.save, p.good * 32))
        if args.wav:
            print("WAV 已写入 %s（%s，%d Hz，%.2f 秒）"
                  % (args.wav,
                     "单声道 ch%d" % args.wav_ch if args.wav_ch else "16 声道",
                     args.fs,
                     p.good / max(args.fs, 1)))
        print("换算：mV = code * 3300 / 4096")

        if args.png and wave_buf:
            try:
                import numpy as np
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt

                arr = np.array(wave_buf, dtype=float)
                n = arr.shape[0]
                tm = np.arange(n) / float(args.fs) * 1000.0
                fig, axes = plt.subplots(CH, 1, figsize=(12, 16), sharex=True)
                for i in range(CH):
                    ax = axes[i]
                    ax.plot(tm, arr[:, i], linewidth=0.6, color="#1f77b4")
                    lo, hi = arr[:, i].min(), arr[:, i].max()
                    ax.set_ylim(lo - 20, hi + 20)
                    ax.set_ylabel("ch%02d" % (i + 1), fontsize=7)
                    ax.grid(True, alpha=0.3, linewidth=0.4)
                    ax.tick_params(labelsize=6)
                    ax.text(0.995, 0.9, "p-p %.0f" % (hi - lo),
                            transform=ax.transAxes, ha="right", fontsize=6, color="#888")
                axes[-1].set_xlabel("time [ms] @ %d Hz" % args.fs, fontsize=8)
                fig.suptitle("MIC16_ADC123_USB  %d ch x %d Hz  (%d frames, %.1f ms)"
                             % (CH, args.fs, n, n * 1000.0 / args.fs), fontsize=10)
                fig.tight_layout(rect=[0, 0, 1, 0.985])
                fig.savefig(args.png, dpi=90)
                plt.close(fig)
                print("waveform saved: %s (%d frames, %.1f ms)" % (args.png, n, n * 1000.0 / args.fs))
            except Exception as e:
                print("plot failed:", e)


if __name__ == "__main__":
    main()
