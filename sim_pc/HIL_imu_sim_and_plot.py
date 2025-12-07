import serial
import struct
import time
import threading
from collections import deque

import matplotlib.pyplot as plt


AA = 0xAA
BB = 0x55

# PC-side names/enums
BUS_MSG_PUBLISH = 1
BUS_MSG_WRITE   = 2
BUS_MSG_INJECT  = 3
BUS_MSG_READREQ = 4

BUS_KIND_STATE  = 1
BUS_KIND_PARAM  = 2
BUS_KIND_STREAM = 3

ID_IMU_RAW   = 1
ID_ALT_STATE = 4


# --------- Payload layouts (must match your C structs) ----------
# IMU: int16 ax ay az gx gy gz temp; uint32 t_us
IMU_FMT  = "<hhhhhhhI"
IMU_SIZE = struct.calcsize(IMU_FMT)

# ALTITUDE: float az_ms2; float vz_mps; float pz_m; uint32 t_us
ALT_FMT  = "<fffI"
ALT_SIZE = struct.calcsize(ALT_FMT)


def pack_imu_raw(ax, ay, az, gx, gy, gz, temp, t_us):
    return struct.pack(IMU_FMT, ax, ay, az, gx, gy, gz, temp, t_us)


def send_frame(ser, msg, kind, id_, payload: bytes):
    if len(payload) > 255:
        raise ValueError("Payload too large for 1-byte len field")
    frame = bytes([AA, BB, msg, kind, id_, len(payload)]) + payload
    ser.write(frame)


# --------- PC RX parser for NEW TX format ----------
# AA 55 msg kind id len payload
class BusFrameParserNew:
    WAIT_AA = 0
    WAIT_55 = 1
    READ_MSG = 2
    READ_KIND = 3
    READ_ID = 4
    READ_LEN = 5
    READ_PAYLOAD = 6

    def __init__(self):
        self.st = self.WAIT_AA
        self.msg = 0
        self.kind = 0
        self.id = 0
        self.length = 0
        self.payload = bytearray()

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if self.st == self.WAIT_AA:
                self.st = self.WAIT_55 if b == AA else self.WAIT_AA

            elif self.st == self.WAIT_55:
                if b == BB:
                    self.st = self.READ_MSG
                else:
                    self.st = self.WAIT_55 if b == AA else self.WAIT_AA

            elif self.st == self.READ_MSG:
                self.msg = b
                self.st = self.READ_KIND

            elif self.st == self.READ_KIND:
                self.kind = b
                self.st = self.READ_ID

            elif self.st == self.READ_ID:
                self.id = b
                self.st = self.READ_LEN

            elif self.st == self.READ_LEN:
                self.length = b
                self.payload = bytearray()
                if self.length == 0:
                    frames.append((self.msg, self.kind, self.id, b""))
                    self.st = self.WAIT_AA
                else:
                    self.st = self.READ_PAYLOAD

            elif self.st == self.READ_PAYLOAD:
                self.payload.append(b)
                if len(self.payload) >= self.length:
                    frames.append((self.msg, self.kind, self.id, bytes(self.payload)))
                    self.st = self.WAIT_AA

        return frames


def reader_loop(ser, stop, shared):
    parser = BusFrameParserNew()

    while not stop["stop"]:
        data = ser.read(256)
        if not data:
            continue

        frames = parser.feed(data)
        now_pc = time.perf_counter()

        for msg, kind, id_, payload in frames:
            # ---- IMU RAW stream ----
            if msg == BUS_MSG_PUBLISH and kind == BUS_KIND_STREAM and id_ == ID_IMU_RAW:
                if len(payload) == IMU_SIZE:
                    ax, ay, az, gx, gy, gz, temp, t_us = struct.unpack(IMU_FMT, payload)
                    shared["t_imu"].append(now_pc)
                    shared["az_raw"].append(az)

            # ---- ALTITUDE state ----
            elif msg == BUS_MSG_PUBLISH and kind == BUS_KIND_STATE and id_ == ID_ALT_STATE:
                if len(payload) == ALT_SIZE:
                    az_ms2, vz_mps, pz_m, t_us = struct.unpack(ALT_FMT, payload)
                    shared["t_alt"].append(now_pc)
                    shared["pz_m"].append(pz_m)
                    shared["vz_mps"].append(vz_mps)
                    shared["az_ms2"].append(az_ms2)


def main():
    port = "COM3"
    baud = 115200

    ser = serial.Serial(port, baud, timeout=0.05)

    stop = {"stop": False}
    shared = {
        # IMU buffers
        "t_imu":  deque(maxlen=500),
        "az_raw": deque(maxlen=500),

        # ALT buffers
        "t_alt":  deque(maxlen=500),
        "pz_m":   deque(maxlen=500),
        "vz_mps": deque(maxlen=500),
        "az_ms2": deque(maxlen=500),
    }

    t_reader = threading.Thread(target=reader_loop, args=(ser, stop, shared), daemon=True)
    t_reader.start()

    # --------- Live plot setup ----------
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6))

    # Top: IMU az raw
    line_imu, = ax1.plot([], [])
    ax1.set_title("IMU_RAW az (live)")
    ax1.set_xlabel("time (relative, s)")
    ax1.set_ylabel("az (raw LSB)")
    ax1.grid(True)

    # Bottom: Altitude pz
    line_pz, = ax2.plot([], [])
    ax2.set_title("ALTITUDE estimate pz (live)")
    ax2.set_xlabel("time (relative, s)")
    ax2.set_ylabel("pz (m)")
    ax2.grid(True)

    start = time.perf_counter()
    az_val = 0
    last_inject = 0.0
    last_plot = 0.0

    try:
        while True:
            now = time.perf_counter()

            # --------- Inject at 10 Hz ----------
            if now - last_inject >= 0.1:
                last_inject = now
                az_val = (az_val + 10) % 200
                t_us = int((now - start) * 1_000_000)

                payload = pack_imu_raw(
                    ax=0, ay=0, az=az_val,
                    gx=0, gy=0, gz=0,
                    temp=0,
                    t_us=t_us
                )

                # NEW header with msg field
                send_frame(ser, BUS_MSG_INJECT, BUS_KIND_STREAM, ID_IMU_RAW, payload)

            # --------- Plot at ~20 Hz ----------
            if now - last_plot >= 0.05:
                last_plot = now

                # ---- IMU plot ----
                if shared["t_imu"]:
                    t0 = shared["t_imu"][0]
                    xs = [tt - t0 for tt in shared["t_imu"]]
                    ys = list(shared["az_raw"])

                    line_imu.set_data(xs, ys)
                    ax1.relim()
                    ax1.autoscale_view()

                # ---- ALT plot (pz) ----
                if shared["t_alt"]:
                    t0 = shared["t_alt"][0]
                    xs = [tt - t0 for tt in shared["t_alt"]]
                    ys = list(shared["pz_m"])

                    line_pz.set_data(xs, ys)
                    ax2.relim()
                    ax2.autoscale_view()

                # This keeps the UI responsive on most backends
                fig.canvas.draw_idle()
                plt.pause(0.001)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        stop["stop"] = True
        time.sleep(0.1)
        ser.close()


if __name__ == "__main__":
    main()
