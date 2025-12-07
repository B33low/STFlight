import serial
import struct
import time
import threading
import sys


# ---------------- Protocol constants ----------------
AA = 0xAA
BB = 0x55

BUS_MSG_PUBLISH = 1
BUS_MSG_WRITE   = 2
BUS_MSG_INJECT  = 3
BUS_MSG_READREQ = 4

BUS_KIND_STATE  = 1
BUS_KIND_PARAM  = 2
BUS_KIND_STREAM = 3

ID_IMU_RAW = 1


# ---------------- Packing helpers ----------------
# If your ImuRawSample is:
# int16 ax ay az gx gy gz temp; uint32 t_us
# => 7*2 + 4 = 18 bytes
def pack_imu_raw(ax, ay, az, gx, gy, gz, temp, t_us):
    return struct.pack("<hhhhhhhI", ax, ay, az, gx, gy, gz, temp, t_us)


def send_frame(ser, msg, kind, id_, payload: bytes):
    if len(payload) > 255:
        raise ValueError("Payload too large for 1-byte len field")
    frame = bytes([AA, BB, msg, kind, id_, len(payload)]) + payload
    ser.write(frame)


# ---------------- Optional pretty names ----------------
def kind_str(k):
    return {
        BUS_KIND_STATE: "STATE",
        BUS_KIND_PARAM: "PARAM",
        BUS_KIND_STREAM: "STREAM",
    }.get(k, f"K{int(k)}")


def msg_str(m):
    return {
        BUS_MSG_PUBLISH: "PUBLISH",
        BUS_MSG_WRITE:   "WRITE",
        BUS_MSG_INJECT:  "INJECT",
        BUS_MSG_READREQ: "READREQ",
    }.get(m, f"M{int(m)}")


def name_for(kind, id_):
    # Extend this map as you add more registry items
    if kind == BUS_KIND_STREAM and id_ == 1:
        return "IMU_RAW"
    if kind == BUS_KIND_PARAM and id_ == 2:
        return "IMU_CONV_META"
    if kind == BUS_KIND_STATE and id_ == 3:
        return "ATTITUDE"
    if kind == BUS_KIND_STATE and id_ == 4:
        return "ALTITUDE"
    return ""


# ---------------- Frame parser ----------------
class BusFrameParser:
    """
    Parses frames:
      AA 55 msg kind id len payload
    """
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


# ---------------- Reader thread ----------------
def reader_loop(ser: serial.Serial, stop_flag):
    parser = BusFrameParser()
    last_print = time.perf_counter()

    while not stop_flag["stop"]:
        try:
            data = ser.read(256)  # non-blocking-ish due to timeout
            if not data:
                continue

            frames = parser.feed(data)
            if frames:
                for msg, kind, id_, payload in frames:
                    nm = name_for(kind, id_)
                    nm_part = f" {nm}" if nm else ""

                    # Print structured line
                    print(
                        f"[RX] {msg_str(msg)} {kind_str(kind)} id={id_}{nm_part} "
                        f"len={len(payload)} payload={payload.hex(' ')}"
                    )
            else:
                # If bytes don't form frames yet, you may still want to see raw traffic:
                # throttle raw prints a bit to avoid spam
                now = time.perf_counter()
                if now - last_print > 0.2:
                    print(f"[RX-RAW] {data.hex(' ')}")
                    last_print = now

        except serial.SerialException as e:
            print("Serial error in reader:", str(e), file=sys.stderr)
            stop_flag["stop"] = True
        except Exception as e:
            print("Unexpected reader error:", str(e), file=sys.stderr)


# ---------------- Main sending loop ----------------
def main():
    import math

    port = "COM3"
    baud = 115200

    ser = serial.Serial(port, baud, timeout=0.05)

    stop_flag = {"stop": False}
    t_reader = threading.Thread(target=reader_loop, args=(ser, stop_flag), daemon=True)
    t_reader.start()

    print(f"Connected to {port} @ {baud}")
    print("Sending IMU tilt simulation every 0.1s. Ctrl+C to stop.\n")

    # ---- Must match your MCU param (ImuConvMeta.accel_lsb_to_ms2) ----
    ACC_LSB_TO_MS2 = 0.01  # you set this in app_init
    G_MS2 = 9.80665
    G_LSB = G_MS2 / ACC_LSB_TO_MS2  # ~981 LSB

    # ---- Optional gyro scale used by your estimator fallback ----
    # In your C attitude estimator:
    #   gx = raw.gx * 0.001f;  // rad/s
    GYRO_LSB_TO_RAD = 0.001

    # ---- Tilt profile ----
    # We'll simulate pitch as a sine wave
    A_DEG = 25.0   # amplitude of tilt
    F_HZ  = 0.2    # 1 cycle every 5 seconds

    start = time.perf_counter()

    def clamp_i16(x):
        xi = int(round(x))
        if xi < -32768:
            return -32768
        if xi > 32767:
            return 32767
        return xi

    try:
        while True:
            elapsed_s = time.perf_counter() - start
            t_us = int(elapsed_s * 1_000_000)

            # Pitch angle theta(t)
            A_RAD = math.radians(A_DEG)
            theta = A_RAD * math.sin(2 * math.pi * F_HZ * elapsed_s)

            # Gravity vector projected into body axes (pitch about Y)
            # Sign convention chosen to match your estimator formulas:
            # roll_acc  = atan2(ay, az)
            # pitch_acc = atan2(-ax, sqrt(ay^2 + az^2))
            ax = -G_LSB * math.sin(theta)
            ay = 0.0
            az =  G_LSB * math.cos(theta)

            # Optional gyro-consistent pitch rate
            # theta_dot = d/dt [A*sin(2π f t)] = A*2π f*cos(2π f t)
            theta_dot = A_RAD * (2 * math.pi * F_HZ) * math.cos(2 * math.pi * F_HZ * elapsed_s)

            gx = 0.0
            gy = theta_dot / GYRO_LSB_TO_RAD   # convert rad/s -> raw LSB
            gz = 0.0

            payload = pack_imu_raw(
                ax=clamp_i16(ax),
                ay=clamp_i16(ay),
                az=clamp_i16(az),
                gx=clamp_i16(gx),
                gy=clamp_i16(gy),
                gz=clamp_i16(gz),
                temp=0,
                t_us=t_us
            )

            # NEW header (msg field included)
            send_frame(ser, BUS_MSG_INJECT, BUS_KIND_STREAM, ID_IMU_RAW, payload)

            print(f"[TX] INJECT STREAM id={ID_IMU_RAW} IMU_RAW "
                  f"ax={int(ax)} ay={int(ay)} az={int(az)} "
                  f"gy={int(gy)} t_us={t_us}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop_flag["stop"] = True
        time.sleep(0.1)
        ser.close()


if __name__ == "__main__":
    main()
