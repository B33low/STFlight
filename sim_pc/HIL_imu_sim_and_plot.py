import serial
import struct
import time
import threading
from collections import deque
import math
import numpy as np
import argparse

import matplotlib.pyplot as plt


# ===================== BUS / PROTOCOL =====================
AA = 0xAA
BB = 0x55

BUS_MSG_PUBLISH = 1
BUS_MSG_WRITE   = 2
BUS_MSG_INJECT  = 3
BUS_MSG_READREQ = 4

BUS_KIND_STATE  = 1
BUS_KIND_PARAM  = 2
BUS_KIND_STREAM = 3

ID_IMU_RAW   = 1
ID_ATT_STATE = 3
ID_ALT_STATE = 4


# ===================== PAYLOAD FORMATS =====================
# IMU: int16 ax ay az gx gy gz temp; uint32 t_us
IMU_FMT  = "<hhhhhhhI"
IMU_SIZE = struct.calcsize(IMU_FMT)

# ATTITUDE: float roll; float pitch; float yaw
ATT_FMT  = "<fff"
ATT_SIZE = struct.calcsize(ATT_FMT)

# ALTITUDE: float az_ms2; float vz_mps; float pz_m; uint32 t_us
ALT_FMT  = "<fffI"
ALT_SIZE = struct.calcsize(ALT_FMT)


# ===================== PHYS / SCALE (match MCU defaults) =====================
G = 9.80665

# Match your MCU fallbacks / defaults
ACC_LSB_TO_MS2  = 0.01
GYRO_LSB_TO_RAD = 0.001


# ===================== PACK / SEND =====================
def pack_imu_raw(ax, ay, az, gx, gy, gz, temp, t_us):
    return struct.pack(IMU_FMT, ax, ay, az, gx, gy, gz, temp, t_us)


def send_frame(ser, msg, kind, id_, payload: bytes):
    if len(payload) > 255:
        raise ValueError("Payload too large for 1-byte len field")
    frame = bytes([AA, BB, msg, kind, id_, len(payload)]) + payload
    ser.write(frame)


# ===================== RX PARSER =====================
class BusFrameParserNew:
    # AA 55 msg kind id len payload
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


# ===================== ORIENTATION MATH =====================
def Rx(r):
    cr, sr = np.cos(r), np.sin(r)
    return np.array([
        [1,  0,   0],
        [0, cr, -sr],
        [0, sr,  cr],
    ], dtype=float)


def Ry(p):
    cp, sp = np.cos(p), np.sin(p)
    return np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp],
    ], dtype=float)


def Rz(y):
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1],
    ], dtype=float)


def rpy_to_R(roll, pitch, yaw):
    # R = Rz * Ry * Rx (same convention as your 3D helper)
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)


def gravity_in_body(roll, pitch, yaw):
    """
    Target behavior for your estimator formulas:
      level => ax=0 ay=0 az=+g

    g_world = [0,0,G]
    a_body = R^T * g_world
    """
    R = rpy_to_R(roll, pitch, yaw)
    g_world = np.array([0.0, 0.0, G], dtype=float)
    a_body = R.T @ g_world
    return float(a_body[0]), float(a_body[1]), float(a_body[2])


def rpy_to_body_z_world(roll, pitch, yaw):
    """
    Direction of BODY +Z axis expressed in world frame.
    v_world = R * [0,0,1]
    """
    R = rpy_to_R(roll, pitch, yaw)
    v = R @ np.array([0.0, 0.0, 1.0])
    n = np.linalg.norm(v)
    if n > 1e-9:
        v = v / n
    return float(v[0]), float(v[1]), float(v[2])


# ===================== SAFE ROLL PROFILE (for HIL mode) =====================
def smoothstep(s):
    return s * s * (3.0 - 2.0 * s)

def smoothstep_derivative(s):
    return 6.0 * s * (1.0 - s)


def safe_roll_maneuver_profile(
    t,
    idle_before=1.0,
    roll_up_duration=0.6,
    hold_duration=0.15,
    ballistic_duration=0.06,  # keep VERY short
    roll_down_duration=0.6,
    idle_after=1.0,
    max_roll_deg=150.0
):
    """
    A 'safe aerobatic' roll:
      level -> roll up to ~150° -> short hold -> tiny ballistic blip -> roll back -> level

    Returns: (roll, roll_rate, accel_scale)
      accel_scale = 1.0 normally
      accel_scale ~ 0.0 during ballistic to mimic near-freefall specific force.
    """
    max_roll = math.radians(max_roll_deg)

    cycle = (
        idle_before +
        roll_up_duration +
        hold_duration +
        ballistic_duration +
        roll_down_duration +
        idle_after
    )
    tc = t % cycle

    # 1) idle before
    if tc < idle_before:
        return 0.0, 0.0, 1.0

    # 2) roll up 0 -> max_roll
    tc -= idle_before
    if tc < roll_up_duration:
        s = tc / roll_up_duration
        s = max(0.0, min(1.0, s))
        roll = max_roll * smoothstep(s)
        roll_rate = max_roll * smoothstep_derivative(s) / roll_up_duration
        return roll, roll_rate, 1.0

    # 3) hold
    tc -= roll_up_duration
    if tc < hold_duration:
        return max_roll, 0.0, 1.0

    # 4) tiny ballistic
    tc -= hold_duration
    if tc < ballistic_duration:
        # Keep gyro calm here
        return max_roll, 0.0, 0.05  # ~0g-ish IMU reading

    # 5) roll down max_roll -> 0
    tc -= ballistic_duration
    if tc < roll_down_duration:
        s = tc / roll_down_duration
        s = max(0.0, min(1.0, s))
        roll = max_roll * (1.0 - smoothstep(s))
        roll_rate = -max_roll * smoothstep_derivative(s) / roll_down_duration
        return roll, roll_rate, 1.0

    # 6) idle after
    return 0.0, 0.0, 1.0


def imu_model(t):
    """
    SAFE aerobatic IMU model.
    Produces raw int16 ax ay az gx gy gz
    Compatible with your MCU complementary filter expectations.
    """
    roll, roll_rate, accel_scale = safe_roll_maneuver_profile(t)

    # Keep pitch/yaw zero for clarity
    pitch = 0.0
    yaw = 0.0
    pitch_rate = 0.0
    yaw_rate = 0.0

    # Accel from gravity projection (specific force)
    ax_ms2, ay_ms2, az_ms2 = gravity_in_body(roll, pitch, yaw)

    # Optional tiny ballistic phase
    ax_ms2 *= accel_scale
    ay_ms2 *= accel_scale
    az_ms2 *= accel_scale

    # Convert to raw
    ax = int(round(ax_ms2 / ACC_LSB_TO_MS2))
    ay = int(round(ay_ms2 / ACC_LSB_TO_MS2))
    az = int(round(az_ms2 / ACC_LSB_TO_MS2))

    gx = int(round(roll_rate  / GYRO_LSB_TO_RAD))
    gy = int(round(pitch_rate / GYRO_LSB_TO_RAD))
    gz = int(round(yaw_rate   / GYRO_LSB_TO_RAD))

    def clamp_i16(v):
        return max(-32768, min(32767, v))

    return (
        clamp_i16(ax), clamp_i16(ay), clamp_i16(az),
        clamp_i16(gx), clamp_i16(gy), clamp_i16(gz),
        roll, pitch, yaw
    )


# ===================== READER THREAD =====================
def reader_loop(ser, stop, shared, lock, debug_frames=False):
    parser = BusFrameParserNew()

    while not stop["stop"]:
        data = ser.read(256)
        if not data:
            continue

        frames = parser.feed(data)
        now_pc = time.perf_counter()

        for msg, kind, id_, payload in frames:
            with lock:
                shared["frames_total"] += 1

            if debug_frames:
                print(f"[RX FRAME] msg={msg} kind={kind} id={id_} len={len(payload)}")

            # ---------- IMU RAW STREAM ----------
            if msg == BUS_MSG_PUBLISH and kind == BUS_KIND_STREAM and id_ == ID_IMU_RAW:
                if len(payload) == IMU_SIZE:
                    ax, ay, az, gx, gy, gz, temp, t_us = struct.unpack(IMU_FMT, payload)
                    with lock:
                        shared["imu_frames"] += 1
                        shared["t_imu"].append(now_pc)
                        shared["ax_raw"].append(ax)
                        shared["ay_raw"].append(ay)
                        shared["az_raw"].append(az)
                        shared["gx_raw"].append(gx)
                        shared["gy_raw"].append(gy)
                        shared["gz_raw"].append(gz)
                else:
                    with lock:
                        shared["imu_bad_len"] += 1
                    # Helpful debug for struct mismatch
                    print(f"[WARN] IMU frame wrong length: {len(payload)} (expected {IMU_SIZE})")

            # ---------- ATTITUDE STATE ----------
            elif msg == BUS_MSG_PUBLISH and kind == BUS_KIND_STATE and id_ == ID_ATT_STATE:
                if len(payload) == ATT_SIZE:
                    roll, pitch, yaw = struct.unpack(ATT_FMT, payload)
                    with lock:
                        shared["att_frames"] += 1
                        shared["att_latest"] = (roll, pitch, yaw)
                        shared["t_att"].append(now_pc)

            # ---------- ALTITUDE STATE ----------
            elif msg == BUS_MSG_PUBLISH and kind == BUS_KIND_STATE and id_ == ID_ALT_STATE:
                if len(payload) == ALT_SIZE:
                    az_ms2, vz_mps, pz_m, t_us = struct.unpack(ALT_FMT, payload)
                    with lock:
                        shared["alt_frames"] += 1
                        shared["t_alt"].append(now_pc)
                        shared["pz_m"].append(pz_m)

            else:
                # Some other message / kind / id we don't care about yet
                with lock:
                    shared["other_frames"] += 1


# ===================== ARGS =====================
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    p.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    p.add_argument(
        "--mode",
        choices=["listen", "inject"],
        default="listen",
        help="listen: just log real IMU; inject: send fake IMU via HIL"
    )
    p.add_argument(
        "--debug-frames",
        action="store_true",
        help="Print every received frame header (msg/kind/id/len)"
    )
    return p.parse_args()


# ===================== MAIN =====================
def main():
    args = parse_args()

    port = args.port
    baud = args.baud
    mode = args.mode
    debug_frames = args.debug_frames

    enable_inject = (mode == "inject")

    INJECT_PERIOD = 0.02  # 50 Hz
    PLOT_PERIOD   = 0.05  # 20 Hz
    DEBUG_PERIOD  = 0.50  # console throttle

    ser = serial.Serial(port, baud, timeout=0.05)

    stop = {"stop": False}
    lock = threading.Lock()

    shared = {
        "t_imu":  deque(maxlen=800),
        "ax_raw": deque(maxlen=800),
        "ay_raw": deque(maxlen=800),
        "az_raw": deque(maxlen=800),
        "gx_raw": deque(maxlen=800),
        "gy_raw": deque(maxlen=800),
        "gz_raw": deque(maxlen=800),

        "t_alt": deque(maxlen=800),
        "pz_m":  deque(maxlen=800),

        "t_att": deque(maxlen=300),
        "att_latest": (0.0, 0.0, 0.0),

        # RX stats
        "frames_total": 0,
        "imu_frames":   0,
        "imu_bad_len":  0,
        "att_frames":   0,
        "alt_frames":   0,
        "other_frames": 0,
    }

    t_reader = threading.Thread(
        target=reader_loop,
        args=(ser, stop, shared, lock, debug_frames),
        daemon=True
    )
    t_reader.start()

    # --------- 2D plots ----------
    plt.ion()
    fig, (ax_acc, ax_gyr, ax_alt) = plt.subplots(3, 1, figsize=(10, 8))

    line_ax, = ax_acc.plot([], [], label="ax")
    line_ay, = ax_acc.plot([], [], label="ay")
    line_az, = ax_acc.plot([], [], label="az")
    ax_acc.set_title("IMU_RAW acceleration (live)")
    ax_acc.set_xlabel("time (relative, s)")
    ax_acc.set_ylabel("a (raw LSB)")
    ax_acc.grid(True)
    ax_acc.legend(loc="upper right")

    line_gx, = ax_gyr.plot([], [], label="gx")
    line_gy, = ax_gyr.plot([], [], label="gy")
    line_gz, = ax_gyr.plot([], [], label="gz")
    ax_gyr.set_title("IMU_RAW gyro (live)")
    ax_gyr.set_xlabel("time (relative, s)")
    ax_gyr.set_ylabel("gyro (raw LSB)")
    ax_gyr.grid(True)
    ax_gyr.legend(loc="upper right")

    line_pz, = ax_alt.plot([], [], label="pz")
    ax_alt.set_title("ALTITUDE estimate pz (live)")
    ax_alt.set_xlabel("time (relative, s)")
    ax_alt.set_ylabel("pz (m)")
    ax_alt.grid(True)
    ax_alt.legend(loc="upper right")

    # --------- 3D attitude arrow ----------
    fig_att = plt.figure(figsize=(6, 6))
    ax_att3d = fig_att.add_subplot(111, projection="3d")
    ax_att3d.set_title("ATTITUDE (estimated body +Z in world)")
    ax_att3d.set_xlabel("X")
    ax_att3d.set_ylabel("Y")
    ax_att3d.set_zlabel("Z")
    ax_att3d.set_xlim(-1, 1)
    ax_att3d.set_ylim(-1, 1)
    ax_att3d.set_zlim(-1, 1)

    # world axes
    ax_att3d.plot([0, 1], [0, 0], [0, 0])
    ax_att3d.plot([0, 0], [0, 1], [0, 0])
    ax_att3d.plot([0, 0], [0, 0], [0, 1])

    vec_handle = ax_att3d.quiver(0, 0, 0, 0, 0, 1, length=1.0, normalize=True)

    start = time.perf_counter()
    last_inject = 0.0
    last_plot   = 0.0
    last_dbg    = 0.0

    # Keep latest TX "truth" (only meaningful in inject mode)
    truth = {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "ax": 0, "ay": 0, "az": 0,
        "gx": 0, "gy": 0, "gz": 0,
    }

    print(f"Connected to {port} @ {baud}")
    print(f"Mode: {mode.upper()} (inject fake IMU: {enable_inject})")
    print("Ctrl+C to stop.\n")

    try:
        while True:
            now = time.perf_counter()
            t = now - start

            # --------- INJECT (only if mode == inject) ----------
            if enable_inject and (now - last_inject >= INJECT_PERIOD):
                last_inject = now
                t_us = int(t * 1_000_000)

                ax_i16, ay_i16, az_i16, gx_i16, gy_i16, gz_i16, roll, pitch, yaw = imu_model(t)

                truth.update({
                    "roll": roll, "pitch": pitch, "yaw": yaw,
                    "ax": ax_i16, "ay": ay_i16, "az": az_i16,
                    "gx": gx_i16, "gy": gy_i16, "gz": gz_i16,
                })

                payload = pack_imu_raw(
                    ax=ax_i16, ay=ay_i16, az=az_i16,
                    gx=gx_i16, gy=gy_i16, gz=gz_i16,
                    temp=0,
                    t_us=t_us
                )
                send_frame(ser, BUS_MSG_INJECT, BUS_KIND_STREAM, ID_IMU_RAW, payload)

            # --------- DEBUG CONSOLE ----------
            if now - last_dbg >= DEBUG_PERIOD:
                last_dbg = now

                with lock:
                    est_roll, est_pitch, est_yaw = shared["att_latest"]
                    frames_total = shared["frames_total"]
                    imu_frames   = shared["imu_frames"]
                    imu_bad_len  = shared["imu_bad_len"]
                    att_frames   = shared["att_frames"]
                    alt_frames   = shared["alt_frames"]
                    other_frames = shared["other_frames"]

                print(
                    f"[STATS] frames_total={frames_total} | "
                    f"IMU ok={imu_frames}, IMU bad_len={imu_bad_len}, "
                    f"ATT={att_frames}, ALT={alt_frames}, OTHER={other_frames}"
                )

                if enable_inject:
                    # Compute accel-only angles from TX sample (as your MCU does)
                    ax_ms2 = truth["ax"] * ACC_LSB_TO_MS2
                    ay_ms2 = truth["ay"] * ACC_LSB_TO_MS2
                    az_ms2 = truth["az"] * ACC_LSB_TO_MS2

                    roll_acc  = math.atan2(ay_ms2, az_ms2)
                    pitch_acc = math.atan2(-ax_ms2, math.sqrt(ay_ms2*ay_ms2 + az_ms2*az_ms2))

                    def deg(x): return x * 180.0 / math.pi

                    print(
                        "[DBG] "
                        f"TX rpy(deg)=({deg(truth['roll']):6.1f},{deg(truth['pitch']):6.1f},{deg(truth['yaw']):6.1f}) "
                        f"TX a=({truth['ax']:6d},{truth['ay']:6d},{truth['az']:6d}) "
                        f"TX g=({truth['gx']:6d},{truth['gy']:6d},{truth['gz']:6d}) | "
                        f"ACC r/p(deg)=({deg(roll_acc):6.1f},{deg(pitch_acc):6.1f}) | "
                        f"RX rpy(deg)=({deg(est_roll):6.1f},{deg(est_pitch):6.1f},{deg(est_yaw):6.1f})"
                    )

            # --------- PLOT ----------
            if now - last_plot >= PLOT_PERIOD:
                last_plot = now

                with lock:
                    t_imu = list(shared["t_imu"])
                    ax_raw = list(shared["ax_raw"])
                    ay_raw = list(shared["ay_raw"])
                    az_raw = list(shared["az_raw"])
                    gx_raw = list(shared["gx_raw"])
                    gy_raw = list(shared["gy_raw"])
                    gz_raw = list(shared["gz_raw"])

                    t_alt = list(shared["t_alt"])
                    pz_m = list(shared["pz_m"])

                    est_roll, est_pitch, est_yaw = shared["att_latest"]

                # IMU plots
                if t_imu:
                    n = min(len(t_imu), len(ax_raw), len(ay_raw), len(az_raw),
                            len(gx_raw), len(gy_raw), len(gz_raw))
                    t_imu = t_imu[-n:]
                    ax_raw = ax_raw[-n:]
                    ay_raw = ay_raw[-n:]
                    az_raw = az_raw[-n:]
                    gx_raw = gx_raw[-n:]
                    gy_raw = gy_raw[-n:]
                    gz_raw = gz_raw[-n:]

                    t0 = t_imu[0]
                    xs = [tt - t0 for tt in t_imu]

                    line_ax.set_data(xs, ax_raw)
                    line_ay.set_data(xs, ay_raw)
                    line_az.set_data(xs, az_raw)
                    ax_acc.relim()
                    ax_acc.autoscale_view()

                    line_gx.set_data(xs, gx_raw)
                    line_gy.set_data(xs, gy_raw)
                    line_gz.set_data(xs, gz_raw)
                    ax_gyr.relim()
                    ax_gyr.autoscale_view()

                # Altitude plot
                if t_alt:
                    n = min(len(t_alt), len(pz_m))
                    t_alt = t_alt[-n:]
                    pz_m = pz_m[-n:]
                    t0 = t_alt[0]
                    xs_alt = [tt - t0 for tt in t_alt]
                    line_pz.set_data(xs_alt, pz_m)
                    ax_alt.relim()
                    ax_alt.autoscale_view()

                # 3D attitude arrow (estimated)
                vx, vy, vz = rpy_to_body_z_world(est_roll, est_pitch, est_yaw)
                try:
                    vec_handle.remove()
                except Exception:
                    pass
                vec_handle = ax_att3d.quiver(0, 0, 0, vx, vy, vz, length=1.0, normalize=True)

                fig.canvas.draw_idle()
                fig_att.canvas.draw_idle()
                plt.pause(0.001)

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop["stop"] = True
        time.sleep(0.1)
        ser.close()


if __name__ == "__main__":
    main()
