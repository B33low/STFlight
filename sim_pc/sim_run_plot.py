import os
import subprocess
import csv
import io
import sys

import matplotlib.pyplot as plt


# --- Config chemins --- #
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
SIM_PC_DIR = os.path.join(REPO_ROOT, ".")
BUILD_DIR = os.path.join(SIM_PC_DIR, "build")
BIN_DIR = os.path.join(BUILD_DIR, "bin")

if os.name == "nt":
    EXE_NAME = "fc_sim_pc.exe"
else:
    EXE_NAME = "fc_sim_pc"

EXE_PATH = os.path.join(BIN_DIR, EXE_NAME)

CSV_PATH = os.path.join(REPO_ROOT, "last_run.csv")


def run(cmd, cwd=None):
    print(f"\n$ {' '.join(cmd)}")
    subprocess.run(cmd, cwd=cwd, check=True)


def main():
    # 1) Configure + build C (sim_pc)
    os.makedirs(BUILD_DIR, exist_ok=True)
    run(["cmake", "-S", ".", "-B", "build"], cwd=SIM_PC_DIR)
    run(["cmake", "--build", "build"], cwd=SIM_PC_DIR)

    # 2) Run exe, capture stdout (CSV)
    print(f"\nRunning {EXE_PATH} ...")
    proc = subprocess.run(
        [EXE_PATH],
        cwd=SIM_PC_DIR,
        capture_output=True,
        text=True,
        check=True,
    )

    stdout = proc.stdout

    # Sauvegarder dans un fichier CSV (optionnel mais pratique)
    with open(CSV_PATH, "w", newline="") as f:
        f.write(stdout)
    print(f"\nCSV saved to: {CSV_PATH}")

    # 3) Parser CSV et tracer
    reader = csv.DictReader(io.StringIO(stdout), skipinitialspace=True)

    time = []
    motor1 = []
    sim_az = []
    sim_vz = []
    sim_pz = []
    setpoint = []

    dt = []
    e_vz = []
    u_p = []
    u_i = []
    u_d = []
    u_raw = []
    u_sat = []
    vz_est = []
    pz_est = []

    for row in reader:
        time.append(float(row["time"]))
        motor1.append(float(row["motor1"]))
        sim_az.append(float(row["sim_az"]))
        sim_vz.append(float(row["sim_vz"]))
        sim_pz.append(float(row["sim_pz"]))
        setpoint.append(float(row["setpoint"]))

        dt.append(float(row["dt"]))
        e_vz.append(float(row["e_vz"]))
        u_p.append(float(row["u_p"]))
        u_i.append(float(row["u_i"]))
        u_d.append(float(row["u_d"]))
        u_raw.append(float(row["u_raw"]))
        u_sat.append(float(row["u_sat"]))
        vz_est.append(float(row["vz_est"]))
        pz_est.append(float(row["pz_est"]))


    # 4) Plot
    fig, axes = plt.subplots(6, 1, sharex=True, figsize=(10, 10))

    axes[0].plot(time, sim_pz, label="pz (sim)")
    axes[0].plot(time, pz_est, "--", label="pz_est (FC)")
    axes[0].set_ylabel("z [m]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(time, sim_vz, label="vz (sim)")
    axes[1].plot(time, vz_est, "--", label="vz_est (FC)")
    axes[1].plot(time, setpoint, ":", label="vz_set")
    axes[1].set_ylabel("vz [m/s]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(time, e_vz)
    axes[2].set_ylabel("e_vz")
    axes[2].grid(True)

    axes[3].plot(time, u_p, label="P")
    axes[3].plot(time, u_i, label="I")
    axes[3].plot(time, u_d, label="D")
    axes[3].set_ylabel("PID terms")
    axes[3].legend()
    axes[3].grid(True)

    axes[4].plot(time, u_raw, label="u_raw")
    axes[4].plot(time, u_sat, label="u_sat")
    axes[4].set_ylabel("u")
    axes[4].legend()
    axes[4].grid(True)

    axes[5].plot(time, dt)
    axes[5].set_ylabel("dt [s]")
    axes[5].set_xlabel("time [s]")
    axes[5].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        print("\nERROR while running command:", e.cmd, file=sys.stderr)
        print("Return code:", e.returncode, file=sys.stderr)
        sys.exit(e.returncode)
