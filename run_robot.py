#!/usr/bin/env python3
"""run_robot.py -- ZNN + modelo dinámico en la laptop; ejecución por SSH en la Pi.

Ejemplos
  # Robot real (SSH con llave, sin contraseña: ssh-copy-id pi@IP)
  python run_robot.py --host pi@192.168.1.50 \
      --remote /home/pi/TurtleTorqueNew/bridge/build/bridge --duration 40

  # Sin hardware: puente local en modo simulación (prueba de todo el lazo)
  python run_robot.py --cmd "../bridge/build/bridge --sim" --duration 30 --plot sim.png
"""
import argparse
import csv
import math
import sys
import time

import numpy as np

import ddmr


def build_cmd(a):
    if a.cmd:
        return a.cmd
    if not a.host:
        sys.exit("Indica --host usuario@ip (o --cmd para un puente local)")
    ssh = ("ssh -T -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=2 "
           "-o IPQoS=lowdelay")
    return f"{ssh} {a.host} {a.remote} {a.bridge_args}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="usuario@ip de la Raspberry Pi")
    ap.add_argument("--remote", default="/home/pi/TurtleTorqueNew/bridge/build/bridge")
    ap.add_argument("--bridge-args", default="", help="argumentos extra para el puente (ej. '--imax 80')")
    ap.add_argument("--cmd", help="comando completo del puente (para pruebas locales)")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=100.0, help="Hz del lazo de la laptop")
    ap.add_argument("--kd", type=float, default=0.05, help="ganancia de realim. local en la Pi [N m s/rad]")
    ap.add_argument("--traj", choices=["circle", "spline"], default="circle")
    ap.add_argument("--ell", type=float, default=0.10)
    ap.add_argument("--gamma", type=float, default=15.0)
    ap.add_argument("--kp", type=float, default=2.0)
    ap.add_argument("--params", help="JSON con Params (modelo dinámico)")
    ap.add_argument("--log", default="run_log.csv")
    ap.add_argument("--plot", help="guardar figura PNG al terminar")
    a = ap.parse_args()

    p = ddmr.Params.from_json(a.params) if a.params else ddmr.Params()
    znn = ddmr.ZNNTracker(ell=a.ell, gamma=a.gamma, kp=a.kp)
    traj = (ddmr.Circle(ell=a.ell) if a.traj == "circle" else
            ddmr.SplineTraj([[a.ell, 0], [0.5, 0.2], [0.9, 0.0], [1.3, 0.3]], a.duration))

    # verificación cruzada de las dos formas del modelo
    e, de = np.array([3.0, 2.2]), np.array([0.5, -0.4])
    assert np.allclose(ddmr.tau_ff(p, e, de), ddmr.tau_ff_lagrange(p, e, de)), "modelo inconsistente"

    link = ddmr.BridgeLink(build_cmd(a))
    rows = []
    try:
        tel = link.wait_tel(15.0)
        print(f"[laptop] telemetría OK. Iniciando {a.duration:.0f} s a {a.rate:.0f} Hz.")
        dt_nom = 1.0 / a.rate
        t0 = time.perf_counter()
        t_prev = 0.0
        k = 0
        while True:
            t = time.perf_counter() - t0
            if t > a.duration:
                break
            tel = link.latest()
            pose = (tel.x, tel.y, tel.th)
            dt = min(max(t - t_prev, 0.5 * dt_nom), 2.0 * dt_nom) if k else dt_nom
            t_prev = t
            pr, dpr, ddpr = traj(t)
            u, du = znn.step(pose, (pr, dpr, ddpr), dt)
            eta_c = ddmr.body_to_wheel(p, *u)
            deta_c = ddmr.body_to_wheel(p, *du)
            tau = ddmr.tau_ff(p, eta_c, deta_c)
            link.send_ref(k, True, eta_c, deta_c, tau, a.kd)

            c, s = math.cos(tel.th), math.sin(tel.th)
            P = (tel.x + a.ell * c, tel.y + a.ell * s)
            rows.append([t, k, pr[0], pr[1], P[0], P[1], tel.th, eta_c[0], eta_c[1], tel.eta_r, tel.eta_l,
                         tau[0], tau[1], tel.cmd_r, tel.cmd_l, tel.cur_r, tel.cur_l, tel.age_ms, tel.yaw_deg])
            if k % int(a.rate) == 0:
                err = math.hypot(pr[0] - P[0], pr[1] - P[1])
                print(f"[laptop] t={t:5.1f}s  err={err*100:5.1f} cm  v={u[0]:.3f} w={u[1]:.3f}  "
                      f"cmd=[{tel.cmd_r:4d} {tel.cmd_l:4d}]")
            k += 1
            delay = t0 + k * dt_nom - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\n[laptop] interrumpido: frenando.")
    finally:
        link.close()

    hdr = ["t", "seq", "pr_x", "pr_y", "P_x", "P_y", "theta", "etac_r", "etac_l", "eta_r", "eta_l",
           "tau_r", "tau_l", "cmd_r", "cmd_l", "cur_r_A", "cur_l_A", "age_ms", "yaw_imu_deg"]
    with open(a.log, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(rows)
    d = np.array(rows)
    if len(d):
        err = np.hypot(d[:, 2] - d[:, 4], d[:, 3] - d[:, 5])
        m = d[:, 0] > 5.0
        rms = float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else float("nan")
        print(f"[laptop] log: {a.log} | error RMS (t>5s) = {rms*100:.2f} cm | máx = {err.max()*100:.1f} cm")
        if a.plot:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, 3, figsize=(15, 4))
            ax[0].plot(d[:, 2], d[:, 3], "k--", label="referencia P")
            ax[0].plot(d[:, 4], d[:, 5], label="medido P")
            ax[0].set_aspect("equal"); ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].legend()
            ax[1].plot(d[:, 0], err * 100); ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("error [cm]")
            ax[2].plot(d[:, 0], d[:, 7], "--", label="eta_c R"); ax[2].plot(d[:, 0], d[:, 9], label="eta R")
            ax[2].plot(d[:, 0], d[:, 8], "--", label="eta_c L"); ax[2].plot(d[:, 0], d[:, 10], label="eta L")
            ax[2].set_xlabel("t [s]"); ax[2].set_ylabel("rad/s"); ax[2].legend()
            fig.tight_layout(); fig.savefig(a.plot, dpi=110)
            print(f"[laptop] figura: {a.plot}")


if __name__ == "__main__":
    main()
