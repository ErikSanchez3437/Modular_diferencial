"""ddmr.py -- Lado laptop: protocolo con el puente, modelo dinámico de Lagrange y tu ZNN."""
from __future__ import annotations

import binascii
import json
import math
import os
import shlex
import struct
import subprocess
import sys
import threading
import time
from collections import namedtuple
from dataclasses import asdict, dataclass

import numpy as np

# ===================================================================== protocolo
SYNC = 0x5AA5
REF_BODY = "<HBBI2f2f2ff"            # 36 bytes + CRC16 = 38
TEL_FMT = "<HBBII2f2f2f2hf3ffH"      # 62 bytes
TEL_SIZE = struct.calcsize(TEL_FMT)
assert struct.calcsize(REF_BODY) + 2 == 38 and TEL_SIZE == 62

Tel = namedtuple(
    "Tel",
    "sync type flags seq_echo t_ms eta_r eta_l phi_r phi_l cur_r cur_l "
    "cmd_r cmd_l yaw_deg x y th age_ms crc",
)


def crc16(b: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF); igual que el puente."""
    return binascii.crc_hqx(b, 0xFFFF)


def pack_ref(seq, enable, eta_c, deta_c, tau_ff, kd) -> bytes:
    body = struct.pack(
        REF_BODY, SYNC, 1, 1 if enable else 0, seq & 0xFFFFFFFF,
        float(eta_c[0]), float(eta_c[1]), float(deta_c[0]), float(deta_c[1]),
        float(tau_ff[0]), float(tau_ff[1]), float(kd),
    )
    return body + struct.pack("<H", crc16(body))


class TelParser:
    """Reensambla tramas del puente con resincronización por SYNC + CRC."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out, i = [], 0
        while len(self.buf) - i >= TEL_SIZE:
            if self.buf[i] == 0xA5 and self.buf[i + 1] == 0x5A:
                chunk = bytes(self.buf[i:i + TEL_SIZE])
                if chunk[2] == 2 and struct.unpack_from("<H", chunk, TEL_SIZE - 2)[0] == crc16(chunk[:-2]):
                    t = Tel(*struct.unpack(TEL_FMT, chunk))
                    out.append(t)
                    i += TEL_SIZE
                    continue
            i += 1
        del self.buf[:i]
        if len(self.buf) > 4096:
            self.buf.clear()
        return out


class BridgeLink:
    """Lanza el puente (por SSH o local) y habla con él por stdin/stdout."""

    def __init__(self, cmd):
        if isinstance(cmd, str):
            cmd = shlex.split(cmd)
        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        self._parser = TelParser()
        self._lock = threading.Lock()
        self._tel: Tel | None = None
        self.n_tel = 0
        self._alive = True
        threading.Thread(target=self._read_out, daemon=True).start()
        threading.Thread(target=self._read_err, daemon=True).start()

    def _read_out(self):
        fd = self.proc.stdout.fileno()
        while self._alive:
            try:
                data = os.read(fd, 4096)
            except OSError:
                break
            if not data:
                break
            for t in self._parser.feed(data):
                with self._lock:
                    self._tel = t
                    self.n_tel += 1

    def _read_err(self):
        for line in iter(self.proc.stderr.readline, b""):
            sys.stderr.write("  " + line.decode(errors="replace"))

    def latest(self) -> Tel | None:
        with self._lock:
            return self._tel

    def wait_tel(self, timeout=10.0) -> Tel:
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout:
            t = self.latest()
            if t is not None:
                return t
            if self.proc.poll() is not None:
                raise RuntimeError("el puente terminó antes de enviar telemetría (¿OpenCR conectado?)")
            time.sleep(0.01)
        raise TimeoutError("sin telemetría del puente")

    def send_ref(self, seq, enable, eta_c, deta_c, tau_ff, kd):
        self.proc.stdin.write(pack_ref(seq, enable, eta_c, deta_c, tau_ff, kd))

    def close(self):
        try:
            for k in range(3):
                self.send_ref(0xFFFF0000 + k, False, (0, 0), (0, 0), (0, 0), 0.0)
        except Exception:
            pass
        self._alive = False
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=3)
        except Exception:
            self.proc.kill()


# ===================================================================== modelo
@dataclass
class Params:
    """Parámetros del modelo dinámico calculados a partir de znnCD_control.py."""
    R: float = 0.033          # radio de rueda [m]
    L: float = 0.144          # mitad de distancia entre ruedas [m]
    m_eff: float = 7.4758     # m + 2*Iw/R^2 [kg] (mc=1.37291, mw=0.02849, Iw=0.003292)
    J_eff: float = 0.21398    # I + 2*L^2*Iw/R^2 [kg m^2] (Ic=0.07525, d=-0.064)
    mu: float = -0.08787      # mc * d [kg m]
    d_r: float = 0.01         # fricción viscosa [N m / (rad/s)] (ajustable)
    d_l: float = 0.01
    c_r: float = 0.01         # fricción de Coulomb [N m] (ajustable)
    c_l: float = 0.01
    eps: float = 0.1          # ancho de tanh [rad/s]

    @staticmethod
    def from_json(path):
        with open(path) as f:
            return Params(**json.load(f))

    def to_json(self, path):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)


def wheel_to_body(p: Params, eta):
    v = p.R * (eta[0] + eta[1]) / 2.0
    w = p.R * (eta[0] - eta[1]) / (2.0 * p.L)
    return v, w


def body_to_wheel(p: Params, v, w):
    return np.array([(v + p.L * w) / p.R, (v - p.L * w) / p.R])


def friction(p: Params, eta):
    return np.array([
        p.d_r * eta[0] + p.c_r * math.tanh(eta[0] / p.eps),
        p.d_l * eta[1] + p.c_l * math.tanh(eta[1] / p.eps),
    ])


def tau_ff(p: Params, eta_c, deta_c):
    """Torque de feedforward (idéntico a T1 y T2 de znnCD_control.py + fricción)."""
    v, w = wheel_to_body(p, eta_c)
    dv, dw = wheel_to_body(p, deta_c)
    T1 = p.m_eff * dv - p.mu * (w ** 2)
    T2 = p.J_eff * dw + p.mu * v * w
    tau_R = (p.R / 2.0) * (T1 + (1.0 / p.L) * T2)
    tau_L = (p.R / 2.0) * (T1 - (1.0 / p.L) * T2)
    return np.array([tau_R, tau_L]) + friction(p, eta_c)


def tau_ff_lagrange(p: Params, eta_c, deta_c):
    """Forma matricial equivalente para verificación cruzada."""
    p1 = p.R**2 * (p.m_eff / 4 + p.J_eff / (4 * p.L**2))
    p2 = p.R**2 * (p.m_eff / 4 - p.J_eff / (4 * p.L**2))
    p3 = p.R**2 * p.mu / (2 * p.L)
    M = np.array([[p1, p2], [p2, p1]])
    w = p.R * (eta_c[0] - eta_c[1]) / (2 * p.L)
    V = p3 * w * np.array([[0, 1], [-1, 0]])
    return M @ np.asarray(deta_c) + V @ np.asarray(eta_c) + friction(p, eta_c)


# ===================================================================== Tu ZNN
class ZNNTracker:
    """Controlador Cinemático ZNN adaptado de tu archivo znnCD_control.py.

    Conserva exactamente tu ley:
        X_h = [x + D cos(th), y + D sin(th)]^T
        U   = X_d_dot + K_znn (X_d - X_h)
        e   = T q - U
        q_dot = T_inv (X_d_ddot - T_dot q - eta_znn * e)
    y la interfaz step(pose, ref, dt) -> (u, du).
    """

    def __init__(self, ell=0.08, gamma=3.0, kp=8.0, v_max=0.26, w_max=1.82):
        self.D = ell                  # Distancia D al punto desacoplado [m]
        self.eta_znn = gamma          # Ganancia de convergencia eta de la ZNN (3.0)
        self.K_znn = np.eye(2) * kp   # Matriz de ganancia K_znn (8.0 * I)
        self.v_max = v_max
        self.w_max = w_max
        self.q = np.zeros((2, 1))     # Estado interno q = [[v], [omega]]

    def step(self, pose, ref, dt):
        x, y, th = pose
        pr, dpr, ddpr = ref

        X_d = np.asarray(pr, dtype=float).reshape(2, 1)
        X_d_dot = np.asarray(dpr, dtype=float).reshape(2, 1)
        X_d_ddot = np.asarray(ddpr, dtype=float).reshape(2, 1)

        c, s, D = math.cos(th), math.sin(th), self.D

        # Matrices T, T_inv y T_dot de tu formulación
        T = np.array([
            [c, -D * s],
            [s,  D * c]
        ])
        T_inv = np.array([
            [c, s],
            [-(1.0 / D) * s, (1.0 / D) * c]
        ])
        omega = float(self.q[1, 0])
        T_dot = np.array([
            [-s * omega, -D * c * omega],
            [ c * omega, -D * s * omega]
        ])

        # Posición actual del punto de control X_h
        X_h = np.array([
            [x + D * c],
            [y + D * s]
        ])

        # Ecuación de la ZNN
        X_h_dot = T @ self.q
        U = X_d_dot + self.K_znn @ (X_d - X_h)
        e = X_h_dot - U
        gamma_e = e  # Función de activación lineal

        q_dot = T_inv @ (X_d_ddot - (T_dot @ self.q) - self.eta_znn * gamma_e)

        # Integración de Euler y saturación física
        self.q = self.q + q_dot * dt
        self.q[0, 0] = np.clip(self.q[0, 0], -self.v_max, self.v_max)
        self.q[1, 0] = np.clip(self.q[1, 0], -self.w_max, self.w_max)

        u = np.array([float(self.q[0, 0]), float(self.q[1, 0])])
        du = np.array([float(q_dot[0, 0]), float(q_dot[1, 0])])
        return u, du


# ===================================================================== trayectorias
class Circle:
    """Círculo para el punto P, arrancando en (ell, 0) con rumbo +x."""

    def __init__(self, ell=0.08, r=0.40, speed=0.10):
        self.ell, self.r, self.Om = ell, r, speed / r

    def __call__(self, t):
        r, Om = self.r, self.Om
        p = [self.ell + r * math.sin(Om * t), r * (1 - math.cos(Om * t))]
        dp = [r * Om * math.cos(Om * t), r * Om * math.sin(Om * t)]
        ddp = [-r * Om**2 * math.sin(Om * t), r * Om**2 * math.cos(Om * t)]
        return p, dp, ddp


class SplineTraj:
    """Spline cúbico de tu znnCD_control.py con escalado automático de tiempo seguro K."""

    def __init__(self, ell=0.08, v_max=0.26, w_max=1.82):
        from scipy.interpolate import CubicSpline

        t_wp = np.array([0, 1, 2, 3, 4], dtype=float) * 5.0
        x_wp = np.array([ell, 1.0, -1.0, -1.0, 1.0], dtype=float)
        y_wp = np.array([0.0, 1.0,  1.0, -1.0, -1.0], dtype=float)

        cs_x_orig = CubicSpline(t_wp, x_wp, bc_type="natural")
        cs_y_orig = CubicSpline(t_wp, y_wp, bc_type="natural")
        t_smooth = np.linspace(t_wp[0], t_wp[-1], 200)

        dx, dy = cs_x_orig(t_smooth, 1), cs_y_orig(t_smooth, 1)
        ddx, ddy = cs_x_orig(t_smooth, 2), cs_y_orig(t_smooth, 2)

        v = np.sqrt(dx**2 + dy**2)
        denom = dx**2 + dy**2
        denom[denom == 0] = 1e-6
        omega = (dx * ddy - dy * ddx) / denom

        factor_v = np.max(np.abs(v)) / v_max
        factor_omega = np.max(np.abs(omega)) / w_max
        K = max(1.0, factor_v, factor_omega) * 2.0

        t_seguro = t_wp * K
        self.cs_x = CubicSpline(t_seguro, x_wp, bc_type="natural")
        self.cs_y = CubicSpline(t_seguro, y_wp, bc_type="natural")
        self.tf_seguro = float(t_seguro[-1])
        print(f"[SplineTraj] Factor K: {K:.2f} | Tiempo seguro de trayectoria: {self.tf_seguro:.2f} s")

    def __call__(self, t):
        if t <= self.tf_seguro:
            p = [float(self.cs_x(t)), float(self.cs_y(t))]
            dp = [float(self.cs_x(t, 1)), float(self.cs_y(t, 1))]
            ddp = [float(self.cs_x(t, 2)), float(self.cs_y(t, 2))]
        else:
            p = [float(self.cs_x(self.tf_seguro)), float(self.cs_y(self.tf_seguro))]
            dp = [0.0, 0.0]
            ddp = [0.0, 0.0]
        return p, dp, ddp
