"""ddmr.py -- Lado laptop: protocolo con el puente, modelo dinámico y ZNN.

Convención de arreglos: índice 0 = rueda DERECHA, índice 1 = rueda IZQUIERDA
(eta = [phi_R_dot, phi_L_dot], tau = [tau_R, tau_L], como en el informe).
"""
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
    """Lanza el puente (por SSH o local) y habla con él por stdin/stdout.

    cmd: lista o string. Ejemplo SSH:
        ssh -T -o BatchMode=yes pi@IP /home/pi/TurtleTorqueNew/bridge/build/bridge
    Si la sesión SSH se cae, el puente ve EOF en stdin y frena solo.
    """

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
            for k in range(3):  # enable=0 explícito
                self.send_ref(0xFFFF0000 + k, False, (0, 0), (0, 0), (0, 0), 0.0)
        except Exception:
            pass
        self._alive = False
        try:
            self.proc.stdin.close()  # EOF -> el puente frena y termina
            self.proc.wait(timeout=3)
        except Exception:
            self.proc.kill()


# ===================================================================== modelo
@dataclass
class Params:
    """Parámetros del modelo reparametrizado (VALORES DE PARTIDA: identificar)."""
    R: float = 0.033      # radio de rueda [m]
    L: float = 0.1435     # semi-separación entre ruedas [m]
    m_eff: float = 2.5    # m + 2 I_w / R^2   [kg]
    J_eff: float = 0.05   # I + 2 L^2 I_w / R^2 [kg m^2]
    mu: float = 0.0       # m_c * d  [kg m]
    d_r: float = 0.02     # fricción viscosa [N m / (rad/s)]
    d_l: float = 0.02
    c_r: float = 0.03     # fricción de Coulomb [N m]
    c_l: float = 0.03
    eps: float = 0.1      # ancho de tanh [rad/s]

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
    """Torque de feedforward: M(eta_c') + V(eta_c) eta_c + F(eta_c), forma (v, w)."""
    v, w = wheel_to_body(p, eta_c)
    dv, dw = wheel_to_body(p, deta_c)
    t_sum = p.R * (p.m_eff * dv - p.mu * w * w)               # tau_R + tau_L
    t_dif = (p.R / p.L) * (p.J_eff * dw + p.mu * w * v)       # tau_R - tau_L
    return np.array([(t_sum + t_dif) / 2, (t_sum - t_dif) / 2]) + friction(p, eta_c)


def tau_ff_lagrange(p: Params, eta_c, deta_c):
    """Misma cosa por la forma matricial (46) del paper principal (verificación)."""
    p1 = p.R**2 * (p.m_eff / 4 + p.J_eff / (4 * p.L**2))
    p2 = p.R**2 * (p.m_eff / 4 - p.J_eff / (4 * p.L**2))
    p3 = p.R**2 * p.mu / (2 * p.L)
    M = np.array([[p1, p2], [p2, p1]])
    w = p.R * (eta_c[0] - eta_c[1]) / (2 * p.L)
    V = p3 * w * np.array([[0, 1], [-1, 0]])
    return M @ np.asarray(deta_c) + V @ np.asarray(eta_c) + friction(p, eta_c)


# ===================================================================== ZNN
class ZNNTracker:
    """ZNN de referencia para el punto P desplazado una distancia ell del eje.

    P = [x + ell cos(th), y + ell sin(th)],   dP/dt = A(th) u,   u = [v, w],
    A = [[cos th, -ell sin th], [sin th, ell cos th]]  (det A = ell > 0).

    Ecuación variante en el tiempo  A(t) u = b(t),  b = dp_r + kp (p_r - p).
    Fórmula de diseño de Zhang  eps' = -gamma eps,  eps = A u - b:
        A u' = b' - A' u - gamma (A u - b)
    El estado u da las velocidades y u' las aceleraciones (v', w') que necesita
    el modelo dinámico. SUSTITUIR por la ZNN del informe si difiere: basta con
    conservar la interfaz  step(pose, ref, dt) -> (u, du).
    """

    def __init__(self, ell=0.10, gamma=15.0, kp=2.0, v_max=0.22, w_max=1.8):
        self.ell, self.gamma, self.kp = ell, gamma, kp
        self.v_max, self.w_max = v_max, w_max
        self.u = np.zeros(2)

    def step(self, pose, ref, dt):
        x, y, th = pose
        pr, dpr, ddpr = (np.asarray(a, float) for a in ref)
        c, s, l = math.cos(th), math.sin(th), self.ell
        A = np.array([[c, -l * s], [s, l * c]])
        p = np.array([x + l * c, y + l * s])
        v, w = self.u
        dp = A @ self.u
        b = dpr + self.kp * (pr - p)
        db = ddpr + self.kp * (dpr - dp)
        Adot = w * np.array([[-s, -l * c], [c, -l * s]])
        du = np.linalg.solve(A, db - Adot @ self.u - self.gamma * (A @ self.u - b))
        u_new = self.u + du * dt
        # saturación de velocidades (anti-windup: se anula la aceleración que empuja contra el límite)
        for i, lim in enumerate((self.v_max, self.w_max)):
            if abs(u_new[i]) > lim:
                u_new[i] = math.copysign(lim, u_new[i])
                if u_new[i] * du[i] > 0:
                    du[i] = 0.0
        self.u = u_new
        return self.u.copy(), du


# ===================================================================== trayectorias
class Circle:
    """Círculo para el punto P, arrancando en el origen con rumbo +x."""

    def __init__(self, ell=0.10, r=0.40, speed=0.10):
        self.ell, self.r, self.Om = ell, r, speed / r

    def __call__(self, t):
        r, Om = self.r, self.Om
        p = [self.ell + r * math.sin(Om * t), r * (1 - math.cos(Om * t))]
        dp = [r * Om * math.cos(Om * t), r * Om * math.sin(Om * t)]
        ddp = [-r * Om**2 * math.sin(Om * t), r * Om**2 * math.cos(Om * t)]
        return p, dp, ddp


class SplineTraj:
    """Spline cúbico por waypoints (x, y) del punto P, con tiempos uniformes."""

    def __init__(self, waypoints, T):
        from scipy.interpolate import CubicSpline

        wp = np.asarray(waypoints, float)
        ts = np.linspace(0.0, T, len(wp))
        self.T = T
        # velocidad inicial/final nula: arranque y parada suaves
        self.cs = CubicSpline(ts, wp, bc_type="clamped")

    def __call__(self, t):
        t = min(max(t, 0.0), self.T)
        return self.cs(t), self.cs(t, 1), self.cs(t, 2)
