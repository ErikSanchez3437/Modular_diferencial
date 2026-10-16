# Control por torque con ZNN en la laptop y puente por SSH (Opción B)

```
Laptop (Python, 100 Hz)                      Raspberry Pi 4                     OpenCR
 trayectoria -> ZNN -> (eta_c, deta_c)  ssh  bridge (C++, 1 kHz)       USB     TorqueControl.ino
 modelo dinámico -> tau_ff   ----------->  tau = tau_ff + kd (eta_c - eta)  --> corriente -> XM430
                              <-----------  telemetría 50 Hz (eta, phi, corriente, odometría, IMU)
```

La laptop calcula ZNN y torque (como en el informe). La Pi mantiene el lazo rápido: extrapola
`eta_c` con `deta_c`, suma una realimentación de velocidad local, convierte a corriente y la
reenvía a 1 kHz (el firmware pone la corriente a cero si pasan >10 ms sin comando).

## Instalación en la Pi
1. Copiar `bridge/` dentro del repo `TurtleTorqueNew/` (junto a `include/`, `src/`, `SerialMessageObj.h`).
2. `cd TurtleTorqueNew/bridge && mkdir build && cd build && cmake .. && make -j4`
3. En la laptop: `ssh-copy-id pi@IP` (entrada sin contraseña) y `pip install numpy scipy matplotlib`.
4. Cargar `TorqueControl.ino` en el OpenCR. `dialout`: `sudo usermod -aG dialout $USER`.

## Prueba sin hardware (verifica todo el lazo)
```
cd laptop
python run_robot.py --cmd "../bridge/build/bridge --sim" --duration 25 --plot sim.png
```

## Primera prueba con el robot (ruedas en el aire)
1. `python run_robot.py --host pi@IP --duration 8 --bridge-args "--imax 40"`
2. Revisa en la consola `[bridge] eta=[R L]`: con avance positivo ambas deben ser **positivas**.
   Si una sale negativa, usa `--bridge-args "--sign-l -1"` (o `--sign-r -1`).
3. Sube `--imax` poco a poco (cada unidad = 2.69 mA). Después prueba en el suelo.
4. Cada corrida guarda `run_log.csv` (referencias, medidas, corriente ordenada y medida):
   sirve para identificar `m_eff`, `J_eff` y fricción por mínimos cuadrados.

## Ajustes
- `ddmr.Params` (o `--params archivo.json`): valores de partida, **hay que identificarlos**.
- `--kd`: realimentación local [N m s/rad]. `--ktau` del puente: 1.05 N m/A, igual que `motor_status.cpp`.
- Si el firmware corre el bus Dynamixel a 115200 bps (`DESIRED_BAUDRATE = 2`), revisa la
  cadencia real de telemetría; si es lenta, sube `DESIRED_BAUDRATE` a 5 (3 Mbps).
