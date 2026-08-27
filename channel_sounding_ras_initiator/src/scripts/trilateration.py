"""
Trilaterazione 2D con 3 Reflector BLE Channel Sounding (Minimi Quadrati)
========================================================================
Legge le distanze stimate dall'initiator via porta seriale e calcola
la posizione (x, y) del tag usando l'ottimizzazione non lineare per 
resistere al rumore radio (multipath).

Dipendenze:
    pip install pyserial numpy matplotlib scipy

Uso:
    python trilateration.py --port /dev/ttyACM0 --baud 115200
"""

import argparse
import re
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import deque
import serial
from scipy.optimize import least_squares

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURAZIONE
# ─────────────────────────────────────────────────────────────────────────────
REFLECTOR_POSITIONS = {
    0: (0.0, 0.0),   # Reflector 0 — origine
    1: (3.0, 0.0),   # Reflector 1 
    2: (1.5, 2.5),   # Reflector 2 
}

SMOOTHING_WINDOW = 5
MAX_DISTANCE = 20.0
MIN_DISTANCE = 0.1

# ─────────────────────────────────────────────────────────────────────────────
# TRILATERAZIONE (MINIMI QUADRATI NON LINEARI)
# ─────────────────────────────────────────────────────────────────────────────

def cost_function(point, reflector_positions, measured_distances):
    """Calcola i residui tra le distanze teoriche e quelle misurate."""
    x, y = point
    residuals = []
    # Ordiniamo le chiavi per garantire che i residui siano sempre nello stesso ordine
    for rid in sorted(reflector_positions.keys()):
        if rid in measured_distances:
            rx, ry = reflector_positions[rid]
            theoretical_dist = np.sqrt((x - rx)**2 + (y - ry)**2)
            residuals.append(theoretical_dist - measured_distances[rid])
    return residuals

def trilaterate_least_squares(positions, distances, initial_guess=(1.5, 1.0)):
    """
    Calcola la posizione 2D tramite minimi quadrati.
    Ritorna (x, y) oppure None.
    """
    if len(distances) < 3:
        return None

    result = least_squares(cost_function, initial_guess, args=(positions, distances))
    
    if result.success:
        return float(result.x[0]), float(result.x[1])
    return None

# ─────────────────────────────────────────────────────────────────────────────
# VISUALIZZAZIONE
# ─────────────────────────────────────────────────────────────────────────────

class LivePlot:
    def __init__(self, reflector_positions):
        self.positions = reflector_positions
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 7))
        self.tag_scatter = None
        self.history = deque(maxlen=30)
        self._setup()

    def _setup(self):
        self.ax.set_title("Trilaterazione BLE CS (Minimi Quadrati)", fontsize=13)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.grid(True, linestyle="--", alpha=0.5)

        for rid, (rx, ry) in self.positions.items():
            self.ax.plot(rx, ry, "bs", markersize=12, zorder=5)
            self.ax.annotate(f"R{rid}\n({rx},{ry})", (rx, ry),
                             textcoords="offset points", xytext=(8, 8),
                             fontsize=9, color="blue")

        xs = [p[0] for p in self.positions.values()]
        ys = [p[1] for p in self.positions.values()]
        margin = 2.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)

        self.circles = {}
        self.fig.canvas.draw()

    def update(self, tag_pos, distances):
        tx, ty = tag_pos
        self.history.append((tx, ty))

        for c in self.circles.values():
            c.remove()
        self.circles.clear()

        for rid, (rx, ry) in self.positions.items():
            d = distances.get(rid, 0)
            circle = patches.Circle((rx, ry), d, fill=False,
                                    linestyle="--", alpha=0.3,
                                    color=["red", "green", "orange"][rid % 3])
            self.ax.add_patch(circle)
            self.circles[rid] = circle

        hx = [p[0] for p in self.history]
        hy = [p[1] for p in self.history]
        if self.tag_scatter:
            self.tag_scatter.remove()
        self.tag_scatter = self.ax.scatter(hx, hy,
                                           c=range(len(hx)),
                                           cmap="autumn", s=40, zorder=6)

        self.ax.plot(tx, ty, "r*", markersize=18, zorder=7)

        self.ax.set_title(
            f"Tag stimato: ({tx:.2f} m, {ty:.2f} m)  —  "
            f"d0={distances.get(0,0):.2f}  d1={distances.get(1,0):.2f}  d2={distances.get(2,0):.2f}",
            fontsize=11
        )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

# ─────────────────────────────────────────────────────────────────────────────
# LETTURA SERIALE E LOGICA PRINCIPALE
# ─────────────────────────────────────────────────────────────────────────────

class DistanceBuffer:
    def __init__(self, window=SMOOTHING_WINDOW):
        self.window = window
        self.buffers = {i: deque(maxlen=window) for i in REFLECTOR_POSITIONS}
        self.pending_id = None

    def set_pending(self, reflector_id):
        self.pending_id = reflector_id

    def add_distance(self, raw_dist):
        if self.pending_id is None:
            return
        if MIN_DISTANCE <= raw_dist <= MAX_DISTANCE:
            self.buffers[self.pending_id].append(raw_dist)
        else:
            print(f"[FILTER] Distanza fuori range ignorata: {raw_dist:.3f} m (R{self.pending_id})")
        self.pending_id = None

    def get_smoothed(self):
        result = {}
        for rid, buf in self.buffers.items():
            if not buf:
                return None
            result[rid] = float(np.mean(buf))
        return result

    def ready(self):
        return all(len(b) > 0 for b in self.buffers.values())


def parse_line(line, buf):
    line = line.strip()

    # CS_DATA:REFLECTOR_ID:N|MAC:...
    m = re.match(r"CS_DATA:REFLECTOR_ID:(\d+)\|", line)
    if m:
        rid = int(m.group(1))
        if rid in REFLECTOR_POSITIONS:
            buf.set_pending(rid)
        return False

    # Correzione fondamentale applicata qui: DIST_DATA:RAW_VAL:<valore>
    m = re.match(r"DIST_DATA:RAW_VAL:([\d.eE+\-]+)", line)
    if m:
        try:
            d = float(m.group(1))
            buf.add_distance(d)
            return True
        except ValueError:
            pass

    return False


def run(port, baud, no_plot):
    print(f"[INFO] Apertura porta seriale {port} @ {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[ERRORE] Impossibile aprire la porta {port}: {e}")
        return
        
    time.sleep(0.5)
    print("[INFO] Connesso. In attesa di dati...\n")

    buf = DistanceBuffer(window=SMOOTHING_WINDOW)
    plot = None if no_plot else LivePlot(REFLECTOR_POSITIONS)
    
    # Memorizziamo l'ultima posizione calcolata per usarla come punto di partenza
    # per la successiva ottimizzazione (rende l'algoritmo più veloce e stabile)
    last_pos = (1.5, 1.0) 

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace")
            except Exception:
                continue

            # print(f"[RAW SERIAL]: {line.strip()}")

            got_dist = parse_line(line, buf)

            if got_dist and buf.ready():
                distances = buf.get_smoothed()
                
                # Usiamo i minimi quadrati passando last_pos come stima iniziale
                pos = trilaterate_least_squares(REFLECTOR_POSITIONS, distances, initial_guess=last_pos)

                if pos:
                    x, y = pos
                    last_pos = pos # Aggiorniamo la stima
                    print(f"\r[TAG] x={x:.3f}m, y={y:.3f}m | d0={distances[0]:.2f} d1={distances[1]:.2f} d2={distances[2]:.2f}", end="")
                    
                    if plot:
                        plot.update(pos, distances)
                else:
                    print("\n[WARN] Calcolo posizione fallito.")

    except KeyboardInterrupt:
        print("\n[INFO] Interruzione utente.")
    finally:
        ser.close()
        print("[INFO] Porta seriale chiusa.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Trilaterazione BLE CS (Least Squares)")
    parser.add_argument("--port",    default="COM3", # <-- Sostituisci il default con la tua porta abituale
                        help="Porta seriale (es. /dev/ttyACM0 o COM3)")
    parser.add_argument("--baud",    type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disabilita la visualizzazione grafica")
    args = parser.parse_args()

    run(args.port, args.baud, args.no_plot)