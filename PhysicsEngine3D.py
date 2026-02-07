import random
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial

PORT = "COM3"
BAUD = 9600
UPDATE_MS = 150

BASELINE_DANGER = 15.0

BASE_TEMP_C = 26.0
BASE_HUM_PCT = 45.0
TEMP_HIGH_ADD = 8.0
HUM_HIGH_ADD = 25.0
TEMP_JITTER_STD = 0.25
HUM_JITTER_STD = 0.8

W_TEMP = 0.35
W_HUM = 0.65
TEMP_MIN, TEMP_MAX = 18.0, 42.0
HUM_MIN, HUM_MAX = 20.0, 100.0


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def normalize(x, x_min, x_max):
    if x_max == x_min:
        return 0.0
    return clamp((x - x_min) / (x_max - x_min), 0.0, 1.0)


def danger_from_temp_humidity(temp_c: float, hum_pct: float) -> int:
    t = normalize(temp_c, TEMP_MIN, TEMP_MAX)
    h = normalize(hum_pct, HUM_MIN, HUM_MAX)
    risk = W_TEMP * t + W_HUM * h
    synergy = (t * h) ** 0.8
    risk = clamp(risk + 0.20 * synergy, 0.0, 1.0)
    return int(round(1 + 99 * risk))


class SmoothEstimator:
    def __init__(self):
        self.temp = BASE_TEMP_C
        self.hum = BASE_HUM_PCT

    def update(self, temp_bit: int, hum_bit: int):
        target_temp = BASE_TEMP_C + (TEMP_HIGH_ADD if temp_bit else 0.0)
        target_hum = BASE_HUM_PCT + (HUM_HIGH_ADD if hum_bit else 0.0)
        self.temp += 0.15 * (target_temp - self.temp) + random.gauss(0, TEMP_JITTER_STD)
        self.hum += 0.12 * (target_hum - self.hum) + random.gauss(0, HUM_JITTER_STD)
        self.temp = clamp(self.temp, -10.0, 60.0)
        self.hum = clamp(self.hum, 0.0, 100.0)
        return self.temp, self.hum


class SpikeyStable:
    def __init__(
        self,
        baseline=15.0,
        jitter=0.35,
        pull=0.18,
        spike_chance=0.035,
        spike_min=8.0,
        spike_max=45.0,
        spike_decay=0.28,
    ):
        self.baseline = baseline
        self.value = baseline
        self.jitter = jitter
        self.pull = pull
        self.spike_chance = spike_chance
        self.spike_min = spike_min
        self.spike_max = spike_max
        self.spike_decay = spike_decay
        self.spike_offset = 0.0

    def step(self):
        if random.random() < self.spike_chance:
            mag = random.uniform(self.spike_min, self.spike_max)
            direction = 1 if random.random() < 0.9 else -1
            self.spike_offset += direction * mag
        self.spike_offset *= (1.0 - self.spike_decay)
        noise = random.gauss(0.0, self.jitter)
        drift = (self.baseline - self.value) * self.pull
        self.value += drift + noise
        out = self.value + self.spike_offset
        return clamp(out, 1.0, 100.0)


def parse_bits(line: str):
    s = line.strip().replace(";", ",").replace(" ", ",")
    parts = [p for p in s.split(",") if p != ""]
    if len(parts) < 2:
        raise ValueError(f"Need 2 bits, got: {line!r}")
    tb = 1 if int(parts[0]) != 0 else 0
    hb = 1 if int(parts[1]) != 0 else 0
    return tb, hb


def build_plot(root):
    fig = Figure(figsize=(8, 5), dpi=100, facecolor="black")
    ax = fig.add_subplot(111, facecolor="black")

    x_data = list(range(1, 101))
    y_data = [int(BASELINE_DANGER)] * 100

    plot_line, = ax.plot(x_data, y_data, color="white", linewidth=2.5)

    ax.set_ylim(1, 100)
    ax.set_xlim(1, 100)
    ax.set_xlabel("Sample", color="white", fontsize=14)
    ax.set_ylabel("Danger Score (1-100)", color="white", fontsize=14)
    ax.tick_params(axis="x", colors="white", labelsize=12)
    ax.tick_params(axis="y", colors="white", labelsize=12)
    for spine in ax.spines.values():
        spine.set_color("white")
    ax.grid(True, color="#444444", linestyle="--", linewidth=0.5, alpha=0.5)

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)

    return canvas, plot_line, y_data


def run_live_graph(use_sensor: bool):
    ser = None
    if use_sensor:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.0)
        except Exception:
            ser = None

    root = tk.Tk()
    root.title("Danger graph")

    canvas, plot_line, y_data = build_plot(root)

    estimator = SmoothEstimator()
    dummy = SpikeyStable(baseline=BASELINE_DANGER)

    sim_temp_bit = 0
    sim_hum_bit = 0

    def read_bits_or_none():
        if ser is None:
            return None
        try:
            if ser.in_waiting > 0:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if raw:
                    return parse_bits(raw)
        except Exception:
            return None
        return None

    def update():
        nonlocal sim_temp_bit, sim_hum_bit

        bits = read_bits_or_none()
        if bits is not None:
            temp_bit, hum_bit = bits
            temp_c, hum_pct = estimator.update(temp_bit, hum_bit)
            danger = danger_from_temp_humidity(temp_c, hum_pct)
        else:
            if random.random() < 0.02:
                sim_temp_bit ^= 1
            if random.random() < 0.05:
                sim_hum_bit ^= 1
            _ = (sim_temp_bit, sim_hum_bit)
            danger = int(round(dummy.step()))

        y_data.pop(0)
        y_data.append(danger)
        plot_line.set_ydata(y_data)
        canvas.draw()

        root.after(UPDATE_MS, update)

    update()
    root.mainloop()

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


def running(use_sensor: bool):
    run_live_graph(use_sensor)


if __name__ == "__main__":
    running(True)
