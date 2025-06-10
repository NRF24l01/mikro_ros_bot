import tkinter as tk
from tkinter import ttk
import requests, json

ESP_HOST = "http://192.168.0.102"      # ← IP вашей ESP32
POLL_MS  = 100                         # период опроса, мс


class Esp32Gui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 PWM / Encoders")

        # ---------------- Энкодеры ----------------
        self.enc_l = tk.StringVar(value="—")
        self.enc_r = tk.StringVar(value="—")

        ttk.Label(self, text="Enc L").grid(row=0, column=0, sticky="e", padx=6)
        ttk.Label(self, textvariable=self.enc_l, width=8
                  ).grid(row=0, column=1, sticky="w")
        ttk.Label(self, text="Enc R").grid(row=1, column=0, sticky="e", padx=6)
        ttk.Label(self, textvariable=self.enc_r, width=8
                  ).grid(row=1, column=1, sticky="w")

        # ---------------- PWM-ползунки -------------
        self.scales, self.dvars = {}, {}
        row = 2
        for name in ("L_A", "L_B", "R_A", "R_B"):
            ttk.Label(self, text=name).grid(row=row, column=0, sticky="e", padx=6)

            sv = tk.IntVar(value=0)             # для цифрового индикатора
            self.dvars[name] = sv
            lbl = ttk.Label(self, textvariable=sv, width=4)
            lbl.grid(row=row, column=2, sticky="w", padx=4)

            scl = ttk.Scale(self, from_=0, to=255, orient="horizontal",
                            command=self.on_change, length=180)
            scl.grid(row=row, column=1, padx=4, pady=4)
            self.scales[name] = scl
            row += 1

        # запускаем цикл опроса
        self.after(200, self.poll_state)

    # ---------- HTTP helpers ----------
    def send_pwm(self, duty):
        try:
            qs = f"/setPWM?la={duty['L_A']}&lb={duty['L_B']}" \
                 f"&ra={duty['R_A']}&rb={duty['R_B']}"
            requests.get(ESP_HOST + qs, timeout=0.3)
        except requests.RequestException:
            pass

    # ---------- callbacks -------------
    def on_change(self, *_):
        duty = {k: int(s.get()) for k, s in self.scales.items()}
        # обновляем локальные цифровые метки
        for k, v in duty.items():
            self.dvars[k].set(v)
        self.send_pwm(duty)

    def poll_state(self):
        try:
            resp = requests.get(ESP_HOST + "/state", timeout=0.3)
            data = resp.json()

            # обновляем энкодеры
            self.enc_l.set(data["enc"]["left"])
            self.enc_r.set(data["enc"]["right"])

            # обновляем PWM-индикаторы/ползунки
            for k, s in self.scales.items():
                val = data["duty"][k]
                self.dvars[k].set(val)
                if abs(s.get() - val) > 1:
                    s.set(val)

        except (requests.RequestException, json.JSONDecodeError):
            self.enc_l.set("—")
            self.enc_r.set("—")

        self.after(POLL_MS, self.poll_state)


if __name__ == "__main__":
    Esp32Gui().mainloop()