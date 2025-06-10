import tkinter as tk
from tkinter import ttk
import requests
import threading
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

ESP_HOST = "http://192.168.0.102"
POLL_MS  = 100

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 PID Tuner/Speed Monitor (Wi-Fi)")

        # --- PID sliders
        tk.Label(root, text="Kp:").grid(row=0, column=0, sticky="e")
        self.kp_scale = tk.Scale(root, from_=0, to=10, resolution=0.1, orient=tk.HORIZONTAL, length=200,
                                 command=self.on_pid_change)
        self.kp_scale.set(2.0)
        self.kp_scale.grid(row=0, column=1)
        tk.Label(root, text="Ki:").grid(row=0, column=2, sticky="e")
        self.ki_scale = tk.Scale(root, from_=0, to=10, resolution=0.1, orient=tk.HORIZONTAL, length=200,
                                 command=self.on_pid_change)
        self.ki_scale.set(2.5)
        self.ki_scale.grid(row=0, column=3)
        tk.Label(root, text="Kd:").grid(row=1, column=0, sticky="e")
        self.kd_scale = tk.Scale(root, from_=0, to=10, resolution=0.01, orient=tk.HORIZONTAL, length=200,
                                 command=self.on_pid_change)
        self.kd_scale.set(0.0)
        self.kd_scale.grid(row=1, column=1)
        tk.Label(root, text="Kff:").grid(row=1, column=2, sticky="e")
        self.kff_scale = tk.Scale(root, from_=0, to=1, resolution=0.05, orient=tk.HORIZONTAL, length=200,
                                  command=self.on_pid_change)
        self.kff_scale.set(0.3)
        self.kff_scale.grid(row=1, column=3)

        # --- Target speed sliders (V, W)
        tk.Label(root, text="Target V (мм/с):").grid(row=2, column=0, columnspan=2, sticky="e")
        self.target_v_scale = tk.Scale(root, from_=-500, to=500, resolution=1, orient=tk.HORIZONTAL, length=200,
                                       command=self.on_vw_change)
        self.target_v_scale.set(0)
        self.target_v_scale.grid(row=2, column=2)
        tk.Label(root, text="Target W (рад/с):").grid(row=3, column=0, columnspan=2, sticky="e")
        self.target_w_scale = tk.Scale(root, from_=-10, to=10, resolution=0.01, orient=tk.HORIZONTAL, length=200,
                                       command=self.on_vw_change)
        self.target_w_scale.set(0)
        self.target_w_scale.grid(row=3, column=2)

        # --- Reset button
        reset_btn = ttk.Button(root, text="Сбросить всё", command=self.reset_all)
        reset_btn.grid(row=4, column=0, columnspan=4, pady=(8,4), sticky="ew")

        # --- Text status
        self.status_text = tk.Text(root, height=5, width=80)
        self.status_text.grid(row=5, column=0, columnspan=4, pady=5)
        
        # --- Current values (odometry, speeds, PWM, VW)
        self.speed_l = tk.StringVar(value="—")
        self.speed_r = tk.StringVar(value="—")
        self.target_l = tk.StringVar(value="—")
        self.target_r = tk.StringVar(value="—")
        self.pwm_l = tk.StringVar(value="—")
        self.pwm_r = tk.StringVar(value="—")
        self.odom_x = tk.StringVar(value="—")
        self.odom_y = tk.StringVar(value="—")
        self.odom_th = tk.StringVar(value="—")
        self.lin_v = tk.StringVar(value="—")
        self.ang_w = tk.StringVar(value="—")

        info_row = 6
        ttk.Label(root, text="V L [мм/с]").grid(row=info_row, column=0, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.speed_l, width=10).grid(row=info_row, column=1, sticky="w")
        ttk.Label(root, text="V R [мм/с]").grid(row=info_row, column=2, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.speed_r, width=10).grid(row=info_row, column=3, sticky="w")
        info_row += 1
        ttk.Label(root, text="Target L [мм/с]").grid(row=info_row, column=0, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.target_l, width=10).grid(row=info_row, column=1, sticky="w")
        ttk.Label(root, text="Target R [мм/с]").grid(row=info_row, column=2, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.target_r, width=10).grid(row=info_row, column=3, sticky="w")
        info_row += 1
        ttk.Label(root, text="PWM L").grid(row=info_row, column=0, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.pwm_l, width=10).grid(row=info_row, column=1, sticky="w")
        ttk.Label(root, text="PWM R").grid(row=info_row, column=2, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.pwm_r, width=10).grid(row=info_row, column=3, sticky="w")
        info_row += 1
        ttk.Label(root, text="x [м]").grid(row=info_row, column=0, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.odom_x, width=10).grid(row=info_row, column=1, sticky="w")
        ttk.Label(root, text="y [м]").grid(row=info_row, column=2, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.odom_y, width=10).grid(row=info_row, column=3, sticky="w")
        info_row += 1
        ttk.Label(root, text="θ [рад]").grid(row=info_row, column=0, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.odom_th, width=10).grid(row=info_row, column=1, sticky="w")
        info_row += 1
        ttk.Label(root, text="V [мм/с] (центр)").grid(row=info_row, column=2, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.lin_v, width=10).grid(row=info_row, column=3, sticky="w")
        info_row += 1
        ttk.Label(root, text="W [рад/с]").grid(row=info_row, column=2, sticky="e", padx=6)
        ttk.Label(root, textvariable=self.ang_w, width=10).grid(row=info_row, column=3, sticky="w")

        # --- Matplotlib graph
        self.figure = Figure(figsize=(12,4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Wheel Speed Response")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (mm/s)")
        self.canvas = FigureCanvasTkAgg(self.figure, master=root)
        self.canvas.get_tk_widget().grid(row=20, column=0, columnspan=4, pady=10)

        # --- Data for plot
        self.time_data = []
        self.measured_left_data = []
        self.target_left_data = []
        self.measured_right_data = []
        self.target_right_data = []
        self.center_v_data = []
        self.center_w_data = []
        self.start_time = time.time()

        # --- Start polling in background
        self._lock = threading.Lock()
        self._stop = False
        threading.Thread(target=self.poll_state, daemon=True).start()
        self.update_plot()

    def log(self, text):
        self.status_text.insert(tk.END, text + "\n")
        self.status_text.see(tk.END)
        
    def on_pid_change(self, value=None):
        params = {
            "kp": self.kp_scale.get(),
            "ki": self.ki_scale.get(),
            "kd": self.kd_scale.get(),
            "kff": self.kff_scale.get()
        }
        try:
            requests.get(ESP_HOST + "/setPID", params=params, timeout=0.3)
            self.log(f"Sent: /setPID {params}")
        except Exception as e:
            self.log("Ошибка отправки PID: " + str(e))

    def on_vw_change(self, value=None):
        params = {
            "v": self.target_v_scale.get(),
            "w": self.target_w_scale.get()
        }
        try:
            requests.get(ESP_HOST + "/setVW", params=params, timeout=0.3)
            self.log(f"Sent: /setVW {params}")
        except Exception as e:
            self.log("Ошибка отправки V/W: " + str(e))

    def reset_all(self):
        try:
            requests.get(ESP_HOST + "/resetAll", timeout=0.3)
            self.log("Sent: /resetAll")
        except Exception as e:
            self.log("Ошибка сброса: " + str(e))
        # Сбросить график
        with self._lock:
            self.time_data.clear()
            self.measured_left_data.clear()
            self.target_left_data.clear()
            self.measured_right_data.clear()
            self.target_right_data.clear()
            self.center_v_data.clear()
            self.center_w_data.clear()
        self.start_time = time.time()

    def poll_state(self):
        while not self._stop:
            try:
                resp = requests.get(ESP_HOST + "/state", timeout=0.3)
                data = resp.json()
                # Скорость колес
                v_l = data.get("speed", {}).get("left", 0.0)
                v_r = data.get("speed", {}).get("right", 0.0)
                # PWM
                pwm_l = data.get("pwm", {}).get("L", 0.0)
                pwm_r = data.get("pwm", {}).get("R", 0.0)
                # Target
                target_l = data.get("pid", {}).get("targetL", 0.0)
                target_r = data.get("pid", {}).get("targetR", 0.0)
                # Odometry
                odom = data.get("odom", {})
                # VW (линейная и угловая)
                vw = data.get("vw", {})
                v_center = vw.get("v", 0.0)
                w_center = vw.get("w", 0.0)
                # Обновить значения на экране
                self.speed_l.set(f"{v_l:.1f}")
                self.speed_r.set(f"{v_r:.1f}")
                self.target_l.set(f"{target_l:.1f}")
                self.target_r.set(f"{target_r:.1f}")
                self.pwm_l.set(f"{pwm_l:.1f}")
                self.pwm_r.set(f"{pwm_r:.1f}")
                self.odom_x.set(f"{odom.get('x', 0):.3f}")
                self.odom_y.set(f"{odom.get('y', 0):.3f}")
                self.odom_th.set(f"{odom.get('theta', 0):.3f}")
                self.lin_v.set(f"{v_center:.1f}")
                self.ang_w.set(f"{w_center:.3f}")

                # Данные для графика
                t = time.time() - self.start_time
                with self._lock:
                    self.time_data.append(t)
                    self.measured_left_data.append(v_l)
                    self.target_left_data.append(target_l)
                    self.measured_right_data.append(v_r)
                    self.target_right_data.append(target_r)
                    self.center_v_data.append(v_center)
                    self.center_w_data.append(w_center)
                    # Ограничить историю графика (например, 1000 точек)
                    if len(self.time_data) > 1000:
                        self.time_data = self.time_data[-1000:]
                        self.measured_left_data = self.measured_left_data[-1000:]
                        self.target_left_data = self.target_left_data[-1000:]
                        self.measured_right_data = self.measured_right_data[-1000:]
                        self.target_right_data = self.target_right_data[-1000:]
                        self.center_v_data = self.center_v_data[-1000:]
                        self.center_w_data = self.center_w_data[-1000:]
            except Exception as e:
                self.speed_l.set("—")
                self.speed_r.set("—")
                self.target_l.set("—")
                self.target_r.set("—")
                self.pwm_l.set("—")
                self.pwm_r.set("—")
                self.odom_x.set("—")
                self.odom_y.set("—")
                self.odom_th.set("—")
                self.lin_v.set("—")
                self.ang_w.set("—")
            time.sleep(POLL_MS / 1000.0)

    def update_plot(self):
        with self._lock:
            self.ax.clear()
            self.ax.set_title("Wheel Speed & Robot V/W")
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Speed (mm/s) | W (rad/s)")
            self.ax.plot(self.time_data, self.measured_left_data, label="Measured Left")
            self.ax.plot(self.time_data, self.target_left_data, label="Target Left", linestyle="--")
            self.ax.plot(self.time_data, self.measured_right_data, label="Measured Right")
            self.ax.plot(self.time_data, self.target_right_data, label="Target Right", linestyle="--")
            # Отдельно добавить V и W (линейная и угловая скорость) другим цветом/осью
            if self.center_v_data:
                self.ax.plot(self.time_data, self.center_v_data, label="V Center (мм/с)", color="black", linestyle=":")
            if self.center_w_data:
                # Масштабируем W для видимости (например, *100)
                w_scaled = [w*100.0 for w in self.center_w_data]
                self.ax.plot(self.time_data, w_scaled, label="W (рад/с)*100", color="magenta", linestyle=":")
            self.ax.legend()
            # Окно по 20 секунд
            if self.time_data:
                current_time = self.time_data[-1]
                if current_time < 20:
                    self.ax.set_xlim(0, 20)
                else:
                    self.ax.set_xlim(current_time - 20, current_time)
        self.canvas.draw()
        self.root.after(1000, self.update_plot)

    def on_closing(self):
        self._stop = True
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()