#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, threading, time
import rospy
from std_msgs.msg import Float64

try:
    import tkinter as tk
    from tkinter import ttk
except Exception as e:
    raise SystemExit("Tkinter não disponível. Instale com: sudo apt-get install python3-tk")

N_MOTORS = 6

def make_controller_names(mode):
    if mode == "effort":
        return [f"Engine_helice_{i}_effort_controller" for i in range(1, N_MOTORS+1)]
    else:
        return [f"Engine_helice_{i}_vel_controller" for i in range(1, N_MOTORS+1)]

class MiniGUI:
    def __init__(self, ns, mode, rate_hz, vmin, vmax, resolution):
        self.ns = ns.strip("/")
        self.mode = mode
        self.rate_hz = rate_hz
        self.vmin = vmin
        self.vmax = vmax
        self.resolution = resolution

        rospy.loginfo("GUI iniciando: ns=/%s, mode=%s", self.ns, self.mode)

        # ROS pubs
        names = make_controller_names(self.mode)
        topics = [f"/{self.ns}/{n}/command" for n in names]
        self.pubs = [rospy.Publisher(t, Float64, queue_size=1) for t in topics]
        for t in topics: rospy.loginfo("Publicando em: %s", t)

        # valores alvo
        self.lock = threading.Lock()
        self.vals = [0.0]*N_MOTORS
        self.sine_on = False

        # GUI
        self.root = tk.Tk()
        self.root.title(f"Thrusters GUI — /{self.ns} ({self.mode})")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        info = ttk.Label(self.root, text=f"Namespace: /{self.ns}   •   Modo: {self.mode}   •   Rate: {self.rate_hz:.0f} Hz")
        info.pack(padx=10, pady=(10,5))

        top = ttk.Frame(self.root)
        top.pack(padx=10, pady=5, fill="x")

        self.link_all_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Link All", variable=self.link_all_var, command=self.on_link_toggle).grid(row=0, column=0, sticky="w")

        ttk.Label(top, text="ALL").grid(row=1, column=0, sticky="w")
        self.all_scale = tk.Scale(top, from_=self.vmin, to=self.vmax, resolution=self.resolution,
                                  orient="horizontal", length=420, command=self.on_all_change)
        self.all_scale.grid(row=1, column=1, padx=6, pady=2)

        # sliders individuais
        self.scales = []
        grid = ttk.Frame(self.root)
        grid.pack(padx=10, pady=5, fill="x")

        for i in range(N_MOTORS):
            row = i // 3
            col = i % 3
            cell = ttk.Frame(grid, borderwidth=1, relief="flat")
            cell.grid(row=row, column=col, padx=6, pady=4, sticky="ew")
            ttk.Label(cell, text=f"Motor {i+1}").pack(anchor="w")
            sc = tk.Scale(cell, from_=self.vmin, to=self.vmax, resolution=self.resolution,
                          orient="horizontal", length=300,
                          command=lambda v, idx=i: self.on_one_change(idx, v))
            sc.pack()
            self.scales.append(sc)

        # botões
        btns = ttk.Frame(self.root)
        btns.pack(padx=10, pady=8, fill="x")
        ttk.Button(btns, text="Zero All", command=self.zero_all).pack(side="left", padx=4)
        ttk.Button(btns, text="Sine ▶", command=self.toggle_sine).pack(side="left", padx=4)

        # campos sine
        self.amp_var = tk.DoubleVar(value=min(0.6 if self.mode=="effort" else 60.0, self.vmax))
        self.freq_var = tk.DoubleVar(value=0.5)
        ttk.Label(btns, text="Amp").pack(side="left"); tk.Entry(btns, width=6, textvariable=self.amp_var).pack(side="left")
        ttk.Label(btns, text="Freq(Hz)").pack(side="left"); tk.Entry(btns, width=6, textvariable=self.freq_var).pack(side="left")

        # thread de publicação
        self._stop = threading.Event()
        self.pub_th = threading.Thread(target=self.publish_loop, daemon=True)
        self.pub_th.start()

    # GUI callbacks
    def on_all_change(self, value):
        if not self.link_all_var.get(): return
        try: v = float(value)
        except: return
        with self.lock:
            self.vals = [v]*N_MOTORS
        for sc in self.scales: sc.set(v)

    def on_one_change(self, idx, value):
        try: v = float(value)
        except: return
        with self.lock:
            if self.link_all_var.get():
                # manter coerência: atualizar ALL também
                self.vals = [v]*N_MOTORS
                self.all_scale.set(v)
                for i, sc in enumerate(self.scales):
                    if i != idx: sc.set(v)
            else:
                self.vals[idx] = v

    def on_link_toggle(self):
        if self.link_all_var.get():
            # sincroniza todos com ALL
            v = self.all_scale.get()
            with self.lock:
                self.vals = [v]*N_MOTORS
            for sc in self.scales: sc.set(v)

    def zero_all(self):
        with self.lock:
            self.vals = [0.0]*N_MOTORS
        self.all_scale.set(0.0)
        for sc in self.scales: sc.set(0.0)

    def toggle_sine(self):
        if not self.sine_on:
            self.sine_on = True
            threading.Thread(target=self.sine_loop, daemon=True).start()
        else:
            self.sine_on = False

    # threads
    def sine_loop(self):
        t0 = time.time()
        while self.sine_on and not rospy.is_shutdown():
            amp = float(self.amp_var.get())
            freq = max(0.0, float(self.freq_var.get()))
            val = amp * __import__("math").sin(2.0*__import__("math").pi*freq*(time.time()-t0))
            with self.lock:
                self.vals = [val]*N_MOTORS if self.link_all_var.get() else self.vals
            if self.link_all_var.get():
                self.all_scale.set(val)
                for sc in self.scales: sc.set(val)
            time.sleep(0.02)

    def publish_loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self._stop.is_set():
            with self.lock:
                arr = list(self.vals)
            for p, v in zip(self.pubs, arr):
                p.publish(Float64(v))
            rate.sleep()

    def on_close(self):
        self._stop.set()
        # zera e encerra
        for p in self.pubs: p.publish(Float64(0.0))
        self.root.after(150, self.root.destroy)
        rospy.signal_shutdown("GUI fechada")

    def run(self):
        self.root.mainloop()

def main():
    parser = argparse.ArgumentParser(description="GUI minimalista p/ 6 thrusters (ROS).")
    parser.add_argument("--ns",   default="migbot1", help="namespace do robô (default: migbot1)")
    parser.add_argument("--mode", default="effort", choices=["effort","velocity"], help="tipo de controlador")
    parser.add_argument("--rate", type=float, default=20.0, help="taxa de publicação (Hz)")
    # Faixas úteis — ajuste se quiser:
    parser.add_argument("--min",  dest="vmin", type=float, default=-100.0, help="mínimo slider")
    parser.add_argument("--max",  dest="vmax", type=float, default= 100.0, help="máximo slider")
    parser.add_argument("--res",  dest="resolution", type=float, default=0.1, help="resolução do slider")
    args = parser.parse_args()

    rospy.init_node("thruster_minigui", anonymous=False, disable_signals=True)

    gui = MiniGUI(ns=args.ns, mode=args.mode, rate_hz=args.rate,
                  vmin=args.vmin, vmax=args.vmax, resolution=args.resolution)
    gui.run()

if __name__ == "__main__":
    main()

