#!/usr/bin/env python3
"""
Dashboard LUNATIK - tableau de bord temps reel

Lit les trames telemetrie relayees par le recepteur LoRa sol (Pico
branche en USB sur ce PC) et affiche : etat de la state machine,
altitude et vitesse (Kalman), position GPS, tension batterie, qualite
de liaison (RSSI/SNR), estimation du taux de perte de paquets.

Prerequis :
    pip install pyserial matplotlib

Lancement :
    python dashboard.py                  (detection auto du port)
    python dashboard.py --port COM5      (Windows)
    python dashboard.py --port /dev/ttyACM0   (Linux/Mac)

Format de trame attendu, une ligne ASCII par paquet. Deux variantes
acceptees automatiquement :

  Avec recepteur_sol.py (prefixe TLM + rssi/snr ajoutes cote sol) :
    TLM,t,phase,ax,ay,az,gx,gy,gz,pression_pa,temp_c,alt_baro_m,lat,lon,alt_gps_m,h,v,batt,rssi,snr

  Avec l'ancien code-recepteur.py (relai brut, sans rssi/snr) :
    t,phase,ax,ay,az,gx,gy,gz,pression_pa,temp_c,alt_baro_m,lat,lon,alt_gps_m,h,v,batt

Les 17 champs "t,...,batt" correspondent exactement a la sortie de
datalog.py::_formatter_compact() cote fusee (src/datalog.py), telle
qu'appelee par main.py. t, phase, h (altitude Kalman), v (vitesse
Kalman), lat/lon/alt_gps_m et batt sont affiches ; ax/ay/az/gx/gy/gz/
pression_pa/temp_c/alt_baro_m sont parses mais non utilises pour
l'instant.

NOTE : le champ "sat" (nombre de satellites GPS) n'est PAS transmis
par main.py actuellement (main.py recupere sat via sen.gps_data mais
ne le passe pas a dat.log()/dat._formatter_compact()). Le dashboard
ne peut donc pas s'appuyer dessus pour detecter un fix GPS valide : il
se base a la place sur lat/lon != (0.0, 0.0). Si sat est ajoute un
jour cote firmware, remettre "sat" dans TELEMETRY_FIELDS et repasser
la detection de fix sur ce champ (plus fiable qu'un test sur lat/lon).

Si _formatter_compact() change de forme, seule la liste TELEMETRY_FIELDS
et parse_line() doivent etre adaptees : le reste du script n'en depend pas.
"""

import argparse
import queue
import sys
import threading
import time
import tkinter as tk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

STATE_COLORS = {
    "SETUP": "#455a64",
    "PRE_LAUNCH": "#607d8b",
    "BOOST": "#e53935",
    "COAST": "#fb8c00",
    "APOGEE": "#8e24aa",
    "APOGEEtimer": "#8e24aa",
    "DESCENT": "#1e88e5",
    "LANDED": "#43a047",
}

# Ordre exact des 17 champs envoyes par datalog.py::_formatter_compact()
# cote fusee, tel qu'appele par main.py aujourd'hui (sans sat). Les noms
# ici sont ceux utilises en interne par le dashboard (pas forcement
# identiques aux noms cote firmware).
TELEMETRY_FIELDS = [
    "t", "phase", "ax", "ay", "az", "gx", "gy", "gz",
    "pression_pa", "temp_c", "alt_baro_m", "lat", "lon", "alt_gps_m",
    "h", "v", "batt",
]

MAX_POINTS = 3000  # plusieurs minutes de vol a une dizaine de Hz


def find_pico_port():
    """Essaie de trouver automatiquement le port serie du Pico.
    Renvoie None si rien d'evident n'est trouve."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "pico" in desc or "circuitpython" in desc or p.vid == 0x2E8A:
            return p.device
    return None


def parse_line(raw_line):
    """Parse une ligne brute recue sur le port serie.

    Deux formats acceptes :
      - avec recepteur_sol.py :  TLM,<17 champs _formatter_compact>,<rssi>,<snr>
      - avec l'ancien code-recepteur.py (sans prefixe, sans rssi/snr) :
        <17 champs _formatter_compact>

    Renvoie un dict, ou None si la ligne ne correspond a aucun des deux."""
    line = raw_line.strip()

    has_tlm = line.startswith("TLM,")
    if has_tlm:
        line = line[len("TLM,"):]

    parts = [p.strip() for p in line.split(",")]

    rssi, snr = None, None
    if has_tlm:
        if len(parts) != len(TELEMETRY_FIELDS) + 2:
            return None
        rssi_str, snr_str = parts[-2], parts[-1]
        parts = parts[:-2]
        try:
            rssi, snr = float(rssi_str), float(snr_str)
        except ValueError:
            return None
    else:
        if len(parts) != len(TELEMETRY_FIELDS):
            return None

    telem = dict(zip(TELEMETRY_FIELDS, parts))
    try:
        for key in telem:
            if key != "phase":
                telem[key] = float(telem[key])
    except ValueError:
        return None

    telem["rssi"] = rssi
    telem["snr"] = snr
    return telem


class SerialReader(threading.Thread):
    """Lit le port serie dans un thread separe et pousse les trames
    parsees dans une queue thread-safe. Ne touche JAMAIS a un widget
    Tkinter directement : Tkinter n'est pas thread-safe, toute mise a
    jour d'interface doit passer par le thread principal."""

    def __init__(self, port, baudrate, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.out_queue = out_queue
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baudrate, timeout=1.0) as ser:
                    self.out_queue.put(("status", "Connecte sur {}".format(self.port)))
                    while not self._stop.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue
                        try:
                            text = raw.decode("utf-8", "replace")
                        except Exception:
                            continue
                        telem = parse_line(text)
                        if telem is not None:
                            self.out_queue.put(("telemetry", telem))
                        else:
                            stripped = text.strip()
                            if stripped:
                                self.out_queue.put(("raw", stripped))
            except serial.SerialException as exc:
                self.out_queue.put(("status", "Deconnecte ({}), retry dans 2s".format(exc)))
                time.sleep(2.0)

    def stop(self):
        self._stop.set()


class SimulatedSource(threading.Thread):
    """Genere de fausses trames TLM et les pousse dans la meme queue que
    SerialReader, avec le meme format de tuple. Sert a valider l'interface
    sans avoir besoin de deux Pico qui se parlent en LoRa : le dashboard
    ne voit aucune difference entre cette source et le port serie reel."""

    def __init__(self, out_queue, dt=0.1):
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self.dt = dt
        self._stop = threading.Event()

    def run(self):
        import random

        self.out_queue.put(("status", "Mode simulation (aucun port serie utilise)"))
        t = 0.0
        batt = 4.15
        lat0, lon0 = 43.9493, 4.8055  # Avignon, point de depart fictif
        while not self._stop.is_set():
            # profil de vol simplifie : montee balistique puis retombee
            if t < 2.0:
                phase, h, v = "PRE_LAUNCH", 0.0, 0.0
            elif t < 4.0:
                phase = "BOOST"
                v = 60.0 * (t - 2.0) / 2.0
                h = 30.0 * (t - 2.0) ** 2
            elif t < 8.2:
                phase = "COAST"
                tc = t - 4.0
                v = 60.0 - 9.81 * tc
                h = 120.0 + 60.0 * tc - 0.5 * 9.81 * tc ** 2
            elif t < 9.0:
                phase, v = "APOGEE", 0.5
                h = 289.4
            elif t < 40.0:
                phase = "DESCENT"
                v = -6.5
                h = max(0.0, 289.4 - 6.5 * (t - 9.0))
            else:
                phase, v, h = "LANDED", 0.0, 0.0

            batt = max(3.55, batt - 0.0003)
            rssi = -40 - 0.05 * h + random.uniform(-2, 2)
            snr = 9.5 - 0.01 * h + random.uniform(-0.5, 0.5)

            # derive laterale fictive pendant la descente, fix GPS seulement
            # apres le decollage (comme sur le vrai firmware, PRE_LAUNCH/
            # DESCENT/LANDED seulement)
            if phase in ("PRE_LAUNCH",):
                lat, lon, alt_gps, sat = lat0, lon0, 24.0, 8
            elif phase in ("DESCENT", "LANDED"):
                drift = 0.0003 * (t - 9.0)
                lat, lon, alt_gps, sat = lat0 + drift, lon0 + drift * 0.5, max(24.0, h), 8
            else:
                lat, lon, alt_gps, sat = lat0, lon0, 24.0, 8

            telem = {"t": t, "phase": phase, "h": h, "v": v, "batt": batt,
                     "lat": lat, "lon": lon, "alt_gps_m": alt_gps, "sat": sat,
                     "rssi": rssi, "snr": snr}
            self.out_queue.put(("telemetry", telem))

            t += self.dt
            time.sleep(self.dt)

    def stop(self):
        self._stop.set()


class Dashboard(tk.Tk):
    def __init__(self, port, baudrate, simulate=False):
        super().__init__()
        self.title("LUNATIK - Tableau de bord" + (" [SIMULATION]" if simulate else ""))
        self.geometry("1100x760")
        self.configure(bg="#1e1e1e")

        self.data_queue = queue.Queue()
        if simulate:
            self.reader = SimulatedSource(self.data_queue)
        else:
            self.reader = SerialReader(port, baudrate, self.data_queue)
        self.reader.start()

        self.ts, self.hs, self.vs = [], [], []
        self.total_packets = 0

        self._build_ui()
        self.after(150, self._poll_queue)

        # Sur macOS, une fenetre Tk peut s'ouvrir derriere le terminal/IDE
        # sans jamais passer au premier plan. On la force a s'afficher.
        self.lift()
        self.attributes("-topmost", True)
        self.after(200, lambda: self.attributes("-topmost", False))
        self.focus_force()

    # ---- construction de l'interface ----

    def _build_ui(self):
        top = tk.Frame(self, bg="#1e1e1e")
        top.pack(side="top", fill="x", padx=10, pady=8)

        self.state_label = tk.Label(
            top, text="EN ATTENTE", font=("Consolas", 26, "bold"),
            fg="white", bg="#607d8b", width=16, pady=8,
        )
        self.state_label.pack(side="left")

        info = tk.Frame(top, bg="#1e1e1e")
        info.pack(side="left", padx=20)

        self.status_var = tk.StringVar(value="Non connecte")
        self.alt_var = tk.StringVar(value="Altitude : -- m")
        self.vel_var = tk.StringVar(value="Vitesse  : -- m/s")
        self.gps_var = tk.StringVar(value="GPS : pas de fix")
        self.batt_var = tk.StringVar(value="Batterie : -- V")
        self.link_var = tk.StringVar(value="RSSI -- dBm   SNR -- dB")
        self.loss_var = tk.StringVar(value="Paquets : 0 (0.0% pertes est.)")

        rows = [
            (self.alt_var, 16), (self.vel_var, 16), (self.gps_var, 14),
            (self.batt_var, 14), (self.link_var, 14), (self.loss_var, 12),
        ]
        for var, size in rows:
            tk.Label(info, textvariable=var, font=("Consolas", size),
                     fg="#e0e0e0", bg="#1e1e1e", anchor="w").pack(anchor="w")

        # graphes altitude / vitesse en fonction du temps
        self.fig = Figure(figsize=(9, 4.2), dpi=100, facecolor="#1e1e1e")
        self.ax_h = self.fig.add_subplot(211)
        self.ax_v = self.fig.add_subplot(212, sharex=self.ax_h)
        for ax in (self.ax_h, self.ax_v):
            ax.set_facecolor("#1e1e1e")
            ax.tick_params(colors="#e0e0e0")
            for spine in ax.spines.values():
                spine.set_color("#555555")
        self.ax_h.set_ylabel("Altitude (m)", color="#e0e0e0")
        self.ax_v.set_ylabel("Vitesse (m/s)", color="#e0e0e0")
        self.ax_v.set_xlabel("t (s)", color="#e0e0e0")
        (self.line_h,) = self.ax_h.plot([], [], color="#42a5f5")
        (self.line_v,) = self.ax_v.plot([], [], color="#ef5350")
        self.fig.tight_layout()

        canvas_frame = tk.Frame(self, bg="#1e1e1e")
        canvas_frame.pack(side="top", fill="both", expand=True, padx=10)
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # console : etat de connexion + lignes brutes non reconnues
        # (utile pour deboguer un probleme de format sans que ca plante le parseur)
        log_frame = tk.Frame(self, bg="#1e1e1e")
        log_frame.pack(side="bottom", fill="x", padx=10, pady=6)
        tk.Label(log_frame, textvariable=self.status_var, fg="#9e9e9e",
                 bg="#1e1e1e", font=("Consolas", 9)).pack(anchor="w")
        self.log_text = tk.Text(log_frame, height=6, bg="#111111", fg="#8bc34a",
                                 font=("Consolas", 9), insertbackground="white")
        self.log_text.pack(fill="x")

    # ---- boucle de mise a jour, executee uniquement dans le thread principal ----

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.data_queue.get_nowait()
                if kind == "telemetry":
                    self._on_telemetry(payload)
                elif kind == "raw":
                    self._log(payload)
                elif kind == "status":
                    self.status_var.set(payload)
        except queue.Empty:
            pass
        self.after(150, self._poll_queue)

    def _on_telemetry(self, telem):
        self.total_packets += 1

        phase = telem["phase"]
        self.state_label.config(
            text=phase,
            bg=STATE_COLORS.get(phase, "#424242"),
        )
        self.alt_var.set("Altitude : {:.1f} m".format(telem["h"]))
        self.vel_var.set("Vitesse  : {:+.1f} m/s".format(telem["v"]))

        # main.py ne transmet pas le champ sat pour l'instant (voir note
        # en tete de fichier), donc on detecte un fix GPS via lat/lon
        # plutot que via sat. (0.0, 0.0) ou absent = pas de fix.
        lat = telem.get("lat")
        lon = telem.get("lon")
        if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
            self.gps_var.set("GPS : pas de fix")
        else:
            self.gps_var.set(
                "GPS : {:.5f}, {:.5f}  alt {:.0f} m".format(
                    lat, lon, telem.get("alt_gps_m", 0.0),
                )
            )

        self.batt_var.set("Batterie : {:.2f} V".format(telem["batt"]))
        if telem["rssi"] is not None and telem["snr"] is not None:
            self.link_var.set("RSSI {:.0f} dBm   SNR {:.1f} dB".format(telem["rssi"], telem["snr"]))
        else:
            self.link_var.set("RSSI --   SNR -- (recepteur sans TLM/rssi)")
        self.loss_var.set("Paquets recus : {}".format(self.total_packets))

        self.ts.append(telem["t"])
        self.hs.append(telem["h"])
        self.vs.append(telem["v"])
        if len(self.ts) > MAX_POINTS:
            self.ts = self.ts[-MAX_POINTS:]
            self.hs = self.hs[-MAX_POINTS:]
            self.vs = self.vs[-MAX_POINTS:]

        self.line_h.set_data(self.ts, self.hs)
        self.line_v.set_data(self.ts, self.vs)
        for ax, ys in ((self.ax_h, self.hs), (self.ax_v, self.vs)):
            ax.set_xlim(self.ts[0], max(self.ts[-1], self.ts[0] + 1))
            ymin, ymax = min(ys), max(ys)
            pad = max(1.0, (ymax - ymin) * 0.1)
            ax.set_ylim(ymin - pad, ymax + pad)
        self.canvas.draw_idle()

    def _log(self, text):
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        # limite la console a 200 lignes pour ne pas saturer la memoire en vol long
        nb_lignes = int(self.log_text.index("end-1c").split(".")[0])
        if nb_lignes > 200:
            self.log_text.delete("1.0", "2.0")

    def on_close(self):
        self.reader.stop()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="Dashboard LUNATIK")
    parser.add_argument("--port", help="Port serie du recepteur (ex: COM5 ou /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Vitesse serie (defaut 115200)")
    parser.add_argument("--simulate", action="store_true",
                         help="Genere des trames de test, sans port serie ni Pico")
    args = parser.parse_args()

    if args.simulate:
        print("Mode simulation : aucun port serie requis")
        app = Dashboard(None, args.baud, simulate=True)
        app.protocol("WM_DELETE_WINDOW", app.on_close)
        app.mainloop()
        return

    port = args.port or find_pico_port()
    if port is None:
        print("Port serie non trouve automatiquement. Ports disponibles :")
        for p in serial.tools.list_ports.comports():
            print("  {} - {}".format(p.device, p.description))
        print("Relance avec --port <nom_du_port>")
        sys.exit(1)

    print("Connexion sur {} @ {} bauds".format(port, args.baud))
    app = Dashboard(port, args.baud)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()


if __name__ == "__main__":
    main()