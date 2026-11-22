# general.py - LUNATIK : la boite a outils des questions du jury
#
# Deux usages, choisis avec les boutons de gauche :
#
#   CARTE EN DIRECT : courbes de TOUS les capteurs de la carte, y compris ceux
#   qui ne sont pas dans les demos (gyroscope, temperature, pression, vitesse
#   du Kalman, cadence reelle, inclinaison, GPS, batterie), histogramme du
#   bruit, et commandes envoyees a la carte : rallumer / couper le GPS,
#   changer le surechantillonnage (x1 a x32), recalage du Kalman.
#   La carte doit faire tourner scripts/code.py (version avec commandes).
#
#   VOL ENREGISTRE : relit un journal de vol (data_flight_2.csv par defaut)
#   et trace : altitude, parabole de l'apogee, qui detecte quand, fausses
#   apogees au sol, accelerometre et phases, trainee (-f contre v^2),
#   poussee et impulsion, bruit au sol, trajectoire et altitude GPS,
#   temperature.
#
# Tout est en Tkinter seul (aucune autre bibliotheque de graphiques).
# Chaque axe a ses graduations et son unite.
#
# Installation : pip install pyserial
#                cartes.py et apogee.py doivent etre a cote de ce fichier.
# Lancement    : python general.py                 (carte A par defaut)
#                python general.py COM5            (port impose)
#                python general.py vol.csv         (ouvre ce journal de vol)
# Touches      : F11 = plein ecran oui/non, Echap = quitter le plein ecran
#
# Attention : un seul programme a la fois par carte. Si une demo utilise
# deja la carte A, general.py ne pourra pas l'ouvrir (et inversement).

import csv
import math
import os
import sys
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog

import serial

import cartes
from apogee import ApogeeDetection

# ---- constantes ----------------------------------------------------------------
G = 9.81
SIGMA = "\u03c3"
RACINE = "\u221a"
PM = "\u00b1"

NB_COLONNES = 17
MAX_MESURES = 12000          # ~5 min a 40 mesures par seconde
FENETRES_S = (10, 30, 60, 300)
OS_VALIDES = (1, 2, 4, 8, 16, 32)

# vol : valeurs du memoire, utilisees comme references sur les graphes
MASSE_DECOLLAGE = 0.812      # kg
MASSE_FIN = 0.776            # kg, apres combustion des 35,6 g de propergol
C_MEMOIRE = 1.4e-3           # 1/m, coefficient de trainee D/m = c v^2
IMPULSION_MEMOIRE = "60 " + PM + " 2 N.s"
POUSSEE_NAR = 25.55          # N, poussee moyenne de la fiche
IMPULSION_NAR = 77.92        # N.s, essai statique de la fiche
SEUIL_DECOLLAGE = 19.6       # m/s2, comme state_machine.py
SEUIL_DEBUT_COMBUSTION = 12  # m/s2 : a_long nettement au-dessus de g
SEUIL_EJECTION = 30.0        # m/s2 : choc de l'ejection sur la norme

NOIR = "#000000"
GRIS = "#777777"
GRIS_CLAIR = "#b0b0b0"
GRIS_GRILLE = "#e6e6e6"
TIRETS = {"ligne": None, "tirets": (8, 5), "pointilles": (2, 4)}


# ---- petites fonctions de calcul -----------------------------------------------
def nombre(x, dec):
    """Nombre a la francaise (virgule), sans '-0'."""
    s = f"{x:.{dec}f}".replace(".", ",")
    if s.startswith("-") and s.strip("-0,") == "":
        s = s[1:]
    return s


def joli_pas(etendue, n=6):
    """Pas de graduation 1, 2 ou 5 x 10^k pour environ n graduations."""
    brut = etendue / n
    p = 10 ** math.floor(math.log10(brut))
    for m in (1, 2, 5, 10):
        if brut <= m * p:
            return m * p
    return 10 * p


def decimales(pas):
    return max(0, -int(math.floor(math.log10(pas) + 1e-9)))


def moyenne(v):
    return sum(v) / len(v) if v else 0.0


def ecart_type(v):
    if len(v) < 2:
        return 0.0
    m = moyenne(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def autocorrelation_1(v):
    """Correlation entre v[n] et v[n+1] (lag 1)."""
    if len(v) < 3:
        return 0.0
    m = moyenne(v)
    var = sum((x - m) ** 2 for x in v)
    if var == 0:
        return 0.0
    cov = sum((v[k] - m) * (v[k + 1] - m) for k in range(len(v) - 1))
    return cov / var


def histogramme(v, nb_classes=31):
    """Centres des classes, effectifs, largeur d'une classe."""
    if len(v) < 2:
        return [], [], 1.0
    lo, hi = min(v), max(v)
    if hi == lo:
        hi = lo + 1.0
    largeur = (hi - lo) / nb_classes
    effectifs = [0] * nb_classes
    for x in v:
        k = min(nb_classes - 1, int((x - lo) / largeur))
        effectifs[k] += 1
    centres = [lo + (k + 0.5) * largeur for k in range(nb_classes)]
    return centres, effectifs, largeur


def gaussienne(centres, n, m, s, largeur):
    """Effectifs attendus si les valeurs suivaient une loi normale."""
    if s <= 0:
        return [0.0] * len(centres)
    a = n * largeur / (s * math.sqrt(2 * math.pi))
    return [a * math.exp(-((x - m) ** 2) / (2 * s * s)) for x in centres]


def ajuster_parabole(points):
    """Moindres carres h = c0 + c1 t + c2 t^2, avec incertitude sur c2.

    Renvoie (c0, c1, c2, u_c2, ecart-type des residus) ou None.
    """
    n = len(points)
    if n < 5:
        return None
    t_moy = moyenne([t for t, _ in points])     # centrer t : calcul plus stable
    s = [sum((t - t_moy) ** p for t, _ in points) for p in range(5)]
    y = [sum(h * (t - t_moy) ** p for t, h in points) for p in range(3)]
    a = [[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]]
    inv = inverse_3x3(a)
    if inv is None:
        return None
    c = [sum(inv[i][j] * y[j] for j in range(3)) for i in range(3)]
    residus = [h - (c[0] + c[1] * (t - t_moy) + c[2] * (t - t_moy) ** 2) for t, h in points]
    sigma = math.sqrt(sum(r * r for r in residus) / (n - 3))
    u_c2 = sigma * math.sqrt(inv[2][2])
    # retour aux coefficients en t (non centre)
    c0 = c[0] - c[1] * t_moy + c[2] * t_moy ** 2
    c1 = c[1] - 2 * c[2] * t_moy
    return c0, c1, c[2], u_c2, sigma


def inverse_3x3(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-300:
        return None
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]


def gps_en_metres(lat, lon, lat0, lon0):
    """Position (est, nord) en metres autour du point (lat0, lon0)."""
    x = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    y = (lat - lat0) * 110540.0
    return x, y


# ---- le graphique : axes gradues, unites, courbes --------------------------------
class Graphe:
    """Dessine des courbes sur un Canvas, avec axes, graduations et unites.

    Une courbe est un dictionnaire :
        x, y      : listes de nombres
        style     : "ligne", "tirets", "pointilles", "points" ou "barres"
        couleur   : NOIR, GRIS ou GRIS_CLAIR
        epaisseur : en pixels (lignes) ou rayon (points)
        legende   : texte de la legende (facultatif)
        largeur   : largeur des barres (style "barres")
    """

    def __init__(self, canvas, police, police_petite):
        self.c = canvas
        self.police = police
        self.petite = police_petite

    def tracer(self, courbes, x_titre, y_titre, x_lim=None, y_lim=None,
               verticales=(), horizontales=(), meme_echelle=False,
               message=None, y_etendue_min=None):
        c = self.c
        c.delete("all")
        largeur, hauteur = c.winfo_width(), c.winfo_height()
        if largeur < 100 or hauteur < 100:
            return
        gauche = max(80, int(largeur * 0.08))
        droite = largeur - 25
        haut = 45
        bas = hauteur - 65

        if message:
            c.create_text(largeur / 2, hauteur / 2, text=message, font=self.police,
                          fill=NOIR, width=largeur * 0.8, justify="center")
            return

        # ---- etendue des axes
        xs, ys = [], []
        for k in courbes:
            for x, y in zip(k["x"], k["y"]):
                if x is None or y is None or x != x or y != y:
                    continue
                xs.append(x)
                ys.append(y)
                if k.get("style") == "barres":
                    xs.append(x - k.get("largeur", 0) / 2)
                    xs.append(x + k.get("largeur", 0) / 2)
                    ys.append(0.0)
        for h in horizontales:
            ys.append(h[0])
        if not xs:
            c.create_text(largeur / 2, hauteur / 2, text="Pas encore de mesures",
                          font=self.police, fill="black")
            return
        x0, x1 = x_lim if x_lim else (min(xs), max(xs))
        if y_lim:
            y0, y1 = y_lim
        else:
            y0, y1 = min(ys), max(ys)
            marge = 0.06 * (y1 - y0) if y1 > y0 else 0.5
            y0, y1 = y0 - marge, y1 + marge
        if y_etendue_min and y1 - y0 < y_etendue_min:
            milieu = (y0 + y1) / 2
            y0, y1 = milieu - y_etendue_min / 2, milieu + y_etendue_min / 2
        if x1 <= x0:
            x1 = x0 + 1.0
        if y1 <= y0:
            y1 = y0 + 1.0

        if meme_echelle:
            # meme nombre de metres par pixel sur les deux axes (cartes)
            px_x = (droite - gauche) / (x1 - x0)
            px_y = (bas - haut) / (y1 - y0)
            if px_x > px_y:
                ajout = ((droite - gauche) / px_y - (x1 - x0)) / 2
                x0, x1 = x0 - ajout, x1 + ajout
            else:
                ajout = ((bas - haut) / px_x - (y1 - y0)) / 2
                y0, y1 = y0 - ajout, y1 + ajout

        def px(x):
            return gauche + (x - x0) / (x1 - x0) * (droite - gauche)

        def py(y):
            y = max(y0, min(y1, y))
            return bas - (y - y0) / (y1 - y0) * (bas - haut)

        # ---- grille et graduations
        pas_x = joli_pas(x1 - x0)
        pas_y = joli_pas(y1 - y0)
        dx, dy = decimales(pas_x), decimales(pas_y)
        v = math.ceil(x0 / pas_x - 1e-9) * pas_x
        while v <= x1 + 1e-9 * pas_x:
            c.create_line(px(v), haut, px(v), bas, fill=GRIS_GRILLE)
            c.create_line(px(v), bas, px(v), bas + 7, fill=NOIR, width=2)
            c.create_text(px(v), bas + 10, text=nombre(v, dx), anchor="n", font=self.petite, fill="black")
            v += pas_x
        v = math.ceil(y0 / pas_y - 1e-9) * pas_y
        while v <= y1 + 1e-9 * pas_y:
            c.create_line(gauche, py(v), droite, py(v), fill=GRIS_GRILLE)
            c.create_line(gauche - 7, py(v), gauche, py(v), fill=NOIR, width=2)
            c.create_text(gauche - 10, py(v), text=nombre(v, dy), anchor="e", font=self.petite, fill="black")
            v += pas_y
        c.create_line(gauche, bas, droite, bas, fill=NOIR, width=2)
        c.create_line(gauche, haut, gauche, bas, fill=NOIR, width=2)
        c.create_text((gauche + droite) / 2, hauteur - 8, text=x_titre, anchor="s",
                      font=self.police, fill="black")
        c.create_text(14, (haut + bas) / 2, text=y_titre, angle=90, anchor="n",
                      font=self.police, fill="black")

        # ---- lignes de reference
        for h in horizontales:
            y, texte = h[0], h[1]
            style = h[2] if len(h) > 2 else "tirets"
            if y0 <= y <= y1:
                c.create_line(gauche, py(y), droite, py(y), fill=GRIS, width=2,
                              dash=TIRETS.get(style))
                c.create_text(droite - 4, py(y) - 3, text=texte, anchor="se",
                              font=self.petite, fill=GRIS)
        for i, vl in enumerate(verticales):
            x, texte = vl[0], vl[1]
            style = vl[2] if len(vl) > 2 else "tirets"
            if x0 <= x <= x1:
                c.create_line(px(x), haut, px(x), bas, fill=GRIS, width=2,
                              dash=TIRETS.get(style))
                # etiquettes decalees en hauteur : deux evenements proches
                # ne s'ecrivent pas l'un sur l'autre
                c.create_text(px(x) + 3, haut + 4 + (i % 3) * (bas - haut) * 0.22,
                              text=texte, angle=90, anchor="ne", font=self.petite,
                              fill=GRIS)

        # ---- courbes
        for k in courbes:
            style = k.get("style", "ligne")
            couleur = k.get("couleur", NOIR)
            ep = k.get("epaisseur", 2)
            pts = [(x, y) for x, y in zip(k["x"], k["y"])
                   if x is not None and y is not None and x == x and y == y
                   and x0 <= x <= x1]
            if style == "barres":
                w = k.get("largeur", 1.0)
                for x, y in pts:
                    c.create_rectangle(px(x - w / 2), py(y), px(x + w / 2), py(0),
                                       fill=GRIS_CLAIR, outline=NOIR)
            elif style == "points":
                pas = max(1, len(pts) // 1500)
                for x, y in pts[::pas]:
                    c.create_oval(px(x) - ep, py(y) - ep, px(x) + ep, py(y) + ep,
                                  fill=couleur, outline="")
            else:
                pas = max(1, len(pts) // 3000)
                coords = []
                for x, y in pts[::pas]:
                    coords += [px(x), py(y)]
                if len(coords) >= 4:
                    c.create_line(*coords, fill=couleur, width=ep,
                                  dash=TIRETS.get(style))

        # ---- legende, en haut a droite, dans la marge
        xl = droite
        for k in reversed([k for k in courbes if k.get("legende")]):
            t = c.create_text(xl, 22, text=k["legende"], anchor="e", font=self.petite, fill="black")
            x_texte = c.bbox(t)[0]
            style = k.get("style", "ligne")
            couleur = k.get("couleur", NOIR)
            if style == "points":
                c.create_oval(x_texte - 18, 18, x_texte - 10, 26, fill=couleur, outline="")
            elif style == "barres":
                c.create_rectangle(x_texte - 22, 16, x_texte - 8, 28, fill=GRIS_CLAIR,
                                   outline=NOIR)
            else:
                c.create_line(x_texte - 40, 22, x_texte - 8, 22, fill=couleur,
                              width=k.get("epaisseur", 2), dash=TIRETS.get(style))
            xl = x_texte - 60


# ---- le journal de vol ----------------------------------------------------------
class Vol:
    """Un journal de vol (format de datalog.py), avec ses evenements."""

    def __init__(self, chemin):
        self.chemin = chemin
        t, phase, cols = [], [], []
        with open(chemin, encoding="utf-8", errors="ignore") as f:
            lecteur = csv.reader(f)
            next(lecteur, None)
            for ligne in lecteur:
                if len(ligne) != NB_COLONNES:
                    continue
                try:
                    valeurs = [float(v) for v in ligne[2:]]
                    t.append(float(ligne[0]))
                except ValueError:
                    continue
                phase.append(ligne[1].strip())
                cols.append(valeurs)
        if not t:
            raise ValueError("aucune ligne lisible")
        noms = ["a_lat1", "a_lat2", "a_long", "gx", "gy", "gz", "pression", "temp",
                "baro", "lat", "lon", "alt_gps", "kal_h", "kal_v", "batt"]
        self.d = {n: [c[k] for c in cols] for k, n in enumerate(noms)}
        self.phase = phase
        self.norme = [math.sqrt(a ** 2 + b ** 2 + c ** 2) for a, b, c in
                      zip(self.d["a_lat1"], self.d["a_lat2"], self.d["a_long"])]

        # t = 0 au premier echantillon BOOST (decollage detecte par la carte)
        k0 = next((k for k, p in enumerate(phase) if p == "BOOST"), None)
        if k0 is None:
            k0 = max(range(len(t)), key=lambda k: self.d["a_long"][k])
        self.k_decollage = k0
        self.t = [x - t[k0] for x in t]

        # zero barometrique : moyenne des 5 dernieres secondes au sol
        sol = [self.d["baro"][k] for k in range(len(t)) if -5.0 < self.t[k] < -0.1]
        self.zero_baro = moyenne(sol)
        self.h = [b - self.zero_baro for b in self.d["baro"]]
        self.kal_h = [k - self.zero_baro for k in self.d["kal_h"]]

        n = len(self.t)
        along = self.d["a_long"]
        self.k_debut_combustion = next(
            (k for k in range(max(0, k0 - 20), n) if along[k] > SEUIL_DEBUT_COMBUSTION), k0)
        self.k_fin_combustion = next(
            (k for k in range(k0 + 5, n) if along[k] < 0), k0 + 5)
        self.k_apogee_carte = next((k for k, p in enumerate(phase) if p == "APOGEE"), None)
        k_haut = max(range(k0, n), key=lambda k: self.h[k])
        self.k_haut = k_haut
        self.k_ejection = next(
            (k for k in range(k_haut, n) if self.norme[k] > SEUIL_EJECTION), None)
        self.k_atterrissage = next((k for k, p in enumerate(phase) if p == "LANDED"), n - 1)

    def te(self, k):
        return self.t[k] if k is not None else None

    def indices(self, t_min, t_max):
        return [k for k in range(len(self.t)) if t_min <= self.t[k] <= t_max]

    def parabole(self, demi_fenetre):
        """Ajustement autour du sommet ; (t_sommet, h_sommet, 2a, u_2a, residus, n)."""
        tc = self.t[self.k_haut]
        # on recentre une fois sur le sommet de la premiere parabole
        for _ in range(2):
            pts = [(self.t[k], self.h[k]) for k in self.indices(tc - demi_fenetre,
                                                               tc + demi_fenetre)]
            res = ajuster_parabole(pts)
            if res is None or res[2] >= 0:
                return None
            c0, c1, c2, u_c2, sigma = res
            tc = -c1 / (2 * c2)
        hm = c0 + c1 * tc + c2 * tc * tc
        return tc, hm, -2 * c2, 2 * u_c2, sigma, len(pts), (c0, c1, c2)


# ---- l'application ----------------------------------------------------------------
class Application:
    def __init__(self, fenetre):
        self.f = fenetre
        self.f.title("LUNATIK - général")
        self.f.configure(bg="white")
        self.f.geometry("1400x850")
        self.f.bind("<F11>", lambda e: self.f.attributes(
            "-fullscreen", not self.f.attributes("-fullscreen")))
        self.f.bind("<Escape>", lambda e: self.f.attributes("-fullscreen", False))

        h = self.f.winfo_screenheight()
        self.police = ("Helvetica", max(11, h // 70))
        self.petite = ("Helvetica", max(10, h // 85))
        self.grande = ("Helvetica", max(14, h // 45), "bold")
        self.gras = ("Helvetica", max(11, h // 70), "bold")

        # ---- etat de la carte en direct
        self.carte = "A"
        self.port = None
        self.tampon = ""
        self.t_dernier_essai = 0.0
        self.mesures = deque(maxlen=MAX_MESURES)
        self.fenetre_s = 30
        self.fige = False
        self.reponse = ""
        self.os_carte = None
        self.gps_carte = None
        self.bruit_alt = []          # altitudes depuis la remise a zero (cm)
        self.bruit_t = []
        self.table_os = {}           # os -> (sigma cm, cadence, n)

        # ---- etat du vol
        self.vol = None
        self.options_vue = {}

        self.vue = None
        self.boutons = {}
        self.construire()

        # journal de vol : argument .csv, sinon ../data_flight_2.csv
        csv_arg = next((a for a in sys.argv[1:] if a.lower().endswith(".csv")), None)
        if csv_arg:
            sys.argv.remove(csv_arg)
        ici = os.path.dirname(os.path.abspath(__file__))
        defaut = os.path.join(ici, "..", "data_flight_2.csv")
        self.charger_vol(csv_arg or (defaut if os.path.exists(defaut) else None))

        self.choisir_vue("d_baro")
        self.f.after(10, self.lire_port)
        self.f.after(150, self.rafraichir)
        self.zone.bind("<Configure>", lambda e: self.redessiner())

    # ---- construction de l'interface
    def bouton(self, parent, texte, commande, cle=None, largeur=None):
        b = tk.Label(parent, text=texte, font=self.petite, bg="white", fg=NOIR,
                     relief="solid", bd=1, padx=6, pady=3, cursor="hand2")
        if largeur:
            b.config(width=largeur)
        b.bind("<Button-1>", lambda e: commande())
        if cle:
            self.boutons[cle] = b
        return b

    def grille_vues(self, parent, vues):
        """Boutons des vues sur deux colonnes, pour tenir sur un ecran de portable."""
        cadre = tk.Frame(parent, bg="white")
        cadre.pack(fill="x")
        cadre.columnconfigure(0, weight=1, uniform="col")
        cadre.columnconfigure(1, weight=1, uniform="col")
        for i, (cle, texte) in enumerate(vues):
            self.bouton(cadre, texte, lambda c=cle: self.choisir_vue(c), cle).grid(
                row=i // 2, column=i % 2, sticky="ew", padx=1, pady=1)

    def titre_section(self, parent, texte):
        tk.Label(parent, text=texte, font=self.gras, bg="white", fg=NOIR,
                 anchor="w").pack(fill="x", pady=(12, 3))

    def construire(self):
        gauche = tk.Frame(self.f, bg="white")
        gauche.pack(side="left", fill="y", padx=(10, 5), pady=5)
        droite = tk.Frame(self.f, bg="white")
        droite.pack(side="left", fill="both", expand=True, padx=(5, 10), pady=5)

        # -- carte en direct
        self.titre_section(gauche, "CARTE EN DIRECT")
        ligne = tk.Frame(gauche, bg="white")
        ligne.pack(fill="x")
        self.bouton(ligne, "Carte A", lambda: self.changer_carte("A"), "carte_A").pack(
            side="left", padx=(0, 4))
        self.bouton(ligne, "Carte B", lambda: self.changer_carte("B"), "carte_B").pack(
            side="left")
        self.etat_carte = tk.Label(gauche, text="", font=self.petite, bg="white", fg=GRIS,
                                   anchor="w", justify="left", wraplength=380)
        self.etat_carte.pack(fill="x", pady=2)

        vues_directes = [
            ("d_baro", "Altitude du baromètre seul"),
            ("d_altitude", "Altitude (baro et Kalman)"),
            ("d_vitesse", "Vitesse du Kalman"),
            ("d_accel", "Accélérations (3 axes)"),
            ("d_angle", "Inclinaison de la carte"),
            ("d_gyro", "Gyroscope"),
            ("d_pression", "Pression"),
            ("d_temp", "Température"),
            ("d_batt", "Batterie"),
            ("d_cadence", "Cadence réelle"),
            ("d_bruit", "Bruit du baromètre"),
            ("d_gps", "Position GPS"),
        ]
        self.grille_vues(gauche, vues_directes)

        ligne = tk.Frame(gauche, bg="white")
        ligne.pack(fill="x", pady=(6, 0))
        tk.Label(ligne, text="durée :", font=self.petite, bg="white", fg="black").pack(side="left")
        for s in FENETRES_S:
            texte = f"{s} s" if s < 60 else f"{s // 60} min"
            self.bouton(ligne, texte, lambda s=s: self.choisir_fenetre(s),
                        f"fen_{s}").pack(side="left", padx=1)

        tk.Label(gauche, text="Commandes à la carte :", font=self.petite, bg="white",
                 anchor="w", fg="black").pack(fill="x", pady=(8, 1))
        ligne = tk.Frame(gauche, bg="white")
        ligne.pack(fill="x")
        self.bouton(ligne, "GPS on", lambda: self.commande("gps on")).pack(side="left", padx=1)
        self.bouton(ligne, "GPS off", lambda: self.commande("gps off")).pack(side="left", padx=1)
        self.bouton(ligne, "Recalage", lambda: self.commande("r")).pack(side="left", padx=1)
        self.bouton(ligne, "État", lambda: self.commande("?")).pack(side="left", padx=1)
        ligne = tk.Frame(gauche, bg="white")
        ligne.pack(fill="x", pady=2)
        for n in OS_VALIDES:
            self.bouton(ligne, f"x{n}", lambda n=n: self.commande(f"os {n}"),
                        f"os_{n}").pack(side="left", padx=1)
        ligne = tk.Frame(gauche, bg="white")
        ligne.pack(fill="x", pady=2)
        self.bouton(ligne, "Figer / reprendre", self.basculer_fige, "fige").pack(
            side="left", padx=1)
        self.bouton(ligne, "Remise à zéro du bruit", self.raz_bruit).pack(side="left", padx=1)

        # -- vol enregistre
        self.titre_section(gauche, "VOL ENREGISTRÉ")
        self.bouton(gauche, "Ouvrir un journal (.csv)...", self.ouvrir_vol).pack(fill="x")
        self.etat_vol = tk.Label(gauche, text="", font=self.petite, bg="white", fg=GRIS,
                                 anchor="w", justify="left", wraplength=380)
        self.etat_vol.pack(fill="x", pady=2)
        vues_vol = [
            ("v_altitude", "Altitude du vol"),
            ("v_parabole", "Apogée : parabole"),
            ("v_detections", "Qui détecte quand ?"),
            ("v_fausses", "Fausses apogées au sol"),
            ("v_accel", "Accéléromètre et phases"),
            ("v_trainee", "Traînée : -f contre v²"),
            ("v_poussee", "Poussée et impulsion"),
            ("v_bruit", "Bruit au sol"),
            ("v_gps", "Trajectoire GPS"),
            ("v_gps_alt", "Altitude GPS et baro"),
            ("v_temp", "Température et batterie"),
        ]
        self.grille_vues(gauche, vues_vol)

        self.bouton(gauche, "Enregistrer l'image (.eps)", self.exporter).pack(
            fill="x", pady=(14, 0))

        # -- partie droite : titre, options, graphique, informations
        self.titre = tk.Label(droite, text="", font=self.grande, bg="white", fg=NOIR,
                              anchor="w")
        self.titre.pack(fill="x")
        self.options = tk.Frame(droite, bg="white")
        self.options.pack(fill="x", pady=(2, 0))
        self.zone = tk.Canvas(droite, bg="white", highlightthickness=0)
        self.zone.pack(fill="both", expand=True)
        self.graphe = Graphe(self.zone, self.police, self.petite)
        self.infos = tk.Label(droite, text="", font=self.police, bg="white", fg=NOIR,
                              anchor="w", justify="left")
        self.infos.pack(fill="x", pady=(4, 0))
        self.texte_reponse = tk.Label(droite, text="", font=self.petite, bg="white",
                                      fg=GRIS, anchor="w")
        self.texte_reponse.pack(fill="x")
        droite.bind("<Configure>", lambda e: (
            self.infos.config(wraplength=max(300, e.width - 10)),
            self.texte_reponse.config(wraplength=max(300, e.width - 10))))
        self.marquer()

    def marquer(self):
        """Bouton choisi en noir : vue, duree, carte, reglage x, figer."""
        actifs = {self.vue, f"fen_{self.fenetre_s}", f"carte_{self.carte}"}
        if self.os_carte:
            actifs.add(f"os_{self.os_carte}")
        if self.fige:
            actifs.add("fige")
        for cle, b in self.boutons.items():
            if cle in actifs:
                b.config(bg=NOIR, fg="white")
            else:
                b.config(bg="white", fg=NOIR)

    def vider_options(self):
        for w in self.options.winfo_children():
            w.destroy()

    def option(self, nom, valeurs, defaut, libelle):
        """Rangee de boutons au-dessus du graphique pour une option de la vue."""
        if nom not in self.options_vue:
            self.options_vue[nom] = defaut
        tk.Label(self.options, text=libelle, font=self.petite, bg="white", fg="black").pack(
            side="left", padx=(0, 4))
        for valeur, texte in valeurs:
            b = self.bouton(self.options, texte,
                            lambda v=valeur: self.changer_option(nom, v))
            if valeur == self.options_vue[nom]:
                b.config(bg=NOIR, fg="white")
            b.pack(side="left", padx=1)
        tk.Label(self.options, text="   ", bg="white", fg="black").pack(side="left")

    def changer_option(self, nom, valeur):
        self.options_vue[nom] = valeur
        self.choisir_vue(self.vue)

    # ---- choix de la vue
    def choisir_vue(self, cle):
        self.vue = cle
        self.vider_options()
        if cle == "v_parabole":
            self.option("fenetre_parabole",
                        [(0.5, PM + "0,5 s"), (1.0, PM + "1 s"), (1.5, PM + "1,5 s"),
                         (2.0, PM + "2 s"), (2.5, PM + "2,5 s")], 2.0,
                        "fenêtre d'ajustement :")
        elif cle == "v_accel":
            self.option("zoom_accel", [("combustion", "combustion"), ("vol", "tout le vol"),
                                       ("rampe", "sur la rampe")], "combustion", "zoom :")
        elif cle == "v_trainee":
            self.option("v_min", [(5.0, "5 m/s"), (10.0, "10 m/s"), (15.0, "15 m/s"),
                                  (20.0, "20 m/s")], 15.0, "ajustement si v >")
        elif cle == "v_poussee":
            self.option("c_poussee", [(C_MEMOIRE, "c du mémoire"), ("ajuste", "c ajusté"),
                                      (0.0, "sans traînée")], C_MEMOIRE, "traînée :")
        elif cle == "v_bruit":
            self.option("duree_sol", [(10.0, "10 s"), (60.0, "60 s"), (300.0, "5 min"),
                                      (1e9, "tout")], 60.0, "avant le décollage :")
        elif cle == "d_bruit":
            self.option("grandeur_bruit", [("diff", "différences successives"),
                                           ("alt", "altitudes")], "diff", "histogramme des :")
        self.marquer()
        self.redessiner()

    def choisir_fenetre(self, s):
        self.fenetre_s = s
        self.marquer()
        self.redessiner()

    def redessiner(self):
        if self.vue is None:
            return
        fonction = getattr(self, "vue_" + self.vue, None)
        if fonction is not None:
            try:
                fonction()
            except Exception as e:      # un graphe rate ne doit pas tout bloquer
                self.titre.config(text="Erreur de tracé")
                self.graphe.tracer([], "", "", message=f"Erreur : {e}")

    def rafraichir(self):
        if self.vue and self.vue.startswith("d_") and not self.fige:
            self.redessiner()
        self.f.after(150, self.rafraichir)

    # ---- carte : connexion, lecture, commandes
    def changer_carte(self, carte):
        if carte == self.carte:
            return
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.carte = carte
        self.mesures.clear()
        self.raz_bruit()
        self.table_os = {}
        self.os_carte = self.gps_carte = None
        self.t_dernier_essai = 0.0
        self.marquer()

    def connecter(self):
        if time.monotonic() - self.t_dernier_essai < 0.5:
            return
        self.t_dernier_essai = time.monotonic()
        nom = cartes.trouver_port(self.carte)
        if nom is not None:
            self.port = cartes.ouvrir(nom)
            self.tampon = ""
            if self.port is not None:
                cartes.envoyer(self.port, "?")      # demande le reglage actuel
        self.etat_carte.config(text=cartes.message)

    def lire_port(self):
        if self.port is None:
            self.connecter()
        else:
            try:
                n = self.port.in_waiting
                if n > 0:
                    self.tampon += self.port.read(n).decode("utf-8", errors="ignore")
                    lignes = self.tampon.split("\n")
                    self.tampon = lignes.pop()
                    for ligne in lignes:
                        self.traiter_ligne(ligne.strip())
            except (serial.SerialException, OSError):
                self.port = None
                cartes.message = "Carte débranchée, en attente..."
                self.etat_carte.config(text=cartes.message)
        self.f.after(10, self.lire_port)

    def traiter_ligne(self, ligne):
        if ligne.startswith("#"):
            self.reponse_carte(ligne)
            return
        champs = ligne.split(",")
        if len(champs) != NB_COLONNES:
            return
        try:
            v = [float(champs[0])] + [float(x) for x in champs[2:]]
        except ValueError:
            return
        if self.mesures and v[0] < self.mesures[-1][0]:
            self.mesures.clear()                 # la carte a redemarre
            self.raz_bruit()
        self.mesures.append(v)
        # bruit : altitude barometrique (index 9 dans v)
        self.bruit_alt.append(100.0 * v[9])
        self.bruit_t.append(v[0])
        if len(self.bruit_alt) > 30000:
            del self.bruit_alt[:5000]
            del self.bruit_t[:5000]

    def reponse_carte(self, ligne):
        self.reponse = ligne
        self.texte_reponse.config(text="Réponse de la carte : " + ligne)
        mots = ligne.replace("=", " ").split()
        if ligne.startswith("#OS") and len(mots) > 1 and mots[1].isdigit():
            self.memoriser_os()             # garde le bruit de l'ancien reglage
            self.os_carte = int(mots[1])
            self.raz_bruit()
        elif ligne.startswith("#GPS") and len(mots) > 1:
            self.gps_carte = mots[1] == "on"
        elif ligne.startswith("#ETAT"):
            for k in range(len(mots) - 1):
                if mots[k] == "os":
                    self.os_carte = int(mots[k + 1])
                elif mots[k] == "gps":
                    self.gps_carte = mots[k + 1] == "1"
        self.marquer()

    def commande(self, texte):
        if self.port is None:
            self.texte_reponse.config(text="Pas de carte connectée : commande non envoyée")
            return
        if not cartes.envoyer(self.port, texte):
            self.texte_reponse.config(text="Envoi impossible : vérifie le câble")
        else:
            self.texte_reponse.config(text=f"Commande envoyée : {texte}   (en attente "
                                           "de la réponse... une carte avec l'ancien "
                                           "code.py ne répond pas)")

    def basculer_fige(self):
        self.fige = not self.fige
        self.marquer()

    def raz_bruit(self):
        self.bruit_alt = []
        self.bruit_t = []

    def memoriser_os(self):
        """Garde le bruit mesure avec le reglage actuel avant d'en changer."""
        if self.os_carte and len(self.bruit_alt) > 50:
            d = [self.bruit_alt[k + 1] - self.bruit_alt[k]
                 for k in range(len(self.bruit_alt) - 1)]
            duree = self.bruit_t[-1] - self.bruit_t[0]
            cadence = (len(self.bruit_t) - 1) / duree if duree > 0 else 0
            self.table_os[self.os_carte] = (ecart_type(d) / math.sqrt(2), cadence,
                                            len(self.bruit_alt))

    # ---- donnees en direct sur la duree choisie
    def recentes(self):
        if not self.mesures:
            return []
        t_fin = self.mesures[-1][0]
        return [m for m in self.mesures if m[0] >= t_fin - self.fenetre_s]

    def tracer_direct(self, titre, series, y_titre, infos="", y_etendue_min=None,
                      horizontales=()):
        """series : liste de (index dans la mesure ou fonction, style, couleur, legende)."""
        self.titre.config(text=titre)
        m = self.recentes()
        if self.port is None and not m:
            self.graphe.tracer([], "", "", message=cartes.message)
            self.infos.config(text="")
            return
        t_fin = m[-1][0] if m else 0.0
        x = [v[0] - t_fin for v in m]
        courbes = []
        for source, style, couleur, legende in series:
            y = [source(v) if callable(source) else v[source] for v in m]
            courbes.append({"x": x, "y": y, "style": style, "couleur": couleur,
                            "epaisseur": 2 if style == "points" else 3,
                            "legende": legende})
        self.graphe.tracer(courbes, "temps (s), 0 = maintenant", y_titre,
                           x_lim=(-self.fenetre_s, 0), y_etendue_min=y_etendue_min,
                           horizontales=horizontales)
        self.infos.config(text=infos)

    # index dans une mesure v : 0 t, 1 a_lat1, 2 a_lat2, 3 a_long, 4 gx, 5 gy, 6 gz,
    # 7 pression, 8 temp, 9 alt baro, 10 lat, 11 lon, 12 alt gps, 13 kal h,
    # 14 kal v, 15 batterie
    def vue_d_baro(self):
        m = self.recentes()
        infos = ""
        if len(m) > 2:
            alt = [v[9] for v in m]
            infos = (f"altitude du baromètre : {nombre(alt[-1], 2)} m      moyenne sur la durée : "
                     f"{nombre(moyenne(alt), 2)} m      {SIGMA} sur la durée : "
                     f"{nombre(100 * ecart_type(alt), 1)} cm      {len(alt)} mesures")
        self.tracer_direct("Altitude du baromètre seul (brute, sans filtre)",
                           [(9, "ligne", GRIS_CLAIR, ""), (9, "points", NOIR, "baromètre brut")],
                           "altitude (m)", infos, y_etendue_min=1.0)

    def vue_d_altitude(self):
        m = self.recentes()
        infos = ""
        if len(m) > 2:
            alt = [v[9] for v in m]
            infos = (f"baromètre : {nombre(alt[-1], 2)} m      Kalman : "
                     f"{nombre(m[-1][13], 2)} m      {SIGMA} du baromètre sur la durée : "
                     f"{nombre(100 * ecart_type(alt), 1)} cm")
        self.tracer_direct("Altitude en direct", [(9, "points", GRIS_CLAIR, "baromètre brut"),
                                                  (13, "ligne", NOIR, "Kalman de la carte")],
                           "altitude (m)", infos, y_etendue_min=1.0)

    def vue_d_vitesse(self):
        m = self.recentes()
        infos = f"vitesse du Kalman : {nombre(m[-1][14], 3)} m/s" if m else ""
        self.tracer_direct("Vitesse verticale estimée par le Kalman", [(14, "ligne", NOIR, "")],
                           "vitesse (m/s)", infos, y_etendue_min=0.5,
                           horizontales=[(0.0, "0", "pointilles")])

    def vue_d_accel(self):
        m = self.recentes()
        infos = ""
        if m:
            v = m[-1]
            n = math.sqrt(v[1] ** 2 + v[2] ** 2 + v[3] ** 2)
            infos = (f"a_long = {nombre(v[3], 2)}    a_lat1 = {nombre(v[1], 2)}    "
                     f"a_lat2 = {nombre(v[2], 2)}    norme = {nombre(n, 2)} m/s²    "
                     f"(au repos, la norme doit valoir g = 9,81)")
        self.tracer_direct(
            "Accélérations mesurées (force spécifique f = a - g)",
            [(3, "ligne", NOIR, "a_long"), (1, "tirets", GRIS, "a_lat1"),
             (2, "pointilles", GRIS, "a_lat2"),
             (lambda v: math.sqrt(v[1] ** 2 + v[2] ** 2 + v[3] ** 2), "ligne", GRIS_CLAIR,
              "norme")],
            "accélération (m/s²)", infos, y_etendue_min=2.0,
            horizontales=[(G, "g = 9,81", "pointilles")])

    def vue_d_angle(self):
        def angle(v):
            return math.degrees(math.atan2(math.sqrt(v[1] ** 2 + v[2] ** 2), v[3]))
        m = self.recentes()
        infos = ""
        if m:
            derniers = [angle(v) for v in m[-20:]]
            infos = (f"angle entre l'axe long et la verticale : {nombre(moyenne(derniers), 1)}°"
                     f"   (moyenne des 20 dernières mesures, formule arctan(latéral / axial))")
        self.tracer_direct("Inclinaison de la carte", [(angle, "ligne", NOIR, "")],
                           "angle avec la verticale (°)", infos, y_etendue_min=2.0)

    def vue_d_gyro(self):
        deg = 180.0 / math.pi
        m = self.recentes()
        infos = ""
        if m:
            v = m[-1]
            infos = (f"gx = {nombre(v[4] * deg, 1)}    gy = {nombre(v[5] * deg, 1)}    "
                     f"gz = {nombre(v[6] * deg, 1)} °/s    (le gyroscope mesure une vitesse "
                     "de rotation, il faudrait l'intégrer pour avoir un angle)")
        self.tracer_direct("Gyroscope", [(lambda v: v[4] * deg, "ligne", NOIR, "gx"),
                                         (lambda v: v[5] * deg, "tirets", GRIS, "gy"),
                                         (lambda v: v[6] * deg, "pointilles", GRIS, "gz")],
                           "vitesse de rotation (°/s)", infos, y_etendue_min=2.0)

    def vue_d_pression(self):
        m = self.recentes()
        infos = ""
        if m:
            p = [v[7] for v in m]
            infos = (f"pression : {nombre(p[-1], 3)} hPa      variation sur la durée : "
                     f"{nombre(100 * (max(p) - min(p)), 1)} Pa      "
                     "1 hPa correspond à environ 8,3 à 8,8 m")
        self.tracer_direct("Pression", [(7, "ligne", NOIR, "")], "pression (hPa)", infos,
                           y_etendue_min=0.05)

    def vue_d_temp(self):
        m = self.recentes()
        infos = f"température du capteur : {nombre(m[-1][8], 2)} °C" if m else ""
        self.tracer_direct("Température (capteur du baromètre)", [(8, "ligne", NOIR, "")],
                           "température (°C)", infos + "      La chaleur des doigts la fait "
                           "monter, et le baromètre dérive avec elle.", y_etendue_min=1.0)

    def vue_d_batt(self):
        m = self.recentes()
        infos = f"tension : {nombre(m[-1][15], 2)} V" if m else ""
        self.tracer_direct("Tension de la batterie", [(15, "ligne", NOIR, "")], "tension (V)",
                           infos, y_etendue_min=0.2)

    def vue_d_cadence(self):
        self.titre.config(text="Cadence réelle des mesures")
        m = self.recentes()
        if len(m) < 3:
            self.graphe.tracer([], "", "", message=cartes.message if self.port is None
                               else "Pas encore assez de mesures")
            return
        t_fin = m[-1][0]
        x, y, dts = [], [], []
        for k in range(1, len(m)):
            dt = m[k][0] - m[k - 1][0]
            if 0 < dt < 1.0:
                dts.append(dt)
                x.append(m[k][0] - t_fin)
                y.append(1000 * dt)
        self.graphe.tracer([{"x": x, "y": y, "style": "points", "couleur": NOIR,
                             "epaisseur": 2, "legende": "durée d'un tour"}],
                           "temps (s), 0 = maintenant", "durée d'un tour de boucle (ms)",
                           x_lim=(-self.fenetre_s, 0), y_etendue_min=5.0)
        if dts:
            self.infos.config(text=(
                f"cadence moyenne : {nombre(1 / moyenne(dts), 1)} mesures par seconde      "
                f"tour moyen {nombre(1000 * moyenne(dts), 1)} ms, plus court "
                f"{nombre(1000 * min(dts), 1)} ms, plus long {nombre(1000 * max(dts), 1)} ms\n"
                f"Réglage du baromètre : x{self.os_carte or '?'}      GPS : "
                f"{'?' if self.gps_carte is None else ('allumé' if self.gps_carte else 'coupé')}"
                "      C'est le tour le plus long qui compte : l'erreur d'intégration croît "
                "comme dt²."))

    def vue_d_bruit(self):
        self.titre.config(text="Bruit du baromètre en direct (carte immobile)")
        alt = self.bruit_alt
        if len(alt) < 20:
            self.graphe.tracer([], "", "", message="Posez la carte et ne la touchez plus : "
                               "il faut quelques secondes de mesures.\n" + cartes.message
                               if self.port is None else "Posez la carte et ne la touchez "
                               "plus : il faut quelques secondes de mesures.")
            self.infos.config(text="")
            return
        d = [alt[k + 1] - alt[k] for k in range(len(alt) - 1)]
        s_naif = ecart_type(alt)
        s_d = ecart_type(d)
        r1 = autocorrelation_1(d)
        duree = self.bruit_t[-1] - self.bruit_t[0]
        cadence = (len(alt) - 1) / duree if duree > 0 else 0
        if self.options_vue.get("grandeur_bruit", "diff") == "diff":
            valeurs, titre_x = d, "différence entre deux mesures successives (cm)"
        else:
            m = moyenne(alt)
            valeurs, titre_x = [a - m for a in alt], "écart à la moyenne (cm)"
        centres, effectifs, largeur = histogramme(valeurs)
        g = gaussienne(centres, len(valeurs), moyenne(valeurs), ecart_type(valeurs), largeur)
        self.graphe.tracer([{"x": centres, "y": effectifs, "style": "barres",
                             "largeur": largeur, "legende": "mesures"},
                            {"x": centres, "y": g, "style": "ligne", "couleur": NOIR,
                             "epaisseur": 3, "legende": "loi normale de même " + SIGMA}],
                           titre_x, "nombre de mesures")
        if self.os_carte:
            self.table_os[self.os_carte] = (s_d / math.sqrt(2), cadence, len(alt))
        tableau = "   ".join(f"x{o} : {nombre(v[0], 1)} cm à {nombre(v[1], 0)}/s"
                             for o, v in sorted(self.table_os.items()))
        self.infos.config(text=(
            f"{len(alt)} mesures ({nombre(duree, 0)} s, {nombre(cadence, 1)} par seconde)"
            f"      {SIGMA} naïf (altitudes) : {nombre(s_naif, 1)} cm      "
            f"{SIGMA}d / {RACINE}2 : {nombre(s_d / math.sqrt(2), 1)} cm      "
            f"autocorrélation des différences r1 = {nombre(r1, 3)} (théorie : -0,5)\n"
            f"Bruit mesuré pour chaque réglage pendant cette séance : "
            f"{tableau or 'change le réglage avec les boutons x1 ... x32'}"))

    def vue_d_gps(self):
        self.titre.config(text="Position GPS en direct")
        m = [v for v in self.recentes() if v[10] != 0.0 and v[11] != 0.0]
        if not m:
            etat = ("coupé : bouton GPS on" if self.gps_carte is False else
                    "pas de position (en intérieur, le GPS ne capte presque jamais)")
            self.graphe.tracer([], "", "", message="Pas de position GPS : " + etat)
            self.infos.config(text="")
            return
        lat0, lon0 = m[0][10], m[0][11]
        xy = [gps_en_metres(v[10], v[11], lat0, lon0) for v in m]
        self.graphe.tracer([{"x": [p[0] for p in xy], "y": [p[1] for p in xy],
                             "style": "points", "couleur": NOIR, "epaisseur": 3}],
                           "vers l'est (m)", "vers le nord (m)", meme_echelle=True,
                           y_etendue_min=10.0)
        v = m[-1]
        self.infos.config(text=f"latitude {nombre(v[10], 6)}°   longitude {nombre(v[11], 6)}°"
                               f"   altitude GPS {nombre(v[12], 1)} m      (l'altitude GPS "
                               "est incertaine de plusieurs mètres)")

    # ---- vol enregistre
    def ouvrir_vol(self):
        chemin = filedialog.askopenfilename(title="Journal de vol",
                                            filetypes=[("CSV", "*.csv"), ("Tous", "*")])
        if chemin:
            self.charger_vol(chemin)
            if self.vue and self.vue.startswith("v_"):
                self.redessiner()

    def charger_vol(self, chemin):
        if not chemin:
            self.etat_vol.config(text="Aucun journal ouvert")
            return
        try:
            self.vol = Vol(chemin)
            v = self.vol
            self.etat_vol.config(text=f"{os.path.basename(chemin)} : {len(v.t)} lignes, "
                                      f"décollage à t = 0")
        except Exception as e:
            self.vol = None
            self.etat_vol.config(text=f"Lecture impossible : {e}")

    def pas_de_vol(self):
        if self.vol is None:
            self.graphe.tracer([], "", "", message="Ouvre d'abord un journal de vol "
                               "(bouton 'Ouvrir un journal')")
            self.infos.config(text="")
            return True
        return False

    def evenements(self):
        """Lignes verticales des evenements du vol."""
        v = self.vol
        ev = [(v.te(v.k_debut_combustion), "allumage"),
              (v.te(v.k_fin_combustion), "fin de poussée"),
              (v.te(v.k_apogee_carte), "APOGEE (carte)"),
              (v.te(v.k_ejection), "éjection"),
              (v.te(v.k_atterrissage), "atterrissage")]
        return [(t, texte, "tirets") for t, texte in ev if t is not None]

    def vue_v_altitude(self):
        self.titre.config(text="Altitude pendant le vol")
        if self.pas_de_vol():
            return
        v = self.vol
        t_fin = v.te(v.k_atterrissage) + 3
        ks = v.indices(-3, t_fin)
        self.graphe.tracer(
            [{"x": [v.t[k] for k in ks], "y": [v.h[k] for k in ks], "style": "points",
              "couleur": GRIS_CLAIR, "epaisseur": 2, "legende": "baromètre brut"},
             {"x": [v.t[k] for k in ks], "y": [v.kal_h[k] for k in ks], "style": "ligne",
              "couleur": NOIR, "epaisseur": 3, "legende": "Kalman (en vol)"}],
            "temps depuis le décollage (s)", "altitude au-dessus de la rampe (m)",
            x_lim=(-3, t_fin), verticales=self.evenements())
        self.infos.config(text=(
            f"point le plus haut du baromètre : {nombre(v.h[v.k_haut], 2)} m à "
            f"t = {nombre(v.t[v.k_haut], 2)} s (biaisé vers le haut par le bruit : voir la "
            f"parabole)      zéro = moyenne des 5 s avant le décollage "
            f"({nombre(v.zero_baro, 2)} m dans le journal)"))

    def vue_v_parabole(self):
        self.titre.config(text="Apogée : parabole ajustée sur le baromètre")
        if self.pas_de_vol():
            return
        v = self.vol
        w = self.options_vue.get("fenetre_parabole", 2.0)
        res = v.parabole(w)
        if res is None:
            self.graphe.tracer([], "", "", message="Ajustement impossible sur cette fenêtre")
            return
        ts, hs, deux_a, u_deux_a, sigma, n, (c0, c1, c2) = res
        ks = v.indices(ts - 3, ts + 3)
        tf = [ts - w + 2 * w * i / 100 for i in range(101)]
        self.graphe.tracer(
            [{"x": [v.t[k] for k in ks], "y": [v.h[k] for k in ks], "style": "points",
              "couleur": GRIS, "epaisseur": 3, "legende": "baromètre brut"},
             {"x": [v.t[k] for k in ks], "y": [v.kal_h[k] for k in ks], "style": "ligne",
              "couleur": GRIS_CLAIR, "epaisseur": 2, "legende": "Kalman"},
             {"x": tf, "y": [c0 + c1 * t + c2 * t * t for t in tf], "style": "tirets",
              "couleur": NOIR, "epaisseur": 3, "legende": "parabole ajustée"}],
            "temps depuis le décollage (s)", "altitude au-dessus de la rampe (m)",
            x_lim=(ts - 3, ts + 3),
            verticales=[(ts, "sommet de la parabole", "pointilles"),
                        (ts - w, "début ajustement", "pointilles"),
                        (ts + w, "fin ajustement", "pointilles")])
        self.infos.config(text=(
            f"sommet : {nombre(hs, 2)} m à t = {nombre(ts, 3)} s      courbure 2a = "
            f"{nombre(deux_a, 2)} {PM} {nombre(u_deux_a, 2)} m/s² (g = 9,81)      "
            f"écart-type des résidus : {nombre(100 * sigma, 1)} cm sur {n} points\n"
            f"Le {PM} est l'incertitude statistique seule. Change la fenêtre : le sommet bouge "
            "très peu, mais 2a change beaucoup plus que ce " + PM + " : loin du sommet, la "
            "vitesse verticale n'est plus nulle, la traînée verticale non plus, et la "
            "trajectoire n'est plus une parabole. Ne pas conclure sur 2a avec une seule "
            "fenêtre."))

    def vue_v_detections(self):
        self.titre.config(text="Qui détecte quand ? (rejeu des méthodes sur le journal)")
        if self.pas_de_vol():
            return
        v = self.vol
        res = v.parabole(self.options_vue.get("fenetre_parabole", 2.0))
        t_sommet = res[0] if res else v.t[v.k_haut]
        k0 = v.k_decollage
        n = len(v.t)
        detections = []

        def premier(fonction):
            det = ApogeeDetection()
            for k in range(k0, n):
                if fonction(det, k):
                    return v.t[k]
            return None

        detections.append(("seuil 0,51 m, Kalman", premier(
            lambda det, k: det.detection(v.kal_h[k]))))
        detections.append(("seuil 0,51 m, baro brut", premier(
            lambda det, k: det.detection(v.h[k]))))
        for nn in (3, 5, 10):
            serie, t_det = 0, None
            for k in range(k0 + 1, n):
                serie = serie + 1 if v.h[k] < v.h[k - 1] else 0
                if serie >= nn:
                    t_det = v.t[k]
                    break
            detections.append((f"{nn} descentes de suite", t_det))
        t_v0 = next((v.t[k] for k in range(k0 + 10, n) if v.d["kal_v"][k] <= 0), None)
        detections.append(("vitesse du Kalman = 0", t_v0))
        detections.append(("état APOGEE de la carte", v.te(v.k_apogee_carte)))
        detections.append(("éjection pyrotechnique", v.te(v.k_ejection)))

        t_max = max([t for _, t in detections if t is not None] + [t_sommet + 2]) + 0.5
        ks = v.indices(t_sommet - 2.5, t_max)
        detections.sort(key=lambda d: 1e9 if d[1] is None else d[1])
        verts = [(t_sommet, "S", "ligne")]
        verts += [(t, str(i + 1), "tirets") for i, (nom, t) in enumerate(detections)
                  if t is not None]
        self.graphe.tracer(
            [{"x": [v.t[k] for k in ks], "y": [v.h[k] for k in ks], "style": "points",
              "couleur": GRIS_CLAIR, "epaisseur": 2, "legende": "baromètre brut"},
             {"x": [v.t[k] for k in ks], "y": [v.kal_h[k] for k in ks], "style": "ligne",
              "couleur": NOIR, "epaisseur": 3, "legende": "Kalman"}],
            "temps depuis le décollage (s)", "altitude au-dessus de la rampe (m)",
            x_lim=(t_sommet - 2.5, t_max), verticales=verts)
        lignes = []
        for i, (nom, t) in enumerate(detections):
            if t is None:
                lignes.append(f"{i + 1} = {nom} : jamais")
            else:
                ecart = t - t_sommet
                lignes.append(f"{i + 1} = {nom} : {'+' if ecart >= 0 else ''}"
                              f"{nombre(ecart, 2)} s")
        self.infos.config(text="Écart avec le sommet S de la parabole :   " +
                               "   |   ".join(lignes) +
                               "\nRejeu fait par general.py sur le journal (seuils "
                               "identiques au vol) : de petites différences avec le mémoire "
                               "sont possibles.")

    def vue_v_fausses(self):
        self.titre.config(text="Fausses apogées au sol (carte immobile sur la rampe)")
        if self.pas_de_vol():
            return
        v = self.vol
        ks = [k for k in range(v.k_decollage) if v.phase[k] == "PRE_LAUNCH"]
        if len(ks) < 10:
            self.graphe.tracer([], "", "", message="Pas assez de mesures au sol")
            return
        duree = v.t[ks[-1]] - v.t[ks[0]]
        det = ApogeeDetection()
        n_seuil = 0
        for k in ks:
            if det.detection(v.d["baro"][k]):
                n_seuil += 1
                det.reset()                     # la carte se rearme
        comptes = [("seuil 0,51 m baro brut", n_seuil)]
        for nn in (3, 5, 10):
            serie, c = 0, 0
            for i in range(1, len(ks)):
                serie = serie + 1 if v.d["baro"][ks[i]] < v.d["baro"][ks[i - 1]] else 0
                if serie >= nn:
                    c += 1
                    serie = 0
            comptes.append((f"{nn} descentes de suite", c))
        cadence = (len(ks) - 1) / duree
        x = list(range(len(comptes)))
        self.graphe.tracer([{"x": x, "y": [c for _, c in comptes], "style": "barres",
                             "largeur": 0.6}],
                           "méthode : " + "   ".join(f"{i} = {nom}" for i, (nom, _) in
                                                     enumerate(comptes)),
                           "fausses apogées (nombre)", x_lim=(-0.5, len(comptes) - 0.5))
        theorie = "   ".join(f"N={nn} : {nombre(cadence * duree / math.factorial(nn + 1), 0)}"
                             for nn in (3, 5, 10))
        self.infos.config(text=(
            "   ".join(f"{nom} : {c} (une toutes les "
                       f"{nombre(duree / c, 1) if c else '...'} s)" for nom, c in comptes) +
            f"\nsur {nombre(duree, 0)} s au sol, {len(ks)} mesures ({nombre(cadence, 1)} par "
            f"seconde). Théorie 1/(N+1)! sans réarmement : {theorie}"))

    def vue_v_accel(self):
        self.titre.config(text="Accéléromètre et phases du vol")
        if self.pas_de_vol():
            return
        v = self.vol
        zoom = self.options_vue.get("zoom_accel", "combustion")
        if zoom == "combustion":
            lim = (-0.5, v.te(v.k_fin_combustion) + 1.5)
        elif zoom == "rampe":
            lim = (-6.0, 0.5)
        else:
            lim = (-1.0, v.te(v.k_atterrissage) + 2)
        ks = v.indices(*lim)
        sol = [k for k in range(len(v.t)) if -5.0 < v.t[k] < -0.1]
        a1 = moyenne([v.d["a_lat1"][k] for k in sol])
        a2 = moyenne([v.d["a_lat2"][k] for k in sol])
        a3 = moyenne([v.d["a_long"][k] for k in sol])
        angle = math.degrees(math.atan2(math.sqrt(a1 * a1 + a2 * a2), a3))
        norme = math.sqrt(a1 * a1 + a2 * a2 + a3 * a3)
        self.graphe.tracer(
            [{"x": [v.t[k] for k in ks], "y": [v.d["a_long"][k] for k in ks],
              "style": "ligne", "couleur": NOIR, "epaisseur": 3, "legende": "a_long"},
             {"x": [v.t[k] for k in ks], "y": [v.norme[k] for k in ks],
              "style": "ligne", "couleur": GRIS_CLAIR, "epaisseur": 2,
              "legende": "norme des 3 axes"}],
            "temps depuis le décollage (s)", "force spécifique f (m/s²)", x_lim=lim,
            verticales=self.evenements(),
            horizontales=[(SEUIL_DECOLLAGE, "seuil de décollage 19,6"),
                          (0.0, "seuil de fin de poussée 0", "pointilles"),
                          (G, "g", "pointilles")])
        duree = v.te(v.k_fin_combustion) - v.te(v.k_debut_combustion)
        k_libre = v.indices(v.te(v.k_fin_combustion) + 0.1, v.te(v.k_fin_combustion) + 1.1)
        f_libre = moyenne([v.d["a_long"][k] for k in k_libre])
        self.infos.config(text=(
            f"combustion : {nombre(duree, 2)} s (fiche : 3,05 s)      a_long maximal : "
            f"{nombre(max(v.d['a_long'][k] for k in ks), 1)} m/s²      "
            f"vol libre, 1re seconde : f = {nombre(f_libre, 2)} m/s² (= -D/m)\n"
            f"sur la rampe (5 s avant) : angle {nombre(angle, 1)}°, norme "
            f"{nombre(norme, 2)} m/s² (g = 9,81)      Le seuil de -12 m/s² de la première "
            "version ne pouvait jamais être franchi : f ne descend pas si bas en vol libre."))

    def points_trainee(self):
        """(v^2, -f) en vol libre, de la fin de poussee au sommet."""
        v = self.vol
        t1 = v.te(v.k_fin_combustion) + 0.05
        t2 = v.t[v.k_haut]
        return [(v.d["kal_v"][k] ** 2, -v.d["a_long"][k], v.d["kal_v"][k])
                for k in v.indices(t1, t2)]

    def c_ajuste(self, v_min):
        pts = [(x, y) for x, y, vz in self.points_trainee() if vz > v_min]
        den = sum(x * x for x, _ in pts)
        return (sum(x * y for x, y in pts) / den if den > 0 else 0.0), len(pts)

    def vue_v_trainee(self):
        self.titre.config(text="Traînée en vol libre : -f = D/m en fonction de v²")
        if self.pas_de_vol():
            return
        v_min = self.options_vue.get("v_min", 15.0)
        pts = self.points_trainee()
        c, n = self.c_ajuste(v_min)
        x_max = max(x for x, _, _ in pts) if pts else 1
        self.graphe.tracer(
            [{"x": [x for x, _, _ in pts], "y": [y for _, y, _ in pts], "style": "points",
              "couleur": NOIR, "epaisseur": 3, "legende": "mesures en vol libre"},
             {"x": [0, x_max], "y": [0, c * x_max], "style": "ligne", "couleur": NOIR,
              "epaisseur": 2, "legende": "droite ajustée (passe par 0)"},
             {"x": [0, x_max], "y": [0, C_MEMOIRE * x_max], "style": "tirets",
              "couleur": GRIS, "epaisseur": 2, "legende": "c du mémoire"}],
            "carré de la vitesse verticale du Kalman, v² (m²/s²)",
            "-f = D/m (m/s²)", x_lim=(0, x_max * 1.03))
        self.infos.config(text=(
            f"pente ajustée c = {nombre(1000 * c, 2)} x 10^-3 par m   ({n} points avec "
            f"v > {nombre(v_min, 0)} m/s)      mémoire : 1,4 x 10^-3 par m\n"
            "Attention : v est ici la vitesse VERTICALE du Kalman. Elle n'est proche de la "
            "vitesse totale qu'en début de vol libre, quand la fusée est presque verticale. "
            "Près du sommet, la vitesse horizontale (~28 m/s) n'est pas comptée."))

    def vue_v_poussee(self):
        self.titre.config(text="Poussée reconstruite : T = m (f + c v²)")
        if self.pas_de_vol():
            return
        v = self.vol
        choix = self.options_vue.get("c_poussee", C_MEMOIRE)
        c = self.c_ajuste(15.0)[0] if choix == "ajuste" else choix
        k1, k2 = v.k_debut_combustion, v.k_fin_combustion
        tb = v.t[k2] - v.t[k1]
        x, y, y_sans = [], [], []
        impulsion, precedent = 0.0, None
        for k in range(k1, k2 + 1):
            m = MASSE_DECOLLAGE - (MASSE_DECOLLAGE - MASSE_FIN) * (v.t[k] - v.t[k1]) / tb
            poussee = m * (v.d["a_long"][k] + c * v.d["kal_v"][k] ** 2)
            x.append(v.t[k])
            y.append(poussee)
            y_sans.append(m * v.d["a_long"][k])
            if precedent is not None:
                impulsion += 0.5 * (poussee + precedent[1]) * (v.t[k] - precedent[0])
            precedent = (v.t[k], poussee)
        moy = impulsion / tb if tb > 0 else 0
        self.graphe.tracer(
            [{"x": x, "y": y, "style": "ligne", "couleur": NOIR, "epaisseur": 3,
              "legende": "poussée T"},
             {"x": x, "y": y_sans, "style": "tirets", "couleur": GRIS, "epaisseur": 2,
              "legende": "m x f seul (sans la traînée)"}],
            "temps depuis le décollage (s)", "poussée (N)",
            x_lim=(v.t[k1] - 0.2, v.t[k2] + 0.3),
            horizontales=[(moy, f"moyenne mesurée {nombre(moy, 1)} N"),
                          (POUSSEE_NAR, "moyenne de la fiche 25,5 N", "pointilles")])
        self.infos.config(text=(
            f"impulsion = aire sous la courbe = {nombre(impulsion, 1)} N.s      combustion "
            f"{nombre(tb, 2)} s      poussée moyenne {nombre(moy, 1)} N      "
            f"c = {nombre(1000 * c, 2)} x 10^-3 par m\n"
            f"mémoire : {IMPULSION_MEMOIRE}      fiche : {nombre(IMPULSION_NAR, 1)} N.s "
            "      Ici v vient du Kalman (dans le mémoire : d'une simulation qui reproduit "
            "l'altitude mesurée) ; la masse va de 812 à 776 g."))

    def vue_v_bruit(self):
        self.titre.config(text="Bruit du baromètre au sol, avant le décollage")
        if self.pas_de_vol():
            return
        v = self.vol
        duree = self.options_vue.get("duree_sol", 60.0)
        ks = [k for k in range(v.k_decollage) if v.t[k] > -duree - 0.5 and v.t[k] < -0.5
              and v.phase[k] == "PRE_LAUNCH"]
        if len(ks) < 10:
            self.graphe.tracer([], "", "", message="Pas assez de mesures au sol")
            return
        alt = [100 * v.d["baro"][k] for k in ks]
        d = [alt[i + 1] - alt[i] for i in range(len(alt) - 1)]
        centres, effectifs, largeur = histogramme(d)
        g = gaussienne(centres, len(d), moyenne(d), ecart_type(d), largeur)
        self.graphe.tracer([{"x": centres, "y": effectifs, "style": "barres",
                             "largeur": largeur, "legende": "différences mesurées"},
                            {"x": centres, "y": g, "style": "ligne", "couleur": NOIR,
                             "epaisseur": 3, "legende": "loi normale de même " + SIGMA}],
                           "différence entre deux mesures successives (cm)",
                           "nombre de mesures")
        t_sol = v.t[ks[-1]] - v.t[ks[0]]
        self.infos.config(text=(
            f"{len(alt)} mesures sur {nombre(t_sol, 0)} s ({nombre((len(alt) - 1) / t_sol, 1)}"
            f" par seconde)      {SIGMA} naïf : {nombre(ecart_type(alt), 1)} cm      "
            f"{SIGMA}d / {RACINE}2 : {nombre(ecart_type(d) / math.sqrt(2), 1)} cm      "
            f"r1 = {nombre(autocorrelation_1(d), 3)} (théorie -0,5)\n"
            "Au sol, la carte tournait avec tous ses périphériques (radio, GPS) : cadence "
            "plus basse qu'en vol."))

    def vue_v_gps(self):
        self.titre.config(text="Trajectoire GPS vue de dessus")
        if self.pas_de_vol():
            return
        v = self.vol
        ks = [k for k in range(v.k_decollage - 50, len(v.t))
              if v.d["lat"][k] != 0 and v.d["lon"][k] != 0 and v.t[k] <= v.te(v.k_atterrissage) + 20]
        if len(ks) < 2:
            self.graphe.tracer([], "", "", message="Pas de position GPS dans ce journal")
            return
        k_dep = min(ks, key=lambda k: abs(v.t[k]))
        lat0, lon0 = v.d["lat"][k_dep], v.d["lon"][k_dep]
        xy = [gps_en_metres(v.d["lat"][k], v.d["lon"][k], lat0, lon0) for k in ks]
        k_haut = min(ks, key=lambda k: abs(v.t[k] - v.t[v.k_haut]))
        xh, yh = gps_en_metres(v.d["lat"][k_haut], v.d["lon"][k_haut], lat0, lon0)
        xa, ya = xy[-1]
        # premier instant ou la position GPS s'eloigne de plus de 5 m de la rampe
        t_bouge = next((v.t[k] for k, p in zip(ks, xy)
                        if v.t[k] > 0 and math.hypot(p[0], p[1]) > 5.0), None)
        self.graphe.tracer(
            [{"x": [p[0] for p in xy], "y": [p[1] for p in xy], "style": "ligne",
              "couleur": GRIS, "epaisseur": 2, "legende": "trajectoire"},
             {"x": [xa], "y": [ya], "style": "points", "couleur": GRIS_CLAIR, "epaisseur": 7,
              "legende": "atterrissage"},
             {"x": [xh], "y": [yh], "style": "points", "couleur": GRIS, "epaisseur": 7,
              "legende": "position lue au sommet"},
             {"x": [0.0], "y": [0.0], "style": "points", "couleur": NOIR, "epaisseur": 5,
              "legende": "rampe"}],
            "vers l'est (m)", "vers le nord (m)", meme_echelle=True, y_etendue_min=20.0)
        retard = ""
        if t_bouge is not None and t_bouge > v.t[v.k_haut]:
            retard = (f"Le GPS ne s'éloigne de la rampe qu'à t = {nombre(t_bouge, 1)} s, après "
                      f"le sommet ({nombre(v.t[v.k_haut], 1)} s) : pendant la montée, sa "
                      "position est en retard ou figée. La position lue au sommet n'est donc "
                      "pas la vraie. ")
        self.infos.config(text=(
            f"atterrissage à {nombre(math.hypot(xa, ya), 0)} m de la rampe      "
            f"(OpenRocket : 108 m de dérive latérale)\n" + retard +
            "Le GPS donne environ une position par seconde, à quelques mètres près."))

    def vue_v_gps_alt(self):
        self.titre.config(text="Altitude : GPS contre baromètre")
        if self.pas_de_vol():
            return
        v = self.vol
        t_fin = v.te(v.k_atterrissage) + 5
        ks = [k for k in v.indices(-5, t_fin) if v.d["lat"][k] != 0]
        if len(ks) < 2:
            self.graphe.tracer([], "", "", message="Pas d'altitude GPS dans ce journal")
            return
        k_dep = min(ks, key=lambda k: abs(v.t[k]))
        g0 = v.d["alt_gps"][k_dep]
        self.graphe.tracer(
            [{"x": [v.t[k] for k in ks], "y": [v.d["alt_gps"][k] - g0 for k in ks],
              "style": "ligne", "couleur": NOIR, "epaisseur": 3, "legende": "GPS"},
             {"x": [v.t[k] for k in ks], "y": [v.h[k] for k in ks], "style": "points",
              "couleur": GRIS_CLAIR, "epaisseur": 2, "legende": "baromètre"}],
            "temps depuis le décollage (s)", "altitude au-dessus de la rampe (m)",
            x_lim=(-5, t_fin), verticales=self.evenements())
        self.infos.config(text=(
            f"maximum GPS : {nombre(max(v.d['alt_gps'][k] for k in ks) - g0, 1)} m      "
            f"maximum baromètre : {nombre(max(v.h[k] for k in ks), 1)} m\n"
            "Le GPS est lent (une position par seconde) et incertain de plusieurs mètres en "
            "altitude : il ne peut pas servir à détecter l'apogée."))

    def vue_v_temp(self):
        self.titre.config(text="Température du capteur et batterie, sur tout le journal")
        if self.pas_de_vol():
            return
        v = self.vol
        pas = max(1, len(v.t) // 3000)
        ks = range(0, len(v.t), pas)
        self.graphe.tracer(
            [{"x": [v.t[k] / 60 for k in ks], "y": [v.d["temp"][k] for k in ks],
              "style": "ligne", "couleur": NOIR, "epaisseur": 3, "legende": "température"}],
            "temps depuis le décollage (min)", "température du baromètre (°C)",
            verticales=[(0.0, "décollage", "tirets")])
        b = [x for x in v.d["batt"] if x > 0]
        self.infos.config(text=(
            f"température : de {nombre(min(v.d['temp']), 1)} à {nombre(max(v.d['temp']), 1)}"
            f" °C      batterie : de {nombre(min(b), 2) if b else '?'} à "
            f"{nombre(max(b), 2) if b else '?'} V\nLa formule de l'altitude suppose 15 °C au "
            "sol : chaque degré de plus sous-estime l'altitude d'environ 0,35 %."))

    # ---- export
    def exporter(self):
        chemin = filedialog.asksaveasfilename(defaultextension=".eps",
                                              filetypes=[("PostScript", "*.eps")])
        if chemin:
            self.zone.postscript(file=chemin, colormode="gray")
            self.texte_reponse.config(text="Image enregistrée : " + chemin)


if __name__ == "__main__":
    racine = tk.Tk()
    Application(racine)
    racine.mainloop()