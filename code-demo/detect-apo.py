# detection_apogee.py - LUNATIK, demo 3 de l'oral : la vraie detection
#
# La carte (code.py = demo_usb.py) est reliee au PC. On l'arme avec la touche
# A, on la monte d'environ 1,5 m le long d'une regle, puis on redescend.
# L'ecran trace en direct :
#   - l'altitude brute du barometre (points gris)
#   - l'altitude du filtre de Kalman de la carte (trait noir)
#   - le maximum retenu et le seuil 51 cm en dessous (pointilles)
# La detection est faite par apogee.py, le MEME fichier que dans la fusee :
# seuil de 0,51 m sous le maximum, confirme sur 2 mesures, sur le Kalman.
# A la detection, l'ecran montre de combien et combien de temps apres le
# sommet elle a eu lieu, puis la courbe se fige.
#
# Le fichier apogee.py du dossier src de la carte doit etre a cote de ce
# programme.
#
# Installation : pip install pyserial
# Lancement    : python detection_apogee.py          (port trouve tout seul)
#                python detection_apogee.py COM5     (Windows, port impose)
# Touches      : A = armer (ou rearmer pour un nouvel essai)
#                Espace = PAUSE : tout s'arrete, l'ecran reste tel quel ;
#                         Espace encore = reprise du direct
#                F11 = plein ecran oui/non, Echap = quitter

import sys
import tkinter as tk

import serial
from serial.tools import list_ports

from apogee import ApogeeDetection     # le detecteur qui vole dans la fusee

VID_PICO = 0x2E8A        # identifiant USB des cartes Raspberry Pi (Pico 2)
BAUDS = 115200           # ignore par la console USB du Pico, mais obligatoire

FENETRE_S = 10.0         # duree affichee a l'ecran (s)
BAS_M, HAUT_M = -1.0, 2.5    # echelle verticale, autour du point d'armement (m)
APRES_S = 1.0            # on montre encore 1 s apres la detection, puis on fige

# position des colonnes dans une trame (meme ordre que le journal SD)
COL_T, COL_ALT_BARO, COL_Z_KALMAN = 0, 10, 14
NB_COLONNES = 17

# ---- etat du programme -------------------------------------------------------
port_serie = None
tampon = ""
mesures = []             # liste de (t_carte, altitude baro, altitude Kalman)
pause = False

etat = "desarmee"        # "desarmee", "armee" ou "apogee"
detecteur = ApogeeDetection()
reference = None         # altitude Kalman au moment de l'armement (le zero)
t_sommet = h_sommet = None
t_detection = h_detection = None


def virgule(x, n):
    """Nombre avec n decimales et une virgule, a la francaise."""
    return f"{x:.{n}f}".replace(".", ",")


def trouver_port():
    """Port impose en argument, sinon le premier port d'une carte Pico."""
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = [p for p in list_ports.comports() if p.vid == VID_PICO]
    if not ports:
        return None
    ports.sort(key=lambda p: p.device)
    return ports[0].device


def connecter():
    global port_serie, tampon
    nom = trouver_port()
    if nom is None:
        return
    try:
        port_serie = serial.Serial(nom, BAUDS, timeout=0)
        port_serie.reset_input_buffer()
        tampon = ""
        message_etat.config(text="Carte connectée sur " + nom)
    except serial.SerialException:
        port_serie = None


def traiter_ligne(ligne):
    """Une mesure de plus : courbe, maximum et detection."""
    global etat, t_sommet, h_sommet, t_detection, h_detection
    champs = ligne.strip().split(",")
    if len(champs) != NB_COLONNES:
        return
    try:
        t = float(champs[COL_T])
        baro = float(champs[COL_ALT_BARO])
        kal = float(champs[COL_Z_KALMAN])
    except ValueError:
        return                      # ligne d'en-tete ou message de la carte
    mesures.append((t, baro, kal))
    while mesures and mesures[0][0] < t - 2 * FENETRE_S:
        mesures.pop(0)

    if etat == "armee":
        if detecteur.maxi is None or kal >= detecteur.maxi:
            t_sommet, h_sommet = t, kal          # nouveau maximum
        if detecteur.detection(kal):
            etat = "apogee"
            t_detection, h_detection = t, kal


def lire_port():
    """Appelee toutes les 10 ms : vide ce que la carte a envoye."""
    global port_serie, tampon
    if port_serie is None:
        connecter()
    else:
        try:
            if pause:
                # en pause, on jette ce que la carte envoie : rien ne change
                port_serie.reset_input_buffer()
            else:
                n = port_serie.in_waiting
                if n > 0:
                    tampon += port_serie.read(n).decode("utf-8", errors="ignore")
                    lignes = tampon.split("\n")
                    tampon = lignes.pop()   # derniere ligne peut etre coupee
                    for ligne in lignes:
                        traiter_ligne(ligne)
        except (serial.SerialException, OSError):
            port_serie = None
            message_etat.config(text="Carte débranchée, en attente...")
    fenetre.after(10, lire_port)


def armer(_evenement=None):
    """Touche A : nouvel essai, le detecteur repart de zero."""
    global etat, reference, t_sommet, h_sommet, t_detection, h_detection
    if pause or not mesures:
        return
    detecteur.reset()
    etat = "armee"
    reference = mesures[-1][2]
    t_sommet = h_sommet = t_detection = h_detection = None


def basculer_pause(_evenement=None):
    """Espace : met en pause, ou reprend le direct si on etait en pause."""
    global pause, tampon
    if not pause:
        pause = True
    else:
        pause = False
        tampon = ""


# ---- dessin ------------------------------------------------------------------
def fleche(x1, y1, x2, y2):
    zone.create_line(x1, y1, x2, y2, fill="black", width=3, arrow="both",
                     arrowshape=(16, 20, 6))


def dessiner():
    """Appelee toutes les 40 ms : redessine tout le graphique."""
    if pause:
        textes()
        fenetre.after(40, dessiner)
        return

    zone.delete("all")
    largeur = zone.winfo_width()
    hauteur = zone.winfo_height()
    gauche, droite = int(largeur * 0.09), largeur - 40
    haut, bas = 20, hauteur - int(hauteur * 0.10)

    if mesures:
        fin = mesures[-1][0]
        if etat == "apogee":
            fin = min(fin, t_detection + APRES_S)    # la courbe se fige
    else:
        fin = FENETRE_S
    debut = fin - FENETRE_S
    visibles = [m for m in mesures if debut <= m[0] <= fin]

    # zero de l'echelle : le point d'armement, sinon la moyenne recente
    if reference is not None:
        zero = reference
    elif visibles:
        zero = sum(m[2] for m in visibles) / len(visibles)
    else:
        zero = 0.0

    def x(t):
        return gauche + (t - debut) / FENETRE_S * (droite - gauche)

    def y(alt):
        e = max(BAS_M, min(HAUT_M, alt - zero))
        return bas - (e - BAS_M) / (HAUT_M - BAS_M) * (bas - haut)

    # graduations verticales tous les 50 cm, comptees depuis le zero
    e = BAS_M
    while e <= HAUT_M + 1e-9:
        zone.create_line(gauche, y(zero + e), droite, y(zero + e), fill="#dddddd")
        texte = ("+" if e > 0 else "") + virgule(e, 1)
        zone.create_text(gauche - 12, y(zero + e), text=texte, anchor="e", font=petit)
        e += 0.5
    zone.create_text(gauche - 12, haut - 2, text="m", anchor="se", font=petit)

    # graduations horizontales : une par seconde, comptees depuis la fin
    for k in range(int(FENETRE_S) + 1):
        xx = x(fin - k)
        zone.create_line(xx, bas, xx, bas + 10, fill="black", width=2)
        zone.create_text(xx, bas + 14, text=f"-{k} s" if k else "0 s", anchor="n", font=petit)
    zone.create_line(gauche, bas, droite, bas, fill="black", width=2)

    # maximum retenu et seuil 51 cm en dessous
    if etat in ("armee", "apogee") and h_sommet is not None:
        zone.create_line(gauche, y(h_sommet), droite, y(h_sommet),
                         fill="black", width=2, dash=(10, 8))
        zone.create_text(gauche + 8, y(h_sommet) - 4, text="maximum", anchor="sw", font=petit)
        seuil = h_sommet - detecteur.seuil
        zone.create_line(gauche, y(seuil), droite, y(seuil),
                         fill="#888888", width=2, dash=(4, 6))
        zone.create_text(gauche + 8, y(seuil) - 4, text="maximum - 51 cm",
                         anchor="sw", font=petit, fill="#666666")

    # courbes : baro brut en points gris, Kalman en trait noir
    if len(visibles) >= 2:
        for t, baro, kal in visibles:
            r = 4
            zone.create_oval(x(t) - r, y(baro) - r, x(t) + r, y(baro) + r,
                             fill="#9a9a9a", outline="")
        points = []
        for t, baro, kal in visibles:
            points += [x(t), y(kal)]
        zone.create_line(*points, fill="black", width=5)

    # resultat de la detection : sommet, point de detection, ecarts
    if etat == "apogee":
        r = 11
        xs, ys = x(t_sommet), y(h_sommet)
        xd, yd = x(t_detection), y(h_detection)
        zone.create_oval(xs - r, ys - r, xs + r, ys + r, fill="black", outline="")
        zone.create_oval(xd - r, yd - r, xd + r, yd + r, fill="white", outline="black", width=4)
        fleche(xd + 40, ys, xd + 40, yd)
        zone.create_text(xd + 55, (ys + yd) / 2, anchor="w", font=moyen_gras,
                         text=f"{virgule(100 * (h_sommet - h_detection), 0)} cm")
        fleche(xs, ys - 45, xd, ys - 45)
        zone.create_text((xs + xd) / 2, ys - 55, anchor="s", font=moyen_gras,
                         text=f"{virgule(t_detection - t_sommet, 2)} s")

    # legende
    zone.create_oval(droite - 340, haut + 12, droite - 330, haut + 22, fill="#9a9a9a", outline="")
    zone.create_text(droite - 315, haut + 17, text="baromètre brut", anchor="w", font=petit)
    zone.create_line(droite - 350, haut + 55, droite - 320, haut + 55, fill="black", width=5)
    zone.create_text(droite - 315, haut + 55, text="Kalman de la carte", anchor="w", font=petit)

    textes()
    if port_serie is None:
        message_etat.config(text="Carte non trouvée : branche le câble USB")
    fenetre.after(40, dessiner)


def textes():
    """Les lignes de texte au-dessus et au-dessous du graphique."""
    debut_txt = "PAUSE - " if pause else ""
    if etat == "desarmee":
        statut.config(text=debut_txt + "Carte désarmée")
        consigne.config(text="A : armer     Espace : pause")
    elif etat == "armee":
        statut.config(text=debut_txt + "Carte ARMÉE")
        consigne.config(text="Montez la carte le long de la règle, puis redescendez")
    else:
        statut.config(text=debut_txt + "APOGÉE DÉTECTÉE")
        consigne.config(
            text=f"Détection {virgule(100 * (h_sommet - h_detection), 0)} cm sous le sommet, "
                 f"{virgule(t_detection - t_sommet, 2)} s après     "
                 f"(A : nouvel essai, Espace : pause)")


def basculer_plein_ecran(_evenement=None):
    fenetre.attributes("-fullscreen", not fenetre.attributes("-fullscreen"))


# ---- interface ---------------------------------------------------------------
fenetre = tk.Tk()
fenetre.title("LUNATIK - détection d'apogée")
fenetre.configure(bg="white")
fenetre.attributes("-fullscreen", True)
fenetre.bind("<F11>", basculer_plein_ecran)
fenetre.bind("<Escape>", lambda e: fenetre.destroy())
fenetre.bind("<space>", basculer_pause)
fenetre.bind("<a>", armer)
fenetre.bind("<A>", armer)

h = fenetre.winfo_screenheight()
petit = ("Helvetica", h // 45)
moyen_gras = ("Helvetica", h // 30, "bold")
grand = ("Helvetica", h // 16, "bold")

statut = tk.Label(fenetre, text="", font=grand, bg="white", fg="black")
statut.pack(pady=(15, 0))
zone = tk.Canvas(fenetre, bg="white", highlightthickness=0)
zone.pack(fill="both", expand=True, padx=20)
consigne = tk.Label(fenetre, text="", font=moyen_gras, bg="white", fg="black")
consigne.pack(pady=5)
message_etat = tk.Label(fenetre, text="Recherche de la carte...", font=petit,
                        bg="white", fg="#666666")
message_etat.pack(pady=(0, 10))

fenetre.after(10, lire_port)
fenetre.after(40, dessiner)
fenetre.mainloop()

if port_serie is not None:
    port_serie.close()