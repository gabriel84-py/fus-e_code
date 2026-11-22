# bruit_baro.py - LUNATIK, demo de l'oral : combien se trompe le barometre ?
#
# La carte (code.py = demo_usb.py) est posee, IMMOBILE. Le programme trace en
# direct l'altitude que donne le barometre, tres zoomee (+-1 m autour de la
# moyenne des 10 dernieres secondes) : la carte ne bouge pas, mais la mesure
# tremble de plusieurs dizaines de centimetres.
#
# Il calcule en direct, sur toutes les mesures depuis le lancement :
#   - l'ecart-type naif des altitudes (il compte aussi la derive lente)
#   - l'ecart-type par differences successives : sigma_d / racine(2),
#     qui ne garde que le bruit (methode de la partie 2 du memoire)
#
# Installation : pip install pyserial
# Lancement    : python bruit_baro.py          (port trouve tout seul)
#                python bruit_baro.py COM5     (Windows, port impose)
# Touches      : Espace = PAUSE : tout s'arrete, l'ecran reste tel quel ;
#                         Espace encore = reprise du direct
#                R = remettre les ecarts-types a zero
#                F11 = plein ecran oui/non, Echap = quitter

import math
import sys
import tkinter as tk

import serial
from serial.tools import list_ports

VID_PICO = 0x2E8A        # identifiant USB des cartes Raspberry Pi (Pico 2)
BAUDS = 115200           # ignore par la console USB du Pico, mais obligatoire

FENETRE_S = 10.0         # duree affichee a l'ecran (s)
ZOOM_CM = 100            # echelle verticale : +- 100 cm autour de la moyenne

# position des colonnes dans une trame (meme ordre que le journal SD)
COL_T, COL_ALT_BARO = 0, 10
NB_COLONNES = 17

# symbole racine ecrit en code unicode : le fichier reste en caracteres du
# clavier, l'ecran affiche le vrai symbole
RACINE = "\u221a"

# ---- etat du programme -------------------------------------------------------
port_serie = None
tampon = ""
mesures = []             # liste de (t_carte, altitude en cm), 20 dernieres s
pause = False
derniere_alt = None      # derniere altitude, pour la difference suivante


class Statistique:
    """Moyenne et ecart-type calcules au fil de l'eau (algorithme de Welford).

    Evite de garder toutes les mesures en memoire et reste precis meme
    apres des dizaines de milliers de valeurs.
    """

    def __init__(self):
        self.n = 0
        self.moyenne = 0.0
        self.m2 = 0.0

    def ajouter(self, x):
        self.n += 1
        ecart = x - self.moyenne
        self.moyenne += ecart / self.n
        self.m2 += ecart * (x - self.moyenne)

    def ecart_type(self):
        if self.n < 2:
            return 0.0
        return math.sqrt(self.m2 / (self.n - 1))


stat_alt = Statistique()     # les altitudes elles-memes
stat_diff = Statistique()    # les differences entre deux mesures successives
t_premier = None             # temps carte de la premiere mesure comptee
t_dernier = None


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


def remettre_a_zero(_evenement=None):
    global stat_alt, stat_diff, t_premier, t_dernier, derniere_alt
    stat_alt = Statistique()
    stat_diff = Statistique()
    t_premier = t_dernier = None
    derniere_alt = None


def traiter_ligne(ligne):
    """Ajoute une mesure a la courbe et aux deux ecarts-types."""
    global derniere_alt, t_premier, t_dernier
    champs = ligne.strip().split(",")
    if len(champs) != NB_COLONNES:
        return
    try:
        t = float(champs[COL_T])
        alt_cm = 100.0 * float(champs[COL_ALT_BARO])
    except ValueError:
        return                      # ligne d'en-tete ou message de la carte
    mesures.append((t, alt_cm))
    while mesures and mesures[0][0] < t - 2 * FENETRE_S:
        mesures.pop(0)

    stat_alt.ajouter(alt_cm)
    if derniere_alt is not None:
        stat_diff.ajouter(alt_cm - derniere_alt)
    derniere_alt = alt_cm
    if t_premier is None:
        t_premier = t
    t_dernier = t


def lire_port():
    """Appelee toutes les 10 ms : vide ce que la carte a envoye."""
    global port_serie, tampon
    if port_serie is None:
        connecter()
    else:
        try:
            if pause:
                # en pause, on jette ce que la carte envoie : rien n'entre
                # dans la courbe ni dans les calculs
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


def basculer_pause(_evenement=None):
    """Espace : met en pause, ou reprend le direct si on etait en pause."""
    global pause, tampon, derniere_alt
    if not pause:
        pause = True
    else:
        pause = False
        tampon = ""
        mesures.clear()             # la courbe repart propre, sans trou
        derniere_alt = None         # pas de difference a cheval sur la pause


# ---- dessin ------------------------------------------------------------------
def dessiner():
    """Appelee toutes les 40 ms : redessine tout le graphique."""
    if pause:
        # rien ne change en pause : l'ecran reste exactement tel quel
        fenetre.after(40, dessiner)
        return

    zone.delete("all")
    largeur = zone.winfo_width()
    hauteur = zone.winfo_height()
    gauche, droite = int(largeur * 0.09), largeur - 40
    haut, bas = 20, hauteur - int(hauteur * 0.10)

    fin = mesures[-1][0] if mesures else FENETRE_S
    debut = fin - FENETRE_S
    visibles = [m for m in mesures if m[0] >= debut]
    centre = sum(m[1] for m in visibles) / len(visibles) if visibles else 0.0

    def x(t):
        return gauche + (t - debut) / FENETRE_S * (droite - gauche)

    def y(alt_cm):
        ecart = max(-ZOOM_CM, min(ZOOM_CM, alt_cm - centre))
        return bas - (ecart + ZOOM_CM) / (2 * ZOOM_CM) * (bas - haut)

    # graduations verticales, en cm autour de la moyenne
    for e in range(-ZOOM_CM, ZOOM_CM + 1, 25):
        zone.create_line(gauche, y(centre + e), droite, y(centre + e), fill="#dddddd")
        texte = f"+{e}" if e > 0 else str(e)
        zone.create_text(gauche - 12, y(centre + e), text=texte, anchor="e", font=petit)
    zone.create_text(gauche - 12, haut - 2, text="cm", anchor="se", font=petit)
    zone.create_line(gauche, y(centre), droite, y(centre), fill="black", width=2, dash=(10, 8))
    zone.create_text(droite, y(centre) - 6, text="moyenne des 10 s", anchor="se", font=petit)

    # graduations horizontales : une par seconde, comptees depuis la fin
    for k in range(int(FENETRE_S) + 1):
        xx = x(fin - k)
        zone.create_line(xx, bas, xx, bas + 10, fill="black", width=2)
        zone.create_text(xx, bas + 14, text=f"-{k} s" if k else "0 s", anchor="n", font=petit)
    zone.create_line(gauche, bas, droite, bas, fill="black", width=2)

    # la courbe : un point par mesure, relies par un trait fin
    if len(visibles) >= 2:
        points = []
        for t, alt_cm in visibles:
            points += [x(t), y(alt_cm)]
        zone.create_line(*points, fill="#999999", width=2)
        for t, alt_cm in visibles:
            r = 4
            zone.create_oval(x(t) - r, y(alt_cm) - r, x(t) + r, y(alt_cm) + r,
                             fill="black", outline="")

    # les deux ecarts-types, en gros
    if stat_diff.n >= 2:
        naif = stat_alt.ecart_type()
        bruit = stat_diff.ecart_type() / math.sqrt(2)
        duree = t_dernier - t_premier
        minutes, secondes = int(duree // 60), int(duree % 60)
        valeur_naif.config(text=f"écart-type naïf : {virgule(naif, 0)} cm")
        valeur_bruit.config(text=f"différences successives : {chr(963)}d / {RACINE}2 = "
                                 f"{virgule(bruit, 0)} cm")
        bilan.config(text=f"calculés sur {stat_alt.n} mesures ({minutes} min {secondes:02d} s)"
                          f"     (Espace : pause, R : remise à zéro)")
    else:
        valeur_naif.config(text="écart-type naïf : ...")
        valeur_bruit.config(text="différences successives : ...")
        bilan.config(text="Posez la carte et ne la touchez plus")

    if port_serie is None:
        message_etat.config(text="Carte non trouvée : branche le câble USB")
    fenetre.after(40, dessiner)


def afficher_pause():
    """Le titre dit si on est en pause (appelee a chaque appui sur Espace)."""
    if pause:
        titre.config(text="PAUSE     (Espace : reprendre le direct)")
    else:
        titre.config(text="Altitude du baromètre, carte immobile")


def touche_espace(_evenement=None):
    basculer_pause()
    afficher_pause()


def basculer_plein_ecran(_evenement=None):
    fenetre.attributes("-fullscreen", not fenetre.attributes("-fullscreen"))


# ---- interface ---------------------------------------------------------------
fenetre = tk.Tk()
fenetre.title("LUNATIK - bruit du baromètre")
fenetre.configure(bg="white")
fenetre.attributes("-fullscreen", True)
fenetre.bind("<F11>", basculer_plein_ecran)
fenetre.bind("<Escape>", lambda e: fenetre.destroy())
fenetre.bind("<space>", touche_espace)
fenetre.bind("<r>", remettre_a_zero)
fenetre.bind("<R>", remettre_a_zero)

h = fenetre.winfo_screenheight()
petit = ("Helvetica", h // 45)
moyen = ("Helvetica", h // 28)
grand = ("Helvetica", h // 20, "bold")

titre = tk.Label(fenetre, text="", font=moyen, bg="white", fg="black")
titre.pack(pady=(15, 0))
valeur_naif = tk.Label(fenetre, text="", font=moyen, bg="white", fg="black")
valeur_naif.pack()
valeur_bruit = tk.Label(fenetre, text="", font=grand, bg="white", fg="black")
valeur_bruit.pack()
zone = tk.Canvas(fenetre, bg="white", highlightthickness=0)
zone.pack(fill="both", expand=True, padx=20)
bilan = tk.Label(fenetre, text="", font=petit, bg="white", fg="black")
bilan.pack(pady=5)
message_etat = tk.Label(fenetre, text="Recherche de la carte...", font=petit,
                        bg="white", fg="#666666")
message_etat.pack(pady=(0, 10))
afficher_pause()

fenetre.after(10, lire_port)
fenetre.after(40, dessiner)
fenetre.mainloop()

if port_serie is not None:
    port_serie.close()