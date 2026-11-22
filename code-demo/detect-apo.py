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
#
# Comme en vol, il y a deux etapes avant la detection :
#   1. a l'armement (touche A), la carte refait la calibration du biais de
#      l'accelerometre et remet son Kalman a zero (commande "r" de code.py),
#      comme la phase PRE_LAUNCH de la fusee. Il faut la tenir IMMOBILE,
#      dans l'orientation de la demo, pendant ~0,5 s. Sans ce recalage, une
#      carte branchee a plat puis redressee detectait l'apogee trop tot dans
#      100 % des essais simules ;
#   2. la detection ne commence qu'apres une montee de MONTEE_MIN au-dessus
#      du point d'armement : c'est le "decollage" de la demo (en vol : 15 m
#      en 1,5 s). Une porte qui claque ou un courant d'air pendant l'attente
#      ne peuvent donc pas declencher d'apogee.
# A la detection, l'ecran montre de combien et combien de temps apres le
# sommet elle a eu lieu, puis la courbe se fige.
#
# Le fichier apogee.py du dossier src de la carte doit etre a cote de ce
# programme.
#
# Installation : pip install pyserial
# Lancement    : python detection_apogee.py          (port trouve tout seul)
#                python detection_apogee.py COM5     (Windows, port impose)
# Touches      : A = recalage puis armement (ou nouvel essai)
#                Espace = PAUSE : tout s'arrete, l'ecran reste tel quel ;
#                         Espace encore = reprise du direct
#                F11 = plein ecran oui/non, Echap = quitter

import sys
import time
import tkinter as tk

import serial

import cartes                           # choix de la carte A ou B, port exclusif

from apogee import ApogeeDetection     # le detecteur qui vole dans la fusee

CARTE = "A"              # carte des demos (numeros de serie dans cartes.py)

FENETRE_S = 10.0         # duree affichee a l'ecran (s)
BAS_M, HAUT_M = -1.0, 2.5    # echelle verticale, autour du point d'armement (m)
APRES_S = 1.0            # on montre encore 1 s apres la detection, puis on fige
MONTEE_MIN = 0.5         # m : montee a franchir avant que la detection commence
DELAI_RECAL_S = 2.0      # sans reponse de la carte (ancien code.py), on arme quand meme

# position des colonnes dans une trame (meme ordre que le journal SD)
COL_T, COL_ALT_BARO, COL_Z_KALMAN = 0, 10, 14
NB_COLONNES = 17

# ---- etat du programme -------------------------------------------------------
port_serie = None
tampon = ""
t_dernier_essai = 0.0
mesures = []             # liste de (t_carte, altitude baro, altitude Kalman)
pause = False

# "desarmee" -> A -> "calibration" -> "armee" (attend la montee)
#   -> "montee" (detection active) -> "apogee"
etat = "desarmee"
t_debut_calibration = 0.0
avertissement = ""       # message particulier a afficher (carte bougee, etc.)
detecteur = ApogeeDetection()
reference = None         # altitude Kalman au moment de l'armement (le zero)
t_sommet = h_sommet = None
t_detection = h_detection = None


def virgule(x, n):
    """Nombre avec n decimales et une virgule, a la francaise."""
    return f"{x:.{n}f}".replace(".", ",")


def connecter():
    """Cherche la carte (au plus deux fois par seconde) et ouvre son port."""
    global port_serie, tampon, t_dernier_essai
    if time.monotonic() - t_dernier_essai < 0.5:
        return
    t_dernier_essai = time.monotonic()
    nom = cartes.trouver_port(CARTE)
    if nom is not None:
        port_serie = cartes.ouvrir(nom)
        tampon = ""
    message_etat.config(text=cartes.message)


def traiter_ligne(ligne):
    """Une mesure de plus : courbe, maximum et detection."""
    global etat, reference, t_sommet, h_sommet, t_detection, h_detection
    if ligne.startswith("#RECAL"):
        reponse_recalage(ligne)
        return
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
        if reference is None:
            reference = kal                      # premiere mesure apres recalage
        elif kal - reference >= MONTEE_MIN:
            etat = "montee"                      # "decollage" : detection active
            detecteur.reset()

    if etat == "montee":
        if detecteur.maxi is None or kal >= detecteur.maxi:
            t_sommet, h_sommet = t, kal          # nouveau maximum
        if detecteur.detection(kal):
            etat = "apogee"
            t_detection, h_detection = t, kal


def reponse_recalage(ligne):
    """La carte a fini (ou rate) sa calibration."""
    global etat, avertissement
    if etat != "calibration":
        return
    if " ok" in ligne:
        etat = "armee"
        avertissement = ""
    elif "bouge" in ligne:
        etat = "desarmee"
        avertissement = "La carte a bougé pendant la calibration : A pour recommencer"
    else:
        etat = "desarmee"
        avertissement = "Erreur de calibration de la carte : A pour recommencer"


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
            cartes.message = "Carte débranchée, en attente..."
            message_etat.config(text=cartes.message)
    fenetre.after(10, lire_port)


def armer(_evenement=None):
    """Touche A : recalage de la carte, puis nouvel essai."""
    global etat, reference, t_sommet, h_sommet, t_detection, h_detection
    global t_debut_calibration, avertissement
    if pause or not mesures or port_serie is None:
        return
    detecteur.reset()
    reference = None
    t_sommet = h_sommet = t_detection = h_detection = None
    avertissement = ""
    if cartes.envoyer(port_serie, "r"):
        etat = "calibration"
        t_debut_calibration = time.monotonic()
    else:
        etat = "desarmee"
        avertissement = "Impossible de parler à la carte : vérifie le câble"


def verifier_calibration():
    """Ancien code.py sans commande "r" : pas de reponse, on arme quand meme."""
    global etat, avertissement
    if etat == "calibration" and time.monotonic() - t_debut_calibration > DELAI_RECAL_S:
        etat = "armee"
        avertissement = "Carte sans recalage (ancien code.py ?) : armée quand même"


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
    verifier_calibration()
    if pause:
        textes()
        fenetre.after(40, dessiner)
        return

    zone.delete("all")
    largeur = zone.winfo_width()
    hauteur = zone.winfo_height()
    gauche, droite = int(largeur * 0.09), largeur - 40
    haut, bas = 45, hauteur - int(hauteur * 0.10)

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
        zone.create_text(gauche - 12, y(zero + e), text=texte, anchor="e", font=petit, fill="black")
        e += 0.5
    zone.create_text(gauche - 40, haut - 14, text="altitude depuis l'armement (m)", anchor="sw", font=petit, fill="black")

    # graduations horizontales : une par seconde, comptees depuis la fin
    for k in range(int(FENETRE_S) + 1):
        xx = x(fin - k)
        zone.create_line(xx, bas, xx, bas + 10, fill="black", width=2)
        zone.create_text(xx, bas + 14, text=f"-{k} s" if k else "0 s", anchor="n", font=petit, fill="black")
    zone.create_line(gauche, bas, droite, bas, fill="black", width=2)

    # avant la montee : la ligne a franchir pour que la detection commence
    if etat == "armee" and reference is not None:
        zone.create_line(gauche, y(reference + MONTEE_MIN), droite,
                         y(reference + MONTEE_MIN), fill="#888888", width=2, dash=(4, 6))
        zone.create_text(gauche + 8, y(reference + MONTEE_MIN) - 4,
                         text=f"+{virgule(100 * MONTEE_MIN, 0)} cm : la détection commence",
                         anchor="sw", font=petit, fill="#666666")

    # maximum retenu et seuil 51 cm en dessous
    if etat in ("montee", "apogee") and h_sommet is not None:
        zone.create_line(gauche, y(h_sommet), droite, y(h_sommet),
                         fill="black", width=2, dash=(10, 8))
        zone.create_text(gauche + 8, y(h_sommet) - 4, text="maximum", anchor="sw", font=petit, fill="black")
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
                         text=f"{virgule(100 * (h_sommet - h_detection), 0)} cm", fill="black")
        fleche(xs, ys - 45, xd, ys - 45)
        zone.create_text((xs + xd) / 2, ys - 55, anchor="s", font=moyen_gras,
                         text=f"{virgule(t_detection - t_sommet, 2)} s", fill="black")

    # legende
    zone.create_oval(droite - 340, haut + 12, droite - 330, haut + 22, fill="#9a9a9a", outline="")
    zone.create_text(droite - 315, haut + 17, text="baromètre brut", anchor="w", font=petit, fill="black")
    zone.create_line(droite - 350, haut + 55, droite - 320, haut + 55, fill="black", width=5)
    zone.create_text(droite - 315, haut + 55, text="Kalman de la carte", anchor="w", font=petit, fill="black")

    textes()
    if port_serie is None:
        message_etat.config(text=cartes.message)
    fenetre.after(40, dessiner)


def textes():
    """Les lignes de texte au-dessus et au-dessous du graphique."""
    debut_txt = "PAUSE - " if pause else ""
    if etat == "desarmee":
        statut.config(text=debut_txt + "Carte désarmée")
        consigne.config(text=avertissement or "Tenez la carte immobile au pied de la règle,"
                                              " puis A : armer     Espace : pause")
    elif etat == "calibration":
        statut.config(text=debut_txt + "Calibration : ne bougez plus")
        consigne.config(text="La carte mesure le biais de son accéléromètre...")
    elif etat in ("armee", "montee"):
        statut.config(text=debut_txt + "Carte ARMÉE")
        consigne.config(text=avertissement or
                        "Montez la carte le long de la règle, puis redescendez")
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