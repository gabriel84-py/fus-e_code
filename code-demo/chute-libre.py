# chute_libre.py - LUNATIK, demo 1 de l'oral : que mesure un accelerometre ?
#
# Trace en direct ce que lit l'accelerometre de la carte (code.py = demo_usb.py) :
#   - a_long : la mesure sur l'axe de la fusee (+9,8 quand la carte est debout)
#   - |a|    : la norme des 3 axes, racine(a_lat1^2 + a_lat2^2 + a_long^2)
#
# Quand on lache la carte, les deux courbes tombent a 0 : un accelerometre
# ne mesure pas la gravite, seulement les forces de contact (f = a - g).
# Le programme detecte la chute tout seul, fige la courbe et affiche la
# duree de la chute et la hauteur 1/2 g t^2 qu'elle donne.
#
# Installation : pip install pyserial
# Lancement    : python chute_libre.py          (port trouve tout seul)
#                python chute_libre.py COM5     (Windows, port impose)
# Touches      : Espace = PAUSE : tout s'arrete (lecture, detection, minuterie),
#                         l'ecran reste tel quel ; Espace encore = reprise du direct
#                F11 = plein ecran oui/non, Echap = quitter

import math
import sys
import time
import tkinter as tk

import serial

import cartes                           # choix de la carte A ou B, port exclusif

CARTE = "A"              # carte des demos (numeros de serie dans cartes.py)
G = 9.81

FENETRE_S = 5.0          # duree affichee a l'ecran (s)
Y_MIN, Y_MAX = -5.0, 25.0   # echelle verticale (m/s2)

SEUIL_CHUTE = 2.0        # |a| sous ce seuil = chute libre (m/s2)
N_CONFIRM = 2            # mesures consecutives sous le seuil pour y croire
DUREE_MIN = 0.10         # une "chute" plus courte est un choc, on l'ignore (s)
DUREE_MAX = 3.0          # au-dela, ce n'est pas une chute : on abandonne (s)
APRES_S = 1.0            # on montre encore 1 s apres l'impact, puis on fige
FIGE_S = 10.0            # duree pendant laquelle la courbe reste figee (s)

# position des colonnes dans une trame (meme ordre que le journal SD)
COL_T, COL_ALAT1, COL_ALAT2, COL_ALONG = 0, 2, 3, 4
NB_COLONNES = 17

# ---- etat du programme -------------------------------------------------------
port_serie = None
tampon = ""
t_dernier_essai = 0.0
mesures = []             # liste de (t_carte, a_long, norme)

etat = "direct"          # "direct", "chute" ou "fige"
pause = False            # True : on n'ecoute plus la carte, l'ecran est fige
compteur = 0             # mesures consecutives sous le seuil
t_debut = 0.0            # debut de la chute (temps carte)
t_fin = 0.0              # impact (temps carte)
t_fin_affichage = None   # fin de la fenetre figee (temps carte)
fige_jusqua = 0.0        # heure du PC a laquelle on relance le direct
resultat = ""


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


def instant_croisement(t0, n0, t1, n1):
    """Instant ou la norme passe le seuil, entre deux mesures (interpolation)."""
    if n0 == n1:
        return t1
    return t0 + (n0 - SEUIL_CHUTE) / (n0 - n1) * (t1 - t0)


def chute_terminee():
    """Calcule le resultat a afficher une fois l'impact detecte."""
    global resultat
    duree = t_fin - t_debut
    dans_chute = [m for m in mesures if t_debut <= m[0] <= t_fin]
    n = len(dans_chute)
    a_moy = sum(m[1] for m in dans_chute) / n if n > 0 else 0.0
    hauteur = 0.5 * G * duree ** 2
    # resolution : l'ecart moyen entre deux mesures autour de la chute
    autour = [m[0] for m in mesures if t_debut - 0.5 <= m[0] <= t_fin + 0.5]
    pas = (autour[-1] - autour[0]) / (len(autour) - 1) if len(autour) > 1 else 0.0
    resultat = (f"Chute libre : {virgule(duree, 2)} s (à {virgule(pas, 2)} s près, "
                f"{n} mesures)     a_long pendant la chute : {virgule(a_moy, 1)} m/s²     "
                f"hauteur 1/2 g t² = {virgule(100 * hauteur, 0)} cm")


def traiter_ligne(ligne):
    """Ajoute une mesure et fait avancer la detection de chute."""
    global etat, compteur, t_debut, t_fin, t_fin_affichage, fige_jusqua
    champs = ligne.strip().split(",")
    if len(champs) != NB_COLONNES:
        return
    try:
        t = float(champs[COL_T])
        a1 = float(champs[COL_ALAT1])
        a2 = float(champs[COL_ALAT2])
        a3 = float(champs[COL_ALONG])
    except ValueError:
        return                      # ligne d'en-tete ou message de la carte
    norme = math.sqrt(a1 ** 2 + a2 ** 2 + a3 ** 2)
    precedente = mesures[-1] if mesures else None
    mesures.append((t, a3, norme))
    while mesures and mesures[0][0] < t - 20.0:
        mesures.pop(0)              # on garde 20 s, largement assez

    if etat == "direct":
        if norme < SEUIL_CHUTE:
            compteur += 1
            if compteur == 1 and precedente is not None:
                t_debut = instant_croisement(precedente[0], precedente[2], t, norme)
            elif compteur == 1:
                t_debut = t
            if compteur >= N_CONFIRM:
                etat = "chute"
        else:
            compteur = 0
    elif etat == "chute":
        if norme >= SEUIL_CHUTE:
            t_fin = instant_croisement(precedente[0], precedente[2], t, norme)
            compteur = 0
            if t_fin - t_debut >= DUREE_MIN:
                chute_terminee()
                etat = "fige"
                t_fin_affichage = t_fin + APRES_S
                fige_jusqua = time.monotonic() + FIGE_S
            else:
                etat = "direct"
        elif t - t_debut > DUREE_MAX:
            etat = "direct"
            compteur = 0


def lire_port():
    """Appelee toutes les 10 ms : vide ce que la carte a envoye."""
    global port_serie, tampon
    if port_serie is None:
        connecter()
    else:
        try:
            if pause:
                # en pause, on jette ce que la carte envoie : rien n'entre
                # dans les mesures, et le tampon USB ne deborde pas
                port_serie.reset_input_buffer()
                fenetre.after(10, lire_port)
                return
            n = port_serie.in_waiting
            if n > 0:
                tampon += port_serie.read(n).decode("utf-8", errors="ignore")
                lignes = tampon.split("\n")
                tampon = lignes.pop()       # derniere ligne peut etre coupee
                for ligne in lignes:
                    traiter_ligne(ligne)
        except (serial.SerialException, OSError):
            port_serie = None
            cartes.message = "Carte débranchée, en attente..."
            message_etat.config(text=cartes.message)
    fenetre.after(10, lire_port)


def relancer():
    global etat, compteur, resultat
    etat = "direct"
    compteur = 0
    resultat = ""


def basculer_pause(_evenement=None):
    """Espace : met en pause, ou reprend le direct si on etait en pause."""
    global pause, tampon
    if not pause:
        pause = True
    else:
        pause = False
        tampon = ""                 # pas de morceau de ligne d'avant la pause
        mesures.clear()             # la courbe repart propre, sans trou
        relancer()


# ---- dessin ------------------------------------------------------------------
def dessiner():
    """Appelee toutes les 40 ms : redessine tout le graphique."""
    if etat == "fige" and time.monotonic() > fige_jusqua and not pause:
        relancer()

    zone.delete("all")
    largeur = zone.winfo_width()
    hauteur = zone.winfo_height()
    gauche, droite = int(largeur * 0.09), largeur - 40
    haut, bas = 45, hauteur - int(hauteur * 0.12)

    if mesures:
        fin = mesures[-1][0]
        if etat == "fige" and t_fin_affichage is not None:
            fin = min(fin, t_fin_affichage)
    else:
        fin = FENETRE_S
    debut = fin - FENETRE_S

    def x(t):
        return gauche + (t - debut) / FENETRE_S * (droite - gauche)

    def y(a):
        a = max(Y_MIN, min(Y_MAX, a))
        return bas - (a - Y_MIN) / (Y_MAX - Y_MIN) * (bas - haut)

    # bande grise sur la chute detectee
    if etat == "fige":
        zone.create_rectangle(x(t_debut), haut, x(t_fin), bas, fill="#e6e6e6", outline="")

    # graduations verticales
    for a in range(int(Y_MIN), int(Y_MAX) + 1, 5):
        zone.create_line(gauche, y(a), droite, y(a), fill="#dddddd")
        zone.create_text(gauche - 12, y(a), text=str(a), anchor="e", font=petit, fill="black")
    zone.create_text(gauche - 40, haut - 14, text="accélération mesurée (m/s²)", anchor="sw", font=petit, fill="black")

    # reperes : 0 et g
    zone.create_line(gauche, y(0), droite, y(0), fill="black", width=2)
    zone.create_line(gauche, y(G), droite, y(G), fill="black", width=2, dash=(10, 8))
    zone.create_text(droite, y(G) - 6, text="g = 9,81", anchor="se", font=petit, fill="black")

    # graduations horizontales : une par seconde, comptees depuis la fin
    for k in range(int(FENETRE_S) + 1):
        xx = x(fin - k)
        zone.create_line(xx, bas, xx, bas + 10, fill="black", width=2)
        zone.create_text(xx, bas + 14, text=f"-{k} s" if k else "0 s", anchor="n", font=petit, fill="black")
    zone.create_line(gauche, bas, droite, bas, fill="black", width=2)

    # courbes
    visibles = [m for m in mesures if debut <= m[0] <= fin]
    if len(visibles) >= 2:
        points_norme = []
        points_along = []
        for t, a3, norme in visibles:
            points_norme += [x(t), y(norme)]
            points_along += [x(t), y(a3)]
        zone.create_line(*points_norme, fill="#888888", width=4)
        zone.create_line(*points_along, fill="black", width=4)
        for t, a3, norme in visibles:
            r = 4
            zone.create_oval(x(t) - r, y(a3) - r, x(t) + r, y(a3) + r, fill="black", outline="")

    # legende
    zone.create_line(droite - 330, haut + 18, droite - 280, haut + 18, fill="black", width=4)
    zone.create_text(droite - 270, haut + 18, text="a_long", anchor="w", font=petit, fill="black")
    zone.create_line(droite - 330, haut + 55, droite - 280, haut + 55, fill="#888888", width=4)
    zone.create_text(droite - 270, haut + 55, text="|a| (3 axes)", anchor="w", font=petit, fill="black")

    # valeurs en gros
    if visibles:
        _, a3, norme = visibles[-1]
        if etat == "fige":
            dans_chute = [m for m in mesures if t_debut <= m[0] <= t_fin]
            if dans_chute:
                a3 = sum(m[1] for m in dans_chute) / len(dans_chute)
                norme = sum(m[2] for m in dans_chute) / len(dans_chute)
        valeurs.config(text=f"a_long = {virgule(a3, 1)} m/s²        |a| = {virgule(norme, 1)} m/s²")

    if pause:
        if etat == "fige":
            titre.config(text="PAUSE - CHUTE LIBRE : l'accéléromètre lit 0")
            bilan.config(text=resultat + "     (Espace : reprendre le direct)")
        else:
            titre.config(text="PAUSE")
            bilan.config(text="Espace : reprendre le direct")
    elif etat == "fige":
        titre.config(text="CHUTE LIBRE : l'accéléromètre lit 0")
        bilan.config(text=resultat + "     (Espace : pause)")
    elif etat == "chute":
        titre.config(text="Chute en cours...")
        bilan.config(text="")
    else:
        titre.config(text="Ce que lit l'accéléromètre, en direct")
        bilan.config(text="Lâchez la carte au-dessus d'un coussin     (Espace : pause)")

    if port_serie is None:
        message_etat.config(text=cartes.message)
    fenetre.after(40, dessiner)


def basculer_plein_ecran(_evenement=None):
    fenetre.attributes("-fullscreen", not fenetre.attributes("-fullscreen"))


# ---- interface ---------------------------------------------------------------
fenetre = tk.Tk()
fenetre.title("LUNATIK - chute libre")
fenetre.configure(bg="white")
fenetre.attributes("-fullscreen", True)
fenetre.bind("<F11>", basculer_plein_ecran)
fenetre.bind("<Escape>", lambda e: fenetre.destroy())
fenetre.bind("<space>", basculer_pause)

h = fenetre.winfo_screenheight()
petit = ("Helvetica", h // 45)
moyen = ("Helvetica", h // 28)
grand = ("Helvetica", h // 16, "bold")

titre = tk.Label(fenetre, text="", font=moyen, bg="white", fg="black")
titre.pack(pady=(15, 0))
valeurs = tk.Label(fenetre, text="", font=grand, bg="white", fg="black")
valeurs.pack()
zone = tk.Canvas(fenetre, bg="white", highlightthickness=0)
zone.pack(fill="both", expand=True, padx=20)
bilan = tk.Label(fenetre, text="", font=petit, bg="white", fg="black")
bilan.pack(pady=5)
message_etat = tk.Label(fenetre, text="Recherche de la carte...", font=petit,
                        bg="white", fg="#666666")
message_etat.pack(pady=(0, 10))

fenetre.after(10, lire_port)
fenetre.after(40, dessiner)
fenetre.mainloop()

if port_serie is not None:
    port_serie.close()