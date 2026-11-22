# angle_cale.py - LUNATIK, demo de l'oral : angle de la carte posee sur la cale
#
# Lit les trames envoyees par la carte (code.py = demo_usb.py) sur le cable USB
# et affiche en direct le calcul de l'angle entre l'axe de la carte et la
# verticale, avec la formule (1) du memoire :
#
#     theta = arctan( racine(a_lat1^2 + a_lat2^2) / a_long )
#
# Les valeurs sont moyennees sur 1 s de mesures de la carte.
#
# Installation : pip install pyserial
# Lancement    : python angle_cale.py          (port trouve tout seul)
#                python angle_cale.py COM5     (Windows, port impose)
#                python angle_cale.py /dev/cu.usbmodem1101
# Touches      : Z = memoriser la position de reference (carte DEBOUT sur la
#                    table, avant de la poser sur la cale)
#                X = oublier la reference
#                F11 = plein ecran oui/non, Echap = quitter
#
# Pourquoi la reference (touche Z) : la formule (1) suppose que "debout" veut
# dire "parfaitement vertical". Si la table penche un peu, ou si l'axe du
# capteur n'est pas parfaitement aligne avec la carte (decalage du zero de
# l'accelerometre), l'angle lu peut etre faux de 1 a 2 degres. En memorisant
# le vecteur g mesure carte debout, puis en calculant l'angle entre ce vecteur
# et celui mesure sur la cale, ces deux erreurs s'annulent :
#     cos(angle) = (g_ref . g_cale) / (|g_ref| x |g_cale|)

import math
import sys
import time
import tkinter as tk

import serial

import cartes                           # choix de la carte A ou B, port exclusif

CARTE = "A"              # carte des demos (numeros de serie dans cartes.py)
FENETRE_S = 1.0          # duree de la moyenne, en secondes de mesures carte
G = 9.81                 # m/s2, pour la verification de la norme
ANGLE_CALE = 30.0        # valeur proposee au demarrage dans le champ

# position des colonnes dans une trame (meme ordre que le journal SD)
COL_T, COL_ALAT1, COL_ALAT2, COL_ALONG = 0, 2, 3, 4
NB_COLONNES = 17

# symboles mathematiques ecrits en code unicode : le fichier reste en
# caracteres du clavier, l'ecran affiche les vrais symboles
THETA = "\u03b8"
RACINE = "\u221a"
PLUS_MOINS = "\u00b1"

# ---- etat du programme -----------------------------------------------------
port_serie = None
tampon = ""
t_dernier_essai = 0.0
mesures = []             # liste de (t_carte, a_lat1, a_lat2, a_long)
reference = None         # (a_lat1, a_lat2, a_long) moyen, carte debout (touche Z)
nb_trames = 0
t_debut_comptage = time.monotonic()
cadence = 0.0


def virgule(x, n):
    """Nombre avec n decimales et une virgule, a la francaise."""
    return f"{x:.{n}f}".replace(".", ",")


def carre(x):
    """x au carre ecrit proprement : (-4,780)² et pas -4,780²."""
    texte = virgule(x, 3)
    if x < 0:
        texte = "(" + texte + ")"
    return texte + "²"


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
    etat.config(text=cartes.message)


def traiter_ligne(ligne):
    """Garde a_lat1, a_lat2, a_long d'une trame complete, ignore le reste."""
    global nb_trames
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
    mesures.append((t, a1, a2, a3))
    nb_trames += 1
    # on ne garde que la derniere seconde de mesures
    while mesures and mesures[0][0] < t - FENETRE_S:
        mesures.pop(0)


def lire_port():
    """Appelee toutes les 20 ms : vide ce que la carte a envoye."""
    global port_serie, tampon
    if port_serie is None:
        connecter()
    else:
        try:
            n = port_serie.in_waiting
            if n > 0:
                tampon += port_serie.read(n).decode("utf-8", errors="ignore")
                lignes = tampon.split("\n")
                tampon = lignes.pop()       # derniere ligne peut etre coupee
                for ligne in lignes:
                    traiter_ligne(ligne)
        except (serial.SerialException, OSError):
            port_serie = None               # cable debranche : on reessaie
            mesures.clear()
            cartes.message = "Carte débranchée, en attente..."
            etat.config(text=cartes.message)
    fenetre.after(20, lire_port)


def lire_angle_cale():
    try:
        return float(champ_cale.get().replace(",", "."))
    except ValueError:
        return None


def moyenne_axes():
    n = len(mesures)
    return (sum(m[1] for m in mesures) / n,
            sum(m[2] for m in mesures) / n,
            sum(m[3] for m in mesures) / n)


def memoriser_reference(_evenement=None):
    """Touche Z : la position actuelle (carte debout) sert de reference."""
    global reference
    if _evenement is not None and _evenement.widget is champ_cale:
        return                      # on tape dans le champ de l'angle, pas une commande
    if len(mesures) >= 5:
        reference = moyenne_axes()


def oublier_reference(_evenement=None):
    global reference
    if _evenement is not None and _evenement.widget is champ_cale:
        return
    reference = None


def angle_entre(u, v):
    """Angle (degres) entre deux vecteurs, par le produit scalaire."""
    scalaire = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
    nu = math.sqrt(u[0] ** 2 + u[1] ** 2 + u[2] ** 2)
    nv = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    c = max(-1.0, min(1.0, scalaire / (nu * nv)))
    return math.degrees(math.acos(c))


def afficher():
    """Appelee toutes les 200 ms : recalcule et reecrit l'ecran."""
    global nb_trames, t_debut_comptage, cadence
    maintenant = time.monotonic()
    if maintenant - t_debut_comptage >= 1.0:
        cadence = nb_trames / (maintenant - t_debut_comptage)
        nb_trames = 0
        t_debut_comptage = maintenant

    if len(mesures) < 5:
        if port_serie is None:
            etat.config(text=cartes.message)
        fenetre.after(200, afficher)
        return

    # moyenne des trois axes sur la derniere seconde
    n = len(mesures)
    a1 = sum(m[1] for m in mesures) / n
    a2 = sum(m[2] for m in mesures) / n
    a3 = sum(m[3] for m in mesures) / n

    # formule (1) du memoire
    lateral = math.sqrt(a1 ** 2 + a2 ** 2)
    angle = math.degrees(math.atan2(lateral, a3))

    # dispersion : ecart-type de l'angle calcule mesure par mesure
    angles = [math.degrees(math.atan2(math.sqrt(m[1] ** 2 + m[2] ** 2), m[3]))
              for m in mesures]
    moy = sum(angles) / n
    ecart_type = math.sqrt(sum((x - moy) ** 2 for x in angles) / (n - 1))

    # verification de l'accelerometre : au repos, la norme doit valoir g
    norme = math.sqrt(a1 ** 2 + a2 ** 2 + a3 ** 2)

    valeurs.config(text=f"a_lat1 = {virgule(a1, 3)}     a_lat2 = {virgule(a2, 3)}"
                        f"     a_long = {virgule(a3, 3)}   m/s²")
    calcul_1.config(text=f"{THETA} = arctan( {RACINE}({carre(a1)} + {carre(a2)})"
                         f" / {virgule(a3, 3)} )")
    calcul_2.config(text=f"{THETA} = arctan( {virgule(lateral, 3)} / {virgule(a3, 3)} )")
    resultat.config(text=f"{THETA} = {virgule(angle, 1)}°")
    dispersion.config(text=f"écart-type sur 1 s : {PLUS_MOINS} {virgule(ecart_type, 2)}°"
                           f"     ({n} mesures, {virgule(cadence, 0)} par seconde)")

    angle_cale = lire_angle_cale()
    if angle_cale is None:
        ecart.config(text="")
    else:
        ecart.config(text=f"écart mesure - cale : {virgule(angle - angle_cale, 1)}°")

    if reference is None:
        texte_reference.config(text="Z : mémoriser la carte debout comme référence")
    else:
        angle_ref = angle_entre(reference, (a1, a2, a3))
        texte = f"par rapport à la référence debout : {virgule(angle_ref, 1)}°"
        if angle_cale is not None:
            texte += f"   (écart avec la cale : {virgule(angle_ref - angle_cale, 1)}°)"
        texte_reference.config(text=texte + "     X : oublier")

    verif.config(text=f"vérification : |a| = {virgule(norme, 2)} m/s²  "
                      f"(g = {virgule(G, 2)}, écart {virgule(100 * (norme - G) / G, 1)} %)")
    fenetre.after(200, afficher)


def basculer_plein_ecran(_evenement=None):
    plein = not fenetre.attributes("-fullscreen")
    fenetre.attributes("-fullscreen", plein)


# ---- interface ---------------------------------------------------------------
fenetre = tk.Tk()
fenetre.title("LUNATIK - angle de la carte")
fenetre.configure(bg="white")
fenetre.attributes("-fullscreen", True)
fenetre.bind("<F11>", basculer_plein_ecran)
fenetre.bind("<Escape>", lambda e: fenetre.destroy())
fenetre.bind("<z>", memoriser_reference)
fenetre.bind("<Z>", memoriser_reference)
fenetre.bind("<x>", oublier_reference)
fenetre.bind("<X>", oublier_reference)

# tailles de police proportionnelles a l'ecran (videoprojecteur)
h = fenetre.winfo_screenheight()
petit = ("Helvetica", h // 40)
moyen = ("Helvetica", h // 26)
grand = ("Helvetica", h // 7, "bold")


def ligne_texte(police, marge=10):
    lab = tk.Label(fenetre, text="", font=police, bg="white", fg="black")
    lab.pack(pady=marge)
    return lab


titre = ligne_texte(moyen, 20)
titre.config(text="Angle entre l'axe de la carte et la verticale")
valeurs = ligne_texte(petit)
calcul_1 = ligne_texte(moyen)
calcul_2 = ligne_texte(moyen)
resultat = ligne_texte(grand, 0)
dispersion = ligne_texte(petit)

zone_cale = tk.Frame(fenetre, bg="white")
zone_cale.pack(pady=15)
tk.Label(zone_cale, text="angle gravé sur la cale :", font=moyen,
         bg="white", fg="black").pack(side="left")
champ_cale = tk.Entry(zone_cale, font=moyen, width=5, justify="center",
                      bg="white", fg="black", relief="solid", bd=2)
champ_cale.insert(0, virgule(ANGLE_CALE, 0))
champ_cale.pack(side="left", padx=10)
tk.Label(zone_cale, text="°", font=moyen, bg="white", fg="black").pack(side="left")

ecart = ligne_texte(moyen)
texte_reference = ligne_texte(petit)
verif = ligne_texte(petit)
etat = ligne_texte(petit, 20)
etat.config(text="Recherche de la carte...")

fenetre.after(20, lire_port)
fenetre.after(200, afficher)
fenetre.mainloop()

if port_serie is not None:
    port_serie.close()