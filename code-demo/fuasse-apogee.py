# fausses_apogees.py - LUNATIK, fil rouge de l'oral
#
# La carte (code.py = demo_usb.py) reste posee, IMMOBILE, pendant tout l'oral.
# Le programme fait tourner en direct, sur l'altitude brute du barometre, le
# critere "N decroissances" de la partie 3.3 du memoire, pour N = 3, 5 et 10.
#
# Une fausse apogee = N baisses d'altitude de suite alors que la carte ne bouge
# pas : en vol, la carte aurait ouvert le parachute a ce moment-la. Comme une
# carte qu'on remettrait sur la rampe apres une ejection, le detecteur repart
# ensuite de zero : il faut N nouvelles baisses pour la fausse apogee suivante.
#
# Au lancement, on choisit : compteurs visibles en direct, ou caches jusqu'a
# la revelation (touche V).
#
# Installation : pip install pyserial
# Lancement    : python fausses_apogees.py          (port trouve tout seul)
#                python fausses_apogees.py COM5     (Windows, port impose)
# Touches      : V = reveler / cacher les compteurs
#                Espace = PAUSE : tout s'arrete, l'ecran reste tel quel ;
#                         Espace encore = reprise
#                F11 = plein ecran oui/non, Echap = quitter

import sys
import time
import tkinter as tk

import serial

import cartes                           # choix de la carte A ou B, port exclusif

CARTE = "B"              # carte du fil rouge (numeros de serie dans cartes.py)

LISTE_N = [3, 5, 10]     # nombres de decroissances testes
TROU_MAX_S = 1.0         # au-dela, deux mesures ne se suivent pas vraiment

# position des colonnes dans une trame (meme ordre que le journal SD)
COL_T, COL_ALT_BARO = 0, 10
NB_COLONNES = 17

# ---- etat du programme -------------------------------------------------------
port_serie = None
tampon = ""
t_dernier_essai = 0.0
pause = False
caches = True            # choisi sur l'ecran de depart
demarre = False          # True une fois le choix fait

derniere = None          # (t_carte, altitude) de la mesure precedente
serie = {n: 0 for n in LISTE_N}          # baisses consecutives en cours
fausses = {n: 0 for n in LISTE_N}        # fausses apogees comptees
nb_mesures = 0
duree_s = 0.0            # temps de mesure cumule (sans les pauses ni les trous)


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


def repartir_a_zero_les_series():
    """Apres une pause ou un trou : on ne compare pas a travers le trou."""
    global derniere
    derniere = None
    for n in LISTE_N:
        serie[n] = 0


def traiter_ligne(ligne):
    """Une mesure de plus : fait avancer les trois detecteurs."""
    global derniere, nb_mesures, duree_s
    champs = ligne.strip().split(",")
    if len(champs) != NB_COLONNES:
        return
    try:
        t = float(champs[COL_T])
        alt = float(champs[COL_ALT_BARO])
    except ValueError:
        return                      # ligne d'en-tete ou message de la carte

    nb_mesures += 1
    if derniere is not None:
        dt = t - derniere[0]
        if dt <= 0 or dt > TROU_MAX_S:
            repartir_a_zero_les_series()    # carte redemarree ou trou
        else:
            duree_s += dt
            baisse = alt < derniere[1]
            for n in LISTE_N:
                if baisse:
                    serie[n] += 1
                    if serie[n] >= n:
                        fausses[n] += 1     # en vol : parachute ejecte ici
                        serie[n] = 0        # carte "remise sur la rampe"
                else:
                    serie[n] = 0
    derniere = (t, alt)


def lire_port():
    """Appelee toutes les 10 ms : vide ce que la carte a envoye."""
    global port_serie, tampon
    if port_serie is None:
        connecter()
    else:
        try:
            if pause or not demarre:
                # en pause, on jette ce que la carte envoie : rien n'est compte
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
            repartir_a_zero_les_series()
            cartes.message = "Carte débranchée, en attente..."
            message_etat.config(text=cartes.message)
    fenetre.after(10, lire_port)


def chrono_texte():
    minutes, secondes = int(duree_s // 60), int(duree_s % 60)
    return f"{minutes} min {secondes:02d} s"


# ---- affichage -----------------------------------------------------------------
def afficher():
    """Appelee toutes les 200 ms : met a jour l'ecran."""
    if demarre and not pause:
        if caches:
            titre.config(text="Mesure en cours, carte immobile")
            for n in LISTE_N:
                compteurs[n].config(text="?")
        else:
            titre.config(text="Fausses apogées, carte immobile")
            for n in LISTE_N:
                compteurs[n].config(text=str(fausses[n]))
        chrono.config(text=f"carte immobile depuis {chrono_texte()}")
        bilan.config(text=f"{nb_mesures} mesures du baromètre     "
                          f"(V : révéler / cacher, Espace : pause)")
        if port_serie is None:
            message_etat.config(text=cartes.message)
    fenetre.after(200, afficher)


def touche_v(_evenement=None):
    global caches
    if demarre:
        caches = not caches


def touche_espace(_evenement=None):
    global pause, tampon
    if not demarre:
        return
    if not pause:
        pause = True
        titre.config(text="PAUSE     (Espace : reprendre)")
    else:
        pause = False
        tampon = ""
        repartir_a_zero_les_series()


def demarrer(avec_caches):
    """Ecran de depart : on choisit si les compteurs sont caches."""
    global caches, demarre
    caches = avec_caches
    demarre = True
    if port_serie is not None:
        port_serie.reset_input_buffer()
    choix.pack_forget()
    ecran.pack(fill="both", expand=True)


def basculer_plein_ecran(_evenement=None):
    fenetre.attributes("-fullscreen", not fenetre.attributes("-fullscreen"))


# ---- interface ---------------------------------------------------------------
fenetre = tk.Tk()
fenetre.title("LUNATIK - fausses apogées")
fenetre.configure(bg="white")
fenetre.attributes("-fullscreen", True)
fenetre.bind("<F11>", basculer_plein_ecran)
fenetre.bind("<Escape>", lambda e: fenetre.destroy())
fenetre.bind("<space>", touche_espace)
fenetre.bind("<v>", touche_v)
fenetre.bind("<V>", touche_v)
fenetre.bind("<Key-1>", lambda e: None if demarre else demarrer(False))
fenetre.bind("<Key-2>", lambda e: None if demarre else demarrer(True))

h = fenetre.winfo_screenheight()
petit = ("Helvetica", h // 45)
moyen = ("Helvetica", h // 26)
enorme = ("Helvetica", h // 6, "bold")

# ecran de depart
choix = tk.Frame(fenetre, bg="white")
choix.pack(fill="both", expand=True)
tk.Label(choix, text="Compteurs de fausses apogées", font=moyen,
         bg="white", fg="black").pack(pady=(h // 6, 40))
for texte, valeur in (("1 : compteurs visibles en direct", False),
                      ("2 : compteurs cachés jusqu'à la révélation (touche V)", True)):
    tk.Button(choix, text=texte, font=moyen, bg="white", fg="black", relief="solid",
              bd=2, padx=30, pady=15,
              command=lambda v=valeur: demarrer(v)).pack(pady=15)
tk.Label(choix, text="Posez la carte avant de choisir, puis ne la touchez plus",
         font=petit, bg="white", fg="#666666").pack(pady=40)

# ecran de mesure
ecran = tk.Frame(fenetre, bg="white")
titre = tk.Label(ecran, text="", font=moyen, bg="white", fg="black")
titre.pack(pady=(30, 10))
chrono = tk.Label(ecran, text="", font=moyen, bg="white", fg="black")
chrono.pack()

bande = tk.Frame(ecran, bg="white")
bande.pack(expand=True)
compteurs = {}
for colonne, n in enumerate(LISTE_N):
    bloc = tk.Frame(bande, bg="white", highlightbackground="black", highlightthickness=3)
    bloc.grid(row=0, column=colonne, padx=h // 30, ipadx=h // 30, ipady=10)
    tk.Label(bloc, text=f"{n} baisses de suite", font=moyen, bg="white",
             fg="black").pack(pady=(10, 0))
    compteurs[n] = tk.Label(bloc, text="?", font=enorme, bg="white", fg="black")
    compteurs[n].pack()
    tk.Label(bloc, text="fausses apogées", font=petit, bg="white", fg="black").pack(pady=(0, 10))

bilan = tk.Label(ecran, text="", font=petit, bg="white", fg="black")
bilan.pack(pady=5)
message_etat = tk.Label(fenetre, text="Recherche de la carte...", font=petit,
                        bg="white", fg="#666666")
message_etat.pack(side="bottom", pady=(0, 10))

fenetre.after(10, lire_port)
fenetre.after(200, afficher)
fenetre.mainloop()

if port_serie is not None:
    port_serie.close()