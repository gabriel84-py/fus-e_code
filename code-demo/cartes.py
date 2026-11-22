# cartes.py - LUNATIK : trouver la BONNE carte sur le bon port USB
#
# Pourquoi ce fichier : le jour de l'oral, deux cartes sont branchees sur le
# meme ordinateur (A pour les demos, B pour le fil rouge). Avant, chaque
# programme prenait "la premiere carte Pico" dans l'ordre alphabetique des
# ports : selon la prise USB utilisee, la demo pouvait lire la carte du fil
# rouge, et inversement. Et sur Mac, deux programmes pouvaient ouvrir le meme
# port et se partager les lignes au hasard.
#
# Ici :
#   - chaque carte est reconnue par son NUMERO DE SERIE USB (unique pour
#     chaque puce, il ne change pas selon la prise utilisee) ;
#   - le port est ouvert en mode EXCLUSIF : un deuxieme programme qui essaie
#     d'ouvrir la meme carte recoit une erreur claire au lieu de voler des
#     lignes en silence.
#
# A FAIRE UNE FOIS : brancher les cartes, lancer   python cartes.py
# et recopier les numeros affiches dans SERIE_CARTE_A et SERIE_CARTE_B.
#
# Utilisation dans un programme (exemple simple) :
#     import cartes
#     nom = cartes.trouver_port("A")      # None si la carte n'est pas la
#     if nom is None:
#         print(cartes.message)           # explique pourquoi
#     port = cartes.ouvrir(nom)           # None si deja ouvert ailleurs

import sys

import serial
from serial.tools import list_ports

# ---- A REMPLIR : numeros affiches par "python cartes.py" --------------------
SERIE_CARTE_A = ""    # carte des demos (demo 1, demo 3, bruit, angle)
SERIE_CARTE_B = ""    # carte du fil rouge (fausses apogees, derive)
# ------------------------------------------------------------------------------

VID_PICO = 0x2E8A     # identifiant USB des cartes Raspberry Pi (Pico 2)
BAUDS = 115200        # ignore par la console USB du Pico, mais obligatoire

# dernier message d'explication, a afficher a l'ecran si un port manque
message = "Recherche de la carte..."


def lister():
    """Toutes les cartes Pico branchees, triees par nom de port."""
    ports = [p for p in list_ports.comports() if p.vid == VID_PICO]
    ports.sort(key=lambda p: p.device)
    return ports


def trouver_port(carte):
    """Nom du port de la carte "A" ou "B", ou None (voir cartes.message).

    Un port donne en argument (python chute-libre.py COM5) passe avant tout.
    """
    global message
    if len(sys.argv) > 1:
        message = ""
        return sys.argv[1]

    serie = SERIE_CARTE_A if carte == "A" else SERIE_CARTE_B
    ports = lister()

    if serie:
        for p in ports:
            if p.serial_number == serie:
                message = ""
                return p.device
        message = "Carte " + carte + " non trouvée : branche son câble USB"
        return None

    # numeros pas encore renseignes : on n'accepte qu'une seule carte
    # branchee, sinon on risque de lire la mauvaise
    if len(ports) == 1:
        message = ""
        return ports[0].device
    if not ports:
        message = "Carte non trouvée : branche le câble USB"
    else:
        message = ("Plusieurs cartes branchées : renseigne SERIE_CARTE_A et "
                   "SERIE_CARTE_B dans cartes.py (lance : python cartes.py)")
    return None


def ouvrir(nom):
    """Ouvre le port en exclusif, ou None (voir cartes.message)."""
    global message
    try:
        port = serial.Serial(nom, BAUDS, timeout=0, exclusive=True)
        port.reset_input_buffer()
        message = "Carte connectée sur " + nom
        return port
    except (serial.SerialException, OSError, ValueError):
        message = ("Port " + nom + " impossible à ouvrir : déjà utilisé par "
                   "un autre programme ?")
        return None


def envoyer(port, commande):
    """Envoie une commande texte a la carte (voir l'en-tete de code.py)."""
    try:
        port.write((commande + "\n").encode("ascii"))
        return True
    except (serial.SerialException, OSError):
        return False


if __name__ == "__main__":
    ports = lister()
    if not ports:
        print("Aucune carte Pico branchée.")
    for p in ports:
        role = ""
        if SERIE_CARTE_A and p.serial_number == SERIE_CARTE_A:
            role = "   <- carte A (démos)"
        elif SERIE_CARTE_B and p.serial_number == SERIE_CARTE_B:
            role = "   <- carte B (fil rouge)"
        print(p.device, "  numéro de série :", p.serial_number, role)
    print()
    print("Branche les cartes UNE PAR UNE pour savoir laquelle est laquelle,")
    print("puis recopie les numéros dans SERIE_CARTE_A et SERIE_CARTE_B.")
