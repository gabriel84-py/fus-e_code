import shutil
rxtx = input('choisit si c est le RX ou le TX : ')
if rxtx == 'rx':
    SOURCE = "/Users/gabrieljeanvermeille/PycharmProjects/couillonne-de-la-lune/code-recepteur.py"
if rxtx == 'tx':
    SOURCE = "/Users/gabrieljeanvermeille/PycharmProjects/couillonne-de-la-lune/code-transiver.py"
DEST = "/Volumes/CIRCUITPY/code.py" # adapte le chemin selon ton OS
shutil.copy(SOURCE, DEST)
print("Copié vers la carte.")