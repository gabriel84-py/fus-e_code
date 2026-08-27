import shutil
rxtx = input('choisit si c est le RX ou le TX : ')
if rxtx == 'rx':
    SOURCE = "/Users/gabrieljeanvermeille/PycharmProjects/couillonne-de-la-lune/src/code-recepteur.py"
if rxtx == 'tx':
    SOURCE = "/Users/gabrieljeanvermeille/PycharmProjects/couillonne-de-la-lune/src/code-transiver.py"
if rxtx == 'bench':
    SOURCE = "/Users/gabrieljeanvermeille/PycharmProjects/couillonne-de-la-lune/tests/benchbaro.py"
DEST = "/Volumes/CIRCUITPY/code.py" 
shutil.copy(SOURCE, DEST)
print("Copié vers la carte.")