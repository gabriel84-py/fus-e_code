#!/usr/bin/env python3
"""Test minimal : si cette fenetre rouge ne s'affiche pas, le probleme
est dans l'installation Tkinter/Python, pas dans dashboard.py."""

import sys
import tkinter as tk

print("Executable Python utilise :", sys.executable)
print("Ouverture de la fenetre de test...")

root = tk.Tk()
root.title("Test Tkinter")
root.geometry("300x200+200+200")
root.configure(bg="red")

label = tk.Label(root, text="Si tu vois ce texte,\nTkinter fonctionne.",
                  bg="red", fg="white", font=("Consolas", 14))
label.pack(expand=True)

# Force la fenetre au premier plan (parfois necessaire sur macOS)
root.lift()
root.attributes("-topmost", True)
root.after(100, lambda: root.attributes("-topmost", False))

root.mainloop()
print("Fenetre fermee.")