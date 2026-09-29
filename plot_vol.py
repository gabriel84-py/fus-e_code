#!/usr/bin/env python3
"""
plot_vol.py - courbes de vol LUNATIK a partir d'un log carte SD (data_XXX.csv)

Produit une figure a 3 graphiques (altitude, vitesse, acceleration) avec une
barre verticale a chaque changement d'etat de la machine d'etat, et l'exporte
en PNG sous le meme nom que le CSV (data_003.csv -> data_003.png).

Format attendu : celui ecrit par src/datalog.py::_formatter()
    timestamp_s, phase, a_lat1, a_lat2, a_long, gx, gy, gz, pression_hpa,
    temp_c, alt_baro_m, lat, lon, alt_gps_m, z_kalman_m, vz_kalman_ms, batt_v

Prerequis :
    pip install pandas matplotlib numpy

Usage :
    
    python plot_vol.py /media/sd/data_*.csv --out figures/
    python plot_vol.py data_003.csv --tout           (garde toute l'attente au sol)
    python plot_vol.py data_003.csv --cinematique    (accel = a_long - g)
"""

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # pas d'ecran necessaire : export PNG uniquement
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

G0 = 9.80665

# memes seuils que src/state_machine.py : les tracer sur la courbe d'accel
# permet de verifier visuellement pourquoi une transition a eu lieu (ou pas)
SEUIL_BOOST = 19.6
SEUIL_BURNOUT = 0.0

# memes couleurs que scripts/dashboard.py, pour rester coherent
STATE_COLORS = {
    "SETUP": "#455a64",
    "PRE_LAUNCH": "#607d8b",
    "BOOST": "#e53935",
    "COAST": "#fb8c00",
    "APOGEE": "#8e24aa",
    "APOGEEtimer": "#8e24aa",
    "DESCENT": "#1e88e5",
    "LANDED": "#43a047",
}
COULEUR_INCONNUE = "#9e9e9e"

COLONNES_REQUISES = ["timestamp_s", "phase", "a_long", "alt_baro_m",
                     "z_kalman_m", "vz_kalman_ms"]


def charger_csv(chemin):
    """Lit le log et le nettoie.

    Pieges geres ici :
    - l'en-tete et les lignes utilisent ", " comme separateur -> espaces a
      retirer, sinon la colonne s'appelle " phase" et la valeur " BOOST" ;
    - la derniere ligne peut etre tronquee (coupure batterie pendant un
      flush) -> champs manquants = NaN, ligne supprimee ;
    - une valeur non numerique (nan, inf, octets corrompus) ne doit pas faire
      planter tout le chargement -> to_numeric(errors="coerce").
    """
    df = pd.read_csv(chemin, skipinitialspace=True, engine="python",
                     on_bad_lines="skip")
    df.columns = df.columns.str.strip()

    manquantes = [c for c in COLONNES_REQUISES if c not in df.columns]
    if manquantes:
        raise ValueError(f"colonnes absentes : {', '.join(manquantes)}")

    df["phase"] = df["phase"].astype(str).str.strip()
    for col in df.columns:
        if col != "phase":
            df[col] = pd.to_numeric(df[col], errors="coerce")

    n_avant = len(df)
    df = df.dropna(subset=COLONNES_REQUISES).reset_index(drop=True)
    if len(df) < n_avant:
        print(f"  {n_avant - len(df)} ligne(s) invalide(s) ignoree(s)")

    if len(df) < 2:
        raise ValueError("pas assez de lignes exploitables")

    if (np.diff(df["timestamp_s"].to_numpy()) < 0).any():
        print("  attention : timestamps non monotones (reboot en cours de log ?)")

    inconnues = set(df["phase"]) - set(STATE_COLORS)
    if inconnues:
        print(f"  attention : phase(s) inconnue(s) : {', '.join(sorted(inconnues))}")

    return df


def trouver_transitions(t, phases):
    """Liste des (temps, ancien_etat, nouvel_etat) a chaque changement."""
    idx = np.flatnonzero(phases[1:] != phases[:-1]) + 1
    return [(t[i], phases[i - 1], phases[i]) for i in idx]


def trouver_decollage(transitions):
    """Instant du VRAI decollage.

    Un faux depart fait BOOST -> PRE_LAUNCH : le premier BOOST n'est donc pas
    forcement le bon. On prend le dernier passage en BOOST qui precede le
    premier COAST.
    """
    t_boost = None
    for t, _, nouveau in transitions:
        if nouveau == "BOOST":
            t_boost = t
        elif nouveau == "COAST" and t_boost is not None:
            return t_boost
    return t_boost  # vol incomplet : dernier BOOST vu, ou None


def segments_de_phase(t, phases):
    """Decoupe en blocs contigus (t_debut, t_fin, phase) pour les fonds colores."""
    idx = np.concatenate(([0], np.flatnonzero(phases[1:] != phases[:-1]) + 1,
                          [len(phases)]))
    return [(t[a], t[b - 1] if b - 1 > a else t[a], phases[a])
            for a, b in zip(idx[:-1], idx[1:])]


def grouper_labels(transitions, ecart_min):
    """Regroupe les transitions trop proches pour que les textes ne se
    chevauchent pas. Cas typique : APOGEE ne dure qu'un cycle de boucle
    (~29 ms), donc COAST->APOGEE et APOGEE->DESCENT tombent au meme pixel."""
    groupes = []
    for t, _, nouveau in transitions:
        if groupes and t - groupes[-1][0] < ecart_min:
            groupes[-1][1].append(nouveau)
        else:
            groupes.append([t, [nouveau]])
    return [(t, " / ".join(noms)) for t, noms in groupes]


def tracer(df, chemin_png, t0_decollage=True, tout=False, marge=5.0,
           cinematique=False, dpi=150):
    t_brut = df["timestamp_s"].to_numpy()
    phases = df["phase"].to_numpy()

    transitions = trouver_transitions(t_brut, phases)
    t_dec = trouver_decollage(transitions)

    # origine des temps : le decollage si on l'a trouve, sinon debut du log
    t0 = t_dec if (t0_decollage and t_dec is not None) else t_brut[0]
    t = t_brut - t0
    transitions = [(tt - t0, a, b) for tt, a, b in transitions]

    # recadrage sur le vol : l'attente au pas de tir peut durer 30 min et
    # ecraser les 60 s de vol dans un coin du graphique
    masque = np.ones(len(t), dtype=bool)
    if not tout and t_dec is not None:
        t_fin_vol = t[-1]
        for tt, _, nouveau in transitions:
            if nouveau == "LANDED" and tt > 0:
                t_fin_vol = tt
                break
        masque = (t >= -marge) & (t <= t_fin_vol + marge)

    t = t[masque]
    phases_m = phases[masque]
    d = df[masque]
    if len(t) < 2:
        raise ValueError("fenetre de vol vide apres recadrage (essayer --tout)")
    transitions = [tr for tr in transitions if t[0] <= tr[0] <= t[-1]]

    z_kal = d["z_kalman_m"].to_numpy()
    z_baro = d["alt_baro_m"].to_numpy()
    vz = d["vz_kalman_ms"].to_numpy()
    acc = d["a_long"].to_numpy()
    if cinematique:
        # force specifique -> acceleration verticale. Valable seulement tant
        # que l'axe longitudinal est vertical (faux apres basculement a
        # l'apogee) et sans compter le biais residuel du capteur.
        acc = acc - G0

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True,
                             constrained_layout=True)
    ax_alt, ax_vit, ax_acc = axes

    # --- altitude ---------------------------------------------------------
    ax_alt.plot(t, z_baro, color="#90a4ae", lw=0.8, label="barometre brut")
    ax_alt.plot(t, z_kal, color="black", lw=1.4, label="Kalman")
    i_max = int(np.argmax(z_kal))
    ax_alt.plot(t[i_max], z_kal[i_max], "v", color="#8e24aa", ms=8)
    ax_alt.annotate(f"max {z_kal[i_max]:.1f} m @ {t[i_max]:.2f} s",
                    (t[i_max], z_kal[i_max]), textcoords="offset points",
                    xytext=(8, -4), fontsize=9)
    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.legend(loc="upper right", fontsize=9)

    # --- vitesse ----------------------------------------------------------
    ax_vit.plot(t, vz, color="#1e88e5", lw=1.4, label="vz Kalman")
    ax_vit.axhline(0.0, color="grey", lw=0.8, ls=":")
    i_vmax = int(np.argmax(vz))
    ax_vit.annotate(f"max {vz[i_vmax]:.1f} m/s", (t[i_vmax], vz[i_vmax]),
                    textcoords="offset points", xytext=(8, -4), fontsize=9)
    ax_vit.set_ylabel("Vitesse verticale (m/s)")
    ax_vit.legend(loc="upper right", fontsize=9)

    # --- acceleration -----------------------------------------------------
    if cinematique:
        ax_acc.plot(t, acc, color="#e53935", lw=1.0, label="a_long - g")
        ax_acc.axhline(0.0, color="grey", lw=0.8, ls=":")
        ax_acc.set_ylabel("Accel. verticale approx (m/s^2)")
    else:
        ax_acc.plot(t, acc, color="#e53935", lw=1.0,
                    label="a_long (force specifique)")
        ax_acc.axhline(G0, color="grey", lw=0.8, ls=":", label="1 g (repos)")
        ax_acc.axhline(SEUIL_BOOST, color="#e53935", lw=0.8, ls="--",
                       alpha=0.6, label=f"seuil BOOST {SEUIL_BOOST}")
        ax_acc.axhline(SEUIL_BURNOUT, color="#fb8c00", lw=0.8, ls="--",
                       alpha=0.6, label=f"seuil burnout {SEUIL_BURNOUT}")
        ax_acc.set_ylabel("Acceleration (m/s^2)")
    ax_acc.legend(loc="upper right", fontsize=9)

    # --- phases : fonds colores + barres verticales -------------------------
    for t_a, t_b, ph in segments_de_phase(t, phases_m):
        for ax in axes:
            ax.axvspan(t_a, t_b, color=STATE_COLORS.get(ph, COULEUR_INCONNUE),
                       alpha=0.07, lw=0)

    for tt, _, nouveau in transitions:
        couleur = STATE_COLORS.get(nouveau, COULEUR_INCONNUE)
        for ax in axes:
            ax.axvline(tt, color=couleur, lw=1.2, ls="--")

    # textes uniquement sur le graphique du haut, en coordonnees "axes" en y
    # pour qu'ils restent en haut quelle que soit l'echelle d'altitude
    ecart_min = 0.015 * (t[-1] - t[0])
    for tt, texte in grouper_labels(transitions, ecart_min):
        ax_alt.text(tt, 1.01, texte, transform=ax_alt.get_xaxis_transform(),
                    rotation=45, ha="left", va="bottom", fontsize=8)

    for ax in axes:
        ax.grid(True, alpha=0.3)
    ax_acc.set_xlabel("Temps depuis le decollage (s)"
                      if (t0_decollage and t_dec is not None)
                      else "Temps depuis le debut du log (s)")
    ax_acc.set_xlim(t[0], t[-1])

    fig.suptitle(chemin_png.stem, fontsize=13, fontweight="bold")
    fig.savefig(chemin_png, dpi=dpi)
    plt.close(fig)  # indispensable en boucle sur plusieurs fichiers

    if t_dec is None:
        print("  aucun decollage detecte : axe des temps = debut du log")


def main():
    p = argparse.ArgumentParser(description="Courbes de vol LUNATIK -> PNG")
    p.add_argument("csv", nargs="+", type=Path, help="fichier(s) data_XXX.csv")
    p.add_argument("--out", type=Path, default=None,
                   help="dossier de sortie (defaut : a cote du CSV)")
    p.add_argument("--tout", action="store_true",
                   help="ne pas recadrer sur le vol, tracer tout le log")
    p.add_argument("--marge", type=float, default=5.0,
                   help="secondes gardees avant decollage / apres LANDED")
    p.add_argument("--t0-debut", action="store_true",
                   help="origine des temps = debut du log et non decollage")
    p.add_argument("--cinematique", action="store_true",
                   help="tracer a_long - g au lieu de la force specifique")
    p.add_argument("--dpi", type=int, default=150)
    args = p.parse_args()

    if args.out is not None:
        args.out.mkdir(parents=True, exist_ok=True)

    n_erreurs = 0
    for chemin in args.csv:
        print(chemin)
        dossier = args.out if args.out is not None else chemin.parent
        chemin_png = dossier / (chemin.stem + ".png")
        try:
            df = charger_csv(chemin)
            tracer(df, chemin_png, t0_decollage=not args.t0_debut,
                   tout=args.tout, marge=args.marge,
                   cinematique=args.cinematique, dpi=args.dpi)
            print(f"  -> {chemin_png}")
        except (OSError, ValueError, pd.errors.ParserError) as exc:
            print(f"  ECHEC : {exc}", file=sys.stderr)
            n_erreurs += 1

    sys.exit(1 if n_erreurs else 0)


if __name__ == "__main__":
    main()
