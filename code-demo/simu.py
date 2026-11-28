"""
LUNATIK - 20 000 vols simules autour de l'apogee (methode de Monte-Carlo)

Question : la regle de detection embarquee (seuil de 0,51 m sous le maximum,
2 confirmations) declenche-t-elle parfois AVANT l'apogee ?

Modele :
  - trajectoire : la parabole ajustee sur notre vol du 27 septembre,
    courbure 2a = 9,08 m/s^2 (et non 9,81), apogee placee a t = 0 ;
  - cadence : 38 mesures par seconde (une toutes les 26 ms), comme en vol ;
  - bruit : blanc et gaussien, independant d'une mesure a l'autre ;
      * 20 cm  = bruit du barometre brut mesure sur la rampe le jour du vol ;
      * 8,7 cm = incertitude annoncee par le filtre de Kalman ;
  - la detection est armee 1,5 s avant l'apogee.

Limites (a dire au jury) :
  - le bruit du Kalman n'est pas blanc (deux sorties successives se
    ressemblent) : la ligne "Kalman" est donc un ordre de grandeur ;
  - la grille de temps est la meme a chaque tirage : la mesure la plus proche
    du sommet tombe toujours 8 ms apres lui (voir PHASE_ALEATOIRE).

Resultat attendu (graine 1) :
  barometre brut : 8,2 % trop tot, retard de -0,10 a +0,37 s
  Kalman         : 0 % trop tot,   retard de +0,29 a +0,40 s
"""

import numpy as np

GRAINE = 1          # graine fixee et notee : le calcul est reproductible
N_VOLS = 20_000
COURBURE = 9.08     # 2a de la parabole ajustee sur le vol (m/s^2)
DT = 0.026          # 38 Hz
SEUIL = 0.51        # 3 x 17 cm (bruit au banc)
CONFIRMATIONS = 2
PHASE_ALEATOIRE = False   # True : la grille est decalee au hasard a chaque vol

rng = np.random.default_rng(GRAINE)
t_base = np.arange(-1.5, 1.5, DT)   # temps relatif a l'apogee (s)


def altitude_vraie(t):
    """Parabole de notre apogee : h = -1/2 x 2a x t^2, sommet en t = 0."""
    return -0.5 * COURBURE * t**2


def detection(t, h_mesuree):
    """Meme regle que apogee.py a bord : maximum courant, apogee declaree
    quand la mesure passe SEUIL sous le maximum CONFIRMATIONS fois de suite.
    Renvoie l'instant de detection, ou None si rien n'est detecte."""
    maxi = -1e9
    compteur = 0
    for ti, hi in zip(t, h_mesuree):
        maxi = max(maxi, hi)
        compteur = compteur + 1 if hi < maxi - SEUIL else 0
        if compteur >= CONFIRMATIONS:
            return ti
    return None


def simuler(sigma, n_vols=N_VOLS):
    """Fait voler n_vols fusees avec un bruit d'ecart-type sigma (m)."""
    trop_tot = 0
    retards = []
    for _ in range(n_vols):
        t = t_base
        if PHASE_ALEATOIRE:
            t = t_base + rng.uniform(0, DT)
        h = altitude_vraie(t) + rng.normal(0, sigma, t.size)
        td = detection(t, h)
        if td is None:
            continue
        retards.append(td)
        if td < 0:              # declenche avant le sommet : parachute trop tot
            trop_tot += 1
    retards = np.array(retards)
    return {
        "trop_tot": trop_tot / n_vols,
        "p_trop_tot_incert": np.sqrt(trop_tot / n_vols * (1 - trop_tot / n_vols) / n_vols),
        "retard_median": np.median(retards),
        "retard_p5": np.percentile(retards, 5),
        "retard_p95": np.percentile(retards, 95),
    }


if __name__ == "__main__":
    for nom, sigma in (("barometre brut", 0.20), ("Kalman", 0.087)):
        r = simuler(sigma)
        print(f"{nom:15s} (sigma = {sigma*100:4.1f} cm) : "
              f"{100*r['trop_tot']:4.1f} +/- {100*r['p_trop_tot_incert']:.1f} % trop tot | "
              f"retard median {r['retard_median']:+.2f} s | "
              f"90 % des vols entre {r['retard_p5']:+.2f} et {r['retard_p95']:+.2f} s")