# LUNATIK

**Calculateur de vol pour fusée expérimentale : quelle méthode détecte réellement l'apogée le plus tôt, et avec quelle incertitude ?**

Projet réalisé dans le cadre des Olympiades de Physique / Concours C.Génial.
Firmware CircuitPython embarqué sur RP2350, chaîne d'analyse Python au sol, banc de
test logiciel rejouant des trajectoires OpenRocket bruitées.

---

## Sommaire

- [La question scientifique](#la-question-scientifique)
- [Architecture matérielle](#architecture-materielle)
- [Architecture logicielle](#architecture-logicielle)
- [Le filtre de Kalman](#le-filtre-de-kalman)
- [La détection d'apogée](#la-detection-dapogee)
- [Performances mesurées](#performances-mesurees)
- [Banc de test logiciel](#banc-de-test-logiciel)
- [Station sol](#station-sol)
- [Triangulation optique](#triangulation-optique)
- [Arborescence du dépôt](#arborescence-du-depot)
- [Mise en route](#mise-en-route)
- [Procédure de vol](#procedure-de-vol)
- [Anomalies documentées](#anomalies-documentees)
- [Limites connues](#limites-connues)

---

## La question scientifique

Une thèse résume tout le projet :

> **L'accéléromètre est une bonne horloge mais un mauvais mètre ;
> le baromètre est un bon mètre mais une mauvaise horloge.**

Ce n'est pas une formule : les deux défauts ont une forme analytique et sont
mesurables séparément.

- **Erreur inertielle.** Un biais résiduel `b` sur l'accéléromètre est intégré
  deux fois : l'erreur d'altitude croît en `1/2 b t^2`. Excellent en dynamique
  rapide, catastrophique sur la durée.
- **Retard barométrique.** Détecter l'apogée par un seuil `dh` sous le maximum
  impose structurellement un retard `t = sqrt(2 dh / g)`, indépendant du capteur.
  Excellent en position absolue, structurellement en retard.

La fusion des deux par un filtre de Kalman à trois états (altitude, vitesse,
biais accéléromètre) est censée prendre le meilleur des deux. **Le projet consiste
à vérifier ce "censé" par la mesure**, au banc puis en vol, et à quantifier
honnêtement ce qui ne marche pas.

---

## Architecture matérielle

| Fonction | Composant | Notes |
|---|---|---|
| Calculateur | RP2350 (CircuitPython) | pas de RTC sauvegardée, voir nommage des logs |
| Baromètre | BMP388 (I2C) | OSx8 pression / OSx1 température / IIR 0 |
| Centrale inertielle | LSM6DSO32 (I2C) | +/-16 g, ODR 208 Hz |
| Télémesure | RFM9x LoRa 868 MHz (SPI) | bande 868.0-868.6 MHz, limite 1 % de cycle de service |
| Stockage | Carte SD (SPI) | alimentée en 5 V par boost TPS61023 |
| Position | GPS (UART) | coupé entre le décollage et l'apogée |
| Énergie | LiPo + boost synchrone | fréquence fixe, pour ne pas polluer le plancher de bruit du BMP388 |

PCB conçu sous EasyEDA, fabriqué chez JLCPCB. Les Gerbers ont été audités par
script (`pygerber` + PIL + numpy) avant commande, ce qui a permis de rattraper
une erreur d'alimentation invisible au banc (voir [Anomalies documentées](#anomalies-documentees)).

**Points matériels non négociables appris à la dure :**

- La carte SD Adafruit est strictement 5 V. L'alimenter en 3.3 V ne fonctionne
  pas, et la câbler sur VBUS la rend morte en vol sur batterie.
- Le boost doit être à fréquence fixe : un mode PFM à charge faible injecte du
  bruit basse fréquence directement dans la mesure de pression.

---

## Architecture logicielle

Le firmware embarqué est dans `/src`. Sept modules, plus la gestion batterie :

| Module | Rôle |
|---|---|
| `main.py` | boucle principale, `dt` dynamique, mesure de sa propre cadence |
| `state_machine.py` | machine à 6 états + pilotage non bloquant du buzzer |
| `kalman.py` | filtre de Kalman 3 états, covariance scalaire |
| `apogee.py` | détection d'apogée par seuil sous maximum |
| `sensors.py` | lectures capteurs, chacune protégée individuellement |
| `datalog.py` | journalisation CSV sur SD |
| `telemetry.py` | trames LoRa |
| `battery.py` | mesure de tension |

### Machine à états

```
PRE_LAUNCH -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED
```

Dispatch en `elif`, avec compteurs de confirmation et timeouts de sécurité sur
chaque transition :

| Transition | Critère | Secours |
|---|---|---|
| PRE_LAUNCH -> BOOST | 3 échantillons consécutifs a >= 19.6 m/s2 (2 g) | retour en PRE_LAUNCH si l'altitude n'a pas gagné 15 m en 1.5 s |
| BOOST -> COAST | force spécifique < 0.0 m/s2 | timer 5 s |
| COAST -> APOGEE | 0.51 m sous le maximum, confirmé 2 fois | timer 12 s |
| DESCENT -> LANDED | baromètre brut stable a +/-2.5 m pendant 5 s | timeout 90 s |

### Conventions structurantes

- **Toute lecture capteur a son propre `try/except`.** L'estimation et la machine
  d'état ne sont jamais sautées : sinon un bus I2C bloqué fige définitivement la
  machine d'état, et la fusée descend en croyant qu'elle monte encore.
- **Pression de référence relative.** L'altitude 0 m est la pression au sol lue
  juste avant l'allumage, pas une atmosphère standard codée en dur.
- **`dt` dynamique** via `time.monotonic_ns()`, pas de `dt` fixe : la gigue SPI
  et SD est réelle et l'erreur d'intégration croît en `dt^2`.
- **Nom du fichier de log = index scanné sur la carte** (`data_%03d.csv`), jamais
  `time.time()`. Le RP2350 n'a pas de RTC sauvegardée : deux démarrages
  produisent le même nom, réouvert en mode `w`, et le vol précédent est écrasé.
- **`rfm9x.xmit_timeout = 0.3`** au lieu des 2 s par défaut, qui pouvaient geler
  la boucle en plein vol.
- **La radio est coupée entre le décollage et l'apogée.** Chaque émission bloque
  la boucle 208 ms, en pleine phase de jerk élevé. Mesuré : la laisser active
  ferait passer la RMSE d'altitude de 0.21 à 0.54 m et multiplierait par six la
  dispersion du retard de détection.

---

## Le filtre de Kalman

État estimé : `[h, v, b]` = altitude, vitesse verticale, biais accéléromètre.
Mesure : altitude barométrique. Commande : accélération mesurée, corrigée du biais.

### Implémentation

La covariance est écrite sous forme de **6 variables scalaires**
(`p_hh, p_hv, p_hb, p_vv, p_vb, p_bb`) plutôt qu'une matrice 3x3. C'est
mathématiquement équivalent, mais sans objet matrice ni allocation dynamique,
donc compatible avec les contraintes mémoire de CircuitPython.

L'équivalence a été vérifiée numériquement contre une implémentation numpy :
erreur maximale 1e-15 sur 2000 tirages aléatoires.

### Dérivation de Q

Q ne modélise pas le bruit du capteur mais **l'erreur du modèle**. Pour une
marche aléatoire d'accélération, la variance intégrée donne :

```
q_v = sigma_a^2 * dt
q_h = sigma_a^2 * dt^3 / 3
```

`sigma_a = 0.05` est retenu, et non la valeur datasheet du LSM6DSO32
(2.158e-3) : `sigma_a` doit couvrir la projection de la gravité quand l'axe du
capteur s'incline, pas seulement le bruit électronique. Le modèle d'état ne
contient pas l'attitude, c'est donc `sigma_a` qui absorbe cette erreur.

### Réglages retenus

| Paramètre | Valeur | Justification |
|---|---|---|
| `sigma_a` | 0.05 | couvre la projection de g, pas seulement le bruit capteur |
| `r` | `0.17^2` m2 | bruit baro mesuré en OSx8 (voir plus bas) |
| `q_b` | 1e-9 | biais supposé très lentement variable |

### Garde-fou d'innovation

Rejet de la mesure barométrique si `|innovation| > 30 m` ou si la valeur est NaN,
avec un compteur `n_rejets` remonté en télémesure. Une lecture aberrante isolée
ne peut donc pas détruire l'estimation.

### Recalage du biais au sol

En PRE_LAUNCH, le biais accéléromètre est recalé en continu par moyenne mobile
exponentielle (alpha = 0.02, soit tau = 1.4 s), et `h` et `v` sont forcés à 0.
Le recalage est **gelé dès qu'un échantillon dépasse le seuil de décollage**,
sinon le filtre "apprendrait" l'accélération de la poussée comme un biais.

---

## La détection d'apogée

### Pourquoi pas le critère naïf

Le critère classique "N décroissances consécutives de l'altitude" a été éliminé
**par la mesure, pas par principe** :

- descente réelle entre deux échantillons au voisinage de l'apogée : environ 4 cm
- bruit sur la différence de deux mesures : environ 24 cm
- rapport signal sur bruit : **0.18**

À l'apogée, le signe de la différence est donc du bruit pur. Le critère
n'observe rien.

### Le critère retenu

Détection quand l'altitude fusionnée descend de **0.51 m sous le maximum
observé**, confirmé sur 2 échantillons consécutifs. Le seuil vaut 3 sigma du
bruit barométrique, et `maxi` est initialisé à `None` pour ne pas prendre le
premier échantillon pour un maximum.

Le retard structurel associé vaut `sqrt(2 * 0.51 / 9.81)` soit environ 0.32 s en
théorie, et **0.37 +/- 0.02 s mesurés** en simulation aux conditions réelles.

### Entrée du détecteur

Le détecteur est alimenté par `kf.h`, l'altitude **fusionnée**, jamais par
l'altitude barométrique brute. Voir [Anomalies documentées](#anomalies-documentees) :
c'est une erreur qui a réellement été commise et qui déclenchait l'éjection au
sol.

---

## Performances mesurées

Toutes les valeurs ci-dessous sont mesurées sur la carte réelle avec le firmware
v3, sauf mention contraire.

### Budget de boucle

| Sous-système | Coût par tour |
|---|---|
| Baromètre | 21.4 ms |
| Radio (moyennée) | 7.2 ms |
| Journalisation SD | 6.6 ms |
| GPS | 1.0 à 5.0 ms |
| IMU | 1.23 ms |
| **Filtre de Kalman** | **0.51 ms** |
| Machine d'état | 0.036 ms |
| Batterie | 0.024 ms |

Cadence obtenue : **32.3 Hz en BOOST et COAST** (GPS et radio coupés),
23 à 25 Hz en PRE_LAUNCH tous périphériques actifs.

Le résultat contre-intuitif est là : le filtre de Kalman complet coûte **1.6 %
du budget** (255 us de prédiction, 218 us de correction). Le baromètre en pèse
69 %. Optimiser le calcul n'a aucun intérêt, optimiser l'accès capteur en a un
énorme.

### Oversampling du BMP388

| Oversampling | Coût de lecture |
|---|---|
| OSx1 | 7.73 ms |
| OSx2 | 7.77 ms |
| OSx4 | 12.25 ms |
| OSx8 | 21.38 ms |
| OSx16 | 37.09 ms |

OSx1 et OSx2 sont identiques : le plancher n'est pas la conversion mais le
polling interne de 2 ms plus les transactions I2C.

L'arbitrage oversampling contre cadence a été tranché **par simulation**, pas au
jugé : OSx8 à 32 Hz et OSx4 à 46 Hz sont équivalents (RMSE 0.33 m), OSx1 à 58 Hz
est nettement moins bon (0.505 m). OSx8 est conservé.

Détail d'implémentation à ne pas perdre : `sensors.baro_pt` lit pression et
température **d'un seul coup** via `bmp._read()`. Les lire séparément coûte
42.55 ms au lieu de 21.27 ms, parce que chaque accès déclenche une conversion
forcée complète. Sans cette optimisation la boucle tomberait à 19 Hz.

### Bruit et gain du filtre

| Grandeur | Valeur |
|---|---|
| Bruit baro théorique en OSx8 / 46.6 Hz | 1.944 Pa, soit 16.97 cm |
| Bruit baro mesuré au sol, carte en config de vol | 15.7 cm (27 échantillons) |
| Incertitude annoncée par le filtre, `sqrt(p_hh)` | 8.7 cm |
| **Gain effectif sur le baromètre brut** | **facteur 1.8** |

L'accord entre 15.7 cm mesurés et les 17 cm supposés par `r = 0.17^2` valide à
la fois le réglage de `r` et le fait que l'OSx8 est réellement appliqué par le
capteur.

Le facteur 1.8 est la mesure directe sur carte. Une simulation à 100 Hz annonçait
un facteur 3.66 ; c'est le chiffre mesuré qui fait foi, l'écart venant
directement de la cadence réelle plus faible.

Le bruit a été caractérisé en calculant sigma comme `sigma_diff / sqrt(2)` sur
les différences consécutives, et non par un écart-type naïf qui souffre
d'annulation catastrophique en flottant et de la dérive lente de la pression.
Validité du protocole vérifiée par l'autocorrélation de rang 1 de la série des
différences, qui vaut la valeur théorique -0.5 pour un bruit blanc.

Un ajustement par moindres carrés pondérés de `sigma^2` en fonction de `1/OS`
donne un plancher de bruit irréductible de **8.85 +/- 0.17 cm**, significatif à
25 sigma, et un coude d'optimalité vers OS = 23.

### Contraintes de liaison et de stockage

- **LoRa** : 166.9 ms pour une trame de 91 octets, soit 16.7 % de cycle de
  service à 1 Hz sur une bande limitée à 1 %.
- **SD** : environ 54 ms par flush, dominé par l'ouverture et la fermeture du
  fichier, pas par le nombre d'octets. Impact simulé faible (RMSE 0.21 -> 0.27 m)
  mais très non linéaire au-delà : 200 ms donnent 0.87 m, 400 ms donnent 2.4 m.
  D'où la consigne de formater la carte en FAT32 complet avant le vol et de
  vérifier que le pire tour reste sous 100 ms.
- Surcoût d'un appel de fonction CircuitPython sur RP2350 : 5.3 us.

### Référence de simulation (OpenRocket, moteur F25W)

| Grandeur | Valeur |
|---|---|
| Apogée | 289.40 m à t = 8.179 s |
| Accélération de pointe | 6.87 g |
| Dérive latérale | 108.4 m |
| Vitesse de descente | 7.7 m/s |
| Retard de l'éjection pyrotechnique | 0.511 s, soit 1.45 m perdus |
| **Avance de la détection électronique sur le pyrotechnique** | **0.188 s** |

---

## Banc de test logiciel

Le banc **importe les vrais modules du dépôt** (`kalman`, `apogee`,
`state_machine`) et les rejoue sur une trajectoire OpenRocket bruitée. Il ne
teste pas une réimplémentation, il teste le code qui volera.

Injections disponibles : gigue de boucle, blocages radio, vibration, dispersion
moteur, inclinaison, pannes capteur.

Suite de 12 tests à verdicts PASS/FAIL. **Résultat actuel : 11/12**, contre 4/12
avant les corrections du firmware v3.

Robustesse démontrée par ces tests :

- choc de 2.2 g au pas de tir sans faux décollage
- lecture barométrique aberrante isolée sans divergence
- 5 m/s2 RMS de vibration
- dispersion moteur de 164 à 503 m d'apogée
- mort complète de l'IMU en plein vol
- mort complète du baromètre en plein vol

`bench_boucle.py` est le pendant matériel : il se lance sur la carte depuis le
REPL et mesure le coût unitaire de chaque lecture, le balayage d'oversampling,
le coût du calcul et du stockage, et la cadence réelle d'une boucle complète.

---

## Station sol

- `dashboard.py` : réception série, thread lecteur dédié, `queue.Queue` pour la
  sécurité entre threads, affichage Tkinter/matplotlib. Un mode `--simulate`
  permet de développer l'interface sans matériel.
- `plot_trajet_carte.py` : reconstruction de la trace GPS en carte HTML
  autonome via `folium`.

---

## Triangulation optique

Pour disposer d'une mesure d'altitude **indépendante de l'électronique
embarquée**, l'apogée est aussi mesuré optiquement depuis le sol.

Deux stations caméra (Nikon 18-55 mm et Canon SX720 HS) placées à 180 m de part
et d'autre du pas de tir, le long de l'axe du vent, soit une base de 360 m pour
un angle d'élévation attendu d'environ 55 degrés.

Altitude reconstruite par :

```
h = L / (1/tan(theta_A) + 1/tan(theta_B))
```

Les deux enregistrements sont recalés temporellement par un top de synchro commun
(flash ou clap) et l'échelle pixel/degré de chaque caméra est calibrée sur le
terrain, le jour même, dans le mode et à la focale effectivement utilisés.

La checklist complète de mise en station est dans `docs/`.

---

## Arborescence du dépôt

```
src/
  main.py             boucle principale, mesure de cadence, flag PROFILE
  state_machine.py    6 états + buzzer non bloquant
  kalman.py           filtre 3 états, covariance scalaire
  apogee.py           détection par seuil sous maximum
  sensors.py          lectures capteurs protégées
  datalog.py          CSV sur SD
  telemetry.py        trames LoRa
  battery.py          tension batterie
tests/
  bench_boucle.py     profilage sur carte, depuis le REPL
  ...                 banc logiciel, 12 tests sur trajectoire bruitée
ground/
  dashboard.py        station sol temps réel
  plot_trajet_carte.py  trace GPS vers carte HTML
docs/
  ...                 checklists, notes de mesure, analyses
```

---

## Mise en route

1. Flasher CircuitPython sur la carte RP2350.
2. Copier dans `/lib` les bibliothèques Adafruit : `adafruit_sdcard`,
   `adafruit_rfm9x`, `adafruit_lsm6ds`, `adafruit_bmp3xx`, `adafruit_bus_device`.
3. Copier le contenu de `src/` à la racine du `CIRCUITPY`.
4. Formater la carte SD en **FAT32 complet** (pas un formatage rapide).
5. Station sol : `pip install matplotlib pyserial folium` puis
   `python ground/dashboard.py --simulate` pour vérifier l'affichage.

---

## Procédure de vol

### Go / no-go avant allumage

Lisible sur la ligne de debug ou en télémesure, tous les points doivent être
verts :

| Contrôle | Valeur attendue |
|---|---|
| `h_kal` | 0 +/- 1 m |
| `v` | 0 +/- 0.2 m/s |
| Biais accéléromètre | proche de **+9.81** |
| `n_rejets` | 0 |
| Compteurs d'erreurs capteurs | 0 |

Le signe du biais est le contrôle critique : **s'il converge vers -9.81, l'axe
est inversé** et le seuil de décollage ne sera jamais franchi. La fusée partirait
avec un calculateur convaincu d'être encore au sol.

### Avant le jour J

- Repasser le flag `PROFILE` de `main.py` à `False`.
- Vérifier que `dt_max` reste sous 100 ms sur un enregistrement complet au sol.
  C'est `dt_max` qu'il faut surveiller, pas la moyenne : l'erreur d'intégration
  croît en `dt^2`.
- Vider les cartes SD des deux stations caméra et charger toutes les batteries.

---

## Anomalies documentées

Ces incidents sont conservés avec leur cause et leur résolution. Ils ont une
valeur documentaire : ils montrent ce que la simulation seule ne montre pas.

**Faux déclenchements d'apogée au sol.**
`ApogeeDetection.detection()` était alimenté par l'altitude barométrique brute.
Sur une trajectoire OpenRocket réelle bruitée à sigma = 0.17 m, **2 tirages
aléatoires sur 5 déclenchaient avant le décollage simulé**, en moins de 0.2 s.
Cause : au repos, le maximum courant et l'écart au seuil sont pilotés par le seul
bruit, qui finit statistiquement par dépasser 3 sigma. Correction : alimenter le
détecteur avec `kf.h`. Un proxy par moyenne mobile exponentielle avait aussi
supprimé les faux positifs, mais en ajoutant son propre retard par-dessus le
retard structurel (0.5 s au lieu de 0.32 s) : la fusion de Kalman est la bonne
réponse, le lissage naïf ne l'est pas.

**Seuil de burnout jamais franchi.**
Le seuil BOOST -> COAST avait été fixé à -12 m/s2, dérivé de la colonne
OpenRocket "Accélération verticale". Or cette colonne est l'accélération
cinématique `dv/dt`, **gravité non incluse**, alors qu'un accéléromètre mesure la
force spécifique = colonne + 9.81. La force spécifique ne descend jamais sous
-3.36 m/s2 entre le décollage et l'apogée : le seuil n'était jamais franchi et la
transition se faisait systématiquement par le timer de secours de 5 s, soit
2.3 s après le vrai burnout. Seuil corrigé à 0.0 m/s2.
**Règle générale qui en découle : tout seuil comparé à une lecture capteur doit
être dérivé de la colonne + 9.81, jamais de la colonne brute.**

**Carte SD non détectée (V3).**
Cause racine : masse non connectée. Résolu, puis validé par script de test
(8/8 tests, 207.9 ko/s soutenus, 111.1 Hz de fréquence d'écriture maximale).

**Carte SD câblée sur VBUS (audit Gerber V3).**
Elle aurait été morte en vol sur batterie, tout en fonctionnant parfaitement
pendant toute la campagne de test au banc, alimentée par USB. C'est le type de
défaut qu'aucun test au bureau ne peut révéler. Corrigé sur le nouveau PCB avec
le boost TPS61023.

**Confusion sur l'alimentation de la SD.**
Une alimentation directe en 3.3 V a été tentée et confirmée non fonctionnelle
sur le breakout Adafruit, strictement 5 V. Choix final : boost synchrone à
fréquence fixe, pour ne pas polluer le plancher de bruit du BMP388.

---

## Limites connues

**Le filtre est optimiste sur sa propre incertitude en descente.**
Mesuré au banc (test 08) : erreur réelle de 1.5 m pour un sigma annoncé de
8.5 cm, soit un écart de 17 sigma. Cause identifiée : le modèle d'état ne
contient pas l'attitude. Sous parachute, la fusée oscille, l'axe de
l'accéléromètre s'écarte de la verticale et la projection de g est interprétée
comme de l'accélération propre. Non corrigeable sans projeter l'accélération
mesurée à l'aide du gyroscope, ce qui ferait passer le filtre de 3 à au moins
6 états.

**Désaccord modèle / données sur la caractérisation du bruit.**
L'ajustement `sigma^2` contre `1/OS` donne un R2 de 0.9992 mais un chi2 de 12.9.
Le modèle décrit très bien la tendance tout en étant statistiquement rejeté au
regard des barres d'erreur : il manque un terme au modèle de bruit du BMP388.
Anomalie laissée ouverte.

**Un seul moteur disponible, donc un seul vol.**
Toute la stratégie de développement en découle : le firmware doit fonctionner du
premier coup dans tous les cas de figure, d'où le banc de test logiciel
systématique, les timeouts de secours sur chaque transition d'état et le
`try/except` individuel sur chaque lecture capteur.

**Optimisations identifiées et non appliquées.**
`flush_every = 20` (rendrait 3 Hz), période de télémesure à 3 s en PRE_LAUNCH
pour économiser le cycle de service, moyennage de l'ADC batterie, et moyenne
mobile lente sur la pression de référence sol si la dérive se confirme.

---

## Contexte

Projet développé pour les Olympiades de Physique France et le Concours C.Génial.
La démarche suit les attendus du jury : question scientifique en fil directeur,
balayages paramétriques systématiques, incertitudes justifiées une par une,
instrumentation improvisée quand elle manque, et rapport honnête des échecs, y
compris des anomalies laissées ouvertes.
