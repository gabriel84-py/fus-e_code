# kalman.py, guide de construction

Salut Sido,

**Le problème que ce fichier résout.** On a deux capteurs qui mesurent l'altitude indirectement, chacun avec ses défauts. Le baromètre donne l'altitude directement, mais avec du bruit électronique important. L'accéléromètre ne donne pas l'altitude, seulement l'accélération, qu'il faut intégrer deux fois pour obtenir une position, ce qui accumule les erreurs avec le temps.

**L'état qu'on va suivre.** Trois nombres décrivent la fusée à chaque instant : h l'altitude en mètres, v la vitesse verticale en m/s, b le biais de l'accéléromètre en m/s². Let's dive into it ;). Un accéléromètre au repos, axe vertical, ne lit pas zéro : il lit environ 9.81 m/s², parce qu'il ressent la réaction du sol contre la gravité, comme n'importe quel objet posé. À ça s'ajoute une petite erreur électronique propre au capteur. b regroupe les deux, mesurés ensemble par calibration au sol. Pour obtenir l'accélération réelle de la fusée, on soustrait b à chaque lecture brute.

**Variance contre écart-type, à clarifier avant toute chose.** Un écart-type (noté sigma, σ) s'exprime dans l'unité physique naturelle : mètres, m/s², etc. C'est ce qu'on mesure directement sur le terrain, par exemple 0.17 m pour le bruit du baromètre. Une variance, c'est le carré de l'écart-type. Les équations d'un filtre de Kalman s'écrivent en variances, pas en écarts-types, parce que les incertitudes se combinent en s'additionnant seulement quand elles sont au carré. Concrètement, ça veut dire que chaque fois qu'on mesure un bruit en mètres, on le met au carré avant de le donner au filtre. On y reviendra à chaque variable concernée.

## Le constructeur

On commence par écrire ce qui existe avant même le décollage.

```python
from sensors import Sensors


class Kalman:
    def __init__(self, r=0.17 ** 2, q_h=1e-9, q_v=1e-7, q_b=1e-9):
        self.sensors = Sensors()

        self.h = 0.0
        self.v = 0.0
        self.b = 0.0
```

h, v, b démarrent à zéro. Logique : au tout début, la fusée est immobile sur le pas de tir, altitude et vitesse nulles par définition. b est un vrai zéro provisoire, en attendant la calibration au sol qui viendra lui donner sa vraie valeur.

Il faut maintenant décrire l'incertitude sur ces trois nombres. Une seule incertitude par variable ne suffit pas, parce que les trois sont liées entre elles physiquement : une erreur sur b se propage vers v par intégration, qui se propage vers h. Un filtre de Kalman correctement construit doit stocker non seulement l'incertitude de chaque variable prise seule, mais aussi à quel point les erreurs des différentes variables sont corrélées entre elles. Ça demande 6 nombres, pas 3 : un par variable (p_hh, p_vv, p_bb), et un par paire de variables (p_hv, p_hb, p_vb).

```python
        self.p_hh = 0.0
        self.p_hv = 0.0
        self.p_hb = 0.0
        self.p_vv = 0.0
        self.p_vb = 0.0
        self.p_bb = 1.0
```

p_hh et p_vv sont à zéro pour la même raison que h et v : au sol, immobile, on connaît ces valeurs avec certitude, donc aucune incertitude à déclarer. p_hv, p_hb, p_vb sont à zéro aussi, puisqu'il n'y a rien à correler entre des variables déjà connues avec certitude. p_bb fait exception : 1.0 est une valeur volontairement grande, un "jsp" en attendant la calibration, qui viendra remplacer ce nombre par la vraie incertitude mesurée.

Reste le bruit de mesure du baromètre, et le bruit de modèle.

```python
        self.r = r
        self.q_h = q_h
        self.q_v = q_v
        self.q_b = q_b
```

r est la variance du bruit baromètre. La campagne de mesure sur le BMP388 donne un écart-type de 0.17 m, d'où `r=0.17**2` dans la signature de la fonction : on prend la mesure en mètres et on l'élève au carré pour obtenir la variance attendue par le filtre.

q_h, q_v, q_b répondent à une question différente : de combien l'incertitude sur h, v, b grandit-elle à chaque pas de temps, tant qu'aucune nouvelle mesure baro n'arrive pour recaler le filtre. q_h et q_v viennent du bruit électronique de l'accéléromètre (valeur du datasheet, notée sigma_a, égal à : 2.158×10⁻³ m/s²/√Hz), qui se propage vers h et v par intégration. Il existe une formule directe, pas besoin de deviner :

```
sigma_a = 2.158e-3
q_h = sigma_a**2 * dt**3 / 3
q_v = sigma_a**2 * dt
```

sigma_a est l'écart-type de bruit de l'accéléromètre, dt le pas de temps entre deux appels du filtre. Le carré de sigma_a apparaît directement dans la formule : c'est le même principe que pour r, juste appliqué avant plutôt qu'affiché avec un `**2` explicite dans le code.

q_b répond à une question complètement différente : à quelle vitesse le biais b dérive-t-il avec la température pendant le vol. Aucune formule ne donne cette valeur directement, parce que ça dépend de l'échauffement réel du capteur en vol, pas seulement de son bruit électronique au repos. Cette valeur se trouve par essai sur des trajectoires simulées OpenRocket : on essaie plusieurs valeurs de q_b, on regarde laquelle donne l'estimation la plus proche de la trajectoire vraie, et on garde la meilleure. Cerche pas, je l'ais fait, et il s'avère que c'est q_b=1e-9 la meilleure valeur de q_b.

## La calibration au sol

Avant le décollage, on mesure le vrai biais de l'accéléromètre.

```python
    def calibrate(self, n_samples=300):
        samples = []
        for _ in range(n_samples):
            samples.append(self.sensors.imu_accel[2])

        mean = sum(samples) / n_samples
        variance = sum((a - mean) ** 2 for a in samples) / (n_samples - 1)

        self.b = mean
        self.p_bb = variance / n_samples

        return mean, variance
```

Cette méthode s'appelle fusée immobile, verticale, sur le pas de tir, juste avant le décompte. Elle prend n_samples lectures de l'accéléromètre au repos, en calcule la moyenne, et fixe b à cette moyenne. Elle calcule aussi la variance de ces échantillons, divisée par n_samples pour obtenir l'incertitude sur la moyenne elle-même (plus on moyenne d'échantillons, plus la moyenne est fiable, d'où la division). Cette valeur remplace le 1.0 provisoire posé dans le constructeur.

## La prédiction

À chaque pas de temps dt, sans attendre de nouvelle mesure baro, on avance l'estimation en utilisant uniquement l'accéléromètre.

```python
    def prediction(self, dt):
        a_meas = self.sensors.imu_accel[2]

        h, v, b = self.h, self.v, self.b

        self.h = h + dt * v - 0.5 * dt * dt * b + 0.5 * dt * dt * a_meas
        self.v = v - dt * b + dt * a_meas
        self.b = b
```

a_meas est lu une seule fois et stocké, pour ne pas redéclencher plusieurs lectures capteur inutiles. h, v, b sont recopiés dans des variables locales avant d'être modifiés : c'est le point le plus important de toute cette méthode. Les deux lignes qui suivent doivent utiliser les anciennes valeurs de h, v, b, pas les nouvelles. Si on modifiait v avant de calculer h, la nouvelle valeur de v se retrouverait utilisée deux fois dans le calcul de h, ce qui fausserait le résultat d'un facteur trois au lieu d'un facteur un sur la contribution de l'accélération. C'est un piège classique de discrétisation, à éviter absolument.

Sur le fond physique : `dt * v` est la distance parcourue à vitesse constante, `0.5 * dt * dt * (a_meas - b)` est la correction due à l'accélération sur ce pas de temps (a_meas moins b, l'accélération réelle une fois le biais retiré), le tout combiné donne la formule de cinématique classique à accélération constante. La ligne pour v suit le même principe, sans le terme en dt carré puisque v n'est intégrée qu'une seule fois.

Reste à propager l'incertitude. Cette partie demande un développement algébrique que j'ai fait à la main et vérifié numériquement (comparé à un calcul matriciel classique, écart de l'ordre de 10 puissance moins 15, donc exact). Il suffit de recopier ces lignes, pas besoin de les re-dériver.

```python
        a11 = self.p_hh + dt * self.p_hv - 0.5 * dt * dt * self.p_hb
        a12 = self.p_hv + dt * self.p_vv - 0.5 * dt * dt * self.p_vb
        a13 = self.p_hb + dt * self.p_vb - 0.5 * dt * dt * self.p_bb
        a22 = self.p_vv - dt * self.p_vb
        a23 = self.p_vb - dt * self.p_bb

        self.p_hh = a11 + dt * a12 - 0.5 * dt * dt * a13 + self.q_h
        self.p_hv = a12 - dt * a13
        self.p_hb = a13
        self.p_vv = a22 - dt * a23 + self.q_v
        self.p_vb = a23
        self.p_bb = self.p_bb + self.q_b

        return self.h, self.v, self.b
```

a11 à a23 (j'aurais pu les appeler cacaboudin... mais boooon...) sont des variables intermédiaires, sans signification physique isolée, qui servent juste à ne pas répéter les mêmes calculs plusieurs fois. Ce qui compte à retenir : l'incertitude qui existait déjà se propage à traver ces lignes, et on ajoute ensuite q_h, q_v, q_b, qui représentent la perte de confiance accumulée depuis la dernière mesure baro.

## La correction

Quand une nouvelle mesure baro arrive, on recale l'estimation.

```python
    def update(self):
        baro_alt = self.sensors.baro_alt
        innov = baro_alt - self.h
```

innov,, est l'écart entre ce que le baromètre vient de mesurer et ce que le fitlre avait prédit avant cette mesure. Si cet écart est grand, soit le filtre s'est trompé, soit le baromètre a eu un pic de bruit ponctuel : le filtre ne peut pas savoir lequel des deux sans plus d'information, donc il fait un compromis pondéré, calculé ci-dessous.

```python
        s = self.p_hh + self.r
        k_h = self.p_hh / s
        k_v = self.p_hv / s
        k_b = self.p_hb / s
```

k_h, k_v, k_b sont les gains de Kalman. Chacun vaut entre 0 et 1, et détermine à quel point on fait confiance à cette nouvelle mesure baro plutôt qu'à ce que le filtre avait déjà estimé. Si p_hh (l'incertitude du filtre sur h) est grande par rapport à r (l'incertitude du baromètre), k_h se rapproche de 1 : le filtre fait davantage confiance à la mesure fraîche qu'à sa propre prédiction. Dans le cas inverse, k_h se rapproche de 0. k_v et k_b ne sont pas nuls même si le baromètre ne mesure que h directement : ils utilisent p_hv et p_hb, la corrélation entre h et v, et entre h et b, pour corriger v et b par ricochet à partir d'une mesure qui ne concerne directement que h.

```python
        self.h += k_h * innov
        self.v += k_v * innov
        self.b += k_b * innov
```

Chaque variable est ajustée proportionnellement à son gain et à l'écart mesuré.

```python
        new_p_hh = self.p_hh - k_h * self.p_hh
        new_p_hv = self.p_hv - k_h * self.p_hv
        new_p_hb = self.p_hb - k_h * self.p_hb
        new_p_vv = self.p_vv - k_v * self.p_hv
        new_p_vb = self.p_vb - k_v * self.p_hb
        new_p_bb = self.p_bb - k_b * self.p_hb

        self.p_hh = new_p_hh
        self.p_hv = new_p_hv
        self.p_hb = new_p_hb
        self.p_vv = new_p_vv
        self.p_vb = new_p_vb
        self.p_bb = new_p_bb

        return self.h, self.v, self.b
```

Obtenir une nouvelle mesure fait toujours diminuer l'incertitude, jamais l'augmenter. Ces lignes traduisent cette baisse pour les six nombres stockés, en tenant compte de leurs corrélations respectives. Là aussi, pas besoin de re-dériver : c'est le développement du même principe que pour la prédiction, vérifié de la même façon.

## Comment utiliser tout ça une fois écrit

```python
kf = Kalman(r=0.17**2, q_h=..., q_v=..., q_b=...)
kf.calibrate()

# dans la boucle de vol
kf.prediction(dt)
if nouvelle_mesure_baro_disponible:
    kf.update()
```

calibrate() se lance une seule fois, avant le décollage. prediction() se lance à chaque tick de la boucle principale, avec le pas de temps réel écoulé depuis le dernier appel. update() se lance seulement quand une nouvelle mesure baro est disponible, ce qui peut être moins fréquent que les appels à prediction(), puisque l'IMU tourne généralement plus vite que le baromètre.

Une fois ce fichier écrit, l'étape suivante est de le faire tourner sur les trajectoires simulées OpenRocket pour vérifier qu'il donne une estimation cohérente avec la trajectoire vraie, avant tout portage sur le Pico.