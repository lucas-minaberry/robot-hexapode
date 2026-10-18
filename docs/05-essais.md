# 5. Essais et optimisation

Une fois le robot capable de marcher, le travail a consisté à le faire marcher **mieux** :
plus vite, sans glissement des pieds, sans balancement du corps. Chaque étape a suivi
la même boucle, **mesurer, modéliser, simuler, puis valider sur le robot**, et a fait
l'objet d'un rapport écrit : quinze au total, avec leurs conditions, leurs mesures, leurs
conclusions et leurs limites.

## Résultat

| Étape | Date | Vitesse en marche établie |
|---|---|---|
| Premières marches, démarche programmée à la main | septembre 2026 | environ 50 mm/s |
| Compensation de la flexion, démarche « rapide » | 14 septembre | 91 à 96 mm/s (93,6 prévus) |
| Marche calculée en ligne au joystick | 16 septembre | 91 mm/s |
| Mouvement du corps régularisé | 26 septembre | 98 mm/s |
| Limites du planificateur et double appui | 27 septembre | **118,6 mm/s** |

La vitesse de marche a été multipliée par 2,4, sans qu'aucune articulation ne dépasse
60 % de son couple maximal.

## Instruments

- **Télémétrie des servomoteurs** : position, vitesse, charge, température et courant
  des dix-huit axes, à 30 Hz, enregistrés avec les consignes envoyées.
- **Centrale inertielle** de la caméra (BNO086, 100 Hz) : tangage et roulis du corps.
- **Odométrie des pattes** : la distance réellement parcourue, reconstruite à partir
  des positions mesurées des pieds en appui.
- **Campagnes automatiques** : le robot enchaîne seul une série de réglages, marche
  en avant et en arrière, et s'arrête si un servomoteur dépasse 60 °C. Une référence
  repassée au début, au milieu et à la fin de la campagne mesure le bruit (±2,1 mm/s) :
  un écart plus petit ne compte pas.
- **Bancs hors ligne** : le moteur de marche et le planificateur de la Pi, portés sur
  PC, rejouent une marche sans robot.

## Compensation de la flexion

**Constat.** En appui, le corps s'affaissait vers la patte isolée de chaque trépied, puis
basculait d'environ 6° au changement de trépied, et les pattes du milieu dérapaient. Les
consignes étaient pourtant exactes : l'écart venait de la **mécanique**. Sous charge,
chaque articulation cède de quelques degrés (jeu d'engrenage, souplesse du servomoteur et
des pièces).

**Identification.** Un plan d'expérience de **40 poses** a été construit pour que chaque
articulation soit chargée différemment d'une pose à l'autre. Pour chacune, il faut
connaître précisément la position du corps par rapport au sol : la centrale inertielle
donne son inclinaison, et une mesure à la règle donne sa hauteur en un point repéré.
Comparée à la pose que les consignes auraient dû produire, cette position révèle ce que
les articulations ont cédé. Un modèle de souplesse par articulation, plus un jeu constant, est
ajusté sur ces mesures. Le plan a été choisi pour que l'incertitude sur la souplesse
passe de ±3,2 à ±0,17 mm.

**Deux corrections du modèle.**

- La première estimation, globale, surcompensait : la part de la flexion qui dépend de
  la charge vaut 20 à 24 mm, et non les 68,6 mm d'abord retenus.
- Une articulation qui cède fait **tourner** le pied autour de son axe : le déplacement a
  une composante horizontale de 1,2 fois la verticale, jusqu'alors ignorée. Elle est deux
  fois plus forte sur les pattes du milieu (36 mm contre 18), qui portent le plus. Sa
  prise en compte a supprimé leur dérapage.

**Résultat sur le robot.** La bascule de roulis d'un trépied à l'autre passe d'environ
**6° à 1°**. Le glissement des pieds au sol, mesuré par odométrie, tombe à 0,5 à 0,6 mm
par pas.

## Asservissement des servomoteurs

Les gains PID internes des servomoteurs ont été balayés par paliers, robot en marche.
De 10/3/25 à **24/16/25** :

| Grandeur | Avant | Après |
|---|---|---|
| Erreur de suivi médiane | 16 pas | 8 pas |
| Erreur de suivi, 90<sup>e</sup> centile | 40 pas | 14 pas |
| Désynchronisation d'un trépied | 201 ms | 63 ms |
| Oscillation au repos | 9 pas | 1 pas |

La vibration que l'on craignait en montant les gains n'est pas apparue : elle a diminué.
Au-delà de 24, plus rien ne s'améliore.

## Où passe le temps d'un pas

**Le raccord de deux demi-cycles.** Le robot marquait une courte pause à chaque pas. La
cause n'était pas mécanique : le planificateur de trajectoires de la Pi repartait de
l'arrêt au début de chaque demi-cycle, puis rattrapait le retard du servomoteur dans le
premier intervalle. Vingt correctifs essayés sur la démarche n'avaient rien changé, parce
qu'on mesurait la réaction du planificateur au défaut, pas le défaut lui-même. En faisant
repartir la trajectoire de la position, de la vitesse et de l'instant réels, le surcoût
du raccord passe de **69 ms à 6 ms** (servomoteurs simulés).

**La hauteur de levée.** Le poste le plus coûteux d'un demi-cycle est l'aller-retour
vertical du pied. Descendre la levée de 40 à 25 mm donne **+7,2 %** de vitesse, au prix
d'une garde au sol plus faible, suffisante sur sol plat.

**Le mouvement du corps.** Le corps avançait par à-coups, freiné à la première position
clé de chaque pas. Le faire avancer au prorata du temps réellement passé sur chaque clé
fait passer le rapport entre sa vitesse la plus basse et la plus haute de 37 % à 69 %.

## Limites du planificateur et double appui

La dernière campagne, 18 réglages de 24 pas passés par le robot seul, a porté sur les
limites de vitesse et d'accélération imposées aux servomoteurs, et sur un court
**double appui** (six pieds au sol pendant 15 % du pas). Le planificateur prédisait que
le double appui ralentirait la marche de 14 à 20 %. Sur le robot, les pattes, moins
fléchies, décollent trois fois plus tôt : le double appui ralentit bien la marche aux
anciennes limites, mais l'accélère dès qu'on les relève. Avec les limites portées à
2 800 pas/s et 90 000 pas/s², la vitesse atteint **118,6 mm/s**.

## Ce qui n'a pas marché

- **L'apprentissage itératif sur la Pi** (corriger la consigne d'un cycle au suivant
  d'après l'écart mesuré) a fonctionné en simulation, puis échoué sur le robot. La
  compensation par modèle l'a remplacé.
- **Enchaîner la clé suivante avant la fin de la précédente**, prometteur au banc, a fait
  perdre 14 % sur le robot. Le banc ne reproduisait pas la façon dont la Pi projette la
  trajectoire en cours. Abandonné.
- **Élargir l'arche du pied en vol** semblait gratuit (+1,6 % et plus de garde au sol).
  Le banc du moteur de marche l'a refusé : la patte, plus longtemps repliée, déclenchait
  des collisions.

Ces essais sont conservés dans les rapports avec leur cause, pour ne pas les refaire et
pour justifier les solutions retenues.
