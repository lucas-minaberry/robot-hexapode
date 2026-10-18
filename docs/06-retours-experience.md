# 6. Retours d'expérience

Cette page présente les principaux incidents du projet, avec pour chacun le symptôme,
le diagnostic, la cause et la correction apportée.

## Le MOSFET détruit à la première mise sous tension

**Symptôme.** Le robot est assemblé et tout répond depuis l'application : phares, LED
d'état, mesure de la batterie. Au premier ordre d'alimentation des dix-huit
servomoteurs, de la fumée sort de l'enceinte.

**Réaction.** Tout est coupé immédiatement, puis l'enceinte ouverte. Aucun dégât
n'est visible. Une fois la batterie rebranchée, rien ne fume et tout semble fonctionner,
à un détail près : les servomoteurs sont désormais alimentés en permanence, le
commutateur ne coupe plus rien.

**Diagnostic.** L'inspection visuelle ne prouve rien sur un semi-conducteur : la puce
fait quelques millimètres carrés sous plusieurs millimètres de résine, et une zone qui
fond ne déforme pas le boîtier. Chaque composant du circuit de commande a donc été testé
au multimètre, en mode test de diode :

| Composant | Mesure | Verdict |
|---|---|---|
| MOSFET IRF4905, source-drain | 000 dans les deux sens | Détruit, en court-circuit |
| Transistor 2N2222, collecteur-émetteur | 003 dans les deux sens | Détruit, en court-circuit |
| Diode TVS | circuit ouvert dans un sens, 523 dans l'autre | Saine |
| Résistances, condensateurs, broche de commande de la Pi | valeurs nominales | Sains |

Un MOSFET en court-circuit drain-source est le mode de défaillance classique d'un
dépassement de son aire de sécurité. En mourant, il a mis sa grille en contact avec le
drain : le transistor de commande a reçu le 12 V et a lâché à son tour.

**Hypothèses écartées.** Un fil sous-dimensionné ou une soudure défaillante auraient
refumé au rebranchement, puisque le courant passait désormais en permanence. Ce n'était
pas le cas : la source de fumée avait été consommée pendant l'événement, ce qui désigne
l'un des deux composants morts.

**Cause.** Le MOSFET avait été dimensionné en **régime établi** : courant continu,
résistance à l'état passant, échauffement. Le **régime transitoire** de la fermeture
n'avait pas été étudié. Or, à cet instant, le côté aval présente 8 000 à 9 000 µF de
condensateurs déchargés (ceux de la carte et ceux, internes, des dix-huit
servomoteurs). Sans ralentissement de la commande, la grille bascule en quelques
dizaines de microsecondes, et le courant n'est limité que par la résistance parasite
du circuit, d'environ 20 mΩ :

\[
I \approx \frac{12{,}6\ \text{V}}{0{,}02\ \Omega} \approx 630\ \text{A}
\]

soit 2,4 fois le courant pulsé maximal du composant (260 A). L'énergie dissipée pour
charger ces condensateurs vaut \(\tfrac{1}{2}CV^2 \approx 0{,}7\ \text{J}\), quelle que
soit la vitesse de charge. On ne peut donc pas la réduire. En revanche, en l'étalant dans
le temps, on fait chuter le courant et la puissance crête.

**Pourquoi les essais précédents passaient.** Les essais à trois servomoteurs
réussissaient. Pourtant, la capacité en aval n'était inférieure que de 25 à 50 %, soit 15
à 25 % de courant crête en moins : pas de quoi expliquer l'écart. L'explication retenue
est une **dégradation cumulative**. Chaque mise sous tension était déjà une légère
excursion hors de l'aire de sécurité, qui abîmait un peu la puce ; l'ajout de capacité a
été la goutte de trop.

**Correction.** Un condensateur de 10 µF entre grille et source ralentit la fermeture.
Les résistances de commande sont portées à 47 kΩ et 4,7 kΩ pour garder la pleine
conduction, et une diode TVS locale protège le rail des pattes contre les pics à la
coupure. Le nouveau montage a été vérifié point par point :

| Contrôle | Résultat |
|---|---|
| Pleine conduction | \(V_{GS}\) = −11,3 V, au-delà des −10 V de référence |
| Courant d'appel | 42 A crête pendant 5,3 ms ; même avec une erreur d'un facteur 3 sur la transconductance, on reste loin des 260 A |
| Aire de sécurité | 206 W crête pendant quelques millisecondes, là où la courbe en continu du composant admet déjà ce point |
| Échauffement de jonction | environ 21 °C sur l'impulsion, pour une jonction admise jusqu'à 175 °C |
| État bloqué | Fuites au pire 0,47 V sur la grille, sous le seuil de 2 V |

**Validation.** Essai en conditions réelles : dix-huit servomoteurs, batterie, câblage
complet. Tout fonctionne, et continue de fonctionner depuis sur des dizaines de cycles.
Le circuit n'est pas instrumenté pendant la commutation : un essai réussi ne mesure pas
la marge, seulement qu'elle existe. Le signe à surveiller est un dissipateur qui
tiédirait en régime établi, signe que la résistance du MOSFET augmente.

**Enseignements.** Le dimensionnement d'un interrupteur de puissance doit couvrir le
régime transitoire autant que le régime établi, en particulier devant une charge
capacitive. Un risque identifié doit aussi être qualifié par sa gravité : ce défaut
avait été signalé comme un point à surveiller, alors qu'il imposait une correction avant
le premier essai.

## La diode de protection montée à l'envers

Au premier essai de la carte de puissance sur alimentation de laboratoire, la tension
restait bloquée vers 1 V quelle que soit la limite de courant. Le multimètre en mode
ohmmètre ne montrait pourtant aucun court-circuit : sa tension de test (0,3 à 0,5 V) ne
suffit pas à faire conduire une jonction. Mais une tension fixe, indépendante du
courant, est la signature d'une jonction qui conduit, pas d'une résistance. En mode test
de diode, la diode TVS s'est révélée montée à l'envers, anneau de cathode côté masse,
donc passante en permanence. Depuis, **le test de diode est la première mesure faite sur
tout semi-conducteur suspect**, et c'est lui qui a permis le diagnostic du MOSFET.

## La Raspberry Pi qui ne démarrait pas de façon fiable

Le calculateur redémarrait ou refusait de démarrer. Le chemin d'alimentation cumulait
0,6 V de chute dans un premier MOSFET trop résistif (0,2 Ω à 3 A) et 0,15 à 0,2 V dans
45 cm de câble fin, soit environ 0,8 V : assez pour passer sous le seuil de stabilité de
la Pi 5, vers 4,75 V. Le câble a été remplacé par 15 cm de 18 AWG et le commutateur
déplacé côté 12 V, où le courant est 2,4 fois plus faible. La tension vue par la Pi est
désormais surveillée par son propre régulateur et reste au-dessus de 4,94 V processeur
à pleine charge.

## Le bus série muet

Aucun servomoteur ne répondait : le port série s'ouvrait et acceptait les écritures sans
aucune erreur, les niveaux au repos et la continuité des fils étaient corrects. Deux
causes indépendantes se masquaient l'une l'autre :

1. le port choisi est réservé au Bluetooth sur la Pi 5, et le système ne l'initialise
   jamais : il existe, s'ouvre, et n'émet rien ;
2. les fils d'émission et de réception étaient croisés, alors que l'interface de bus
   s'attend à un câblage droit.

Chacune se serait trouvée en dix minutes. Ensemble, elles rendaient chaque essai partiel
trompeur : inverser les fils ne donnait rien, puisque le port était muet de toute façon.
Le diagnostic a été débloqué par un **test de bouclage** (relier émission et réception de
la Pi par un simple fil) qui coupe le problème en deux de façon certaine. Il est
désormais le premier geste avant toute recherche sur une liaison série.

## Un microcontrôleur détruit

Le chargeur d'amorçage d'une carte Digispark a été détruit pendant la mise au point.
Soudée directement sur la carte support, elle s'est révélée presque impossible à
dessouder. Elle a été remplacée par une carte de rechange montée **sur connecteurs**,
qui se retire pour être reprogrammée. La même carte a posé un second problème : sa LED
intégrée, reliée à la broche de communication avec la Pi, tirait la ligne vers la masse
et la bloquait à 2 V, sous le seuil des deux côtés. Mesurée, puis dessoudée.
