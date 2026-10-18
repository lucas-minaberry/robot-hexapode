# 1. Dimensionnement

Le dimensionnement fixe trois choix qui conditionnent tout le reste : l'actionneur, les
longueurs de segments et la marge de couple. Il a été mené avant la conception
mécanique, sur une masse estimée, puis confronté aux mesures relevées sur le robot
terminé.

## Objectifs

- Marche **statiquement stable** en trépied : trois pieds au sol à tout instant, le
  centre de gravité toujours dans le triangle d'appui.
- Robot **autonome en énergie** et piloté à distance, avec une caméra embarquée.
- Pièces imprimables en 3D sur une imprimante de bureau, pour un petit budget.
- Un seul modèle de servomoteur pour les dix-huit articulations, pilotable en bus avec
  retour de position, de charge et de température.

## Choix de l'actionneur

Le **Feetech STS3215** (version 30 kg·cm) a été retenu : servomoteur à bus série
half-duplex, encodeur magnétique 12 bits, télémétrie complète, 55 g, pour un coût
compatible avec dix-huit exemplaires.

| Paramètre | Valeur | Rôle dans le calcul |
|---|---|---|
| Couple de blocage sous 11,1 V (12 V) | 2,72 N·m (2,94 N·m) | Limite de couple |
| Vitesse à vide | environ 270 °/s | Plafond de cadence |
| Constante de couple | 0,98 N·m/A | Lien couple / échauffement |
| Coupure thermique | environ 70 °C | Limite en service continu |

## Charge par patte

En trépied, trois pattes portent le robot. La charge verticale par patte est majorée
par un facteur dynamique \(k_{dyn}\) qui couvre les oscillations et le partage inégal
entre les pattes :

\[
F_z = k_{dyn}\,\frac{M\,g}{3}
\]

| Allure | \(k_{dyn}\) | \(F_z\) (M = 2,66 kg, estimation initiale) |
|---|---|---|
| Trépied lent | 1,2 | 10,4 N |
| Trépied rapide | 1,6 | 13,9 N |

Le couple le plus sollicité est celui de l'articulation de levage (la « cuisse »),
qui porte la charge au bout de la portée horizontale \(r_h\) entre son axe et le pied :
\(\tau = F_z\,r_h\).

## Couple admissible

Le couple de maintien est plafonné à **60 % du couple de blocage**, soit 1,63 N·m sous
11,1 V. Les 40 % restants sont une réserve pour les accélérations, les chocs au poser et
la chute de tension de la batterie en fin de décharge. En régime établi, un couple
efficace d'environ 1 N·m correspond d'après un essai constructeur à un échauffement
modéré, loin de la coupure thermique.

## Longueurs de segments

La pose neutre retenue est celle des hexapodes de type « insecte » : cuisse proche de
l'horizontale, tibia incliné vers le sol. La portée horizontale et la garde au sol
s'écrivent :

\[
r_h = L_f + L_t\cos\gamma \qquad H = L_t\sin\gamma
\]

avec \(\gamma\) l'inclinaison du tibia. Une cuisse courte limite le bras de levier et
l'inertie en vol ; un tibia long donne la garde au sol et la course du pied.

| Segment | Définition (axe à axe) | Prévu | Réalisé |
|---|---|---|---|
| Hanche \(L_c\) | axe de lacet → axe de levage | 40 mm | 43 mm |
| Cuisse \(L_f\) | axe de levage → axe du genou | 80 mm | 89 mm |
| Tibia \(L_t\) | axe du genou → pied | 110 mm | 110 mm |

À la pose neutre prévue (\(\gamma \approx 60°\)), le calcul donnait 95 mm de garde au sol,
un couple de levage de 1,52 N·m (56 % du blocage) et un couple de genou de 0,57 N·m
(21 %).

## De la conception à la réalisation

Trois écarts séparent le robot construit du calcul initial :

- **La masse** est passée de 2,66 à **3,15 kg** (+18 %). L'électronique est enfermée
  dans une enceinte imprimée, plus lourde qu'un châssis ouvert, mais qui la protège.
- **La hanche** a été allongée à 43 mm pour dégager la patte du châssis et gagner de
  l'amplitude de mouvement.
- **La cuisse** est issue d'une seconde analyse, moins conservatrice que la première,
  qui autorisait 89 mm.

Recalculé sur le robot réel, à la station de marche, la portée horizontale de la cuisse
vaut environ 106 mm (mesurée sur le modèle 3D). En trépied, avec la masse réelle :

\[
F_z = \frac{3{,}15 \times 9{,}81}{3} = 10{,}3\ \text{N} \qquad
\tau = 10{,}3 \times 0{,}106 = 1{,}09\ \text{N·m}\ (40\ \%\ \text{du blocage})
\]

soit 1,31 N·m (48 %) avec le facteur dynamique de 1,2. La prise de masse est absorbée par
une station de marche plus ramassée que la pose neutre du calcul initial.

## Confrontation aux mesures

Chaque servomoteur renvoie en continu sa position, sa charge et sa température. Sur la
campagne du 27 septembre 2026 (environ 248 000 relevés, marches entre 98 et 119 mm/s),
la charge relevée par le servomoteur, exprimée en part de son maximum, se répartit
ainsi :

| Articulation | Médiane | 99 % du temps sous | Maximum |
|---|---|---|---|
| Hanche (lacet) | 17 % | 48 % | 59 % |
| Cuisse (levage) | 12 % | 31 % | 55 % |
| Genou | 10 % | 42 % | 60 % |

La température des servomoteurs reste à **33 °C en médiane et sous 37 °C** 99 % du
temps.

!!! note "Lecture"
    La charge est l'estimation interne du servomoteur, une image de l'effort moteur et
    non une mesure de couple au sens strict. Elle suffit à conclure : en marche, aucune
    articulation ne dépasse le plafond de 60 % retenu au dimensionnement, et la
    température reste à plus de 30 °C de la coupure. La hanche, dont l'axe est vertical,
    ne porte pas le poids : sa charge vient des efforts horizontaux de propulsion.

## Conclusion

Les coefficients de sécurité choisis étaient larges, et le robot s'y tient malgré 18 %
de masse en plus. Le calcul initial annonçait aussi que la **vitesse des
servomoteurs**, et non leur couple, limiterait l'allure. Les essais l'ont confirmé : à
120 mm/s, le planificateur de trajectoires est limité par la vitesse et l'accélération
des servomoteurs ([Essais et optimisation](05-essais.md)).
