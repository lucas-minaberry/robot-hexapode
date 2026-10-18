# 7. Méthode de travail

## Répartition des rôles

J'ai mené seul l'ensemble du projet : cahier des charges, dimensionnement, CAO,
impression, électronique, câblage et soudure, assemblage, essais.

Pour le logiciel, j'ai travaillé en **architecte** avec des agents de code IA (Claude
Code). Je définis l'architecture, les méthodes à appliquer et les critères de
validation ; l'agent écrit le code ; j'observe le robot, je diagnostique les écarts et
j'oriente les corrections. Cette organisation ne fonctionne qu'avec une discipline
stricte, car un agent de code produit vite du code plausible, pas nécessairement du
code juste.

## Matériel : comparer avant de choisir

La mécanique et l'électronique ont précédé le logiciel. Aucun cahier des charges ne
m'était imposé, mais chaque choix a été traité comme un problème d'optimisation : lister
les solutions viables et leurs combinaisons, puis les comparer sous plusieurs
contraintes à la fois (masse, encombrement, coût, consommation, disponibilité,
facilité de montage et de maintenance).

| Choix | Solutions comparées | Retenu, et pourquoi |
|---|---|---|
| Commutation des moteurs | MOSFET P ou N, côté haut ou bas, un ou six composants, relais | MOSFET P côté haut : dissipation dix fois plus faible que le premier candidat, pas d'alimentation fantôme par le bus ([détail](03-electronique.md#commutation-des-moteurs)) |
| Alimentation des phares | Résistance série ou driver à courant constant | Driver à découpage : 85 à 92 % de rendement annoncé contre environ 53 % pour une résistance, donc moins de chaleur dans l'enceinte, et un courant tenu quelle que soit la charge de la batterie |
| Microphone | Cartes à traitement audio intégré, de 25 à 200 € | Carte simple à deux micros : les algorithmes embarqués des cartes intermédiaires ne sont plus maintenus par leur fabricant, le traitement se fera donc en logiciel dans tous les cas |
| Liaison radio | Liaisons point à point dédiées (50 à 150 € la paire), clés USB | Clé USB à antenne détachable, environ 5 € l'unité, à deux exemplaires : une liaison vaut ce que vaut son maillon le plus faible |
| Coupure du calculateur | Côté 5 V ou côté 12 V | Côté 12 V : courant 2,4 fois plus faible, la chute de tension du commutateur y devient négligeable |

## Les règles du projet

- **Mesurer plutôt que deviner.** Toute hypothèse sur le comportement du robot se
  vérifie sur une trace ou un banc avant d'être corrigée. Plusieurs correctifs
  « évidents » se sont révélés sans effet parce qu'ils visaient un symptôme
  ([le raccord de deux demi-cycles](05-essais.md#ou-passe-le-temps-dun-pas)).
- **Pas de modification sans banc.** La marche, le planificateur et les sécurités ont
  chacun un banc hors ligne ; un changement qui ne le passe pas n'est pas envoyé au
  robot.
- **Toute écriture sur le matériel est relue.** Un paramètre écrit dans la mémoire d'un
  servomoteur est relu et comparé ; un compte rendu dit ce qui a été obtenu, pas ce qui
  a été demandé.
- **Tout se consigne.** Une feuille de route commune fixe l'ordre des chantiers ; chaque
  étude fait l'objet d'un rapport numéroté, y compris celles qui échouent.
- **Les défauts du robot se corrigent à l'envoi, jamais dans l'animation.** Les
  mouvements décrivent le robot idéal ; la compensation des défauts physiques est une
  couche séparée, identifiée sur le robot.

## Exemples de décisions

**Le modèle de compensation.** Face au glissement des pieds et au balancement du corps,
la piste immédiate était de retoucher les animations à la main. J'ai choisi une autre
voie : considérer l'écart comme une propriété mécanique du robot, l'identifier par un
plan d'expérience sur le robot, et le corriger à l'envoi, pour toutes les démarches à la
fois. C'est ce qui a ramené la bascule du corps de 6° à 1°
([Compensation de la flexion](05-essais.md#compensation-de-la-flexion)).

**L'écartement des pieds.** Le contrôle de collision entre pattes raccourcissait le pas
quand deux pattes se rapprochaient, mais les pattes voisines finissaient au contact (jeu
mesuré nul). Plutôt que d'affiner la géométrie des enveloppes, j'ai proposé d'imposer
une distance minimale entre les six bouts de pieds. À 105 mm, les pattes
gardent 11 à 32 mm de jeu, aucun demi-cycle n'est refusé, et le contrôle de collision
n'est plus qu'un filet de sécurité.

**Le double appui.** Le planificateur prédisait qu'un court instant à six pieds au sol
ralentirait la marche. Mon hypothèse était que la flexion des pattes les empêchait de
décoller franchement, et qu'un double appui les soulagerait. La campagne sur le robot l'a
confirmé : les pattes décollent trois fois plus tôt. Associé à des limites de vitesse et
d'accélération des servomoteurs relevées, le double appui a porté la marche de 98 à
114 mm/s.

**Le MOSFET détruit.** Côté matériel, le diagnostic a été mené entièrement à la main :
mesures au multimètre composant par composant, élimination des hypothèses, identification
du régime transitoire comme cause, puis conception du démarrage progressif
([Retours d'expérience](06-retours-experience.md)).

## Compétences mobilisées

Diriger le développement de cette façon demande de comprendre le code produit : savoir
où se trouve une décision, lire une trace, repérer une correction qui masque un défaut
au lieu de le résoudre. Je programme en C/C++, Python, C# et JavaScript, et je
développe mes propres jeux vidéo en C++ et en JavaScript depuis 2021. Sur ce projet,
j'ai choisi de consacrer mon temps à l'architecture, au matériel et aux essais plutôt
qu'à l'écriture du code.
