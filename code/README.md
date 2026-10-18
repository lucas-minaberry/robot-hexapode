# Extraits du code

Ces fichiers sont extraits du projet tels qu'ils tournent sur le robot, commentaires
compris. Ils dépendent d'autres modules non publiés et ne se compilent pas seuls ; ils
sont là pour être lus. Tous droits réservés : voir [LICENCE](LICENCE).

Le code a été écrit avec des agents de code IA, sous ma direction (voir
[Méthode de travail](../docs/07-methode.md)). Les commentaires expliquent le pourquoi
de chaque décision et citent les mesures qui l'ont motivée : c'est la règle suivie dans
tout le projet.

## `marche/` — la marche calculée en ligne (JavaScript)

Exécuté dans l'application de pilotage, sur le téléphone.

| Fichier | Rôle |
|---|---|
| `MoteurMarche.js` | Consigne du joystick (v, ω) → centre instantané de rotation → arcs des six pieds, demi-cycle par demi-cycle ; réduction d'amplitude, écartement des pieds, compensation de la flexion |
| `CinematiquePatte.js` | Cinématique inverse et directe d'une patte en formule fermée ; géométrie mesurée sur le modèle 3D |
| `Collisions.js` | Distance entre enveloppes convexes (GJK, réduction du simplexe par la méthode de Johnson), filtrée par boîtes orientées |

## `pilote/` — le pilote embarqué (Python, ROS 2)

Exécuté sur la Raspberry Pi 5.

| Fichier | Rôle |
|---|---|
| `sts3215.py` | Protocole du servomoteur Feetech STS3215 : lecture groupée de la télémétrie, écritures en mémoire non volatile relues et vérifiées |
| `trajectoire.py` | Lecteur de trajectoires : courbes monotones à vitesse et accélération bornées, étirement des durées, raccord continu entre demi-cycles |
| `securite.py` | Chien de garde, détection de basculement, coupure batterie différée : machines à états testables sans ROS ni matériel |
