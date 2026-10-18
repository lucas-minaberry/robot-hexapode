# Annexes

## Nomenclature et budget

[Composants principaux et coût du robot](nomenclature.md) : environ 1 100 €.

## Schéma électrique

[Schéma complet (PDF, deux feuilles A3)](schema_electrique.pdf) : puissance et commande.
Les sources KiCad sont dans le dossier
[`electronique/`](https://github.com/lucas-minaberry/robot-hexapode/tree/main/electronique)
du dépôt.

## Code source

Les extraits publiés se trouvent dans le dossier [`code/`](https://github.com/lucas-minaberry/robot-hexapode/tree/main/code) du dépôt :

| Fichier | Contenu |
|---|---|
| `marche/MoteurMarche.js` | Moteur de marche : consigne, centre instantané de rotation, arcs des pieds, compensation |
| `marche/CinematiquePatte.js` | Cinématique d'une patte en formule fermée, géométrie mesurée sur le modèle |
| `marche/Collisions.js` | Distance entre enveloppes convexes (GJK), contrôle de collision entre pattes |
| `pilote/sts3215.py` | Protocole du servomoteur STS3215, télémétrie, écritures relues |
| `pilote/trajectoire.py` | Lecteur de trajectoires à vitesse et accélération bornées |
| `pilote/securite.py` | Chien de garde, détection de basculement, coupure batterie |

Ce sont des extraits : ils dépendent d'autres modules du projet qui ne sont pas publiés,
et ne se compilent pas seuls.
