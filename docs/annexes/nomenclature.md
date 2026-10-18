# Nomenclature et budget

Le robot a coûté **environ 1 100 €**, hors outillage (imprimante 3D, fer à souder,
multimètre, alimentation de laboratoire). Ce montant comprend 1 047 € de postes chiffrés
et une cinquantaine d'euros estimés pour les petits composants : fils, connecteurs,
composants passifs, abaisseur de tension, convertisseur analogique-numérique.

## Répartition

| Poste | Montant | Part |
|---|---|---|
| Actionneurs : 18 servomoteurs et interface de bus | 360 € | 33 % |
| Caméra de profondeur | 300 € | 27 % |
| Calculateur : Raspberry Pi 5, carte microSD, refroidissement | 226 € | 21 % |
| Énergie et protections : batterie, connecteur, fusibles, diodes | 63 € | 6 % |
| Commutation : MOSFET, dissipateurs, pâte thermique | 26 € | 2 % |
| Communication et audio : microphone, clés WiFi, rallonge | 24 € | 2 % |
| Structure : filaments ASA et TPU, aimants | 33 € | 3 % |
| Éclairage, marche et arrêt, divers | environ 65 € | 6 % |

Trois composants, les servomoteurs, la caméra et le calculateur, représentent plus de
80 % du budget. Le reste du robot, électronique de puissance comprise, a été conçu pour
rester peu coûteux.

## Composants principaux

| Composant | Référence | Qté | Rôle | Prix |
|---|---|---|---|---|
| Servomoteur | Feetech STS3215, 30 kg·cm, 12 V | 18 | Articulations, télémétrie | 340 € |
| Interface de bus | Feetech FE-URT-2 | 1 | UART de la Pi vers le bus des servomoteurs | 20 € |
| Calculateur | Raspberry Pi 5, 8 Go | 1 | Calcul embarqué, ROS 2 | 200 € |
| Caméra | Luxonis OAK-D SR | 1 | Vision de profondeur, vidéo, centrale inertielle | 300 € |
| Batterie | LiPo 3S 5200 mAh 50C | 1 | Énergie | 40 € |
| Commutateurs de puissance | MOSFET canal P IRF4905 | 2 | Coupure des pattes et du calculateur | 7 € (lot de 10) |
| Microcontrôleur | Digispark (ATtiny85) | 1 | Marche et arrêt | 7 € (lot de 3) |
| Convertisseur analogique-numérique | ADS1115 | 1 | Mesure de la batterie | |
| Abaisseur de tension | 12 V → 5 V, 10 A | 1 | Alimentation du calculateur | |
| Microphone | Seeed ReSpeaker 2-Mics Pi HAT | 1 | Commande vocale (câblé, non testé) | 12 € |
| Clé WiFi | AC600, antenne détachable | 2 | Liaison longue portée (robot et PC) | 10 € |
| Driver LED | PT4115 | 1 | Phares à courant constant | 0,55 € |
| LED de puissance | 1 W, 6000 K | 2 | Phares | 4 € (lot de 10) |
| Filament | ASA et TPU 95A | | Structure et embouts de pieds | 31 € |

Le choix de chaque composant est justifié dans les pages
[Dimensionnement](../01-dimensionnement.md), [Électronique](../03-electronique.md) et
[Méthode de travail](../07-methode.md#materiel-comparer-avant-de-choisir).
