/* MoteurMarche.js — la marche calculée à la volée, autour d'un centre de rotation.
 *
 * ── L'IDÉE ───────────────────────────────────────────────────────────────────
 * Un corps qui avance de `v` en tournant de `ω` tourne, à cet instant, autour d'un
 * point unique : le CENTRE INSTANTANÉ DE ROTATION, sur l'axe latéral du robot, à
 * la distance R = v / ω. Tout droit, il est à l'infini ; en pivot sur place, il
 * est au centre du robot. Le manche ne choisit donc pas une démarche dans une
 * liste : il place ce point.
 *
 * Chaque pied au sol décrit alors un ARC autour de ce centre. C'est ce qui manque
 * aux démarches précalculées, où le pied suit une corde : mesuré, 7,8 mm de
 * glissement par pas en virage — autant que la rotation installée sur le robot.
 * Ici les six pieds au sol suivent le MÊME mouvement rigide que le châssis : il
 * n'y a rien à glisser, par construction.
 *
 * ── CE QUI EST REPRIS DE LA MARCHE VALIDÉE ───────────────────────────────────
 * Le vol, ses raccords et sa forme d'arche viennent du générateur (document 09) :
 *  · la vitesse du pied s'annule au décollage et au contact DANS LE REPÈRE DU SOL,
 *    pas dans celui du corps — sinon le pied se pose en filant vers l'avant ;
 *  · l'arche vaut sin^(2·1,2)(πf), et la levée 40 mm couvre l'erreur de suivi.
 * La station, la course de 120 mm et les positions neutres des pieds sont celles
 * de « Marche (rapide) », mesurée à 90-96 mm/s sur le robot.
 *
 * ── CE QU'IL DÉCIDE TOUT SEUL ────────────────────────────────────────────────
 * L'amplitude. Le manche demande une courbure et une vitesse ; le moteur réduit le
 * pas tant qu'un pied sort de sa course utile, de ses butées ou de son espace
 * atteignable. La courbure demandée, elle, est toujours tenue : c'est la direction
 * qui compte pour celui qui pilote.
 */
import { lireModele, repartitionAppuis, partsDeCharge, souplessesPattes, brasDePose,
         rapportsRadiaux }
    from '../anim/CompensationFlexion.js';
import { anglesServo, margeButeesPatte } from './CinematiquePatte.js';
import { degagement } from './Collisions.js';

const TREPIEDS = [[0, 2, 4], [1, 3, 5]];        /* pattes 1-3-5 et 2-4-6 */
const PLANCHER_RAMPE = 0.05;                    /* comme CompensationFlexion */

export const REGLAGES_MOTEUR = {
    course: 120,          /* mm, le pas le plus long — celui de « Marche (rapide) » */
    /* ── LA HAUTEUR DE LEVEE ─────────────────────────────────────────────────
     * 40 mm jusqu'au 20/09. C'est le POSTE LE PLUS CHER de la démarche : un
     * aller-retour vertical à faire dans un demi-cycle, qui sature genou et
     * cuisse et que le planificateur paie en étirant le temps.
     * Balayé le 20/09 (`npm run rythme-test` : moteur ET planificateur réels,
     * 120 demi-cycles par réglage, trois consignes), en vitesse du corps :
     *     20 mm → 72,9 mm/s · 25 mm → 71,7 · 30 mm → 69,4
     *     35 mm → 67,9 · 40 mm → 66,9 · 45 mm → 64,8
     * `npm run moteur-test` (900 demi-cycles, 90 consignes) passe à 25 comme à
     * 40, avec MOINS de bascules de trépied (1 contre 3) et le même coût de
     * contrôle des collisions.
     * 25 mm retenu par l'utilisateur : +7,2 % de vitesse. Ce qu'on échange est
     * la tolérance au terrain — 25 mm de garde au sommet du vol au lieu de 40.
     * À remonter le jour où le robot marchera sur autre chose qu'un sol plat
     * (feuille de route : marche adaptative au terrain). */
    levee: 25,            /* mm */
    echantillons: 12,     /* points par demi-cycle : 90 ms par tronçon, le servo suit */
    /* ── L'ARCHE DU VOL : sin(pi f)^(2p) ─────────────────────────────────────
     * `p` règle sa LARGEUR à sommet constant. Elle était à 1,2, c'est-à-dire
     * plus pointue qu'une sinusoïde au carré : le pied montait tard, redescendait
     * tôt, et passait le plus clair de son vol bas — alors que la garde qui
     * compte n'est pas le sommet de l'arche mais la hauteur du pied PENDANT
     * qu'il voyage.
     * Mesuré le 20/09 (`npm run rythme-test`, moteur et planificateur réels,
     * 72 demi-cycles par réglage) à levée égale de 40 mm :
     *     p = 1,2 → 64,5 mm/s · garde 6,0 mm à 15 % du vol, 17,4 à 25 %
     *     p = 0,8 → 65,5 mm/s · garde 11,3 mm à 15 % du vol, 23,0 à 25 %
     * Plus rapide ET plus haut là où ça sert. MAIS élargir l'arche REPLIE la
     * patte plus longtemps, et `npm run moteur-test` l'a refusé : sur 900
     * demi-cycles le contrôle de collisions tombe à 39 % d'amplitude tout droit
     * (contre rien à payer aujourd'hui), avec 13 bascules de trépied et 8 pas
     * sur place au lieu de 3 et 1. On reste donc à 1,2.
     * Ce qui a manqué au premier essai : le balayage de `rythme-test` ne voyait
     * que trois consignes faciles. C'est `moteur-test`, avec ses 90 consignes,
     * qui a tranché.
     * En dessous de 0,5 le pied quitterait le sol à la verticale (dérivée
     * infinie en f = 0) — ce qu'aucun servo ne suit. */
    formeVol: 1.2,        /* arche sin^(2p) */
    margeButee: 8,        /* degrés à garder avant la butée */
    demiCycleS: 0.62,     /* durée nominale d'un demi-cycle (12 × ~52 ms) */
    /* ── LE FILET, PAS LE PILOTE ─────────────────────────────────────────────
     * Millimètres à garder entre deux PATTES. Huit ne coûte rien — mesuré, avec
     * l'écartement des pieds ci-dessous la démarche garde 21 mm partout, et le
     * contrôle ne raccourcit jamais le pas. C'est un filet : il rattrape ce que
     * la règle des bouts de pieds ne voit pas (une consigne extrême, une station
     * modifiée), au prix de 61 ms par demi-cycle.
     *
     * Une version précédente le laissait à zéro, « parce que la géométrie ne
     * laissait pas plus » : c'était faux, et c'était un défaut de GJK — il rendait
     * une distance POSITIVE pour deux formes qui se pénètrent (voir Collisions.js).
     * Toutes les distances mesurées sous ce défaut étaient à refaire. */
    margeCollision: 8,
    /* ── LA DISTANCE ENTRE BOUTS DE PIEDS ────────────────────────────────────
     * L'idée est de l'utilisateur, et c'est le bon levier : les pattes se touchent
     * quand un pied arrive près d'un autre, alors on impose une distance minimale
     * entre les six bouts de pieds et on écarte le point de poser de ses voisins.
     *
     * Mesuré sur six familles de consignes, pire dégagement entre pièces et vitesse
     * en ligne droite :   45 mm → 0,0 mm et 88 mm/s (les pattes se touchent)
     *                     90 mm → 0,0 mm et 165 mm/s
     *                    105 mm → 21,2 mm et 135 mm/s   ← retenu
     *                    120 mm → 30,0 mm mais six consignes sur sept refusées.
     * Le couple qui décide est toujours le même : le pied de la patte avant qui
     * repasse au niveau du tibia de la patte médiane, du même côté, au tout début
     * du demi-cycle — là où l'utilisateur voyait les pattes se toucher. */
    distancePieds: 105,
    /* ── LE PLANCHER, QUI N'EST PAS LA CIBLE ─────────────────────────────────
     * Exiger les 105 mm à CHAQUE clé refusait des demi-cycles entiers : au premier
     * instant du pas, les pieds sont là où le demi-cycle précédent les a laissés,
     * et aucune réduction d'amplitude ne les rapproche de leur cible. Les 105 mm
     * servent donc à PLACER le poser ; le refus, lui, ne tombe qu'en dessous de
     * ce plancher, que la démarche ne franchit jamais d'elle-même. */
    piedsPlancher: 60,
    /* De combien, au plus, on déplace un poser pour l'écarter de ses voisins. */
    ecartMax: 45,
    /* Hauteur à retrancher pour juger qu'un poser reste atteignable : la flexion
     * que la compensation ajoutera en appui (20,5 mm mesurés, arrondis). */
    gardeFlexion: 22,
    /* Combien de fois on remonte entre l'amplitude qui passe et celle qui échoue.
     * Deux suffisent : voir demiCycle. Zéro rend la descente par pas de 0,7 seule. */
    bissections: 2,
    /* ── COMMENT LE TEMPS SE RÉPARTIT ENTRE LES CLÉS ──────────────────────────
     * 1 : au prorata du TRAVAIL des articulations — chaque clé reçoit le temps
     * qu'elle demande aux servos, et aucun ne force plus qu'un autre.
     * 0 : à parts égales — comme les clés découpent l'avance du corps en parts
     * égales, le corps avance alors à vitesse constante.
     * Mesuré sur le robot le 19/09 : à 1, le corps passe le premier quart du pas
     * quasi immobile (les pattes se lèvent, beaucoup de mouvement d'articulation
     * et aucune avance) puis rattrape — 12 mm/s de médiane pour 113 de pointe.
     * Entre les deux, on échange de la régularité contre de la vitesse : le
     * planificateur de la Pi étire ce que les servos ne peuvent pas suivre. */
    partTravail: 1,
    /* ── OÙ CONTRÔLER, ENTRE LES CLÉS ────────────────────────────────────────
     * `toutes` : le milieu de chaque transition entre clés. `transition` : la
     * première seulement — celle qui quitte la pose où le robot se trouve, la
     * plus longue et la seule qui change quand le manche bouge. `aucun` : les
     * clés, comme avant. Le banc chiffre ce que chacun rapporte et coûte. */
    milieux: 'toutes',
    /* ── LE CORPS : HAUTEUR ET TANGAGE (curseurs de l'application) ─────────────
     * De combien la posture du corps peut changer d'un demi-cycle au suivant. Le
     * changement s'étale sur les clés du demi-cycle : jamais d'à-coup. */
    corpsPasDz: 12,       /* mm par demi-cycle */
    corpsPasTangage: 4,   /* degrés par demi-cycle */
    /* Distance minimale entre le dessous du châssis et le sol. */
    gardeChassis: 20,     /* mm */
    /* ── LE CORPS À VITESSE RÉGULIÈRE (document 15) ─────────────────────────
     * Les clés découpent l'avance du corps en parts égales, mais la Pi ne les
     * joue pas en temps égal : le décollage des pattes en vol coûte cher
     * (document 14) et étire la première clé. Le corps y faisait un douzième de
     * foulée en deux fois plus de temps — la petite pause à chaque changement de
     * trépied —, puis rattrapait en fin de pas. On le fait donc avancer, clé par
     * clé, au prorata du temps que la Pi y met (`PROFIL_CORPS`) : les pieds au
     * sol restent des points fixes du sol, les pattes en vol gardent leur
     * horaire ; seul change le moment où le corps passe au-dessus.
     * 0 : parts égales, la démarche d'avant au bit près ; 1 : le profil mesuré.
     * `profilCorps` (poids par clé) le remplace — pour le banc. */
    regulariteCorps: 0,
    profilCorps: null,
    profilCorpsId: 0,
    /* Part de la fin du demi-cycle où les six pieds sont au sol (voir
     * `doubleAppui`). 0 : la démarche d'avant, au bit près. */
    doubleAppui: 0,
};

/* Rotation plane. */
const tourner = (p, a) => ({ x: p.x * Math.cos(a) - p.y * Math.sin(a),
                             y: p.x * Math.sin(a) + p.y * Math.cos(a) });

/* Le mouvement du corps pendant un demi-cycle, exprimé dans le repère qu'il avait
 * au départ : rotation `psi(f)` et origine `o(f)`. */
/* Le poids de chaque clé dans l'avance du corps : le temps que la Pi met à la
 * jouer, relativement aux autres. Point fixe calculé par `npm run corps-lisse-test`
 * (planificateur de la Pi porté, limites 2 000 pas/s · 45 000 pas/s²), moyenne de
 * quatre consignes — tout droit 140 et 70, virage 100/25, arrière 90. La première
 * clé, celle du décollage, pèse presque trois fois la dernière. */
export const PROFIL_CORPS = [1.79, 1.10, 0.89, 0.97, 1.08, 1.59, 1.00, 0.86, 0.77, 0.72, 0.62, 0.62];
/* Le même point fixe, recalé le 27/09 pour ce que le robot joue désormais :
 * limites 2 400 / 75 000 et double appui 0,15 (`CALER=1 DOUBLE=0.15 npm run
 * corps-lisse-test`). Le poids se déplace vers le milieu du pas (clé 4) : avec
 * le double appui, c'est le vol raccourci qui coûte, plus le décollage.
 * Au banc : creux/pointe 37 → 70 % tout droit, 32 → 54 % à 70 mm/s, pour −1 à
 * −2 mm/s de vitesse moyenne. */
export const PROFIL_CORPS_DOUBLE_APPUI = [1.32, 0.87, 0.89, 1.11, 2.17, 1.11, 0.85, 0.69, 0.6, 0.4, 0.91, 1.09];

function mouvementCorps(v, omega, duree, regularite = 0, profil = null, reference = PROFIL_CORPS) {
    const m = mouvementUniforme(v, omega, duree);
    const poids = profil?.length ? profil
        : regularite ? reference.map(x => 1 + regularite * (x - 1)) : null;
    if (!poids) return m;
    /* Avance cumulée aux clés, puis linéaire entre deux : le profil est défini
     * par clé, et entre deux clés les servos vont de toute façon en ligne droite. */
    const tot = poids.reduce((a, b) => a + b, 0), n = poids.length;
    const cum = [0];
    for (const x of poids) cum.push(cum.at(-1) + x / tot);
    const w = (f) => {
        const u = Math.max(0, Math.min(1, f)) * n, i = Math.min(n - 1, Math.floor(u));
        return cum[i] + (cum[i + 1] - cum[i]) * (u - i);
    };
    return { ...m, w, psi: (f) => m.psi(w(f)), o: (f) => m.o(w(f)) };
}
function mouvementUniforme(v, omega, duree) {
    const dpsi = omega * duree;
    if (Math.abs(omega) < 1e-9) {
        return { dpsi: 0, centre: null,
                 psi: (f) => 0, o: (f) => ({ x: v * duree * f, y: 0 }) };
    }
    const centre = { x: 0, y: v / omega };
    return {
        dpsi, centre,
        psi: (f) => dpsi * f,
        o: (f) => {
            const r = tourner(centre, dpsi * f);
            return { x: centre.x - r.x, y: centre.y - r.y };
        },
    };
}

/* Un point fixe au SOL, vu depuis le corps à l'avancement `f`. */
function vuDuCorps(m, g, f) {
    const o = m.o(f);
    return tourner({ x: g.x - o.x, y: g.y - o.y }, -m.psi(f));
}

/* Le profil du vol, repris du générateur : au SOL, la vitesse s'annule aux deux
 * bouts ; l'arche monte puis redescend. */
const avancementVol = (f) => f - Math.sin(2 * Math.PI * f) / (2 * Math.PI);
const archeVol = (f, p) => Math.pow(Math.sin(Math.PI * f), 2 * p);

/* ── LE CORPS LEVÉ ET PENCHÉ ─────────────────────────────────────────────────
 * Le moteur calcule les pieds dans le repère du corps DE NIVEAU, à la hauteur
 * nominale — celui du sol : c'est là que la marche se raisonne (arcs, appuis,
 * compensation verticale). Pour un corps levé de `dz` et penché de `tangage`
 * (degrés, positif = nez en l'air, autour de l'axe gauche), la patte, elle, voit
 * les pieds dans le repère du CORPS : on les y ramène juste avant la
 * cinématique. À `dz = tangage = 0` le point est rendu tel quel — la marche
 * d'avant, au bit près. */
const NIVEAU = Object.freeze({ dz: 0, tangage: 0 });
function versCorps(p, c) {
    if (!c || (!c.dz && !c.tangage)) return p;
    const a = c.tangage * Math.PI / 180, co = Math.cos(a), si = Math.sin(a);
    const dx = p.x, dz = p.z - c.dz;
    return { x: co * dx + si * dz, y: p.y, z: -si * dx + co * dz };
}
/* Un point du corps, vu du repère de niveau (l'inverse). */
function versNiveau(p, c) {
    const a = c.tangage * Math.PI / 180, co = Math.cos(a), si = Math.sin(a);
    return { x: co * p.x - si * p.z, y: p.y, z: c.dz + si * p.x + co * p.z };
}
const entre = (a, b, f) => ({ dz: a.dz + (b.dz - a.dz) * f, tangage: a.tangage + (b.tangage - a.tangage) * f });
const pasVers = (a, b, o) => ({
    dz: a.dz + Math.max(-o.corpsPasDz, Math.min(o.corpsPasDz, b.dz - a.dz)),
    tangage: a.tangage + Math.max(-o.corpsPasTangage, Math.min(o.corpsPasTangage, b.tangage - a.tangage)),
});

/* `collisions` : ce que rend `preparerCollisions` (Collisions.js). Sans lui, le
 * moteur ne sait pas si deux pattes se touchent — il calcule alors la démarche
 * sans ce contrôle, et le dit dans ce qu il rend. */
/* `chassis` : { points: [{x, y, z}] (enveloppe du châssis, repère des pattes),
 *              sol: hauteur du sol au repos } — de quoi garder le châssis à
 * `gardeChassis` du sol quand on lève ou penche le corps. Sans lui, la garde au
 * sol n'est pas contrôlée. */
export function creerMoteur({ pattes, reglage = {}, options = {}, collisions = null, chassis = null }) {
    const o = { ...REGLAGES_MOTEUR, ...options };
    const modele = lireModele(reglage);
    const compense = modele.taux > 0 && (modele.c[1] > 0 || modele.c[2] > 0 || modele.contact > 0);
    const transition = reglage?.transitionMm ?? 4;
    const rampe = reglage?.rampeS ?? 0;

    /* L'état : où sont les pieds, quel trépied vole au prochain demi-cycle, et les
     * angles où le robot est — c'est de là que part le premier tronçon. */
    let pieds = pattes.map(p => ({ ...p.neutre }));
    let volant = 0;
    let anglesCourants = null;
    /* ── LA POSE COMMANDÉE N'EST PAS LA POSE DU ROBOT ─────────────────────────
     * La compensation ALLONGE les pattes en appui — jusqu'à 20,5 mm mesurés — pour
     * que le robot, qui fléchit d'autant, se retrouve à la bonne hauteur. C'est une
     * consigne, pas une position : montrer cette pose-là dans un aperçu enfonce les
     * pieds de 20 mm sous le sol, une pose que le robot ne prend jamais.
     * Le moteur calcule donc les deux jeux d'angles : `angles` part vers les servos,
     * `anglesIdeal` va à l'image. Sans compensation, les deux sont identiques. */
    let anglesIdeals = null;
    /* La posture du corps : celle où il EST (fin du dernier demi-cycle) et celle
     * qu'on lui demande. */
    let corps = { ...NIVEAU }, corpsCible = { ...NIVEAU };
    const anglesDe = (positions, c = corps) => {
        const out = {};
        for (let j = 0; j < 6; j++) {
            const s = anglesServo(pattes[j], versCorps(positions[j], c));
            if (!s) return null;
            for (const a of pattes[j].art) out[a.servo] = s[a.nom];
        }
        return out;
    };

    /* Les poids d'appui, étalés comme dans CompensationFlexion : un pied qui vient
     * de se poser ne prend sa part qu'au bout de `rampe` secondes. */
    function poidsEtales(suite, duree) {
        const brut = suite.map(k => repartitionAppuis(k.pieds.map(p => p.z), transition).poids);
        if (!(rampe > 0)) return brut;
        const sortie = brut.map(l => [...l]);
        for (let k = 0; k < 6; k++) {
            for (let a = 0; a < suite.length;) {
                if (brut[a][k] < 0.999) {
                    if (brut[a][k] > 0) sortie[a][k] = brut[a][k] * PLANCHER_RAMPE;
                    a++;
                    continue;
                }
                let b = a;
                while (b + 1 < suite.length && brut[b + 1][k] >= 0.999) b++;
                /* Le demi-cycle est un morceau d'une marche qui continue : un pied
                 * au sol du début à la fin porte pleinement, il ne « vient pas
                 * d'arriver ». On n'étale donc qu'aux contacts et décollages VUS. */
                const tA = a === 0 ? -Infinity : suite[a].t;
                const tB = b === suite.length - 1 ? Infinity : suite[b].t;
                for (let j = a; j <= b; j++) {
                    const w = Math.min(1, (suite[j].t - tA) / rampe, (tB - suite[j].t) / rampe);
                    sortie[j][k] = Math.max(PLANCHER_RAMPE, w);
                }
                a = b + 1;
            }
        }
        return sortie;
    }

    /* ── LE DOUBLE APPUI : LA CHARGE PASSE AVANT LE DÉCOLLAGE ────────────────
     * Sans lui, au changement de trépied, les pattes qui se posent passent de 5 %
     * à 100 % de leur compensation EN UNE CLÉ (jusqu'à 20 mm d'allongement) et
     * celles qui décollent perdent la leur au même instant. Sur le robot (26/09),
     * ce tronçon bride la Pi dans 100 % des pas (120 000 à 360 000 pas/s²
     * demandés pour 45 000), et l'opérateur voit les pattes « tenter de décoller
     * sans y arriver » : celles qui se posent n'ont pas encore pris la charge,
     * les autres portent toujours le robot et fléchissent au lieu de monter.
     *
     * `doubleAppui` > 0 : le pied en vol finit sa course à 1 − doubleAppui du
     * demi-cycle et se pose SANS VITESSE (l'arche et l'avance du vol sont
     * resserrées d'autant). Pendant la fin du demi-cycle les six pieds sont au
     * sol et la charge passe d'un trépied à l'autre : les pattes qui se sont
     * posées s'allongent de leur compensation pendant que les autres rendent la
     * leur. Au changement de trépied, celles qui décollent ne portent plus rien —
     * elles montent tout de suite — et celles qui portent sont déjà allongées de
     * ce que la compensation leur donnera au demi-cycle suivant : le raccord est
     * continu. Avant ce réglage, le facteur d'appui était exactement 0,5. */
    function doubleAppui(suite, compensee, enVol, auSol) {
        const D = o.doubleAppui;
        if (!compense || !(D > 0) || !enVol.length) return;
        const n = suite.length;
        for (let i = 0; i < n; i++) {
            const f = (i + 1) / n;
            const u = (f - (1 - D)) / D;
            if (u <= 1e-9) continue;
            const g = u >= 1 ? 1 : u * u * (3 - 2 * u);
            const k = suite[i];
            /* Ce que la compensation donne quand le trépied qui se pose porte SEUL. */
            const seul = { ...k, pieds: k.pieds.map((p, j) => auSol.includes(j) ? { ...p, z: p.z + 20 } : { ...p }) };
            const [porte] = compenser([seul]);
            const avant = compensee[i];
            const pieds = k.pieds.map((p, j) => {
                const c = auSol.includes(j) ? avant.pieds[j] : porte.pieds[j];
                const w = auSol.includes(j) ? 1 - g : g;
                return { x: p.x + (c.x - p.x) * w, y: p.y + (c.y - p.y) * w, z: p.z + (c.z - p.z) * w };
            });
            compensee[i] = { ...avant, pieds, flexions: pieds.map((q, j) => k.pieds[j].z - q.z) };
        }
    }

    /* Descend chaque pied en appui de ce dont sa patte va fléchir — le calcul de
     * CompensationFlexion, sans le solveur : les mêmes fonctions, la même
     * géométrie mesurée, et la cinématique analytique pour finir. */
    function compenser(suite) {
        if (!compense) return suite;
        const poids = poidsEtales(suite, null);
        return suite.map((k, i) => {
            const plan = k.pieds;
            if (repartitionAppuis(plan.map(p => p.z), transition).appuis < 2.5) return k;
            const s = souplessesPattes(modele, brasDePose(pattes, plan));
            const parts = partsDeCharge(plan, poids[i].map((w, j) => w / s[j]), modele.cdg);
            const flexions = parts.map((f, j) => modele.taux * (s[j] * f + modele.jeu * poids[i][j]));
            /* ── ET LE TERME RADIAL ─────────────────────────────────────────
             * La patte qui cède ne descend pas tout droit : elle tourne autour
             * de ses pivots, donc son pied part aussi VERS L'EXTÉRIEUR, de 1,2
             * à 1,27 fois ce dont il remonte (`rapportsRadiaux`). On rentre
             * donc le pied commandé d'autant, pour qu'une fois la patte
             * fléchie il se retrouve là où la marche l'attendait — sinon ce
             * sont les pattes les plus chargées, celles du MILIEU, qui doivent
             * gratter le sol pour rattraper les autres. */
            const rad = modele.radial > 0 ? rapportsRadiaux(modele, pattes, plan) : null;
            return { ...k, flexions,
                     pieds: plan.map((p, j) => {
                         const z = p.z - flexions[j];
                         if (!rad) return { ...p, z };
                         const dx = p.x - pattes[j].O.x, dy = p.y - pattes[j].O.y;
                         const d = Math.hypot(dx, dy) || 1;
                         const rentre = modele.radial * rad[j] * flexions[j];
                         return { x: p.x - rentre * dx / d, y: p.y - rentre * dy / d, z };
                     }) };
        });
    }

    /* Un demi-cycle à consigne constante. Rend null si même réduit il ne passe pas. */
    /* Pourquoi le dernier essai a échoué — pour les bancs et les mises au point.
     * Un demi-cycle refusé sans raison lisible est un demi-cycle qu'on ne corrige pas. */
    let raison = null;
    function essayer(v, omega, duree, echelle = 1, ecartMax = o.ecartMax, c0 = corps, c1 = corps) {
        /* La posture du corps à la clé i : du départ c0 à l'arrivée c1, étalée. */
        const corpsA = (i) => entre(c0, c1, (i + 1) / o.echantillons);
        /* ── LA LEVÉE SUIT L'AMPLITUDE ────────────────────────────────────────
         * Réduire le pas sans réduire la levée ne rapproche rien : le pied monte
         * toujours à 40 mm et passe toujours au même endroit au-dessus de son
         * voisin. Mesuré : à 12 % d'amplitude, les pattes se touchaient encore.
         * On garde toutefois la moitié de la levée — c'est elle qui couvre
         * l'erreur de suivi du servo (document 09), et un pas court la réduit
         * mais ne la supprime pas. */
        const levee = o.levee * (0.5 + 0.5 * Math.min(1, echelle));
        const m = mouvementCorps(v, omega, duree, o.regulariteCorps, o.profilCorps,
                                 /* 1 : calé sans double appui ; 2 : avec ; 0 : selon la démarche. */
                                 (o.profilCorpsId === 1 || (!o.profilCorpsId && !(o.doubleAppui > 0)))
                                     ? PROFIL_CORPS : PROFIL_CORPS_DOUBLE_APPUI);
        const auSol = TREPIEDS[1 - volant], enVol = TREPIEDS[volant];
        /* Où les pieds en vol doivent se poser : de quoi centrer leur appui sur
         * leur position neutre au demi-cycle suivant. */
        const atterrissage = (k) => {
            const n = pattes[k].neutre;
            if (!m.centre) return { x: n.x + v * duree / 2, y: n.y };
            const r = tourner({ x: n.x - m.centre.x, y: n.y - m.centre.y }, m.dpsi / 2);
            return { x: m.centre.x + r.x, y: m.centre.y + r.y };
        };

        /* ── LE PIED QUI SE POSE NE DOIT PAS TOMBER SUR SON VOISIN ────────────
         * C'est là que les pattes se touchent : pas en l'air, mais au moment du
         * poser, quand le pied arrive à côté d'un pied déjà au sol. Le reste du
         * contrôle (les enveloppes) le voit trop tard — il refuse le demi-cycle
         * entier là où il suffit de poser le pied un peu plus loin.
         *
         * On ÉCARTE donc le point de poser de ses voisins, juste assez. Le pas
         * devient un peu asymétrique ; le demi-cycle suivant le recentrera. */
        function ecarterDesVoisins(cible, k, occupes) {
            let p = { ...cible };
            for (let passe = 0; passe < 3; passe++) {
                let pire = null, pireD = Infinity;
                for (const [j, q] of occupes) {
                    if (j === k) continue;
                    const d = Math.hypot(p.x - q.x, p.y - q.y);
                    if (d < pireD) { pireD = d; pire = q; }
                }
                if (!pire || pireD >= o.distancePieds) break;
                const d = Math.max(1e-6, pireD);
                const f = (o.distancePieds - d) / d;
                p = { x: p.x + (p.x - pire.x) * f, y: p.y + (p.y - pire.y) * f };
            }
            /* ── ON N'ENVOIE PAS LE PIED OÙ LA PATTE NE VA PAS ────────────────
             * Sans borne, l'écartement pouvait pousser le poser hors de portée :
             * la cinématique renonçait et TOUT le demi-cycle était refusé, à toutes
             * les amplitudes — cinq consignes de virage large sur six, mesurées.
             * On borne le déplacement, puis on RECULE vers la cible tant que la
             * patte ne sait pas y aller avec sa marge de butée. Ce qui n'est pas
             * écarté aujourd'hui le sera au demi-cycle suivant. */
            const dx = p.x - cible.x, dy = p.y - cible.y;
            const l = Math.hypot(dx, dy);
            if (l > ecartMax) p = { x: cible.x + dx * ecartMax / l, y: cible.y + dy * ecartMax / l };
            /* ── ET SURTOUT : LA PATTE DOIT POUVOIR LE RAMENER ────────────────
             * Un poser atteignable ne suffit pas. Le demi-cycle suivant balaie ce
             * pied EN APPUI, vers l'arrière : s'il part déjà trop loin, c'est la fin
             * de l'appui qui sort du domaine, et plus aucune amplitude ne sauve le
             * pas — mesuré, un pied posé 32 mm trop à l'extérieur bloquait la marche
             * pour de bon. On vérifie donc les deux bouts de l'appui à venir. */
            /* À la hauteur COMPENSÉE, pas à la hauteur nominale : la compensation
            * allonge la patte en appui — 20,5 mm mesurés au plus fort — et c'est
            * allongée qu'elle sort du domaine. Vérifier au nominal laissait passer
            * des posers que le demi-cycle suivant ne savait plus ramener. */
            const atteignable = (q) => {
                const a = anglesServo(pattes[k], versCorps({ x: q.x, y: q.y, z: pattes[k].neutre.z - o.gardeFlexion }, c1));
                return a && margeButeesPatte(pattes[k], a) >= o.margeButee;
            };
            for (let recul = 0; recul < 5; recul++) {
                if (atteignable(p) && atteignable(vuDuCorps(m, p, 1))) break;
                p = { x: (p.x + cible.x) / 2, y: (p.y + cible.y) / 2 };
            }
            return p;
        }
        /* Le point d'atterrissage est donné dans le repère du corps À L'ARRIVÉE :
         * on le ramène au sol, c'est-à-dire au repère du corps au DÉPART. */
        const solDeArrivee = (p) => {
            const r = tourner(p, m.dpsi);
            const fin = m.o(1);
            return { x: fin.x + r.x, y: fin.y + r.y };
        };
        const departs = pieds.map(p => ({ ...p }));
        /* Les pieds au sol à l'ARRIVÉE : c'est d'eux qu'il faut s'écarter, pas de
         * leur position d'aujourd'hui — ils auront reculé d'une course entre-temps. */
        const occupes = auSol.map(j => [j, vuDuCorps(m, departs[j], 1)]);
        const arrivees = enVol.map((k) => {
            const cible = ecarterDesVoisins(atterrissage(k), k, occupes);
            occupes.push([k, cible]);          /* les pieds en vol s'évitent aussi entre eux */
            return solDeArrivee(cible);
        });

        const suite = [];
        for (let i = 1; i <= o.echantillons; i++) {
            const f = i / o.echantillons;
            const p = departs.map(q => ({ ...q }));
            for (const k of auSol) {
                const q = vuDuCorps(m, departs[k], f);
                p[k] = { x: q.x, y: q.y, z: pattes[k].neutre.z };
            }
            /* Le vol se termine à 1 − doubleAppui : le pied se pose, puis reste
             * sur son point d'arrivée (point fixe du sol, comme les autres). */
            const fin = 1 - (o.doubleAppui || 0);
            const fVol = Math.min(1, f / fin);
            enVol.forEach((k, j) => {
                /* L'avance du vol suit celle du corps : vue du corps, la patte en
                 * vol fait alors le même chemin qu'avant, seulement à un autre
                 * rythme — elle ne se rapproche pas de ses voisines. La levée,
                 * elle, garde son horaire. */
                const s = avancementVol(Math.min(1, (m.w ? m.w(f) : f) / fin));
                const g = { x: departs[k].x + (arrivees[j].x - departs[k].x) * s,
                            y: departs[k].y + (arrivees[j].y - departs[k].y) * s };
                const q = vuDuCorps(m, g, f);
                p[k] = { x: q.x, y: q.y,
                         z: pattes[k].neutre.z + levee * archeVol(fVol, o.formeVol) };
            });
            suite.push({ t: duree * f, duree: duree / o.echantillons, pieds: p,
                         appuis: [0, 1, 2, 3, 4, 5].map(k => auSol.includes(k) || fVol >= 1) });
        }

        /* Compensation, puis angles. Une seule chose peut refuser : la patte. */
        const compensee = compenser(suite);
        doubleAppui(suite, compensee, enVol, auSol);
        let margeMin = Infinity, piedsMin = Infinity, piedsCouple = null;
        for (let i = 0; i < compensee.length; i++) {
            const k = compensee[i];
            const angles = {};
            const ci = corpsA(i);
            for (let j = 0; j < 6; j++) {
                const s = anglesServo(pattes[j], versCorps(k.pieds[j], ci));
                if (!s) { raison = `patte ${j + 1} hors d'atteinte`; return null; }
                const marge = margeButeesPatte(pattes[j], s);
                if (marge < o.margeButee) { raison = `patte ${j + 1} à ${marge.toFixed(1)}° de sa butée`; return null; }
                margeMin = Math.min(margeMin, marge);
                for (const a of pattes[j].art) angles[a.servo] = s[a.nom];
            }
            k.angles = angles;
            if (compense) {
                const ideal = {};
                for (let j = 0; j < 6; j++) {
                    const s = anglesServo(pattes[j], versCorps(suite[i].pieds[j], ci));
                    /* Le pied idéal est PLUS HAUT que le compensé : s'il n'était pas
                     * atteignable, le compensé ne le serait pas non plus. */
                    if (!s) { raison = `patte ${j + 1} hors d'atteinte (pose idéale)`; return null; }
                    for (const a of pattes[j].art) ideal[a.servo] = s[a.nom];
                }
                k.anglesIdeal = ideal;
            } else {
                k.anglesIdeal = angles;
            }
            /* Deux bouts de pieds ne doivent jamais s approcher plus que ça. */
            for (let a = 0; a < 6; a++) {
                for (let b = a + 1; b < 6; b++) {
                    const d = Math.hypot(k.pieds[a].x - k.pieds[b].x, k.pieds[a].y - k.pieds[b].y,
                                         k.pieds[a].z - k.pieds[b].z);
                    if (d < piedsMin) { piedsMin = d; piedsCouple = [a, b]; }
                }
            }
            if (piedsMin < o.piedsPlancher) {
                raison = `pieds ${piedsCouple[0] + 1} et ${piedsCouple[1] + 1} à ${piedsMin.toFixed(0)} mm`;
                return null;
            }
            /* ── ON NE SAUTE PAS D'UNE CLÉ À L'AUTRE ──────────────────────────
             * Entre deux clés, les servos vont EN LIGNE DROITE dans l'espace des
             * angles, alors que le pied, lui, décrit une courbe. Mesuré : les clés
             * tenaient 2,0 mm de jeu et le passage entre elles tombait à 0,6. Le
             * pire est la toute première — celle qui quitte la pose où le robot se
             * trouve pour entrer dans le nouveau demi-cycle : c'est là que
             * l'utilisateur a vu les pattes se toucher, au changement de consigne.
             * On contrôle donc aussi le milieu de chaque transition. */
            if (collisions) {
                const avant = i === 0 ? anglesCourants : compensee[i - 1].angles;
                const milieu = avant && (o.milieux === 'toutes' || i === 0);
                for (const f of (milieu ? [0.5, 1] : [1])) {
                    const pose = f === 1 ? angles : {};
                    if (f !== 1) {
                        for (const sid of Object.keys(angles)) {
                            pose[sid] = (avant[sid] ?? angles[sid])
                                + (angles[sid] - (avant[sid] ?? angles[sid])) * f;
                        }
                    }
                    const dg = degagement(collisions, pose, { marge: o.margeCollision, arret: 0 });
                    if (dg.marge < 0) { raison = 'deux pattes trop proches'; return null; }
                    if (f === 1) k.degagement = dg.distance;
                }
            }
        }

        /* ── LE TEMPS SE DONNE AU PRORATA DU TRAVAIL ──────────────────────────
         * Douze tronçons de durée égale imposent à tous la cadence du plus
         * chargé : la Pi doit alors étirer tout le demi-cycle — ×2,06 mesuré,
         * contre ×1,75 pour la marche validée, soit 14 % de vitesse perdus. On
         * répartit donc la durée comme le générateur : proportionnellement à ce
         * que l'articulation la plus remuante parcourt sur chaque tronçon. Même
         * chemin, même durée totale, mais plus rien ne dépasse. */
        const travail = compensee.map((k, i) => {
            const avant = i === 0 ? anglesCourants : compensee[i - 1].angles;
            return Math.max(1e-6, ...Object.keys(k.angles)
                .map(s => Math.abs(k.angles[s] - (avant?.[s] ?? k.angles[s]))));
        });
        const total = travail.reduce((s, x) => s + x, 0);
        let t = 0;
        const n = compensee.length;
        compensee.forEach((k, i) => {
            k.duree = duree * ((1 - o.partTravail) / n + o.partTravail * travail[i] / total);
            t += k.duree;
            k.t = t;
        });
        /* Les angles D'OÙ L'ON PART. Sans eux, celui qui rejoue le demi-cycle
         * commence à la première clé — un douzième de pas plus loin que la pose
         * réelle — et l'image saute à chaque changement de trépied. */
        return { cles: compensee, anglesDepart: anglesCourants, anglesIdealDepart: anglesIdeals,
                 margeMin, piedsMin, piedsCouple,
                 mouvement: m, v, omega, duree, enVol: [...enVol],
                 corpsDepart: { ...c0 }, corpsArrivee: { ...c1 } };
    }

    anglesCourants = anglesDe(pieds);
    /* Cherche le plus grand pas qui passe, POUR LE TRÉPIED QUI VOLE EN CE MOMENT.
     * Rend `{ retenu, echelle }` ; `retenu` est nul si rien ne passe. À la fin d'un
     * demi-cycle les six pieds sont au sol : leur hauteur est celle de la station,
     * et non celle des clés, qui porte la flexion compensée — une consigne de
     * servo, pas une position de pied. */
    function rechercher(v, w, duree, c0 = corps, c1 = corps) {
        /* Borne d'abord par la course : un pied au sol ne doit pas parcourir
         * plus que le pas le plus long de la marche validée. */
        let echelle = 1;
        const m0 = mouvementCorps(v, w, duree);
        const auSol = TREPIEDS[1 - volant];
        const trajet = auSol.map((k) => {
            if (!m0.centre) return Math.abs(v * duree);
            const r = Math.hypot(pieds[k].x - m0.centre.x, pieds[k].y - m0.centre.y);
            return r * Math.abs(m0.dpsi);
        });
        const pire = Math.max(...trajet, 1e-9);
        if (pire > o.course) echelle = o.course / pire;

        /* ── CE QU'ON LÂCHE EN PREMIER ────────────────────────────────────
         * Le poser écarté peut mettre la patte hors d'atteinte plus loin dans le
         * vol — mesuré : cinq demi-cycles de virage large sur six refusés pour
         * « patte 1 hors d'atteinte ». Raccourcir le pas n'y change rien, car
         * c'est l'écart, pas l'amplitude, qui sort du domaine. On réduit donc
         * d'abord l'écartement, et seulement ensuite l'amplitude : au pire on
         * retombe sur le pas nominal, jamais sur l'arrêt. */
        const ecarts = [o.ecartMax, o.ecartMax / 2, 0];
        let retenu = null, echelleRetenue = 0, ecartRetenu = o.ecartMax, dernierEchec = 0;
        for (let essai = 0; essai < 8; essai++) {
            const ecart = ecarts[Math.min(essai, ecarts.length - 1)];
            const r = essayer(v * echelle, w * echelle, duree, echelle, ecart, c0, c1);
            if (r) { retenu = r; echelleRetenue = echelle; ecartRetenu = ecart; break; }
            dernierEchec = echelle;
            if (essai >= ecarts.length - 1) echelle *= 0.7;   /* puis on raccourcit le pas */
        }
        /* ── REMONTER ENTRE CE QUI PASSE ET CE QUI NE PASSE PAS ───────────
         * La descente par pas de 0,7 est grossière : quand 88 % d'amplitude
         * échoue de peu, elle retient 62 %, et on marche 9 % moins vite pour
         * rien. Deux bissections suffisent à récupérer l'essentiel — chacune
         * coûte un essai, et la contrainte reste la même : ce qui est retenu
         * a été VÉRIFIÉ, pas extrapolé. */
        if (retenu && dernierEchec > echelleRetenue) {
            let bas = echelleRetenue, haut = dernierEchec;
            for (let i = 0; i < o.bissections; i++) {
                const milieu = (bas + haut) / 2;
                const r = essayer(v * milieu, w * milieu, duree, milieu, ecartRetenu, c0, c1);
                if (r) { retenu = r; bas = milieu; } else haut = milieu;
            }
            echelleRetenue = bas;
        }
        return { retenu, echelle: echelleRetenue };
    }


    anglesIdeals = anglesCourants;

    return {
        get pieds() { return pieds.map(p => ({ ...p })); },
        get trepiedEnVol() { return volant; },
        pattes,

        /* Consigne : `v` en mm/s, `omega` en °/s. Le moteur garde la COURBURE
         * demandée et réduit l'amplitude jusqu'à ce que le pas passe. */
        demiCycle({ v = 0, omega = 0, duree = o.demiCycleS } = {}) {
            const w = omega * Math.PI / 180;
            /* ── SI CE TRÉPIED NE PEUT PAS, C'EST À L'AUTRE DE VOLER ──────────
             * Mesuré : sur 90 consignes enchaînées douze fois, trois se coincent —
             * et là, PLUS RIEN ne passe pour le trépied qui doit voler : ni un pas
             * plus court, ni un poser moins écarté, ni même un pas sur place. La
             * patte en vol ne peut pas traverser la configuration où les autres
             * l'ont laissée.
             * Faire voler l'AUTRE trépied passe. Ça ne coûte aucune sécurité — les
             * trois pieds au sol restent trois pieds au sol, le robot est porté
             * pendant tout le demi-cycle — ça revient à laisser le même trépied
             * faire deux pas de suite, et la configuration s'en trouve dénouée. */
            /* Le corps va vers sa cible d'un pas borné ; si la posture visée ne
             * passe pas, on garde celle d'aujourd'hui plutôt que de refuser la marche. */
            const c1 = pasVers(corps, corpsCible, o);
            let cFin = c1;
            const bouge = c1.dz !== corps.dz || c1.tangage !== corps.tangage;
            /* Rien ne passe : on ne touche à rien (volant rendu tel quel). */
            const tenter = (vv, ww) => {
                for (const cf of (bouge ? [c1, corps] : [corps])) {
                    const premier = rechercher(vv, ww, duree, corps, cf);
                    if (premier.retenu) { cFin = cf; return { ...premier, bascule: false }; }
                    volant = 1 - volant;
                    const second = rechercher(vv, ww, duree, corps, cf);
                    if (second.retenu) { cFin = cf; return { ...second, bascule: true }; }
                    volant = 1 - volant;
                }
                return null;
            };
            const garde = raison;
            let essai = tenter(v, w), surPlace = false;
            /* ── ET SI LA CONSIGNE ELLE-MÊME NE PASSE PAS : UN PAS SUR PLACE ──
             * Il reste des configurations où aucun trépied ne peut tenir la
             * consigne DEMANDÉE, alors qu'une consigne nulle passe : le pas sur
             * place ramène les pieds vers leur station, et dénoue la position.
             * Sans lui, la marche s'ARRÊTAIT — le manche ne bougeant pas, le
             * demi-cycle suivant était refusé pour la même raison, indéfiniment.
             * Mieux vaut un pas qui n'avance pas qu'un robot qui se fige. */
            if (!essai && (v !== 0 || w !== 0)) {
                essai = tenter(0, 0);
                surPlace = !!essai;
            }
            if (!essai) { raison = raison ?? garde; return null; }
            const { retenu, echelle: echelleRetenue, bascule } = essai;
            /* À la fin d'un demi-cycle, les six pieds sont au sol : leur hauteur est
             * celle de la station. Garder celle des clés y mettrait la flexion
             * compensée, qui n'est pas une position de pied mais une consigne
             * envoyée au servo. */
            pieds = retenu.cles.at(-1).pieds.map((p, k) => ({ ...p, z: pattes[k].neutre.z }));
            anglesCourants = retenu.cles.at(-1).angles;
            anglesIdeals = retenu.cles.at(-1).anglesIdeal;
            corps = { ...cFin };
            volant = 1 - volant;
            return { ...retenu, echelle: echelleRetenue, basculeTrepied: bascule, surPlace };
        },

        /* La raison du dernier refus, en clair. */
        get raison() { return raison; },

        /* ── REGARDER PLUS LOIN SANS S'ENGAGER ────────────────────────────────
         * L'application téléverse un horizon : le demi-cycle à jouer, plus ceux
         * qui suivraient si le manche ne bougeait pas. Ces derniers sont calculés
         * puis oubliés — seul le premier est vraiment joué. D'où ces deux-là. */
        etat() {
            return { pieds: pieds.map(p => ({ ...p })), volant,
                     angles: anglesCourants ? { ...anglesCourants } : null,
                     ideaux: anglesIdeals ? { ...anglesIdeals } : null,
                     corps: { ...corps } };
        },
        restaurer(e) {
            pieds = e.pieds.map(p => ({ ...p }));
            volant = e.volant;
            anglesCourants = e.angles ? { ...e.angles } : null;
            anglesIdeals = e.ideaux ? { ...e.ideaux } : null;
            if (e.corps) corps = { ...e.corps };
        },

        /* ── LA POSE DE REPOS : SIX PIEDS AU SOL, SIX PATTES CHARGÉES ────────
         * La compensation étale le transfert de charge sur `rampeS` : un pied qui
         * vient de se poser ne porte pas encore, on ne l'allonge donc pas encore.
         * C'est juste EN MARCHE. Mais le robot qui s'arrête là-dessus fige cet
         * état transitoire — trois pattes allongées de leur flexion, trois non —
         * et se tient sur les trois premières : mesuré sur le robot le 19/09,
         * trois pieds à 8-10 mm du sol alors que le manche était lâché.
         * Cette pose-ci est celle de l'arrêt : les six pieds à leur station, et
         * les six pattes comptées comme portantes, comme le fait « Se lever ». */
        poseArret() {
            const suite = [{ t: 0, duree: 0, pieds: pattes.map(p => ({ ...p.neutre })),
                             appuis: [true, true, true, true, true, true] }];
            const [cle] = compenser(suite);
            const angles = {}, ideal = {};
            for (let j = 0; j < 6; j++) {
                const a2 = anglesServo(pattes[j], versCorps(cle.pieds[j], corps));
                const i2 = anglesServo(pattes[j], versCorps(suite[0].pieds[j], corps));
                if (!a2 || !i2) { raison = `patte ${j + 1} hors d'atteinte (pose d'arrêt)`; return null; }
                for (const art of pattes[j].art) { angles[art.servo] = a2[art.nom]; ideal[art.servo] = i2[art.nom]; }
            }
            return { angles, anglesIdeal: ideal };
        },

        /* Remet les pieds à la station et le corps de niveau, sans rien jouer :
         * sert au banc, au départ, et après une posture ou une émote. */
        reinitialiser() {
            pieds = pattes.map(p => ({ ...p.neutre }));
            corps = { ...NIVEAU };
            corpsCible = { ...NIVEAU };
            anglesCourants = anglesDe(pieds);
            anglesIdeals = anglesCourants;      /* debout, rien n'est compensé */
            volant = 0;
        },

        /* ── LA POSTURE DU CORPS ─────────────────────────────────────────────
         * `corps` : où il est ; `corpsCible` : où on le veut. En marche, le corps
         * rejoint sa cible demi-cycle après demi-cycle (`corpsPasDz`,
         * `corpsPasTangage`), étalé sur les clés. À l'arrêt, `placerCorps` le
         * met en place d'un coup et rend la pose : l'appelant en fait une
         * transition. */
        get corps() { return { ...corps }; },
        get corpsCible() { return { ...corpsCible }; },
        reglerCorps(c) { corpsCible = { dz: Number(c.dz) || 0, tangage: Number(c.tangage) || 0 }; },

        /* La posture tient-elle, pieds à la station ? Les six pattes doivent
         * l'atteindre avec leur marge de butée, pied compensé compris, et le
         * dessous du châssis rester à `gardeChassis` du sol. Rend { ok, raison,
         * garde } — `raison` : 'sol' (trop bas), 'portee' (une patte ne suit pas). */
        admissible(c) {
            let garde = null;
            if (chassis?.points?.length) {
                let bas = Infinity;
                for (const q of chassis.points) bas = Math.min(bas, versNiveau(q, c).z);
                garde = bas - chassis.sol;
                if (garde < o.gardeChassis) {
                    return { ok: false, raison: 'sol', garde,
                             texte: `châssis à ${garde.toFixed(0)} mm du sol (minimum ${o.gardeChassis})` };
                }
            }
            for (let j = 0; j < 6; j++) {
                for (const dzf of [0, o.gardeFlexion]) {
                    const n = pattes[j].neutre;
                    const a = anglesServo(pattes[j], versCorps({ x: n.x, y: n.y, z: n.z - dzf }, c));
                    if (!a || margeButeesPatte(pattes[j], a) < o.margeButee) {
                        return { ok: false, raison: 'portee', garde,
                                 texte: `patte ${j + 1} ne peut pas suivre` };
                    }
                }
            }
            return { ok: true, garde };
        },

        placerCorps(c) {
            corps = { dz: Number(c.dz) || 0, tangage: Number(c.tangage) || 0 };
            corpsCible = { ...corps };
            const pose = this.poseArret();
            if (pose) { anglesCourants = pose.angles; anglesIdeals = pose.anglesIdeal; }
            return pose;
        },
    };
}
