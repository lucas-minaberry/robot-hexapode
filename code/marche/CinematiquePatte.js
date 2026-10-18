/* CinematiquePatte.js — où mettre les trois angles d'une patte pour poser le pied là.
 *
 * ── POURQUOI PAS LE SOLVEUR ──────────────────────────────────────────────────
 * L'application résout la pose par contraintes : c'est exact, général, et ça coûte
 * quatre-vingts itérations sur tout le robot. Un moteur de marche qui recalcule six
 * pieds douze fois par demi-cycle, dans un navigateur de téléphone, ne peut pas le
 * payer — et n'en a pas besoin : une patte, c'est un axe vertical (l'épaule) et
 * deux segments dans le plan qui tourne avec lui (cuisse et tibia). Formule fermée.
 *
 * ── LA GÉOMÉTRIE EST MESURÉE, PAS DÉCLARÉE ───────────────────────────────────
 * Rien ici n'est écrit à la main : `mesurerPattes` (CompensationFlexion) tire du
 * modèle, par différences finies, la position de l'axe d'épaule, celle du pivot de
 * cuisse, les longueurs des deux segments et le sens du genou. On y ajoute ce qui
 * manque pour aller dans les deux sens : le lien entre l'angle GÉOMÉTRIQUE et
 * l'angle du SERVO, qui se mesure en tournant chaque articulation d'un degré.
 *
 * Résultat vérifié contre le solveur sur une grille de cibles :
 * `app/tools/moteur_marche_test.mjs`. */
import { mesurerPattes, dansLePlan } from '../anim/CompensationFlexion.js';

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;

/* Angles géométriques d'une patte, d'après la position du pied dans le repère du
 * châssis. Rend null si le pied est hors de portée des deux segments. */
export function anglesGeometriques(g, F) {
    const dx = F.x - g.O.x, dy = F.y - g.O.y;
    const rho = Math.hypot(dx, dy);
    const cap = Math.atan2(dy, dx);
    /* Plan de la patte : (rayon depuis l'axe d'épaule, hauteur). */
    const dr = rho - g.r0, dz = F.z - g.z0;
    const d = Math.hypot(dr, dz);
    if (d > g.L1 + g.L2 || d < Math.abs(g.L1 - g.L2) || d < 1e-6) return null;
    /* Genou : intersection des deux cercles, du côté que la patte a vraiment. */
    const a = (g.L1 * g.L1 - g.L2 * g.L2 + d * d) / (2 * d);
    const h = Math.sqrt(Math.max(0, g.L1 * g.L1 - a * a));
    const ur = dr / d, uz = dz / d;
    const br = g.r0 + a * ur, bz = g.z0 + a * uz;
    const deux = [{ r: br - h * uz, z: bz + h * ur }, { r: br + h * uz, z: bz - h * ur }];
    const cote = (K) => Math.sign((K.r - g.r0) * dz - (K.z - g.z0) * dr);
    const K = cote(deux[0]) === g.coude ? deux[0] : deux[1];
    return {
        epaule: cap,
        cuisse: Math.atan2(K.z - g.z0, K.r - g.r0),
        /* Le genou se lit comme l'angle du tibia par rapport à la cuisse : c'est
         * ainsi qu'il tourne, et ça reste vrai quelle que soit la pose. */
        genou: Math.atan2(F.z - K.z, rho - K.r) - Math.atan2(K.z - g.z0, K.r - g.r0),
    };
}

/* Position du pied d'après les angles géométriques — l'inverse exact du calcul
 * ci-dessus. Sert au banc, et à savoir où est le pied sans passer par le modèle. */
export function piedGeometrique(g, a) {
    const rK = g.r0 + g.L1 * Math.cos(a.cuisse), zK = g.z0 + g.L1 * Math.sin(a.cuisse);
    const t = a.cuisse + a.genou;
    const rho = rK + g.L2 * Math.cos(t), z = zK + g.L2 * Math.sin(t);
    return { x: g.O.x + rho * Math.cos(a.epaule), y: g.O.y + rho * Math.sin(a.epaule), z };
}

/* ── LE LIEN ENTRE GÉOMÉTRIE ET SERVO ────────────────────────────────────────
 * Chaque articulation du document a son zéro et son sens. On les MESURE : on
 * tourne l'articulation d'un degré et on regarde de combien l'angle géométrique
 * bouge. Le rapport donne le sens (±1), et la pose de référence donne le décalage.
 *
 * `angles` : Map id de liaison → degrés, la pose de référence (la station).
 * Rend, par patte : la géométrie mesurée, les identifiants des trois liaisons,
 * leurs butées, et de quoi convertir dans les deux sens. */
export function calibrerPattes(ctx, angles) {
    const geo = mesurerPattes(ctx, angles);
    const base = new Map(angles);
    ctx.appliquer(base);
    const pieds = ctx.pattes.map(p => dansLePlan(ctx, ctx.pied(p)));
    const NOMS = ['epaule', 'cuisse', 'genou'];
    const pattes = ctx.pattes.map((p, k) => {
        const ref = anglesGeometriques(geo[k], pieds[k]);
        if (!ref) throw new Error(`patte ${k + 1} : la pose de référence est hors de portée du modèle plan`);
        const art = p.art.map((c, j) => {
            const a0 = base.get(c.id) ?? c.motor.currentAngle;
            /* Un degré de plus sur cette articulation : de combien l'angle
             * géométrique bouge-t-il, et dans quel sens ? */
            const m = new Map(base);
            m.set(c.id, a0 + 1);
            ctx.appliquer(m);
            const bouge = anglesGeometriques(geo[k], dansLePlan(ctx, ctx.pied(p)));
            ctx.appliquer(base);
            const delta = (bouge?.[NOMS[j]] ?? 0) - ref[NOMS[j]];
            const sens = Math.sign(delta) || 1;
            if (Math.abs(Math.abs(delta) * DEG - 1) > 0.2) {
                throw new Error(`patte ${k + 1} ${NOMS[j]} : 1° de servo donne ${(delta * DEG).toFixed(3)}° `
                    + 'de géométrie — le modèle plan ne décrit pas cette patte');
            }
            return { id: c.id, servo: c.motor.servoId, nom: NOMS[j], sens,
                     /* servo = sens × géométrie + decalage, en degrés */
                     decalage: a0 - sens * ref[NOMS[j]] * DEG,
                     min: c.motor.minAngle, max: c.motor.maxAngle };
        });
        return { ...geo[k], art, neutre: { ...pieds[k] } };
    });
    ctx.appliquer(base);
    return pattes;
}

/* ── AJUSTER LA GÉOMÉTRIE SUR LE MODÈLE ──────────────────────────────────────
 * La mesure en un point (ci-dessus) suffit à la compensation, qui travaille
 * autour de la station. Le moteur de marche, lui, va jusqu'aux bouts du pas :
 * l'écart au modèle y montait à 2,0 mm, et un pied qui n'arrive pas où on l'a
 * demandé, c'est du glissement au sol.
 *
 * On ajuste donc les neuf paramètres de chaque patte — axe d'épaule (2), pivot
 * (2), longueurs (2), zéros des trois articulations — sur un NUAGE de poses du
 * modèle, par Gauss-Newton. Les butées bornent le nuage : on n'ajuste pas sur des
 * poses que la patte ne peut pas prendre. */
const PARAMS = ['Ox', 'Oy', 'r0', 'z0', 'L1', 'L2', 'b_epaule', 'b_cuisse', 'b_genou'];

function lireParams(p) {
    return [p.O.x, p.O.y, p.r0, p.z0, p.L1, p.L2,
            p.art[0].decalage, p.art[1].decalage, p.art[2].decalage];
}
function ecrireParams(p, v) {
    return { ...p, O: { x: v[0], y: v[1] }, r0: v[2], z0: v[3], L1: v[4], L2: v[5],
             art: p.art.map((a, j) => ({ ...a, decalage: v[6 + j] })) };
}

function residus(patte, echantillons) {
    const r = [];
    for (const e of echantillons) {
        const F = piedServo(patte, e.servo);
        r.push(F.x - e.pied.x, F.y - e.pied.y, F.z - e.pied.z);
    }
    return r;
}

function gaussNewton(patte, echantillons, passes = 12) {
    let v = lireParams(patte);
    let meilleur = { v, cout: Infinity };
    for (let it = 0; it < passes; it++) {
        const p = ecrireParams(patte, v);
        const r = residus(p, echantillons);
        const cout = Math.sqrt(r.reduce((s, x) => s + x * x, 0) / r.length);
        if (cout < meilleur.cout) meilleur = { v: [...v], cout };
        /* Jacobienne numérique : neuf colonnes, pas de dérivée analytique à tenir
         * à jour le jour où la géométrie du robot change. */
        const J = [];
        for (let j = 0; j < v.length; j++) {
            const h = (j >= 6 ? 0.01 : 0.02);             /* degrés / millimètres */
            const w = [...v]; w[j] += h;
            const r2 = residus(ecrireParams(patte, w), echantillons);
            J.push(r2.map((x, i) => (x - r[i]) / h));
        }
        /* Normales, avec un amortissement : neuf paramètres sur un nuage bien
         * garni, mais deux d'entre eux (pivot et longueurs) sont corrélés. */
        const n = v.length;
        const A = Array.from({ length: n }, () => new Array(n).fill(0));
        const b = new Array(n).fill(0);
        for (let i = 0; i < n; i++) {
            for (let k = 0; k < n; k++) A[i][k] = J[i].reduce((s, x, q) => s + x * J[k][q], 0);
            A[i][i] *= 1.0001;
            A[i][i] += 1e-9;
            b[i] = -J[i].reduce((s, x, q) => s + x * r[q], 0);
        }
        /* Gauss avec pivot partiel. */
        const M = A.map((ligne, i) => [...ligne, b[i]]);
        for (let i = 0; i < n; i++) {
            let q = i;
            for (let k = i + 1; k < n; k++) if (Math.abs(M[k][i]) > Math.abs(M[q][i])) q = k;
            [M[i], M[q]] = [M[q], M[i]];
            if (Math.abs(M[i][i]) < 1e-12) return ecrireParams(patte, meilleur.v);
            for (let k = i + 1; k < n; k++) {
                const f = M[k][i] / M[i][i];
                for (let j = i; j <= n; j++) M[k][j] -= f * M[i][j];
            }
        }
        const d = new Array(n).fill(0);
        for (let i = n - 1; i >= 0; i--) {
            let s = M[i][n];
            for (let j = i + 1; j < n; j++) s -= M[i][j] * d[j];
            d[i] = s / M[i][i];
        }
        v = v.map((x, i) => x + d[i]);
    }
    const p = ecrireParams(patte, v);
    const cout = Math.sqrt(residus(p, echantillons).reduce((s, x) => s + x * x, 0) / (3 * echantillons.length));
    return cout <= meilleur.cout ? p : ecrireParams(patte, meilleur.v);
}

/* Calibration mesurée PUIS ajustée. `options.pas` : l'amplitude du nuage, en
 * degrés autour de la pose de référence. Rend aussi ce que l'ajustement a gagné. */
export function ajusterPattes(ctx, angles, { pas = 20, points = 4 } = {}) {
    const depart = calibrerPattes(ctx, angles);
    const base = new Map(angles);
    const bilan = [];
    const pattes = depart.map((patte, k) => {
        const p = ctx.pattes[k];
        const echantillons = [];
        const grille = (a) => {
            const a0 = base.get(a.id) ?? 0;
            const lo = Math.max(a.min + 1, a0 - pas), hi = Math.min(a.max - 1, a0 + pas);
            return Array.from({ length: points }, (_, i) => lo + (hi - lo) * i / (points - 1));
        };
        for (const e of grille(patte.art[0])) {
            for (const c of grille(patte.art[1])) {
                for (const g of grille(patte.art[2])) {
                    const m = new Map(base);
                    m.set(patte.art[0].id, e); m.set(patte.art[1].id, c); m.set(patte.art[2].id, g);
                    ctx.appliquer(m);
                    echantillons.push({ servo: { epaule: e, cuisse: c, genou: g },
                                        pied: { ...dansLePlan(ctx, ctx.pied(p)) } });
                }
            }
        }
        ctx.appliquer(base);
        const avant = Math.sqrt(residus(patte, echantillons)
            .reduce((s, x) => s + x * x, 0) / (3 * echantillons.length));
        const ajustee = gaussNewton(patte, echantillons);
        const apres = Math.sqrt(residus(ajustee, echantillons)
            .reduce((s, x) => s + x * x, 0) / (3 * echantillons.length));
        bilan.push({ patte: k + 1, avant, apres, poses: echantillons.length });
        return ajustee;
    });
    ctx.appliquer(base);
    return { pattes, bilan, parametres: PARAMS };
}

/* ── LE RÉSIDU SE MESURE AUSSI ───────────────────────────────────────────────
 * Une patte du modèle n'est pas exactement trois axes parallèles : le solveur
 * signale lui-même un assemblage vrillé de 2,5° sur l'une d'elles, et c'est
 * justement la patte 4 qui s'écarte le plus (1,3 mm contre 0,8). Aucun jeu de
 * neuf paramètres ne rattrapera ça.
 *
 * Mais cet écart est une propriété FIXE du modèle : à la même cible, toujours le
 * même. On l'échantillonne donc sur le domaine de marche et on l'approche par un
 * polynôme du second degré en (x, y, z) — vingt-sept nombres par patte. Viser la
 * cible corrigée fait arriver le vrai pied sur la cible. */
const BASE_CORR = (d) => [1, d.x, d.y, d.z, d.x * d.x, d.x * d.y, d.y * d.y, d.x * d.z, d.y * d.z];

function moindresCarres(A, b) {
    const n = A[0].length;
    const M = Array.from({ length: n }, (_, i) => {
        const ligne = new Array(n + 1).fill(0);
        for (let k = 0; k < n; k++) ligne[k] = A.reduce((s, r) => s + r[i] * r[k], 0);
        ligne[i] += 1e-9;
        ligne[n] = A.reduce((s, r, q) => s + r[i] * b[q], 0);
        return ligne;
    });
    for (let i = 0; i < n; i++) {
        let q = i;
        for (let k = i + 1; k < n; k++) if (Math.abs(M[k][i]) > Math.abs(M[q][i])) q = k;
        [M[i], M[q]] = [M[q], M[i]];
        if (Math.abs(M[i][i]) < 1e-12) return new Array(n).fill(0);
        for (let k = i + 1; k < n; k++) {
            const f = M[k][i] / M[i][i];
            for (let j = i; j <= n; j++) M[k][j] -= f * M[i][j];
        }
    }
    const x = new Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
        let s = M[i][n];
        for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    return x;
}

/* Mesure l'écart entre la cible demandée et le pied que le MODÈLE donne, sur le
 * domaine de marche, et en tire la correction. `rayon` : le demi-pas (mm). */
/* Le domaine couvre TOUT ce que la marche demande à un pied : le demi-pas autour
 * de sa station (`rayon`), la levée en l'air (+40 mm) et la descente que la
 * compensation de flexion lui impose en appui (jusqu'à −25 mm). Mesuré trop plat,
 * le résidu remontait à 0,32 mm sur les clés compensées. */
export function mesurerResidus(ctx, pattes, angles,
                               { rayon = 60, pas = 15, bas = -30, haut = 45, pasHauteur = 15 } = {}) {
    const base = new Map(angles);
    const bilan = [];
    const corriges = pattes.map((g, k) => {
        const p = ctx.pattes[k];
        const A = [], b = { x: [], y: [], z: [] };
        let avant = 0, compte = 0;
        for (let dx = -rayon; dx <= rayon; dx += pas) {
            for (let dy = -rayon; dy <= rayon; dy += pas) {
                if (Math.hypot(dx, dy) > rayon) continue;
                for (let dz = bas; dz <= haut; dz += pasHauteur) {
                    const cible = { x: g.neutre.x + dx, y: g.neutre.y + dy, z: g.neutre.z + dz };
                    const s = anglesServo(g, cible);
                    if (!s || margeButeesPatte(g, s) < 0) continue;
                    const m = new Map(base);
                    for (const a of g.art) m.set(a.id, s[a.nom]);
                    ctx.appliquer(m);
                    const reel = dansLePlan(ctx, ctx.pied(p));
                    A.push(BASE_CORR({ x: dx, y: dy, z: dz }));
                    b.x.push(reel.x - cible.x); b.y.push(reel.y - cible.y); b.z.push(reel.z - cible.z);
                    avant += (reel.x - cible.x) ** 2 + (reel.y - cible.y) ** 2 + (reel.z - cible.z) ** 2;
                    compte++;
                }
            }
        }
        ctx.appliquer(base);
        if (compte < 30) return g;
        const correction = { x: moindresCarres(A, b.x), y: moindresCarres(A, b.y), z: moindresCarres(A, b.z) };
        bilan.push({ patte: k + 1, poses: compte, avant: Math.sqrt(avant / compte) });
        return { ...g, correction };
    });
    return { pattes: corriges, bilan };
}

/* Angles de servo (degrés) pour poser le pied de `patte` en `cible`, ou null.
 * La correction mesurée est retirée de la cible : viser un peu à côté fait
 * arriver le pied au bon endroit. */
export function anglesServo(patte, cible) {
    let vise = cible;
    if (patte.correction) {
        const d = { x: cible.x - patte.neutre.x, y: cible.y - patte.neutre.y, z: cible.z - patte.neutre.z };
        const base = BASE_CORR(d);
        const ecart = (c) => base.reduce((s, v, i) => s + v * c[i], 0);
        vise = { x: cible.x - ecart(patte.correction.x),
                 y: cible.y - ecart(patte.correction.y),
                 z: cible.z - ecart(patte.correction.z) };
    }
    const g = anglesGeometriques(patte, vise);
    if (!g) return null;
    const out = {};
    for (const a of patte.art) out[a.nom] = a.sens * g[a.nom] * DEG + a.decalage;
    return out;
}

/* Position du pied d'après les angles de SERVO — l'inverse de `anglesServo`, et
 * la correction s'y applique dans l'autre sens : là où viser retire le résidu,
 * prédire l'ajoute. Les deux restent donc réciproques. */
export function piedServo(patte, servo) {
    const g = {};
    for (const a of patte.art) g[a.nom] = a.sens * (servo[a.nom] - a.decalage) * RAD;
    const F = piedGeometrique(patte, g);
    if (!patte.correction) return F;
    const d = { x: F.x - patte.neutre.x, y: F.y - patte.neutre.y, z: F.z - patte.neutre.z };
    const base = BASE_CORR(d);
    const ecart = (c) => base.reduce((s, v, i) => s + v * c[i], 0);
    return { x: F.x + ecart(patte.correction.x),
             y: F.y + ecart(patte.correction.y),
             z: F.z + ecart(patte.correction.z) };
}

/* Marge aux butées (degrés) de la patte dans cette pose : la plus petite des six
 * distances aux butées. Négative = butée dépassée. */
export function margeButeesPatte(patte, servo) {
    let marge = Infinity;
    for (const a of patte.art) {
        marge = Math.min(marge, servo[a.nom] - a.min, a.max - servo[a.nom]);
    }
    return marge;
}
