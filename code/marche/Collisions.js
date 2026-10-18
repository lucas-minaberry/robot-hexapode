/* Collisions.js — les pattes se touchent-elles ? Assez vite pour le dire à chaque pas.
 *
 * ── CE QU'ON MESURE ──────────────────────────────────────────────────────────
 * Le dégagement : la plus petite distance entre deux pièces de PATTES DIFFÉRENTES,
 * ou entre une patte et le châssis. Les pièces d'une même patte, elles, se touchent
 * par construction — elles sont assemblées l'une sur l'autre.
 *
 * ── LA FORME RETENUE : DEUX NIVEAUX PAR TRANCHE ──────────────────────────────
 * Le maillage exact (150 000 triangles) est le juge, mais il est trop lourd pour
 * une vérification à chaque clé. Chaque pièce est donc découpée en tranches, et
 * chaque tranche reçoit DEUX enveloppes, cuites hors ligne par pilote_bundle :
 *  · sa BOÎTE orientée — huit sommets, qui contient tout ;
 *  · son enveloppe CONVEXE exacte — 176 sommets en moyenne.
 * La boîte, qui englobe l'exacte, MINORE toujours la distance : on s'en sert pour
 * éliminer, et on ne repèse sur l'exacte que les rares couples qui peuvent changer
 * le verdict. Mesuré : la boîte seule annonçait 21 mm de moins que le maillage —
 * assez pour refuser toutes les démarches dès qu'on demandait 4 mm de jeu.
 *
 * ── UN DÉFAUT QUI A FAUSSÉ TOUTES LES MESURES ────────────────────────────────
 * Jusqu'au 17/09, la réduction du simplexe de GJK ne savait pas reconnaître une
 * origine ENFERMÉE dans le tétraèdre : deux formes qui se pénètrent rendaient une
 * distance POSITIVE, et la recherche tournait en rond sur le même sous-simplexe.
 * Sur des boîtes à huit sommets le défaut ne se voyait pas ; sur les enveloppes
 * exactes, un nuage de 300 points DANS son cube annonçait 1,4 mm. Toute distance
 * proche de zéro mesurée AVEC CE MODULE avant cette date est à refaire : un
 * chevauchement pouvait passer pour un dégagement. En revanche le banc mesure aussi
 * la distance entre SOMMETS de maillages, par nuages de points (⑤) : ce chemin-là
 * n'a jamais utilisé GJK, et ce qu'il a dit reste vrai.
 * La réduction se fait maintenant par la méthode de Johnson (voir plus bas).
 *
 * Pas de dépendance à three : ce module doit pouvoir être porté sur la Pi avec le
 * reste du moteur de marche. */

const RAD = Math.PI / 180;

/* ── Petite algèbre ────────────────────────────────────────────────────────── */
const sous = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const point = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norme = (a) => Math.sqrt(point(a, a));

/* Rotation d'un point autour d'un axe (point `p`, direction unitaire `u`). */
function tournerAutour(v, p, u, angle) {
    const c = Math.cos(angle), s = Math.sin(angle);
    const d = sous(v, p);
    const k = point(u, d) * (1 - c);
    return [
        p[0] + d[0] * c + (u[1] * d[2] - u[2] * d[1]) * s + u[0] * k,
        p[1] + d[1] * c + (u[2] * d[0] - u[0] * d[2]) * s + u[1] * k,
        p[2] + d[2] * c + (u[0] * d[1] - u[1] * d[0]) * s + u[2] * k,
    ];
}

/* ── Distance entre deux segments, puis entre deux capsules ────────────────── */
/* Segment [p, p+d] et [q, q+e] : la plus courte distance, cas dégénérés compris.
 * C'est le cœur du calcul — tout le reste n'est que de la géométrie autour. */
function distanceSegments(p1, q1, p2, q2) {
    const d1 = sous(q1, p1), d2 = sous(q2, p2), r = sous(p1, p2);
    const a = point(d1, d1), e = point(d2, d2), f = point(d2, r);
    const EPS = 1e-9;
    let s = 0, t = 0;
    if (a <= EPS && e <= EPS) return norme(r);
    if (a <= EPS) {
        t = Math.max(0, Math.min(1, f / e));
    } else {
        const c = point(d1, r);
        if (e <= EPS) {
            s = Math.max(0, Math.min(1, -c / a));
        } else {
            const b = point(d1, d2);
            const denom = a * e - b * b;
            s = denom > EPS ? Math.max(0, Math.min(1, (b * f - c * e) / denom)) : 0;
            t = (b * s + f) / e;
            if (t < 0) { t = 0; s = Math.max(0, Math.min(1, -c / a)); }
            else if (t > 1) { t = 1; s = Math.max(0, Math.min(1, (b - c) / a)); }
        }
    }
    const c1 = [p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s];
    const c2 = [p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t];
    return norme(sous(c1, c2));
}

export function distanceCapsules(A, B) {
    return distanceSegments(A.a, A.b, B.a, B.b) - A.r - B.r;
}

/* ── DISTANCE ENTRE DEUX ENVELOPPES CONVEXES (GJK) ───────────────────────────
 * Les capsules ont montré leurs limites : une pièce trapue — un servo de 40 × 20 ×
 * 35 mm — y prend un rayon de 25 mm, et deux servos distants de 30 mm paraissaient
 * enfoncés l'un dans l'autre de 44. Pour des formes pareilles, seule l'enveloppe
 * CONVEXE colle : elle est exacte sur une boîte, et découpée en tranches elle suit
 * aussi les pièces allongées et courbes.
 *
 * GJK travaille sur la différence de Minkowski A ⊖ B : la distance entre les deux
 * formes est celle de cette différence à l'origine. À chaque tour on ajoute le
 * point le plus avancé dans la direction de l'origine, et on garde le sous-simplexe
 * qui en est le plus proche. Quatre points suffisent : au-delà, l'origine est
 * dedans et les formes se touchent. */
const EPS_GJK = 1e-9;

/* Le point d'un nuage le plus avancé dans la direction `d`. */
function support(points, d) {
    let meilleur = 0, pire = -Infinity;
    for (let i = 0; i < points.length; i += 3) {
        const v = points[i] * d[0] + points[i + 1] * d[1] + points[i + 2] * d[2];
        if (v > pire) { pire = v; meilleur = i; }
    }
    return [points[meilleur], points[meilleur + 1], points[meilleur + 2]];
}

/* ── LE POINT DU SIMPLEXE LE PLUS PROCHE DE L'ORIGINE (JOHNSON) ──────────────
 * On cherche les coordonnées barycentriques λ qui minimisent |Σ λᵢ pᵢ| sous Σ λᵢ = 1,
 * et on ne garde que le sous-simplexe dont toutes les λ sont positives : c'est,
 * par construction, celui qui PORTE le point le plus proche. Un tétraèdre qui
 * enferme l'origine rend alors λ toutes positives et le point nul — donc zéro,
 * ce que la version « cas par cas » ne savait pas voir.
 *
 * L'ancienne version choisissait le meilleur sous-simplexe parmi les faces et les
 * arêtes. Quand ce choix laissait tomber le point qu'on venait d'ajouter, la
 * recherche revenait au simplexe précédent, redemandait le même appui, et tournait
 * en rond : mesuré, un nuage de 300 points DANS son cube annonçait 1,4 mm de
 * distance. Sur des boîtes à huit sommets le défaut ne se voyait pas. */
function moindresCarresBarycentriques(S) {
    /* ── LE SYSTÈME DE LAGRANGE, PAS LA SEULE MATRICE DE GRAM ────────────────
     * Minimiser |Σλᵢpᵢ|² sous Σλᵢ = 1 donne 2Gλ = μ1. On serait tenté d'en tirer
     * λ ∝ G⁻¹1 — mais G est SINGULIÈRE dès que le simplexe enferme l'origine :
     * quatre points de R³ sont toujours liés. C'est justement le cas qui nous
     * intéresse. On résout donc le système complet, bordé du multiplicateur μ,
     * qui, lui, reste inversible tant que le simplexe n'est pas plat. */
    const n = S.length;
    const M = [];
    for (let i = 0; i < n; i++) M.push([...S.map(q => 2 * point(S[i], q)), 1, 0]);
    M.push([...S.map(() => 1), 0, 1]);
    const t = n + 1;
    for (let c = 0; c < t; c++) {
        let pivot = c;
        for (let r = c + 1; r < t; r++) if (Math.abs(M[r][c]) > Math.abs(M[pivot][c])) pivot = r;
        if (Math.abs(M[pivot][c]) < 1e-12) return null;              /* simplexe plat */
        [M[c], M[pivot]] = [M[pivot], M[c]];
        for (let r = 0; r < t; r++) {
            if (r === c) continue;
            const f = M[r][c] / M[c][c];
            for (let k = c; k <= t; k++) M[r][k] -= f * M[c][k];
        }
    }
    return S.map((q, i) => M[i][t] / M[i][i]);
}

function plusProcheDeOrigine(W) {
    /* Du plus grand sous-ensemble au plus petit, et on s'arrête au PREMIER dont
     * toutes les λ sont positives : il est unique (Johnson), donc le chercher plus
     * loin ne fait que coûter. Mesuré sur un demi-cycle tout droit : 193 ms avec la
     * recherche complète, 124 avec l'arrêt au premier. */
    for (let masque = (1 << W.length) - 1; masque >= 1; masque--) {
        const S = [];
        for (let i = 0; i < W.length; i++) if (masque & (1 << i)) S.push(W[i]);
        const lambda = S.length === 1 ? [1] : moindresCarresBarycentriques(S);
        if (!lambda) continue;
        let dehors = false;
        for (const x of lambda) if (x < -1e-9) { dehors = true; break; }
        if (dehors) continue;
        const v = [0, 0, 0];
        for (let k = 0; k < S.length; k++) {
            v[0] += S[k][0] * lambda[k]; v[1] += S[k][1] * lambda[k]; v[2] += S[k][2] * lambda[k];
        }
        return { v, W: S };
    }
    /* Tout dégénéré : on garde le sommet le plus proche, faute de mieux. */
    let q = W[0];
    for (const x of W) if (point(x, x) < point(q, q)) q = x;
    return { v: q, W: [q] };
}

/* Distance entre deux nuages CONVEXES (leurs enveloppes). 0 si elles se touchent
 * ou se pénètrent — de combien, on ne le demande pas : ça ne passe pas, point. */
export function distanceConvexes(A, B, { tours = 48 } = {}) {
    const minkowski = (d) => {
        const a = support(A, d);
        const b = support(B, [-d[0], -d[1], -d[2]]);
        return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    };
    let W = [minkowski([1, 0, 0])];
    let v = W[0];
    for (let i = 0; i < tours; i++) {
        const n2 = point(v, v);
        if (n2 < EPS_GJK) return 0;                       /* l'origine est dedans */
        const w = minkowski([-v[0], -v[1], -v[2]]);
        /* Plus de progrès possible : la distance est |v|. */
        if (n2 - point(v, w) <= 1e-10 * n2) return Math.sqrt(n2);
        const r = plusProcheDeOrigine([...W, w]);
        v = r.v;
        W = r.W;
        if (W.length > 4) W = W.slice(-4);
    }
    return Math.sqrt(point(v, v));
}

/* ── DÉCOUPER UNE PIÈCE EN TRANCHES (hors ligne) ─────────────────────────────
 * Une seule enveloppe convexe par pièce comblerait ses creux — le pied est une
 * griffe courbe. On tranche donc la pièce le long de sa plus grande dimension, et
 * chaque tranche recevra SON enveloppe : ensemble, elles suivent la forme.
 *
 * Le nombre de tranches se choisit pour que chacune soit à peu près aussi longue
 * que le quart de la largeur — plus fin ne gagnerait rien.
 *
 * `points` : Float32Array de sommets (x, y, z), dans le repère de la pièce.
 * Rend un tableau de nuages, un par tranche. */
export function tranchesDeMaillage(points, { tranchesMax = 24, epaisseurMin = 4 } = {}) {
    const n = points.length / 3;
    if (!n) return [];
    const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < points.length; i += 3) {
        for (let k = 0; k < 3; k++) {
            min[k] = Math.min(min[k], points[i + k]);
            max[k] = Math.max(max[k], points[i + k]);
        }
    }
    const taille = [0, 1, 2].map(k => max[k] - min[k]);
    const axe = taille.indexOf(Math.max(...taille));
    const autres = [0, 1, 2].filter(k => k !== axe);
    const largeur = Math.max(taille[autres[0]], taille[autres[1]]);
    /* Une tranche courte serre mieux la forme — la capsule prend le rayon du plus
     * gros écart à son axe. On vise le quart de la largeur : deux à cinq capsules
     * par pièce laissaient une gaine trop grosse de plusieurs millimètres. */
    const tranches = Math.max(1, Math.min(tranchesMax,
        Math.round(taille[axe] / Math.max(epaisseurMin, largeur / 4))));
    const pas = taille[axe] / tranches;

    const paquets = Array.from({ length: tranches }, () => []);
    for (let i = 0; i < points.length; i += 3) {
        const u = (points[i + axe] - min[axe]) / pas;
        paquets[Math.max(0, Math.min(tranches - 1, Math.floor(u)))].push(i);
    }
    return paquets.filter(l => l.length).map(idx => {
        const t = new Float32Array(idx.length * 3);
        idx.forEach((i, n) => { t[n * 3] = points[i]; t[n * 3 + 1] = points[i + 1]; t[n * 3 + 2] = points[i + 2]; });
        return t;
    });
}

/* ── Le contrôle, à l'exécution ────────────────────────────────────────────── */
/* `squelette` : ce que pilote_bundle écrit — pièces (place au repos, gaines,
 * patte) et articulations (axe, pièces entraînées, angle au repos). */
/* ── CE QU'ON SURVEILLE, ET POURQUOI PAS LE RESTE ────────────────────────────
 * DEUX PATTES DIFFÉRENTES. C'est là, et là seulement, que le PAS décide : deux
 * pieds voisins se croisent quand l'un pousse et que l'autre revient, et rien dans
 * les butées ne l'empêche.
 *
 * Le châssis, lui, est hors sujet : ce qui l'approche, c'est l'épaule, dont le
 * débattement est borné par ses butées — que le moteur respecte déjà avec huit
 * degrés de marge. Mesuré : l'épaule passe à 0,1 mm du corps en fin de course, dans
 * la marche VALIDÉE comme dans celle du moteur. Le compter comme une collision
 * refuserait toutes les démarches sans rien protéger. `chassis: true` le remet
 * pour qui veut regarder.
 *
 * `seuilRepos` : deux pièces déjà à moins de ça au repos sont assemblées, pas en
 * passe de se heurter. */
/* ── LA BOÎTE FILTRE, L'ENVELOPPE EXACTE TRANCHE ─────────────────────────────
 * La boîte orientée ENGLOBE sa tranche : la distance qu'elle annonce MINORE
 * toujours la vraie. On peut donc l'utiliser pour éliminer — si elle promet déjà
 * plus que le meilleur connu, l'enveloppe exacte promettra plus encore — et ne
 * repeser que le reste. Mesuré au banc : la boîte seule annonçait 21,6 mm de moins
 * que le maillage ; ce tri ramène l'écart au dixième de millimètre pour un coût
 * qui reste celui de la boîte sur l'immense majorité des couples. */
function entrePieces(A, B, plafond) {
    let mini = plafond, boite = Infinity;
    for (let a = 0; a < A.enveloppes.length; a++) {
        for (let b = 0; b < B.enveloppes.length; b++) {
            /* Filtre à deux sous, TRANCHE PAR TRANCHE : une pièce en compte jusqu'à
             * dix, et dix fois dix distances exactes pour un couple qui ne se touche
             * pas, c'est cher. La sphère de la tranche le dit en six opérations. */
            const ca = A.centres[a], cb = B.centres[b];
            if (Math.sqrt((ca[0] - cb[0]) ** 2 + (ca[1] - cb[1]) ** 2 + (ca[2] - cb[2]) ** 2)
                - A.rayons[a] - B.rayons[b] >= mini) continue;
            const d = distanceConvexes(A.enveloppes[a], B.enveloppes[b]);
            if (d < boite) boite = d;
            if (d >= mini) continue;
            const fin = distanceConvexes(A.fine(a), B.fine(b));
            if (fin < mini) mini = fin;
        }
    }
    /* `boite` minore toujours : c'est ce qu'on annonce quand on n'a pas cherché
     * plus loin, faute d'intérêt. */
    return { distance: mini < plafond ? mini : Math.min(plafond, boite), atteint: mini < plafond };
}

const auRepos = (p) => ({ enveloppes: p.enveloppes, centres: p.centres, rayons: p.rayons,
                          fine: (k) => p.fines[k] ?? p.enveloppes[k] });

export function preparerCollisions(squelette,
                                   { seuilRepos = 1, chassis = false, portee = 260 } = {}) {
    const pieces = squelette.pieces.map((p, i) => {
        const enveloppes = (p.enveloppes ?? []).map(e => Float32Array.from(e));
        /* L'enveloppe EXACTE de chaque tranche, dans le même ordre : la boîte
         * filtre, celle-ci tranche. Voir `degagement`. */
        const fines = (p.fines ?? p.enveloppes ?? []).map(e => Float32Array.from(e));
        /* Sphère englobante de la pièce : le filtre à deux sous. */
        let centre = [0, 0, 0], n = 0;
        for (const e of enveloppes) {
            for (let k = 0; k < e.length; k += 3) {
                centre[0] += e[k]; centre[1] += e[k + 1]; centre[2] += e[k + 2];
                n++;
            }
        }
        centre = n ? centre.map(v => v / n) : [0, 0, 0];
        let rayon = 0;
        for (const e of enveloppes) {
            for (let k = 0; k < e.length; k += 3) {
                rayon = Math.max(rayon, norme(sous([e[k], e[k + 1], e[k + 2]], centre)));
            }
        }
        /* Sphère de chaque tranche : le filtre fin (voir entrePieces). */
        const centres = enveloppes.map((e) => {
            const c = [0, 0, 0];
            for (let k = 0; k < e.length; k += 3) { c[0] += e[k]; c[1] += e[k + 1]; c[2] += e[k + 2]; }
            return c.map(v => v / (e.length / 3));
        });
        const rayons = enveloppes.map((e, n) => {
            let r = 0;
            for (let k = 0; k < e.length; k += 3) {
                r = Math.max(r, norme(sous([e[k], e[k + 1], e[k + 2]], centres[n])));
            }
            return r;
        });
        return { i, patte: p.patte ?? -1, enveloppes, fines, centres, rayons, centre, rayon };
    });
    const parPatte = new Map();
    for (const a of squelette.articulations ?? []) {
        if (!parPatte.has(a.patte)) parPatte.set(a.patte, []);
        parPatte.get(a.patte).push(a);
    }
    /* ── CE QU IL NE FAUT PAS COMPTER ─────────────────────────────────────────
     * Un tibia frôle son propre servo d épaule à chaque pas, et une épaule touche
     * le châssis : ce sont des voisins d assemblage. On écarte donc les pièces
     * d une même patte, celles du châssis SUR lesquelles la patte est montée, et
     * tout couple déjà proche au repos. Reste ce qui compte : deux pattes
     * différentes, ou une patte et un châssis lointain. */
    const couples = [];
    for (let i = 0; i < pieces.length; i++) {
        for (let j = i + 1; j < pieces.length; j++) {
            if (!pieces[i].enveloppes.length || !pieces[j].enveloppes.length) continue;
            const pi = pieces[i].patte, pj = pieces[j].patte;
            if (pi >= 0 && pi === pj) continue;                          /* même patte */
            if (pi < 0 || pj < 0) {
                if (!chassis) continue;                                  /* voir l'en-tête */
                if ((squelette.pieces[i].montage ?? -1) === pj) continue;
                if ((squelette.pieces[j].montage ?? -1) === pi) continue;
            }
            const repos = entrePieces(auRepos(pieces[i]), auRepos(pieces[j]), Infinity).distance;
            if (repos < seuilRepos) continue;                            /* déjà assemblées */
            /* Deux pattes opposées ne se rejoindront jamais : la course du pas fait
             * 120 mm et leurs pièces sont à `portee` l'une de l'autre au repos. */
            if (repos > portee) continue;
            couples.push([i, j, repos]);
        }
    }
    return { pieces, parPatte, couples, squelette };
}

/* Place les gaines d'après les angles de servo, puis rend le plus petit
 * dégagement et le couple qui le tient. `arret` : on peut s'arrêter dès qu'on
 * passe sous un seuil — quand on veut juste savoir si ça passe. */
export function placerGaines(col, angles) {
    /* `tours` : les rotations subies, dans l'ordre. Les boîtes sont tournées tout
     * de suite — huit sommets — mais les enveloppes exactes en comptent cent
     * cinquante : on ne les tourne QUE pour les couples qui le méritent, et une
     * seule fois grâce au cache. */
    const placees = col.pieces.map(p => {
        const cache = [];
        const o = { centre: p.centre, rayon: p.rayon, enveloppes: p.enveloppes.map(e => e),
                    centres: p.centres.map(c => c), rayons: p.rayons, tours: [] };
        o.fine = (k) => {
            if (cache[k]) return cache[k];
            const e = p.fines[k] ?? p.enveloppes[k];
            if (!o.tours.length) return (cache[k] = e);
            const out = new Float32Array(e.length);
            for (let q = 0; q < e.length; q += 3) {
                let v = [e[q], e[q + 1], e[q + 2]];
                for (const t of o.tours) v = tournerAutour(v, t.p, t.u, t.angle);
                out[q] = v[0]; out[q + 1] = v[1]; out[q + 2] = v[2];
            }
            return (cache[k] = out);
        };
        return o;
    });
    for (const arts of col.parPatte.values()) {
        /* Une articulation tourne ce qu'elle porte, autour de son axe déjà tourné
         * par celles d'avant : épaule, puis cuisse, puis genou. */
        const chaine = [];
        for (const a of arts) {
            let p = a.point, u = a.direction;
            for (const avant of chaine) {
                p = tournerAutour(p, avant.p, avant.u, avant.angle);
                u = tournerAutour(u, [0, 0, 0], avant.u, avant.angle);
            }
            const deg = angles?.[a.servo];
            const angle = (Number.isFinite(deg) ? deg - a.repos : 0) * RAD;
            chaine.push({ p, u, angle, pieces: a.pieces });
        }
        for (const t of chaine) {
            for (const i of t.pieces) {
                const p = placees[i];
                p.tours.push(t);
                p.centre = tournerAutour(p.centre, t.p, t.u, t.angle);
                p.centres = p.centres.map(c => tournerAutour(c, t.p, t.u, t.angle));
                p.enveloppes = p.enveloppes.map((e) => {
                    const out = new Float32Array(e.length);
                    for (let k = 0; k < e.length; k += 3) {
                        const v = tournerAutour([e[k], e[k + 1], e[k + 2]], t.p, t.u, t.angle);
                        out[k] = v[0]; out[k + 1] = v[1]; out[k + 2] = v[2];
                    }
                    return out;
                });
            }
        }
    }
    return placees;
}

/* Le dégagement : la plus petite distance entre deux pièces surveillées. */
/* ── LE SEUIL SE MESURE, IL NE SE DÉCRÈTE PAS ────────────────────────────────
 * Exiger huit millimètres partout refusait tout : certaines pièces n'ont pas huit
 * millimètres de jeu AU REPOS — un tibia frôle son voisinage par construction. La
 * règle est donc : ne pas s'approcher plus qu'au repos, et jamais sous la marge
 * demandée quand le repos la laisse.
 *
 * Rend la marge restante (négative = trop près), la distance, et le couple. */
export function degagement(col, angles, { marge = 8, tolerance = 2, arret = -Infinity } = {}) {
    const placees = placerGaines(col, angles);
    let pire = Infinity, distance = Infinity, couple = null;
    for (const [i, j, repos] of col.couples) {
        const seuil = Math.min(marge, Math.max(0, repos - tolerance));
        const A = placees[i], B = placees[j];
        /* Filtre : deux pièces dont les sphères sont loin n'ont rien à se dire. */
        const d = norme(sous(A.centre, B.centre)) - A.rayon - B.rayon;
        /* ── ON NE CHERCHE LA VRAIE DISTANCE QUE LÀ OÙ ELLE DÉCIDE ────────────
         * Savoir s'il reste vingt ou soixante millimètres n'intéresse personne, et
         * l'enveloppe exacte coûte vingt fois la boîte. Quand l'appelant ne demande
         * qu'un verdict — `arret` —, on plafonne donc la recherche au seuil :
         * au-dessus, la boîte suffit puisqu'elle MINORE. */
        const plafond = Math.min(pire + seuil, Number.isFinite(arret) ? arret + seuil : Infinity);
        /* Filtre à deux sous : deux pièces dont les sphères sont plus éloignées que
         * ce plafond n'ont rien à dire. Le comparer au plafond et non au meilleur
         * connu est ce qui rend le verdict rapide : quand on ne cherche qu'un
         * verdict, le plafond vaut le seuil dès le premier couple. */
        if (d >= plafond) continue;
        const r = entrePieces(A, B, plafond);
        if (r.atteint && r.distance - seuil < pire) {
            pire = r.distance - seuil;
            distance = r.distance;
            couple = [i, j];
            if (pire <= arret) return { marge: pire, distance, couple };
        }
    }
    return { marge: pire, distance, couple };
}
