"""trajectoire.py — jouer une sequence comme une TRAJECTOIRE, a l'horloge.

── CE QUE FAIT L'INDUSTRIE ──────────────────────────────────────────────────
ros2_control (joint_trajectory_controller), les robots a servos Dynamixel,
LeRobot sur ces memes STS3215 : aucun n'attend qu'un servo « arrive » a un point
de passage pour lui donner le suivant. Tous font la meme chose :
  1. la trajectoire est connue d'avance, point par point ET dans le temps ;
  2. une boucle a cadence fixe l'echantillonne (interpolation) et envoie une
     consigne a tous les servos d'un coup ;
  3. la mesure sert a surveiller des TOLERANCES : si une articulation decroche
     de sa trajectoire, on ralentit ou on s'arrete. L'arrivee n'est attendue
     qu'aux points d'arret (tolerance d'arrivee).

── POURQUOI ON Y EST VENU ────────────────────────────────────────────────────
La marche compte 100 cles de 55 ms, et la Pi lit les positions toutes les
40 ms. « Enchainer a l'arrivee » ne pouvait decider qu'a 40, 80 ou 120 ms, et
chaque servo arrive avant les autres les attendait a l'arret. Rejoue sur la
vraie sequence (system/rejouer_sequence.py), part du temps ou un servo est a
l'arret alors que sa cle lui demande de bouger :
    a l'arrivee, lecture 40 ms ........ 17,7 %
    a l'arrivee, lecture 5 ms ......... 7,2 %
    a l'arrivee, cles 3x plus longues . 12,5 %
    a l'horloge ....................... 2,1 %
Sur le robot, la marche durait 2,3 a 2,9 fois sa duree prevue.

── COMMENT ──────────────────────────────────────────────────────────────────
· Chaque servo suit une courbe position(temps) qui passe EXACTEMENT par ses
  cles, aux instants prevus : Hermite cubique monotone (PCHIP). Vitesse
  continue, aucun depassement entre deux cles, vitesse nulle aux demi-tours.
· Le TEMPS est planifie avant de jouer : un passage plus rapide ou plus sec que
  le servo (`vitesse_max`, `accel_max`) est allonge juste assez. L'horloge n'a
  plus a freiner en pleine marche (voir etirer_instants).
· Toutes les `periode_s` (20 ms), UNE trame SYNC_WRITE pour tous les servos.
· Consigne = la courbe un peu EN AVANCE, vitesse = celle de la courbe (x1,3).
  Un STS3215 roule a la vitesse commandee et freine en approchant de son but
  (~10 000 pas/s2 releves). Viser l'instant present le ferait freiner a chaque
  consigne ; on vise donc au-dela de sa distance de freinage :
      avance = periode + vitesse / (2 x deceleration)
· L'avance ne franchit jamais un POINT D'ARRET (cle a atteindre en entier,
  derniere cle, cle avant un relachement) : la, l'horloge s'arrete et on attend
  l'arrivee mesuree.
· Surveillance : si un servo s'ecarte de sa courbe de plus de
  `tolerance_chemin` pas (200 ms de retard toleres), l'horloge RALENTIT, jusqu'a
  s'arreter. Un servo qui ne suit toujours pas au bout de 1,5 s est signale,
  ignore, et la sequence reprend.

── APPRENDRE CE QUI SE REPETE (ILC) ─────────────────────────────────────────
Le servo n'est jamais exactement sur sa courbe : il cede sous le poids, il
traine dans les phases rapides, il a du jeu. Mesure sur sept marches : ecart
RMS 16,4 pas, dont 82 % se REPETE d'une marche a l'autre (91 % sur les genoux
arriere, qui plient sous le poids du robot). Ce qui se repete se corrige.

C'est l'apprentissage iteratif des controleurs industriels pour les mouvements
repetes (Iterative Learning Control) : a chaque passage termine, l'ecart mesure
a chaque instant de la trajectoire est ajoute, avec un gain, a une table de
corrections ; le passage suivant vise d'autant plus loin. La table est lissee
(sans dephasage) et bornee. L'animation reste IDEALE : la correction est une
connaissance du robot, pas de l'animation, et elle vit sur la Pi, un fichier par
sequence (signature des cles et des limites de planification).
"""
import bisect
import hashlib
import math
import threading

PERIODE_DEFAUT_S = 0.020
# Acceleration du STS3215, MESUREE sur la marche jouee en l'air (registre de
# vitesse, echantillons de 40 ms — qui sous-estiment les pointes) : p90 5100,
# p99 10 000, p99.9 14 100, max 19 700 pas/s2.
#   · l'avance se calcule sur le p99 : mieux vaut viser un peu trop loin ;
#   · la limite d'etirement prend le max observe. Rejoue sur la vraie marche :
#     10 000 -> temps x2,01 · 20 000 -> x1,49 · 40 000 -> x1,22, avec dans tous
#     les cas moins de 1 % d'arrets en plein mouvement et une horloge qui ne
#     ralentit jamais. Reglable : parametre `trajectoire_accel_max`.
DECEL_DEFAUT = 10000.0             # pas/s2 : sert a calculer l'avance
ACCEL_MAX_DEFAUT = 20000.0         # pas/s2 : au-dela, le temps est etire
VITESSE_MIN = 20                   # jamais 0 : « au maximum » sur un STS
VITESSE_MAX_DEFAUT = 2000
MARGE_VITESSE = 1.3
AVANCE_MAX_S = 0.25
RATTRAPAGE_S = 0.25                # un servo en retard rattrape en ~ce temps
TOLERANCE_ARRIVEE_DEFAUT = 17      # pas
TOLERANCE_CHEMIN_DEFAUT = 60       # pas
# Au-dela, un ecart entre la consigne et la mesure n'est plus un retard de suivi
# (171 a 219 pas releves en marche) mais un servo bloque ou ignore : on ne
# repart pas de la consigne, on repart de la ou il est vraiment.
ANCRAGE_MAX_PAS = 400
FENETRE_RETARD = (-0.04, 0.20)     # s : avance et retard toleres sur la courbe
PAS_RETARD_S = 0.02
VARIATION_ECHELLE_PAR_S = 4.0      # l'horloge ne change pas d'allure d'un coup
PAUSE_MAX_S = 1.5
# ── L'HORLOGE NE DESCEND JAMAIS SOUS CETTE ALLURE ───────────────────────────
# Quand un servo s'ecarte de sa courbe, `_surveiller` ralentit le temps de tous
# — jusqu'a ZERO. L'arret coute alors plus que ce qu'il rattrape : le robot perd
# son elan, et le raccord suivant doit le relancer. Reglable en marche :
# parametre ROS `trajectoire_allure_min`.
ALLURE_MIN_DEFAUT = 0.0
MOUVEMENT_MIN = 3
FENETRE_IMMOBILE_S = 0.25
DELAI_ARRIVEE_S = 4.0

# ── Apprentissage ───────────────────────────────────────────────────────────
PAS_APPRENTISSAGE_S = 0.02         # une case par consigne
GAIN_APPRENTISSAGE = 0.5           # part de l'ecart corrigee a chaque passage
RETARD_APPRENTISSAGE_S = 0.0       # decalage mesure -> consigne (l'avance l'absorbe deja)
BORNE_APPRENTISSAGE = 60.0         # pas : jamais plus loin de la courbe ideale
LISSAGE_APPRENTISSAGE = 2          # cases de part et d'autre (moyenne sans dephasage)


def pente_imposee(m0, d0):
    """Borner une pente de depart imposee (Fritsch-Carlson).

    Une pente libre ferait DEPASSER la courbe : au raccord d'un demi-cycle, un
    pied passerait sous le sol. On garde donc la regle du schema — au plus trois
    fois la pente de corde, et jamais a contresens."""
    if d0 == 0.0 or m0 * d0 <= 0.0:
        return 0.0
    return math.copysign(min(abs(m0), 3.0 * abs(d0)), d0)


def pentes_monotones(ts, qs, m0=None):
    """Pentes de Fritsch-Carlson : la courbe ne depasse jamais ses points.
    Nulles aux extremites (depart et arrivee au repos) et aux extremums.

    ── SAUF QUAND ON NE PART PAS DU REPOS ───────────────────────────────────
    `m0` impose la pente du premier point. Il existe pour le RACCORD : quand une
    sequence en remplace une autre en pleine marche, le robot est lance, et une
    pente nulle lui demande de s'arreter puis de repartir. Mesure hors ligne
    (`npm run raccord-test`, moteur et convertisseur reels) : a l'instant du
    raccord la consigne d'epaule retombe a 2 a 3 % de sa vitesse de croisiere ;
    avec la vitesse conservee, 73 a 104 %. Sur servos simules, marche coupee a
    sept instants (`python system/test_trajectoire.py`) : le raccord coutait
    69 ms, il en coute 6, et l'ecart de suivi passe de 17,8 a 9,3 pas."""
    n = len(ts)
    m = [0.0] * n
    if n < 2:
        return m
    h = [ts[i + 1] - ts[i] for i in range(n - 1)]
    d = [(qs[i + 1] - qs[i]) / h[i] if h[i] > 0 else 0.0 for i in range(n - 1)]
    for i in range(1, n - 1):
        if d[i - 1] * d[i] > 0:
            w1 = 2 * h[i] + h[i - 1]
            w2 = h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    if m0:
        m[0] = pente_imposee(m0, d[0])
    return m


class Courbe:
    """Position d'un servo en fonction du temps, et sa vitesse."""

    def __init__(self, ts, qs, m0=None):
        self.ts, self.qs = list(ts), list(qs)
        self.ms = pentes_monotones(self.ts, self.qs, m0)

    def __call__(self, t):
        ts, qs = self.ts, self.qs
        if t <= ts[0]:
            return qs[0], 0.0
        if t >= ts[-1]:
            return qs[-1], 0.0
        i = bisect.bisect_right(ts, t) - 1
        h = ts[i + 1] - ts[i]
        s = (t - ts[i]) / h
        s2, s3 = s * s, s * s * s
        q0, q1 = qs[i], qs[i + 1]
        m0, m1 = self.ms[i] * h, self.ms[i + 1] * h
        q = ((2 * s3 - 3 * s2 + 1) * q0 + (s3 - 2 * s2 + s) * m0
             + (-2 * s3 + 3 * s2) * q1 + (s3 - s2) * m1)
        dq = ((6 * s2 - 6 * s) * q0 + (3 * s2 - 4 * s + 1) * m0
              + (-6 * s2 + 6 * s) * q1 + (3 * s2 - 2 * s) * m1) / h
        return q, dq

    def pics(self, i):
        """Vitesse et acceleration maximales (en valeur absolue) sur l'intervalle
        i, echantillonne en cinq points bords compris. L'acceleration d'une
        Hermite cubique est lineaire sur l'intervalle : ses extremes sont aux
        bords, la vitesse peut culminer au milieu."""
        h = self.ts[i + 1] - self.ts[i]
        if h <= 0:
            return 0.0, 0.0
        q0, q1 = self.qs[i], self.qs[i + 1]
        m0, m1 = self.ms[i] * h, self.ms[i + 1] * h
        v = a = 0.0
        for s in (0.0, 0.25, 0.5, 0.75, 1.0):
            s2 = s * s
            dq = ((6 * s2 - 6 * s) * q0 + (3 * s2 - 4 * s + 1) * m0
                  + (-6 * s2 + 6 * s) * q1 + (3 * s2 - 2 * s) * m1) / h
            d2q = ((12 * s - 6) * q0 + (6 * s - 4) * m0
                   + (-12 * s + 6) * q1 + (6 * s - 2) * m1) / (h * h)
            v = max(v, abs(dq))
            a = max(a, abs(d2q))
        return v, a


def etirer_instants(par_servo, t_debut, vitesse_max, accel_max, passes=8, detail=None,
                    pentes=None):
    """PLANIFIER le temps au lieu de freiner au dernier moment.

    Une trajectoire que le servo ne peut pas suivre — trop rapide, ou qui tourne
    trop sec pour son acceleration — le fait decrocher ; l'horloge ralentissait
    alors en pleine marche, et ce ralentissement se voyait comme une hesitation.
    Releve sur la vraie marche : l'horloge ralentie 37 % du temps, pour des
    courbes qui demandaient jusqu'a 2950 pas/s et 62 000 pas/s2.

    C'est la parametrisation temporelle de l'industrie (TOPP, en plus simple) :
    chaque intervalle entre deux instants de cle est allonge juste assez pour
    que TOUS les servos restent sous `vitesse_max` et `accel_max`. Les poses et
    leur ordre ne changent pas ; seuls les passages impossibles prennent le temps
    qu'il leur faut, et tout le reste garde son rythme.

    `par_servo` : id -> [(t, q)], instants tries. Rend une fonction
    t_prevu -> t_etire, monotone, et le facteur d'allongement global.

    `detail` : dictionnaire a remplir, facultatif. On y note CE QUI A BRIDE —
    quel servo, quel intervalle, la vitesse ou l'acceleration, et de combien.
    Sans ca on ne peut que deviner pourquoi un demi-cycle s'allonge, et deviner
    a coute une soiree entiere le 19/09."""
    instants = sorted({t for pts in par_servo.values() for (t, _q) in pts} | {t_debut})
    if len(instants) < 2 or vitesse_max <= 0 or accel_max <= 0:
        return (lambda t: t), 1.0
    nouveaux = list(instants)
    # Ce qui bride, releve a la PREMIERE passe : c'est celle qui voit la
    # trajectoire telle qu'on la demande, avant tout allongement.
    pire = None
    for passe in range(passes):
        carte = dict(zip(instants, nouveaux))
        facteurs = [1.0] * (len(instants) - 1)
        for sid, pts in par_servo.items():
            ts = [carte[t] for (t, _q) in pts]
            if len(ts) < 2:
                continue
            # La MEME pente de depart que la courbe qui sera jouee : sinon on
            # etirerait un premier intervalle plus dur qu'il ne sera.
            c = Courbe(ts, [q for (_t, q) in pts], (pentes or {}).get(sid))
            for i in range(len(ts) - 1):
                v, a = c.pics(i)
                kv = v / (0.95 * vitesse_max)
                ka = math.sqrt(a / (0.9 * accel_max))
                k = max(kv, ka)
                if k <= 1.0:
                    continue
                if passe == 0 and (pire is None or k > pire['facteur']):
                    pire = {'facteur': k, 'servo': sid, 'intervalle': i,
                            'cause': 'acceleration' if ka >= kv else 'vitesse',
                            'vitesse': v, 'accel': a, 'instant': pts[i][0]}
                # Cet intervalle de CE servo couvre un ou plusieurs intervalles
                # globaux : on les allonge tous du meme facteur.
                j0 = bisect.bisect_left(instants, pts[i][0])
                j1 = bisect.bisect_left(instants, pts[i + 1][0])
                for j in range(j0, j1):
                    facteurs[j] = max(facteurs[j], min(k, 3.0))
        if max(facteurs) <= 1.001:
            break
        suite = [nouveaux[0]]
        for j, f in enumerate(facteurs):
            suite.append(suite[-1] + (nouveaux[j + 1] - nouveaux[j]) * f)
        nouveaux = suite

    def etirer(t):
        if t <= instants[0]:
            return nouveaux[0] + (t - instants[0])
        if t >= instants[-1]:
            return nouveaux[-1] + (t - instants[-1])
        j = bisect.bisect_right(instants, t) - 1
        span = instants[j + 1] - instants[j]
        r = (t - instants[j]) / span if span > 0 else 0.0
        return nouveaux[j] + r * (nouveaux[j + 1] - nouveaux[j])

    duree = instants[-1] - instants[0]
    facteur = (nouveaux[-1] - nouveaux[0]) / duree if duree > 0 else 1.0
    if detail is not None:
        detail.clear()
        detail['facteur'] = facteur
        detail['intervalles'] = len(instants) - 1
        if pire:
            detail.update(pire)
    return etirer, facteur


class Apprentissage:
    """Table de corrections apprise, par servo, indexee sur le temps de trajectoire.

    Le temps compte depuis l'ORIGINE : l'instant planifie de la premiere cle. Le
    trajet qui y mene depend de la pose de depart, qui change d'un passage a
    l'autre ; on n'apprend donc rien avant."""

    def __init__(self, pas_s=PAS_APPRENTISSAGE_S, gain=GAIN_APPRENTISSAGE,
                 retard_s=RETARD_APPRENTISSAGE_S, borne=BORNE_APPRENTISSAGE,
                 lissage=LISSAGE_APPRENTISSAGE, signature=''):
        self.pas_s = float(pas_s)
        self.gain = float(gain)
        self.retard_s = float(retard_s)
        self.borne = float(borne)
        self.lissage = int(lissage)
        self.signature = signature
        self.essais = 0
        self.u = {}                # id -> {case: correction en pas}
        self.historique = []
        self.commencer(0.0)

    @staticmethod
    def calculer_signature(cles, **limites):
        """Meme sequence et memes limites de planification = meme trajectoire."""
        h = hashlib.sha1()
        for c in cles:
            h.update(repr((round(float(c.get('t') or 0.0), 4), bool(c.get('release')),
                           bool(c.get('wait_full')),
                           [(int(s), int(p)) for (s, p, _v) in c.get('points', [])])).encode())
        h.update(repr(sorted(limites.items())).encode())
        return h.hexdigest()[:16]

    def commencer(self, origine):
        self._origine = float(origine)
        self._somme = {}
        self._nombre = {}
        self._se = 0.0
        self._n = 0

    def correction(self, sid, tau):
        """(correction en pas, sa derivee en pas/s) a l'instant `tau`."""
        table = self.u.get(sid)
        x = (tau - self._origine) / self.pas_s
        if not table or x < 0:
            return 0.0, 0.0
        k = int(math.floor(x))
        a, b = table.get(k, 0.0), table.get(k + 1, 0.0)
        return a + (x - k) * (b - a), (b - a) / self.pas_s

    def mesurer(self, sid, tau, erreur):
        """`erreur` = position planifiee - position mesuree, a l'instant `tau`."""
        if tau < self._origine:
            return
        k = int(round((tau - self._origine) / self.pas_s))
        s = self._somme.setdefault(sid, {})
        n = self._nombre.setdefault(sid, {})
        s[k] = s.get(k, 0.0) + erreur
        n[k] = n.get(k, 0) + 1
        self._se += erreur * erreur
        self._n += 1

    def conclure(self):
        """Met la table a jour avec le passage qui s'acheve. None si rien mesure."""
        if not self._n:
            return None
        decal = int(round(self.retard_s / self.pas_s))
        for sid, somme in self._somme.items():
            nb = self._nombre[sid]
            ecart = {k: somme[k] / nb[k] for k in somme}
            ancien = self.u.get(sid, {})
            brut = {}
            for k in set(ancien) | {k - decal for k in ecart}:
                e = ecart.get(k + decal)
                brut[k] = ancien.get(k, 0.0) + (self.gain * e if e is not None else 0.0)
            table = {}
            for k in brut:
                if k < 0:
                    continue
                voisins = [brut[j] for j in range(k - self.lissage, k + self.lissage + 1)
                           if j in brut]
                table[k] = max(-self.borne, min(self.borne, sum(voisins) / len(voisins)))
            self.u[sid] = table
        self.essais += 1
        bilan = {
            'essai': self.essais,
            'rms_pas': math.sqrt(self._se / self._n),
            'correction_max_pas': max((abs(v) for t in self.u.values() for v in t.values()),
                                      default=0.0),
            'mesures': self._n,
        }
        self.historique.append(bilan)
        return bilan

    def exporter(self):
        return {
            'version': 1, 'signature': self.signature, 'pas_s': self.pas_s,
            'essais': self.essais, 'historique': self.historique[-50:],
            'u': {str(sid): {str(k): round(v, 2) for k, v in sorted(t.items())}
                  for sid, t in self.u.items()},
        }

    @classmethod
    def importer(cls, donnees, **reglages):
        app = cls(signature=donnees.get('signature', ''), **reglages)
        if abs(float(donnees.get('pas_s', app.pas_s)) - app.pas_s) > 1e-9:
            return app                     # autre grille : on repart de zero
        app.essais = int(donnees.get('essais', 0))
        app.historique = list(donnees.get('historique', []))
        app.u = {int(sid): {int(k): float(v) for k, v in t.items()}
                 for sid, t in donnees.get('u', {}).items()}
        return app


class ExecuteurTrajectoire:
    """Joue une sequence a l'horloge. Ne connait ni ROS ni le bus.

    `cycle(maintenant)` est appele toutes les `periode_s` par une boucle dediee ;
    `positions(mesures, maintenant)` a chaque lecture du bus. Les deux peuvent
    venir de fils differents : tout passe par un verrou."""

    def __init__(self, envoyer, relacher, journal=None, horloge=None):
        self._envoyer = envoyer
        self._relacher = relacher
        self._journal = journal or (lambda m: None)
        self._horloge = horloge or _horloge_defaut
        self._verrou = threading.RLock()
        self.periode_s = PERIODE_DEFAUT_S
        self.decel = DECEL_DEFAUT
        self.vitesse_min = VITESSE_MIN
        self.vitesse_max = VITESSE_MAX_DEFAUT
        self.tolerance = TOLERANCE_ARRIVEE_DEFAUT
        self.tolerance_chemin = TOLERANCE_CHEMIN_DEFAUT
        # ── CE QUI DOIT SURVIVRE A `_reinitialiser` ─────────────────────────
        # Comparer une sequence a la PRECEDENTE n'a de sens qu'en traversant la
        # remise a zero. Mis d'abord dans `_reinitialiser` par erreur : la
        # comparaison n'avait jamais lieu, `diag_doublon` restait a 0, et le
        # journal annoncait trente doublons a « 0,0 pas d ecart » — c'est-a-dire
        # rien du tout. D'ou le None : il veut dire PAS CALCULE, et zero veut
        # dire IDENTIQUE. Les confondre a coute une mesure.
        self._pose_precedente = None
        # Ce que la trajectoire remplacee jouait, releve par `demande()` : les
        # poses de ses cles, dans l'ordre ou on les compte, et combien etaient
        # passees. Sert au raccord PAR IDENTITE (voir `_identifier`).
        self._memoire_raccord = None
        self.diag_identite = None
        self._ordre_compte = []
        self.diag_avance = 0.0
        self.diag_passees = 0
        self.diag_recul = 0.0
        self.diag_doublon = None
        self.allure_min = ALLURE_MIN_DEFAUT
        self.delai_s = DELAI_ARRIVEE_S
        self.accel_max = ACCEL_MAX_DEFAUT
        self._reinitialiser()

    def _reinitialiser(self):
        self.cles = []
        self.en_cours = False
        self.total = 0
        self.confirmees = 0
        self.enchainees = 0
        self.stagnantes = 0
        self.bloquees = 0
        self.message = ''
        self.cycles = 0
        self._comptees = 0
        self._positions = {}
        self._segments = []
        self._n_segment = 0
        self._courbes = {}
        self._arrets = []
        self._p_arret = 0
        self._a_compter = []
        self._pc = 0
        self._t = 0.0
        self._t_mur = 0.0
        self._t0 = 0.0
        self._echelle = 1.0
        self._echelle_cible = 1.0
        self._histo = []
        self._attente = None
        self._t_arret = None
        self._ref_mvt = {}
        self._t_mouvement = 0.0
        self._t_pause = None
        self._ignores = set()
        self.etirement = 1.0
        self.bride = {}
        self._arret_libre = None
        # ── CE QUE LE SEGMENT A TROUVE EN ARRIVANT ─────────────────────────
        # L'ecart mesure-consigne a l'instant ou le segment commence, et cet
        # instant : `_surveiller` s'en sert pour ne pas prendre pour un
        # decrochage le retard de suivi dont il HERITE. Voir la methode.
        self._t_segment = 0.0
        self._ecart_herite = None
        # ── L'ALLURE DE L'HORLOGE, GARDEE ──────────────────────────────────
        # `_surveiller` ralentit le temps de TOUS quand le pire des servos
        # s'ecarte de sa courbe : plein regime sous `tolerance_chemin`, arret
        # complet au double. C'est le seul mecanisme du planificateur capable
        # de doubler la duree d'un intervalle, et rien ne le montrait. On garde
        # donc le pire de chaque sequence.
        self.echelle_min = 1.0
        self.echelle_min_cle = 0
        self.ecart_pire = 0
        self.ecart_pire_servo = None
        self._etirer = lambda t: t
        self._decalage = 0.0
        self.apprentissage = None
        self.bilan = None
        self.avance_servos = {}

    # ── Demarrage / arret ───────────────────────────────────────────────────
    def demarrer(self, cles, positions, tolerance=TOLERANCE_ARRIVEE_DEFAUT,
                 vitesse_min=VITESSE_MIN, vitesse_max=VITESSE_MAX_DEFAUT,
                 periode_s=PERIODE_DEFAUT_S, decel=DECEL_DEFAUT,
                 tolerance_chemin=TOLERANCE_CHEMIN_DEFAUT, timeout_s=DELAI_ARRIVEE_S,
                 accel_max=ACCEL_MAX_DEFAUT, apprentissage=None, avance_servos=None,
                 ancrage=None, vitesses=None, depuis=0.0,
                 allure_min=ALLURE_MIN_DEFAUT):
        """`cles` : [{points: [(id, position, vitesse)], release, wait_full,
        t}] — `t` = instant ou la cle doit etre atteinte, en s depuis le debut."""
        with self._verrou:
            self._reinitialiser()
            utiles = [c for c in (cles or []) if c.get('points') or c.get('release')]
            if not utiles:
                return (False, 'sequence vide')
            instants = [float(c.get('t') or 0.0) for c in utiles if not c.get('release')]
            if not instants or max(instants) <= 0.0:
                return (False, 'les cles ne portent pas leur instant (time_s) : '
                               'application a mettre a jour')
            self.cles = utiles
            self.total = len(utiles)
            self.tolerance = max(1, int(tolerance))
            self.vitesse_min = max(1, int(vitesse_min or 1))
            self.vitesse_max = max(self.vitesse_min, int(vitesse_max))
            self.periode_s = max(0.005, float(periode_s))
            self.decel = max(100.0, float(decel))
            self.tolerance_chemin = max(5, int(tolerance_chemin))
            self.allure_min = max(0.0, min(0.9, float(allure_min)))
            self.delai_s = max(0.5, float(timeout_s))
            self.accel_max = max(500.0, float(accel_max))
            # Avance par servo, en s, bornee : -50 ms (servo en avance) a +150 ms.
            self.avance_servos = {int(k): max(-0.05, min(0.15, float(v)))
                                  for k, v in (avance_servos or {}).items()}
            self._positions = {int(k): float(v) for k, v in (positions or {}).items()
                               if v is not None}
            seg = []
            for i, c in enumerate(utiles):
                if c.get('release'):
                    self._segments.append({'cles': seg, 'relache': i})
                    seg = []
                else:
                    seg.append(i)
            if seg or not self._segments:
                self._segments.append({'cles': seg, 'relache': None})
            self._t0 = self._horloge()
            self._t_mur = self._t0
            self.en_cours = True
            # ── ON NE REPART PAS DU REPOS QUAND ON EST LANCE ─────────────────
            # `ancrage` / `vitesses` : ou la trajectoire REMPLACEE disait qu'on
            # etait, et a quelle vitesse. Le driver les releve avant de
            # l'arreter. Sans eux (demarrage a froid, reprise apres un stop) on
            # part de la mesure et de l'arret, comme avant.
            # `depuis` en decalage NEGATIF : les cles qu'on nous donne sont
            # datees a partir d'une cle que la trajectoire a deja passee ; on
            # entre donc dans la suite la ou elle en est, et non a son debut.
            # ── EST-CE LE MEME DEMI-CYCLE QU'ON NOUS REDONNE ? ──────────────
            # Si la premiere pose de la sequence qu'on recoit est celle de la
            # PRECEDENTE, l'appelant nous renvoie le morceau qu'il vient
            # d'envoyer — le robot repart alors au debut du pas qu'il jouait.
            # C'est le defaut decrit par l'operateur les 19 et 20/09. On le
            # mesure au lieu de le supposer.
            premiere = {int(sid): float(q)
                        for (sid, q, _v) in (utiles[0].get('points') or [])}
            self.diag_doublon = None
            if self._pose_precedente and premiere:
                communs = [k for k in premiere if k in self._pose_precedente]
                if len(communs) >= 3:
                    self.diag_doublon = max(
                        abs(premiere[k] - self._pose_precedente[k]) for k in communs)
            self._pose_precedente = premiere or None
            decalage = -max(0.0, float(depuis or 0.0))
            identite = self._identifier(utiles, ancrage) if ancrage else None
            self._memoire_raccord = None
            self.diag_identite = identite
            if identite is not None:
                # On SAIT ou l'on est sur le chemin recu : la derniere cle passee
                # est sa cle `m` (ou celle d'avant sa premiere si m = -1). On y
                # entre donc a `t_m + depuis`, sans projection.
                m = identite[1]
                t_m = float(utiles[m].get('t') or 0.0) if m >= 0 else 0.0
                decalage = -(t_m + max(0.0, float(depuis or 0.0)))
            self._entrer_segment(0, 0.0, decalage=decalage,
                                 ancrage=self._ancrage_sur(ancrage),
                                 vitesses=vitesses, projeter=identite is None)
            if apprentissage is not None and self.en_cours:
                self.apprentissage = apprentissage
                apprentissage.commencer(self._a_compter[0] if self._a_compter else 0.0)
            return (True, '')

    def ajouter(self, cles):
        """Ajouter des cles A LA SUITE de celles qui jouent, sans rien replanifier.

        POURQUOI. `demarrer` REMPLACE : il repart de la position MESUREE, qui
        traine toujours de quelques pas, et doit rattraper cet ecart dans le
        premier intervalle. Mesure du 19/09 sur le robot, en alignant la vitesse
        du corps sur l'index de cle : a la cle 0 le corps ne fait plus que 45 %
        de sa vitesse, et cette cle occupe 18,7 % du temps de marche — alors que
        les cles 2 a 11 sont plates a 96-114 %. L'arret a chaque pas, que nous
        avons longtemps mis sur le compte de l'inertie, est ENTIEREMENT
        fabrique par ce remplacement. Il coute environ 13 % de la vitesse.

        Ici, rien n'est replanifie : le segment en cours va jusqu'a son terme,
        et les nouvelles cles forment un segment de plus. La replanification
        tombe alors a la FIN d'un demi-cycle — six pieds au sol, tout est lent,
        le retard a rattraper est minime.

        Rend (True, nombre de cles) ou (False, raison). Si plus rien ne joue,
        on REFUSE : c'est a l'appelant de repartir par `demarrer`, en sachant
        d'ou le robot part."""
        with self._verrou:
            if not self.en_cours:
                return (False, 'rien ne joue')
            utiles = [c for c in (cles or []) if c.get('points') or c.get('release')]
            if not utiles:
                return (False, 'sequence vide')
            instants = [float(c.get('t') or 0.0) for c in utiles if not c.get('release')]
            if not instants or max(instants) <= 0.0:
                return (False, 'les cles ne portent pas leur instant (time_s)')
            # ── CE QUI N'A PAS COMMENCE N'A PAS A ETRE JOUE ──────────────────
            # Empiler les demi-cycles les uns derriere les autres fige la
            # trajectoire loin devant : la consigne du manche attend que toute
            # la file soit jouee, et le robot tourne une seconde et demie trop
            # tard (constate le 19/09). Or ce qui est en reserve n'a pas
            # commence : on peut le jeter sans rien interrompre.
            # On ne garde donc que le segment EN COURS, et ce qu'on ajoute
            # devient la suite immediate. Le manche entre au demi-cycle suivant,
            # jamais plus tard, et rien n'est coupe en plein vol.
            if len(self._segments) > self._n_segment + 1:
                gardees = max((max(seg['cles']) if seg['cles'] else -1)
                              for seg in self._segments[:self._n_segment + 1])
                relache = max((seg['relache'] if seg['relache'] is not None else -1)
                              for seg in self._segments[:self._n_segment + 1])
                fin_gardee = max(gardees, relache)
                del self._segments[self._n_segment + 1:]
                del self.cles[fin_gardee + 1:]
                self.total = len(self.cles)
                self._arret_libre = None      # ce segment redevient le dernier

            # Les nouvelles cles continuent APRES les anciennes : leurs instants
            # sont relatifs au debut de leur propre sequence.
            fin = max((float(c.get('t') or 0.0) for c in self.cles
                       if not c.get('release')), default=0.0)
            depart = len(self.cles)
            for c in utiles:
                d = dict(c)
                if not d.get('release'):
                    d['t'] = float(d.get('t') or 0.0) + fin
                self.cles.append(d)
            self.total = len(self.cles)

            # ── CE QU'ON AJOUTE CONTINUE-T-IL, OU REVIENT-IL EN ARRIERE ? ────
            # Symptome rapporte le 19/09 : « le robot est sur le point de finir
            # un pas et revient brusquement au debut de celui-ci ». Cela ne peut
            # arriver que si le bloc ajoute commence pres de la pose de DEBUT du
            # demi-cycle en cours au lieu de sa FIN. La Pi a les deux : on
            # mesure, on le dit, et on cesse de deviner.
            seg_cour = self._segments[self._n_segment]['cles']
            if seg_cour and depart > 0:
                def pose(i):
                    return {int(sid): float(q) for (sid, q, _v) in self.cles[i]['points']}
                def ecart(a, b):
                    communs = set(a) & set(b)
                    return max((abs(a[k] - b[k]) for k in communs), default=0.0)
                debut, fin_cour = pose(seg_cour[0]), pose(depart - 1)
                neuf = pose(depart)
                d_debut, d_fin = ecart(neuf, debut), ecart(neuf, fin_cour)
                if d_debut < d_fin:
                    self._journal('AJOUT QUI RECULE : le bloc ajoute est a %d pas du '
                                  'DEBUT du demi-cycle en cours et a %d pas de sa fin'
                                  % (d_debut, d_fin))
                else:
                    self._journal('ajout continu : %d pas de la fin, %d du debut'
                                  % (d_fin, d_debut))

            seg = []
            for i in range(depart, len(self.cles)):
                if self.cles[i].get('release'):
                    self._segments.append({'cles': seg, 'relache': i})
                    seg = []
                else:
                    seg.append(i)
            if seg:
                self._segments.append({'cles': seg, 'relache': None})
            return (True, len(utiles))

    def arreter(self, release=False):
        with self._verrou:
            if not self.en_cours:
                return False
            self.en_cours = False
            if release:
                self._relacher()
            else:
                # S'arreter SUR la trajectoire, la ou elle en est — pas sur le
                # point vise en avance.
                pts = []
                for sid, courbe in self._courbes.items():
                    q, _ = courbe(self._t)
                    if self.apprentissage is not None:
                        q += self.apprentissage.correction(sid, self._t)[0]
                    mes = self._positions.get(sid, q)
                    v = abs(q - mes) / RATTRAPAGE_S
                    pts.append((sid, int(round(max(0, min(4095, q)))), self._borner(v)))
                if pts:
                    self._envoyer(pts)
            return True

    # ── Segments : entre deux relachements ──────────────────────────────────
    # ── LE RACCORD PAR IDENTITE ─────────────────────────────────────────────
    # La projection (`_avance_sur`) DEVINE ou l'on est sur le chemin recu en
    # cherchant le segment le plus proche de la pose courante. Au changement de
    # trepied, le chemin se replie sur lui-meme (les genoux s'inversent) : deux
    # segments y sont presque confondus, et la projection peut choisir le
    # mauvais. Mesure sur le robot le 27/09 quand la page renvoyait la fin du pas
    # en cours : corps arrete puis RECULANT (-9 mm/s) a chaque relance.
    # Or l'appelant ne fabrique pas un chemin quelconque : il nous redonne des
    # cles que nous avions DEJA, a l'identique (memes angles, memes pas). On les
    # reconnait donc, sans rien deviner : si la premiere cle recue est notre cle
    # k, sa cle j est notre cle k + j, et la derniere que nous avons passee est
    # sa cle m = passees - 1 - k. Rend (k, m), ou None si rien ne correspond
    # (on retombe alors sur la projection, comme avant).
    TOLERANCE_IDENTITE = 1.5          # pas : l'arrondi des positions, rien de plus

    def _identifier(self, utiles, ancrage):
        mem = self._memoire_raccord
        if not mem or not mem['poses'] or not utiles:
            return None
        premiere = {int(sid): float(q) for (sid, q, _v) in (utiles[0].get('points') or [])}
        if len(premiere) < 3:
            return None
        passees = mem['passees']
        candidats = []
        for k, pose in enumerate(mem['poses']):
            communs = [sid for sid in premiere if sid in pose]
            if len(communs) < len(premiere):
                continue
            if max(abs(premiere[sid] - pose[sid]) for sid in communs) <= self.TOLERANCE_IDENTITE:
                candidats.append(k)
        if not candidats:
            return None
        # Plusieurs poses identiques (un pas sur place) : la plus proche de la ou
        # l'on est — on ne saute ni en avant ni en arriere d'un pas entier.
        k = min(candidats, key=lambda c: abs(c - passees))
        # La deuxieme cle doit correspondre aussi : une seule pose commune peut
        # etre une coincidence (la station, par exemple).
        if len(utiles) >= 2 and k + 1 < len(mem['poses']):
            seconde = {int(sid): float(q) for (sid, q, _v) in (utiles[1].get('points') or [])}
            pose = mem['poses'][k + 1]
            if any(abs(seconde[sid] - pose.get(sid, 1e9)) > self.TOLERANCE_IDENTITE for sid in seconde):
                return None
        m = passees - 1 - k
        if m < -1 or m >= len(utiles):
            return None
        return (k, m)

    def _avance_sur(self, indices, ancrage, t_debut, regarder=5):
        """De combien sommes-nous DEJA avances sur le chemin qu'on nous donne ?

        L'appelant date ses cles a partir d'une pose qu'il croit courante ; le
        temps qu'elles arrivent, nous avons bouge. `ancrage` dit ou nous sommes
        vraiment. On projette donc cette pose sur la ligne brisee des premieres
        cles et on rend l'instant, sur LEUR horloge, qui lui correspond.

        Pourquoi seulement les premieres (`regarder`) : la marche est cyclique,
        et un chemin repasse pres de lui-meme. Chercher loin trouverait un
        sosie. On ne depasse jamais de plus de deux ou trois cles (mesure du
        20/09 : index 12 a 14 pour un morceau qui finit a 11), regarder cinq
        cles est large.

        Rend 0.0 s'il n'y a pas d'ancrage, ou si nous sommes avant la premiere
        cle — le cas normal, ou il n'y a rien a retrancher."""
        if not ancrage or not indices:
            return 0.0
        pris = indices[:max(2, regarder)]
        poses = []
        for i in pris:
            p = {int(sid): float(q) for (sid, q, _v) in self.cles[i]['points']}
            poses.append((float(self.cles[i].get('t') or 0.0), p))
        communs = [sid for sid in ancrage if all(sid in p for _t, p in poses)]
        if len(communs) < 3:
            return 0.0
        if len(poses) < 2:
            return 0.0
        # Le segment de la ligne brisee des CLES le plus proche, et ou l'on est
        # dessus. La ligne ne commence PAS a l'ancrage : c'est justement sa
        # position par rapport aux cles qu'on cherche.
        meilleur = None
        for k, ((ta, pa), (tb, pb)) in enumerate(zip(poses, poses[1:])):
            num = sum((ancrage[s] - pa[s]) * (pb[s] - pa[s]) for s in communs)
            den = sum((pb[s] - pa[s]) ** 2 for s in communs)
            u = 0.0 if den <= 1e-9 else num / den
            # Sur le PREMIER segment on laisse u devenir negatif : c'est ainsi
            # qu'on reconnait le cas normal, ou l'on est encore EN AMONT de la
            # premiere cle et ou il n'y a rien a retrancher. Ailleurs, on reste
            # dans le segment.
            u = min(1.0, u) if k == 0 else max(0.0, min(1.0, u))
            sur = {s: pa[s] + u * (pb[s] - pa[s]) for s in communs}
            e = sum((ancrage[s] - sur[s]) ** 2 for s in communs)
            if meilleur is None or e < meilleur[0]:
                meilleur = (e, ta + u * (tb - ta))
        # `avance` compte depuis l'instant de depart de l'appelant, pas depuis
        # le notre : c'est bien la difference qu'il faut retrancher.
        return max(0.0, meilleur[1] - t_debut) if meilleur else 0.0

    def _ancrage_sur(self, ancrage):
        """Ne garder de l'ancrage que ce qui est plausible.

        Repartir de la consigne suppose que le servo la suivait. S'il en est
        tres loin — bloque, ignore, ou la sequence remplacee etait d'une tout
        autre nature — on repart de la mesure pour ce servo-la. Le reste du
        raccord en profite quand meme."""
        if not ancrage:
            return None
        sur = {}
        for sid, q in ancrage.items():
            sid = int(sid)
            mes = self._positions.get(sid)
            if mes is None or abs(float(q) - mes) <= ANCRAGE_MAX_PAS:
                sur[sid] = float(q)
        return sur or None

    def demande(self):
        """Ou la trajectoire en cours DIT que le robot doit etre, et a quelle
        vitesse. Rend (positions, vitesses) ou (None, None).

        C'est ce qu'il faut donner au `demarrer` suivant pour que le raccord ne
        soit pas une remise a zero : la consigne envoyee aux servos reste alors
        continue, en position ET en vitesse. Repartir de la MESURE, comme on le
        faisait, fait sauter la consigne EN ARRIERE du retard de suivi — d'ou
        l'impression que le robot revient au debut de son pas.

        ── ET DEPUIS COMBIEN DE TEMPS ───────────────────────────────────────
        Le troisieme nombre est le temps ecoule depuis la derniere cle comptee.
        Il est indispensable, et l'oublier COUTE : l'appelant nous redonne la
        suite en la datant a partir de cette cle-la, alors que la trajectoire
        l'a passee depuis. Ancrer sur la consigne sans en tenir compte avance la
        POSITION sans avancer le TEMPS : le premier intervalle a moins de chemin
        a faire dans autant de temps, et le robot y ralentit. Mesure au banc
        (`python system/test_trajectoire.py`, sept instants de coupure) : ecart
        de suivi RMS 26 pas au lieu de 18, et 28 ms de plus sur la marche.
        Avec ce decalage, le raccord retombe sur ses pieds."""
        with self._verrou:
            if not self.en_cours or not self._courbes:
                return None, None, 0.0
            pos, vit = {}, {}
            for sid, courbe in self._courbes.items():
                q, dq = courbe(self._t)
                pos[sid] = q
                # L'horloge peut etre ralentie : la vitesse vraiment demandee
                # est celle de la courbe multipliee par son allure.
                vit[sid] = dq * self._echelle
            # Il est borne tout seul : passe l'intervalle, la cle suivante est
            # comptee et le compte repart. C'est donc au plus l'ecart entre deux
            # cles de la sequence en cours — une centaine de millisecondes en
            # marche. Pour une sequence qui n'est PAS une suite (se poser,
            # posture demandee en pleine marche), il avance son premier
            # intervalle d'autant : le robot est deja en mouvement, entrer un
            # peu en avance y est juste aussi.
            depuis = (self._t - self._a_compter[self._pc - 1]) if self._pc > 0 else 0.0
            self._memoire_raccord = {
                'poses': [{int(sid): float(q) for (sid, q, _v) in (self.cles[i].get('points') or [])}
                          for i in self._ordre_compte],
                'passees': self._pc,
            }
            return pos, vit, max(0.0, depuis)

    def _entrer_segment(self, n, t_debut, decalage=0.0, ancrage=None, vitesses=None,
                        projeter=True):
        while n < len(self._segments):
            seg = self._segments[n]
            if seg['cles']:
                break
            if seg['relache'] is not None:          # un relachement seul
                self._relacher()
                self.confirmees += 1
                self._comptees += 1
            n += 1
        else:
            self._finir()
            return
        self._n_segment = n
        # ── OU EN EST-ON DEJA SUR LE CHEMIN QU'ON NOUS DONNE ? ───────────────
        # `play_sequence` REMPLACE, et l'appelant date ses cles a partir d'une
        # pose qu'il croit courante. Entre sa decision et notre reception, nous
        # avons avance. Mesure du 20/09 sur 198 remplacements : au moment ou la
        # sequence arrive, l'index est a 12-14 trois fois sur quatre, alors que
        # le morceau de tete s'arrete a 11 — nous avons donc deja entame le pas
        # SUIVANT de une a trois cles, et on nous le redonne depuis son debut.
        # L'operateur le decrit exactement : « en plein milieu du pas 2, il
        # revient au debut du pas 2 ».
        #
        # Nous, nous savons ou nous sommes : c'est `ancrage`. On projette donc
        # cette pose sur le chemin des premieres cles, et on retire ce qui est
        # derriere nous. L'appelant n'a rien a calculer, et aucune latence ne
        # peut plus faire reculer le robot.
        # `depuis` (porte par `decalage`, negatif) et la projection mesurent la
        # MEME chose — de combien nous sommes deja avances — mais chacune dans
        # son domaine, et elles ne s'ajoutent pas :
        #   · avant la premiere cle, la projection ne peut rien dire (il n'y a
        #     pas encore de chemin) et `depuis` fait le travail ;
        #   · au-dela, la projection est exacte et remplace `depuis`.
        # Les cumuler decalait deux fois : mesure au banc, l'ecart de suivi
        # passait de 9,3 a 22,9 pas.
        avance = self._avance_sur(seg['cles'], ancrage, t_debut) if projeter else 0.0
        if avance > 0.0:
            decalage = max(0.0, decalage) - avance
        self.diag_avance = avance
        instant = {i: float(self.cles[i].get('t') or 0.0) + decalage for i in seg['cles']}
        # Ce qui tombe a l'instant de depart ou avant est DERRIERE nous : on le
        # retire du chemin — le coller sur l'ancrage ferait repartir la courbe
        # d'un saut. On garde toujours au moins la derniere cle du segment.
        passees = [i for i in seg['cles'] if instant[i] <= t_debut + 1e-3]
        cles_utiles = [i for i in seg['cles'] if instant[i] > t_debut + 1e-3]
        if not cles_utiles:
            cles_utiles, passees = seg['cles'][-1:], seg['cles'][:-1]
        # Elles sont COMPTEES quand meme : l'appelant raisonne sur le nombre de
        # cles qu'il a envoyees, et son seuil d'enchainement se decalerait
        # d'autant si notre index prenait du retard sur le sien.
        if passees:
            self.enchainees += len(passees)
            self._comptees += len(passees)
        self.diag_passees = len(passees)
        # ── EST-CE QU'ON RECULE ? ────────────────────────────────────────────
        # On le dit ici, pas ailleurs : la premiere cle gardee est-elle DERRIERE
        # nous ? « Derriere » veut dire que pour y aller il faut rebrousser
        # chemin par rapport a la suite du trajet. On compare donc le pas
        # ancrage -> cle1 au pas cle1 -> cle2, servo par servo.
        self.diag_recul = 0.0
        if ancrage and len(cles_utiles) >= 2:
            a1 = {int(sid): float(q) for (sid, q, _v) in self.cles[cles_utiles[0]]['points']}
            a2 = {int(sid): float(q) for (sid, q, _v) in self.cles[cles_utiles[1]]['points']}
            recul = 0.0
            for sid, q1 in a1.items():
                q0, q2 = ancrage.get(sid), a2.get(sid)
                if q0 is None or q2 is None:
                    continue
                # Les deux pas en sens contraire : on recule de |q1 - q0|.
                if (q1 - q0) * (q2 - q1) < 0:
                    recul = max(recul, abs(q1 - q0))
            self.diag_recul = recul
        par_servo = {}
        for i in cles_utiles:
            for (sid, q, _v) in self.cles[i]['points']:
                par_servo.setdefault(int(sid), []).append((instant[i], float(q)))
        points = {}
        for sid, pts in par_servo.items():
            pts.sort(key=lambda x: x[0])            # stable : l'ordre d'emission
            net = []
            # ── D'OU L'ON PART : LA CONSIGNE, PAS LA MESURE ──────────────────
            # En DEMARRANT, on ne sait rien d'autre que la position mesuree.
            # Mais en CONTINUANT une trajectoire, on sait ou elle disait qu'on
            # devait etre — et c'est de la qu'il faut repartir. Repartir de la
            # mesure, qui traine toujours de quelques pas, oblige a rattraper
            # cet ecart dans le premier intervalle : le planificateur depasse
            # ses limites et etire tout le segment. Mesure sur le robot le
            # 19/09 : a la premiere cle d'un raccord le corps tombe a 45 % de
            # sa vitesse, et cette cle occupe 18,7 % du temps de marche.
            # Le retard du servo n'est pas ignore pour autant : `cycle` le
            # rattrape servo par servo (RATTRAPAGE_S), sans ralentir les autres.
            q0 = (ancrage or {}).get(sid, self._positions.get(sid))
            if q0 is not None and pts[0][0] > t_debut + 1e-3:
                net.append((t_debut, q0))           # on part de LA OU ON EST
            for (t, q) in pts:
                if net and t <= net[-1][0] + 1e-4:
                    net[-1] = (net[-1][0], q)       # meme instant : la derniere gagne
                else:
                    net.append((t, q))
            points[sid] = net
        # Le temps est PLANIFIE avant de jouer : les passages que les servos ne
        # peuvent pas suivre sont allonges (voir etirer_instants).
        # ── A QUELLE VITESSE ON Y ENTRE ──────────────────────────────────────
        # Zero en demarrant du repos ; la vitesse que la trajectoire precedente
        # demandait quand on la remplace en pleine marche. Seul le premier point
        # la porte : la suite est libre, comme avant.
        pentes = {sid: v for sid, v in (vitesses or {}).items()
                  if sid in points and v}
        bride = {}
        etirer, facteur = etirer_instants(points, t_debut, self.vitesse_max,
                                          self.accel_max, detail=bride, pentes=pentes)
        # On garde le pourquoi, pas seulement le combien : le driver le
        # journalise avec le reste du bilan du segment.
        self.bride = bride
        self._etirer = etirer
        self._decalage = decalage
        self.etirement = max(self.etirement, facteur)
        self._courbes = {sid: Courbe([etirer(t) for (t, _q) in net],
                                     [q for (_t, q) in net], pentes.get(sid))
                         for sid, net in points.items()}
        instant = {i: etirer(t) for i, t in instant.items() if i in set(cles_utiles)}
        fin = max(instant.values())
        arrets = {instant[i] for i in cles_utiles if self.cles[i].get('wait_full')}
        arrets.add(fin)
        # ── UNE FIN DE SEGMENT N'EST PAS UNE ARRIVEE ─────────────────────────
        # `fin` doit rester dans les arrets : c'est lui qui borne l'horloge au
        # bout des courbes. Mais SI UN AUTRE SEGMENT SUIT et que celui-ci ne se
        # termine pas par un relachement, il n'y a rien a confirmer — la
        # trajectoire continue. Attendre l'arrivee des dix-huit servos a 17 pas
        # pres y fabrique une pause franche a chaque demi-cycle : c'est ce qu'a
        # senti l'operateur le 19/09 des que l'ajout a fonctionne. Le retard
        # d'un servo reste traite, mais par `_surveiller`, qui ralentit
        # l'horloge au lieu de l'arreter.
        self._arret_libre = (fin if (n + 1 < len(self._segments)
                                     and seg['relache'] is None) else None)
        self._arrets = sorted(arrets)
        self._p_arret = 0
        self._a_compter = sorted(instant.values())
        self._ordre_compte = sorted(instant, key=lambda i: instant[i])
        self._pc = 0
        self._t = t_debut
        self._t_segment = t_debut
        self._ecart_herite = None          # releve a la premiere mesure recue
        self._t_arret = None
        self._histo = []
        self._echelle = 1.0
        self._echelle_cible = 1.0

    def _finir(self):
        self.en_cours = False
        self._attente = None
        # Seul un passage ALLE AU BOUT enseigne quelque chose : un Stop laisse
        # une mesure partielle, qu'on ne melange pas a la table.
        if self.apprentissage is not None:
            self.bilan = self.apprentissage.conclure()

    # ── Boucle a cadence fixe ───────────────────────────────────────────────
    def cycle(self, maintenant=None):
        with self._verrou:
            if not self.en_cours:
                return False
            maintenant = self._horloge() if maintenant is None else maintenant
            dt = max(0.0, min(0.1, maintenant - self._t_mur))
            self._t_mur = maintenant
            pas = VARIATION_ECHELLE_PAR_S * dt
            self._echelle += max(-pas, min(pas, self._echelle_cible - self._echelle))
            if self._echelle < self.echelle_min:
                self.echelle_min = self._echelle
                self.echelle_min_cle = self._comptees
            t_arret = self._arrets[self._p_arret]
            self._t = min(t_arret, self._t + dt * self._echelle)
            if self._t >= t_arret - 1e-9 and self._t_arret is None:
                self._t_arret = maintenant
                self._t_mouvement = maintenant
                self._ref_mvt = {}
            self._histo.append((maintenant, self._t))
            if len(self._histo) > 200:
                del self._histo[:100]
            while (self._pc < len(self._a_compter)
                   and self._a_compter[self._pc] < t_arret - 1e-9
                   and self._a_compter[self._pc] <= self._t):
                self._pc += 1
                self.enchainees += 1
                self._comptees += 1

            points = []
            app = self.apprentissage
            for sid, courbe in self._courbes.items():
                # AVANCE PROPRE A CE SERVO : un servo qui traine plus que les autres
                # vise d'autant plus tot sur SA courbe (calibration mesuree,
                # outils/analyse_marche/retard_servos.py). Les pieds d'un trepied
                # se posent alors ensemble. Jamais au-dela d'un point d'arret.
                decal = self.avance_servos.get(sid, 0.0)
                _q, dq = courbe(min(t_arret, self._t + decal * self._echelle))
                # La vitesse reste celle de la courbe IDEALE. Y ajouter la derivee de
                # la table de corrections (en marches de 20 ms) faisait sauter la
                # vitesse commandee d'une case a l'autre, et l'apprentissage
                # divergeait (23 -> 34 pas en six passages simules). Le rattrapage
                # sur la position mesuree suffit a suivre la correction.
                v_ref = abs(dq) * self._echelle
                avance = min(AVANCE_MAX_S, self.periode_s + v_ref / (2.0 * self.decel))
                t_vise = min(t_arret, self._t + (avance + decal) * self._echelle)
                but, _ = courbe(t_vise)
                if app is not None:
                    but += app.correction(sid, t_vise)[0]
                v = v_ref * MARGE_VITESSE
                mes = self._positions.get(sid)
                if mes is not None:
                    v = max(v, abs(but - mes) / RATTRAPAGE_S)
                points.append((sid, int(round(max(0, min(4095, but)))), self._borner(v)))
            if points:
                self._envoyer(points)
            self.cycles += 1
            return True

    def _borner(self, v):
        return int(max(self.vitesse_min, min(self.vitesse_max, round(v))))

    # ── Mesures ─────────────────────────────────────────────────────────────
    def positions(self, mesures, maintenant=None):
        """`mesures` : id -> position. Rend True si la sequence a avance."""
        with self._verrou:
            maintenant = self._horloge() if maintenant is None else maintenant
            for sid, p in (mesures or {}).items():
                if p is None:
                    continue
                sid = int(sid)
                self._positions[sid] = float(p)
                ref = self._ref_mvt.get(sid)
                if ref is None:
                    self._ref_mvt[sid] = p
                elif abs(p - ref) >= MOUVEMENT_MIN:
                    self._ref_mvt[sid] = p
                    self._t_mouvement = maintenant
            if not self.en_cours:
                return False
            avant = self._comptees
            t_mes = self._temps_a(maintenant)
            if self.apprentissage is not None:
                for sid, p in (mesures or {}).items():
                    courbe = self._courbes.get(int(sid))
                    if courbe is not None and p is not None:
                        self.apprentissage.mesurer(int(sid), t_mes, courbe(t_mes)[0] - float(p))
            if self._t_arret is not None:
                self._juger_arret(maintenant)
            else:
                self._surveiller(t_mes, maintenant)
            return self._comptees != avant or not self.en_cours

    def _temps_a(self, mur):
        """Temps de trajectoire a l'instant `mur` (la mesure a un age)."""
        h = self._histo
        if not h or mur >= h[-1][0]:
            return self._t
        for j in range(len(h) - 1, -1, -1):
            if h[j][0] <= mur:
                return h[j][1]
        return h[0][1]

    def _surveiller(self, t_mes, maintenant):
        """Ralentir l'horloge quand un servo quitte sa courbe.

        ── LE RETARD DONT ON HERITE N'EST PAS UN DECROCHAGE ─────────────────
        L'ecart est cherche dans une fenetre de retard (jusqu'a 200 ms) : un
        servo qui suit sa courbe en la trainant y est reconnu, et l'horloge ne
        bronche pas. Ce mecanisme ne peut PAS fonctionner dans les 200 premieres
        millisecondes d'un segment : avant son premier instant la courbe est
        plate, il n'y a pas de passe ou retrouver la mesure, et tout le retard de
        suivi apparait d'un coup comme un ecart.

        Or un raccord de demi-cycle herite justement de ce retard — 171 a 219
        pas releves sur le robot le 19/09, pour une tolerance de 60 : l'horloge
        tombait a zero a la cle 0, et c'est l'arret a chaque pas.

        On escompte donc l'ecart TROUVE EN ARRIVANT, et seulement le temps que
        la courbe se fasse un passe : la remise est pleine a l'entree du segment,
        nulle au bout de la fenetre. Ce qui s'AGGRAVE, lui, compte tout de suite
        — un servo qui decroche vraiment est vu comme avant."""
        decalages = [FENETRE_RETARD[0] + k * PAS_RETARD_S for k in
                     range(int(round((FENETRE_RETARD[1] - FENETRE_RETARD[0]) / PAS_RETARD_S)) + 1)]
        ecarts = {}
        for sid, courbe in self._courbes.items():
            if sid in self._ignores:
                continue
            mes = self._positions.get(sid)
            if mes is None:
                continue
            ecarts[sid] = min(abs(mes - courbe(t_mes - d)[0]) for d in decalages)
        if not ecarts:
            self._echelle_cible = 1.0
            return
        if self._ecart_herite is None:
            self._ecart_herite = dict(ecarts)
        # Le pire ecart BRUT reste journalise : c'est lui qui dit la verite sur
        # le suivi, meme quand l'horloge n'en tient pas compte.
        brut = max(ecarts.items(), key=lambda kv: kv[1])
        if brut[1] > self.ecart_pire:
            self.ecart_pire = int(brut[1])
            self.ecart_pire_servo = brut[0]
        remise = max(0.0, 1.0 - (t_mes - self._t_segment) / FENETRE_RETARD[1])
        pire = None
        for sid, e in ecarts.items():
            e = max(0.0, e - remise * self._ecart_herite.get(sid, 0.0))
            if pire is None or e > pire[1]:
                pire = (sid, e)
        self._attente = (pire[0], int(pire[1]))
        tol = float(self.tolerance_chemin)
        e = pire[1]
        if e <= tol:
            cible = 1.0
        elif e >= 2 * tol:
            cible = 0.0
        else:
            cible = 1.0 - (e - tol) / tol
        # ── ON RALENTIT, ON NE S'ARRETE PAS ──────────────────────────────────
        # L'idee est de l'operateur, et elle est juste : s'arreter pour attendre
        # le dernier servo coute PLUS que ce que ca rattrape. Le robot perd son
        # elan, et le raccord suivant doit le relancer — la depense qu'on passe
        # la journee a chasser. Ralentir de moitie sur quelques cles laisse le
        # servo revenir sans rien arreter.
        # Le plancher ne touche QUE l'horloge. La detection d'un servo vraiment
        # bloque, elle, continue de compter sur la cible NON plancheee : sans
        # cela un servo coince ne serait plus jamais signale, et la trajectoire
        # s'en irait sans lui.
        self._echelle_cible = max(self.allure_min, cible)
        if cible > 0.0:
            self._t_pause = None
        elif self._t_pause is None:
            self._t_pause = maintenant
        elif maintenant - self._t_pause >= PAUSE_MAX_S:
            self._ignores.add(pire[0])
            self.bloquees += 1
            self._noter('servo %d ne suit pas sa trajectoire (%d pas d ecart) : ignore, '
                        'la sequence reprend' % (pire[0], e))
            self._t_pause = None
            self._echelle_cible = 1.0

    def _juger_arret(self, maintenant):
        t_h = self._arrets[self._p_arret]
        if self._arret_libre is not None and abs(t_h - self._arret_libre) < 1e-9:
            return self._passer_arret('enchainees')      # on continue, rien a confirmer
        connus = 0
        pire = None
        for sid, courbe in self._courbes.items():
            mes = self._positions.get(sid)
            if mes is None:
                continue
            connus += 1
            e = abs(mes - courbe(t_h)[0])
            if e > self.tolerance and (pire is None or e > pire[1]):
                pire = (sid, e)
        ecoule = maintenant - self._t_arret
        if connus and pire is None:
            return self._passer_arret('confirmees')
        if pire is not None:
            self._attente = (pire[0], int(pire[1]))
        if (connus and ecoule >= FENETRE_IMMOBILE_S
                and maintenant - self._t_mouvement >= FENETRE_IMMOBILE_S):
            self._noter('servo %d arrete a %d pas de sa cle (erreur statique : voir I)'
                        % (pire[0], pire[1]))
            return self._passer_arret('stagnantes')
        if ecoule >= self.delai_s:
            self._noter('servo %d hors delai (%d pas restants)' % pire if pire
                        else 'aucune position mesuree : impossible de confirmer l arrivee')
            return self._passer_arret('bloquees')
        return False

    def _passer_arret(self, compteur):
        t_h = self._arrets[self._p_arret]
        n = 0
        while self._pc < len(self._a_compter) and self._a_compter[self._pc] <= t_h + 1e-9:
            self._pc += 1
            n += 1
        setattr(self, compteur, getattr(self, compteur) + n)
        self._comptees += n
        self._p_arret += 1
        self._t_arret = None
        if self._p_arret < len(self._arrets):
            return True
        seg = self._segments[self._n_segment]
        if seg['relache'] is not None:
            self._relacher()
            self.confirmees += 1
            self._comptees += 1
            t_prevu = float(self.cles[seg['relache']].get('t') or 0.0)
            t_rel = max(t_h, self._etirer(t_prevu + self._decalage))
            self._entrer_segment(self._n_segment + 1, t_rel, t_rel - t_prevu)
        else:
            # ── LE SEGMENT SUIVANT REPREND OU CELUI-CI S'ARRETE ──────────────
            # `t_h` est un instant ETIRE ; les cles, elles, portent leur instant
            # d'origine. Reutiliser l'ancien decalage collait toutes les cles du
            # segment suivant a `t_debut` des que le precedent avait ete etire —
            # un segment de duree nulle, donc un etirement enorme (x7,4 mesure).
            # On recalcule donc le decalage comme le fait la branche du
            # relachement : de quoi poser la derniere cle jouee exactement sur
            # l'instant ou l'on est.
            # Sans cle ajoutee ce chemin finit la sequence, et le decalage n'a
            # aucun effet ; il ne sert qu'aux segments AJOUTES en cours de route.
            seg_fini = self._segments[self._n_segment]['cles']
            t_fin = max((float(self.cles[i].get('t') or 0.0) for i in seg_fini),
                        default=0.0)
            # La trajectoire CONTINUE : on l'ancre sur elle-meme, la ou elle
            # vient de s'arreter, et non sur la mesure.
            ancrage = {sid: courbe(t_h)[0] for sid, courbe in self._courbes.items()}
            self._entrer_segment(self._n_segment + 1, t_h, t_h - t_fin, ancrage)
        return True

    def _noter(self, msg):
        self.message = msg
        self._journal(msg)

    # ── Etat, pour le topic de progression ─────────────────────────────────
    @property
    def index(self):
        return min(self._comptees, self.total)

    def etat(self):
        with self._verrou:
            return {
                'running': self.en_cours,
                'index': self.index,
                'total': self.total,
                'confirmed': self.confirmees,
                'blended': self.enchainees,
                'stalled': self.stagnantes,
                'blocked': self.bloquees,
                'waiting_id': self._attente[0] if self._attente else 0,
                'waiting_error': int(self._attente[1]) if self._attente else 0,
                'elapsed_ms': int((self._horloge() - self._t0) * 1000),
                'message': self.message,
            }


def _horloge_defaut():
    import time
    return time.monotonic()
