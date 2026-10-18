"""Sécurités de la marche au manche — la décision, sans ROS ni matériel.

Trois organes, chacun une petite machine à états qui ne fait que DÉCIDER ; le
driver exécute (arrêt de séquence, couple relâché) et le dit. Séparés du nœud
pour s'éprouver sans robot : `python system/test_securite.py`.

CHIEN DE GARDE (feuille de route 37). L'application de pilotage envoie un
battement dix fois par seconde tant qu'elle pilote en réel (`pilote_vivant`,
vrai). Si les battements cessent — téléphone verrouillé, page fermée, WiFi
perdu — la Pi arrête la séquence en cours au bout de `delai_s` au lieu de jouer
jusqu'au bout les ~3 s de marche déjà téléversées. Un battement « faux » (la page
quitte le mode réel proprement) désarme. Le logiciel, qui n'envoie pas de
battement, n'arme jamais le chien : ses séquences ne sont pas concernées.

BASCULEMENT (feuille de route 44). D'après l'inclinaison TOTALE de l'IMU (service
caméra, étalonnée à plat) : au-delà de `seuil_deg` tenu `tenue_s`, on arrête la
séquence ; au-delà de `renversement_deg` tenu `renversement_s`, le robot est
tombé — on relâche le couple plutôt que de laisser les servos forcer dans le
vide. Une mesure trop vieille (service caméra arrêté) ne décide rien.

COUPURE BATTERIE DIFFÉRÉE. À 20 %, le driver coupe le rail des pattes : le robot
s'affaisse. L'application, elle, voit la batterie critique et POSE le robot —
mais la coupure tombait avant la fin. Tant qu'une séquence joue, la coupure
attend donc, au plus `grace_s`.
"""


class ChienDeGarde:
    def __init__(self, delai_s=0.3):
        self.delai_s = float(delai_s)
        self.arme = False
        self.dernier = None

    def battement(self, vivant, t):
        if vivant:
            self.arme = True
            self.dernier = t
        else:
            self.arme = False
            self.dernier = None

    def silence(self, t):
        """Secondes depuis le dernier battement (None si désarmé)."""
        return None if not self.arme or self.dernier is None else t - self.dernier

    def doit_arreter(self, t, sequence_en_cours):
        """Vrai UNE fois, quand le silence dépasse le délai pendant qu'une
        séquence joue. Le chien se désarme alors : il ne se réarme qu'au
        battement suivant — pas d'arrêt en rafale."""
        s = self.silence(t)
        if s is None or s <= self.delai_s:
            return False
        self.arme = False
        return bool(sequence_en_cours)


class Basculement:
    def __init__(self, seuil_deg=30.0, tenue_s=0.3, renversement_deg=60.0,
                 renversement_s=1.0, peremption_s=0.5):
        self.seuil_deg = float(seuil_deg)
        self.tenue_s = float(tenue_s)
        self.renversement_deg = float(renversement_deg)
        self.renversement_s = float(renversement_s)
        self.peremption_s = float(peremption_s)
        self.derniere = None          # (t, totale_deg)
        self._au_dela = None          # depuis quand au-delà du seuil
        self._renverse = None         # depuis quand au-delà du renversement
        self._arret_fait = False
        self._relache_fait = False

    def mesure(self, totale_deg, t):
        self.derniere = (t, float(totale_deg))
        v = abs(float(totale_deg))
        if v > self.seuil_deg:
            if self._au_dela is None:
                self._au_dela = t
        else:
            self._au_dela = None
            self._arret_fait = False
        if v > self.renversement_deg:
            if self._renverse is None:
                self._renverse = t
        else:
            self._renverse = None
            self._relache_fait = False

    def decision(self, t, sequence_en_cours):
        """None, 'arreter' (séquence en cours, trop penché) ou 'relacher' (renversé).
        Chaque décision n'est rendue qu'une fois par épisode."""
        if self.derniere is None or t - self.derniere[0] > self.peremption_s:
            return None
        if (self._renverse is not None and not self._relache_fait
                and t - self._renverse >= self.renversement_s):
            self._relache_fait = True
            return 'relacher'
        if (self._au_dela is not None and not self._arret_fait and sequence_en_cours
                and t - self._au_dela >= self.tenue_s):
            self._arret_fait = True
            return 'arreter'
        return None


class DelaiCoupure:
    """Diffère la coupure batterie tant qu'une séquence joue, au plus `grace_s`."""

    def __init__(self, grace_s=10.0):
        self.grace_s = float(grace_s)
        self.depuis = None

    def couper_maintenant(self, t, sequence_en_cours):
        if self.depuis is None:
            self.depuis = t
        if sequence_en_cours and t - self.depuis < self.grace_s:
            return False
        self.depuis = None
        return True

    def annuler(self):
        self.depuis = None
