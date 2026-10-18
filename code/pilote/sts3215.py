"""Driver série STS3215 (Feetech/Waveshare) pour Raspberry Pi.

Portage fidèle de l'ancien firmware ESP32 (esp32/src/sts3215.c) en pyserial.
Protocole demi-duplex Feetech STS : trame  FF FF ID LEN INSTR PARAMS CHECKSUM,
positions little-endian, 1 Mbaud. La bascule TX/RX est faite en matériel par
l'adaptateur bus-servo → aucune broche DIR à piloter côté Pi.

Toutes les transactions (write + read associé) sont protégées par un verrou :
plusieurs callbacks ROS (timer de streaming, réception des ordres de position)
peuvent solliciter le bus, il ne doit y avoir qu'une transaction à la fois.
"""
from collections import namedtuple
import threading
import time
import termios

import serial
import serial.rs485

# NB : lgpio n'est PAS importé au niveau module — son import démarre un thread qui
# crée un fichier de notification dans le répertoire courant, ce qui plante sous
# systemd (cwd = /). On passe par gpio_util.open_chip, qui gère ce piège, et
# uniquement en mode bit-bang DIR (le mode RS485 n'a besoin d'aucun GPIO).

# ── Registres (identiques à sts3215.h) ──────────────────────────────────────
REG_ID            = 0x05
REG_RETURN_DELAY  = 0x07
REG_MIN_ANGLE_L   = 0x09
REG_LOCK          = 0x37
REG_MAX_ANGLE_L   = 0x0B
REG_KP            = 0x15    # coefficient proportionnel (EEPROM, 0-255)
REG_KD            = 0x16    # coefficient dérivé        (EEPROM, 0-255)
REG_KI            = 0x17    # coefficient intégral      (EEPROM, 0-255)
REG_OFFSET_L      = 0x1F    # « Position Offset » : zéro matériel (EEPROM, 2 o)
REG_MODE          = 0x21    # mode de fonctionnement (EEPROM, 1 o) — voir MODES
REG_TORQUE_EN     = 0x28

# Modes du STS3215. Le seul qui obéisse à une consigne de POSITION est le 0.
#
# C'EST LE PIÈGE LE PLUS SILENCIEUX DE CE SERVO. En mode 1 (rotation continue,
# celui d'une roue), le servo IGNORE la position demandée : il n'interprète plus
# que la vitesse. Tout le reste répond normalement — le bus, l'EEPROM, la
# lecture de position — mais l'axe ne rejoint jamais sa consigne. On conclut à
# une calibration cassée, à un zéro mal placé, à un bus défaillant : le mode
# n'est visible nulle part tant qu'on ne le lit pas.
MODES = {0: 'position', 1: 'rotation continue', 2: 'PWM', 3: 'pas à pas'}
MODE_POSITION = 0

REG_ACC           = 0x29
REG_GOAL_POS_L    = 0x2A
REG_PRESENT_POS_L = 0x38

# ── LE COURANT, VRAIMENT MESURÉ ─────────────────────────────────────────────
# Registre 69 (0x45), deux octets, une unité valant 6,5 mA. C'est le SEUL
# ampèremètre de ce robot : la carte de puissance ne mesure que la tension du
# pack, et jusqu'ici le courant était DÉDUIT de la charge — un modèle, pas une
# mesure. La charge (0x3C) est une image du couple ; le courant en est une
# autre, mais celle-là est lue, pas calculée.
#
# On le lit dans LE MÊME paquet que le reste : de 0x38 à 0x46 il y a quinze
# octets au lieu de huit, soit sept de plus, ~70 µs à 1 Mbaud. Pour dix-huit
# servos à 30 Hz cela représente 4 % de temps de bus en plus — le prix d'une
# mesure qui n'existait pas.
#
# TOUS LES FIRMWARES NE LE RENDENT PAS. Les SCS et les premiers STS n'ont rien
# à cette adresse et peuvent refuser la longueur de lecture. Un refus ferait
# perdre TOUTE la télémétrie du servo, position comprise : c'est la régression
# qu'il ne faut pas introduire. `get_feedback` retombe donc d'elle-même sur la
# lecture courte de huit octets après plusieurs échecs d'affilée, et le dit.
REG_PRESENT_CURRENT = 0x45
COURANT_UNITE_A = 0.0065          # A par unité de registre (6,5 mA)
LONGUEUR_COURTE = 0x08            # 0x38..0x3F — l'historique
LONGUEUR_ETENDUE = 0x0F           # 0x38..0x46 — courant compris

# Une trame perdue ne prouve rien : ce bus demi-duplex en perd, et conclure sur
# un seul échantillon est l'erreur déjà corrigée dans `recentrer`. Il faut cinq
# échecs CONSÉCUTIFS de la lecture étendue pour décider que ce servo ne sait pas
# la faire — après quoi on ne la retente plus, et son courant est déclaré
# indisponible plutôt que rapporté à zéro.
ECHECS_AVANT_REPLI = 5

INSTR_PING  = 0x01
INSTR_READ  = 0x02
INSTR_WRITE = 0x03
# ECRITURE GROUPEE : une trame diffusee (id 0xFE) qui porte les donnees de
# plusieurs servos. Les servos n'y repondent JAMAIS — c'est exactement ce qu'il
# faut pour commander une cle entiere : aucune reponse ne peut entrer en
# collision avec l'ordre suivant, et tous partent au meme instant.
INSTR_SYNC_WRITE = 0x83

BROADCAST_ID = 0xFE

# Valeur « magique » du registre Torque Enable : ni 0 ni 1, c'est une COMMANDE.
# Le servo prend sa position courante comme milieu de course (2048) et ajuste
# lui-même son registre d'offset. C'est la calibration « une touche » de Feetech,
# celle qui permet de monter un palonnier à n'importe quel angle.
CALIB_MILIEU = 128

POS_MIN = 0
POS_MAX = 4095

# Résultat de `recentrer`. Un nom par champ plutôt qu'un tuple de quatre valeurs
# muettes : `redemarrage` en particulier ne se devine pas à sa position.
#
# `redemarrage` = la correction de zéro est BIEN ENREGISTRÉE dans le servo, mais
# sa position rapportée n'en tient pas encore compte — ce firmware n'applique le
# décalage qu'au démarrage. Ce n'est ni une réussite ni un échec : c'est une
# manœuvre en attente d'une coupure d'alimentation, et l'appelant doit le dire
# au lieu d'enchaîner sur un zéro qui va bouger sous ses pieds.
# Retour de `get_feedback`. Un nom par champ : l'ajout du courant a fait passer
# ce tuple de cinq à sept valeurs, et un appelant qui dépaquetait cinq positions
# muettes aurait échoué sans rien dire de la raison.
#
# `courant_ok` = ce servo a bien rendu le registre 0x45. False signifie « on ne
# sait pas », jamais « zéro ampère » — un servo au repos tire vraiment très peu,
# les deux se confondraient sans ce drapeau.
Feedback = namedtuple(
    'Feedback', 'pos speed load voltage temperature courant courant_ok')

Recentrage = namedtuple('Recentrage', 'ok position offset message redemarrage')
Recentrage.__new__.__defaults__ = (False,)


class Sts3215:
    """Accès bas niveau au bus de servos STS3215."""

    def __init__(self, port='/dev/serial0', baudrate=1_000_000,
                 reply_timeout=0.02, dir_gpio=None, gpiochip=4,
                 rs485=False, rts_tx_high=True):
        self.reply_timeout = reply_timeout
        self._lock = threading.Lock()
        # Par servo : nombre d'echecs CONSECUTIFS de la lecture etendue. Voir
        # ECHECS_AVANT_REPLI — c'est ce compteur qui evite de perdre toute la
        # telemetrie sur un firmware sans registre de courant.
        self._courant_absent = {}
        # timeout court par read() ; la limite réelle est gérée par une deadline.
        self.ser = serial.Serial(port, baudrate, timeout=0.005)
        self._fd = self.ser.fileno()

        # ── Mode RS485 : le NOYAU pilote DIR via la ligne RTS (GPIO17) ───────
        # Timing matériel précis pour le half-duplex, sans bit-bang Python.
        # Nécessite `dtoverlay=uart0,ctsrts` (GPIO17 = RTS0) et le fil DIR sur
        # la broche 11. rts_tx_high : niveau de RTS pendant l'émission.
        self._rs485 = bool(rs485)
        if self._rs485:
            try:
                self.ser.rs485_mode = serial.rs485.RS485Settings(
                    rts_level_for_tx=rts_tx_high,
                    rts_level_for_rx=not rts_tx_high,
                    delay_before_tx=None,
                    delay_before_rx=None,
                )
            except Exception as e:      # noqa: BLE001
                raise RuntimeError(
                    f"Mode RS485 refusé par le driver série ({e}). Vérifie que "
                    "`dtoverlay=uart0,ctsrts` est bien dans /boot/firmware/config.txt "
                    "et que la Pi a redémarré.")
            dir_gpio = None             # en RS485 le RTS EST le DIR → pas de lgpio

        # ── Broche DIR bit-bang (mode hérité, si rs485=False) ────────────────
        # DIR=1 → émission, DIR=0 → réception. dir_gpio = numéro BCM ; None = off.
        self._gpio_h = None
        self._lgpio = None
        self._dir = dir_gpio
        if dir_gpio is not None:
            from .gpio_util import get_chip      # import tardif (voir note en tête)
            self._lgpio, self._gpio_h = get_chip(gpiochip)
            self._lgpio.gpio_claim_output(self._gpio_h, dir_gpio, 0)   # 0 = réception

    @staticmethod
    def _busy_us(us):
        """Petite attente active (µs) : plus précise que time.sleep pour le DIR."""
        end = time.perf_counter() + us / 1e6
        while time.perf_counter() < end:
            pass

    def _dir_tx(self):
        if self._gpio_h is not None:
            self._lgpio.gpio_write(self._gpio_h, self._dir, 1)

    def _dir_rx(self):
        if self._gpio_h is not None:
            self._lgpio.gpio_write(self._gpio_h, self._dir, 0)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass
        if self._gpio_h is not None:
            from .gpio_util import free_line
            free_line(self._gpio_h, self._dir)      # le chip reste partagé
            self._gpio_h = None

    # ── Checksum ────────────────────────────────────────────────────────────
    @staticmethod
    def _checksum(payload):
        return (~sum(payload)) & 0xFF

    # ── Envoi d'un paquet (payload = octets après FF FF) ────────────────────
    def _send(self, payload):
        pkt = bytes([0xFF, 0xFF]) + bytes(payload) + bytes([self._checksum(payload)])
        self.ser.reset_input_buffer()   # purge le bruit avant d'émettre

        # Mode RS485 : le noyau bascule RTS (=DIR) autour de l'écriture, timing HW.
        if self._rs485:
            self.ser.write(pkt)
            self.ser.flush()
            return

        # Mode bit-bang (hérité) : on pilote DIR à la main.
        self._dir_tx()                  # DIR=1 : passe en émission
        if self._gpio_h is not None:
            self._busy_us(10)           # laisse le buffer basculer (comme l'ESP)
        self.ser.write(pkt)
        self.ser.flush()
        # tcdrain : bloque jusqu'à ce que le DERNIER bit soit physiquement parti,
        # sinon on couperait l'émission en repassant en réception trop tôt.
        try:
            termios.tcdrain(self._fd)
        except Exception:
            pass
        if self._gpio_h is not None:
            self._busy_us(15)           # marge pour le bit de stop @1Mbps
        self._dir_rx()                  # DIR=0 : le servo peut répondre
        # PAS de reset_input_buffer ici : à 1Mbps le servo a peut-être déjà répondu.

    # ── Lecture d'une réponse : FF FF ID LEN ERR [params] CHECKSUM ──────────
    def _read_exact(self, n, deadline):
        buf = bytearray()
        while len(buf) < n:
            if time.monotonic() > deadline:
                return None
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def _recv(self, expected_id, nparams):
        deadline = time.monotonic() + self.reply_timeout
        state = 0
        length = 0
        # Machine à état pour ignorer le bruit et caler sur FF FF ID LEN.
        while state < 4:
            if time.monotonic() > deadline:
                return None
            b = self.ser.read(1)
            if not b:
                continue
            c = b[0]
            if state == 0:
                state = 1 if c == 0xFF else 0
            elif state == 1:
                if c == 0xFF:
                    state = 2
                # sinon on reste en état 1 (attente du 2e FF)
            elif state == 2:
                if c == expected_id:
                    state = 3
                elif c == 0xFF:
                    state = 2
                else:
                    state = 0
            elif state == 3:
                length = c
                state = 4

        param_count = length - 2
        if param_count < 0 or param_count > nparams:
            return None

        body = self._read_exact(param_count + 2, deadline)   # ERR + params + CHK
        if body is None:
            return None
        err = body[0]
        params = body[1:1 + param_count]
        chk = body[param_count + 1]
        calc = self._checksum([expected_id, length, err] + list(params))
        if calc != chk:
            return None
        return list(params)

    # ── API publique (mêmes sémantiques que le firmware) ────────────────────
    def ping(self, servo_id):
        with self._lock:
            self._send([servo_id, 0x02, INSTR_PING])
            if servo_id == BROADCAST_ID:
                return True
            return self._recv(servo_id, 0) is not None

    def scan(self, max_id=20):
        found = []
        for servo_id in range(1, max_id + 1):
            if self.ping(servo_id):
                found.append(servo_id)
            time.sleep(0.005)
        return found

    def set_position(self, servo_id, position, speed=0, acc=0):
        position = max(POS_MIN, min(POS_MAX, int(position)))
        speed = int(speed) & 0xFFFF
        acc = int(acc) & 0xFF
        # WritePosEx : écriture groupée depuis REG_ACC (0x29) :
        # [acc, posL, posH, timeL=0, timeH=0, speedL, speedH]
        payload = [
            servo_id, 0x0A, INSTR_WRITE, REG_ACC, acc,
            position & 0xFF, (position >> 8) & 0xFF,
            0x00, 0x00,
            speed & 0xFF, (speed >> 8) & 0xFF,
        ]
        with self._lock:
            self._send(payload)   # pas d'ACK (commande haute fréquence)

    def sync_write_positions(self, points, acc=0):
        """Commande PLUSIEURS servos en une seule trame SYNC_WRITE.

        `points` : [(id, position, vitesse)]. Meme bloc de sept octets que
        `set_position`, a la meme adresse (0x29) : acceleration, position,
        temps (0), vitesse. On ne change que la facon de le dire au servo, pas
        ce qu'on lui dit.

        POURQUOI. Ecrire dix-huit servos un par un, coup sur coup, perdait des
        ordres : chaque servo repond a une ecriture individuelle, et sur un bus
        demi-duplex cette reponse entre en collision avec l'ordre du servo
        suivant. Releve sur le robot pendant une marche : servos immobiles 57 a
        100 % du temps. Une ecriture groupee n'appelle aucune reponse.

        Format (SDK Feetech) : FF FF FE LONGUEUR 83 adresse longueur_bloc
        [id + bloc] x N somme, avec LONGUEUR = N x (1 + longueur_bloc) + 4.
        """
        if not points:
            return
        acc = int(acc) & 0xFF
        donnees = []
        for (sid, position, vitesse) in points:
            position = max(POS_MIN, min(POS_MAX, int(position)))
            vitesse = int(vitesse) & 0xFFFF
            donnees += [int(sid) & 0xFF, acc,
                        position & 0xFF, (position >> 8) & 0xFF,
                        0x00, 0x00,
                        vitesse & 0xFF, (vitesse >> 8) & 0xFF]
        payload = [BROADCAST_ID, len(donnees) + 4, INSTR_SYNC_WRITE, REG_ACC, 7] + donnees
        with self._lock:
            self._send(payload)   # diffusion : aucune reponse a attendre

    def get_position(self, servo_id):
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_PRESENT_POS_L, 0x02])
            p = self._recv(servo_id, 2)
        if p is None or len(p) < 2:
            return None
        return p[0] | (p[1] << 8)

    def _lire_bloc(self, servo_id, longueur):
        """Une lecture groupée depuis 0x38, `longueur` octets. None si rien."""
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_PRESENT_POS_L, longueur])
            p = self._recv(servo_id, longueur)
        return p if (p is not None and len(p) >= longueur) else None

    def get_feedback(self, servo_id):
        """Lit d'un coup POS(38-39) SPEED(3A-3B) LOAD(3C-3D) VOLT(3E) TEMP(3F),
        et — quand le firmware le rend — le COURANT (0x45-0x46).

        Un seul paquet : quinze octets au lieu de huit, ~70 us de plus a 1 Mbaud.

        Vitesse et charge restent en SIGNE-MAGNITUDE, comme les registres : la
        conversion en valeur signee est faite par l'appelant. Le courant, lui,
        est rendu en AMPERES — sa conversion n'a rien d'ambigu et la refaire
        dans chaque appelant serait trois occasions de se tromper d'unite.

        @returns Feedback, ou None si le servo n'a rien repondu du tout.
        """
        etendu = self._courant_absent.get(servo_id, 0) < ECHECS_AVANT_REPLI
        if etendu:
            p = self._lire_bloc(servo_id, LONGUEUR_ETENDUE)
            if p is not None:
                self._courant_absent[servo_id] = 0
                return self._decoder(p, True)
            # Echec : peut-etre une trame perdue, peut-etre un firmware qui
            # refuse cette longueur. On compte, et on retente la lecture COURTE
            # tout de suite pour ne pas perdre l'echantillon en attendant.
            self._courant_absent[servo_id] =                 self._courant_absent.get(servo_id, 0) + 1

        p = self._lire_bloc(servo_id, LONGUEUR_COURTE)
        if p is None:
            return None
        return self._decoder(p, False)

    @staticmethod
    def _decoder(p, etendu):
        pos = p[0] | (p[1] << 8)
        speed = p[2] | (p[3] << 8)
        load = p[4] | (p[5] << 8)
        courant, courant_ok = 0.0, False
        if etendu and len(p) >= LONGUEUR_ETENDUE:
            brut = p[13] | (p[14] << 8)
            # Le registre est une MAGNITUDE : la fiche ne lui donne pas de bit de
            # sens, contrairement a la charge. Un courant negatif n'aurait de
            # toute facon pas de sens pour une consommation.
            courant = brut * COURANT_UNITE_A
            courant_ok = True
        return Feedback(pos, speed, load, p[6], p[7], courant, courant_ok)

    def courant_indisponible(self):
        """Les identifiants des servos qui ont renonce a rendre leur courant."""
        return sorted(i for i, n in self._courant_absent.items()
                      if n >= ECHECS_AVANT_REPLI)

    def ecrire_butees(self, servo_id, min_pos, max_pos):
        """Écrit les butées (EEPROM 0x09-0x0C), VÉRIFIE, et RAPPORTE.

        Cette fonction écrivait autrefois à cru, sans lever le verrou d'EEPROM
        (registre LOCK). Or le servo ACCEPTE la trame dans ce cas : il répond
        normalement, ne signale rien, et garde ses anciennes butées. Toutes les
        écritures pouvaient donc être sans effet sans que rien ne le dise — et
        l'application affichait des butées que le matériel ignorait. Même piège
        que `set_return_delay`, qui lui prenait déjà la précaution.

        DEUX TENTATIVES, PAS UNE. L'EEPROM met un temps non spécifié à se
        graver, et la synchronisation d'un moteur enchaîne trois écritures
        rapprochées (mode, offset, butées). Une écriture qui arrive pendant que
        la précédente se grave est perdue — sans erreur, comme toujours ici. Le
        premier essai échouait donc sur les servos qui avaient pris le chemin
        long, et sur eux seuls : exactement le tableau observé.

        UNE RELECTURE NE SUFFIT PAS NON PLUS. Ce bus demi-duplex perd des trames ;
        une relecture vide vaut « je n'ai pas vu », pas « le servo a refusé ».
        Juger un refus sur un seul échantillon, c'est la panne qu'on a déjà
        corrigée dans `recentrer` (cf. `_insister`).

        On ne réécrit pas si le servo porte déjà ces valeurs : une écriture
        EEPROM inutile est de l'usure gratuite (endurance ~10⁵ cycles), et cette
        fonction est appelée aussi par la détection de calage.

        @returns (ok, lues, message) — `lues` est ce que le servo PORTE (ou None
                 si illisible), jamais ce qu'on lui a demandé.
        """
        min_pos = max(POS_MIN, min(POS_MAX, int(min_pos)))
        max_pos = max(POS_MIN, min(POS_MAX, int(max_pos)))
        vise = (min_pos, max_pos)

        lues = self.limites_sures(servo_id)
        if lues == vise:
            return (True, lues, f'servo {servo_id} : butées déjà {min_pos}…{max_pos}, '
                                f'aucune écriture faite.')

        journal = [f'avant={lues}']
        for essai, repos in enumerate((0.0, 0.15), start=1):
            if repos:
                time.sleep(repos)          # laisser la gravure précédente finir
            self._ecrire_eeprom(servo_id, REG_MIN_ANGLE_L, [
                min_pos & 0xFF, (min_pos >> 8) & 0xFF,
                max_pos & 0xFF, (max_pos >> 8) & 0xFF,
            ])
            lues = self.limites_sures(servo_id)
            journal.append(f'essai {essai} → relu={lues}')
            if lues == vise:
                return (True, lues, f'servo {servo_id} : butées {min_pos}…{max_pos} '
                                    f'écrites et confirmées ({" | ".join(journal)}).')

        # Échec confirmé : dire ce que le servo porte, et pourquoi il peut
        # refuser. Un « REFUSÉ » sans mesure n'est pas un diagnostic.
        mode = self.get_mode(servo_id)
        detail = ''
        if mode not in (None, MODE_POSITION):
            detail = (f' Il est en mode « {MODES.get(mode, mode)} » : dans ce mode le '
                      f'servo gère lui-même ses butées et ignore les consignes '
                      f'd\'angle. Le remettre en mode position d\'abord.')
        elif lues is None:
            detail = (' Ses butées sont ILLISIBLES après plusieurs tentatives : '
                      'c\'est la lecture qui échoue, pas forcément l\'écriture. '
                      'Vérifier le câblage et le Return Delay de ce servo.')
        else:
            detail = (f' Il porte {lues[0]}…{lues[1]}. L\'EEPROM refuse la gravure : '
                      f'verrou (0x37), ou servo alimenté trop faiblement pour '
                      f'graver. Voir diag_butees.py.')
        return (False, lues,
                f'servo {servo_id} : butées {min_pos}…{max_pos} REFUSÉES.{detail} '
                f'Mesures : {" | ".join(journal)}')

    def set_angle_limits(self, servo_id, min_pos, max_pos):
        """Forme booléenne de `ecrire_butees`, pour les appelants qui n'ont que
        faire du détail (détection de calage)."""
        return self.ecrire_butees(servo_id, min_pos, max_pos)[0]

    def get_angle_limits(self, servo_id):
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_MIN_ANGLE_L, 0x04])
            p = self._recv(servo_id, 4)
        if p is None or len(p) < 4:
            return None
        return (p[0] | (p[1] << 8), p[2] | (p[3] << 8))

    def set_torque(self, servo_id, enable):
        payload = [servo_id, 0x04, INSTR_WRITE, REG_TORQUE_EN, 0x01 if enable else 0x00]
        with self._lock:
            self._send(payload)
        return True

    # ── Écriture registre générique (aveugle, sans réponse attendue) ────────
    def write_register(self, servo_id, reg, values):
        values = list(values)
        payload = [servo_id, len(values) + 3, INSTR_WRITE, reg] + values
        with self._lock:
            self._send(payload)
        return True

    # ── EEPROM : déverrouiller, écrire, revérrouiller ──────────────────────
    def _ecrire_eeprom(self, servo_id, reg, values):
        """Écriture EEPROM encadrée du dé/reverrouillage (registre LOCK).

        Sans cela l'écriture est ignorée SANS erreur : le servo répond
        normalement et garde son ancienne valeur. C'est la panne la plus
        trompeuse de ce bus — d'où la relecture systématique chez les appelants.
        """
        self.write_register(servo_id, REG_LOCK, [0])
        time.sleep(0.02)
        self.write_register(servo_id, reg, list(values))
        time.sleep(0.02)
        self.write_register(servo_id, REG_LOCK, [1])
        time.sleep(0.02)
        return True

    # ── Mode de fonctionnement ─────────────────────────────────────────────
    def get_mode(self, servo_id):
        """Mode courant (voir MODES), ou None."""
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_MODE, 0x01])
            p = self._recv(servo_id, 1)
        return None if not p else p[0]

    def set_mode(self, servo_id, mode):
        """Écrit le mode et le RELIT. Retourne le mode effectivement porté.

        EEPROM : sans levée du verrou, l'écriture est acceptée sans effet et le
        servo garde son ancien mode — la panne la plus trompeuse de ce bus.
        """
        self._ecrire_eeprom(servo_id, REG_MODE, [int(mode) & 0xFF])
        time.sleep(0.05)
        return self.get_mode(servo_id)

    def mode_sur(self, servo_id, essais=5):
        """Mode relu avec insistance. Une trame perdue n'est pas un mode."""
        return self._insister(lambda: self.get_mode(servo_id), essais)

    def forcer_mode(self, servo_id, mode=MODE_POSITION):
        """IMPOSE le mode, quel que soit celui de départ, et le CONFIRME.

        POURQUOI « QUEL QUE SOIT ». La version précédente lisait le mode et
        n'écrivait que s'il différait — et si la lecture échouait, elle ne
        faisait RIEN, silencieusement. Un mode illisible se comportait donc
        exactement comme un mode correct : aucune ligne au journal, aucune
        écriture, et la garantie « ce servo est en mode position » ne reposait
        sur rien. Ici, tout ce qui n'est pas une lecture CONFIRMÉE au bon mode
        déclenche une écriture, puis une vérification.

        Deux tentatives : l'EEPROM met un temps non spécifié à se graver, et
        cette écriture arrive souvent derrière une autre (cf. `ecrire_butees`).

        @returns (ok, avant, apres) — `ok` signifie « le servo PORTE ce mode,
                 relecture à l'appui », jamais « la trame est partie ».
        """
        avant = self.mode_sur(servo_id)
        if avant == mode:
            return (True, avant, avant)

        apres = None
        for essai, repos in enumerate((0.0, 0.15), start=1):
            if repos:
                time.sleep(repos)
            self._ecrire_eeprom(servo_id, REG_MODE, [int(mode) & 0xFF])
            time.sleep(0.05)
            apres = self.mode_sur(servo_id)
            if apres == mode:
                return (True, avant, apres)
        return (False, avant, apres)

    def remettre_en_position(self, servo_id):
        """Ramène un servo en mode POSITION. Forme historique de `forcer_mode`.

        Sert aux servos récupérés d'un autre montage : une roue de voiture est
        configurée en rotation continue, et rebranchée sur une patte elle
        n'obéit à aucune consigne d'angle — sans que rien ne le dise.

        Retourne (change, mode_avant, mode_apres).
        """
        ok, avant, apres = self.forcer_mode(servo_id, MODE_POSITION)
        return (ok and avant != apres, avant, apres)

    # ── PID de l'asservissement interne ────────────────────────────────────
    def get_pid(self, servo_id):
        """(Kp, Ki, Kd) ou None. Les registres sont contigus : 0x15 P, 0x16 D, 0x17 I.
        Attention à l'ordre — ce n'est PAS P, I, D dans la mémoire."""
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_KP, 0x03])
            p = self._recv(servo_id, 3)
        if p is None or len(p) < 3:
            return None
        return (p[0], p[2], p[1])          # Kp, Ki, Kd

    def get_asservissement(self, servo_id):
        """Registres EEPROM 0x15 à 0x1B, d'un seul paquet, ou None :
        P (0x15), D (0x16), I (0x17), force minimale de démarrage (0x18), 0x19,
        zones mortes horaire (0x1A) et antihoraire (0x1B), en pas.

        Lecture seule. Sert à comprendre les à-coups à l'arrêt : un servo dont
        l'écart sort de sa zone morte repart avec au moins sa force minimale —
        c'est exactement un à-coup quand l'articulation est contrainte."""
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_KP, 0x07])
            p = self._recv(servo_id, 7)
        if p is None or len(p) < 7:
            return None
        return {'p': p[0], 'd': p[1], 'i': p[2], 'demarrage_min': p[3], 'r19': p[4],
                'zone_morte_cw': p[5], 'zone_morte_ccw': p[6]}

    def set_pid(self, servo_id, kp, ki, kd):
        """Écrit les trois coefficients puis RELIT. Retourne le triplet relu, ou None.

        Un PID mal réglé s'entend et se voit : trop de P et le servo vibre autour
        de sa consigne, trop peu et il traîne. On ne peut donc pas se permettre
        de croire une écriture sur parole.
        """
        vals = [max(0, min(255, int(kp))), max(0, min(255, int(kd))),
                max(0, min(255, int(ki)))]                 # ordre mémoire : P, D, I
        self._ecrire_eeprom(servo_id, REG_KP, vals)
        return self.get_pid(servo_id)

    # ── Zéro matériel (Position Offset) ────────────────────────────────────
    #
    # La convention de signe de ce registre n'est pas la même partout : certains
    # firmwares placent le bit de signe en bit 11 (magnitude sur 11 bits), d'autres
    # en bit 15. Se tromper décale le zéro de plusieurs milliers de pas. On ne
    # DEVINE donc pas : `recentrer` essaie et vérifie.
    _SIGNES = (0x0800, 0x8000)

    @staticmethod
    def _encoder_signe(valeur, bit_signe):
        magnitude = min(abs(int(valeur)), bit_signe - 1)
        return magnitude | (bit_signe if valeur < 0 else 0)

    @staticmethod
    def _decoder_signe(brut, bit_signe):
        magnitude = brut & (bit_signe - 1)
        return -magnitude if (brut & bit_signe) else magnitude

    def get_offset_brut(self, servo_id):
        """Contenu brut du registre 0x1F (non interprété), ou None."""
        with self._lock:
            self._send([servo_id, 0x04, INSTR_READ, REG_OFFSET_L, 0x02])
            p = self._recv(servo_id, 2)
        if p is None or len(p) < 2:
            return None
        return p[0] | (p[1] << 8)

    def _ecrire_offset_brut(self, servo_id, brut):
        self._ecrire_eeprom(servo_id, REG_OFFSET_L,
                            [brut & 0xFF, (brut >> 8) & 0xFF])
        time.sleep(0.05)                   # le servo réapplique son zéro
        return self.get_offset_brut(servo_id)

    # ── Lecture insistante ─────────────────────────────────────────────────
    def _insister(self, lecture, essais=5, pause=0.012):
        """Répète une lecture jusqu'à obtenir une réponse, ou renonce.

        Sur ce bus demi-duplex à 1 Mbaud, une lecture isolée échoue de temps en
        temps — trame perdue dans la bascule TX/RX, `Return Delay` trop court sur
        un servo. C'est sans conséquence pour le flux de télémétrie, qui
        réessaie 10 fois par seconde. Ça en a une pour une CALIBRATION : la
        vérification lisait None, la manœuvre était déclarée en échec, et
        l'utilisateur voyait « sans effet » alors que le servo venait de se
        recentrer correctement. Une décision ne se prend pas sur un seul
        échantillon d'un canal qui perd des trames.
        """
        for i in range(essais):
            valeur = lecture()
            if valeur is not None:
                return valeur
            if i + 1 < essais:
                time.sleep(pause)
        return None

    def position_sure(self, servo_id, essais=5):
        return self._insister(lambda: self.get_position(servo_id), essais)

    def offset_sur(self, servo_id, essais=5):
        return self._insister(lambda: self.get_offset_brut(servo_id), essais)

    def limites_sures(self, servo_id, essais=5):
        """Butées relues avec insistance. Une trame perdue ne doit pas se lire
        « le servo a refusé l'écriture » : c'est un contresens, et il envoie
        chercher la panne à l'opposé de là où elle est."""
        return self._insister(lambda: self.get_angle_limits(servo_id), essais)

    def calibrer_milieu(self, servo_id):
        """Calibration « une touche » : la position courante devient 2048.

        Voie CONSTRUCTEUR — écrire 128 dans le registre *Torque Enable* (40) est
        une commande, pas un état : le servo prend sa position du moment comme
        milieu de course et calcule LUI-MÊME son offset. On n'a donc ni encodage
        de signe à deviner, ni sens de correction à choisir : les deux pièges de
        l'écriture directe du registre 0x1F.

        LE VERROU D'EEPROM VAUT AUSSI POUR ELLE. C'est le servo qui écrit, mais
        il écrit dans SON EEPROM, et le registre LOCK (0x37) la protège de la
        même façon. Verrou en place, la valeur calculée n'atteint que la RAM :
        on la relit correctement, et elle a disparu au démarrage suivant.
        Symptôme exact du servo #18 — offset relu à 2829 juste après la
        manœuvre, revenu à 85 après une coupure d'alimentation. Toutes les
        autres écritures d'EEPROM du driver lèvent ce verrou ; celle-ci était la
        seule à ne pas le faire, parce qu'on ne la voyait pas comme une écriture.

        Elle ne sait viser que 2048. Retourne la position relue après coup, ou None.
        """
        self.write_register(servo_id, REG_LOCK, [0])
        time.sleep(0.02)
        self.write_register(servo_id, REG_TORQUE_EN, [CALIB_MILIEU])
        time.sleep(0.12)                   # le servo réécrit son EEPROM d'offset
        self.write_register(servo_id, REG_LOCK, [1])
        time.sleep(0.02)
        # 128 n'est ni 0 ni 1 : l'état du couple après la commande n'est pas
        # spécifié. On le remet explicitement à zéro — l'axe ne doit pas se
        # raidir sous les doigts juste après un recentrage.
        self.set_torque(servo_id, False)
        return self.position_sure(servo_id)

    def recentrer(self, servo_id, cible=2048, tolerance=12):
        """Décale le zéro matériel pour que la position courante devienne `cible`.

        Sert quand le palonnier a été vissé de travers : la couture 0/4095 du
        codeur tombe alors dans la course utile du joint, et aucun calcul côté PC
        ne peut l'en sortir — c'est un endroit physique de l'axe.

        Deux voies, essayées dans cet ordre :
          1. la calibration « une touche » du constructeur (registre 40 ← 128) ;
          2. à défaut, l'écriture directe du registre d'offset (0x1F).

        La voie 2 comporte DEUX inconnues, pas une : le bit de signe (11 ou 15
        selon les firmwares) ET le sens dans lequel le servo applique l'offset
        (`position ± offset`). Une première version n'essayait que les deux
        conventions de signe, dans un seul sens : quand le sens était l'autre,
        les quatre combinaisons utiles n'étaient jamais atteintes, l'offset
        d'origine était restauré, et la position ne bougeait pas d'un pas — sans
        rien dire de plus. On balaie donc les quatre, et **le message d'échec
        porte le journal des mesures** : sans elles, il n'y a rien à diagnostiquer.

        Le couple doit être RELÂCHÉ : on déplace le repère, pas l'axe.
        Retourne (ok, position_relue, offset_applique, message).
        """
        cible = max(POS_MIN, min(POS_MAX, int(cible)))
        journal = []

        # LE MODE D'ABORD. Un servo en rotation continue ignore toute consigne de
        # position : recentrer son zéro, lui rendre sa course, écrire son offset
        # — tout réussit, et rien ne bouge. C'est exactement le tableau d'un
        # servo récupéré d'un montage à roues. On le remet donc en mode position
        # avant quoi que ce soit d'autre, sinon la suite ne veut rien dire.
        mode_ok, mode_avant, mode_apres = self.forcer_mode(servo_id, MODE_POSITION)
        nom_avant = MODES.get(mode_avant, mode_avant) if mode_avant is not None else 'illisible'
        if mode_ok and mode_avant == MODE_POSITION:
            journal.append('mode position (déjà)')
        elif mode_ok:
            journal.append(f'mode {nom_avant} → position, gravé et confirmé '
                           f'(le servo ignorait les consignes d\'angle)')
        else:
            return Recentrage(
                False, 0, 0,
                f'servo {servo_id} : mode de départ « {nom_avant} », et il REFUSE de '
                f'passer en mode position (relu : {MODES.get(mode_apres, mode_apres)}). '
                f'Hors du mode position il ignore toute consigne d\'angle — aucun '
                f'recalage ne peut aboutir, et rien d\'autre n\'a été tenté. '
                f'Piste : l\'EEPROM de ce servo ne grave plus '
                f'(diag_recentrage.py --persistance).')

        brut_initial = self.offset_sur(servo_id)
        depart = self.position_sure(servo_id)
        if depart is None:
            return Recentrage(False, 0, 0, f'servo {servo_id} : position illisible même '
                                 'après plusieurs tentatives. Le servo répond-il ?')
        journal.append(f'départ pos={depart} offset_brut={brut_initial}')

        # Déjà en place : ne rien écrire. Une écriture EEPROM gratuite est de
        # l'usure gratuite, et un recentrage inutile ferait perdre la calibration
        # logicielle du moteur pour rien.
        if abs(depart - cible) <= tolerance:
            return Recentrage(True, depart, self._decoder_signe(brut_initial or 0, self._SIGNES[0]),
                    f'servo {servo_id} déjà à {depart} (cible {cible}) : rien à '
                    f'recentrer, aucune écriture faite.')

        # ── Voie 1 : commande de calibration du constructeur ────────────────
        if cible == 2048:
            obtenu = self.calibrer_milieu(servo_id)
            brut_apres = self.offset_sur(servo_id)
            journal.append(f'une-touche(reg40←128) → pos={obtenu} '
                           f'offset_brut={brut_apres}')
            if obtenu is not None and abs(obtenu - cible) <= tolerance:
                return Recentrage(True, obtenu, 0,
                        f'servo {servo_id} recentré par calibration une touche : '
                        f'{depart} → {obtenu}')

            # LA CORRECTION EST ÉCRITE MAIS PAS APPLIQUÉE.
            #
            # Le servo a calculé LUI-MÊME sa correction — c'est sa propre
            # routine — et l'a bien rangée dans son EEPROM, mais la position
            # qu'il rapporte n'en tient pas compte. Signature d'un firmware qui
            # n'applique le décalage qu'au démarrage.
            #
            # Ne surtout PAS balayer les quatre conventions par-dessus : on
            # écraserait une correction juste par des essais faux, et le
            # dernier geste du balayage — restaurer l'offset d'origine —
            # effacerait la seule chose qui ait marché. C'est ce qui s'est
            # passé sur le servo récupéré d'un ancien montage.
            attendu = abs(depart - cible)
            if brut_apres is not None and any(
                    abs(abs(self._decoder_signe(brut_apres, bit)) - attendu) <= max(tolerance, 4)
                    for bit in self._SIGNES):
                return Recentrage(
                    False, obtenu or depart,
                    self._decoder_signe(brut_apres, self._SIGNES[0]),
                    f'servo {servo_id} : la correction de zéro EST ENREGISTRÉE '
                    f'({depart - cible} pas, exactement ce qu\'il faut), mais la '
                    f'position rapportée n\'a pas bougé ({obtenu}). Ce firmware '
                    f'n\'applique le décalage qu\'AU DÉMARRAGE du servo. '
                    f'Coupe puis rétablis l\'alimentation des pattes, puis '
                    f'resynchronise ce moteur — la correction est conservée, '
                    f'rien d\'autre n\'est à refaire. '
                    f'Mesures : ' + ' | '.join(journal),
                    True)                              # redemarrage

            if obtenu is None:
                # On a écrit, on ne sait pas ce que ça a donné. Enchaîner sur la
                # voie 2 écraserait peut-être une calibration réussie ; annoncer
                # un échec serait tout aussi faux. On s'arrête et on le dit.
                return Recentrage(False, depart, 0,
                        f'servo {servo_id} : la commande de calibration est partie, '
                        f'mais la position reste ILLISIBLE — impossible de dire si '
                        f'elle a pris. Ne rien conclure : relire la position '
                        f'(diag_recentrage.py) avant de retenter. '
                        f'Mesures : ' + ' | '.join(journal))

        # ── Voie 2 : écriture directe de l'offset, 2 sens × 2 signes ────────
        if brut_initial is None:
            return Recentrage(False, depart, 0,
                    f'servo {servo_id} : registre d\'offset illisible, la voie de '
                    f'secours est donc hors de portée (on ne peut pas restaurer ce '
                    f'qu\'on n\'a pas pu lire). Mesures : ' + ' | '.join(journal))

        # La voie 1 a pu bouger le repère : on repart de la mesure fraîche.
        courant = self.position_sure(servo_id) or depart
        for bit in self._SIGNES:
            for sens in (+1, -1):
                delta = self._decoder_signe(brut_initial, bit) \
                    + sens * (courant - cible)
                brut = self._encoder_signe(delta, bit)
                relu = self._ecrire_offset_brut(servo_id, brut)
                obtenu = self.position_sure(servo_id)
                journal.append(
                    f'offset signe=0x{bit:04X} sens={sens:+d} écrit={brut} '
                    f'relu={relu} → pos={obtenu}')
                # On ne juge PAS sur la relecture de l'offset : certains
                # firmwares le normalisent en le rangeant. Seule la position
                # obtenue tranche — c'est elle qu'on cherchait à déplacer.
                if obtenu is not None and abs(obtenu - cible) <= tolerance:
                    return Recentrage(True, obtenu, self._decoder_signe(brut, bit),
                            f'servo {servo_id} recentré : {depart} → {obtenu} '
                            f'(offset direct, signe 0x{bit:04X}, sens {sens:+d})')

        restaure = self._ecrire_offset_brut(servo_id, brut_initial) is not None
        return Recentrage(False, depart, 0,
                f'servo {servo_id} : recentrage SANS EFFET'
                + (', état d\'origine restauré.' if restaure
                   else ' — et la restauration de l\'offset n\'a PAS pu être '
                        'confirmée : vérifier avec diag_recentrage.py.')
                + ' Mesures : ' + ' | '.join(journal))

    def set_return_delay(self, value, servo_id=BROADCAST_ID):
        """Règle le *Return Delay Time* (EEPROM) : délai avant la réponse du servo.
        value × 2 µs (ex. 100 → 200 µs). Laisse au Pi le temps de repasser DIR en
        réception avant que le servo ne parle. À faire UNE fois par servo (persistant).
        Écriture aveugle → fonctionne même si la relecture ne marche pas encore.
        """
        self.write_register(servo_id, REG_LOCK, [0])          # déverrouille l'EEPROM
        time.sleep(0.02)
        self.write_register(servo_id, REG_RETURN_DELAY, [value & 0xFF])
        time.sleep(0.02)
        self.write_register(servo_id, REG_LOCK, [1])          # reverrouille
        time.sleep(0.02)
        return True
