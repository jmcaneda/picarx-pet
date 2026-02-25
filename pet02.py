############################################################
# pet02.py
############################################################

import os
import time
from vilib import Vilib
from enum import Enum
from libs import hello_px, check_robot
# from picarx.music import Music

# music = Music()

# ============================================================
# CONSTANTES
# ============================================================

BALIZA_COLOR = "red"

PAN_MIN = -35
PAN_MAX = 35

TILT_MIN = -20
TILT_MAX = 20

SERVO_ANGLE_MIN = -30
SERVO_ANGLE_MAX = 30

CX = 320
CY = 240

FAST_SPEED = 20 
SLOW_SPEED = 5
TURN_SPEED = 1 

CAM_STEP = 4

SAFE_DISTANCE = 35
DANGER_DISTANCE = 20

LOG_PATH = os.path.join(os.path.dirname(__file__), "pet02.log")

# SOUNDS_DIR = "/home/jmcaneda/picarx-projects/autonomous/sounds"

# ============================================================
# CLASES
# ============================================================

class Estado(Enum):
    IDLE = 1
    RESET = 2
    SEARCH = 3
    RECENTER = 5
    TRACK = 10
    NEAR = 15
    ERR = 89
    CHK = 99

class Cmd(Enum):
    STOP = 1
    FORWARD = 2
    FORWARD_SLOW = 3
    WHEELS_TURN_LEFT = 4
    WHEELS_TURN_RIGHT = 5
    BACKWARD = 6
    SCAPE = 8
    CAM_PAN_LEFT = 9
    CAM_PAN_RIGHT = 10
    CAM_TILT_TOP = 20
    CAM_TILT_BOTTOM = 30

class Det:
    """
    Representa una detección de la baliza.
    Incluye centroide, tamaño y utilidades para TRACK.
    """

    def __init__(self, n, x, y, w, h, cx=CX, cy=CY):
        self.n = n          # número de detecciones
        self.x = x          # centroide X
        self.y = y          # centroide Y
        self.w = w          # ancho detectado
        self.h = h          # alto detectado
        self.cx = cx        # centro ideal X
        self.cy = cy        # centro ideal Y

    @property
    def area(self):
        return self.w * self.h

    @property
    def error_x(self):
        return self.x - self.cx

    @property
    def error_y(self):
        return self.y - self.cy

    @property
    def is_centered(self):
        if not self.valid_for_search:
            return False
        return abs(self.error_x) <= 40

    @property
    def valid_for_search(self):
        if self.w == 0 or self.h == 0 or self.n == 0:
            return False

        return (
            self.n >= 1 and
            20 < self.w < 350 and     # coherente con Search
            20 < self.h < 480 and
            800 < self.area < 80000 and
            0 < self.x < 640 and
            0 < self.y < 480
        )
    
    @property
    def valid_for_near(self):
        if not self.valid_for_search:
            return False

        # 1. Área realmente grande (cerca de verdad)
        if self.area < 12000:
            return False

        # 2. Centrado horizontal más estricto
        if abs(self.error_x) > 40:
            return False

        return True

    def __repr__(self):
        return (
            "Det("
            f"n={self.n}, "
            f"x={self.x}, y={self.y}, "
            f"w={self.w}, h={self.h}, "
            f"area={self.area}, "
            f"error_x={self.error_x}, error_y={self.error_y}, "
            f"is_centered={self.is_centered}, "
            f"valid_for_search={self.valid_for_search}, "
            f"valid_for_near={self.valid_for_near}"
            ")"
        )

class RobotState:
    def __init__(self):

        # SEARCH
        self.search_no_det_frames = 0

        # RECENTER
        self.recenter_centered_frames = 0
        self.recenter_lost_frames = 0

        # TRACK
        self.track_lost_frames = 0

        # Histeresis para NEAR 
        self.near_enter_frames = 0 
        self.near_exit_frames = 0
        self.near_done_backward = False
        self.near_lost_frames = 0

        # Cooldown tras backward en NEAR
        self.near_cooldown = None

        # Cooldown tras RECENTER
        self.just_recentered = None
        
# ============================================================
# INICIALIZACIÓN
# ============================================================

def init_camera(px):
    # Iniciar cámara
    Vilib.camera_start(vflip=False, hflip=False)

    # Mostrar por web (local fallará si no hay GUI, es normal)
    Vilib.display(local=False, web=False)

    # Activar detección de color predefinido
    Vilib.color_detect(BALIZA_COLOR)

    time.sleep(0.5)


def init_internal_state(px):
    return Estado.IDLE, Cmd.STOP


def init_flags(px):
    # Logging
    px.last_log = None

    # Estado previo
    px.estado_actual = None
    px.last_state = None

    # Detección
    px.last_raw_n = 0

    # Seguridad
    px.last_sec = "safe"
    px.dist = 999

    # SEARCH
    px.search_dir = 1          # 1 = derecha, -1 = izquierda
    # px.search_steps = 0
    px.search_seen = 0

    # Cámara
    px.last_pan = 0
    px.last_tilt = 0

# ============================================================
# LOGGING
# ============================================================

def log_event(px, estado, msg):
    # Si es igual al último mensaje, no lo repitas
    if px.last_log == (estado, msg):
        return

    px.last_log = (estado, msg)

    ts = time.strftime("%H:%M:%S", time.localtime())
    line = f"[{ts}] [{estado}] {msg}\n"

    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line)


# ============================================================
# ACCIONES BÁSICAS v2 — Movimiento fluido y continuo
# ============================================================

def stop(px):
    px.stop()
    # No tocamos el servo. La FSM decide cuándo centrarlo.

def forward(px, speed=FAST_SPEED):
    px.forward(speed)

def forward_slow(px, speed=SLOW_SPEED):
    px.forward(speed)

def backward(px, speed=SLOW_SPEED):
    px.backward(speed)
    # No bloqueamos la FSM. NEAR controla su propio backward.

# ------------------------------------------------------------
# GIRO CONTINUO REAL
# ------------------------------------------------------------

def turn_left(px, speed=TURN_SPEED):
    # Mantener el servo girado continuamente
    px.set_dir_servo_angle(SERVO_ANGLE_MIN)
    px.forward(speed)
    # Sin STOP, sin servo=0, sin sleeps.
    # La FSM decidirá cuándo parar o centrar.


def turn_right(px, speed=TURN_SPEED):
    px.set_dir_servo_angle(SERVO_ANGLE_MAX)
    px.forward(speed)
    # Igual que turn_left: giro continuo real.


# ------------------------------------------------------------
# SCAPE seguro (solo si se usa)
# ------------------------------------------------------------

def scape_danger(px, speed=SLOW_SPEED):
    # Retroceso suave sin giros bruscos
    px.set_dir_servo_angle(0)
    px.backward(speed)
    time.sleep(0.3)
    px.stop()

# ============================================================
# MOVIMIENTOS DE CÁMARA SEGUROS
# ============================================================

def pan_right(px, step=CAM_STEP):
    new_angle = px.last_pan + step
    if new_angle >= PAN_MAX:
        new_angle = PAN_MAX
    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)

def pan_left(px, step=CAM_STEP):
    new_angle = px.last_pan - step
    if new_angle <= PAN_MIN:
        new_angle = PAN_MIN
    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)

def tilt_top(px, step=CAM_STEP):
    new_angle = px.last_tilt + step
    if new_angle >= TILT_MAX:
        new_angle = TILT_MAX
    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)

def tilt_bottom(px, step=CAM_STEP):
    new_angle = px.last_tilt - step
    if new_angle <= TILT_MIN:
        new_angle = TILT_MIN
    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)


# ============================================================
# MAPEO DE COMANDOS
# ============================================================

def execute_motion(px, estado, cmd: Cmd, test_mode=False):

    # ============================================================
    # MODO SIMULADO
    # ============================================================
    if test_mode:

        # --- Comandos permitidos en sim ---
        if cmd in (Cmd.CAM_PAN_LEFT, Cmd.CAM_PAN_RIGHT,
                   Cmd.CAM_TILT_TOP, Cmd.CAM_TILT_BOTTOM,
                   Cmd.STOP):

            # Ejecutar realmente (para ver movimiento en sim)
            try:
                if cmd == Cmd.STOP:
                    stop(px)
                elif cmd == Cmd.CAM_PAN_LEFT:
                    pan_left(px)
                elif cmd == Cmd.CAM_PAN_RIGHT:
                    pan_right(px)
                elif cmd == Cmd.CAM_TILT_TOP:
                    tilt_top(px)
                elif cmd == Cmd.CAM_TILT_BOTTOM:
                    tilt_bottom(px)

                log_event(px, estado or Estado.CHK, f"[SIM] Ejecutado: {cmd.name}")
                return True

            except Exception as e:
                log_event(px, estado or Estado.ERR, f"[SIM] Error ejecutando {cmd}: {e}")
                return False

        # --- Comandos bloqueados en sim ---
        else:
            log_event(px, estado or Estado.CHK, f"[SIM] BLOQUEADO (solo imprimir): {cmd.name}")
            return True

    # ============================================================
    # MODO REAL
    # ============================================================
    try:
        if cmd == Cmd.STOP:
            stop(px)

        elif cmd == Cmd.FORWARD:
            forward(px, FAST_SPEED)

        elif cmd == Cmd.FORWARD_SLOW:
            forward_slow(px, SLOW_SPEED)

        elif cmd == Cmd.WHEELS_TURN_LEFT:
            turn_left(px)

        elif cmd == Cmd.WHEELS_TURN_RIGHT:
            turn_right(px)

        elif cmd == Cmd.BACKWARD:
            backward(px)

        elif cmd == Cmd.SCAPE:
            scape_danger(px, TURN_SPEED)

        # --- Cámara ---
        elif cmd == Cmd.CAM_PAN_LEFT:
            pan_left(px)

        elif cmd == Cmd.CAM_PAN_RIGHT:
            pan_right(px)

        elif cmd == Cmd.CAM_TILT_TOP:
            tilt_top(px)

        elif cmd == Cmd.CAM_TILT_BOTTOM:
            tilt_bottom(px)

        else:
            log_event(px, Estado.ERR, f"Comando desconocido: {cmd}")
            stop(px)
            return False

        log_event(px, estado, f"Ejecutado: {cmd.name}")
        return True

    except Exception as e:
        log_event(px, Estado.ERR, f"Error ejecutando {cmd}: {e}")
        stop(px)
        return False

# ============================================================
# SEGURIDAD
# ============================================================
def update_safety(px):
    d = px.get_distance()

    # Filtro de valores basura
    if d <= 0 or d > 400:   # ajusta 400 si tu sensor tiene otro rango
        d = 999

    return {
        "distance": d,
        "timestamp": time.time(),
        "raw": d
    }

def apply_safety(px, safety, estado, accion):
    d = safety.get("distance", 999)

    # --- 1. Si no hay detección válida, el ultrasonido manda ---
    det, raw = get_detection(px)
    no_vision = not det.valid_for_search

    # --- 2. Si estamos avanzando recto sin visión → SCAPE inmediato ---
    if no_vision and accion == Cmd.FORWARD_SLOW:
        log_event(px, estado, f"[SEC] SCAPE por avance sin visión (d={d}cm)")
        return estado, Cmd.SCAPE

    # --- 3. Si distancia es crítica (< 20 cm) → SCAPE siempre ---
    if d < 20:
        log_event(px, estado, f"[SEC] CRITICAL: objeto a {d} cm → SCAPE")
        return estado, Cmd.SCAPE

    # --- 4. Si distancia es peligrosa (< 30 cm) y no vemos la baliza → SCAPE ---
    if d < 30 and no_vision:
        log_event(px, estado, f"[SEC] DANGER: sin visión y objeto a {d} cm → SCAPE")
        return estado, Cmd.SCAPE

    return estado, accion

# ============================================================
# FUNCIONES
# ============================================================

def get_detection(px):
    params = Vilib.detect_obj_parameter

    raw = {
        "color_x": params.get("color_x", -1),
        "color_y": params.get("color_y", -1),
        "color_w": params.get("color_w", 0),
        "color_h": params.get("color_h", 0),
        "color_n": params.get("color_n", 0),
    }

    det = Det(
        n = raw["color_n"],
        x = raw["color_x"],
        y = raw["color_y"],
        w = raw["color_w"],
        h = raw["color_h"]
    )

    # 🔥 Filtro anti-fantasma
    if det.n >= 1 and det.w == 0 and det.h == 0:
        det.n = 0

    # Log solo cuando aparece una detección nueva
    if raw["color_n"] > 0 and px.last_raw_n == 0:
        log_event(
            px, px.estado_actual,
            f"Det valid_for_search={det.valid_for_search} "
            f"is_centered={det.is_centered} "
            f"n={raw['color_n']} w={raw['color_w']} h={raw['color_h']} "
            f"area={det.area} x={raw['color_x']} y={raw['color_y']} "
            f"error_x={det.error_x} error_y={det.error_y}"
        )

    px.last_raw_n = 1 if raw["color_n"] > 0 else 0

    return det, raw

def log_det(px, estado, det, raw, prefix=""):
    msg = (
        f"{prefix}"
        f"valid_for_search={det.valid_for_search} "
        f"valid_for_near={det.valid_for_near} "
        f"is_centered={det.is_centered} "
        f"n={raw['color_n']} w={raw['color_w']} h={raw['color_h']} "
        f"area={det.area} x={raw['color_x']} y={raw['color_y']} "
        f"error_x={det.error_x} error_y={det.error_y}"
    )
    log_event(px, estado, msg)

def do_yes(px, estado=Estado.NEAR):
    
    try:
        log_event(px, estado, "Ejecutando gesto de 'SI' (tilt arriba-abajo)")
        for _ in range(2):
            # Gesto hacia arriba
            px.set_cam_tilt_angle(TILT_MAX)
            time.sleep(0.12)

            # Gesto hacia abajo
            px.set_cam_tilt_angle(TILT_MIN)
            time.sleep(0.12)

            # Volver al centro
            px.set_cam_tilt_angle(0)
            time.sleep(0.12)

    except Exception as e:
        print(f"[WARN] Error en gesto de 'sí': {e}")


# ============================================================
# ESTADOS
# ============================================================

def state_idle(px):
    log_event(px, Estado.IDLE, "Entrando en IDLE")
    return Estado.RESET, Cmd.STOP

def state_reset(px):
    log_event(px, Estado.RESET, "Entrando en RESET")
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    px.set_dir_servo_angle(0)
    px.last_pan = 0
    px.last_tilt = 0

    return Estado.SEARCH, Cmd.STOP

def state_search(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # ------------------------------------------------------------
    # Entrada al estado SEARCH
    # ------------------------------------------------------------
    if px.last_state != Estado.SEARCH:
        log_event(px, Estado.SEARCH, "Entrando en SEARCH")

        px.search_seen = 0
        robot_state.search_no_det_frames = 0   # ← ahora sí, en robot_state

        px.last_state = Estado.SEARCH

        # Cámara al centro
        px.set_cam_pan_angle(0)
        px.last_pan = 0
        px.set_cam_tilt_angle(0)
        px.last_tilt = 0

    # ------------------------------------------------------------
    # Seguridad
    # ------------------------------------------------------------
    estado, accion = apply_safety(px, update_safety(px), estado, accion)
    if estado != Estado.SEARCH:
        return estado, accion

    # ------------------------------------------------------------
    # 1. Si hay detección válida → RECENTER
    # ------------------------------------------------------------
    if det.valid_for_search:

        px.search_seen += 1
        robot_state.search_no_det_frames = 0   # reset del plan B

        # Centrada → RECENTER
        if abs(det.error_x) < 40 and px.search_seen >= 2:
            log_det(px, estado, det, raw, prefix="Baliza encontrada → RECENTER | ")
            return Estado.RECENTER, Cmd.STOP

        # Corregir con cámara
        if det.error_x > 40:
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT
        if det.error_x < -40:
            return Estado.SEARCH, Cmd.CAM_PAN_LEFT

        return Estado.SEARCH, Cmd.STOP

    # ------------------------------------------------------------
    # 2. NO hay detección → plan A y plan B
    # ------------------------------------------------------------
    px.search_seen = 0
    robot_state.search_no_det_frames += 1

    # 🔥 PLAN B: búsqueda activa si llevamos mucho sin ver nada
    if robot_state.search_no_det_frames > 20:
        robot_state.search_no_det_frames = 0
        px.set_dir_servo_angle(25)
        return Estado.SEARCH, Cmd.FORWARD_SLOW

    # ------------------------------------------------------------
    # PLAN A: barrido suave con cámara
    # ------------------------------------------------------------
    if px.last_pan >= PAN_MAX:
        px.search_dir = -1
    elif px.last_pan <= PAN_MIN:
        px.search_dir = 1

    return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if px.search_dir == 1 else Cmd.CAM_PAN_LEFT


def state_recenter(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # Entrada al estado
    if px.last_state != estado:
        log_event(px, estado, "Entrando en RECENTER")
        robot_state.recenter_centered_frames = 0
        robot_state.recenter_lost_frames = 0
        px.set_dir_servo_angle(0)
        px.last_state = estado

    # Seguridad
    estado, accion = apply_safety(px, update_safety(px), estado, accion)
    if estado != Estado.RECENTER:
        return estado, accion

    # ------------------------------------------------------------
    # 1. Si NO hay detección válida → tolerar 5 frames
    # ------------------------------------------------------------
    if not det.valid_for_search:
        robot_state.recenter_lost_frames += 1
        if robot_state.recenter_lost_frames >= 5:
            log_det(px, estado, det, raw, prefix="RECENTER sin detección → SEARCH ")
            return Estado.SEARCH, Cmd.STOP
        return Estado.RECENTER, Cmd.STOP

    robot_state.recenter_lost_frames = 0

    # ------------------------------------------------------------
    # 2. Corrección horizontal (PAN)
    # ------------------------------------------------------------
    if abs(det.error_x) > 40:
        robot_state.recenter_centered_frames = 0
        if det.error_x > 0:
            return Estado.RECENTER, Cmd.CAM_PAN_RIGHT
        else:
            return Estado.RECENTER, Cmd.CAM_PAN_LEFT

    # ------------------------------------------------------------
    # 4. Centrado → acumular frames
    # ------------------------------------------------------------
    # 🔥 Si la cámara está en el límite y la baliza sigue visible
    if (px.last_pan == PAN_MAX or px.last_pan == PAN_MIN) and det.valid_for_search:

        if abs(det.error_x) > 120 and det.area > 12000:
            log_event(px, estado, "PAN límite + baliza lateral → micro-backward FSM")
            robot_state.just_recentered = time.time()
            return Estado.RECENTER, Cmd.BACKWARD

        # Caso normal → pasar a TRACK
        log_event(px, estado, "PAN en límite → pasar a TRACK para corregir con ruedas")
        robot_state.just_recentered = time.time()
        return Estado.TRACK, Cmd.FORWARD_SLOW

    robot_state.recenter_centered_frames += 1

    if robot_state.recenter_centered_frames >= 2:
        log_event(px, estado, f"px.last_pan={px.last_pan} Alineado ✔ (cuerpo)")
        robot_state.just_recentered = time.time()
        return Estado.TRACK, Cmd.FORWARD_SLOW

    return Estado.RECENTER, accion

def state_track(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # Cooldown tras RECENTER
    if robot_state.just_recentered:
        if time.time() - robot_state.just_recentered < 0.3:
            return Estado.TRACK, Cmd.FORWARD_SLOW
        robot_state.just_recentered = None

    # 0. Si no hay detección → SEARCH
    if not det.valid_for_search:
        robot_state.track_lost_frames += 1
        if robot_state.track_lost_frames >= 3:
            log_det(px, Estado.TRACK, det, raw, prefix="Perdida baliza → SEARCH | ")
            return Estado.SEARCH, Cmd.STOP
        return Estado.TRACK, Cmd.FORWARD_SLOW
    robot_state.track_lost_frames = 0

    # 1. Si está cerca → NEAR
    if det.valid_for_near:
        robot_state.near_enter_frames += 1
        if robot_state.near_enter_frames >= 3:
            log_det(px, Estado.TRACK, det, raw, prefix="NEAR confirmado (3 frames) → NEAR | ")
            return Estado.NEAR, Cmd.STOP
        return Estado.TRACK, Cmd.FORWARD_SLOW
    robot_state.near_enter_frames = 0

    # 2. Corrección lateral proporcional
    if abs(det.error_x) > 40:
        # Normalizar error_x a rango 0–1
        k = min(abs(det.error_x) / 160, 1.0)

        # Ángulo dinámico entre 10° y 30°
        angle = 10 + k * 20

        # Aplicar giro suave
        px.set_dir_servo_angle(angle if det.error_x < 0 else -angle)
        return Estado.TRACK, Cmd.FORWARD_SLOW

    # 3. Avance recto si está centrado
    px.set_dir_servo_angle(0)
    return Estado.TRACK, Cmd.FORWARD_SLOW

def state_near(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # Entrada al estado NEAR
    if px.last_state != Estado.NEAR:
        log_event(px, Estado.NEAR, "Entrando en NEAR")

        robot_state.near_done_backward = False
        robot_state.near_lost_frames = 0
        robot_state.near_exit_frames = 0
        robot_state.near_did_yes = False
        robot_state.near_cooldown = None

        px.set_dir_servo_angle(0)
        px.set_cam_tilt_angle(0)
        px.last_tilt = 0

        px.last_state = Estado.NEAR

    # Seguridad
    estado, accion = apply_safety(px, update_safety(px), estado, accion)
    if estado != Estado.NEAR:
        return estado, accion

    # 1. Pérdida de baliza
    if not det.valid_for_search:
        robot_state.near_lost_frames += 1
        if robot_state.near_lost_frames >= 5:
            log_event(px, Estado.NEAR, "Baliza perdida en NEAR → SEARCH")
            return Estado.SEARCH, Cmd.STOP
        return Estado.NEAR, Cmd.STOP

    robot_state.near_lost_frames = 0

    # 2. Salida de NEAR
    if not det.valid_for_near:
        robot_state.near_exit_frames += 1
    else:
        robot_state.near_exit_frames = 0

    if robot_state.near_exit_frames >= 5:
        log_det(px, Estado.NEAR, det, raw, prefix="Salida NEAR confirmada (5 frames) → TRACK | ")
        return Estado.TRACK, Cmd.STOP

    # 3. Corrección horizontal
    if abs(det.error_x) > 120:
        return Estado.NEAR, Cmd.CAM_PAN_LEFT if det.error_x < 0 else Cmd.CAM_PAN_RIGHT

    # 4. Backward corto solo una vez
    if not robot_state.near_done_backward:
        robot_state.near_done_backward = True
        robot_state.near_cooldown = time.time()
        return Estado.NEAR, Cmd.BACKWARD

    # Cooldown tras backward (evita conflicto con forward)
    if robot_state.near_cooldown:
        if time.time() - robot_state.near_cooldown < 0.3:
            return Estado.NEAR, Cmd.STOP
        robot_state.near_cooldown = None

    # 5. Gesto de "sí"
    if not robot_state.near_did_yes:
        robot_state.near_did_yes = True
        do_yes(px, estado)
        return Estado.NEAR, Cmd.STOP

    # 6. Estado estable
    return Estado.NEAR, Cmd.STOP

# ============================================================
# BUCLE PRINCIPAL
# ============================================================

def pet_mode(px, test_mode):
    print("🐾 Pet02 mode")

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("=== Start of pet02.log ===\n")

    # music.music_set_volume(20)
    # sound_path = os.path.join(SOUNDS_DIR, "sounds_angry.wav")
    # music.sound_play(sound_path)
    #time.sleep(0.05)

    hello_px(px)
    init_camera(px)
    init_flags(px)
    estado, accion = init_internal_state(px)
    check_robot(px,log_event)
    state = RobotState()
    log_event(px, estado, "Inicio del sistema")

    while True:
        px.estado_actual = estado

        if estado == Estado.IDLE:
            estado, accion = state_idle(px)
        elif estado == Estado.RESET:
            estado, accion = state_reset(px)
        elif estado == Estado.SEARCH:
            estado, accion = state_search(px, estado, accion, state)
        elif estado == Estado.RECENTER:
            estado, accion = state_recenter(px, estado, accion, state)
        elif estado == Estado.TRACK:
            estado, accion = state_track(px, estado, accion, state)
        elif estado == Estado.NEAR:
            estado, accion = state_near(px, estado, accion, state)

        execute_motion(px, estado, accion, test_mode)

        time.sleep(0.1)

# ============================================================
# ENTRYPOINT
# ============================================================

if __name__ == "__main__":
    import sys
    import os
    from picarx import Picarx
    from libs import get_px

    modo = "sim"
    if len(sys.argv) > 1:
        modo = sys.argv[1].lower()

    test_mode = (modo != "real")
    print(f"Arrancando pet02.py en modo: {modo.upper()}")

    px = get_px()
    px.test_mode = test_mode
    pet_mode(px, test_mode=test_mode)
