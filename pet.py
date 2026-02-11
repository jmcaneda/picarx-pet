####################################################
# pet.py - Autonomous "pet" behavior for PiCar-X
####################################################

import os
import time
from vilib import Vilib
from sound import sound_normal, sound_angry
from libs import hello_px, check_robot
from enum import Enum

# ============================
# STATES
# ============================

class Estado(Enum):
    IDLE = 1
    RESET = 2
    SEARCH = 3
    TRACK = 4
    RECENTER = 5

# ============================
# CONFIGURATION
# ============================

BALIZA_COLOR = "red"

CX = 160   # image center X (for 320x240)
CY = 120   # image center Y

DRIVE_POWER = 20

LOG_PATH = os.path.join(os.path.dirname(__file__), "pet.log")

# ============================
# SAFETY CONFIGURATION (ULTRASONIC ONLY)
# ============================

SAFE_DISTANCE = 40     # cm - safe zone
DANGER_DISTANCE = 20   # cm - hard stop zone

# ============================
# LOGGING (COMPACT, FILTERED)
# ============================

_last_raw_sig = None
_last_filter_sig = None

def log_event(px, estado, mensaje=""):
    """
    Compact logging with basic filtering and anti-duplicates.
    Uses Vilib.detect_obj_parameter for detection context.
    """
    global _last_raw_sig, _last_filter_sig

    params = Vilib.detect_obj_parameter
    timestamp = time.strftime("%H:%M:%S", time.localtime())

    x = params.get("color_x", -1)
    y = params.get("color_y", -1)
    w = params.get("color_w", 0)
    h = params.get("color_h", 0)
    n = params.get("color_n", 0)

    # 1) Filter: only important events
    importante = (
        "[OK]" in mensaje or
        "[MEM]" in mensaje or
        "RECENTER" in mensaje or
        "ESCAPE" in mensaje or
        "Baliza perdida" in mensaje or
        "Baliza encontrada" in mensaje or
        "Iniciando" in mensaje or
        "Entrando" in mensaje or
        "STOP" in mensaje or
        "FORWARD" in mensaje or
        "WARNING" in mensaje or
        "CRITICAL" in mensaje or
        "CLIFF" in mensaje or
        "SIM" in mensaje or
        "[SEC]" in mensaje or
        "[CHK]" in mensaje
    )

    if not importante:
        return

    # 2) Anti-duplicates (state + message + DET)
    det_sig = (n, x, y, w, h)
    sig = (estado, mensaje, det_sig)

    if hasattr(px, "_last_log"):
        if px._last_log == sig:
            return
    px._last_log = sig

    # 3) Optional RAW/FILTER signatures
    if "[RAW]" in mensaje:
        raw_sig = det_sig
        if raw_sig == _last_raw_sig:
            return
        _last_raw_sig = raw_sig

    if "[FILTRO]" in mensaje:
        filter_sig = mensaje
        if filter_sig == _last_filter_sig:
            return
        _last_filter_sig = filter_sig

    # 4) Build log line
    pan = getattr(px, "last_pan", 0)
    tilt = getattr(px, "last_tilt", 0)

    if n > 0 and w > 15:
        det = f"DET x={x} y={y} w={w} h={h}"
    else:
        det = "DET None"

    estado_str = estado.name if hasattr(estado, "name") else str(estado)
    line = f"[{timestamp}] [{estado_str}] PAN={pan:.1f} TILT={tilt:.1f} {det} {mensaje}"

    print(line)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line + "\n")


# ============================
# CAMERA INITIALIZATION
# ============================

def init_camera():
    """
    Start camera, enable local + web display and color detection.
    """
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=True, web=True)
    Vilib.color_detect(BALIZA_COLOR)
    time.sleep(1)


# ============================
# ROBOT RESET
# ============================

def reset_robot(px):
    """
    Reset internal state and physical actuators.
    """
    # Camera internal state
    px.last_pan = 0
    px.last_tilt = 0

    # Body direction state
    px.last_dir = 0

    # Vision counters
    px.visible_counter = 0
    px.lost_counter = 0

    # Turn flags
    px.turning = False
    px.turning_action = None

    # TRACK counters
    px.track_escape_ticks = 0

    # RECENTER
    px.recenter_dir = None
    px.recenter_ticks = 0
    px.post_recenter_ticks = 0

    # SEARCH
    px.search_fail_ticks = 0
    px.escape_ticks = 0

    # Detection memory
    px.last_det = None
    px.last_det_age = 999
    px.stable_counter = 0
    px.last_safety_state = None

    # Safety flags (ultrasonic)
    px.distance = 999.0
    px.obstacle_very_close = False
    px.obstacle_near = False

    # Physical actuators
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    px.set_dir_servo_angle(0)

    time.sleep(0.2)


# ============================
# ROBUST BEACON DETECTION WITH MEMORY
# ============================

def baliza_visible(px, estado):
    """
    Robust beacon detection with basic shape/area filters and short-term memory.
    Returns True if:
      - a valid detection is present, or
      - a recent detection (memory) is still considered usable.
    Updates px.last_det and px.last_det_age.
    """
    params = Vilib.detect_obj_parameter

    n = params.get("color_n", 0)
    w = params.get("color_w", 0)
    h = params.get("color_h", 0)
    x = params.get("color_x", -1)
    y = params.get("color_y", -1)

    # Ensure stability counter exists
    if not hasattr(px, "stable_counter"):
        px.stable_counter = 0
    if not hasattr(px, "last_det_age"):
        px.last_det_age = 999
    if not hasattr(px, "last_det"):
        px.last_det = None

    # 1) Real detection with basic filters
    if n > 0 and w >= 20 and h >= 20 and (w * h) >= 2500 and 40 <= y <= 260:
        ratio = w / h if h > 0 else 0
        if 0.3 <= ratio <= 2.5:
            # Increase stability counter
            px.stable_counter += 1

            # Require 2 consecutive valid frames
            if px.stable_counter < 2:
                return False

            # Valid detection: store and reset age
            px.last_det = params.copy()
            px.last_det_age = 0
            log_event(px, estado, "[OK] Baliza confirmada estable")
            return True

    # 2) No valid detection: use memory if recent
    px.stable_counter = 0
    px.last_det_age += 1

    if px.last_det is not None and px.last_det_age <= 2:
        log_event(px, estado, "[MEM] Using recent detection")
        return True

    # 3) No valid detection and memory expired
    return False


def get_baliza_position(px):
    """
    Return last known beacon position (x, y) from memory.
    """
    if px.last_det is None:
        return -1, -1
    return px.last_det.get("color_x", -1), px.last_det.get("color_y", -1)


# ============================================================
# Get current detection in a simple object form
# ============================================================

def get_detection():
    """
    Wrap current detection into a simple object with .x, .y, .w, .h.
    Returns None if no valid detection.
    """
    params = Vilib.detect_obj_parameter
    n = params.get("color_n", 0)
    w = params.get("color_w", 0)
    h = params.get("color_h", 0)

    if n == 0 or w < 20 or h < 20:
        return None

    class Det:
        pass

    det = Det()
    det.x = params.get("color_x", -1)
    det.y = params.get("color_y", -1)
    det.w = w
    det.h = h
    return det


# ============================
# CAMERA SCAN (zigzag)
# ============================

def scan_camera(px, estado):
    """
    Zigzag scan with pan/tilt to search for the beacon.
    Returns (found: bool, det: dict or None).
    """
    pan_min, pan_max = -40, 40
    tilt_min, tilt_max = -10, 20

    pan_step = 3
    tilt_step = 5

    pan = int(px.last_pan)
    tilt = int(px.last_tilt)

    full_scans = 0

    log_event(px, estado, "Full scan (zigzag)")

    while full_scans < 2:

        # Ensure tilt within limits
        tilt = max(tilt_min, min(tilt_max, tilt))
        px.set_cam_tilt_angle(tilt)
        time.sleep(0.1)

        # RIGHT sweep
        for pan in range(pan, pan_max + 1, pan_step):
            px.set_cam_pan_angle(pan)
            time.sleep(0.05)

            if baliza_visible(px, estado):
                params = Vilib.detect_obj_parameter
                det = params if params.get("color_n", 0) > 0 else None

                if det is not None:
                    log_event(px, estado, "Baliza encontrada (derecha)")
                    px.last_pan = pan
                    px.last_tilt = tilt
                    return True, det

        # Next tilt line
        tilt += tilt_step
        if tilt > tilt_max:
            full_scans += 1
            tilt = tilt_min
            pan = pan_min
            continue

        px.set_cam_tilt_angle(tilt)
        time.sleep(0.1)

        # LEFT sweep
        for pan in range(pan_max, pan_min - 1, -pan_step):
            px.set_cam_pan_angle(pan)
            time.sleep(0.05)

            if baliza_visible(px, estado):
                det = px.last_det.copy() if px.last_det is not None else None

                if det is not None:
                    log_event(px, estado, "Baliza encontrada (izquierda)")
                    px.last_pan = pan
                    px.last_tilt = tilt
                    return True, det

        # Next tilt line
        tilt += tilt_step
        if tilt > tilt_max:
            full_scans += 1
            tilt = tilt_min
            pan = pan_max
            continue

        px.set_cam_tilt_angle(tilt)
        time.sleep(0.1)

        pan = pan_min

    # End of scan
    px.last_pan = pan
    px.last_tilt = tilt

    log_event(px, estado, "Scan failed after 2 cycles")
    return False, None


# ============================
# STATE: IDLE
# ============================

def state_idle(px):
    """
    IDLE: entry state, immediately transitions to RESET.
    """
    log_event(px, Estado.IDLE, "Entering IDLE")
    px.lost_counter = 0
    return Estado.RESET


# ============================
# STATE: RESET
# ============================

def state_reset(px):
    """
    RESET: reset camera and internal state, then go to SEARCH.
    """
    log_event(px, Estado.RESET, "Camera reset")
    reset_robot(px)
    return Estado.SEARCH


# ============================
# STATE: SEARCH
# ============================

def state_search(px):
    """
    SEARCH: perform camera scan to find the beacon.
    """
    log_event(px, Estado.SEARCH, "Starting SEARCH")

    found, det = scan_camera(px, Estado.SEARCH)

    if found and det is not None:
        log_event(px, Estado.SEARCH, "Beacon found, switching to TRACK")
        sound_angry()
        return Estado.TRACK

    log_event(px, Estado.SEARCH, "SEARCH failed, repeat")
    return Estado.SEARCH


# ============================================================
# GLOBAL MOVEMENT SAFETY FILTER (ULTRASONIC ONLY)
# ============================================================

def aplicar_seguridad_movimiento(px, estado, cmd):
    """
    Final safety filter before executing movement.
    Uses only ultrasonic distance and internal flags.
    """
    # Hard stop if very close obstacle
    if px.obstacle_very_close:
        if cmd != "STOP":
            log_event(px, estado, "[SEC] CRITICAL: obstacle very close, forcing STOP")
        return estado, "STOP"

    return estado, cmd


# ============================================================
# STATE: TRACK (follow beacon)
# ============================================================

def state_track(px, det):
    """
    TRACK: follow the beacon using last detection and memory.
    """
    # 0) Update detection memory
    if det is not None:
        px.last_detection = det
        px.last_area = det.w * det.h
    else:
        det = getattr(px, "last_detection", None)

    # 1) Is beacon visible (real or memory)?
    if not baliza_visible(px, Estado.TRACK):
        px.lost_counter += 1
        px.last_area = 0

        # Short grace period: STOP only
        if px.lost_counter <= 2:
            log_event(px, Estado.TRACK, "Beacon lost briefly, STOP")
            estado, cmd = Estado.TRACK, "STOP"
            return aplicar_seguridad_movimiento(px, estado, cmd)

        # One short ESCAPE backward
        if px.lost_counter == 3:
            if px.track_escape_ticks == 0:
                px.track_escape_ticks = 1
                log_event(px, Estado.TRACK, "Beacon lost near, short ESCAPE BACKWARD")
                estado, cmd = Estado.TRACK, "BACKWARD"
                return aplicar_seguridad_movimiento(px, estado, cmd)
            else:
                log_event(px, Estado.TRACK, "ESCAPE done, STOP")
                estado, cmd = Estado.TRACK, "STOP"
                return aplicar_seguridad_movimiento(px, estado, cmd)

        # Too long without beacon: go back to SEARCH
        log_event(px, Estado.TRACK, "Beacon lost, switching to SEARCH")
        px.track_escape_ticks = 0
        px.lost_counter = 0
        estado, cmd = Estado.SEARCH, "STOP"
        return aplicar_seguridad_movimiento(px, estado, cmd)

    # Beacon visible
    px.lost_counter = 0
    px.track_escape_ticks = 0

    # 2) Simple zone + steering logic
    cmd = "STOP"
    estado = Estado.TRACK

    # Use last known detection (from memory or current)
    if det is None:
        det = getattr(px, "last_detection", None)

    if det is not None:
        area = det.w * det.h
        error_x = det.x - CX

        # Basic steering: turn if beacon is far from center
        if error_x > 25:
            cmd = "TURN_RIGHT"
        elif error_x < -25:
            cmd = "TURN_LEFT"
        else:
            # Centered: decide forward speed based on area (distance)
            if area < 4000:
                cmd = "FORWARD"
            elif area < 8000:
                cmd = "FORWARD_SLOW"
            else:
                cmd = "STOP"

    log_event(px, Estado.TRACK, f"TRACK command: {cmd}")
    return aplicar_seguridad_movimiento(px, estado, cmd)


# ============================================================
# STATE: RECENTER (align body with camera)
# ============================================================

def state_recenter(px, det):
    """
    RECENTER: align body with camera when pan is high.
    """
    if det is not None:
        px.last_detection = det
        px.last_area = det.w * det.h
    else:
        det = getattr(px, "last_detection", None)

    # If beacon not visible, behave similar to TRACK loss
    if not baliza_visible(px, Estado.RECENTER):
        px.lost_counter += 1
        px.last_area = 0

        if px.lost_counter <= 2:
            log_event(px, Estado.RECENTER, "Beacon lost briefly, STOP")
            estado, cmd = Estado.RECENTER, "STOP"
            return aplicar_seguridad_movimiento(px, estado, cmd)

        log_event(px, Estado.RECENTER, "Beacon lost, switching to SEARCH")
        px.lost_counter = 0
        estado, cmd = Estado.SEARCH, "STOP"
        return aplicar_seguridad_movimiento(px, estado, cmd)

    # Beacon visible: reset loss counter
    px.lost_counter = 0

    # Simple RECENTER logic:
    # 1) Move forward slowly while pan returns to near 0
    # 2) When aligned, go back to TRACK
    pan = getattr(px, "last_pan", 0)

    if abs(pan) > 3:
        cmd = "FORWARD_SLOW"
        estado = Estado.RECENTER
        log_event(px, Estado.RECENTER, f"RECENTER moving, pan={pan:.1f}")
    else:
        cmd = "STOP"
        estado = Estado.TRACK
        log_event(px, Estado.RECENTER, "RECENTER done, back to TRACK")

    return aplicar_seguridad_movimiento(px, estado, cmd)


# ============================
# PET MODE (main loop)
# ============================

def pet_mode(px, test_mode):
    """
    Main "pet" behavior loop.
    test_mode=True: simulate movement, real camera.
    test_mode=False: real movement + camera.
    """
    print("🐾 Pet mode")

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("=== Start of pet.log ===\n")

    sound_normal()
    hello_px(px)

    # Esperar a que libcamera esté lista
    camera_ok = False
    for i in range(10):
        try:
            Vilib.camera_start(vflip=False, hflip=False)
            camera_ok = True
            break
        except Exception as e:
            print(f"Reintentando cámara... {i+1}/10")
            time.sleep(0.5)

    if not camera_ok:
        print("❌ No se pudo iniciar la cámara tras 10 intentos. Entrando en modo SIMULADO.")
        test_mode = True
    else:
        Vilib.display(local=True, web=True)
        Vilib.color_detect(BALIZA_COLOR)
        time.sleep(1)

    reset_robot(px)

    estado = Estado.IDLE
    accion = "STOP"

    log_event(px, estado, "Pet mode started")
    check_robot(px, logger=log_event)

    try:
        while True:
            # ==========================================
            # SAFETY UPDATE (ULTRASONIC)
            # ==========================================
            try:
                distance = round(px.ultrasonic.read(), 2)
            except:
                distance = 999.0

            px.distance = distance
            px.obstacle_very_close = distance < DANGER_DISTANCE
            px.obstacle_near = DANGER_DISTANCE <= distance < SAFE_DISTANCE

            # Determinar estado actual
            if distance >= SAFE_DISTANCE:
                safety_state = "SAFE"
            elif distance >= DANGER_DISTANCE:
                safety_state = "NEAR"
            else:
                safety_state = "CRITICAL"

            # Inicializar memoria si no existe
            if not hasattr(px, "last_safety_state"):
                px.last_safety_state = None

            # Log solo si el estado cambia
            if safety_state != px.last_safety_state:
                if safety_state == "SAFE":
                    log_event(px, estado, f"[SEC] Safe zone, object > {distance} cm")
                elif safety_state == "NEAR":
                    log_event(px, estado, "[SEC] WARNING: object near")
                else:
                    log_event(px, estado, f"[SEC] CRITICAL: object < {distance} cm")

                px.last_safety_state = safety_state


            # ==========================================
            # DECISION
            # ==========================================
            det = get_detection()

            if estado == Estado.IDLE:
                estado = state_idle(px)
                accion = "STOP"

            elif estado == Estado.RESET:
                estado = state_reset(px)
                accion = "STOP"

            elif estado == Estado.SEARCH:
                estado = state_search(px)
                accion = "STOP"

            elif estado == Estado.TRACK:
                estado, accion = state_track(px, det)

            elif estado == Estado.RECENTER:
                estado, accion = state_recenter(px, det)

            # ==========================================
            # LOG MOVEMENT INTENT
            # ==========================================
            log_event(px, estado, f"MOV ? {accion}")

            # ==========================================
            # EXECUTION (SIMULATED)
            # ==========================================
            if test_mode:
                log_event(px, estado, f"[SIM] Simulated action: {accion}")
                time.sleep(0.05)

                # Solo simular acciones que NO implican desplazamiento
                if accion == "TURN_LEFT":
                    px.last_dir -= 5

                elif accion == "TURN_RIGHT":
                    px.last_dir += 5

                elif accion == "STOP":
                    # STOP no altera la dirección, pero lo dejamos explícito
                    pass

                # Ignorar completamente acciones de movimiento lineal
                # FORWARD, FORWARD_SLOW, BACKWARD → no hacer nada

                # Limitar dirección simulada
                px.last_dir = max(-30, min(30, px.last_dir))

                # Simular corrección de pan basada en la dirección
                px.last_pan -= px.last_dir * 0.1
                px.last_pan = max(-45, min(45, px.last_pan))

                continue


            # ==========================================
            # EXECUTION (REAL)
            # ==========================================
            if accion == "FORWARD":
                px.last_dir = 0
                px.set_dir_servo_angle(0)
                px.forward(DRIVE_POWER)

            elif accion == "FORWARD_SLOW":
                px.last_dir = 0
                px.set_dir_servo_angle(0)
                px.forward(10)

            elif accion == "BACKWARD":
                px.last_dir = 0
                px.set_dir_servo_angle(0)
                px.backward(6)

            elif accion == "TURN_LEFT":
                px.last_dir = -20
                px.set_dir_servo_angle(-20)
                px.forward(6)

            elif accion == "TURN_RIGHT":
                px.last_dir = 20
                px.set_dir_servo_angle(20)
                px.forward(6)

            elif accion == "STOP":
                px.last_dir = 0
                px.set_dir_servo_angle(0)
                px.stop()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting pet mode")
        reset_robot(px)

if __name__ == "__main__":
    import sys
    from picarx import Picarx

    # Modo por defecto: simulado
    modo = "sim"

    # Si el usuario escribe: python3 pet.py real
    if len(sys.argv) > 1:
        modo = sys.argv[1].lower()

    # Determinar test_mode
    test_mode = (modo != "real")

    print(f"Arrancando pet.py en modo: {modo.upper()}")

    px = Picarx()
    pet_mode(px, test_mode=test_mode)
