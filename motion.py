# motion.py

# Velocidades recomendadas
SLOW_SPEED = 20
TURN_SPEED = 25
FAST_SPEED = 40


# ============================================================
# ACCIONES BÁSICAS
# ============================================================

def stop(px):
    px.set_motor_speed(0, 0)


def forward(px, speed=FAST_SPEED):
    px.set_motor_speed(speed, speed)


def forward_slow(px):
    px.set_motor_speed(SLOW_SPEED, SLOW_SPEED)


def turn_left(px, speed=TURN_SPEED):
    px.set_motor_speed(-speed, speed)


def turn_right(px, speed=TURN_SPEED):
    px.set_motor_speed(speed, -speed)


# ============================================================
# ACCIONES DE ALTO NIVEL
# ============================================================

def rotate_until(px, angle):
    """
    Gira hasta que px.last_dir alcance 'angle'.
    Requiere que px.last_dir se actualice en pet01.py.
    """
    if px.last_dir < angle:
        turn_right(px)
    else:
        turn_left(px)


def advance_until(px, distance):
    """
    Avanza hasta que el sensor ultrasónico detecte 'distance' cm.
    """
    d = px.ultrasonic.read()
    if d is None or d > distance:
        forward_slow(px)
    else:
        stop(px)


def center_on_beacon(px, det):
    """
    Centra el robot respecto a la baliza usando error_x().
    """
    if det.error_x() > 20:
        turn_right(px)
    elif det.error_x() < -20:
        turn_left(px)
    else:
        stop(px)


# ============================================================
# MAPEO DE COMANDOS
# ============================================================

def execute_motion(px, cmd, test_mode=False):
    if test_mode:
        print(f"[SIM] Ejecutaría: {cmd}")
        return

    if cmd == "STOP":
        stop(px)
    elif cmd == "FORWARD":
        forward(px)
    elif cmd == "FORWARD_SLOW":
        forward_slow(px)
    elif cmd == "TURN_LEFT":
        turn_left(px)
    elif cmd == "TURN_RIGHT":
        turn_right(px)
    else:
        stop(px)
