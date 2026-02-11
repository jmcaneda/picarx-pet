# safety.py

SAFE_DISTANCE = 45      # cm
DANGER_DISTANCE = 25    # cm
MAX_DISTANCE = 300      # filtro de ruido

def read_distance(px):
    """Lee distancia del sensor ultrasónico con filtro básico."""
    d = px.ultrasonic.read()

    # Filtro de valores imposibles
    if d < 0 or d > MAX_DISTANCE:
        return None

    return d


def update_safety(px, logger=None):
    """
    Actualiza el estado de seguridad del robot.
    Aplica:
      - filtro de ruido
      - buffer de suavizado
      - histéresis
      - debounce
    """

    # Inicializar atributos si no existen
    if not hasattr(px, "safety_buffer"):
        px.safety_buffer = []
    if not hasattr(px, "last_safety_state"):
        px.last_safety_state = None

    # Leer distancia
    d = read_distance(px)
    if d is None:
        return px.last_safety_state

    # Añadir al buffer
    px.safety_buffer.append(d)
    if len(px.safety_buffer) > 5:
        px.safety_buffer.pop(0)

    # Promedio suavizado
    avg_d = sum(px.safety_buffer) / len(px.safety_buffer)

    # Determinar estado con histéresis
    if avg_d > SAFE_DISTANCE + 5:
        safety_state = "SAFE"
    elif avg_d > DANGER_DISTANCE + 2:
        safety_state = "NEAR"
    else:
        safety_state = "CRITICAL"

    # Si no cambia, no logueamos
    if safety_state == px.last_safety_state:
        return safety_state

    # Loguear cambio de estado
    if logger:
        if safety_state == "SAFE":
            logger(px, px.estado_actual, f"[SEC] Safe zone, object > {avg_d:.2f} cm")
        elif safety_state == "NEAR":
            logger(px, px.estado_actual, "[SEC] WARNING: object near")
        else:
            logger(px, px.estado_actual, f"[SEC] CRITICAL: object < {avg_d:.2f} cm")

    px.last_safety_state = safety_state
    return safety_state
