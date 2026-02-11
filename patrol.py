# patrol.py
import time
from security import SecurityModule

def patrol_mode(px):
    """
    Modo patrulla básico.
    Avanza continuamente mientras el módulo de seguridad lo permita.
    """
    security = SecurityModule(px, power=20)

    print("🚓 Iniciando modo patrulla básico… (Ctrl+C para salir)")

    try:
        while True:
            # El módulo de seguridad decide si es seguro avanzar
            if security.update():
                px.forward(security.power)
            else:
                # El módulo de seguridad ya tomó el control
                # Aquí no hacemos nada, solo esperamos
                pass

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nSaliendo del modo patrulla…")
        px.stop()
        time.sleep(0.1)
