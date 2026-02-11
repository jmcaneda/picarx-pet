# battery.py
import time
import os
import sound
from smbus2 import SMBus

# ============================
# FUNCIONES DE LECTURA I2C
# ============================

def fake_battery(bus, address):
    """Simulación temporal del voltaje."""
    return 6.2

def read_ina219(bus, address):
    """Ejemplo simplificado de lectura INA219."""
    try:
        bus.write_word_data(address, 0x02, 0)  # Registro de voltaje
        msb = bus.read_byte(address)
        lsb = bus.read_byte(address)
        raw = (msb << 8) | lsb
        voltage = raw * 0.001  # Conversión ficticia
        return round(voltage, 2)
    except Exception as e:
        return f"Error INA219: {e}"

# ============================
# MENÚ DE CONFIRMACIÓN
# ============================

def confirmar_apagado(voltage):
    print(f"\n❌ Batería crítica: {voltage:.1f} V")
    print("¿Deseas apagar el PiCar‑X?")
    print("1. Sí, apagar ahora")
    print("2. No, continuar sin apagar")
    print("3. Volver al menú principal")

    while True:
        opcion = input("Selecciona una opción: ").strip()

        if opcion == "1":
            print("Apagando sistema…")
            time.sleep(1)
            os.system("sudo shutdown now")
            return "apagado"

        elif opcion == "2":
            print("Continuando sin apagar. ¡Precaución: batería muy baja!")
            return "continuar"

        elif opcion == "3":
            print("Volviendo al menú principal…")
            return "menu"

        else:
            print("Opción no válida. Intenta de nuevo.")

# ============================
# CLASE MONITOR DE BATERÍA
# ============================

class BatteryMonitor:
    def __init__(self, address, read_fn):
        self.address = address
        self.read_fn = read_fn
        self.bus = SMBus(1)
        self.last_warning = 0
        self.critical_confirm_time = 0

    def get_voltage(self):
        """Devuelve el voltaje leído por la función asignada."""
        value = self.read_fn(self.bus, self.address)
        if isinstance(value, str):
            return value
        return float(value)

    def check(self):
        voltage = self.get_voltage()

        if isinstance(voltage, str):
            return voltage

        # Nivel bajo
        if 6.6 <= voltage < 7.0:
            now = time.time()
            if now - self.last_warning > 10:
                sound.sound_caution()
                print(f"⚠️ Batería baja: {voltage} V")
                self.last_warning = now

        # Nivel crítico
        elif voltage < 6.6:
            now = time.time()

            if self.critical_confirm_time == 0:
                self.critical_confirm_time = now
                print(f"❗ Batería crítica detectada ({voltage} V). Confirmando…")
                return voltage

            if now - self.critical_confirm_time > 3:
                sound.sound_danger()
                accion = confirmar_apagado(voltage)

                self.critical_confirm_time = 0  # reset SIEMPRE

                return accion  # <-- devolvemos "apagado", "continuar" o "menu"

        else:
            self.critical_confirm_time = 0

        return voltage

    def close(self):
        self.bus.close()
