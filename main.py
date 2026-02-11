# main.py

from time import sleep
import os
import readchar

from libs import clear_screen, cleanup_fifos, get_px

from sound import sound_normal, sound_angry

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

from robot_hat import utils

from pet import pet_mode

# Instancia global del PiCar-X
px = None

def menu():
    print("\n=== Selección de modo ===")
    print("1. Modo patrulla")
    print("2. Modo carga")
    print("3. Modo científico")
    print("4. Modo mascota")
    print("7. Mostrar estado de batería")
    print("8. Salir")
    print("9. Apagar PiCar-X")
    print("===========================")


def show_info():
    clear_screen()
    menu()


def main():
    global px

    try:
        # Limpieza de FIFOs residuales
        cleanup_fifos()

        # Inicialización segura del PiCar-X (solo una vez)
        if px is None:
            px = get_px()

            # ============================================
            # MEMORIA DE DETECCIÓN (añadido por Copilot)
            # ============================================
            px.last_det = None
            px.last_det_age = 999   # 999 = sin detección reciente
            # ============================================

        show_info()

        while True:
            try:
                key = readchar.readkey()

                if len(key) == 1:
                    key = key.lower()

                if key in ('1', '2', '3', '4', '7', '8', '9'):

                    if key == '1':
                        print("1 Modo patrulla seleccionado.")
                        patrol_mode(px)
                        show_info()

                    elif key == '2':
                        print("2 Modo carga seleccionado.")

                    elif key == '3':
                        print("3 Modo científico seleccionado.")

                    elif key == '4':
                        print("4 Modo mascota seleccionado.")
                        pet_mode(px, True)   # modo simulado True, modo real False
                        show_info()

                    elif key == '7':
                        print("7 Monitor de batería (Ctrl+C para salir).")
                        bm = BatteryMonitor(address=0x00, read_fn=fake_battery)

                        try:
                            while True:
                                result = bm.check()

                                if result == "menu":
                                    bm.critical_confirm_time = 0
                                    show_info()
                                    break

                                elif result == "apagado":
                                    return

                                elif isinstance(result, (int, float)):
                                    print("Voltaje:", result)

                                sleep(1)

                        except KeyboardInterrupt:
                            print("Saliendo del monitor de batería.")

                    elif key == '8':
                        print("8 Saliendo del gestor de modos.")
                        Vilib.camera_close()
                        break

                    elif key == '9':
                        print("9 Apagado seguro del sistema.")
                        Vilib.camera_close()
                        sleep(0.5)
                        os.system("sudo shutdown -h now")

                else:
                    print(f"{key} Opción no válida.")

            except KeyboardInterrupt:
                print("\nInterrupción detectada. Saliendo.")
                Vilib.camera_close()
                break

    finally:
        # Limpieza segura del PiCar-X
        if px is not None:
            try:
                px.set_cam_tilt_angle(0)
                px.set_cam_pan_angle(0)
                px.set_dir_servo_angle(0)
                px.stop()
            except:
                pass

            sleep(0.2)

            try:
                utils.reset_mcu()
            except:
                pass

            try:
                GPIO.cleanup()
            except:
                pass


if __name__ == "__main__":
    main()
