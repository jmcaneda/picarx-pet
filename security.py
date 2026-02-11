# security.py
import time

class SecurityState:
    SAFE = "safe"
    WARNING = "warning"
    CRITICAL = "critical"
    CLIFF = "cliff"


class SecurityModule:

    def __init__(
        self,
        px,
        power=20,
        safe_distance=60,
        danger_distance=35,
        cliff_ref=(200, 200, 200),
        max_fail=5
    ):
        self.px = px
        self.power = power
        self.safe_distance = safe_distance
        self.danger_distance = danger_distance

        self.max_fail = max_fail
        self.ultra_fail = 0
        self.cliff_fail = 0

        self.last_state = SecurityState.SAFE
        self.override_active = False

        self.log_fn = None

        self.px.set_cliff_reference(list(cliff_ref))

    # --------------------------------------------------
    # Logging externo
    # --------------------------------------------------
    def attach_logger(self, log_fn):
        self.log_fn = log_fn

    def log(self, msg):
        if self.log_fn:
            self.log_fn(self.px, "SEC", msg)
        else:
            print(msg)

    # --------------------------------------------------
    # Lecturas robustas
    # --------------------------------------------------
    def read_ultrasonic(self):
        try:
            d1 = self.px.get_distance()
            time.sleep(0.01)
            d2 = self.px.get_distance()

            if not d1 or not d2 or d1 <= 0 or d2 <= 0:
                raise ValueError

            self.ultra_fail = 0
            return min(d1, d2)

        except Exception:
            self.ultra_fail += 1
            if self.ultra_fail >= self.max_fail:
                self.log("Ultrasonidos inválidos → SAFE temporal")
            return None

    def read_cliff(self):
        try:
            gm = self.px.get_grayscale_data()
            state = self.px.get_cliff_status(gm)
            self.cliff_fail = 0
            return state

        except Exception:
            self.cliff_fail += 1
            if self.cliff_fail >= self.max_fail:
                self.log("Sensor cliff inválido → SAFE temporal")
            return False

    # --------------------------------------------------
    # Evaluación
    # --------------------------------------------------
    def evaluate(self):
        dist = self.read_ultrasonic()
        cliff = self.read_cliff()

        self.log(f"dist={dist}")
        self.log(f"cliff={cliff}")

        if cliff:
            return SecurityState.CLIFF

        if dist is None:
            return SecurityState.SAFE

        if dist >= self.safe_distance:
            return SecurityState.SAFE

        if dist >= self.danger_distance:
            return SecurityState.WARNING

        return SecurityState.CRITICAL

    # --------------------------------------------------
    # Maniobras
    # --------------------------------------------------
    def avoid_warning(self):
        self.log("⚠️ WARNING → frenado + giro suave")
        self.px.stop()
        time.sleep(0.05)
        self.warning_toggle = getattr(self, "warning_toggle", 1)
        angle = 25 if self.warning_toggle > 0 else -25
        self.warning_toggle *= -1
        self.px.set_dir_servo_angle(angle)
        # self.px.set_dir_servo_angle(25)
        self.px.forward(self.power)
        time.sleep(0.1)
        self.px.stop()
        self.px.set_dir_servo_angle(0)

    def avoid_critical(self):
        self.log("❌ CRITICAL → retroceso")
        self.px.stop()
        time.sleep(0.05)
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(0)
        self.px.backward(self.power + 5)
        time.sleep(0.2)
        self.px.stop()

    def avoid_cliff(self):
        self.log("⚠️ CLIFF → retroceso largo")
        self.px.stop()
        time.sleep(0.05)
        self.px.backward(self.power)
        time.sleep(0.5)
        self.px.stop()
        time.sleep(0.3)

    # --------------------------------------------------
    # API principal
    # --------------------------------------------------
    def update(self):
        state = self.evaluate()
        self.last_state = state

        if state == SecurityState.SAFE:
            self.override_active = False
            self.log("SAFE")
            return True

        if state == SecurityState.WARNING:
            self.override_active = True
            self.avoid_warning()
            return True   # ⚠️ NO bloquea

        if state == SecurityState.CRITICAL:
            self.override_active = True
            self.avoid_critical()
            return False  # ⛔ bloquea

        if state == SecurityState.CLIFF:
            self.override_active = True
            self.avoid_cliff()
            return False  # ⛔ bloquea

        return True
