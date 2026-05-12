# 🧠 PiCarX_Pet – Sistema Autónomo con Lógica de FSM y Fusión Sensorial
**PiCarX_Pet** es un sistema de navegación autónoma para el PiCar-X que transforma al robot en una "mascota" inteligente. A diferencia de un simple seguidor de líneas, este sistema utiliza una **Máquina de Estados Finita (FSM) y Fusión de Sensores (Visión + Ultrasonidos)** para tomar decisiones complejas en tiempo real.

---

## 🎯 Capacidades Avanzadas
🤖 **Gestión de Estados (FSM)**
El comportamiento no es lineal; el robot evalúa su entorno y cambia de estrategia dinámicamente:

- **SEARCH:** Barrido de cámara (PAN) y rastreo circular para localizar la baliza.

- **RECENTER:** Alineación fina entre el chasis y la cámara antes de iniciar la marcha.

- **TRACK:** Seguimiento dinámico con velocidad adaptativa y corrección de dirección Ackerman.

- **NEAR:** Interacción de proximidad con gestos de asentimiento, retroceso de cortesía y modo de espera (IDLE).

- **SCAPE:** Protocolo de emergencia que interrumpe cualquier estado ante una colisión inminente.

📡 **Fusión de Sensores (Robustez)**
El robot no confía en un solo dato. Implementa lógica para escenarios críticos:

- **Predicción por Área:** Si el ultrasonido falla por cercanía extrema (punto ciego), el robot usa el área de la baliza en la cámara para confirmar que está "Cerca".

- **Filtro Anti-Ruido:** Contadores de frames (Lost Frames) para evitar reacciones bruscas ante fallos momentáneos de detección.

- **Velocidad Adaptativa:** Reducción automática de velocidad al detectar obstáculos en el rango de advertencia. 

---

## 📁 Estructura del proyecto

```Code
picarx-projects/
└── autonomous/
    ├── pet02.py         <-- Núcleo: FSM y lógica de control
    ├── libs.py          <-- Abstracción de hardware y utilidades
    ├── pet02.log        <-- Registro detallado de decisiones
    ├── sounds/          <-- Efectos de sonido (WAV)
    └── README.md
```
---
### 📊 Dashboard de Telemetría
El sistema incluye un panel en consola que muestra en tiempo real:

- **Estado Actual vs Anterior**

- **Ángulos de Servos** (Dirección, Pan, Tilt)

- **Error Visual (ERR_X) y Área del Objeto**

- **Distancia Real Filtrada** y alertas de seguridad.

### 🛠️ Especificaciones Técnicas Característica Detalle
**Visión** Vilib (Detección de color/forma proporcional)
**Dirección** Control proporcional limitado para evitar bloqueos mecánicos
**Seguridad** Tres niveles: SAFE, WARNING (velocidad lenta), DANGER (escape)
**Interacción** Gesto "YES" (Tilt) + Timeout a IDLE para ahorro de energía

--- 
### ▶️ Ejecución
Conectar a la Raspberry Pi mediante VS Code Remote‑SSH y ejecutar:
Para iniciar la mascota en modo real:
```bash
python3 pet02.py real
```
Para modo de pruebas (sin movimiento de motores):
```bash
python3 pet02.py sim
```
--- 
### El robot iniciará:

- Comprobación del sistema

- Activación de cámara

- Entrada en modo autónomo

- Búsqueda de la baliza

### ⏹️ Detener
Presiona Ctrl + C en cualquier momento para salir del modo activo.
- El robot se detenga inmediatamente.
- Todos los servos vuelvan a la posición neutral.
- La cámara se apague correctamente.

## 📝 Licencia y Uso

Desarrollado para fines educativos y experimentales en plataformas Raspberry Pi con arquitectura Picar X (Robot Hat V4).
---