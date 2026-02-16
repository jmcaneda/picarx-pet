# 🧠 PiCarX_Pet – Sistema Autónomo Modular con Comportamiento de Mascota

El proyecto **PiCarX_Pet** implementa un sistema autónomo de seguimiento visual con comportamientos expresivos inspirados en una mascota.  
El robot detecta una baliza de color, la persigue de forma estable y natural, evita obstáculos, recupera la baliza cuando la pierde y expresa estados mediante gestos y sonidos.

---

## 🎯 Mascota — Seguimiento visual inteligente de una baliza

El robot actúa como una mascota que reconoce, persigue y reacciona ante una baliza roja.  
La **cámara lidera** el comportamiento: detecta, corrige, centra y decide.  
Las **ruedas siguen** las órdenes de la cámara, generando un movimiento suave, estable y animal‑like.

### Características principales

- Detección robusta de baliza mediante Vilib  
- Corrección con cámara y ruedas según error visual  
- Búsqueda activa con giro continuo cuando no ve la baliza  
- Recentrado automático del cuerpo  
- Persecución estable con control de distancia  
- Escape ante obstáculos críticos  
- Gesto de “sí” con sonido de perro cuando alcanza la distancia segura  
- Arquitectura modular y fácil de extender  

---

## 📁 Estructura del proyecto

```Code
picarx-projects/
└── autonomous/
    ├── battery.py
    ├── libs.py
    ├── pet02.log
    ├── pet02.py
    ├── readme-picarx_pet.md
    ├── security.py
    ├── sound.py
    └── sounds/          ← efectos de sonido (wav)
```
### 📄 Archivos clave

| Archivo      | Función |
|--------------|---------|
| **pet02.py** | Lógica completa del robot: FSM, visión, movimiento, gestos y sonido. |
| **sound.py** | Reproducción de sonidos (ladrido, alerta, etc.). |
| **libs.py**  | Utilidades comunes del proyecto. |
| **sounds/**  | Carpeta con efectos de sonido en formato WAV. |
| **pet02.log** | Registro de eventos del robot en tiempo real. |

### ▶️ Ejecución
Conectar a la Raspberry Pi mediante VS Code Remote‑SSH y ejecutar:

```bash
python3 pet02.py
```
### El robot iniciará:

- Comprobación del sistema

- Activación de cámara

- Entrada en modo autónomo

- Búsqueda de la baliza

### ⏹️ Detener
Presiona Ctrl + C en cualquier momento para salir del modo activo.

#### 🐾 Comportamiento del robot

#### 🔍 SEARCH – Búsqueda activa

- Cámara centrada
- Giro continuo del cuerpo
- Inversión de giro para evitar bucles
- Cambio inmediato a RECENTER cuando detecta la baliza

#### 🎯 RECENTER – Alineación

- Corrige orientación del cuerpo
- Mantiene la baliza centrada
- Pasa a TRACK cuando está estable

#### 🐕 TRACK – Persecución

- Avanza hacia la baliza
- Corrige con ruedas o cámara según error
- Mantiene distancia segura
- Si la baliza está demasiado cerca → retrocede
- Si la pierde → vuelve a SEARCH
- Si alcanza la distancia segura → doble gesto de “sí” + sonido de perro

#### 🛑 RESET – Seguridad

- Reposiciona cámara y dirección
- Se activa ante obstáculos críticos
- Vuelve a SEARCH

#### 🔊 Gestos y sonidos

#### El robot expresa estados mediante:

- Gesto doble de “sí” cuando alcanza la distancia segura
- Sonido de perro (sound_dog()) sincronizado con el gesto
- Posibilidad de añadir sonidos de alerta, curiosidad o enfado

#### 🧩 Arquitectura modular

#### El sistema está dividido en módulos independientes:

- Visión → detección de baliza
- FSM → estados y transiciones
- Movimiento → ruedas y servos
- Seguridad → ultrasonidos y SCAPE
- Expresión → gestos y sonidos
- Esto permite extender fácilmente:

nuevos gestos

nuevas emociones

nuevos modos de seguimiento

nuevas balizas o colores

integración con sensores adicionales

## 📝 Licencia

- Uso personal y educativo.
- Modificable libremente.