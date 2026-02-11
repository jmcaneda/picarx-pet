# 🧠 PiCar‑X – Sistema Autónomo Modular
Este proyecto implementa un conjunto de modos autónomos para el robot PiCar‑X, gestionados desde un menú interactivo en main.py.
Cada modo está encapsulado en su propio módulo y puede activarse dinámicamente mediante teclado.

## 🚦 Modos disponibles
Modo	Descripción
1. Patrulla	Navegación autónoma con lógica de evitación de obstáculos basada en sensores ultrasónicos.
2. Carga	(Reservado para futuras funciones de gestión de batería o retorno a base).
3. Científico	(Modo experimental para pruebas o extensiones futuras).
4. Mascota	Seguimiento visual inteligente de una baliza de color. La cámara lidera, las ruedas siguen.
7. Monitor de batería	Lectura continua del voltaje simulado o real, con lógica de apagado seguro.
8. Salir	Finaliza el gestor de modos.
9. Apagar PiCar‑X	Apagado seguro del sistema mediante shutdown.
## 📁 Estructura del proyecto

```Code
picarx-projects/
└── autonomous/
    ├── battery.py
    ├── libs.py
    ├── main.py          ← punto de entrada principal
    ├── patrol.py
    ├── pet.py
    ├── readme.md
    ├── security.py
    ├── sound.py
    ├── sounds/          ← efectos de sonido
    └── .vscode/         ← configuración de entorno
```
## ▶️ Ejecución
Conectar a la Raspberry Pi mediante VS Code Remote‑SSH y ejecutar:

```bash
python3 main.py
```

Se mostrará un menú interactivo.
Selecciona el modo deseado presionando la tecla correspondiente.

## ⏹️ Detener
Presiona Ctrl+C en cualquier momento para salir del modo activo.
El sistema realiza limpieza segura de servos, cámara y GPIO antes de finalizar.

## 🛡️ Seguridad integrada
#### El módulo security.py se encarga de:

- detener el robot si se detecta riesgo

- controlar la distancia mínima

- evitar colisiones

- permitir que otros modos se centren en lógica de comportamiento