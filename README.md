# MrRootBot - Control por WebSocket y Python

Este proyecto implementa un sistema de control inalámbrico para un robot diferencial basado en ESP32, utilizando WebSockets para una comunicación de baja latencia y telemetría en tiempo real.

## 🚀 Características
- **Comunicación:** Servidor WebSocket en el ESP32 (Puerto 81).
- **Control:** Interfaz interactiva en Python con comandos de texto.
- **Telemetría:** Visualización en tiempo real de RPM y PWM mediante Matplotlib.
- **Equilibrio:** Sistema de control automático para mantener ambas ruedas sincronizadas a velocidad máxima base.

## 🛠 Requisitos

### Hardware
- ESP32 (Cualquier variante compatible con Arduino).
- Puente H (L298N o similar) para los motores.
- Encoders de cuadratura en ambos motores.

### Software (Arduino)
Instalar las siguientes librerías desde el Gestor de Librerías:
1. **WebSockets** (por Markus Sattler / Links2004).

### Software (Python)
Instalar las dependencias necesarias:
```bash
pip install websockets matplotlib
```

## 📂 Estructura del Proyecto
- `Rmejorado.ino`: Código principal del robot (WiFi, WebSocket y lógica de comandos).
- `control.ino`: Lógica de control PWM y balance de motores.
- `python_test/interfaz_test.py`: Script de Python para control y gráficas en tiempo real.

## ⚙️ Configuración y Ejecución

### 1. Preparar el Robot (Arduino)
1. Abre `Rmejorado.ino` en el IDE de Arduino.
2. Busca las líneas de configuración de red y pon tus datos:
   ```cpp
   const char* ssid = "TU_SSID";
   const char* password = "TU_PASSWORD";
   ```
3. Carga el código al ESP32.
4. Abre el Monitor Serial (115200 baudios) para ver la dirección IP asignada (por defecto debería ser `192.168.0.10` si así lo configuraste en tu router).

### 2. Ejecutar la Interfaz (Python)
Navega a la carpeta del proyecto y ejecuta:
```bash
python python_test/interfaz_test.py
```
*Si la IP del robot es diferente a la de por defecto, ejecútalo así:*
```bash
python python_test/interfaz_test.py 192.168.X.XX
```

## 🎮 Comandos Disponibles
Puedes escribir estos comandos directamente en la terminal de Python:
- `D:dist` -> Mover cierta distancia en cm (ej: `D:50`).
- `A:grados` -> Girar grados específicos (ej: `A:90` para derecha, `A:-90` para izquierda).
- `S` -> Parada de emergencia inmediata.
- `R` -> Resetear los contadores de los encoders.
- `V:izq,der` -> Activar motores detectando dirección (usa velocidad base máxima).
- `ADELANTE` / `ATRAS` / `STOP` -> Comandos directos de movimiento.

## 📊 Protocolo de Telemetría
El robot envía cada 150ms una cadena de texto que el script de Python parsea automáticamente:
`Posicion_L:0.00 Posicion_R:0.00 RPM_Izq:0.0 RPM_Der:0.0 PWM_Izq:246 PWM_Der:255`

---
*Desarrollado para el sistema Rmejorado.*
