import asyncio
import websockets
import sys
import re
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# Configuración por defecto
DEFAULT_IP = "192.168.0.10"
PORT = 81

# Configuración de la gráfica (últimos 50 puntos)
MAX_POINTS = 50
time_data = deque(maxlen=MAX_POINTS)
rpm_izq_data = deque(maxlen=MAX_POINTS)
rpm_der_data = deque(maxlen=MAX_POINTS)
pwm_izq_data = deque(maxlen=MAX_POINTS)
pwm_der_data = deque(maxlen=MAX_POINTS)

# Variable para el comando actual
current_cmd = ""

async def receptor(websocket):
    """Tarea para recibir, mostrar y parsear la telemetría del robot"""
    print(f"[RECEPTOR] Escuchando telemetría...")
    count = 0
    try:
        async for message in websocket:
            # Mostrar mensaje crudo en consola
            # print(f"\r[TELEMETRÍA] {message}", end="")
            
            # Parsear RPM y PWM usando expresiones regulares
            # Formato esperado: Posicion_L:0.00 Posicion_R:0.00 RPM_Izq:0.0 RPM_Der:0.0 PWM_Izq:246 PWM_Der:255
            try:
                rpm_match = re.search(r"RPM_Izq:([\d.-]+) RPM_Der:([\d.-]+)", message)
                pwm_match = re.search(r"PWM_Izq:(\d+) PWM_Der:(\d+)", message)
                
                if rpm_match and pwm_match:
                    count += 1
                    time_data.append(count)
                    rpm_izq_data.append(float(rpm_match.group(1)))
                    rpm_der_data.append(float(rpm_match.group(2)))
                    pwm_izq_data.append(int(pwm_match.group(1)))
                    pwm_der_data.append(int(pwm_match.group(2)))
            except Exception as e:
                pass

    except websockets.ConnectionClosed:
        print("\n[RECEPTOR] Conexión cerrada.")

async def emisor(websocket):
    """Tarea para enviar comandos desde la terminal"""
    print("\n" + "="*50)
    print(" COMANDOS: D:dist, A:grados, S (Stop), R (Reset), V:i,d")
    print(" Escribe 'exit' para salir.")
    print("="*50 + "\n")

    loop = asyncio.get_event_loop()
    while True:
        # Usamos run_in_executor para no bloquear el loop de asyncio
        cmd = await loop.run_in_executor(None, input, "Enviar comando > ")
        if cmd.lower() in ['exit', 'quit']:
            plt.close('all')
            break
        await websocket.send(cmd)
        print(f"[EMISOR] Enviado: {cmd}")

def update_plot(frame):
    """Función de actualización para la animación de Matplotlib"""
    if not time_data:
        return ax1, ax2

    # Actualizar gráfica de RPM
    line_rpm_izq.set_data(time_data, rpm_izq_data)
    line_rpm_der.set_data(time_data, rpm_der_data)
    
    # Actualizar gráfica de PWM
    line_pwm_izq.set_data(time_data, pwm_izq_data)
    line_pwm_der.set_data(time_data, pwm_der_data)

    # Ajustar límites de los ejes automáticamente
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    return line_rpm_izq, line_rpm_der, line_pwm_izq, line_pwm_der

async def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    uri = f"ws://{ip}:{PORT}"
    
    print(f"Intentando conectar a {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print("¡Conectado exitosamente!")
            
            # Ejecutar receptor y emisor
            await asyncio.gather(
                receptor(websocket),
                emisor(websocket)
            )
    except Exception as e:
        print(f"Error de conexión: {e}")

def start_async_loop():
    asyncio.run(main())

if __name__ == "__main__":
    # Configuración de la figura de Matplotlib
    plt.style.use('ggplot')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.canvas.manager.set_window_title('Telemetría MrRootBot')

    # Subplot 1: RPM
    line_rpm_izq, = ax1.plot([], [], 'r-', label='RPM Izq')
    line_rpm_der, = ax1.plot([], [], 'b-', label='RPM Der')
    ax1.set_title('Velocidad de Motores (RPM)')
    ax1.set_ylabel('RPM')
    ax1.legend(loc='upper left')

    # Subplot 2: PWM
    line_pwm_izq, = ax2.plot([], [], 'r--', label='PWM Izq')
    line_pwm_der, = ax2.plot([], [], 'b--', label='PWM Der')
    ax2.set_title('Señal de Control (PWM)')
    ax2.set_ylabel('Valor PWM')
    ax2.set_xlabel('Tiempo (muestras)')
    ax2.legend(loc='upper left')

    # Iniciar el hilo de comunicación (asyncio)
    t = threading.Thread(target=start_async_loop, daemon=True)
    t.start()

    # Iniciar la animación (esto bloquea el hilo principal, lo cual es correcto para la GUI)
    ani = FuncAnimation(fig, update_plot, interval=150, blit=False, cache_frame_data=False)
    
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print("\nPrograma finalizado.")
