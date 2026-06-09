import asyncio
import websockets
import json

async def robot_client():
    uri = "ws://IP_DEL_ESP32:81"
    async with websockets.connect(uri) as websocket:
        print("Conectado al robot")
        
        # Enviar un comando de ejemplo
        await websocket.send("ADELANTE")
        
        try:
            while True:
                # Recibir telemetría
                data = await websocket.recv()
                telemetry = json.loads(data)
                
                print(f"RPM L: {telemetry['rpmL']:.2f} | R: {telemetry['rpmR']:.2f}")
                print(f"Pos L: {telemetry['posL']:.2f} | R: {telemetry['posR']:.2f}")
                print("-" * 30)
                
        except websockets.ConnectionClosed:
            print("Conexión cerrada")

if __name__ == "__main__":
    asyncio.run(robot_client())
