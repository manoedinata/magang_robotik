# Simple WebSocket server
# that echoes messages to all connected clients
# Default listening on ws://localhost:8765

import asyncio
import websockets

CONNECTIONS = set()

async def echo(websocket):
    # Tambahkan koneksi saat klien terhubung
    CONNECTIONS.add(websocket)
    print(f"Client connected. Total connections: {len(CONNECTIONS)}") # Opsional
    
    try:
        # Loop ini akan berjalan selama koneksi aktif
        async for message in websocket:
            # Kirim pesan ke SEMUA klien yang terhubung
            websockets.broadcast(CONNECTIONS, message)
            
    except websockets.exceptions.ConnectionClosedError:
        print("Client disconnected (ConnectionClosedError).")
    except websockets.exceptions.ConnectionClosedOK:
        print("Client disconnected (ConnectionClosedOK).")
    except Exception as e:
        print(f"An error occurred: {e}")
        
    finally:
        # Apapun yang terjadi (putus normal atau error),
        # hapus klien dari set
        CONNECTIONS.remove(websocket)
        print(f"Client removed. Total connections: {len(CONNECTIONS)}") # Opsional

async def main():
    async with websockets.serve(echo, "localhost", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
