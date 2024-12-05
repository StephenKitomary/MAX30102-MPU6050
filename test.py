import asyncio
from bleak import BleakClient

# Replace with your ESP32 MAC address
mac_address = '54:32:04:21:52:E6'
UUID3 = "0000fff0-0000-1000-8000-00805f9b34fb"

async def notification_handler(sender, data):
    decoded_data = data.decode('utf-8')
    print(f"Received data: {decoded_data}")

    # Optionally parse the data (assuming comma-separated values)
    try:
        parts = decoded_data.split(", ")
        movement = [p for p in parts if "Movement" in p][0]
        bpm = [p for p in parts if "BPM" in p][0]
        avg_bpm = [p for p in parts if "Avg BPM" in p][0]
        
        print(f"Movement: {movement}, {bpm}, {avg_bpm}")
    except Exception as e:
        print(f"Error parsing data: {e}")

async def run():
    async with BleakClient(mac_address) as client:
        print("Connected to ESP32")
        await client.start_notify(UUID3, notification_handler)
        while True:
            await asyncio.sleep(1)

asyncio.run(run())
