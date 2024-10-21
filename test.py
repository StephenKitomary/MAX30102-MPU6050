import asyncio
from bleak import BleakClient

# MAC address of the ESP32
mac_address = '54:32:04:21:52:E6'  # Replace with your ESP32 MAC address
UUID3 = "0000fff0-0000-1000-8000-00805f9b34fb"  # Replace with the UUID used in the ESP32 code

async def notification_handler(sender, data):
    print(f"Received data: {data.decode('utf-8')}")

async def run():
    async with BleakClient(mac_address) as client:
        print("Connected to ESP32")
        await client.start_notify(UUID3, notification_handler)

        # Keep the connection open
        while True:
            await asyncio.sleep(1)

asyncio.run(run())