import asyncio
from bleak import BleakClient

mac_address = '54:32:04:21:52:E6'
UUID3 = "0000fff0-0000-1000-8000-00805f9b34fb"

async def notification_handler(sender, data):
    decoded_data = data.decode('utf-8')
    print(f"Received data: {decoded_data}")
    # Add further processing of accelerometer and heart rate data if necessary

async def run():
    async with BleakClient(mac_address) as client:
        print("Connected to ESP32")
        await client.start_notify(UUID3, notification_handler)
        while True:
            await asyncio.sleep(1)

asyncio.run(run())
