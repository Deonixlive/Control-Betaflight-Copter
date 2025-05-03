import asyncio
import serial_asyncio

class EchoClient(asyncio.Protocol):
    def connection_made(self, transport):
        self.transport = transport
        print("Connected.")
        self.transport.write(b'$M<\x00\x64\x64')
        print("Command sent.")

    def data_received(self, data):
        print(f"Received: {data.hex()}")

    def connection_lost(self, exc):
        print("Port closed")

async def main():
    loop = asyncio.get_running_loop()
    await serial_asyncio.create_serial_connection(
        loop, EchoClient, '/dev/ttyACM0', baudrate=1_000_000
    )

    while True:
        await asyncio.sleep(1)

asyncio.run(main())