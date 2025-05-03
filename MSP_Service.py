#
# Created on Sat May 03 2025
# Author: Dimetri Chau
# https://github.com/Deonixlive
#
# Copyright (c) 2025 Dimetri Chau
#



# Interface to Betaflight using the MSP protocol
# see http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol for more details

# not used but also helpful for further configuration (mainly speed)
# https://github.com/christianrauch/msp/blob/master/README.md

import messages as msg
# import logger
import asyncio
import serial_asyncio

from typing import List, Tuple
import struct
from time import perf_counter


class MSP_Requests(object):
    """
    Catalogue of MSP cmd's to request information.
    This doesn't include any ID to send data to the FC.
    """
    # Multiwii Standard ID's
    MSP_IDENT = 100
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_SERVO = 103
    MSP_MOTOR = 104
    MSP_RC = 105
    MSP_RAW_GPS = 106
    MSP_COMP_GPS = 107
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110
    MSP_RC_TUNING = 111

    # Those below aren't really supported
    MSP_PID = 112
    MSP_BOX = 113
    MSP_MISC = 114
    MSP_MOTOR_PINS = 115
    MSP_BOXNAMES = 116
    MSP_PIDNAMES = 117
    MSP_WP = 118
    MSP_BOXIDS = 119
    MSP_SERVO_CONF = 120

    # Custom Betaflight
    # https://github.com/betaflight/betaflight/pull/6509
    # https://github.com/betaflight/betaflight-configurator/blob/master/src/js/msp/MSPCodes.js
    # Note that MSP_MULTIPLE_MSP has a return format depending on the MSP's requested
    # (B) Length of first MSP MSG, (MSP_MSG_FORMAT) Data of 1st MSP MSG, (B) Length of 2nd MSP MSG, ...
    MSP_MULTIPLE_MSP = 230

    # Mapping ID's to a format for struct.unpack
    id_to_format = {
        MSP_IDENT: 'BBBI',
        MSP_STATUS: 'HHHIB',
        MSP_RAW_IMU: 'h' * 9,
        MSP_SERVO: 'H' * 16,
        MSP_MOTOR: 'H' * 16,
        MSP_RC: 'H' * 16,
        MSP_RAW_GPS: 'BBIIHHH' + 'I', # the last H was needed, otherwise this would result in an error
        MSP_COMP_GPS: 'HHB',
        MSP_ATTITUDE: 'hhh',
        MSP_ALTITUDE: 'ih',
        MSP_ANALOG: 'BHHH',
        MSP_RC_TUNING: 'B' * 7,
        # The rest doesn't get used in my project
    }

    # helper function to get formats in a safe way
    @staticmethod
    def get_format(enum):
        if enum in MSP_Requests.id_to_format.keys():
            return MSP_Requests.id_to_format[enum]
        
        raise Exception(f'The enum {enum} is not supported')
    
class Command:
    """
    Only submit commands and requests in this format
    helper class to prioritize commands and enables async execution
    For controls set isAsync to False, otherwise an error occurs.
    """
    def __init__(self, priority, cmd_id: int, data: bytearray = b'', isAsync: bool = True):
        self.priority = priority
        self.cmd_id = cmd_id
        self.data = data 
        self.result = asyncio.Future() if isAsync else None

    def __repr__(self):
        return f"Command(priority={self.priority}, cmd_id={self.cmd_id}, data={self.data})"

    def __lt__(self, other):
        return self.priority < other.priority
    
class MSPClient(asyncio.Protocol):
    """
    For further information see:
    https://pyserial-asyncio.readthedocs.io/en/latest/
    Use callback style asynchronous client. Should enable higher performance
    """
    def __init__(self):
        self.transport = None
        self.buffer = bytearray()

        self.cmd_queue = asyncio.Queue()
        self.stop_cmd = asyncio.Event()

        self.cmd_map = {}  # cmd_id -> list of Command objects waiting for result
        self.pending_requests = set()  # cmd_ids currently being processed

    # Callback: Gets called when a connection is initiated
    def connection_made(self, transport):
        self.transport = transport
        msg.display(msg.msp_service_connected)

        asyncio.create_task(self.execution_loop())
        print("Execution coroutine scheduled")
    
    # gets called when a connection is initiated
    async def execution_loop(self):
        while not self.stop_cmd.is_set():
            command = await self.cmd_queue.get()

            try:
                self.send_msp_command(command.cmd_id, payload=command.data)
            except Exception as e:
                print(f"Error sending command {command.cmd_id}: {e}")
                self.pending_requests.discard(command.cmd_id)
    
    def submit_command(self, command: Command):
        """
        Commands with no data requests should not be mapped.
        These commands do not expect a response.
        """
        self.cmd_queue.put_nowait(command)

    def submit_request(self, command: Command):
        """
        CAREFUL MSP_MULTIPLE_MSP commands get mapped to the same id.
        """
        if command.cmd_id not in self.cmd_map:
            self.cmd_map[command.cmd_id] = []
        self.cmd_map[command.cmd_id].append(command)

        # If there's already a pending request of this type, don't enqueue another send
        if command.cmd_id not in self.pending_requests:
            self.pending_requests.add(command.cmd_id)
            self.cmd_queue.put_nowait(command)

    def build_msp_command(self, data: List[Tuple]):
        """
        helper function, for external use
        builds the payload for an msp command.
        This is for the data part of the msp command. [(INTEGERTYPE, VALUE), ...]
        """
        payload = bytearray()

        if data is not None:
            for (format, value) in data:
                assert format in ['I', 'H', 'B'], f"Invalid format: {format}"
                assert isinstance(value, int), f"Value must be an integer: {value} {type(value)}"

                # little-endian (LSB) order
                payload += struct.pack(f'<1{format}', value)
        return payload


    def send_msp_command(self, cmd, payload=b''):
        size = len(payload)
        frame = bytearray(b'$M<')
        frame.append(size)
        frame.append(cmd)
        frame.extend(payload)
        checksum = size ^ cmd
        for b in payload:
            checksum ^= b
        frame.append(checksum)

        self.transport.write(frame)


    # Callback: Gets called when data is being received.
    # The incoming data might not be a whole MSP block
    def data_received(self, data):
        # print(f"📥 Received {len(data)} bytes: {data.hex()}")
        self.buffer.extend(data)
        self._parse_messages()

    # we need this, because the client doesn't receive whole msp blocks
    # especially when using higher frequencies
    def _parse_messages(self):
        while True:
            # Look for start of message
            start = self.buffer.find(b'$M>')
            if start == -1:
                # No valid header found; discard garbage
                # self.buffer.clear()
                return

            # Wait until we have at least header+size+cmd (5 bytes)
            if len(self.buffer) - start < 5:
                return

            size = self.buffer[start + 3]
            total_length = 6 + size  # header(3) + size(1) + cmd(1) + payload(size) + checksum(1)

            # Wait until the full message is in the buffer
            if len(self.buffer) - start < total_length:
                return

            # Extract full message
            message = self.buffer[start:start + total_length]
            # Check checksum
            calc_checksum = 0
            for b in message[3:-1]:
                calc_checksum ^= b

            if calc_checksum != message[-1]:
                print("❌ Checksum error, discarding byte and retrying")
                # Skip 1 byte and try again
                self.buffer = self.buffer[start + 1:]
                continue

            # Valid message, process it
            cmd = message[4]
            payload = message[5:-1]
            self.handle_msp_message(cmd, payload)
            # Remove processed bytes
            self.buffer = self.buffer[start + total_length:]

    # Map incoming data to waiting commands
    def handle_msp_message(self, msg_id, payload):
        if msg_id in self.cmd_map and self.cmd_map[msg_id]:
            for cmd in self.cmd_map[msg_id]:
                if not cmd.result.done():
                    cmd.result.set_result(payload)

            self.cmd_map[msg_id] = []
            # Mark this cmd_id as no longer pending
            self.pending_requests.discard(msg_id)

        else:
            # NOTE: You might wan't to raise an e exception here
            # print(f"⚠️ Received message {msg_id}, but no pending commands found for it.")
            # print(f"{msg_id} {payload}")
            pass

    # Callback: disconnect
    def connection_lost(self, exc):
        print("🔌 MSP connection lost")
        self.stop_cmd.set()

    # TODO Fix this function
    # For testing the speed use the copter class with telemetry logging instead
    def test_speed(self, N=1000):
        id = 102

        commands = []
        self.t1 = perf_counter()

        for i in range(N):
            cmd = Command(1, id)
            commands.append(cmd)
            self.submit_command(cmd)


# For testing purposes
async def main():
    loop = asyncio.get_running_loop()
    transport, protocol = await serial_asyncio.create_serial_connection(
        loop, MSPClient, '/dev/ttyACM0', baudrate=1_000_000
    )

    command = Command(1, 105)
    protocol.submit_command(command)

    # await command.done  # Wait for the command to complete
    # print(f"Command finished, time took: {t2-t1}")
    # protocol.test_speed(N=400)
    await asyncio.sleep(5)
    transport.close()

if __name__ == "__main__":
    asyncio.run(main())