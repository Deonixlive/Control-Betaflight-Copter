#
# Created on Sat May 03 2025
# Author: Dimetri Chau
# https://github.com/Deonixlive
#
# Copyright (c) 2025 Dimetri Chau
#

import messages as msg
import threading
import time
import asyncio
from asynciolimiter import Limiter


def empty_buffer_handler(func):
    async def ignoreemptybuffer():
        try:
            await func()
        except Exception as e:
            pass
    return ignoreemptybuffer


# Telemetry thread for async telemetry updates
# It doens't submit commands to MSP_Client directly.
# Instead it executes an update function in update_functs
# The processing still happens in the telemetry thread
class Telemetry_Thread:
    def __init__(self,
                 msp_client,
                 stop_cmd: threading.Event,
                 update_functs: dict,
                 telemetry_freq=10):

        self.telemetry_freq = telemetry_freq
        self.stop_cmd = stop_cmd
        self.msp = msp_client
        self.update_functs = update_functs

        self.limiter = Limiter(self.telemetry_freq)
        msg.display(msg.telemetry_msp_service_init, ["Async MSPClient", "1_000_000"])

    async def telemetry_loop_iteration(self):
        await self.limiter.wait()

        async with asyncio.TaskGroup() as tg:
            tasks = [tg.create_task(empty_buffer_handler(func)()) for name, func in self.update_functs.items()]

    async def telemetry_thread(self):
        print("enter telem thread")
        while not self.stop_cmd.is_set() and (self.msp.transport is None):
            await asyncio.sleep(1)

        if not self.stop_cmd.is_set():
            msg.display(msg.telemetry_process_connected, ["Async MSPClient"])

        while not self.stop_cmd.is_set():
            try:
                await self.telemetry_loop_iteration()
            except Exception as e:
                print(e)
