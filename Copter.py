#
# Created on Sat May 03 2025
# Author: Dimetri Chau
# https://github.com/Deonixlive
#
# Copyright (c) 2025 Dimetri Chau
#


"""
WARNING: When switching to MSP_OVERRIDE mode you must have already set some values
        in overwritten the rc channels.
        Otherwise this might result in an RX_LOSS failsafe where the copter disables itself.

This base class represents a copter.
Basic commands are implemented here.

The following components interact as follows:
1. The copter.py file contains the Copter class, 
   which is responsible for managing the drone's telemetry and control.
2. The telemetry.py file contains the Telemetry_Thread class, 
   which is responsible for handling the telemetry data from the MSP_SERVICE 
   and processes the raw data as defined in the update functions below.
3. The MSP_Client class is responsible for managing the connection to the drone's flight controller and executing the commands.
"""
import threading
import messages as msg

import struct

# DEBUG
from time import perf_counter

# Custom scripts
from MSP_Service import MSPClient, Command, MSP_Requests
from Telemetry_Service import Telemetry_Thread

import asyncio
from ratelimit import limits, sleep_and_retry

import serial_asyncio

from contextlib import contextmanager
from enum import Enum


# Controls the drone
# Operates autonomously
# Not Intended to use directly, instead subclass Copter as shown in TestCopter.py
class Copter:
    def __init__(self, stop_cmd: threading.Event = None):
        self.stop_cmd = threading.Event() if stop_cmd is None else stop_cmd

        # TODO Add configurable via config file
        # Copter settings
        # Name used for logging (and maybe elsewhere)
        self.name = 'DEFAULT-COPTER'
        # Serial port to Betaflight Flight Controller
        self.serial_port = '/dev/ttyACM0'
        # self.serial_baud_rate = 115200
        # self.serial_baud_rate = 2500000
        self.serial_baud_rate = 1_000_000
        self.logger_directory = '~/logs/'
        # DEFAULT THRUST RATES (Corresponding to AUX input)
        self.default_control_rates = {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1100}
        # DEFAULT AUX_CHANNEL (Starting from AUX1=CH5)
        self.default_aux_values = {'aux'+i: 1500 for i in [range(1, 5)]}

        # Control settings
        # NOTE: By default Betaflight limits the loop the 100Hz.
        #       To change this, you have to recompile the firmware by
        #       setting 'serial_update_rate_hz' in betaflight/src/main/io/serial.c to the appropiate value.
        #       If you notice the frequency dropping, the FC probably can't handle the value.
        # Updates per second (Hz)
        self.telemetry_freq = 10

        # Sets the frequency of the control loop (Copter.control_iteration) in Hz
        self.control_freq = 10

        # functions to execute asynchronously when updating and processing copter data
        # these get send to the telemetry thread
        self.update_functs = {
                              'update_msp_multiple': self.update_msp_multiple,
                              'report_telemetry': self.log_copter_data
                            #   those below are deprecated. Instead modify update_msp_multiple
                            #   'update_rc_values': self.update_rc_values, 
                            #   'update_altitide': self.update_altitude,
                            #   'update_attitude': self.update_attitude,
                            #   'update_gps': self.update_gps,
                            #   'update_analog_values': self.update_analog_values,
                              }
        
        # access this data and other internal states here
        self.copter_data = {'roll': None, 'pitch': None, 'yaw': None, 'throttle': None,
                            'aux1': None, 'aux2': None, 'aux3': None, 'aux4': None,
                            'altitude': None,         # Altitude from FC in cm
                            'vario': None,             # vertical speed cm/s
                            'attitude': {'angx': None, # roll in deg (raw value in 1 / 10 deg)
                                         'angy': None, # same here for pitch
                                         'heading': None}, # [-180, 180] in deg (raw value in 1 / 10 deg)    
                            'gps': {'fix': None,           # 0 -> no fix, 1 -> fix
                                    'num_sats': None,
                                    'lat': None,           # in deg, (raw value in 1 / 10_000_000 deg)
                                    'lon': None,
                                    'altitude': None,      # meter from GPS
                                    'ground_speed': None,  # cm/s
                                    'ground_course': None,}, # degree * 10 
                            'battery': None,           # battery voltage in V (raw values in 0.1V)
                            'rssi': None, 
                            'signal': None,
                            'battery_voltage': None,
                            'stop': None} 
        
        # Copter Logic
        # we use decorates to rate limit the control loop
        # Note the we limit the period insteads of calls
        # this results in a more even firing rate.
        # 
        # If we were instead to set calls=freq and period=1
        # then we would execute the iteration in a burst and then wait.
        self.control_iteration_rate_limited = (sleep_and_retry) (\
                                                (limits(calls=1, period=1/self.control_freq)) \
                                                    (self.control_iteration)
                                              )
        

        # LOGGING VARS

        # DEBUG VARS
        self.counter = 0
        self.counter_interval = 5
        self.start_time = perf_counter()
        self.end_time = perf_counter()

        self.loop_ready = threading.Event()
        self.telemetry_is_ready = False


    @contextmanager
    def event_loop_context(self):
        try:
            # Start event loop thread
            self.loop_ready = threading.Event()
            self.loop_thread = threading.Thread(target=self.start_event_loop)
            self.loop_thread.start()

            # Wait until event loop is ready
            self.loop_ready.wait()

            yield  # hand control back to the caller

        finally:
            # Clean shutdown sequence
            if hasattr(self, 'loop'):
                self.loop.call_soon_threadsafe(self.loop.stop)
            if hasattr(self, 'loop_thread'):
                self.loop_thread.join()


    # this ensures that the copter_iteration function has the necessary wrappers
    # in the subclasses
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if hasattr(cls, 'copter_iteration'):
            cls.copter_iteration = (sleep_and_retry) (\
                                        (limits(calls=1, period=1/cls.control_freq)) \
                                            (cls.control_iteration)
                                        )

    # start subroutines
    async def _init_subroutines_(self):
        loop = asyncio.get_running_loop()
        transport, protocol = await serial_asyncio.create_serial_connection(
            loop, MSPClient, self.serial_port, baudrate=self.serial_baud_rate
        )
        self.msp_client = protocol

        self.telemetry = Telemetry_Thread(
            self.msp_client, self.stop_cmd, self.update_functs, self.telemetry_freq
        )
        self.copter_logic = threading.Thread(target=self.control_logic)

        # Schedule telemetry task on the loop
        self.loop.call_soon_threadsafe(
            lambda: asyncio.create_task(self.telemetry.telemetry_thread())
        )

        self.copter_logic.start()

        self.transport = transport  # to close later cleanly

    def start_event_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop_ready.set()  # signal that loop is ready
        self.loop.run_forever()

    def start(self):
        with self.event_loop_context():
            # schedule init subroutines
            self.loop.call_soon_threadsafe(
                lambda: asyncio.create_task(self._init_subroutines_())
            )

            # ✅ this blocking input remains in main thread
            input('Press enter to stop process...\n')
            msg.display(msg.copter_stopping_threads)
            self.stop_cmd.set()

            # close MSP transport cleanly
            if hasattr(self, 'transport'):
                self.transport.close()

            self.copter_logic.join()

    # Main control logic
    def control_logic(self):
        #wait for telemetry before we act
        while (not self.telemetry_is_ready):
            pass

        while not self.stop_cmd.is_set():
            self.control_iteration_rate_limited()
    
    def control_iteration(self):
        """
        Note: If you subclass, this is the main component that you want to modify        
        pack single iteration in function to rate limit it
        """
        print("MAIN LOOP RUNNING")
        # pass

    # SET THRUST VALUES
    # helper function
    def get_or_set_copter_data(self, dict, key):
        return dict[key] if key in dict.keys() else self.copter_data[key]
    
    def set_rc(self, vals):
        """
        INPUT: dict of values to modify. Unspecified values remain as the latest update of the telemetry.
        """
        settable = ['roll', 'pitch', 'yaw', 'throttle', 'aux1', 'aux2', 'aux3', 'aux4']
        payload = {key: self.get_or_set_copter_data(vals, key) for key in settable}

        # CAREFUL for some reason the ordering differs then the one shown on multiwii
        # yaw and throttle are switched up
        # this has been corrected here

        # print(payload)
        data = struct.pack('<8H', payload['roll'], payload['pitch'], payload['throttle'], payload['yaw'],
                                payload['aux1'], payload['aux2'], payload['aux3'], payload['aux4'])
       
        # print(data)
        # NOTE: We don't include a catalogue of command ID's.
        #       These must be created manually.
        MSP_SET_RAW_RC = 200
        # set isAsync to False since we don't run this in an async loop
        cmd = Command(1, MSP_SET_RAW_RC, data=data, isAsync=False)

        self.msp_client.submit_command(cmd)


    async def log_copter_data(self):
        # msg.display(msg.telemetry_log_copter_state, [self.copter_data])
        self.counter += 1

        if (self.counter == self.counter_interval):
            self.counter = 0

            self.end_time = perf_counter()
            delta = self.end_time - self.start_time
            f = self.counter_interval / delta
            # prepare for next iteration
            self.start_time = self.end_time

            self.copter_data['stop'] = self.stop_cmd.is_set()
            msg.display(msg.telemetry_log_copter_state, [self.copter_data])
            msg.display(msg.telemetry_log_frequency, [f])

    async def update_msp_multiple(self):
        """
        EXAMPLE ON HOW TO BUILD A TELEMETRY UPDATE FUNCTION

        Don't forget to add it to the update_functs dict
        The incoming data (cmd.result) is structured as follows:

        (B) Length of first MSP Data Squence, (B) Length of 2nd MSP Data Sequence, ...

        TODO implement a better way to control which data to request.
        For instance, one might want to get rc data more often than GPS data.
        """
        
        # Default data to request per telemetry cycle
        self.msp_reqs = [
                        MSP_Requests.MSP_RC,
                        MSP_Requests.MSP_ALTITUDE,
                        MSP_Requests.MSP_ATTITUDE,
                        MSP_Requests.MSP_RAW_GPS,
                        MSP_Requests.MSP_ANALOG,
                        ]
        
        msg_formats = [MSP_Requests.get_format(req) for req in self.msp_reqs]
        format = ''.join(['B' + form for form in msg_formats])

        #specify payload
        payload = [('B', req) for req in self.msp_reqs]
        payload_bytes = self.msp_client.build_msp_command(payload)
        
        # we currently mix the priorities of controls and telemetry
        cmd = Command(2, MSP_Requests.MSP_MULTIPLE_MSP, data=payload_bytes)
        self.msp_client.submit_request(cmd)

        # wait for result
        data = await cmd.result

        try:
            unpacked = struct.unpack('<' + format, data)
        except Exception as e:
            print(len(data))
            print(e)

        update_dict  = {}
        pointer = 0
        for (id, format) in zip(self.msp_reqs,  msg_formats):
            length = unpacked[pointer]
            pointer += 1
            update_dict[id] = unpacked[pointer:pointer+len(format)]
            pointer += len(format)
        
        if MSP_Requests.MSP_RC in self.msp_reqs:
            # RC data
            self.copter_data['roll'], self.copter_data['pitch'], self.copter_data['yaw'], \
            self.copter_data['throttle'], self.copter_data['aux1'], self.copter_data['aux2'], \
            self.copter_data['aux3'], self.copter_data['aux4'] = update_dict[MSP_Requests.MSP_RC][:8]

        if MSP_Requests.MSP_ALTITUDE in self.msp_reqs:
            # Altitude data
            self.copter_data['altitude'], self.copter_data['vario'] = update_dict[MSP_Requests.MSP_ALTITUDE]

        if MSP_Requests.MSP_ATTITUDE in self.msp_reqs:
            # Attitude data
            self.copter_data['attitude']['angx'], \
            self.copter_data['attitude']['angy'], \
            self.copter_data['attitude']['heading'] = update_dict[MSP_Requests.MSP_ATTITUDE]

        if MSP_Requests.MSP_RAW_GPS in self.msp_reqs:
            # GPS data
            self.copter_data['gps']['fix'], \
            self.copter_data['gps']['num_sats'], \
            self.copter_data['gps']['lat'], \
            self.copter_data['gps']['lon'],\
            self.copter_data['gps']['altitude'], \
            self.copter_data['gps']['ground_speed'], \
            self.copter_data['gps']['ground_course'] = update_dict[MSP_Requests.MSP_RAW_GPS][:7]

        if MSP_Requests.MSP_ANALOG in self.msp_reqs:
            # Analog data
            self.copter_data['battery'], \
            self.copter_data['rssi'], \
            self.copter_data['signal'], \
            self.copter_data['battery_voltage'] = update_dict[MSP_Requests.MSP_ANALOG][:4]


        # Set telemetry to ready
        self.telemetry_is_ready = True

    async def update_rc_values(self):
        """
        WARNING: As of now, those are deprecated and requesting information should be done via MSP_MULTIPLE_MSP
        for performance reasons.

        Use async to enable the use of await, such that we can implement priority
        if we request data from the FC we can create a Command object of the form: Command(PRIORITY, FUNCTION)
        then we can submit the command to the MSP_Service module
        and wait for the result, which gets saved in the result attribute
        """
        cmd = Command(1, MSP_Requests.MSP_RC)
        self.msp_client.submit_request(cmd)

        data = await cmd.result

        if cmd.result is None:
            raise Exception("Command result is None")

        rc_chs = struct.unpack('<' + 'H' * (len(data) // 2), data)

        self.copter_data['roll'], self.copter_data['pitch'], self.copter_data['yaw'], \
        self.copter_data['throttle'], self.copter_data['aux1'], self.copter_data['aux2'], \
        self.copter_data['aux3'], self.copter_data['aux4'] = rc_chs[:8]

    # depr
    async def update_altitude(self):
        cmd = Command(1, MSP_Requests.MSP_ALTITUDE)
        self.msp_client.submit_request(cmd)

        data = await cmd.result
        # This segments the Hex data into 2 byte segments and packs them into a list
        alt_data = struct.unpack('<' + 'ih', data)
        self.copter_data['altitude'] = alt_data[0]
        self.copter_data['vario'] = alt_data[1]

    # depr
    async def update_attitude(self):
        cmd = Command(1, self.msp.MSP_ATTITUDE)
        self.msp.submit_request(cmd)
        await cmd.done.wait()
        if (cmd.result is None):
            raise Exception("Command result is None")
        msg_id, data = cmd.result
        assert msg_id == 108, print(msg_id)
        attitude_data = struct.unpack('<' + 'hhh', data)
        self.copter_data['attitude']['angx'] = attitude_data[0]
        self.copter_data['attitude']['angy'] = attitude_data[1]
        self.copter_data['attitude']['heading'] = attitude_data[2]

    # depr
    async def update_gps(self):
        cmd = Command(1, self.msp.MSP_RAW_GPS)
        self.msp.submit_request(cmd)
        await cmd.done.wait()
        if (cmd.result is None):
            raise Exception("Command result is None")
            
        msg_id, data = cmd.result
        assert msg_id == 106

        gps_data = struct.unpack('<' + 'BBIIHHH', data[:16])
        self.copter_data['gps']['fix'] = gps_data[0]
        self.copter_data['gps']['num_sats'] = gps_data[1]
        self.copter_data['gps']['lat'] = gps_data[2] * 1e7
        self.copter_data['gps']['lon'] = gps_data[3] * 1e7
        self.copter_data['gps']['altitude'] = gps_data[4]
        self.copter_data['gps']['ground_speed'] = gps_data[5]
        self.copter_data['gps']['ground_course'] = gps_data[6]

    # depr
    async def update_analog_values(self):
        cmd = Command(1, self.msp.MSP_ANALOG)
        self.msp.submit_request(cmd)
        await cmd.done.wait()
        if (cmd.result is None):
            print(cmd.result)
            raise Exception("Command result is None")

        msg_id, data = cmd.result
        assert msg_id == 110, print(msg_id)
        analog_data = struct.unpack('<' + 'BHHH', data[:7])

        self.copter_data['battery'] = analog_data[0] / 10
        self.copter_data['rssi'] = analog_data[1]
        self.copter_data['signal'] = analog_data[2]
        self.copter_data['battery_voltage'] = analog_data[3]


if __name__ == '__main__':
    stop_cmd = threading.Event()
    # Create a copter instance
    copter = Copter()
    # copter = Copter()
    copter.start()


        