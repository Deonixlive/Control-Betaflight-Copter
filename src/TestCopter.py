#
# Created on Sat May 03 2025
# Author: Dimetri Chau
# https://github.com/Deonixlive
#
# Copyright (c) 2025 Dimetri Chau
#


# This file is an example for subclassing Copter
from Copter import Copter
import messages as msg
import time

class TestCopter(Copter):
    def __init__(self, stop_cmd = None):

        super().__init__(stop_cmd)

        # Append a value for the internal state
        self.copter_data["copter_state"] = None
        self.copter_data["altitude_hold"] = 0 # cm, altitude to hold in AUTO mode
        self.copter_data["alt_roll"] = 0 # cm, rolling average altitude for smoothing
        self.copter_data["prev_altitude"] = 0 # cm, previous altitude for smoothing
        self.copter_data['prev_alt_time'] = time.time() # time of the previous altitude update
        self.copter_data['alt_change'] = 0 # cm/s, change in altitude per second

        self.copter_data["thrust_hold"] = 1400 # approx aux value for holding altitude, can be adjusted
        self.copter_data["thrust_min"] = 1100 # minimum thrust value
        self.copter_data["thrust_max"] = 1900

        # self.copter_data["thrust_hold"] = 1400 # thrust value calculated
 
        # Append PID values for altitude control
        # copied from ArduPilot https://ardupilot.org/copter/docs/tuning.html
        self.copter_data["alt_pid"] = {
            "Kp": 6,
            "Ki": 0,
            "Kd": 0,
            "P": 0,
            "I": 0,
            "D": 0,
            "prev_time": time.time(),
            "prev_error": 0,
            'imax': 5, # cm/s, maximum integral value to prevent windup
            'alt_diff_limit': 250, # cm, maximum difference between current and target altitude
            'lpf_alpha': 0.5, # low pass filter alpha value for altitude rate
        }

        self.copter_data["thrust_pid"] = {
            "Kp": 0.75,
            "Ki": 1.5,
            "Kd": 0,
            "P": 0,
            "I": 0,
            "D": 0,
            "imax": 5, # cm/s, maximum integral value to prevent windup
            "prev_time": time.time(),
            "prev_delta": 0,
            "lpf_alpha": 0.5, # low pass filter alpha value for thrust adjustment   
        }

        # add calculation for rolling average altitude
        self.update_functs = self.update_functs | {
            'cal_rolling_avg_altitude': self.cal_rolling_avg_altitude,
            'cal_alt_rate': self.cal_alt_rate
        }



    def update_copter_state(self):
        if (self.copter_data['aux3'] == None):
            msg.display(msg.copter_msp_not_ready)
            return
        
        if (self.copter_data['aux3'] >= 1600):
            # also capture current altitude for hold if there was a transition
            if (self.copter_data['copter_state'] != 'AUTO'):
                self.copter_data['altitude_hold'] = self.copter_data['alt_roll']

            self.copter_data['copter_state'] = 'AUTO'


        elif (self.copter_data['aux3'] <= 1400):
            self.copter_data['copter_state'] = 'FAILSAFE'

        else:
            self.copter_data['copter_state'] = 'REMOTE'
    
    # function must be async to be used in the update_functs
    async def cal_rolling_avg_altitude(self):
        # smooth out altitude data
        self.copter_data['alt_roll'] = self.copter_data['altitude'] * 0.7 + self.copter_data['alt_roll'] * 0.3

    async def cal_alt_rate(self):
        curr_time = time.time()

        self.copter_data['alt_change'] = (self.copter_data['alt_roll'] - self.copter_data['prev_altitude']) / (curr_time - self.copter_data['prev_alt_time'])

        self.copter_data['prev_altitude'] = self.copter_data['altitude']
        self.copter_data['prev_alt_time'] = curr_time

    async def report_pid_hold(self):
        print(f"Current Altitude: {self.copter_data["altitude"]}cm | Altitude Hold: {self.copter_data['altitude_hold']}cm")

    def control_by_tilt(self):
        tilt = abs(self.copter_data['attitude']['angy'] / 10)
        throttle_min = 1000
        throttle_max = 2000

        tilt_min = 0
        tilt_max = 90

        tilt_to_throttle = int(((tilt - tilt_min) / tilt_max) * (throttle_max - throttle_min) + throttle_min)
        # self.set_rc({'throttle': tilt_to_throttle})
        
        # t = self.copter_data["pitch"]
        # print("set throttle")
        # print(tilt)
        self.set_rc({
                    #  'roll': 1000,
                    #  'pitch': 1000,
                    #  'yaw': 1000,
                     'throttle': tilt_to_throttle,
                    #  'aux2': 1000,
                     })
        
    def clip(self, value, min_value, max_value):
        """Clip a value to a specified range."""
        return max(min(value, max_value), min_value)
    
    def alt_to_desc_rate(self, altitude, target):
        # convert altitude to a descent/ascent rate via PID control ( rate = error / s)
        alt_error = self.clip(target - altitude, -self.copter_data['alt_pid']['alt_diff_limit'], self.copter_data['alt_pid']['alt_diff_limit'])
        
        error = alt_error - self.copter_data['alt_change']
        # print(f"Altitude: {altitude} cm, Target: {target} cm, Error: {error} cm/s")
        # apply low pass filter to the error for the derivitve term
        error_lpf = self.copter_data['alt_pid']['lpf_alpha'] * error + (1 - self.copter_data['alt_pid']['lpf_alpha']) * self.copter_data['alt_pid']['prev_error']


        current_time = time.time()
        dt = current_time - self.copter_data['alt_pid']['prev_time']

        self.copter_data['alt_pid']['P'] = self.copter_data['alt_pid']['Kp'] * error
        #  clip integral to prevent windup
        self.copter_data['alt_pid']['I'] = self.clip(self.copter_data['alt_pid']['I'] + self.copter_data['alt_pid']['Ki'] * error * dt,
                                                    -self.copter_data['alt_pid']['imax'],
                                                     self.copter_data['alt_pid']['imax'])
        self.copter_data['alt_pid']['D'] = self.copter_data['alt_pid']['Kd'] * (error_lpf - self.copter_data['alt_pid']['prev_error']) / dt
        # update values for next iteration
        self.copter_data['alt_pid']['prev_error'] = error
        self.copter_data['alt_pid']['prev_time'] = current_time

        sum = (self.copter_data['alt_pid']['P'] + self.copter_data['alt_pid']['I'] + self.copter_data['alt_pid']['D']) / 100
        sum += 1
        # calculate altitude acc (in g), clipped
        # print(sum)

        altitude_acc = self.clip(sum, 0.5, 1.5)
        # altitude_acc = sum
        # print(f"D {self.copter_data['alt_pid']['D']}, P {self.copter_data['alt_pid']['P']}, I {self.copter_data['alt_pid']['I']}, sum {sum}, acc {altitude_acc}")
        return altitude_acc

        
    def adjust_thrust_by_altitude(self):
        # This function can be used to adjust the thrust based on the altitude
        # For example, if the altitude is too high, we can reduce the throttle
        # or if the altitude is too low, we can increase the throttle
        altitude = self.copter_data['altitude']
        target = self.copter_data['altitude_hold']

        acc = self.alt_to_desc_rate(altitude, target)
        # print calculated g value
        # print(f"Acceleration: {acc}g")
        # TODO adjust vertical thrust based on angle of copter relative to earth
        acc_earth = acc

        imu_z_acc = self.copter_data['imu']['acc_z']

        # calculate delta and convert to cm/s^2
        acc_delta = (acc_earth - imu_z_acc) * 980.665 # convert g to cm/s^2
        acc_delta_lpf = self.copter_data['thrust_pid']['prev_delta'] * \
                        self.copter_data['thrust_pid']['lpf_alpha'] \
                        + acc_delta * (1 - self.copter_data['thrust_pid']['lpf_alpha'])
        
        current_time = time.time()
        dt = current_time - self.copter_data['alt_pid']['prev_time']

        self.copter_data['thrust_pid']['P'] = self.copter_data['thrust_pid']['Kp'] * acc_delta_lpf
        self.copter_data['thrust_pid']['I'] = self.clip(self.copter_data['thrust_pid']['I'] + self.copter_data['thrust_pid']['Ki'] * acc_delta * dt,
                                                        -self.copter_data['thrust_pid']['imax'],
                                                        self.copter_data['thrust_pid']['imax'])
        self.copter_data['thrust_pid']['D'] = self.copter_data['thrust_pid']['Kd'] * (acc_delta_lpf - self.copter_data['thrust_pid']['prev_delta']) / dt


        self.copter_data['thrust_pid']['prev_delta'] = acc_delta_lpf
        self.copter_data['alt_pid']['prev_time'] = current_time

        sum = (self.copter_data['thrust_pid']['P'] \
               + self.copter_data['thrust_pid']['I'] \
               + self.copter_data['thrust_pid']['D']) \
               + self.copter_data['thrust_hold']
        
        sum_clipped = self.clip(sum,
                        self.copter_data['thrust_min'],
                        self.copter_data['thrust_max'])
        
        # print(f"Thrust PID: P {self.copter_data['thrust_pid']['P']}, I {self.copter_data['thrust_pid']['I']}, D {self.copter_data['thrust_pid']['D']}, sum {sum}")
        return sum_clipped

    def control_iteration(self):
        self.update_copter_state()

        # only control in 'AUTO' state
        if (self.copter_data['copter_state'] != 'AUTO'):
            # enforcing always sent aux commands so we dont get an rx loss
            self.set_rc(self.default_control_rates | self.default_aux_values)
            return

        # assert self.copter_data['copter_state'] == 'AUTO', f'NOT IN AUTO STATE, INSTEAD {self.copter_data['copter_state']}'
        # self.control_by_tilt()
        thrust = int(self.adjust_thrust_by_altitude())

        # self.copter_data['thrust_hold'] = thrust
        print(thrust)

        # send thrust to the copter
        self.set_rc({
            'throttle': thrust,
        })


if __name__ == "__main__":
    c = TestCopter()
    c.start()