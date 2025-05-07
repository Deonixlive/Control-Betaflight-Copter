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

class TestCopter(Copter):
    def __init__(self, stop_cmd = None):
        super().__init__(stop_cmd)

        # Append a value for the internal state
        self.copter_data["copter_state"] = None


    def update_copter_state(self):
        if (self.copter_data['aux3'] == None):
            msg.display(msg.copter_msp_not_ready)
            return
        
        if (self.copter_data['aux3'] >= 1600):
            self.copter_data['copter_state'] = 'AUTO'
        elif (self.copter_data['aux3'] <= 1400):
            self.copter_data['copter_state'] = 'FAILSAFE'
        else:
            self.copter_data['copter_state'] = 'REMOTE'
        
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
        

    def control_iteration(self):
        # self.update_copter_state()
        # only control in 'AUTO' state
        if (self.copter_data['copter_state'] != 'AUTO'):
            # enforcing always sent aux commands so we dont get an rx loss
            self.set_rc(self.default_control_rates | self.default_aux_values)
            return

        # assert self.copter_data['copter_state'] == 'AUTO', f'NOT IN AUTO STATE, INSTEAD {self.copter_data['copter_state']}'
        self.control_by_tilt()



if __name__ == "__main__":
    c = TestCopter()
    c.start()