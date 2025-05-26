# Control Betaflight Copter

![Python](https://img.shields.io/badge/language-Python-blue)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

This repository provides an easy way to control FPV drones equipped with Betaflight via a highly performant MSP (Multiwii Serial Protocol) implementation. It allows for precise and fast control of FPV Drones while enabling the compute power and intelligence of a companion computer.

## Table of Contents

- [About the Project](#about-the-project)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Customization](#customization)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

---

## About the Project

Drones using Betaflight firmware are widely popular in the FPV community for their flexibility and performance. This repository aims to provide a Python implementation to interact with these drones using MSP commands, enabling advanced control and customization.
Everything is currently highly experimental and is still being tested. Always test in a safe manner and in a controlled environment. We take no responsibility for injuries or other damages.

Key features:
- Highly performant asynchronous MSP implementation. (Tested for up to 200Hz telemetry update rate and command rate)
- Implementing Betaflight's 'MSP_MULTIPLE_MSP' command for faster telemetry reporting
- Compatibility with a range of Betaflight-equipped drones
- Easy-to-use Python interface
- Easy modifications via subclassing
---

## Getting Started
Before testing anything make sure that you don't a battery connected or at least have taken the propellers off.
### Prerequisites

Before using this repository, ensure you have Python installed on your machine. You can download Python from [here](https://www.python.org/downloads/).
This repo has been tested for Python >= 1.12 and there aren't many dependencies.

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/Deonixlive/Control-Betaflight-Copter.git
   ```

2. Navigate to the project directory:
   ```bash
   cd Control-Betaflight-Copter
   ```

3. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## Usage
### Setting the Betaflight firmware up
Before we can control the copter we need to do the following things:
- Check the Betaflight FC version (>= 4.5.2, 4.6 recommended)
- Setting the `msp_override_channels_mask`.
- Enabling the `MSP_OVERRIDE` flight mode.
- [OPTIONAL] Recompile Betaflight for MSP command rates >= 100Hz.

By default, Betaflight will only send MSP responses with a rate of 100Hz. If you want to request more data or send control signals while doing so, the package rate might slow down.
#### Recompiling Betaflight and enabling higher package rates
If you dont have Betaflight 4.5.2 or higher or want to enable higher package rate follow these instructions. Note that you do these at your own risk.
To compile your own Betaflight firmware, follow the instructions at the [Betaflight website](https://betaflight.com/docs/category/building).
Note that we won't need to recompile the Betaflight configurator.
Before compiling a target with `make TARGETNAME` change the variable `serial_update_rate_hz` in the file 'betaflight/src/main/io/serial.c' to your desired rate.
We recommend you start off with 150Hz but not to exceed 400Hz depending on the Flight Controller. 

Save the changes and run `make TARGETNAME`. After this is done there should be a `.hex` file in the folder `obj`.
Now start Betaflight Configurator or use the [web version](https://app.betaflight.com/). It is highly advised that you back up your configuration file before flashing.
Flash the new firmware and optionally load your old configuration again.

#### Enabling the MSP_OVERRIDE mode
The value `msp_override_channels_mask` determines which rc channels can be overwritten when enabling `MSP_OVERRIDE`.
The format is: `AUX16|...|AUX4|AUX3|AUX2|AUX1|YAW|THROTTLE|PITCH|Roll`. Setting it to 1 enables an overwrite.

Note that we highly advise against overwriting AUX1, since the arming switch is usually there. 
The following codes might be useful:
```bash
| AUX4 	| AUX3 	| AUX2 	| AUX1 	| YAW 	| THR 	| ROLL 	| PTCH 	| VAL 	|
|------	|------	|------	|------	|-----	|-----	|------	|------	|-----	|
|      	|      	|      	|      	|     	| X   	|      	|      	| 4   	|
|      	|      	|      	|      	| X   	| X   	| X    	| X    	| 15  	|
|      	|      	| X    	|      	| X   	| X   	| X    	| X    	| 47  	|
|      	| X    	| X    	|      	| X   	| X   	| X    	| X    	| 111 	|
| X    	| X    	| X    	|      	| X   	| X   	| X    	| X    	| 239 	|
| X    	|      	| X    	|      	| X   	| X   	| X    	| X    	| 175 	|
| X    	|      	|      	|      	| X   	| X   	| X    	| X    	| 143 	|
```
__WARNING:__ _On our firmware the YAW and THROTTLE channel was swapped. This has been corrected here, but you might want to revert that if you notice that it is wrong. You will also need to adjust the set_rc function in Copter.py_

For our example we only need to enable the throttle channel which corresponds to `100 (Binary) = 4 (Decimal)`. Notice the we can ignore the leftmost zeros.
Now we can set and save the mask:
```bash
set msp_override_channels_mask = 4
save
get msp_override_channels_mask
```
In the modes section map `MSP_OVERRIDE` to an channel and save. Confirm that you can't control the set channels when in MSP_OVERRIDE in the remote tab.
You should enable higher Baud rates for the used MSP Port in the ports section when using higher frequencies. We used 1'000'000.

### Running the python file
__WARNING__: Before testing anything, at least screw the propellers off. Always test in a controlled environment and in a safe manner.

To start controlling your Betaflight drone, follow the steps below:
It is highly recommended that you confirm the telemetry before running TestCopter.py,
because TestCopter.py will actually control the motors.

1. Modify the appropriate fields (`serial_port` and the `serial_bard_rate`) to suit your copter in Copter.py
2. Connect your drone to your computer via USB or any supported communication interface.
3. Run the script:
   ```bash
   python src/Copter.py
   ```
4. Confirm the telemetry received is correct and working as it should.
5. Now you can test with TestCopter.py. Depending on the tilt it should adjust the motors speed. If you encounter issues, check that the throttle in the telemetry get's adjusted and that you have the MSP_OVERRIDE mode on. If it still does not work, check wether the THROTTLE and YAW channels are swapped when communication via MSP. The latter was an issue that we found while testing. 

---

## Customization
Many things, such as requested telemetry, can be added with modifying the Copter class in `Copter.py`.
There are also various field to control the update frequencies and internal states.

To add customized control loops, either modify the Copter class directly or subclass Copter as shown in `TestCopter.py`.

A few notes:
- You want to issue default channel values or other values to the overwritten channels as specified in the MSP mask values. We found out during testing that switching to `MSP_OVERRIDE` while having no values set, let the copter panic into failsafe. This is due to no signals in the channel, despite the transition time being very small.
- When subclassing Copter and defining your own `control_iteration(self)` function, it automatically gets wrapped into a rate limited function and that instead gets executed. This is to enforce the control_freq set.
- You want to ensure that no packet gets lost, when setting high telemetry update rates. Consider lowering these, for more stable update rates.
- Telemetry updates and the processing of commands happen in an Asyncio.event_loop. If you also program asynchronously, consider running these in their seperate threads.

---

## Contributing

Contributions are welcome! If you have ideas for new features or improvements, feel free to open an issue or submit a pull request. For major changes, please open an issue first to discuss what you would like to change.

---

## License

This project is licensed under the terms of the MIT license. See the `LICENSE` file for more details.

---

## Contact

- **Author**: [Deonixlive](https://github.com/Deonixlive)

For any additional questions or feedback, feel free to create an issue in this repository.

---
