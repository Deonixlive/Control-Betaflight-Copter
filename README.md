# Control Betaflight Copter

![Python](https://img.shields.io/badge/language-Python-blue)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

This repository provides code to control FPV drones equipped with Betaflight via a highly performant MSP (Multiwii Serial Protocol) implementation.

## Table of Contents

- [About the Project](#about-the-project)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

---

## About the Project

Drones using Betaflight firmware are widely popular in the FPV community for their flexibility and performance. This repository aims to provide a Python implementation to interact with these drones using MSP commands, enabling advanced control and customization.

Key features:
- Highly performant MSP implementation. (Tested for up to 200Hz telemetry update rate and command rate)
- Compatibility with a range of Betaflight-equipped drones
- Easy-to-use Python interface

---

## Getting Started
Before tasting anything make sure that you don't a battery connected or at least have taken the propellers off.
### Prerequisites

Before using this repository, ensure you have Python installed on your machine. You can download Python from [here](https://www.python.org/downloads/).
This repo was tested for Python >= 1.12

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
- Check the Betaflight FC version
- Setting the `msp_override_channels_mask`
- Enabling the 's'

### Running the python file
To start controlling your Betaflight drone, follow the steps below:
It is highly recommended that you confirm the telemetry before running TestCopter.py 
Because TestCopter.py will actually control the motors

1. Modify the appropiate fields (serial_port and the baud rate) to suit your copter in Copter.py
2. Connect your drone to your computer via USB or any supported communication interface.
3. Run the script:
   ```bash
   python src/Copter.py
   ```
4. Confirm the the telemetry received is correct and working as it should.
5. Now you can test with TestCopter.py

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
