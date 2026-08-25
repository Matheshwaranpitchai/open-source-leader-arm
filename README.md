# Open-source, low-cost, 3D printed leader arm for teleoperating an SO-ARM 

<figure class="video_container">
  <iframe src="docs/leader_arm_demo.mp4" frameborder="0" allowfullscreen="true"> 
</iframe>
</figure>

A leader arm is moved by an operator to teleoperate a follower robot arm. A leader arm's joints are never driven, they simply report their joint angles, which the follower mirrors. So instead of using expensive servos, a cheap magnetic encoder like the AS5600 can be used. It results in an arm which is lighter to move by hand and is much cheaper than the one built with servos. 

Currently, the leader arm is tested against a simulated SO-ARM in MuJoCo with live physics, so you can pick up and move objects in the scene without a physical follower arm.


## Bill of materials

|Component| Quantity | Price|
|-|-|-|
|AS5600 encoder + Diametric magnet | 6 | 6 x 186 INR = 1,116 INR (~11.7 USD)|
|608 Bearing | 6 | 6 x 30 INR = 180 INR (~1.9 USD)|
| CJMCU TCA9548A I2C 8 Channel Multiplexer| 1 | 59 INR (~0.6 USD)|
|ESP32 | 1 | 550 INR (~5.8 USD)|
|28 AWG Silicon Wires | ~5m per color (4 colors) | 477 INR (~ 5 USD)|
|M3x10mm screws | 60 | 195 INR (~2 USD)|
|7x9cm Perfboard | 1 | 42 INR (~0.45 USD)|
| Rubber band | 1 | Negligible|
| |Total | 2,619 INR (~27.45 USD)|

Excluding the price of the 3D printed parts.

For context: the STS3215 servo used in the standard LeRobot leader arm costs around 24 USD each, and the leader arm needs six, roughly 144 USD in servos alone. Our entire component list comes in under 28 USD.


## How it works?
![How it works explainer](docs/how_it_works.png)

**Why a mux?** - The AS5600 encoder has a fixed I2C address (0x36) and can't be changed. So we can't put more than one on a bus as they will all share the same address. The TCA9548A mux gives each encoder its own separate channel and connects one channel to the ESP32 at a time, so all six encoders can be read without any collision. 


## Wiring
![wiring diagram](docs/wiring.png)

| TCA9548A | ESP32 | 
| ------ | ------ |
| VIN | 3V3 |
| GND | GND |
| SDA | D18 |
| SCL | D19 |
| RST | 3V3 |
| A0, A1, A2 | GND |

| AS5600 | ESP32 |
| ------ | ------ |
| VCC | 3V3 |
| GND, DIR | GND |

| AS5600 | TCA9548A |
|--------|----------|
| SDA | SDn |
| SCL | SCn |

n = the channel for that joint \
joint 0 → SD0/SC0, ... joint 5 → SD5/SC5


## Print Settings

Printed in PLA, 0.2mm layer height and 15% infill.

There are 19 unique parts and a total of 43 parts. All the parts are designed to not need support while printing. All the parts are already oriented for printing; if any part loads at an odd angle, lay its flat face on the bed. 


## Firmware

Flash firmware.ino to the ESP32 using the Arduino IDE: 

1. Install the ESP32 board support (Tools → Board → Boards Manager → search "esp32")
2. Open 'firmware/firmware.ino' and select your ESP32 board and port, and upload.
3. After uploading, open the serial monitor at **115200 baud rate** to check the output.

The firmware uses only the built-in 'Wire' library, so there's nothing else to install.

On boot it prints a channel scan. It prints "-1" if there is an issue with the connection, missing magnet or miswired encoder etc.,

If everything works then it prints "magnet ok"

Example : # ch0: AS5600 found, magnet ok, raw 1234

After the scan, it streams the six encoder readings as a comma-separated line, at 100 Hz.


## Running the software

The python side reads the encoder stream and drives a simulated SO-ARM in MuJoCo.

Install the dependencies and clone the robot model (the scripts expect it in the working directory)

```bash
pip install "mujoco>=3.2" pyserial
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

Then run one of the two scripts, passing your ESP32's port:

```bash
python teleop.py --port COM3             #plain teleop, empty scene
python teleop_sort_task.py --port COM3    #teleop + block sorting
```

On Linux, the port looks like '/dev/ttyUSB0'.

The script opens the simulated follower arm in its natural rest pose (hardcoded as the reference pose), so when the script starts, hold the leader arm in its reference pose and press ENTER. This makes sure that both the leader and the follower arm starts with the same pose.


## Acknowledgements

This project builds on the work of several open-source projects:

- [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) - the follower arm design this leader is built to teleoperate.
- [LeRobot](https://github.com/huggingface/lerobot) - the leader/follower teleoperation approach this project is based on.
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - the SO-ARM simulation model used by the teleop scripts.
