# Open-source Leader Arm

Open-source, low-cost, 3D-printed leader arm for teleoperating an SO-ARM 

https://github.com/user-attachments/assets/977a0b56-c7d2-4a90-b86a-445dd3963871

A leader arm is moved by an operator to teleoperate a follower robot arm. A leader arm's joints are never driven, they simply report their joint angles, which the follower mirrors. So instead of using expensive servos, a cheap magnetic encoder like the AS5600 can be used. It results in an arm which is lighter to move by hand and is much cheaper than the one built with servos. 

Currently, the leader arm is tested against a simulated SO-ARM in MuJoCo with live physics, so you can pick up and move objects in the scene without a physical follower arm. It also plugs into LeRobot as a teleoperator, so you can record training datasets entirely in simulation.

## Featured Build

Oliver Choy independently built the Leader Arm and documented the process, including the ROS 2 implementation. He'll also be showcasing it at ROSCon 2026. [Read about his build →](https://www.linkedin.com/pulse/building-sub-50-3d-printed-robotic-arm-ros-2-little-oliver-choy-22amc/)

## Bill of materials

|Component| Quantity | Price|
|-|-|-|
|AS5600 encoder + Diametric magnet (5mm dia,2mm thick) | 6 | 6 x 186 INR = 1,116 INR (~11.7 USD)|
|608 Bearing (8x22x7mm) | 6 | 6 x 30 INR = 180 INR (~1.9 USD)|
| CJMCU TCA9548A I2C 8 Channel Multiplexer| 1 | 59 INR (~0.6 USD)|
|ESP32 | 1 | 550 INR (~5.8 USD)|
|28 AWG Silicon Wires | ~5m per color (4 colors) | 477 INR (~ 5 USD)|
|M3x10mm screws | 60 | 195 INR (~2 USD)|
|7x9cm Perfboard | 1 | 42 INR (~0.45 USD)|
| Rubber band | 1 | Negligible|
| |Total | 2,619 INR (~27.45 USD)|

Excluding the price of the 3D printed parts.

For context: the STS3215 servo used in the standard LeRobot leader arm costs around 24 USD each, and the leader arm needs six, roughly 144 USD in servos alone. Our entire component list comes in under 28 USD.

Note: The encoder and diametric magnet almost always come as a combo. Try to buy them together, the diametric magnet on its own can be difficult to source. 

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

There are 19 unique parts and a total of 49 parts. All the parts are designed to not need support while printing. All the parts are already oriented for printing; if any part loads at an odd angle, lay its flat face on the bed. 

The part 'bearing_housing' needs a bearing inserted mid-print. Add a pause at 10mm height (layer 50 at 0.2mm layer height setting). In Bambu studio, right-click the corresponding layer on the vertical slider and choose "Add Pause". When the printer pauses, insert the 608 bearing into the pocket, make sure it is seated flat, and then resume.

### Parts list and count

```
1 x base
1 x base_connector
6 x bearing_housing
6 x encoder_housing
5 x encoder_housing_cap
6 x rotor 
3 x link_perpendicular_base
3 x link_perpendicular_body
1 x link_shoulder_to_elbow
1 x link_elbow_to_wrist
1 x link_tooltip 
3 x washer_perpendicular_link 
3 x washer_round
1 x handle_base
1 x handle_body
1 x rest_pose_holder_base
1 x rest_pose_holder_body
3 x wire_holder
2 x rubberband_housing
```

## Firmware

Flash firmware.ino to the ESP32 using the Arduino IDE: 

1. Install the ESP32 board support (Tools → Board → Boards Manager → search "esp32")
2. Open 'firmware/firmware.ino' and select your ESP32 board and port, and upload.
3. After uploading, open the serial monitor at **115200 baud rate** to check the output.

The firmware uses only the built-in 'Wire' library, so there's nothing else to install.

On boot it prints a channel scan, one line per channel, naming whatever is wrong: "no device at 0x36" for a wiring or power fault, or "magnet too weak / too far", "too strong / too close" or "not detected" for a magnet fault.

If everything works then it prints "magnet ok"

Example : # ch0: AS5600 found, magnet ok, raw 1234

After the scan, it streams the six encoder readings as a comma-separated line, at 100 Hz. A channel that cannot be read sends "-1" in place of its angle.


## Running the software

Two ways to drive the follower arm: using LeRobot plugins (recommended), which work with a physical or a simulated follower, or the standalone scripts, which drive the simulated follower only. The follower arm in both methods starts in the SO-ARM rest pose, so hold the leader in its reference pose and press ENTER when prompted. That aligns both arms.

### Simulator setup

Skip this if you are using a physical follower. The simulator needs the SO-ARM model from MuJoCo Menagerie; the full repo is 2.35 GB, so pull only the SO-ARM100 folder (6.5 MB).
From the repo root:
```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco_menagerie
git sparse-checkout set trs_so_arm100
cd ..
```
### Method 1 - using LeRobot plugins

Two plugins in this repo make the arm work inside [LeRobot](https://github.com/huggingface/lerobot):

- `lerobot_teleoperator_encoder_leader` - registers this leader arm as a LeRobot `Teleoperator`, so it replaces the servo leader in LeRobot's standard commands.
- `lerobot_robot_mujoco_so_arm` - registers a MuJoCo-simulated SO-ARM100 as a LeRobot `Robot`, so you can run the whole pipeline without a physical follower.

From the repo root (Python 3.10 or newer):
```bash
pip install lerobot
pip install -e lerobot_teleoperator_encoder_leader
pip install -e lerobot_robot_mujoco_so_arm
```

#### Teleoperate the arm

```bash
lerobot-teleoperate `
  --robot.type=so101_follower --robot.port=COM6 --robot.id=my_follower `
  --teleop.type=encoder_leader --teleop.port=COM3 --teleop.id=my_leader
```

Replace `COM3` with your ESP32's port. It's the same one the Arduino IDE showed when you flashed the firmware; on Linux and macOS it looks like `/dev/ttyUSB0`. Replace `COM6` with your follower's port.

When prompted, make sure the follower is in its rest pose, then hold the leader in its reference pose and press ENTER. That zeroes the encoders against the follower's rest pose.

Swap `--robot.type=mujoco_so_arm --robot.id=sim --robot.scene=mujoco_menagerie/trs_so_arm100/scene.xml` and drop `--robot.port` to drive the follower in simulation.

#### Record a dataset with a physical follower

```bash
lerobot-record `
  --robot.type=so101_follower --robot.port=COM6 --robot.id=my_follower `
  --robot.cameras="{top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --teleop.type=encoder_leader --teleop.port=COM3 --teleop.id=my_leader `
  --dataset.repo_id=<hf_user>/so_arm_sort --dataset.num_episodes=50 `
  --dataset.single_task="Put the red block in the left bin"
```
Replace `<hf_user>` with your huggingface username.
For more, see [Record a dataset](https://huggingface.co/docs/lerobot/main/en/lerobot-dataset-v3#record-a-dataset) in the LeRobot docs.

#### Record a dataset in simulation

```bash
lerobot-record `
  --robot.type=mujoco_so_arm --robot.id=sim `
  --robot.scene=mujoco_menagerie/trs_so_arm100/scene.xml --robot.viewer=true `
  --teleop.type=encoder_leader --teleop.port=COM3 --teleop.id=my_leader `
  --dataset.repo_id=<hf_user>/so_arm_sort --dataset.num_episodes=50 `
  --dataset.single_task="Put the red block in the left bin"
```
Replace `<hf_user>` with your huggingface username.
The scene comes with a block-sorting task: three coloured blocks and three matching bins.

Between episodes the blocks get rearranged into new positions. The plugin watches the same keys `lerobot-record` already uses (Right/Left, or `n`/`r`), so there is nothing extra to press and nothing to do during the reset window.

If you end episodes by timeout rather than by keypress, match both of `lerobot-record`'s timers or the episode tracking drifts:

```bash
  --dataset.episode_time_s=30 --robot.episode_time_s=30 `
  --dataset.reset_time_s=10  --robot.reset_time_s=10
```

Set the recording rate with `--dataset.fps` (default is 30). On integrated graphics 30 is about the limit; drop to 25 if the loop lags behind it.

Shadows and reflections are left out of the recorded images to keep the render cheap.

### Method 2 - using standalone scripts

No LeRobot needed; the encoder stream drives MuJoCo directly.

```bash
pip install "mujoco>=3.2" pyserial
```
Run either one.
```bash
python teleop.py --port COM3              # plain teleop, empty scene
python teleop_sort_task.py --port COM3    # teleop + block sorting
```

On Linux, the port looks like `/dev/ttyUSB0`.

In `teleop_sort_task.py`, press `R` in the viewer to reset the blocks.





## Assembly guide

This leader arm has six joints, `joint_0` to `joint_5`.
### Before you start : 
- See the print settings section for the full parts list and count
- Use the images in the guide for parts orientation and alignment. The assembly files under cad_files are also helpful for this.
- Notation : A + B means "attach part B to part A or assembly A". assembly_step_N refers to the result of step N. 
- Only joint_0 and joint_5 get their encoder attached immediately. The remaining four joints get their encoders at the very end, so that there is less wire clutter during the assembly.
 
### Steps : 
1. Start by wiring all the six encoders as shown in `wiring.png`.
2. insert the diametric magnet into the circular pocket provided in the `rotor` (for all 6 rotors)
3. `rotor + bearing_housing + link_perpendicular_base + link_perpendicular_body + washer_perpendicular_link` (`joint_0`)![step_3](docs/step_3.png)
4. `assembly_step_3 + base_connector` ![step_4](docs/step_4.png)
5. `assembly_step_4 + encoder_housing + AS5600_encoder` ![step_5](docs/step_5.png)
6. Tie a knot in the wire, then attach the `encoder_housing_cap`, so that a wire pull doesn't act on the encoder. 
7. `assembly_step_6 + base` ![step_7](docs/step_7.png)
8. `assembly_step_7 + rest_pose_holder_base + rest_pose_holder_body` ![step_8](docs/step_8.png)
9. `assembly_step_8 + bearing_housing` ![step_9](docs/step_9.png)
10. `assembly_step_9 + rotor + link_shoulder_to_elbow + washer_round` (`joint_1`) ![step_10](docs/step_10.png)
11. `link_shoulder_to_elbow + bearing_housing` (similar to step 9)
12. `assembly_step_11 + rotor + link_elbow_to_wrist + washer_round` (similar to step 10) (`joint_2`)
13. `link_elbow_to_wrist + bearing_housing` (similar to step 11)
14. `assembly_step_13 + rotor + link_perpendicular_base + link_perpendicular_body + washer_perpendicular_link` (similar to step 3) (`joint_3`)
15. `assembly_step_14 + bearing_housing` (similar to step 9)
16. `assembly_step_15 + rotor + link_perpendicular_base + link_perpendicular_body + washer_perpendicular_link` (similar to step 14) (`joint_4`)
17. `assembly_step_16 + bearing_housing` (similar to steps 9 and 15)
18. `assembly_step_17 + rotor + link_tooltip + washer_round` (`joint_5`)
19. `assembly_step_18 + encoder_housing + AS5600_encoder`
20. Tie a knot in the wire as in step 6
21. `assembly_step_20 + handle_base` (the `handle_base` also acts here as the `encoder_housing_cap`) ![step_21](docs/step_21.png)
22. `assembly_step_21 + handle_body` ![step_22](docs/step_22.png)
23. attach the three `wire_holder`s on `link_shoulder_to_elbow` and `link_elbow_to_wrist`. ![step_23](docs/step_23.png)
24. Route a single rubberband through both `rubberband_housings`, then fix one housing to `link_shoulder_to_elbow` and one to `link_elbow_to_wrist`. ![step_24](docs/step_24.png)
25. attach the `encoder_housing + AS5600_encoder`, tie a knot, and attach the `encoder_housing_cap` for the four remaining joints, in the same way as step 5 or step 19. 

## Acknowledgements

This project builds on the work of several open-source projects:

- [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) - the follower arm design this leader is built to teleoperate.
- [LeRobot](https://github.com/huggingface/lerobot) - the leader/follower teleoperation approach this project is based on.
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - the SO-ARM model used by the simulated follower.
