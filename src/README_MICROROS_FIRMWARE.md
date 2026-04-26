# ESP32 micro-ROS Gantry Firmware

This firmware runs on an ESP32 (`esp32dev`) and controls a stepper-driven gantry via micro-ROS over serial.

It uses:
- **micro_ros_platformio** (ROS 2 Jazzy, serial transport)
- **FastAccelStepper** for motion control
- Two FreeRTOS tasks (motion + ROS executor)

## 1. Features

- Subscribes to motion commands:
  - `position` (`std_msgs/msg/Float32`, mm target)
  - `speed` (`std_msgs/msg/Float32`, mm/s)
  - `acceleration` (`std_msgs/msg/Float32`, mm/s^2)
- Publishes feedback:
  - `current_position` (`std_msgs/msg/Float32`, mm)
- Homing switch support on `HOMING_PIN` with automatic backoff.
- Input clamping for safety (position, speed, acceleration).

## 2. Hardware mapping

Defined in `src/main.cpp`:

- `STEP_PIN = 25`
- `DIR_PIN = 26`
- `HOMING_PIN = 32` (active low, uses `INPUT_PULLUP`)
- `ENABLE_PIN = -1` (disabled; set GPIO number if used)

Driver/motion constants:
- `STEPS_PER_MM = (200 * 16) / 8 / 5 = 80`
- Position clamp: `±2000 mm`
- Speed clamp: `0.1 .. 200 mm/s`
- Acceleration clamp: `1 .. 400 mm/s²`

## 3. Build and flash

From project root:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

`platformio.ini` currently uses:
- `board = esp32dev`
- `board_microros_distro = jazzy`
- `board_microros_transport = serial`

## 4. Start micro-ROS Agent (ROS 2 PC)

Connect ESP32 over USB, then run one of these:

### Native agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Docker agent
```bash
docker run -it --rm --privileged -v /dev:/dev microros/micro-ros-agent:jazzy \
  serial --dev /dev/ttyUSB0 -b 115200
```

## 5. Run and command the firmware

In another terminal (with ROS 2 sourced), send commands:

```bash
ros2 topic pub /position std_msgs/msg/Float32 "{data: 100.0}" -1
ros2 topic pub /speed std_msgs/msg/Float32 "{data: 60.0}" -1
ros2 topic pub /acceleration std_msgs/msg/Float32 "{data: 150.0}" -1
```

Read position feedback:

```bash
ros2 topic echo /current_position
```

List active interfaces:

```bash
ros2 node list
ros2 topic list
```

Expected node name: `/gantry_controller`.

## 6. Homing behavior

When the homing switch is pressed (`HOMING_PIN == LOW`):
- Motion is force-stopped.
- Position is reset to `±BACKOFF_MM` (currently `5 mm`, sign depends on motion direction).
- Internal target is reset to `0 mm`.

## 7. Notes and tuning

- If direction is inverted, adjust `DIR_HIGH_IS_POSITIVE` and/or motor wiring.
- If your driver uses enable pin, set `ENABLE_PIN` and `ENABLE_LOW_ENABLES` correctly.
- Topic names in code are relative (`position`, `speed`, `acceleration`, `current_position`), so they typically appear as `/position`, `/speed`, `/acceleration`, `/current_position`.

## 8. Troubleshooting

### micro-ROS agent not connecting
- Check USB port (`/dev/ttyUSB0` vs `/dev/ttyACM0`)
- Ensure baud is `115200`
- Verify cable supports data (not charge-only)

### Motor does not move
- Verify STEP/DIR wiring and driver power
- Confirm commands are being received (`ros2 topic echo /position`)
- Check speed/accel are not near minimum

### Position feedback not updating
- Confirm node exists: `ros2 node list`
- Confirm topic exists: `ros2 topic list | grep current_position`
- Check serial monitor logs for repeated `micro-ROS init failed, retrying...`
