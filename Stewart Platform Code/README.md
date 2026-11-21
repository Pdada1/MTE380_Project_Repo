# MTE380 Stewart Platform

The PC tracks a ball with a camera and sends its position and target to the Arduino. The Arduino runs the PID controller and translates its tilt commands into servo angles.

## Run the interface

From this folder:

```powershell
conda env create -f environment.yml
conda activate mte_380_env
python run.py --preview
```

Use the matching firmware from `servo_controller` to move the platform. Run `python run.py` after uploading it, or run `.\rebuild.ps1` to build, upload and launch the interface. The default camera index is `0` in `helper.py`. The default Arduino port is `COM6` in `track_ball.py`. Use `--port COM7` to select a different port or `--serial-debug` to print firmware output. Close other programs using the camera or serial port before starting.

## Actions and controls

- **Calibrate Ball**. Click several spots on the ball and select **Save Calibration**. **Reset Samples** clears the pending color samples.
- **Calibrate Platform**. The camera detects the red, green and blue markers and fits a circle through them. Confirm the outline and select **Save Calibration**. The red marker defines +X with the default configuration. The two camera calibrations update `config.json` independently and take effect immediately.
- **Calibrate Servos**. Hold neutral tilt and trim the three servos with the controls below the camera. Each click changes the offset by 1° within ±10°. Offsets are applied by physical PWM channel and reset when the interface starts.
- **Draw Circle**. Move the target around a circle using the circle PID gains.
- **Draw Polygon**. Click three or more points, then press **Go**. The ball visits them in order and returns to the first using the regular point PID gains. The vertices and edges stay visible until the run ends.
- **Level Platform**. Hold neutral tilt with the current servo trims.
- **Autotune Motors** and **Autolevel Platform**. Reserved firmware modes.
- **Sweep Platform**. Sweep X and Y once, then hold level. Press the button again to restart the sweep.
- Click inside the platform circle to set a target, press **C** to recenter, or press **Esc** to cancel the active action. Starting a new action replaces the current one.
- The PID buttons adjust each gain by 0.1. Gain and trim controls remain available during actions. The Arduino uses the gains during ball tracking and applies trims to its servo angles.

The graphs show the last 30 seconds of X and Y. Position is ball minus target in platform radii. Velocity and acceleration come from the ball measurements, so changing the target alone does not cause a spike. Gaps indicate missing detections or an interrupted camera stream.

## Extend the interface

Add a row to `ACTIONS` in `run.py` for a button and its description. For a firmware-backed action, add a matching value to `Status` in `track_ball.py` and `servo_controller/src/main.cpp`.

## Serial packet

The interface uses a 39-byte packet at 115200 baud. Numeric values use little-endian encoding.

| Offset | Field |
| --- | --- |
| 0–2 | Sync bytes `0B 00 0B` |
| 3 | Status |
| 4–6 | Servo pins in +X, counterclockwise order. Trim packets carry the physical pins for their three offsets. |
| 7–22 | Four floats. Ball X, ball Y, target X, target Y. For trim packets, the first three floats are offsets in degrees. |
| 23–26 | Unsigned 32-bit frame timestamp in milliseconds. Sweep packets use this field for the sweep ID. |
| 27–38 | Kp, Ki and Kd as floats |

| Status | Action |
| --- | --- |
| 0 | Ball lost |
| 1 | Tracking |
| 2 | Level |
| 3 | Autotune placeholder |
| 4 | Autolevel placeholder |
| 5 | Sweep |
| 6 | Update servo trims alongside the active action |
| 7 | Calibrate servos at neutral tilt |
