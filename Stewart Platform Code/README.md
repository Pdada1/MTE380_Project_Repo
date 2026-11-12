# MTE380 Stewart Platform

The goal is to balance a ball on a circular platform and move it to a target selected in the camera preview. The PC measures the ball and target positions. The Arduino side is intended to use those measurements for PID control of the three servos.

## How to run

From this folder, create and activate the Conda environment:

```powershell
conda env create -f environment.yml
conda activate mte_380_env
```

The camera index is `1` in `helper.py`. The Arduino serial port is `COM6` in `track_ball.py`. If the camera fails to open, check the index and close any other app using the camera.

1. Run `python calibrate.py` with the platform level. Click a few spots on the ball to sample its color, then press **C**. Press **R** during color sampling to clear the samples.
2. Click the three servo attachment points in pin order **19, 20, 21**. Press **U** to undo a point or **S** to save the calibration to `config.json`.
3. Run `python track_ball.py --preview` to check detection and the coordinate system without serial. The target starts at the center. Click anywhere in the preview to set a new target, press **C** to recenter, or press **Q** to quit.
4. Run `python track_ball.py` to stream measurements to the Arduino. The firmware in `servo_controller` decodes packets and runs the PID after enough valid samples arrive. `setTilt()` is still a placeholder.

### Check Arduino reception

Close Arduino Serial Monitor or PlatformIO Device Monitor, then run:

```powershell
python track_ball.py --serial-debug
```

The tracker sends packets and prints the Arduino's debug output through the same connection. With the ball detected, look for `Data valid!` and the decoded `x`, `y`, `xr`, and `yr` values. If opening `COM6` returns `Access is denied`, close any other tracker or serial monitor and retry.

## Stage 1 — Calibration

**Ball color.** Sample small patches on the ball and use the HSV values to set color thresholds. Sampling several spots helps capture variation across the ball. The thresholds are saved so tracking can reuse them.

**Platform bounds.** Fit a circle through the three clicked servo points. The circle center is the origin, and the line from the origin to the first point defines +X. +Y is 90° counterclockwise from +X. The radius sets the scale for both axes.

**Servo indices.** `identify_servos()` in `calibrate.py` is the placeholder for Step C. It currently selects clicked index 0 as +X and assigns pins 19, 20, and 21 in click order. The planned approach is to tilt one servo at a time, measure how the apparent center moves, and use that shift to match each pin to a clicked point.

## Stage 2 — Tracking

Threshold each frame using the saved HSV range, clean up small blobs, and use the largest sufficiently round contour as the ball. The center of its enclosing circle gives the ball position in pixels.

Convert both the ball and target to platform coordinates. Subtract the calibrated origin, flip the image Y direction, rotate into the platform axes, and divide by the calibrated radius. This gives the same normalized Cartesian coordinates as finding the normalized radius and angle first. A distance of 0 is the center, 1 is the calibrated edge, and greater than 1 is outside the calibrated circle. The transform assumes the platform stays close to its level calibration geometry. Tilt distortion can be revisited after testing.

## Serial packet

Tracking sends one 23-byte binary packet per frame at 115200 baud. The pin order tells the Arduino which servo defines +X and which two follow counterclockwise.

| Byte offset | Field | Encoding |
| --- | --- | --- |
| 0-2 | Sync | `0x0B 0x00 0x0B` |
| 3 | Ball valid | Unsigned byte, 1 if detected and 0 otherwise |
| 4-6 | Servo pins, starting at +X and going counterclockwise | Three unsigned bytes |
| 7-22 | Ball X, ball Y, target X, target Y | Four little-endian 32-bit floats |

When the ball is lost, the packet still includes the pins and target. Ball X and Y are set to zero, and the validity byte distinguishes a lost ball from one at the center.
