# Hardware reference

## Platform

| Item | Configuration |
| --- | --- |
| Compute | NVIDIA Jetson with CUDA-capable Python environment |
| Motor interface | Arduino over USB serial, 115200 baud |
| Drive | Two tracked BLDC drive motors and BL01 drivers |
| Motor label | 90BL-200-24-3000; 24 V, 200 W, 11 A, 3000 RPM |
| Gearbox | 20:1 |
| Drive sprocket | 140 mm diameter |
| Camera | OAK-D Pro; RGB and aligned depth at 15 FPS |
| Lidar | RPLidar S2 with USB serial adapter; DenseBoost mode |
| IMU | BNO055 telemetry through Arduino |
| Battery sensing | ADS1115 voltage monitoring |

## Dimensions and navigation frame

| Measurement | Value |
| --- | --- |
| Overall length, including rear collection frame | 91.44 cm |
| Overall width | 50.8 cm |
| Track centre spacing | 43.18 cm |
| Track length | 52.07 cm |
| Track midpoint to front | 26.035 cm |
| Track midpoint to rear | 65.405 cm |
| Lidar behind front edge | 30.48 cm |
| Camera behind front edge | 6.35 cm |
| Extra obstacle buffer | 3 cm |

`base_footprint` is the midpoint of the tracks projected onto the floor. Positive
x points forward and positive y points left. Its rectangular footprint is
x = [-0.65405, 0.26035] m and y = [-0.254, 0.254] m. The rear collection frame is
included when turning and checking clearance.

The lidar is centred across the width, x = -0.04445 m from the track midpoint,
with yaw 0. The camera is centred, x = +0.19685 m. Height and camera pitch are
unmeasured; lidar TF represents a 2D floor projection rather than a calibrated
3D mount. No camera-to-map target projection is claimed.

## Arduino connections

| Function | Left motor | Right motor |
| --- | --- | --- |
| PWM / VSP | D5 | D10 |
| Direction | D6 | D9 |
| Enable | D7 | D8 |
| FG pulse input | D2 | D3 |

Brush relay is D11, **LOW on / HIGH off**. IMU and ADC share I2C; confirm pin
labels against the actual Arduino board. BNO055 address is 0x28 and ADS1115
is 0x48 in the firmware configuration.

Reported FG wiring is FG output -> 2.2 kOhm series resistor -> Arduino input,
without an external 5 V wire. The firmware assumes 4 FG pulses per motor
revolution and a 20:1 reduction; pulse count still needs physical verification.
The series resistor description is a wiring record, not a verified electrical
level specification for every driver revision.

## Feedback interpretation

The dashboard shows motor RPM before the gearbox and sprocket RPM after it.
Motor RPM = sprocket RPM x 20. Direction is inferred from the commanded motor
direction, not independently measured by FG. Travel from sprocket RPM can drift
with track slip and uncertain pulse count.

Yaw integrates 80% bias-corrected gyro rotation and 20% wheel rotation when the
IMU is fresh and valid. Translation uses wheel travel. It falls back to wheel
yaw when gyro data is unavailable; absolute compass heading is not used.
