// DWM2 tracked drive and brush controller.
// BL01 drivers: left PWM/DIR/EN/FG = D5/D6/D7/D2;
// right PWM/DIR/EN/FG = D10/D9/D8/D3. Brush relay = D11 (active-low).
// I2C: BNO055 at 0x28 and ADS1115 at 0x48.
// Serial 115200 baud; drive/relay watchdog 500 ms.
// Gear ratio 20:1; four FG pulses per motor revolution assumed.
// Protocol and operating details: firmware/README.md and docs/hardware.md.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADS1X15.h>


// ================================================================
// PINS
// ================================================================

#define M1_PWM 5
#define M1_DIR 6
#define M1_EN  7
#define M1_FG  2

#define M2_PWM 10
#define M2_DIR 9
#define M2_EN  8
#define M2_FG  3

#define RELAY_PIN 11


// ================================================================
// DRIVER SETTINGS
// ================================================================

const bool ENABLE_LEVEL  = LOW;
const bool DISABLE_LEVEL = HIGH;


// ================================================================
// RELAY SETTINGS
// ================================================================

/*
 Flip these two if your relay module is active-LOW
 (most cheap 1-channel relay boards are active-LOW --
 double check with a multimeter before first power-up).
*/

const bool RELAY_ON_LEVEL  = LOW;
const bool RELAY_OFF_LEVEL = HIGH;

/*
 relayMode:
   0 = OFF
   1 = ON, topic/serial controlled (subject to COMMAND_TIMEOUT failsafe)
   DWM2 removes ALWAYS ON; relay ON always uses the communication timeout.
*/

int relayMode = 0;
bool permissionEnabled = false; // DWM2: explicit E after every stop/timeout


// ================================================================
// IMU (BNO055)
// ================================================================

/*
 Address is 0x28 by default, 0x29 if the ADR/COM3 pin is pulled
 high. If the sensor fails to initialize, this is the first
 thing to check.
*/

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool imuOK = false;

const unsigned long IMU_PUBLISH_INTERVAL = 50; // ms (~20 Hz)

unsigned long lastImuPublish = 0;


// ================================================================
// BATTERY MONITOR (ADS1115)
// ================================================================

/*
 Default I2C address 0x48 (ADDR pin to GND). Different range from
 the BNO055 (0x28/0x29) -- no collision on the shared bus.
*/

Adafruit_ADS1115 ads;

bool adsOK = false;

const unsigned long BATTERY_PUBLISH_INTERVAL = 1000; // ms (1 Hz --
                                                        // battery
                                                        // voltage
                                                        // doesn't
                                                        // need to be
                                                        // read fast)

unsigned long lastBatteryPublish = 0;


// ================================================================
// WHEEL ODOMETRY PUBLISH
// ================================================================

/*
 wheel1RPM/wheel2RPM are already computed every loop (see the FG
 tachometer processing above) for the human-readable telemetry
 line -- this just re-sends the same numbers in a parseable format
 at a steady rate, for whatever ROS-side node wants to integrate
 them into real odometry.
*/

const unsigned long WHEEL_PUBLISH_INTERVAL = 50; // ms (~20 Hz --
                                                   // matches
                                                   // IMU_PUBLISH_INTERVAL,
                                                   // reasonable rate
                                                   // for odometry
                                                   // integration)

unsigned long lastWheelPublish = 0;

/*
 Voltage divider: R1 (100k) from the 24V rail down to the ADC tap,
 R2 (18k) from the tap to GND. ADS1115 reads the tap voltage;
 multiply back up by (R1+R2)/R2 to get the real pack voltage.
*/

const float BATTERY_DIVIDER_R1 = 100000.0; // ohms
const float BATTERY_DIVIDER_R2 = 18000.0;  // ohms
const float BATTERY_DIVIDER_RATIO =
  (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2;

/*
 Rough linear voltage -> percentage curve for a 7S Li-ion pack
 (7 cells in series, 4.2V/cell full -- the 4P doesn't affect
 voltage, only capacity). NOT a real fuel-gauge curve (lithium
 packs are non-linear, especially in the middle of their range) --
 good enough for "is this thing about to die" at a glance, not for
 precise SOC.

 BATTERY_MIN_V uses 3.0V/cell as the safe cutoff (protects cell
 longevity) rather than the more aggressive 2.5V/cell some packs
 allow -- raise/lower if your BMS uses a different cutoff.

 ADS1115 headroom check (done when this was set up): at 29.4V
 full pack, tap voltage through the 100k/18k divider is
 29.4 * (18k/118k) = 4.49V. ADS1115 VDD here is 5V, and its
 default PGA full-scale is +-6.144V, so this reads fine with
 headroom. If you ever change VDD to 3.3V or change the divider
 resistors, re-check this math before trusting new readings.
*/

const float BATTERY_MAX_V = 29.4; // 7S Li-ion, 4.2V/cell full
const float BATTERY_MIN_V = 21.0; // 7S Li-ion, 3.0V/cell cutoff


// ================================================================
// PHYSICAL FORWARD DIRECTION
// ================================================================

/*
 If W on your dashboard makes one wheel rotate backward,
 change ONLY that motor from HIGH to LOW.

 Example:

 const bool M1_FORWARD_LEVEL = HIGH;
 const bool M2_FORWARD_LEVEL = LOW;

*/

const bool M1_FORWARD_LEVEL = HIGH;
const bool M2_FORWARD_LEVEL = HIGH;


// ================================================================
// MOTOR / GEARBOX SETTINGS
// ================================================================

/*
 Motor:
 90BL-200-24-3000

 Rated motor speed:
 3000 rpm

 Gearbox output:
 approximately 150 rpm

 Ratio:
 20:1 (confirmed by owner)

 IMPORTANT:
 The RPM equation currently assumes 4 FG pulses per motor revolution.
 The BL01 default pole-pair setting does not establish the FG pulse count;
 confirm this divisor with the driver datasheet or a tachometer.
*/

const float POLE_PAIRS = 4.0;

const float GEAR_RATIO = 20.0;


// ================================================================
// SPEED SAMPLING
// ================================================================

const unsigned long SPEED_SAMPLE_INTERVAL = 100; // ms

const unsigned long TELEMETRY_INTERVAL = 200;    // ms


// ================================================================
// SAFE RESTART
// ================================================================

/*
 The driver will NOT be enabled while the motor is
 still rotating rapidly after a stop.

 Your normal PWM80 speed gave approximately 60 Hz.

 We wait until FG falls to approximately 10 Hz or below.
*/

const float SAFE_REENABLE_FG_HZ = 15.0;


// Minimum EN-off time before re-enable.
const unsigned long MIN_DISABLE_TIME = 120; // ms


// ================================================================
// PWM RAMP
// ================================================================

const unsigned long RAMP_INTERVAL = 20; // ms


// Acceleration
const int ACCEL_STEP = 4;


// Slower commanded deceleration while driver is running.
const int DECEL_STEP = 2;


// ================================================================
// SERIAL FAILSAFE
// ================================================================

/*
 Pi dashboard currently sends commands repeatedly while
 a key is held.

 If communication disappears for >1 second:
 motors disable automatically.
*/

const unsigned long COMMAND_TIMEOUT = 500;


// ================================================================
// FG COUNTERS
// ================================================================

volatile unsigned long fg1Count = 0;
volatile unsigned long fg2Count = 0;

// Rated 3000 motor RPM and 4 FG pulses/revolution give a 5 ms period.
// Reject edges less than 2 ms apart. Update the timestamp on EVERY edge:
// continuous PWM-frequency interference must not be counted once per 2 ms.
// This still permits 250 wheel RPM, above the host's 150 RPM stop limit.
const unsigned long FG_MIN_EDGE_GAP_US = 2000;
volatile unsigned long fg1LastEdgeUs = 0;
volatile unsigned long fg2LastEdgeUs = 0;
volatile unsigned long fg1Rejected = 0;
volatile unsigned long fg2Rejected = 0;
volatile bool fg1EdgeSeen = false;
volatile bool fg2EdgeSeen = false;


unsigned long previousFG1 = 0;
unsigned long previousFG2 = 0;


// ================================================================
// FG SPEED
// ================================================================

float fg1RawHz = 0.0;
float fg2RawHz = 0.0;


float fg1Hz = 0.0;
float fg2Hz = 0.0;


// ================================================================
// RPM
// ================================================================

float motor1RPM = 0.0;
float motor2RPM = 0.0;

float wheel1RPM = 0.0;
float wheel2RPM = 0.0;


// ================================================================
// TARGET COMMAND FROM PI
// ================================================================

int targetM1 = 0;
int targetM2 = 0;


// ================================================================
// ACTUAL PWM CURRENTLY APPLIED
// ================================================================

int actualM1 = 0;
int actualM2 = 0;


// ================================================================
// DRIVER STATE
// ================================================================

bool motor1Enabled = false;
bool motor2Enabled = false;


// Last actual physical/electrical direction.
//
// +1 forward
// -1 reverse
//
int motor1Direction = 1;
int motor2Direction = 1;


// ================================================================
// DISABLE TIMES
// ================================================================

unsigned long motor1DisabledAt = 0;
unsigned long motor2DisabledAt = 0;


// ================================================================
// TIMING
// ================================================================

unsigned long lastSpeedSample = 0;
unsigned long lastTelemetry = 0;
unsigned long lastRamp = 0;
unsigned long lastCommand = 0;


// ================================================================
// SERIAL BUFFER
// ================================================================

String serialBuffer;


// ================================================================
// FG INTERRUPTS
// ================================================================

void fg1ISR()
{
  unsigned long now = micros();
  unsigned long gap = now - fg1LastEdgeUs;
  fg1LastEdgeUs = now;
  if(fg1EdgeSeen && gap < FG_MIN_EDGE_GAP_US)
  {
    fg1Rejected++;
    return;
  }
  fg1EdgeSeen = true;
  fg1Count++;
}


void fg2ISR()
{
  unsigned long now = micros();
  unsigned long gap = now - fg2LastEdgeUs;
  fg2LastEdgeUs = now;
  if(fg2EdgeSeen && gap < FG_MIN_EDGE_GAP_US)
  {
    fg2Rejected++;
    return;
  }
  fg2EdgeSeen = true;
  fg2Count++;
}


// ================================================================
// IMMEDIATE MOTOR 1 COAST
// ================================================================

void coastMotor1()
{
  analogWrite(M1_PWM, 0);

  digitalWrite(
    M1_EN,
    DISABLE_LEVEL
  );

  actualM1 = 0;

  motor1Enabled = false;

  motor1DisabledAt = millis();
}


// ================================================================
// IMMEDIATE MOTOR 2 COAST
// ================================================================

void coastMotor2()
{
  analogWrite(M2_PWM, 0);

  digitalWrite(
    M2_EN,
    DISABLE_LEVEL
  );

  actualM2 = 0;

  motor2Enabled = false;

  motor2DisabledAt = millis();
}


// ================================================================
// COAST BOTH
// ================================================================

void coastBoth()
{
  coastMotor1();
  coastMotor2();
}


// ================================================================
// ENABLE MOTOR 1 SAFELY
// ================================================================

void enableMotor1(int direction)
{
  // -------------------------------
  // Set direction FIRST
  // -------------------------------

  bool dirLevel;

  if(direction > 0)
  {
    dirLevel = M1_FORWARD_LEVEL;
  }
  else
  {
    dirLevel = !M1_FORWARD_LEVEL;
  }


  digitalWrite(
    M1_DIR,
    dirLevel
  );


  // Start with zero PWM.
  analogWrite(
    M1_PWM,
    0
  );


  delayMicroseconds(100);


  // Enable LAST.
  digitalWrite(
    M1_EN,
    ENABLE_LEVEL
  );


  motor1Enabled = true;

  motor1Direction = direction;

  actualM1 = 0;
}


// ================================================================
// ENABLE MOTOR 2 SAFELY
// ================================================================

void enableMotor2(int direction)
{
  bool dirLevel;

  if(direction > 0)
  {
    dirLevel = M2_FORWARD_LEVEL;
  }
  else
  {
    dirLevel = !M2_FORWARD_LEVEL;
  }


  digitalWrite(
    M2_DIR,
    dirLevel
  );


  analogWrite(
    M2_PWM,
    0
  );


  delayMicroseconds(100);


  digitalWrite(
    M2_EN,
    ENABLE_LEVEL
  );


  motor2Enabled = true;

  motor2Direction = direction;

  actualM2 = 0;
}


// ================================================================
// UPDATE MOTOR 1
// ================================================================

void updateMotor1()
{
  // ============================================================
  // TARGET ZERO -> COAST
  // ============================================================

  if(targetM1 == 0)
  {
    if(
      motor1Enabled ||
      actualM1 != 0
    )
    {
      coastMotor1();
    }

    return;
  }


  // ============================================================
  // DESIRED DIRECTION
  // ============================================================

  int desiredDirection;

  if(targetM1 > 0)
  {
    desiredDirection = 1;
  }
  else
  {
    desiredDirection = -1;
  }


  // ============================================================
  // DIRECTION CHANGE WHILE ENABLED
  //
  // Disable immediately.
  // Do NOT reverse electrically while rotor is moving.
  // ============================================================

  if(
    motor1Enabled &&
    desiredDirection != motor1Direction
  )
  {
    coastMotor1();

    return;
  }


  // ============================================================
  // DRIVER CURRENTLY DISABLED
  // ============================================================

  if(!motor1Enabled)
  {
    // Motor still spinning too fast?
    if(
      fg1RawHz >
      SAFE_REENABLE_FG_HZ
    )
    {
      return;
    }


    // Minimum EN-off time
    if(
      millis() - motor1DisabledAt <
      MIN_DISABLE_TIME
    )
    {
      return;
    }


    enableMotor1(
      desiredDirection
    );

    return;
  }


  // ============================================================
  // SOFT PWM RAMP
  // ============================================================

  int desiredPWM =
    abs(targetM1);


  int currentPWM =
    abs(actualM1);


  // Accelerate
  if(
    currentPWM <
    desiredPWM
  )
  {
    currentPWM += ACCEL_STEP;


    if(
      currentPWM >
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  // Decelerate gently
  else if(
    currentPWM >
    desiredPWM
  )
  {
    currentPWM -= DECEL_STEP;


    if(
      currentPWM <
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  actualM1 =
    motor1Direction *
    currentPWM;


  analogWrite(
    M1_PWM,
    currentPWM
  );
}


// ================================================================
// UPDATE MOTOR 2
// ================================================================

void updateMotor2()
{
  if(targetM2 == 0)
  {
    if(
      motor2Enabled ||
      actualM2 != 0
    )
    {
      coastMotor2();
    }

    return;
  }


  int desiredDirection;

  if(targetM2 > 0)
  {
    desiredDirection = 1;
  }
  else
  {
    desiredDirection = -1;
  }


  // Direction change
  if(
    motor2Enabled &&
    desiredDirection != motor2Direction
  )
  {
    coastMotor2();

    return;
  }


  // Driver disabled
  if(!motor2Enabled)
  {
    if(
      fg2RawHz >
      SAFE_REENABLE_FG_HZ
    )
    {
      return;
    }


    if(
      millis() - motor2DisabledAt <
      MIN_DISABLE_TIME
    )
    {
      return;
    }


    enableMotor2(
      desiredDirection
    );

    return;
  }


  // Soft PWM ramp
  int desiredPWM =
    abs(targetM2);


  int currentPWM =
    abs(actualM2);


  if(
    currentPWM <
    desiredPWM
  )
  {
    currentPWM += ACCEL_STEP;


    if(
      currentPWM >
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  else if(
    currentPWM >
    desiredPWM
  )
  {
    currentPWM -= DECEL_STEP;


    if(
      currentPWM <
      desiredPWM
    )
    {
      currentPWM =
        desiredPWM;
    }
  }


  actualM2 =
    motor2Direction *
    currentPWM;


  analogWrite(
    M2_PWM,
    currentPWM
  );
}


// ================================================================
// UPDATE BOTH MOTORS
// ================================================================

void updateMotors()
{
  unsigned long now =
    millis();


  if(
    now - lastRamp <
    RAMP_INTERVAL
  )
  {
    return;
  }


  lastRamp = now;


  updateMotor1();

  updateMotor2();
}


// ================================================================
// FG SPEED CALCULATION
// ================================================================

void updateSpeed()
{
  unsigned long now =
    millis();


  unsigned long elapsed =
    now - lastSpeedSample;


  if(
    elapsed <
    SPEED_SAMPLE_INTERVAL
  )
  {
    return;
  }


  // ============================================================
  // SAFE COPY INTERRUPT VALUES
  // ============================================================

  unsigned long count1;
  unsigned long count2;


  noInterrupts();

  count1 = fg1Count;
  count2 = fg2Count;

  interrupts();


  // ============================================================
  // NEW PULSES
  // ============================================================

  unsigned long delta1 =
    count1 - previousFG1;


  unsigned long delta2 =
    count2 - previousFG2;


  // ============================================================
  // TIME
  // ============================================================

  float dt =
    elapsed / 1000.0;


  // ============================================================
  // RAW FG FREQUENCY
  // ============================================================

  fg1RawHz =
    delta1 / dt;


  fg2RawHz =
    delta2 / dt;


  // ============================================================
  // SIMPLE LOW-PASS FILTER
  //
  // Makes dashboard RPM smoother.
  // ============================================================

  const float alpha = 0.35;


  fg1Hz =
    alpha * fg1RawHz +
    (1.0 - alpha) * fg1Hz;


  fg2Hz =
    alpha * fg2RawHz +
    (1.0 - alpha) * fg2Hz;


  // ============================================================
  // MOTOR RPM
  //
  // RPM = FG Hz × 60 / pole pairs
  // ============================================================

  motor1RPM =
    fg1Hz *
    60.0 /
    POLE_PAIRS;


  motor2RPM =
    fg2Hz *
    60.0 /
    POLE_PAIRS;


  // ============================================================
  // APPLY ROTATION DIRECTION
  // ============================================================

  if(
    motor1Direction < 0
  )
  {
    motor1RPM =
      -motor1RPM;
  }


  if(
    motor2Direction < 0
  )
  {
    motor2RPM =
      -motor2RPM;
  }


  // ============================================================
  // GEARBOX / WHEEL RPM
  // ============================================================

  wheel1RPM =
    motor1RPM /
    GEAR_RATIO;


  wheel2RPM =
    motor2RPM /
    GEAR_RATIO;


  // ============================================================
  // SAVE
  // ============================================================

  previousFG1 =
    count1;


  previousFG2 =
    count2;


  lastSpeedSample =
    now;
}


// ================================================================
// IMU: READ + PUBLISH
// ================================================================

/*
 Sent as its own line, separate from printTelemetry()'s
 human-readable format, so the Pi-side parser can branch on the
 leading character ('I' vs the unlabeled M1/M2 telemetry line)
 without needing to touch the existing telemetry parsing.

 Format:
 I,qw,qx,qy,qz,gx,gy,gz,ax,ay,az

   qw..qz : orientation quaternion (unitless)
   gx..gz : angular velocity, rad/s
   ax..az : linear acceleration, m/s^2
*/

void publishImu()
{
  if(
    !imuOK
  )
  {
    return;
  }


  imu::Quaternion q =
    bno.getQuat();

  imu::Vector<3> gyro =
    bno.getVector(
      Adafruit_BNO055::VECTOR_GYROSCOPE
    );

  imu::Vector<3> accel =
    bno.getVector(
      Adafruit_BNO055::VECTOR_LINEARACCEL
    );


  Serial.print(
    "I,"
  );

  Serial.print(q.w(), 4);
  Serial.print(",");
  Serial.print(q.x(), 4);
  Serial.print(",");
  Serial.print(q.y(), 4);
  Serial.print(",");
  Serial.print(q.z(), 4);
  Serial.print(",");

  Serial.print(gyro.x() * SENSORS_DPS_TO_RADS, 4);
  Serial.print(",");
  Serial.print(gyro.y() * SENSORS_DPS_TO_RADS, 4);
  Serial.print(",");
  Serial.print(gyro.z() * SENSORS_DPS_TO_RADS, 4);
  Serial.print(",");

  Serial.print(accel.x(), 4);
  Serial.print(",");
  Serial.print(accel.y(), 4);
  Serial.print(",");
  Serial.println(accel.z(), 4);
}


// ================================================================
// BATTERY: READ + PUBLISH
// ================================================================

/*
 Sent as its own line, same pattern as publishImu() -- distinct
 leading character so the Pi-side parser can branch on it without
 touching existing telemetry/IMU parsing.

 Format:
 B,voltage,percentage
*/

void publishBattery()
{
  if(
    !adsOK
  )
  {
    return;
  }


  int16_t raw =
    ads.readADC_SingleEnded(0);

  float tapVoltage =
    ads.computeVolts(raw);

  float packVoltage =
    tapVoltage * BATTERY_DIVIDER_RATIO;


  float percentage =
    (packVoltage - BATTERY_MIN_V) /
    (BATTERY_MAX_V - BATTERY_MIN_V) *
    100.0;

  // Clamp -- the linear curve above can go outside 0-100 if the
  // pack is charged past BATTERY_MAX_V or has dropped below
  // BATTERY_MIN_V, neither of which is a real "over/under 100%".
  if(percentage > 100.0) { percentage = 100.0; }
  if(percentage < 0.0)   { percentage = 0.0;   }


  Serial.print(
    "B,"
  );

  Serial.print(packVoltage, 2);
  Serial.print(",");
  Serial.println(percentage, 1);
}


// ================================================================
// WHEEL ODOMETRY: PUBLISH
// ================================================================

/*
 Sent as its own line, same pattern as publishImu()/publishBattery()
 -- distinct leading character so the Pi-side parser can branch on
 it without touching existing parsing.

 Format:
 W,left_wheel_rpm,right_wheel_rpm

 No separate "read" step -- wheel1RPM/wheel2RPM are already kept
 current by the FG tachometer processing that runs every loop
 (needed for the human-readable telemetry line regardless of
 whether this function ever gets called).
*/

void publishWheelOdom()
{
  // Motor shaft RPM before the gearbox, from the same FG sample as W.
  Serial.print("N,");
  Serial.print(motor1RPM, 2);
  Serial.print(",");
  Serial.println(motor2RPM, 2);
  Serial.print(
    "W,"
  );

  Serial.print(wheel1RPM, 2);
  Serial.print(",");
  Serial.println(wheel2RPM, 2);

  unsigned long rejected1;
  unsigned long rejected2;
  noInterrupts();
  rejected1 = fg1Rejected;
  rejected2 = fg2Rejected;
  interrupts();
  Serial.print("F,");
  Serial.print(rejected1);
  Serial.print(",");
  Serial.println(rejected2);
}


// ================================================================
// TELEMETRY
// ================================================================

void printTelemetry()
{
  /*
   Keep this format unchanged because your Pi dashboard
   already understands it.
  */


  Serial.print(
    "M1 | PWM="
  );

  Serial.print(
    actualM1
  );


  Serial.print(
    " | FG="
  );

  Serial.print(
    fg1Hz,
    1
  );

  Serial.print(
    "Hz | Motor="
  );

  Serial.print(
    motor1RPM,
    0
  );

  Serial.print(
    "rpm | Wheel="
  );

  Serial.print(
    wheel1RPM,
    1
  );


  Serial.print(
    "rpm     ||     "
  );


  Serial.print(
    "M2 | PWM="
  );

  Serial.print(
    actualM2
  );


  Serial.print(
    " | FG="
  );

  Serial.print(
    fg2Hz,
    1
  );

  Serial.print(
    "Hz | Motor="
  );

  Serial.print(
    motor2RPM,
    0
  );

  Serial.print(
    "rpm | Wheel="
  );

  Serial.print(
    wheel2RPM,
    1
  );


  Serial.print(
    "rpm"
  );

  Serial.print(
    "     ||     RELAY="
  );

  Serial.println(
    relayMode
  );
}


// ================================================================
// SET TARGET MOTORS
// ================================================================

void setTargets(
  int left,
  int right
)
{
  left =
    constrain(
      left,
      -255,
      255
    );


  right =
    constrain(
      right,
      -255,
      255
    );


  targetM1 =
    left;


  targetM2 =
    right;


  lastCommand =
    millis();
}


// ================================================================
// PROCESS SERIAL
// ================================================================

// DWM2 protocol: S/D/timeout latch off BOTH wheels and rear brush.
void allStop() {
  permissionEnabled = false;
  targetM1 = 0; targetM2 = 0;
  coastBoth();
  relayMode = 0;
  digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL);
}

bool parsePWM(String text, int &value) {
  if(text.length() == 0 || text.length() > 4) return false;
  unsigned int start = text[0] == '-' ? 1 : 0;
  if(start == text.length()) return false;
  for(unsigned int i=start; i<text.length(); i++) {
    if(text[i] < '0' || text[i] > '9') return false;
  }
  long parsed = text.toInt();
  if(parsed < -255 || parsed > 255) return false;
  value = (int)parsed;
  return true;
}

void processCommand(String command) {
  command.trim();
  if(command == "V") { Serial.println("K,DWM2"); return; }
  if(command == "S" || command == "s" || command == "D" || command == "d") {
    allStop(); lastCommand = millis(); Serial.println("A,STOP"); return;
  }
  if(command == "R,0") {
    relayMode = 0; digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL);
    lastCommand = millis(); return;
  }
  if(command == "E") {
    // Already moving? Reject re-arm instead of bypassing the stop latch.
    if(targetM1 != 0 || targetM2 != 0 || relayMode != 0) {
      allStop(); Serial.println("ERR,STOP_FIRST"); return;
    }
    permissionEnabled = true; lastCommand = millis();
    Serial.println("A,ENABLED"); return;
  }
  if(command == "Q" || command == "q") { printTelemetry(); return; }
  if(!permissionEnabled) { Serial.println("ERR,DISABLED"); return; }
  if(command == "R,1") {
    relayMode = 1; digitalWrite(RELAY_PIN, RELAY_ON_LEVEL);
    lastCommand = millis(); return;
  }
  if(command == "R,A" || command == "R,a") {
    // No timeout bypass in a cleaning robot.
    allStop(); Serial.println("ERR,ALWAYS_ON_REMOVED"); return;
  }
  if(command.startsWith("M,")) {
    int comma = command.indexOf(',', 2);
    int left, right;
    if(comma < 0 || !parsePWM(command.substring(2,comma), left) ||
       !parsePWM(command.substring(comma+1), right)) {
      allStop(); Serial.println("ERR,BAD_COMMAND"); return;
    }
    setTargets(left,right); return;
  }
  allStop(); Serial.println("ERR,UNKNOWN_COMMAND");
}

// ================================================================
// ================================================================
// READ SERIAL
// ================================================================

void readSerial()
{
  while(
    Serial.available()
  )
  {
    char c =
      Serial.read();


    if(
      c == '\n' ||
      c == '\r'
    )
    {
      if(
        serialBuffer.length() > 0
      )
      {
        processCommand(
          serialBuffer
        );


        serialBuffer =
          "";
      }
    }


    else
    {
      serialBuffer += c;


      // Protect Nano SRAM; overflow cannot leave an armed output running.
      if(
        serialBuffer.length() >
        40
      )
      {
        serialBuffer =
          "";
        allStop();
        Serial.println("ERR,BUFFER_OVERFLOW");
      }
    }
  }
}


// ================================================================
// SETUP
// ================================================================

void setup()
{
  // Preload OFF before selecting OUTPUT to avoid a low pulse at startup.
  digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL);
  pinMode(RELAY_PIN, OUTPUT);

  Serial.begin(
    115200
  );


  serialBuffer.reserve(
    40
  );


  // ============================================================
  // OUTPUTS
  // ============================================================

  pinMode(
    M1_PWM,
    OUTPUT
  );

  pinMode(
    M1_DIR,
    OUTPUT
  );

  pinMode(
    M1_EN,
    OUTPUT
  );


  pinMode(
    M2_PWM,
    OUTPUT
  );

  pinMode(
    M2_DIR,
    OUTPUT
  );

  pinMode(
    M2_EN,
    OUTPUT
  );


  // ============================================================
  // FG INPUTS
  // ============================================================

  pinMode(
    M1_FG,
    INPUT_PULLUP
  );


  pinMode(
    M2_FG,
    INPUT_PULLUP
  );


  // ============================================================
  // START SAFE
  // ============================================================

  analogWrite(
    M1_PWM,
    0
  );


  analogWrite(
    M2_PWM,
    0
  );


  digitalWrite(
    M1_EN,
    DISABLE_LEVEL
  );


  digitalWrite(
    M2_EN,
    DISABLE_LEVEL
  );


  pinMode(
    RELAY_PIN,
    OUTPUT
  );

  digitalWrite(
    RELAY_PIN,
    RELAY_OFF_LEVEL
  );

  relayMode = 0;


  // ============================================================
  // IMU (BNO055)
  //
  // Non-fatal if it fails to init -- motor control must keep
  // working even with the IMU unplugged/faulty. imuOK gates
  // publishImu() so no garbage is sent if init failed.
  // ============================================================

  Wire.begin();

  if(
    bno.begin()
  )
  {
    imuOK = true;

    bno.setExtCrystalUse(
      true
    );

    Serial.println(
      "IMU: BNO055 OK"
    );
  }
  else
  {
    imuOK = false;

    Serial.println(
      "IMU: BNO055 NOT FOUND"
    );
  }


  // ============================================================
  // BATTERY MONITOR (ADS1115)
  //
  // Same non-fatal pattern as the IMU -- motor control must keep
  // working even if this sensor is unplugged/faulty.
  // ============================================================

  if(
    ads.begin()
  )
  {
    adsOK = true;

    Serial.println(
      "BATTERY: ADS1115 OK"
    );
  }
  else
  {
    adsOK = false;

    Serial.println(
      "BATTERY: ADS1115 NOT FOUND"
    );
  }


  motor1Enabled = false;
  motor2Enabled = false;


  motor1DisabledAt =
    millis();


  motor2DisabledAt =
    millis();


  // ============================================================
  // FG INTERRUPTS
  // ============================================================

  attachInterrupt(
    digitalPinToInterrupt(
      M1_FG
    ),
    fg1ISR,
    RISING
  );


  attachInterrupt(
    digitalPinToInterrupt(
      M2_FG
    ),
    fg2ISR,
    RISING
  );


  // ============================================================
  // D10 PWM FREQUENCY
  //
  // Match D10 approximately to D5 (~976 Hz).
  // ============================================================

  TCCR1A =
    _BV(WGM10);


  TCCR1B =
    _BV(WGM12) |
    _BV(CS11) |
    _BV(CS10);


  // ============================================================
  // TIMERS
  // ============================================================

  unsigned long now =
    millis();


  lastSpeedSample =
    now;


  lastTelemetry =
    now;


  lastRamp =
    now;


  lastCommand =
    now;


  lastImuPublish =
    now;


  lastBatteryPublish =
    now;


  lastWheelPublish =
    now;


  // ============================================================
  // READY
  // ============================================================

  Serial.println();

  Serial.println(
    "============================================"
  );

  Serial.println(
    "SAFE BL-4815F MOTOR CONTROLLER"
  );

  Serial.println(
    "Arduino Nano"
  );

  Serial.println(
    "============================================"
  );

  Serial.println(
    "EN-stop + FG restart protection + soft ramp"
  );

  Serial.println(
    "Baud: 115200"
  );

  Serial.println("K,DWM2");
  Serial.println("READY");
}


// ================================================================
// LOOP
// ================================================================

void loop()
{
  // ============================================================
  // SERIAL
  // ============================================================

  readSerial();


  // ============================================================
  // FG / RPM
  // ============================================================

  updateSpeed();


  // ============================================================
  // MOTOR CONTROL
  // ============================================================

  if(permissionEnabled && millis() - lastCommand > COMMAND_TIMEOUT) {
    allStop(); Serial.println("FAILSAFE,STOP");
  }
  updateMotors();


  unsigned long now =
    millis();


  // ============================================================
  // TELEMETRY
  // ============================================================

  if(
    now - lastTelemetry >=
    TELEMETRY_INTERVAL
  )
  {
    printTelemetry();

    lastTelemetry =
      now;
  }


  // ============================================================
  // IMU
  // ============================================================

  if(
    now - lastImuPublish >=
    IMU_PUBLISH_INTERVAL
  )
  {
    publishImu();

    lastImuPublish =
      now;
  }


  // ============================================================
  // BATTERY
  // ============================================================

  if(
    now - lastBatteryPublish >=
    BATTERY_PUBLISH_INTERVAL
  )
  {
    publishBattery();

    lastBatteryPublish =
      now;
  }


  // ============================================================
  // WHEEL ODOMETRY
  // ============================================================

  if(
    now - lastWheelPublish >=
    WHEEL_PUBLISH_INTERVAL
  )
  {
    publishWheelOdom();

    lastWheelPublish =
      now;
  }


  // Timeout has already been checked before motor updates above.
}
