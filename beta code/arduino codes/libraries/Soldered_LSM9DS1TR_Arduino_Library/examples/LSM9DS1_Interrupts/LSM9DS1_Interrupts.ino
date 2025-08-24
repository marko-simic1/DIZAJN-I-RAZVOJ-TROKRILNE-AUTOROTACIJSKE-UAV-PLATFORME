/**
 **************************************************
 *
 * @file        LSM9DS1_Interrupts.ino
 * @brief       This example will show you how to:
 *              -Use the interrupts INT1-A, INT2-A, INTM and RDY-M
 *              -INT1-A is configured to interrupt when accelerometer or gyroscope values exceed a set threshold
 *              -INT2-A is configured to interrupt when new accelerometer or gyroscope values are available
 *              -INT-M is configured to interrupt when the magnetometer exceeds a set threshold
 *              -RDY-M (doesn't need configuring) is used to alert when new magnetometer data is available
 *
 *              To successfully run the sketch:
 *              -Connect the breakout to your Dasduino board according to the diagram below
 *              -Make sure to connect the interrupt pins as well!
 *              -Open the Serial monitor at 115200 baud!
 *
 *              LSM9DS1TR Accelerometer, Gyroscope & Magnetometer: solde.red/333069
 *              Dasduino Core: www.solde.red/333037
 *              Dasduino Connect: www.solde.red/333034
 *              Dasduino ConnectPlus: www.solde.red/333033
 *
 * @authors     Originally made by Jim Lindblom @ SparkFun Electronics
 *              Modified by Soldered
 ***************************************************/

// Include needed libraries
#include "LSM9DS1TR-SOLDERED.h"
#include "Wire.h"

// Set the digital input pins for the interrupts
// These can be swapped to any available digital pin with interrupt support
// NOTE: these pins are 3v3 only on the breakout!
const int INT1_PIN_THS = 3;  // INT1-A pin to D3 - will be attached to gyro
const int INT2_PIN_DRDY = 4; // INT2-A pin to D4 - attached to accel
const int INTM_PIN_THS = 5;  // INT-M pin to D5
const int RDYM_PIN = 6;      // RDY-M pin to D6

/**
 * Connecting diagram:
 *
 * LSM9DS1TR                    Dasduino Core / Connect / ConnectPlus
 * VCC------------------------->VCC
 * GND------------------------->GND
 * SCL------------------------->A5/IO5/IO22
 * SDA------------------------->A4/IO4/IO21
 *
 * Or, simply use an easyC cable!
 *
 * Also connect the interrupts
 * LSM9DS1TR                    Dasduino
 * INT1-A                       INT1_PIN_THS (set by user)
 * INT2-A                       INT2_PIN_DRDY (set by user)
 * INT-M                        INTM_PIN_THS (set by user)
 * RDY-M                        RDYM_PIN (set by user)
 *
 * Make sure to set the pins in the code above
 */

/**
 * After uploading the sketch, accel/gyro/mag values will stream by every second, but that's not really what we're
 * looking for. Try rotating the sensor along the z-axis - an interrupt should fire if you rotate fast enough - exceeding
 * the gyroscope threshold. Then try rotating along the y-axis. The accelerometer int is configured to fire when the x-axis
 * acceleration exceeds a threshold. Finally, try bringing something magnetic near the sensor. The magnetometer interrupt
 * is calibrated to fire when the x-axis mag reading gets too high.
 */

LSM9DS1TR imu; // Create an LSM9DS1 object to use from here on.

// Variable to keep track of when we print sensor readings:
unsigned long lastPrint = 0;

// Function Definitions
void printStats();

// configureIMU sets up our LSM9DS1 interface, sensor scales
// and sample rates.
uint16_t configureIMU()
{
    // gyro.latchInterrupt controls the latching of the
    // gyro and accelerometer interrupts (INT1 and INT2).
    // false = no latching
    imu.settings.gyro.latchInterrupt = false;

    // Set gyroscope scale to +/-245 dps:
    imu.settings.gyro.scale = 245;
    // Set gyroscope (and accel) sample rate to 14.9 Hz
    imu.settings.gyro.sampleRate = 1;
    // Set accelerometer scale to +/-2g
    imu.settings.accel.scale = 2;
    // Set magnetometer scale to +/- 4g
    imu.settings.mag.scale = 4;
    // Set magnetometer sample rate to 0.625 Hz
    imu.settings.mag.sampleRate = 0;

    // Call imu.begin() to initialize the sensor and instill
    // it with our new settings.
    return imu.begin(LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1), Wire); // set addresses and wire port
}

void configureLSM9DS1Interrupts()
{
    /////////////////////////////////////////////
    // Configure INT1 - Gyro & Accel Threshold //
    /////////////////////////////////////////////
    // For more information on setting gyro interrupt, threshold,
    // and configuring the intterup, see the datasheet.
    // We'll configure INT_GEN_CFG_G, INT_GEN_THS_??_G,
    // INT_GEN_DUR_G, and INT1_CTRL.
    // 1. Configure the gyro interrupt generator:
    //  - ZHIE_G: Z-axis high event (more can be or'd together)
    //  - false: and/or (false = OR) (not applicable)
    //  - false: latch interrupt (false = not latched)
    imu.configGyroInt(ZHIE_G, false, false);
    // 2. Configure the gyro threshold
    //   - 500: Threshold (raw value from gyro)
    //   - Z_AXIS: Z-axis threshold
    //   - 10: duration (based on ODR)
    //   - true: wait (wait duration before interrupt goes low)
    imu.configGyroThs(500, Z_AXIS, 10, true);
    // 3. Configure accelerometer interrupt generator:
    //   - XHIE_XL: x-axis high event
    //     More axis events can be or'd together
    //   - false: OR interrupts (N/A, since we only have 1)
    imu.configAccelInt(XHIE_XL, false);
    // 4. Configure accelerometer threshold:
    //   - 20: Threshold (raw value from accel)
    //     Multiply this value by 128 to get threshold value.
    //     (20 = 2600 raw accel value)
    //   - X_AXIS: Write to X-axis threshold
    //   - 10: duration (based on ODR)
    //   - false: wait (wait [duration] before interrupt goes low)
    imu.configAccelThs(20, X_AXIS, 1, false);
    // 5. Configure INT1 - assign it to gyro interrupt
    //   - XG_INT1: Says we're configuring INT1
    //   - INT1_IG_G | INT1_IG_XL: Sets interrupt source to
    //     both gyro interrupt and accel
    //   - INT_ACTIVE_LOW: Sets interrupt to active low.
    //         (Can otherwise be set to INT_ACTIVE_HIGH.)
    //   - INT_PUSH_PULL: Sets interrupt to a push-pull.
    //         (Can otherwise be set to INT_OPEN_DRAIN.)
    imu.configInt(XG_INT1, INT1_IG_G | INT_IG_XL, INT_ACTIVE_LOW, INT_PUSH_PULL);

    ////////////////////////////////////////////////
    // Configure INT2 - Gyro and Accel Data Ready //
    ////////////////////////////////////////////////
    // Configure interrupt 2 to fire whenever new accelerometer
    // or gyroscope data is available.
    // Note XG_INT2 means configuring interrupt 2.
    // INT_DRDY_XL is OR'd with INT_DRDY_G
    imu.configInt(XG_INT2, INT_DRDY_XL | INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);

    //////////////////////////////////////
    // Configure Magnetometer Interrupt //
    //////////////////////////////////////
    // 1. Configure magnetometer interrupt:
    //   - XIEN: axis to be monitored. Can be an or'd combination
    //     of XIEN, YIEN, or ZIEN.
    //   - INT_ACTIVE_LOW: Interrupt goes low when active.
    //   - true: Latch interrupt
    imu.configMagInt(XIEN, INT_ACTIVE_LOW, true);
    // 2. Configure magnetometer threshold.
    //   There's only one threshold value for all 3 mag axes.
    //   This is the raw mag value that must be exceeded to
    //   generate an interrupt.
    imu.configMagThs(10000);
}

void setup()
{
    Serial.begin(115200);
    // Set up our Arduino pins connected to interrupts.
    // We configured all of these interrupts in the LSM9DS1
    // to be active-low.
    pinMode(INT2_PIN_DRDY, INPUT_PULLUP);
    pinMode(INT1_PIN_THS, INPUT_PULLUP);
    pinMode(INTM_PIN_THS, INPUT_PULLUP);
    // The magnetometer DRDY pin (RDY) is not configurable.
    // It is active high and always turned on.
    pinMode(RDYM_PIN, INPUT);

    Wire.begin();

    // Turn on the IMU with configureIMU() (defined above)
    // check the return status of imu.begin() to make sure
    // it's connected.
    uint16_t status = configureIMU();
    if (status == false)
    {
        Serial.print("Failed to connect to IMU: 0x");
        Serial.println(status, HEX);
        while (1)
        {
            delay(100);
        }
    }

    // After turning the IMU on, configure the interrupts:
    configureLSM9DS1Interrupts();
}

void loop()
{
    // Every 1 second (1000 ms), print the last sensor values
    // that were read:
    if (millis() > (lastPrint + 1000))
    {
        printStats();
        lastPrint = millis();
    }

    // INT2 fires when new accelerometer or gyroscope data
    // is available.
    // It's configured to be active LOW:
    if (digitalRead(INT2_PIN_DRDY) == LOW)
    {
        // We don't know if accelerometer or gyroscope data is
        // available.
        // Use accelAvailable and gyroAvailable to check, then
        // read from those sensors if it's new data.
        if (imu.accelAvailable())
            imu.readAccel();
        if (imu.gyroAvailable())
            imu.readGyro();
    }

    // INT1 fires when our gyro or accelerometer thresholds
    // are exceeded.
    // It's configured to be active LOW:
    if (digitalRead(INT1_PIN_THS) == LOW)
    {
        // Let's keep track of how long the interrupt is active.
        // We turned off latching, so this pin will stay low
        // as long as the threshold is exceeded:
        unsigned long durationStart = millis();

        // Call getGyroIntSrc() and getAccelIntSrc() to determine
        // if the gyro or accel generated the input (and why).
        Serial.println("\tINT1 Active!");
        Serial.print("\t\tGyro int: 0x");
        Serial.println(imu.getGyroIntSrc(), HEX);
        Serial.print("\t\tAccel int: 0x");
        Serial.println(imu.getAccelIntSrc(), HEX);

        // While the interrupt remains active, loop:
        while (digitalRead(INT1_PIN_THS) == LOW)
        {
            imu.getGyroIntSrc();
            imu.getAccelIntSrc();
        }
        Serial.print("\tINT1 Duration: ");
        Serial.println(millis() - durationStart);
    }

    // INTM fires when the magnetometer exceeds our set
    // threshold.
    // It's configured to be active LOW:
    if (digitalRead(INTM_PIN_THS) == LOW)
    {
        // Once again, we'll keep track of how line the interrupt
        // stays low
        unsigned long durationStart = millis();

        // Read getMagIntSrc() to see why the interrupt was
        // generated.
        Serial.print("\t\tMag int: 0x");
        Serial.println(imu.getMagIntSrc(), HEX);

        // Loop until the interrupt stops firing
        while (digitalRead(INTM_PIN_THS) == LOW)
        {
        }
        Serial.print("\t\tINTM_PIN_THS Duration: ");
        Serial.println(millis() - durationStart);
    }

    // RDY goes HIGH when new magnetometer is available.
    // AFAICT the active high/low isn't configurable:
    if (digitalRead(RDYM_PIN) == HIGH)
    {
        if (imu.magAvailable())
        {
            imu.readMag();
        }
    }
}

// Print the last read accelerometer, gyro, and mag values:
void printStats()
{
    Serial.println();
    Serial.print("A: ");
    Serial.print(imu.ax);
    Serial.print(", ");
    Serial.print(imu.ay);
    Serial.print(", ");
    Serial.println(imu.az);
    Serial.print("G: ");
    Serial.print(imu.gx);
    Serial.print(", ");
    Serial.print(imu.gy);
    Serial.print(", ");
    Serial.println(imu.gz);
    Serial.print("M: ");
    Serial.print(imu.mx);
    Serial.print(", ");
    Serial.print(imu.my);
    Serial.print(", ");
    Serial.println(imu.mz);
}
