// Bring-up firmware for the SparkFun Qwiic OTOS on a bare Teensy 4.1.
//
// setup() runs every check in order and prints the result of each step:
// I2C scan, identification, self-test, factory configuration, IMU calibration,
// settings, static drift, I2C read timing. loop() then streams the measurements
// so the sensor can be moved by hand and the readings watched live.
// No keyboard input: every setting lives in include/config.h.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

#include "config.h"

static constexpr float kMeterToMm = 1000.0f;
static constexpr float kMmToMeter = 0.001f;
static constexpr float kRadToDeg = 180.0f / PI;
static constexpr float kDegToRad = PI / 180.0f;

// Consecutive failed reads before the sensor is declared lost.
static constexpr uint8_t kMaxConsecutiveErrors = 3;

// Prefix of every non-data line printed during the stream, so CSV plotters skip them.
static const char *const kEventPrefix = STREAM_CSV ? "# " : "";

QwiicOTOS otos;

struct StreamState
{
    sfe_otos_pose2d_t start;
    float lastHeadingRad;
    float unwrappedHeadingRad;
    uint8_t lastStatus;
    uint32_t reads;
    uint32_t errors;
    uint32_t opticalWarnings;
    uint32_t tiltWarnings;
    uint8_t consecutiveErrors;
    bool sensorLost;
    uint32_t lastStreamMs;
    uint32_t lastSummaryMs;
    uint32_t lastReconnectMs;
    sfe_otos_pose2d_t lastPose;
};

StreamState stream;

// === HELPERS ===

// Wraps an angle to [-pi, pi].
static float wrapAngle(float angleRad)
{
    while (angleRad > PI)
        angleRad -= TWO_PI;
    while (angleRad < -PI)
        angleRad += TWO_PI;
    return angleRad;
}

static bool check(const char *what, sfTkError_t err)
{
    if (err == ksfTkErrOk)
        return true;
    Serial.printf("  FAIL: %s (err=%d)\n", what, (int)err);
    return false;
}

static void printStatus(const char *prefix, sfe_otos_status_t status)
{
    Serial.printf("%sstatus 0x%02X (tilt=%u optical=%u paa=%u lsm=%u)\n", prefix, status.value,
                  status.warnTiltAngle, status.warnOpticalTracking, status.errorPaa, status.errorLsm);
    if (status.errorPaa)
        Serial.printf("%s  FATAL: optical sensor (PAA) error\n", prefix);
    if (status.errorLsm)
        Serial.printf("%s  FATAL: IMU (LSM) error\n", prefix);
}

static void printPoseMmDeg(const char *label, const sfe_otos_pose2d_t &pose)
{
    Serial.printf("  %s: x=%.1f mm  y=%.1f mm  h=%.2f deg\n", label, pose.x * kMeterToMm,
                  pose.y * kMeterToMm, pose.h * kRadToDeg);
}

// Resets the tracking and moves it to the configured start pose.
static bool resetPose()
{
    bool ok = check("resetTracking", otos.resetTracking());
    sfe_otos_pose2d_t startPose = {START_X_MM * kMmToMeter, START_Y_MM * kMmToMeter,
                                   START_H_DEG * kDegToRad};
    ok &= check("setPosition", otos.setPosition(startPose));
    return ok;
}

// === STEP 1: I2C SCAN ===

static bool scanI2cBus()
{
    Serial.printf("[STEP 1] I2C scan on Wire1 (SDA=%d, SCL=%d) @ %lu Hz\n", OTOS_SDA_PIN, OTOS_SCL_PIN,
                  (unsigned long)OTOS_I2C_CLOCK_HZ);
    uint8_t found = 0;
    bool otosFound = false;
    for (uint8_t address = 1; address < 127; address++)
    {
        OTOS_WIRE.beginTransmission(address);
        if (OTOS_WIRE.endTransmission() == 0)
        {
            Serial.printf("  device found at 0x%02X\n", address);
            found++;
            if (address == QwiicOTOS::kDefaultAddress)
                otosFound = true;
        }
    }
    if (!otosFound)
    {
        Serial.printf("  FAIL: %u device(s), nothing at 0x%02X - check wiring, 3.3 V, bus\n", found,
                      QwiicOTOS::kDefaultAddress);
        return false;
    }
    Serial.printf("  OK: %u device(s), OTOS present at 0x%02X\n", found, QwiicOTOS::kDefaultAddress);
    return true;
}

// === STEP 2: IDENTIFICATION ===

static bool identifySensor()
{
    Serial.println("[STEP 2] Identification");
    if (!otos.begin(OTOS_WIRE))
    {
        Serial.println("  FAIL: begin() - no ACK at 0x17 or product ID != 0x5F");
        return false;
    }
    Serial.println("  product ID OK (0x5F)");

    sfe_otos_version_t hwVersion;
    sfe_otos_version_t fwVersion;
    if (!check("getVersionInfo", otos.getVersionInfo(hwVersion, fwVersion)))
        return false;
    Serial.printf("  HW v%u.%u, FW v%u.%u\n", hwVersion.major, hwVersion.minor, fwVersion.major,
                  fwVersion.minor);
    return true;
}

// === STEP 3: SELF-TEST ===

static void runSelfTest()
{
    Serial.println("[STEP 3] Self-test");
    Serial.println(otos.selfTest() == ksfTkErrOk ? "  PASS" : "  FAIL: self-test did not pass");

    sfe_otos_status_t status;
    if (check("getStatus", otos.getStatus(status)))
        printStatus("  ", status);
}

// === STEP 4: FACTORY CONFIGURATION ===

static void dumpConfiguration()
{
    Serial.println("[STEP 4] Configuration read from the sensor");

    // Units are a library-side setting, not a sensor register: the library
    // defaults to inches and degrees until setLinearUnit/setAngularUnit is called.
    Serial.printf("  library default units: %s / %s\n",
                  otos.getLinearUnit() == kSfeOtosLinearUnitMeters ? "meter" : "inch",
                  otos.getAngularUnit() == kSfeOtosAngularUnitRadians ? "radian" : "degree");

    float scalar;
    if (check("getLinearScalar", otos.getLinearScalar(scalar)))
        Serial.printf("  linear scalar  %.3f\n", scalar);
    if (check("getAngularScalar", otos.getAngularScalar(scalar)))
        Serial.printf("  angular scalar %.3f\n", scalar);

    // Read the offset in SI units so it can be printed in mm.
    otos.setLinearUnit(kSfeOtosLinearUnitMeters);
    otos.setAngularUnit(kSfeOtosAngularUnitRadians);
    sfe_otos_pose2d_t offset;
    if (check("getOffset", otos.getOffset(offset)))
        printPoseMmDeg("offset", offset);

    sfe_otos_signal_process_config_t config;
    if (check("getSignalProcessConfig", otos.getSignalProcessConfig(config)))
        Serial.printf("  signal process 0x%02X (lut=%u acc=%u rot=%u var=%u)\n", config.value, config.enLut,
                      config.enAcc, config.enRot, config.enVar);
}

// === STEP 5: IMU CALIBRATION ===

static void calibrateImuWithProgress()
{
    Serial.printf("[STEP 5] IMU calibration - put the sensor flat and still, starting in %d s\n",
                  CALIB_SETTLE_MS / 1000);
    for (int remaining = CALIB_SETTLE_MS / 1000; remaining > 0; remaining--)
    {
        Serial.printf("  %d...\n", remaining);
        delay(1000);
    }

    uint32_t startMs = millis();
    if (!check("calibrateImu", otos.calibrateImu(255, false)))
        return;

    uint8_t samplesLeft = 255;
    uint8_t lastPrinted = 255;
    while (samplesLeft > 0)
    {
        if (!check("getImuCalibrationProgress", otos.getImuCalibrationProgress(samplesLeft)))
            return;
        if (samplesLeft + 50 <= lastPrinted || samplesLeft == 0)
        {
            Serial.printf("  samples left %u\n", samplesLeft);
            lastPrinted = samplesLeft;
        }
        if (millis() - startMs > 2000)
        {
            Serial.println("  FAIL: calibration did not finish within 2 s");
            return;
        }
        delay(3);
    }
    Serial.printf("  OK: done in %lu ms\n", (unsigned long)(millis() - startMs));
}

// === STEP 6: SETTINGS ===

static void applySettings()
{
    Serial.println("[STEP 6] Apply settings from config.h");

    otos.setLinearUnit(kSfeOtosLinearUnitMeters);
    otos.setAngularUnit(kSfeOtosAngularUnitRadians);
    Serial.println("  units: meter / radian (printed as mm / deg)");

    if (check("setLinearScalar (must be within [0.872, 1.127])", otos.setLinearScalar(LINEAR_SCALAR)))
        Serial.printf("  linear scalar  %.3f\n", LINEAR_SCALAR);
    if (check("setAngularScalar (must be within [0.872, 1.127])", otos.setAngularScalar(ANGULAR_SCALAR)))
        Serial.printf("  angular scalar %.3f\n", ANGULAR_SCALAR);

    sfe_otos_pose2d_t offset = {OFFSET_X_MM * kMmToMeter, OFFSET_Y_MM * kMmToMeter, OFFSET_H_DEG * kDegToRad};
    if (check("setOffset", otos.setOffset(offset)))
        printPoseMmDeg("offset", offset);

    // Read back what the sensor actually stored (scalars are quantized to 0.1 %).
    float scalar;
    if (check("getLinearScalar", otos.getLinearScalar(scalar)))
        Serial.printf("  read back linear scalar  %.3f\n", scalar);
    if (check("getAngularScalar", otos.getAngularScalar(scalar)))
        Serial.printf("  read back angular scalar %.3f\n", scalar);
    if (check("getOffset", otos.getOffset(offset)))
        printPoseMmDeg("read back offset", offset);
}

// === STEP 7: STATIC DRIFT ===

static void runDriftTest()
{
    Serial.printf("[STEP 7] Static drift over %d s - do not touch the sensor\n", DRIFT_TEST_DURATION_S);
    if (!resetPose())
        return;

    sfe_otos_pose2d_t start;
    if (!check("getPosition", otos.getPosition(start)))
        return;

    sfe_otos_pose2d_t pose = start;
    float maxDistanceMm = 0.0f;
    float maxHeadingDeg = 0.0f;
    uint32_t opticalWarnings = 0;
    uint32_t errors = 0;

    for (int second = 1; second <= DRIFT_TEST_DURATION_S; second++)
    {
        delay(1000);
        if (otos.getPosition(pose) != ksfTkErrOk)
        {
            errors++;
            continue;
        }
        float distanceMm = hypotf(pose.x - start.x, pose.y - start.y) * kMeterToMm;
        float headingDeg = fabsf(wrapAngle(pose.h - start.h)) * kRadToDeg;
        maxDistanceMm = max(maxDistanceMm, distanceMm);
        maxHeadingDeg = max(maxHeadingDeg, headingDeg);

        sfe_otos_status_t status;
        if (otos.getStatus(status) == ksfTkErrOk && status.warnOpticalTracking)
            opticalWarnings++;

        if (second % 10 == 0)
            Serial.printf("  t=%2d s  dist=%.2f mm  dh=%.3f deg\n", second, distanceMm, headingDeg);
    }

    Serial.printf("  final: dx=%.2f mm  dy=%.2f mm  dh=%.3f deg\n", (pose.x - start.x) * kMeterToMm,
                  (pose.y - start.y) * kMeterToMm, wrapAngle(pose.h - start.h) * kRadToDeg);
    Serial.printf("  max:   dist=%.2f mm  dh=%.3f deg\n", maxDistanceMm, maxHeadingDeg);
    Serial.printf("  heading drift rate %.3f deg/min\n",
                  wrapAngle(pose.h - start.h) * kRadToDeg * 60.0f / DRIFT_TEST_DURATION_S);

    sfe_otos_pose2d_t stdDev;
    if (check("getPositionStdDev", otos.getPositionStdDev(stdDev)))
        Serial.printf("  filter std dev: x=%.2f mm  y=%.2f mm  h=%.3f deg\n", stdDev.x * kMeterToMm,
                      stdDev.y * kMeterToMm, stdDev.h * kRadToDeg);
    Serial.printf("  optical warnings %lu, read errors %lu\n", (unsigned long)opticalWarnings,
                  (unsigned long)errors);
}

// === STEP 8: I2C READ TIMING ===

static void runI2cBenchmark()
{
    Serial.printf("[STEP 8] I2C benchmark: %d getPosVelAcc reads @ %lu Hz\n", BENCHMARK_READS,
                  (unsigned long)OTOS_I2C_CLOCK_HZ);
    sfe_otos_pose2d_t pos;
    sfe_otos_pose2d_t vel;
    sfe_otos_pose2d_t acc;
    uint32_t minUs = UINT32_MAX;
    uint32_t maxUs = 0;
    uint64_t totalUs = 0;
    uint32_t errors = 0;

    for (int i = 0; i < BENCHMARK_READS; i++)
    {
        uint32_t startUs = micros();
        sfTkError_t err = otos.getPosVelAcc(pos, vel, acc);
        uint32_t elapsedUs = micros() - startUs;
        if (err != ksfTkErrOk)
            errors++;
        minUs = min(minUs, elapsedUs);
        maxUs = max(maxUs, elapsedUs);
        totalUs += elapsedUs;
    }
    Serial.printf("  per read: mean %lu us, min %lu us, max %lu us | errors %lu / %d\n",
                  (unsigned long)(totalUs / BENCHMARK_READS), (unsigned long)minUs, (unsigned long)maxUs,
                  (unsigned long)errors, BENCHMARK_READS);
}

// === MEASUREMENT STREAM ===

static void startStream()
{
    resetPose();
    otos.getPosition(stream.start);
    stream.lastPose = stream.start;
    stream.lastHeadingRad = stream.start.h;
    stream.unwrappedHeadingRad = stream.start.h;
    stream.lastStatus = 0;
    stream.reads = 0;
    stream.errors = 0;
    stream.opticalWarnings = 0;
    stream.tiltWarnings = 0;
    stream.consecutiveErrors = 0;
    stream.sensorLost = false;
    stream.lastStreamMs = millis();
    stream.lastSummaryMs = millis();

    Serial.println("[STREAM] pose reset - move the sensor by hand");
    if (STREAM_CSV)
        Serial.println("t_ms,x_mm,y_mm,h_deg,h_unwrap_deg,vx_mms,vy_mms,vh_dps,ax_mss,ay_mss,status");
    else
        Serial.println("  x_mm    y_mm    h_deg   h_unwrap | vx_mm/s vy_mm/s vh_deg/s | ax_m/s2 ay_m/s2 | status");
}

// Prints a line only when a status bit changes, so transitions are not lost in the stream.
static void reportStatusChange(sfe_otos_status_t status)
{
    sfe_otos_status_t previous;
    previous.value = stream.lastStatus;
    if (status.value == previous.value)
        return;

    if (status.warnOpticalTracking != previous.warnOpticalTracking)
        Serial.printf("%s[EVENT] optical tracking %s\n", kEventPrefix,
                      status.warnOpticalTracking ? "LOST (IMU only)" : "recovered");
    if (status.warnTiltAngle != previous.warnTiltAngle)
        Serial.printf("%s[EVENT] tilt %s\n", kEventPrefix,
                      status.warnTiltAngle ? "exceeded (accelerometer ignored)" : "back to normal");
    if (status.errorPaa || status.errorLsm)
        printStatus(kEventPrefix, status);

    if (status.warnOpticalTracking && !previous.warnOpticalTracking)
        stream.opticalWarnings++;
    if (status.warnTiltAngle && !previous.warnTiltAngle)
        stream.tiltWarnings++;
    stream.lastStatus = status.value;
}

static void streamMeasurement()
{
    sfe_otos_pose2d_t pos;
    sfe_otos_pose2d_t vel;
    sfe_otos_pose2d_t acc;
    sfe_otos_status_t status;

    sfTkError_t err = otos.getPosVelAcc(pos, vel, acc);
    if (err == ksfTkErrOk)
        err = otos.getStatus(status);
    if (err != ksfTkErrOk)
    {
        stream.errors++;
        Serial.printf("%s[ERROR] read failed (err=%d)\n", kEventPrefix, (int)err);
        if (++stream.consecutiveErrors >= kMaxConsecutiveErrors)
        {
            stream.sensorLost = true;
            stream.lastReconnectMs = millis();
            Serial.printf("%s[ERROR] sensor lost, retrying every %d ms\n", kEventPrefix, RECONNECT_PERIOD_MS);
        }
        return;
    }
    stream.consecutiveErrors = 0;
    stream.reads++;
    reportStatusChange(status);

    stream.unwrappedHeadingRad += wrapAngle(pos.h - stream.lastHeadingRad);
    stream.lastHeadingRad = pos.h;
    stream.lastPose = pos;

    if (STREAM_CSV)
        Serial.printf("%lu,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.2f,%.3f,%.3f,%u\n", (unsigned long)millis(),
                      pos.x * kMeterToMm, pos.y * kMeterToMm, pos.h * kRadToDeg,
                      stream.unwrappedHeadingRad * kRadToDeg, vel.x * kMeterToMm, vel.y * kMeterToMm,
                      vel.h * kRadToDeg, acc.x, acc.y, status.value);
    else
        Serial.printf("%7.1f %7.1f %8.2f %9.2f | %7.1f %7.1f %8.2f | %7.3f %7.3f | 0x%02X\n", pos.x * kMeterToMm,
                      pos.y * kMeterToMm, pos.h * kRadToDeg, stream.unwrappedHeadingRad * kRadToDeg,
                      vel.x * kMeterToMm, vel.y * kMeterToMm, vel.h * kRadToDeg, acc.x, acc.y, status.value);
}

static void printSummary()
{
    float dxMm = (stream.lastPose.x - stream.start.x) * kMeterToMm;
    float dyMm = (stream.lastPose.y - stream.start.y) * kMeterToMm;
    Serial.printf("%s[SUMMARY] from start: dx=%.1f dy=%.1f dist=%.1f mm | h_unwrap=%.2f deg (%.3f turns) | "
                  "reads %lu errors %lu optical_lost %lu tilt %lu\n",
                  kEventPrefix, dxMm, dyMm, hypotf(dxMm, dyMm),
                  (stream.unwrappedHeadingRad - stream.start.h) * kRadToDeg,
                  (stream.unwrappedHeadingRad - stream.start.h) / TWO_PI, (unsigned long)stream.reads,
                  (unsigned long)stream.errors, (unsigned long)stream.opticalWarnings,
                  (unsigned long)stream.tiltWarnings);
}

// A power-cycled OTOS loses its calibration, scalars and offset: redo them all.
static void tryReconnect()
{
    if (millis() - stream.lastReconnectMs < RECONNECT_PERIOD_MS)
        return;
    stream.lastReconnectMs = millis();

    if (!otos.begin(OTOS_WIRE))
    {
        Serial.printf("%s[ERROR] still lost\n", kEventPrefix);
        return;
    }
    Serial.printf("%s[RECOVERED] sensor back - calibration and settings were lost, reapplying\n",
                  kEventPrefix);
    calibrateImuWithProgress();
    applySettings();
    startStream();
}

// === ARDUINO ENTRY POINTS ===

void setup()
{
    Serial.begin(SERIAL_BAUD);
    uint32_t startMs = millis();
    while (!Serial && millis() - startMs < WAIT_FOR_SERIAL_MS)
        ;
    Serial.println();
    Serial.println("=== OTOS bring-up - Teensy 4.1 ===");

    OTOS_WIRE.begin();
    OTOS_WIRE.setClock(OTOS_I2C_CLOCK_HZ);

    while (!scanI2cBus() || !identifySensor())
    {
        Serial.printf("  retrying in %d ms\n", RECONNECT_PERIOD_MS);
        delay(RECONNECT_PERIOD_MS);
    }

    runSelfTest();
    dumpConfiguration();
    calibrateImuWithProgress();
    applySettings();
#if RUN_DRIFT_TEST
    runDriftTest();
#endif
#if RUN_I2C_BENCHMARK
    runI2cBenchmark();
#endif
    startStream();
}

void loop()
{
    if (stream.sensorLost)
    {
        tryReconnect();
        return;
    }

    uint32_t now = millis();
    if (now - stream.lastStreamMs >= STREAM_PERIOD_MS)
    {
        stream.lastStreamMs = now;
        streamMeasurement();
    }
    if (now - stream.lastSummaryMs >= SUMMARY_PERIOD_MS)
    {
        stream.lastSummaryMs = now;
        printSummary();
    }
}
