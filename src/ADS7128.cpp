#include "ADS7128.h"

/**
 * @brief Constructs an ADS7128 driver instance.
 *
 * Stores the I2C address for use in all subsequent communication.
 * Does not initialise hardware — call begin() after construction.
 *
 * @param i2cAddr 7-bit I2C address of the device.
 *                Set by the ADDR pin:
 *                  - ADDR to GND  → 0x10 (default)
 *                  - ADDR to VDD  → 0x11
 *                  - ADDR to SDA  → 0x12
 *                  - ADDR to SCL  → 0x13
 */
ADS7128::ADS7128(uint8_t i2cAddr) {
    _i2cAddr = i2cAddr;
    systemState.autoMode = 0;
}

/**
 * @brief Initialises the ADS7128 and verifies it is present on the I2C bus.
 *
 * Performs the following startup sequence:
 *  1. Probes the I2C bus for the device at the configured address.
 *  2. Issues a software reset to restore all registers to datasheet defaults.
 *  3. Verifies the key configuration registers match their default values.
 *  4. Checks the device-ready status (no CRC errors).
 *  5. Clears the power-on reset (FL_POR) flag in SYSTEM_STATUS.
 *
 * Must be called once before any other library function.
 * Wire.begin() (and Wire.begin(SDA, SCL) on ESP32) should be called
 * before this function.
 *
 * @param wirePort Reference to the TwoWire instance to use (default: Wire).
 *                 Pass Wire1 etc. for alternative I2C buses.
 * @return true  Device found and initialised successfully.
 * @return false Device not found at the configured I2C address.
 */
bool ADS7128::begin(TwoWire &wirePort) {
    _i2cPort = &wirePort;
    
    // Check if device is present
    _i2cPort->beginTransmission(_i2cAddr);
    if (_i2cPort->endTransmission(true) != 0) {
        return false;
    }
    
    #if ADS7128_DEBUG 
    Serial.println("Performing software reset..."); 
    #endif
    if (softwareReset()) {
        #if ADS7128_DEBUG 
        Serial.println("Software reset successful\n");
        #endif
    } else {
        #if ADS7128_DEBUG
        Serial.println("Software reset failed!\n");
        #endif
        return false;
    }
    
    #if ADS7128_DEBUG
    Serial.println("Verifying default configuration...");
    #endif
    if (verifyDefaultConfiguration()) {
        #if ADS7128_DEBUG
        Serial.println("All registers match datasheet defaults\n");
        #endif
    } else {
        #if ADS7128_DEBUG
        Serial.println("Warning: Some registers don't match defaults");
        #endif
        return false;
    }
    
    #if ADS7128_DEBUG
    Serial.println("Checking device ready status...");
    #endif
    if (isDeviceReady()) {
        #if ADS7128_DEBUG
        Serial.println("Device is ready for operation\n");
        #endif
    } else {
        #if ADS7128_DEBUG
        Serial.println("Device is NOT ready - check for errors\n");
        #endif
        return false;
    }
    
    #if ADS7128_DEBUG
    Serial.println("Clearing power-on reset flag...");
    #endif
    if (clearPowerOnResetFlag()) {
        #if ADS7128_DEBUG
        Serial.println("Power-on reset flag cleared\n");
        #endif
    } else {
        #if ADS7128_DEBUG
        Serial.println("Could not clear power-on reset flag\n");
        #endif
        return false;
    }
    
    return true;
}

/**
 * @brief Configures an individual pin as an analog input, digital input,
 *        or digital output.
 *
 * Each of the ADS7128's 8 pins (AIN0–AIN7 / GPIO0–GPIO7) can be
 * independently assigned to one of three functions by writing to the
 * PIN_CFG and GPIO_CFG registers:
 *
 *  - PIN_MODE_ANALOG_INPUT  : Pin used as an ADC input channel (default).
 *  - PIN_MODE_DIGITAL_INPUT : Pin used as a general-purpose digital input.
 *  - PIN_MODE_DIGITAL_OUTPUT: Pin used as a general-purpose digital output.
 *                             Use setDriveMode() to choose open-drain or
 *                             push-pull, and digitalWrite() to set the level.
 *
 * @param pin  Pin number to configure (0–7, corresponding to AIN0/GPIO0
 *             through AIN7/GPIO7).
 * @param mode Desired pin function. One of:
 *               PIN_MODE_ANALOG_INPUT
 *               PIN_MODE_DIGITAL_INPUT
 *               PIN_MODE_DIGITAL_OUTPUT
 * @return true  Configuration written successfully.
 * @return false Pin number out of range (> 7).
 */
bool ADS7128::setPinMode(uint8_t pin, ADS7128PinMode mode) {
    if (pin > 7) return false;
    
    switch (mode) {
        case PIN_MODE_ANALOG_INPUT: {
            // Configure as analog input
            uint8_t pinCfg = readRegister(ADS7128_REG_PIN_CFG);
            pinCfg &= ~(1 << pin);  // Clear bit = analog input
            writeRegister(ADS7128_REG_PIN_CFG, pinCfg);
            break;
        }
            
        case PIN_MODE_DIGITAL_INPUT: {
            // Step 1: Set as GPIO in PIN_CFG
            uint8_t pinCfg = readRegister(ADS7128_REG_PIN_CFG);
            pinCfg |= (1 << pin);
            writeRegister(ADS7128_REG_PIN_CFG, pinCfg);
            
            // Step 2: Set direction as input in GPIO_CFG
            uint8_t gpioCfg = readRegister(ADS7128_REG_GPIO_CFG);
            gpioCfg &= ~(1 << pin);  // Clear bit = input
            writeRegister(ADS7128_REG_GPIO_CFG, gpioCfg);
            break;
        }
            
        case PIN_MODE_DIGITAL_OUTPUT: {
            // Step 1: Set as GPIO in PIN_CFG
            uint8_t pinCfg = readRegister(ADS7128_REG_PIN_CFG);
            pinCfg |= (1 << pin);
            writeRegister(ADS7128_REG_PIN_CFG, pinCfg);
            
            // Step 2: Set direction as output in GPIO_CFG
            uint8_t gpioCfg = readRegister(ADS7128_REG_GPIO_CFG);
            gpioCfg |= (1 << pin);  // Set bit = output
            writeRegister(ADS7128_REG_GPIO_CFG, gpioCfg);
            break;
        }
    }
    
    return true;
}

/**
 * @brief Issues a software reset, restoring all registers to their
 *        power-on default values.
 *
 * Sets the RST bit (bit 0) in the GENERAL_CFG register, waits
 * ADS7128_RESET_DELAY_MS (10 ms) for the reset to complete, then
 * clears the bit. An additional 1 ms settling delay is applied
 * before returning.
 *
 * All pin configurations, thresholds, and operating modes are lost.
 * Reconfigure the device after calling this function.
 *
 * @return true  Reset completed successfully.
 * @return false I2C write failed during reset sequence.
 */
bool ADS7128::softwareReset() {
    // Issue software reset by setting RST bit in GENERAL_CFG
    if (!writeRegister(ADS7128_REG_GENERAL_CFG, 0x01)) {
        return false;
    }
    
    // Wait for reset to complete (minimum 10ms per datasheet)
    delay(ADS7128_RESET_DELAY_MS);
    
    // Clear the reset bit
    if (!writeRegister(ADS7128_REG_GENERAL_CFG, 0x00)) {
        return false;
    }
    
    // Additional small delay for device stabilization
    delay(1);
    
    return true;
}

/**
 * @brief Reads key configuration registers and verifies they match the
 *        datasheet power-on default values.
 *
 * Checks the following registers against their expected defaults:
 *  - GENERAL_CFG  (0x01) expected 0x00
 *  - DATA_CFG     (0x02) expected 0x00
 *  - OSR_CFG      (0x03) expected 0x00
 *  - OPMODE_CFG   (0x04) expected 0x00
 *  - PIN_CFG      (0x05) expected 0x00
 *  - GPIO_CFG     (0x07) expected 0x00
 *  - GPO_DRIVE_CFG(0x09) expected 0x00
 *
 * Register values are printed to Serial for debugging.
 * Typically called automatically by begin().
 *
 * @return true  All checked registers match their datasheet defaults.
 * @return false One or more registers do not match the expected defaults,
 *               which may indicate a communication error or that the device
 *               was not reset before calling this function.
 */
bool ADS7128::verifyDefaultConfiguration() {
    // Read and verify key registers match datasheet defaults
    uint8_t generalCfg = readRegister(ADS7128_REG_GENERAL_CFG);
    uint8_t dataCfg = readRegister(ADS7128_REG_DATA_CFG);
    uint8_t osrCfg = readRegister(ADS7128_REG_OSR_CFG);
    uint8_t opmodeCfg = readRegister(ADS7128_REG_OPMODE_CFG);
    uint8_t pinCfg = readRegister(ADS7128_REG_PIN_CFG);
    uint8_t gpioCfg = readRegister(ADS7128_REG_GPIO_CFG);
    uint8_t gpoDriveCfg = readRegister(ADS7128_REG_GPO_DRIVE_CFG);

    #if ADS7128_DEBUG
    Serial.print("generalCfg=0x"); Serial.println(generalCfg, HEX);
    Serial.print("dataCfg=0x"); Serial.println(dataCfg, HEX);
    Serial.print("osrCfg=0x"); Serial.println(osrCfg, HEX);
    Serial.print("opmodeCfg=0x"); Serial.println(opmodeCfg, HEX);
    Serial.print("pinCfg=0x"); Serial.println(pinCfg, HEX);
    Serial.print("gpioCfg=0x"); Serial.println(gpioCfg, HEX);
    Serial.print("gpoDriveCfg=0x"); Serial.println(gpoDriveCfg, HEX);
    #endif
    
    // Verify critical configuration registers
    bool valid = true;
    
    if (generalCfg != ADS7128_DEFAULT_GENERAL_CFG) {
        valid = false;
    }
    if (dataCfg != ADS7128_DEFAULT_DATA_CFG) {
        valid = false;
    }
    if (osrCfg != ADS7128_DEFAULT_OSR_CFG) {
        valid = false;
    }
    if (opmodeCfg != ADS7128_DEFAULT_OPMODE_CFG) {
        valid = false;
    }
    if (pinCfg != ADS7128_DEFAULT_PIN_CFG) {
        valid = false;
    }
    if (gpioCfg != ADS7128_DEFAULT_GPIO_CFG) {
        valid = false;
    }
    if (gpoDriveCfg != ADS7128_DEFAULT_GPO_DRIVE_CFG) {
        valid = false;
    }
    
    return valid;
}

/**
 * @brief Reads and parses the SYSTEM_STATUS register (address 0x00).
 *
 * Returns a SystemStatus struct with the following fields populated:
 *  - sequenceActive : true if an auto-sequence conversion is in progress.
 *  - highSpeedI2C   : true if the device detected a high-speed I2C start
 *                     condition (3.4 MHz mode).
 *  - osrDone        : true if an oversampling operation has completed.
 *  - crcError       : true if a CRC mismatch was detected on the last
 *                     I2C transaction. Indicates bus noise or wiring issues.
 *  - powerOnReset   : true if the device has undergone a power-on reset
 *                     since the flag was last cleared.
 *  - rawValue       : the raw 8-bit register value for direct inspection.
 *
 * @return SystemStatus struct containing all parsed status flags and the
 *         raw register byte.
 */
SystemStatus ADS7128::getSystemStatus() {
    SystemStatus status;
    
    // Read SYSTEM_STATUS register
    status.rawValue = readRegister(ADS7128_REG_SYSTEM_STATUS);
    
    // Parse individual status bits
    status.sequenceActive = (status.rawValue & ADS7128_STATUS_SEQ_STATUS) != 0;
    status.highSpeedI2C = (status.rawValue & ADS7128_STATUS_I2C_SPEED) != 0;
    status.osrDone = (status.rawValue & ADS7128_STATUS_OSR_DONE) != 0;
    status.crcError = (status.rawValue & ADS7128_STATUS_CRC_ERR) != 0;
    status.powerOnReset = (status.rawValue & ADS7128_STATUS_FL_POR) != 0;
    
    return status;
}

/**
 * @brief Checks whether the device is in a healthy, error-free state.
 *
 * Reads the SYSTEM_STATUS register and returns false if a CRC error is
 * present. Additional error conditions may be checked here in future.
 *
 * A CRC error (FL_CRC_ERR bit) indicates corrupted I2C communication
 * since the last status read. Check wiring, pull-up resistors, and
 * I2C bus speed if this returns false.
 *
 * @return true  Device is ready — no error flags detected.
 * @return false A CRC error (or other fault) has been detected.
 */
bool ADS7128::isDeviceReady() {
    SystemStatus status = getSystemStatus();
    
    // Device is ready if:
    // - No CRC errors
    // - OSR is done (if applicable)
    // - Not in an error state
    
    if (status.crcError) {
        return false;
    }
    
    return true;
}

/**
 * @brief Clears the power-on reset flag (FL_POR) in the SYSTEM_STATUS register.
 *
 * FL_POR is set automatically by the device whenever it completes a
 * power-on or software reset. Writing a 1 to bit 0 of SYSTEM_STATUS
 * clears the flag (write-1-to-clear / W1C behaviour).
 *
 * This is called automatically by begin(). You may call it again any time
 * to acknowledge a reset event detected via getSystemStatus().
 *
 * @return true  FL_POR flag was successfully cleared and verified.
 * @return false FL_POR flag could not be cleared (I2C error or device fault).
 */
bool ADS7128::clearPowerOnResetFlag() {
    // TODO: Change to constant mask, do not read then write
    uint8_t value = readRegister(ADS7128_REG_SYSTEM_STATUS);

    value |= (1 << 0);

    writeRegister(ADS7128_REG_SYSTEM_STATUS, value);
    
    // Verify flag is cleared
    SystemStatus status = getSystemStatus();
    return !status.powerOnReset;
}

/**
 * @brief Returns the current operating mode of a pin.
 *
 * Reads PIN_CFG and GPIO_CFG registers to determine how the given pin
 * is currently configured. The pin number maps directly to the bit
 * position in both registers.
 *
 * @param pin Pin number to query (0–7, corresponding to AIN0/GPIO0
 *            through AIN7/GPIO7).
 * @return PIN_MODE_ANALOG_INPUT   Pin is configured as an ADC channel.
 * @return PIN_MODE_DIGITAL_INPUT  Pin is configured as a GPIO input.
 * @return PIN_MODE_DIGITAL_OUTPUT Pin is configured as a GPIO output.
 * @return PIN_MODE_INVALID        Pin number out of range (> 7).
 */
ADS7128PinMode ADS7128::getPinMode(uint8_t pin) {
    if (pin > 7) return PIN_MODE_INVALID;
    
    uint8_t pinCfg = readRegister(ADS7128_REG_PIN_CFG);
    uint8_t gpioCfg = readRegister(ADS7128_REG_GPIO_CFG);
    
    if (!(pinCfg & (1 << pin))) {
        return PIN_MODE_ANALOG_INPUT;
    } else if (gpioCfg & (1 << pin)) {
        return PIN_MODE_DIGITAL_OUTPUT;
    } else {
        return PIN_MODE_DIGITAL_INPUT;
    }
}

/**
 * @brief Sets the output driver type for a GPIO output pin.
 *
 * Configures bit N of the GPO_DRIVE_CFG register for the given pin.
 * Only meaningful when the pin has been configured as PIN_MODE_DIGITAL_OUTPUT
 * via setPinMode(). Has no effect on analog input or digital input pins.
 *
 *  - DRIVE_MODE_OPEN_DRAIN : Pin is driven low when output is 0; floats
 *                            (high-Z) when output is 1. Requires an
 *                            external pull-up resistor.
 *  - DRIVE_MODE_PUSH_PULL  : Pin actively drives both high and low levels.
 *                            No external pull-up required.
 *
 * @param pin  Pin number to configure (0–7).
 * @param mode DRIVE_MODE_OPEN_DRAIN or DRIVE_MODE_PUSH_PULL.
 * @return true  Drive mode written successfully.
 * @return false Pin number out of range (> 7), or I2C write failed.
 */
bool ADS7128::setDriveMode(uint8_t pin, DriveMode mode) {
    if (pin > 7) return false;
    
    uint8_t gpoDrive = readRegister(ADS7128_REG_GPO_DRIVE_CFG);
    
    if (mode == DRIVE_MODE_PUSH_PULL) {
        gpoDrive |= (1 << pin);   // Set bit = push-pull
    } else {
        gpoDrive &= ~(1 << pin);  // Clear bit = open-drain
    }
    
    return writeRegister(ADS7128_REG_GPO_DRIVE_CFG, gpoDrive);
}

/**
 * @brief Returns the current output driver type for a GPIO pin.
 *
 * Reads bit N of the GPO_DRIVE_CFG register.
 *
 * @param pin Pin number to query (0–7).
 * @return DRIVE_MODE_PUSH_PULL  Pin is configured for push-pull drive.
 * @return DRIVE_MODE_OPEN_DRAIN Pin is configured for open-drain drive,
 *                               or pin number was out of range (> 7).
 */
DriveMode ADS7128::getDriveMode(uint8_t pin) {
    if (pin > 7) return DRIVE_MODE_OPEN_DRAIN;
    
    uint8_t gpoDrive = readRegister(ADS7128_REG_GPO_DRIVE_CFG);
    
    if (gpoDrive & (1 << pin)) {
        return DRIVE_MODE_PUSH_PULL;
    } else {
        return DRIVE_MODE_OPEN_DRAIN;
    }
}

/**
 * @brief Sets the output level of a GPIO output pin.
 *
 * Writes bit N of the GPO_VALUE register (address 0x0B).
 * The pin must have been configured as PIN_MODE_DIGITAL_OUTPUT via
 * setPinMode() before calling this function.
 *
 * In open-drain mode (DRIVE_MODE_OPEN_DRAIN), writing true (1) releases
 * the pin to float; writing false (0) actively pulls it low.
 * In push-pull mode (DRIVE_MODE_PUSH_PULL), the pin is driven to VDD
 * or GND respectively.
 *
 * @param pin   Pin number to drive (0–7, corresponding to GPIO0–GPIO7).
 * @param value true = logic HIGH; false = logic LOW.
 * @return true  Output level written successfully.
 * @return false Pin number out of range (> 7), or I2C write failed.
 */
bool ADS7128::digitalWrite(uint8_t pin, bool value) {
    if (pin > 7) return false;
    
    uint8_t gpoValue = readRegister(ADS7128_REG_GPO_VALUE);
    
    if (value) {
        gpoValue |= (1 << pin);
    } else {
        gpoValue &= ~(1 << pin);
    }
    
    return writeRegister(ADS7128_REG_GPO_VALUE, gpoValue);
}

/**
 * @brief Reads the logic level of a GPIO input pin.
 *
 * Reads bit N of the GPI_VALUE register (address 0x0D).
 * The pin must have been configured as PIN_MODE_DIGITAL_INPUT via
 * setPinMode() before calling this function.
 *
 * @param pin Pin number to read (0–7, corresponding to GPIO0–GPIO7).
 * @return true  Pin is at logic HIGH.
 * @return false Pin is at logic LOW, or pin number out of range (> 7).
 */
bool ADS7128::digitalRead(uint8_t pin) {
    if (pin > 7) return false;
    
    uint8_t gpiValue = readRegister(ADS7128_REG_GPI_VALUE);
    return (gpiValue & (1 << pin)) != 0;
}

/**
 * @brief Reads a 12-bit raw ADC value from the specified channel.
 *
 * Automatically dispatches to the correct internal read method based on
 * the current operating mode:
 *  - Manual mode   (enableManualMode)   : triggers a single conversion,
 *                                         waits 10 µs, then reads the result.
 *  - Autonomous mode (enableAutonomousMode): reads the most recently
 *                                         completed conversion from the
 *                                         RECENT_CHn registers directly.
 *
 * The pin must be configured as PIN_MODE_ANALOG_INPUT via setPinMode()
 * and must be included in the channel mask passed to the mode-enable function.
 *
 * @param channel ADC channel to read (0–7, corresponding to AIN0–AIN7).
 * @return uint16_t Raw 12-bit conversion result (0–4095).
 *                  Returns 0 if the channel number is out of range.
 */
uint16_t ADS7128::analogRead(uint8_t channel) {

    if (systemState.autoMode == 1)
        return readChannelDataAutonomous(channel);
    else
        return readChannelDataManual(channel);
}

/**
 * @brief Reads the voltage on an ADC channel, converted to floating-point volts.
 *
 * Calls analogRead() internally to obtain the raw 12-bit result, then
 * scales it against the supplied reference voltage:
 *
 *   voltage = (rawValue / 4095.0) * vref
 *
 * The pin must be configured as PIN_MODE_ANALOG_INPUT via setPinMode().
 *
 * @param channel ADC channel to read (0–7, corresponding to AIN0–AIN7).
 * @param vref    Reference voltage in volts. Must match the voltage applied
 *                to the AVDD pin of the ADS7128 (default: 3.3V).
 * @return float  Measured voltage in volts (0.0 V – vref).
 */
float ADS7128::analogReadVoltage(uint8_t channel, float vref) {
    uint16_t rawValue = analogRead(channel);
    return (rawValue * vref) / 4096.0f;
}

/**
 * @brief Switches the device to manual (on-demand) conversion mode.
 *
 * In manual mode, each call to analogRead() triggers a single conversion
 * on the selected channel. This is suitable for low-speed or infrequent
 * measurements where autonomous background scanning is not required.
 *
 * Sets CONV_MODE = 0 (manual) in OPMODE_CFG and SEQ_MODE = 0 / SEQ_START = 0
 * in SEQUENCE_CFG. Updates the internal mode flag used by analogRead().
 *
 * @return true  Mode registers written successfully.
 * @return false One or more I2C writes failed.
 */
bool ADS7128::enableManualMode() {
    uint8_t reg = readRegister(ADS7128_REG_GENERAL_CFG);
    reg |= (1 << 5);
    bool success = writeRegister(ADS7128_REG_GENERAL_CFG, reg);           // Enable STATES registers

    // Configure for manual mode
    success &= writeRegister(ADS7128_REG_OPMODE_CFG, 0x00);    // CONV_MODE=0 (manual)
    success &= writeRegister(ADS7128_REG_SEQUENCE_CFG, 0x00);  // SEQ_MODE=0 (manual), SEQ_START=0

    systemState.autoMode = 0;

    return success;
}

/**
 * @brief Switches the device to autonomous (continuous background) conversion mode.
 *
 * In autonomous mode, the ADS7128 continuously cycles through the selected
 * channels without host intervention. Each channel's most recent result is
 * stored in its RECENT_CHn_LSB / RECENT_CHn_MSB registers and can be read
 * at any time via analogRead() without triggering a new conversion.
 *
 * This mode is ideal for continuous monitoring or multi-channel scanning.
 *
 * Internally sets:
 *  - STATS_EN (bit 5 of GENERAL_CFG) to enable RECENT registers.
 *  - AUTO_SEQ_CH_SEL to the provided channel bitmask.
 *  - CONV_MODE = 1 (autonomous) in OPMODE_CFG.
 *  - SEQ_MODE = 1 (auto-sequence) and SEQ_START = 1 in SEQUENCE_CFG.
 *
 * @param channels Bitmask of channels to include in the autonomous scan.
 *                 Bit N corresponds to channel N (AIN0–AIN7).
 *                 Example: 0x1F enables channels 0–4.
 * @return true  Mode configured and scan started successfully.
 * @return false One or more I2C writes failed.
 */
bool ADS7128::enableAutonomousMode(uint8_t channels) {

    // Enable autonomous on selected channels
    uint8_t reg = readRegister(ADS7128_REG_GENERAL_CFG);
    reg |= (1 << 5);
    bool success = writeRegister(ADS7128_REG_GENERAL_CFG, reg);           // Enable STATES registers

    success &= writeRegister(ADS7128_REG_AUTO_SEQ_CH_SEL, channels);   // AIN0..7 enabled
    success &= writeRegister(ADS7128_REG_OPMODE_CFG, 0x20);            // CONV_MODE=1 (autonomous)
    success &= writeRegister(ADS7128_REG_SEQUENCE_CFG, 0x11);          // SEQ_MODE=1 (auto-seq), SEQ_START=1

    systemState.autoMode = 1;

    return success;
}

/**
 * @brief Enables or disables the window comparator feature.
 *
 * The window comparator allows the ADS7128 to autonomously monitor ADC
 * channels against programmable high and low voltage thresholds. When a
 * channel's value crosses a threshold, an alert event is generated which
 * can drive the ALERT pin or trigger GPIO output pins.
 *
 * Sets or clears the WIN_CMP_EN bit (bit 4) in GENERAL_CFG.
 *
 * Configure thresholds with setChannelWindow() and setChannelHysteresis()
 * before enabling. Set alert routing with setAlertLogic(), setAlertChannels(),
 * setWindowRegion(), setTriggerOn(), setTriggerPins(), and setValueOnTrigger().
 *
 * @param state true  = Enable window comparator mode.
 *              false = Disable window comparator mode.
 * @return true  Register written successfully.
 * @return false I2C write failed.
 */
bool ADS7128::enableWindowMode(bool state) {

    // Enable window on all channels
    uint8_t reg = readRegister(ADS7128_REG_GENERAL_CFG);

    if (state) reg |= (1 << 4);
    else       reg &= ~(1 << 4);

    return writeRegister(ADS7128_REG_GENERAL_CFG, reg);           // Enable WINDOW mode
}

/**
 * @brief Configures the high threshold, low threshold, and event count for
 *        the window comparator on a single ADC channel.
 *
 * The window comparator monitors the channel's ADC result against the
 * defined voltage window. An alert event is generated after the result
 * has been outside (or inside, see setWindowRegion()) the window for
 * the specified number of consecutive conversions.
 *
 * Threshold voltages are converted to 12-bit codes and written across
 * the four per-channel registers:
 *   HYSTERESIS_CHn (0x20 + 4n) — stores high threshold LSB nibble [3:0]
 *   HIGH_TH_CHn    (0x21 + 4n) — stores high threshold MSB [11:4]
 *   EVENT_COUNT_CHn(0x22 + 4n) — stores low threshold LSB nibble [3:0]
 *                                 and event count [3:0]
 *   LOW_TH_CHn     (0x23 + 4n) — stores low threshold MSB [11:4]
 *
 * Call setChannelHysteresis() separately to set the hysteresis band.
 * This function preserves the hysteresis nibble already stored in
 * HYSTERESIS_CHn.
 *
 * Requires enableWindowMode(true) and autonomous mode to be active.
 *
 * @param channel    Channel to configure (0–7, corresponding to AIN0–AIN7).
 * @param windowMax  Upper voltage threshold in volts. Must be ≤ Vref.
 * @param windowMin  Lower voltage threshold in volts. Must be ≥ 0 V.
 * @param eventCount Number of consecutive threshold violations required
 *                   before an alert fires (1–15; clamped to 4 bits).
 * @param Vref       Reference voltage in volts, matching AVDD (default 3.3V).
 * @return true  All threshold registers written successfully.
 * @return false Channel out of range (> 7), or one or more I2C writes failed.
 */
bool ADS7128::setChannelWindow(uint8_t channel, float windowMax, float windowMin, uint8_t eventCount, float Vref) {
    if (channel > 7) return false;

    uint16_t Vmax = voltageToCode(windowMax, Vref);
    uint16_t Vmin = voltageToCode(windowMin, Vref);

    // Clamp 12‑bit codes and 4‑bit eventCount
    Vmax  &= 0x0FFF;
    Vmin  &= 0x0FFF;
    eventCount &= 0x0F;

    // Base addresses for this channel block
    uint8_t hystReg   = ADS7128_REG_HYSTERESIS_CH0   + 4 * channel; // 0x20 + 4n
    uint8_t highReg   = ADS7128_REG_HIGH_TH_CH0      + 4 * channel; // 0x21 + 4n
    uint8_t eventReg  = ADS7128_REG_EVENT_COUNT_CH0  + 4 * channel; // 0x22 + 4n
    uint8_t lowReg    = ADS7128_REG_LOW_TH_CH0       + 4 * channel; // 0x23 + 4n

    // Split high / low threshold into MSB [11:4] and LSB [3:0]
    uint8_t high_msb = (Vmax >> 4) & 0xFF;
    uint8_t high_lsb = (Vmax & 0x0F);

    uint8_t low_msb  = (Vmin >> 4) & 0xFF;
    uint8_t low_lsb  = (Vmin & 0x0F);

    // HYSTERESIS_CHn:
    //  bits 7:4 = HIGH_THRESHOLD_CHn_LSB[3:0]
    //  bits 3:0 = HYSTERESIS_CHn[3:0] (leave as‑is, set via setChannelHysteresis)
    uint8_t hyst_val = readRegister(hystReg);
    hyst_val = (uint8_t)((high_lsb << 4) | (hyst_val & 0x0F));
    if (!writeRegister(hystReg, hyst_val)) return false;

    // HIGH_TH_CHn: MSB aligned high threshold bits [11:4]
    if (!writeRegister(highReg, high_msb)) return false;

    // EVENT_COUNT_CHn:
    //  bits 7:4 = LOW_THRESHOLD_CHn_LSB[3:0]
    //  bits 3:0 = EVENT_COUNT_CHn[3:0]
    uint8_t event_val = (uint8_t)((low_lsb << 4) | (eventCount & 0x0F));
    if (!writeRegister(eventReg, event_val)) return false;

    // LOW_TH_CHn: MSB aligned low threshold bits [11:4]
    if (!writeRegister(lowReg, low_msb)) return false;

    return true;
}

/**
 * @brief Sets the hysteresis band for the window comparator on a single channel.
 *
 * Hysteresis prevents rapid alert toggling ("chattering") when the ADC
 * result hovers near a threshold. Once an alert fires, the input must
 * move back inside the window by at least the hysteresis voltage before
 * the alert is de-asserted.
 *
 * The hardware encodes hysteresis as a 4-bit value (0–15) in steps of
 * 8 ADC codes each. The actual hysteresis in volts is therefore:
 *   hysteresis = hyst_steps × 8 × (Vref / 4096)
 *
 * This function converts the supplied voltage to the nearest 4-bit step
 * and writes it to bits [3:0] of HYSTERESIS_CHn, preserving bits [7:4]
 * which hold the high-threshold LSB (set by setChannelWindow()).
 *
 * Call setChannelWindow() before this function to avoid overwriting the
 * hysteresis register with incorrect data.
 *
 * @param channel      Channel to configure (0–7, corresponding to AIN0–AIN7).
 * @param hysteresis_v Desired hysteresis in volts (e.g. 0.1 for 100 mV).
 *                     Clamped to the maximum representable value (~0.29 V
 *                     at 3.3 V reference; 15 steps × 8 codes × Vref/4096).
 * @param Vref         Reference voltage in volts, matching AVDD (default 3.3V).
 * @return true  Hysteresis register written successfully.
 * @return false Channel out of range (> 7), or I2C write failed.
 */
bool ADS7128::setChannelHysteresis(uint8_t channel, float hysteresis_v, float Vref) {
    if (channel > 7) return false;

    // Step 1: codes of hysteresis
    float lsb = Vref / 4096.0f;
    float codes = hysteresis_v / lsb;

    // Step 2: 4-bit hysteresis code (multiple of 8 codes internally)
    uint8_t hyst_steps = (uint8_t)roundf(codes / 8.0f);
    // Datasheet: 4‑bit HYSTERESIS_CHn[3:0], left‑shifted by 3 internally
    // You can pass a 0–15 “step” value; clamp just in case.
    if (hyst_steps > 0x0F) hyst_steps = 0x0F;

    uint8_t hystReg = ADS7128_REG_HYSTERESIS_CH0 + 4 * channel;

    // Preserve high‑threshold LSB nibble (bits 7:4), update hysteresis nibble (3:0)
    uint8_t hyst_val = readRegister(hystReg);
    hyst_val = (uint8_t)((hyst_val & 0xF0) | hyst_steps);

    return writeRegister(hystReg, hyst_val);
}

/**
 * @brief Configures the ALERT pin drive type and active logic level.
 *
 * The ALERT pin is an output that the ADS7128 asserts when a window
 * comparator event occurs on any monitored channel. This function
 * writes the ALERT_PIN_CFG register (address 0x17) to set:
 *   - bits [1:0] : Alert logic / polarity mode (see below).
 *   - bit  [2]   : Alert pin drive type (0 = open-drain, 1 = push-pull).
 *
 * Logic mode values (bits [1:0]):
 *   0x00 = Active-low, non-latching
 *   0x01 = Active-high, non-latching
 *   0x02 = Active-low, latching
 *   0x03 = Active-high, latching
 *
 * @param drive false = Open-drain (requires external pull-up resistor).
 *              true  = Push-pull (actively driven both ways).
 * @param logic Alert logic/polarity mode (0–3, see above).
 * @return true  ALERT_PIN_CFG written successfully.
 * @return false Logic value out of range (> 3), or I2C write failed.
 */
bool ADS7128::setAlertLogic(bool drive, uint8_t logic) {
    if (logic > 3) return false;

    // Mask upper bits,  
    uint8_t val = (uint8_t)(logic & 0x03) | (drive << 2);

    return writeRegister(ADS7128_REG_ALERT_PIN_CFG, val);
}

/**
 * @brief Selects which channels are monitored by the window comparator
 *        alert system.
 *
 * Writes a bitmask to the ALERT_CH_SEL register (address 0x14).
 * Each set bit enables alert monitoring for the corresponding channel.
 * Channels must also have thresholds configured via setChannelWindow().
 *
 * @param channels Bitmask of channels to monitor.
 *                 Bit N corresponds to channel N (AIN0–AIN7).
 *                 Example: 0x18 enables alert monitoring on channels 3 and 4.
 * @return true  Register written successfully.
 * @return false I2C write failed.
 */
bool ADS7128::setAlertChannels(uint8_t channels) {

    return writeRegister(ADS7128_REG_ALERT_CH_SEL, channels);
}

/**
 * @brief Defines whether alert events are generated when the ADC result
 *        is inside or outside the configured voltage window.
 *
 * Writes to the EVENT_RGN register (address 0x1E). Each bit independently
 * controls the trigger region for the corresponding channel:
 *   - Bit = 0 : Alert fires when the result is OUTSIDE the window
 *               (below windowMin or above windowMax). Default behaviour.
 *   - Bit = 1 : Alert fires when the result is INSIDE the window
 *               (between windowMin and windowMax).
 *
 * @param regions Bitmask where each bit selects the alert region for
 *                the corresponding channel (0–7).
 *                Example: 0x00 = all channels alert on out-of-window.
 *                         0xFF = all channels alert on in-window.
 * @return true  Register written successfully.
 * @return false I2C write failed.
 */
bool ADS7128::setWindowRegion(uint8_t regions) {

    return writeRegister(ADS7128_REG_EVENT_RGN, regions);   // 0 = outside 1 = inside window
}

/**
 * @brief Enables or disables hardware GPIO triggering for output pins
 *        in response to window comparator alert events.
 *
 * Writes a bitmask to the GPO_TRIGGER_CFG register (address 0xE9).
 * When a bit is set, the corresponding GPIO output pin is automatically
 * driven to the level defined by setValueOnTrigger() whenever its
 * mapped alert event fires (configured via setTriggerPins()).
 *
 * GPIO pins must be configured as outputs via setPinMode() before
 * hardware triggering will take effect.
 *
 * @param pins Bitmask of GPIO output pins to enable for hardware triggering.
 *             Bit N corresponds to GPIO N.
 *             Example: 0x60 enables hardware triggering on GPIO5 and GPIO6.
 * @return true  Register written successfully.
 * @return false I2C write failed.
 */
bool ADS7128::setTriggerOn(uint8_t pins) {

    return writeRegister(ADS7128_REG_GPO_TRIGGER_CFG, pins);   // 0 = no trigger 1 = trigger
}

/**
 * @brief Maps window comparator alert sources to a specific GPIO output pin.
 *
 * Writes an alert-source bitmask to the GPO_TRIG_EVENT_SEL register for
 * the given output pin (base address 0xC3, offset 2 × pin).
 * When any of the mapped alert channels fires, the GPIO pin is automatically
 * driven to the level set by setValueOnTrigger().
 *
 * Must be used together with setTriggerOn() to enable triggering on the pin,
 * and setAlertChannels() / setChannelWindow() to configure the alert sources.
 *
 * @param pin    GPIO output pin to configure (0–7).
 * @param alerts Bitmask of alert channels that should trigger this pin.
 *               Bit N corresponds to the alert from channel N (AIN0–AIN7).
 *               Example: To trigger GPIO5 when channels 3 or 4 alert:
 *                 setTriggerPins(5, (1 << 3) | (1 << 4));
 * @return true  Register written successfully.
 * @return false Pin out of range (> 7), or I2C write failed.
 */
bool ADS7128::setTriggerPins(uint8_t pin, uint8_t alerts) {
    if  (pin > 7) return false;

    uint8_t reg = ADS7128_REG_GPO0_TRIG_EVENT_SEL + (2 * pin); 

    return writeRegister(reg, alerts); //Set which pins trigger which IO
}

/**
 * @brief Sets the output logic level that a GPIO pin will be driven to
 *        when its mapped window comparator alert fires.
 *
 * Writes bit N of the GPO_VALUE_ON_TRIGGER register (address 0xEB).
 * The hardware automatically drives the pin to this level when the
 * trigger event occurs, without any firmware intervention.
 *
 * Use setTriggerOn() to enable triggering on the pin and setTriggerPins()
 * to map the alert source.
 *
 * @param pin   GPIO output pin to configure (0–7, corresponding to GPIO0–GPIO7).
 * @param state true  = Pin is driven HIGH when the alert fires.
 *              false = Pin is driven LOW when the alert fires.
 * @return true  Register written successfully.
 * @return false I2C write failed.
 */
bool ADS7128::setValueOnTrigger(uint8_t pin, bool state) {

    uint8_t reg = readRegister(ADS7128_REG_GPO_VALUE_ON_TRIGGER);

    if (state) reg |= (1 << pin);
    else       reg &= ~(1 << pin);

    return writeRegister(ADS7128_REG_GPO_VALUE_ON_TRIGGER, reg);  // Set pin value on trigger
}

/**
 * @brief Triggers a single on-demand conversion and reads the result for
 *        the specified channel. Used internally by analogRead() in manual mode.
 *
 * Sequence:
 *  1. Writes the channel number to MANUAL_CHID (bits [3:0] of CHANNEL_SEL),
 *     preserving the ZCD_CHID field (bits [7:4]).
 *  2. Sets the CNVST bit (bit 3) in GENERAL_CFG to start conversion.
 *  3. Waits 10 µs for the conversion to complete (blocking).
 *  4. Reads RECENT_CHn_LSB and RECENT_CHn_MSB and assembles the 12-bit result.
 *
 * Note: The fixed 10 µs delay may not be sufficient at high OSR settings.
 * Polling the OSR_DONE flag in SYSTEM_STATUS is a more robust alternative.
 *
 * @param channel Channel to convert (0–7, corresponding to AIN0–AIN7).
 * @return uint16_t Raw 12-bit conversion result (0–4095).
 *                  Returns 0 if channel > 7.
 */
uint16_t ADS7128::readChannelDataManual(uint8_t channel) {
    if (channel > 7) return 0;

    // Set MANUAL_CHID (bits 3:0) and preserve ZCD_CHID (bits 7:4)
    uint8_t chSel = readRegister(ADS7128_REG_CHANNEL_SEL);
    chSel = (chSel & 0xF0) | (channel & 0x0F);
    writeRegister(ADS7128_REG_CHANNEL_SEL, chSel);

    // Start conversion via CNVST bit in GENERAL_CFG
    uint8_t reg = readRegister(ADS7128_REG_GENERAL_CFG);
    reg |= (1 << 3);          // write 1 to start
    writeRegister(ADS7128_REG_GENERAL_CFG, reg);

    // Blocking delay, can be replaced by polling conversion bit
    delayMicroseconds(10);
    
    // Read LSB and MSB
    uint8_t lsbReg = ADS7128_REG_RECENT_CH0_LSB + (channel * 2);
    uint8_t msbReg = ADS7128_REG_RECENT_CH0_MSB + (channel * 2);
    
    uint8_t lsb = readRegister(lsbReg);
    uint8_t msb = readRegister(msbReg);
    
    // Combine into 12-bit value (MSB contains upper 4 bits)
    uint16_t value = (msb << 4) | lsb;
    
    return value;
}

/**
 * @brief Reads the most recently completed autonomous conversion result for
 *        a channel. Used internally by analogRead() in autonomous mode.
 *
 * In autonomous mode the ADS7128 continuously scans all enabled channels
 * and stores each result in a pair of read-only RECENT registers:
 *   RECENT_CHn_LSB (0xA0 + 2n) : bits [7:0] of the 12-bit result.
 *   RECENT_CHn_MSB (0xA1 + 2n) : bits [3:0] contain bits [11:8] of the result.
 *
 * This function simply reads those two registers and assembles the value.
 * No conversion is triggered; the result reflects the last completed scan.
 *
 * The channel must be included in the bitmask passed to enableAutonomousMode().
 *
 * @param channel Channel to read (0–7, corresponding to AIN0–AIN7).
 * @return uint16_t Most recent 12-bit conversion result (0–4095).
 *                  Returns 0 if channel > 7.
 */
uint16_t ADS7128::readChannelDataAutonomous(uint8_t channel) {
    if (channel > 7) return 0;

    // Read LSB and MSB
    uint8_t lsbReg = ADS7128_REG_RECENT_CH0_LSB + (channel * 2);
    uint8_t msbReg = ADS7128_REG_RECENT_CH0_MSB + (channel * 2);
    
    uint8_t lsb = readRegister(lsbReg);
    uint8_t msb = readRegister(msbReg);
    
    // Combine into 12-bit value (MSB contains upper 4 bits)
    uint16_t value = (msb << 4) | lsb;
    
    return value;
}

/**
 * @brief Writes a single byte to an ADS7128 register over I2C.
 *
 * Sends a three-byte I2C transaction:
 *   [OPCODE_SINGLE_WRITE (0x08)] [register address] [data byte]
 *
 * @param reg   8-bit register address to write.
 * @param value 8-bit value to write into the register.
 * @return true  I2C transmission acknowledged successfully.
 * @return false I2C transmission failed (NACK or bus error).
 */
bool ADS7128::writeRegister(uint8_t reg, uint8_t value) {
    _i2cPort->beginTransmission(_i2cAddr);
    _i2cPort->write(OPCODE_SINGLE_WRITE);
    _i2cPort->write(reg);
    _i2cPort->write(value);
    return (_i2cPort->endTransmission(true) == 0);
}

/**
 * @brief Reads a single byte from an ADS7128 register over I2C.
 *
 * Sends a two-byte write (opcode + register address) with a repeated
 * START, then reads one byte back:
 *   Write: [OPCODE_SINGLE_READ (0x10)] [register address]
 *   Read:  [data byte]
 *
 * Returns 0 on I2C failure; note that 0 is also a valid register value,
 * so this return code is ambiguous. Check isDeviceReady() for error state.
 *
 * @param reg 8-bit register address to read.
 * @return uint8_t Value read from the register, or 0 on I2C failure.
 */
uint8_t ADS7128::readRegister(uint8_t reg) {
    _i2cPort->beginTransmission(_i2cAddr);
    _i2cPort->write(OPCODE_SINGLE_READ);
    _i2cPort->write(reg);
    _i2cPort->endTransmission(false);
    
    _i2cPort->requestFrom(_i2cAddr, (uint8_t)1);
    
    if (_i2cPort->available()) {
        return _i2cPort->read();
    }
    
    return 0;
}

/**
 * @brief Converts a voltage in volts to a 12-bit ADC code.
 *
 * Uses the formula:  code = round( V × 4096 / Vref )
 * Result is clamped to the valid 12-bit range [0, 4095].
 *
 * Used internally by setChannelWindow() to convert user-supplied
 * threshold voltages into register values.
 *
 * @param V    Input voltage in volts. Values below 0 V are clamped to 0.
 * @param Vref Full-scale reference voltage in volts (must be > 0).
 * @return uint16_t Corresponding 12-bit ADC code (0–4095).
 */
uint16_t ADS7128::voltageToCode(float V, float Vref) {
    float code_f = (V * 4096.0f) / Vref;
    if (code_f < 0.0f)      code_f = 0.0f;
    if (code_f > 4095.0f)   code_f = 4095.0f;
    return (uint16_t)roundf(code_f);
}
