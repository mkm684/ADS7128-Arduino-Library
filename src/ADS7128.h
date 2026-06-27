#ifndef ADS7128_H
#define ADS7128_H

#include <Arduino.h>
#include <Wire.h>

// ADS7128 Register Addresses
#define ADS7128_REG_SYSTEM_STATUS       0x00
#define ADS7128_REG_GENERAL_CFG         0x01
#define ADS7128_REG_DATA_CFG            0x02
#define ADS7128_REG_OSR_CFG             0x03
#define ADS7128_REG_OPMODE_CFG          0x04
#define ADS7128_REG_PIN_CFG             0x05
#define ADS7128_REG_GPIO_CFG            0x07
#define ADS7128_REG_GPO_DRIVE_CFG       0x09
#define ADS7128_REG_GPO_VALUE           0x0B
#define ADS7128_REG_GPI_VALUE           0x0D
#define ADS7128_REG_SEQUENCE_CFG        0x10
#define ADS7128_REG_CHANNEL_SEL         0x11
#define ADS7128_REG_AUTO_SEQ_CH_SEL     0x12
#define ADS7128_REG_ALERT_CH_SEL        0x14
// Window comparator registers
#define ADS7128_REG_HYSTERESIS_CH0      0x20
#define ADS7128_REG_HIGH_TH_CH0         0x21
#define ADS7128_REG_EVENT_COUNT_CH0     0x22
#define ADS7128_REG_LOW_TH_CH0          0x23
// Alert registers
#define ADS7128_REG_ALERT_PIN_CFG       0x17
#define ADS7128_REG_EVENT_RGN           0x1E
#define ADS7128_REG_GPO0_TRIG_EVENT_SEL 0xC3
#define ADS7128_REG_GPO_TRIGGER_CFG     0xE9
#define ADS7128_REG_GPO_VALUE_ON_TRIGGER 0xEB

// Channel data registers (read-only)
#define ADS7128_REG_RECENT_CH0_LSB      0xA0
#define ADS7128_REG_RECENT_CH0_MSB      0xA1
#define ADS7128_REG_RECENT_CH1_LSB      0xA2
#define ADS7128_REG_RECENT_CH1_MSB      0xA3
#define ADS7128_REG_RECENT_CH2_LSB      0xA4
#define ADS7128_REG_RECENT_CH2_MSB      0xA5
#define ADS7128_REG_RECENT_CH3_LSB      0xA6
#define ADS7128_REG_RECENT_CH3_MSB      0xA7
#define ADS7128_REG_RECENT_CH4_LSB      0xA8
#define ADS7128_REG_RECENT_CH4_MSB      0xA9
#define ADS7128_REG_RECENT_CH5_LSB      0xAA
#define ADS7128_REG_RECENT_CH5_MSB      0xAB
#define ADS7128_REG_RECENT_CH6_LSB      0xAC
#define ADS7128_REG_RECENT_CH6_MSB      0xAD
#define ADS7128_REG_RECENT_CH7_LSB      0xAE
#define ADS7128_REG_RECENT_CH7_MSB      0xAF

// Read Write OP CODES
#define OPCODE_SINGLE_READ   0x10
#define OPCODE_SINGLE_WRITE  0x08 
#define OPCODE_SET_BIT       0x18
#define OPCODE_CLEAR_BIT     0x20
#define OPCODE_BLOCK_READ    0x30
#define OPCODE_BLOCK_WRITE   0x28

// SYSTEM_STATUS register bit definitions
#define ADS7128_STATUS_SEQ_STATUS       (1 << 4)  // Sequence status
#define ADS7128_STATUS_I2C_SPEED        (1 << 3)  // I2C speed indicator
#define ADS7128_STATUS_OSR_DONE         (1 << 2)  // OSR operation complete
#define ADS7128_STATUS_CRC_ERR          (1 << 1)  // CRC error flag
#define ADS7128_STATUS_FL_POR           (1 << 0)  // Power-on reset flag

// Default register values after reset (from datasheet)
#define ADS7128_DEFAULT_SYSTEM_STATUS   0x81
#define ADS7128_DEFAULT_GENERAL_CFG     0x00
#define ADS7128_DEFAULT_DATA_CFG        0x00
#define ADS7128_DEFAULT_OSR_CFG         0x00
#define ADS7128_DEFAULT_OPMODE_CFG      0x00
#define ADS7128_DEFAULT_PIN_CFG         0x00
#define ADS7128_DEFAULT_GPIO_CFG        0x00
#define ADS7128_DEFAULT_GPO_DRIVE_CFG   0x00
#define ADS7128_DEFAULT_SEQUENCE_CFG    0x00

// Timing constants
#define ADS7128_RESET_DELAY_MS          10    // Minimum delay after reset

// User variable for debug serial outputs
#ifndef ADS7128_DEBUG
  #define ADS7128_DEBUG 0
#endif

// Pin modes
enum ADS7128PinMode {
    PIN_MODE_ANALOG_INPUT = 0,
    PIN_MODE_DIGITAL_INPUT = 1,
    PIN_MODE_DIGITAL_OUTPUT = 2,
    PIN_MODE_INVALID = -1
};

// Drive modes for output pins
enum DriveMode {
    DRIVE_MODE_OPEN_DRAIN = 0,
    DRIVE_MODE_PUSH_PULL = 1
};

// System status structure
struct SystemStatus {
    bool sequenceActive;      // Sequence is currently active
    bool highSpeedI2C;        // I2C is operating in high-speed mode
    bool osrDone;             // OSR operation completed
    bool crcError;            // CRC error detected
    bool powerOnReset;        // Power-on reset occurred
    uint8_t rawValue;         // Raw register value
};

struct SystemState {
    bool autoMode;      // True if automode enabled
};

class ADS7128 {
public:
    ADS7128(uint8_t i2cAddr = 0x10);
    
    bool begin(TwoWire &wirePort = Wire);
    
    // Reset, configuration, and status
    bool softwareReset();
    bool verifyDefaultConfiguration();
    SystemStatus getSystemStatus();
    bool isDeviceReady();
    bool clearPowerOnResetFlag();
    
    // Pin configuration
    bool setPinMode(uint8_t pin, ADS7128PinMode mode);
    ADS7128PinMode getPinMode(uint8_t pin);
    
    // Drive mode configuration (for output pins)
    bool setDriveMode(uint8_t pin, DriveMode mode);
    DriveMode getDriveMode(uint8_t pin);
    
    // Digital output control
    bool digitalWrite(uint8_t pin, bool value);
    bool digitalRead(uint8_t pin);
    
    // Analog reading
    uint16_t analogRead(uint8_t channel);
    float analogReadVoltage(uint8_t channel, float vref = 3.3);
    
    // Register access
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

    // Function selection
    bool enableManualMode();
    bool enableAutonomousMode(uint8_t channels);
    bool enableWindowMode(bool state);

    // Comparitor window settings
    bool setChannelWindow(uint8_t channel, float windowMax, float windowMin, uint8_t eventCount, float Vref = 3.3);
    bool setChannelHysteresis(uint8_t channel, float hysteresis_v, float Vref = 3.3);
    bool setWindowRegion(uint8_t regions);

    // Trigger settings
    bool setAlertLogic(bool drive, uint8_t logic);
    bool setAlertChannels(uint8_t channels);
    bool setTriggerOn(uint8_t pins);
    bool setTriggerPins(uint8_t pin, uint8_t alerts);
    bool setValueOnTrigger(uint8_t pin, bool state);
    
private:
    uint8_t _i2cAddr;
    TwoWire *_i2cPort;
    
    uint16_t readChannelDataManual(uint8_t channel);
    uint16_t readChannelDataAutonomous(uint8_t channel);
    uint16_t voltageToCode(float V, float Vref);

    SystemState systemState; 
};

#endif