#ifndef MAX30102_DRIVER_H
#define MAX30102_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

/* MAX30102 7-bit I2C address */
#define MAX30102_ADDR      0x57

/* Registers (datasheet / kernel) */
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PILOT_PA        0x10
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF

/* Useful values (per datasheet/kernel) */
#define MODE_SHUTDOWN       0x80
#define MODE_RESET          0x40
#define MODE_SPO2           0x03

/* FIFO config: avg=4, rollover off, almost-full=17 (kernel MAXIM value) */
#define FIFO_CONFIG_DEFAULT 0x4F

/* SPO2 config kernel value: 400Hz, 18-bit, 411us PW */
#define SPO2_CONFIG_400HZ_18B_411US 0x2F

/* LED pulse amplitude defaults (~7mA) */
#define LED_PA_DEFAULT    0x24

typedef struct {
    uint32_t red;
    uint32_t ir;
} ppg_data_t;

class MAX30102_Driver {
public:
    MAX30102_Driver();

    // Initialize driver with a TwoWire port
    bool begin(TwoWire& wirePort);

    // Read FIFO data (up to max_samples)
    // returns samples read
    uint8_t readFIFO(ppg_data_t data_buffer[], uint8_t max_samples);

    // Check if any samples are available (splits pointers)
    bool isDataAvailable();

    // Single-sample read
    bool readSample(ppg_data_t *sample);

    // Basic register utilities
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegisters(uint8_t reg, uint8_t *buf, uint8_t len);

    // Part ID
    uint8_t getPartID();

    // Diagnostics
    void printDiagnostics();

private:
    TwoWire* _i2c;
    uint8_t _fifo_write_ptr;
    uint8_t _fifo_read_ptr;

    // stats for diagnostics
    uint32_t _fifo_reads;
    uint32_t _total_samples_read;

    // internal helpers
    void reset();
    void configureSensor(); // perform full config from datasheet sequence
};

#endif // MAX30102_DRIVER_H