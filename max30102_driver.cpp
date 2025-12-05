#include "max30102_driver.h"

MAX30102_Driver::MAX30102_Driver() {
    _i2c = nullptr;
    _fifo_write_ptr = 0;
    _fifo_read_ptr = 0;
    _fifo_reads = 0;
    _total_samples_read = 0;
}

bool MAX30102_Driver::begin(TwoWire& wirePort) {
    _i2c = &wirePort;

    // Ensure bus speed
    _i2c->setClock(400000);

    Serial.println("MAX30102: starting init...");

    // Reset the device and let it stabilize
    reset();
    delay(500);

    // Verify Part ID
    uint8_t pid = readRegister(REG_PART_ID);
    Serial.print("MAX30102 Part ID: 0x");
    Serial.println(pid, HEX);
    if (pid != 0x15) {
        Serial.println("MAX30102: Part ID mismatch or wrong chip!");
        // We'll still try to configure but signal failure
    }

    // Clear interrupts
    readRegister(REG_INTR_STATUS_1);
    readRegister(REG_INTR_STATUS_2);

    // Configure sensor using datasheet-based/distro driver sequence.
    configureSensor();

    // Optionally finalize: read back settings for debug
    Serial.print("MODE: 0x");
    Serial.println(readRegister(REG_MODE_CONFIG), HEX);
    Serial.print("FIFO_CFG: 0x");
    Serial.println(readRegister(REG_FIFO_CONFIG), HEX);
    Serial.print("SPO2_CFG: 0x");
    Serial.println(readRegister(REG_SPO2_CONFIG), HEX);
    Serial.print("LED1: 0x");
    Serial.println(readRegister(REG_LED1_PA), HEX);
    Serial.print("LED2: 0x");
    Serial.println(readRegister(REG_LED2_PA), HEX);
    Serial.print("INTR1: 0x");
    Serial.println(readRegister(REG_INTR_ENABLE_1), HEX);

    return (pid == 0x15);
}

void MAX30102_Driver::reset() {
    // Set reset bit
    writeRegister(REG_MODE_CONFIG, MODE_RESET);
    delay(250);
    // Wait for reset bit to clear
    uint8_t mode = readRegister(REG_MODE_CONFIG);
    unsigned long t0 = millis();
    while (mode & MODE_RESET) {
        if (millis() - t0 > 1000) break;
        delay(50);
        mode = readRegister(REG_MODE_CONFIG);
    }
}

void MAX30102_Driver::configureSensor() {
    // 1) Shutdown (must be in shutdown to change LED currents per datasheet)
    writeRegister(REG_MODE_CONFIG, MODE_SHUTDOWN);
    delay(50);

    // 2) Set LED pulse amplitudes (LED1=RED, LED2=IR)
    writeRegister(REG_LED1_PA, LED_PA_DEFAULT);
    delay(10);
    writeRegister(REG_LED2_PA, LED_PA_DEFAULT);
    delay(20);
    // pilot amplitude (optional)
    writeRegister(REG_PILOT_PA, 0x7F);
    delay(20);

    // 3) FIFO config - sample averaging & threshold
    writeRegister(REG_FIFO_CONFIG, FIFO_CONFIG_DEFAULT);
    delay(10);

    // 4) SPO2 config - from linux kernel/datasheet (400Hz, 18-bit, 411us)
    writeRegister(REG_SPO2_CONFIG, SPO2_CONFIG_400HZ_18B_411US);
    delay(10);

    // 5) Clear and initialize FIFO pointers
    writeRegister(REG_FIFO_WR_PTR, 0);
    writeRegister(REG_OVF_COUNTER, 0);
    writeRegister(REG_FIFO_RD_PTR, 0);
    delay(50);

    // 6) Interrupt config: enable FIFO almost full and/or PPG_RDY
    // Kernel driver enables FIFO RDY. Optionally enable PPG_RDY if you want frequent updates.
    writeRegister(REG_INTR_ENABLE_1, 0x80); // 0x80=AFULL; 0x40=PPG_RDY. Use 0xC0 for both.
    writeRegister(REG_INTR_ENABLE_2, 0x00);
    delay(10);

    // 7) Exit shutdown and set SpO2 mode (active)
    writeRegister(REG_MODE_CONFIG, MODE_SPO2);
    delay(200); // give driver time to start collecting
}

void MAX30102_Driver::writeRegister(uint8_t reg, uint8_t value) {
    if (!_i2c) return;
    _i2c->beginTransmission(MAX30102_ADDR);
    _i2c->write(reg);
    _i2c->write(value);
    uint8_t err = _i2c->endTransmission();
    if (err != 0) {
        Serial.print("I2C write error (reg 0x");
        Serial.print(reg, HEX);
        Serial.print(") : ");
        Serial.println(err);
    }
    delayMicroseconds(100);
}

uint8_t MAX30102_Driver::readRegister(uint8_t reg) {
    if (!_i2c) return 0;
    _i2c->beginTransmission(MAX30102_ADDR);
    _i2c->write(reg);
    _i2c->endTransmission(false); // repeated start
    uint8_t n = _i2c->requestFrom(MAX30102_ADDR, (uint8_t)1);
    if (n == 0) return 0;
    return _i2c->read();
}

uint8_t MAX30102_Driver::readRegisters(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (!_i2c) return 0;
    _i2c->beginTransmission(MAX30102_ADDR);
    _i2c->write(reg);
    _i2c->endTransmission(false);
    uint8_t n = _i2c->requestFrom(MAX30102_ADDR, len);
    for (uint8_t i = 0; i < n && i < len; ++i) {
        buf[i] = _i2c->read();
    }
    return n;
}

uint8_t MAX30102_Driver::getPartID() {
    return readRegister(REG_PART_ID);
}

bool MAX30102_Driver::isDataAvailable() {
    _fifo_write_ptr = readRegister(REG_FIFO_WR_PTR);
    _fifo_read_ptr = readRegister(REG_FIFO_RD_PTR);
    int cnt = _fifo_write_ptr - _fifo_read_ptr;
    if (cnt < 0) cnt += 32;
    return cnt > 0;
}

uint8_t MAX30102_Driver::readFIFO(ppg_data_t data_buffer[], uint8_t max_samples) {
    /* read INT status to clear flags (kernel driver does this in its interrupt path) */
    uint8_t intr1 = readRegister(REG_INTR_STATUS_1);
    uint8_t intr2 = readRegister(REG_INTR_STATUS_2);

    _fifo_write_ptr = readRegister(REG_FIFO_WR_PTR);
    _fifo_read_ptr = readRegister(REG_FIFO_RD_PTR);

    int samples_available = _fifo_write_ptr - _fifo_read_ptr;
    if (samples_available < 0) samples_available += 32;
    if (samples_available == 0) return 0;

    uint8_t to_read = (uint8_t)min((int)max_samples, samples_available);
    uint8_t total_bytes = to_read * 6; // 3 bytes RED + 3 bytes IR

    // Set register pointer & request bytes
    _i2c->beginTransmission(MAX30102_ADDR);
    _i2c->write(REG_FIFO_DATA);
    _i2c->endTransmission(false); // repeated start
    uint8_t n = _i2c->requestFrom(MAX30102_ADDR, total_bytes);
    if (n == 0) return 0;

    uint8_t samples_read = 0;
    for (uint8_t i = 0; i < to_read; ++i) {
        if (_i2c->available() >= 6) {
            uint8_t b0 = _i2c->read();
            uint8_t b1 = _i2c->read();
            uint8_t b2 = _i2c->read();
            uint8_t b3 = _i2c->read();
            uint8_t b4 = _i2c->read();
            uint8_t b5 = _i2c->read();

            uint32_t red = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
            uint32_t ir  = ((uint32_t)b3 << 16) | ((uint32_t)b4 << 8) | b5;
            red &= 0x03FFFF; // 18-bit
            ir  &= 0x03FFFF;

            data_buffer[samples_read].red = red;
            data_buffer[samples_read].ir  = ir;
            ++samples_read;
            ++_total_samples_read;
        } else {
            break;
        }
    }

    ++_fifo_reads;
    return samples_read;
}

bool MAX30102_Driver::readSample(ppg_data_t *sample) {
    if (!isDataAvailable()) return false;
    ppg_data_t buf[4];
    uint8_t r = readFIFO(buf, 1);
    if (r == 1) {
        *sample = buf[0];
        return true;
    }
    return false;
}

void MAX30102_Driver::printDiagnostics() {
    Serial.println("\n===== MAX30102 Diagnostics =====");
    Serial.print("FIFO reads : "); Serial.println(_fifo_reads);
    Serial.print("Total samples: "); Serial.println(_total_samples_read);
    Serial.print("WR_PTR: 0x"); Serial.println(readRegister(REG_FIFO_WR_PTR), HEX);
    Serial.print("RD_PTR: 0x"); Serial.println(readRegister(REG_FIFO_RD_PTR), HEX);
    Serial.print("INTR1: 0x"); Serial.println(readRegister(REG_INTR_STATUS_1), HEX);
    Serial.print("INTR_EN1: 0x"); Serial.println(readRegister(REG_INTR_ENABLE_1), HEX);
    Serial.print("MODE: 0x"); Serial.println(readRegister(REG_MODE_CONFIG), HEX);
    Serial.println("================================\n");
}