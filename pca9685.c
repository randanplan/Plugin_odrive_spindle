
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//Register addresses
#define PCA9685_REG_MODE1           (0x00) // Mode1 register
#define PCA9685_REG_MODE2           (0x01) // Mode2 register
#define PCA9685_REG_SUBADR1         (0x02) // I2C bus subaddress 1 register
#define PCA9685_REG_SUBADR2         (0x03) // I2C bus subaddress 2 register
#define PCA9685_REG_SUBADR3         (0x04) // I2C bus subaddress 3 register
#define PCA9685_REG_ALLCALLADDR     (0x05) // LED All Call I 2 C-bus address
#define PCA9685_REG_LED0_ON_L       (0x06) // LED0 ON control low byte
#define PCA9685_REG_ALL_LED_ON      (0xfa) // Load all LEDn_OFF register word
#define PCA9685_REG_ALL_LED_OFF     (0xfc) // Load all LEDn_OFF register word
#define PCA9685_REG_PRE_SCALE       (0xfe) // Prescaler for PWM output frequency
#define PCA9685_REG_TEST_MODE       (0xff) // Enter test mode register
#define PCA9685_REG_LED_ON(n)       (0x06 + (n << 2))   // LEDn ON control word
#define PCA9685_REG_LED_OFF(n)      (0x08 + (n << 2))   // LEDn OFF control word
// PCA9685_REG_MODE1 
#define PCA9685_MODE1_RESTART   (0x80)  // State of restart logic, write 1 to clear
#define PCA9685_MODE1_EXTCLK    (0x40)  // Use EXTCLK pin
#define PCA9685_MODE1_AI        (0x20)  // Enable register auto-increment*/
#define PCA9685_MODE1_SLEEP     (0x10)  // Enter low power mode, PWM is off
#define PCA9685_MODE1_SUB1      (0x08)  // Enable I2C subaddress 1
#define PCA9685_MODE1_SUB2      (0x04)  // Enable I2C subaddress 2
#define PCA9685_MODE1_SUB3      (0x02)  // Enable I2C subaddress 3
#define PCA9685_MODE1_ALLCALL   (0x01)  // Enable I2C all call address
// PCA9685_REG_MODE2
#define PCA9685_MODE2_INVERT    (0x10)  // Invert outputs
#define PCA9685_MODE2_OCH       (0x08)  // Output change change configuration
#define PCA9685_MODE2_OUTDRV    (0x04)  // Output driver configuration
#define PCA9685_MODE2_OUTNE     (0x03)  // Output enabled configuration

#define PCA9685_I2C_BASE_ADDR   (0x40)      // I2C slave base address
#define PCA9685_RESOLUTION      (1 << 12)
#define PCA9685_OSC_FREQ        (25000000)


#include "pca9685.h"
#include "i2c.h"

typedef struct {
    uint8_t i2c_addr;       // I2C slave address
    uint32_t freq;          // PWM frequency in Hz (default 100)
    uint16_t res;           // PWM resolution (default 4096)
    uint32_t ext_freq;      // If not 0, EXTCLK pin is used with this frequency
    bool initOK;
} pca9685_t;

static pca9685_t pwm_dev = {
    .i2c_addr = PCA9685_I2C_BASE_ADDR,
    .res = 4096,
    .freq = 100000,
    .initOK = false
};

void setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    if (!pwm_dev.initOK)
        return;
    uint8_t bytes[5] = {PCA9685_REG_LED_ON(num), on ,on >> 8, off, off >> 8};
    I2C_Send(pwm_dev.i2c_addr, bytes, 5, true);
}

void pca9685_pwm_set(uint8_t chn, uint16_t val)
{
    val = min(val, (uint16_t)4095);
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(chn, 4096, 0);
    } else if (val == 0) {
      // Special value for signal fully off.
      setPWM(chn, 0, 4096);
    } else {
        setPWM(chn, 0, val);
    }
}

void pca9685_pwm_set_duty(uint8_t chn, uint16_t duty) {
    // Clamp value between 0 and 4095 inclusive.
    if (duty == 0) 
      setPWM(chn, 0, 4096);
    else if (duty >= 100)
        setPWM(chn, 4096, 0);
    else {
        uint16_t dc_val = (uint16_t)(40.95f * duty);
        setPWM(chn, 0, dc_val);
    }
}

void pca9685_pwm_all_off(){
    // if (!I2C_Send(pwm_dev.i2c_addr, NULL, 1, true))
        // return;
    I2C_ReadRegister(pwm_dev.i2c_addr, PCA9685_REG_MODE1, 1, 1, true);
    uint8_t data[] = {PCA9685_REG_ALL_LED_OFF, 0 & 0xFF, (0>>8) & 0xFF};
    I2C_Send(pwm_dev.i2c_addr, data, 3, true);
    hal.delay_ms(5,NULL);
}

bool pca9685_init(void)
{
    if (pwm_dev.initOK){
        pca9685_pwm_all_off();
        return true;
    }
    i2c_init();
    pca9685_pwm_all_off();
    uint8_t bytes[3] = {PCA9685_REG_MODE1, 0,0};
    I2C_ReadRegister(pwm_dev.i2c_addr, bytes, 1, 1, true);
    bytes[0] = PCA9685_REG_MODE1;
    bytes[1] = PCA9685_MODE1_RESTART;
    I2C_Send(pwm_dev.i2c_addr, bytes, 2, true);
    hal.delay_ms(5,NULL);

    I2C_ReadRegister(pwm_dev.i2c_addr, bytes, 1, 1, true);
    uint8_t oldmode = bytes[1];
    uint8_t newmode = (oldmode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP; // sleep
    bytes[1] = newmode;
    I2C_Send(pwm_dev.i2c_addr, bytes, 2, true);
    hal.delay_ms(2,NULL);
    uint32_t div = PCA9685_RESOLUTION * pwm_dev.freq;
    bytes[0] = PCA9685_REG_PRE_SCALE;
    bytes[1] = ((pwm_dev.ext_freq ? pwm_dev.ext_freq : PCA9685_OSC_FREQ) + div/2) / div - 1;
    I2C_Send(pwm_dev.i2c_addr, bytes, 2, true);

    bytes[0] = PCA9685_REG_MODE1;
    bytes[1] = oldmode;
    I2C_Send(pwm_dev.i2c_addr, bytes, 2, true);

    bytes[0] = PCA9685_REG_MODE1;
    bytes[1] = oldmode | PCA9685_MODE1_RESTART | PCA9685_MODE1_AI;
    I2C_Send(pwm_dev.i2c_addr, bytes, 2, true);
    hal.delay_ms(5, NULL);

    pwm_dev.initOK = true;
    // pca9685_pwm_set(0,800);
    return true;
}

