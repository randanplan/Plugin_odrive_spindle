#ifndef PCA9685_H
#define PCA9685_H

#include "driver.h"

void pca9685_pwm_set(uint8_t chn, uint16_t val);
void pca9685_pwm_set_duty(uint8_t chn, uint16_t duty);
bool pca9685_init (void);

#endif /* PCA9685_H */
