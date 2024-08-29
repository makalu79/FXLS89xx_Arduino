/*
 * Copyright 2024 ryraki
 * All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */
/**
 * @file FXLS89xx_Arduino.cpp
 * @brief This file contains the functions for FXLS89xx Accelerometer evaluation by Arduino
 */
 
#include "FXLS89xx_Arduino.h"

FXLS89xx::FXLS89xx(uint8_t i2c_address) : I2C_device(i2c_address) {}
FXLS89xx::FXLS89xx(TwoWire& wire, uint8_t i2c_address) : I2C_device(wire, i2c_address) {}
FXLS89xx::~FXLS89xx() {}

void FXLS89xx::run(bool sdcd) {
  bit_op8(_SENS_CONFIG1, ~SENS_CONFIG1::ACTIVE_MASK, 0);
  bit_op8(_SENS_CONFIG1, ~SENS_CONFIG1::FSR_MASK, sensor_range<<SENS_CONFIG1::FSR_SHIFT);
  bit_op8(_SENS_CONFIG2, ~(SENS_CONFIG2::SLEEP_PM_MASK|SENS_CONFIG2::WAKE_PM_MASK),
    (wake_pm<<SENS_CONFIG2::WAKE_PM_SHIFT) | (sleep_pm<<SENS_CONFIG2::SLEEP_PM_SHIFT));
  write_r8(_SENS_CONFIG3, (wake_odr<<SENS_CONFIG3::WAKE_ODR_SHIFT) | sleep_odr);
  if (!sdcd)
    bit_op8(_INT_EN, ~INT_EN::DRDY_EN_EN, INT_EN::DRDY_EN_EN);
  bit_op8(_SENS_CONFIG1, ~SENS_CONFIG1::ACTIVE_ACT, SENS_CONFIG1::ACTIVE_ACT);
}

void FXLS89xx::enable_sleep(uint16_t asleep) {
  uint8_t aslp_reg[2] = {asleep&0xff, asleep>>8};
  reg_w(_ASLP_COUNT_LSB, aslp_reg, 2);

  bit_op8(_SENS_CONFIG4, ~(SENS_CONFIG4::WK_SDCD_OT_EN | SENS_CONFIG4::WK_SDCD_WT_EN),
    sdcd_wt ? SENS_CONFIG4::WK_SDCD_WT_EN : SENS_CONFIG4::WK_SDCD_OT_EN);
}

void FXLS89xx::read_XYZ(float *pOutBuffer) {
  uint8_t output[6];
  reg_r(_OUT_X_LSB, output, 6);
  for (int i=0; i<3; i++) {
    if (output[2*i+1]&0xf8)
      pOutBuffer[i] = ((float)((((uint8_t)~output[2*i+1])&0x0f)<<8)+((uint8_t)~output[2*i])+1)*(-0.98)*(sensor_range+1);
    else pOutBuffer[i] = ((float)(((uint8_t)output[2*i+1]&0x0f)<<8)+(uint8_t)output[2*i])*0.98*(sensor_range+1);
  }
}

uint8_t FXLS89xx::init() {
  write_r8(_SENS_CONFIG1, SENS_CONFIG1::RST_RESET);
  return read_r8(_WHO_AM_I);
}
