// driver for ADS1110 i2c 16-bit analog to digital converter
// --github.com/epvuc 12/2017

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"

#define SDA_PIN 19
#define SCL_PIN 18
#define DEV_ADDR 0x4d
#define ACK_VAL 0
#define NACK_VAL 1

static char tag[] = "adc";

void ads1110_writecfg(uint8_t val)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (DEV_ADDR<<1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd, val, 1);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}

uint8_t ads1110_readcfg(void)
{
  uint8_t val1, val2, val3;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (DEV_ADDR<<1) | I2C_MASTER_READ, 1);
  i2c_master_read_byte(cmd, &val1,  ACK_VAL);
  i2c_master_read_byte(cmd, &val2,  ACK_VAL);
  i2c_master_read_byte(cmd, &val3,  NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return(val3);
}

int16_t ads1110_read(void)
{
  uint8_t hbyte, lbyte;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (DEV_ADDR<<1) | I2C_MASTER_READ, 1);
  i2c_master_read_byte(cmd, &hbyte,  ACK_VAL);
  i2c_master_read_byte(cmd, &lbyte, NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return((hbyte<<8) | lbyte);
}

