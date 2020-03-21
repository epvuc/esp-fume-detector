#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <esp_log.h>
#include "bme280.h"

extern QueueHandle_t xUdpSendQueue;

void my_i2c_setup(void);
void my_bme280_init(void);
void ads1110_writecfg(uint8_t val);
uint8_t ads1110_readcfg(void);
int16_t ads1110_read(void);

extern struct bme280_dev bme280; // from bme280_sup.c
extern QueueHandle_t xUdpSendQueue; // from main.c

void measurement_task(void *p)
{
  int8_t rslt;
  uint32_t count = 0;
  int32_t val;
  //  double volts;
  char msgbuf[128];
  struct bme280_data comp_data;

  // config the actual i2c bus
  my_i2c_setup();
  // assign my hw functions to the bme280 driver & call it's init function
  // & set its sampling configuration
  my_bme280_init();
  
  while(1) { 
    // take BME280 readings. 
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
    bme280.delay_ms(40);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280);
    if (rslt != BME280_OK)
      ESP_LOGI("BME280", "bme280_get_sensor_data() returned %d", rslt);

    // take gas sensor readings.
    ads1110_writecfg(0b10011100); // single conversion, start conv, 15sps, gain 1
    // wait for conversion to finish
    while(ads1110_readcfg() & (1<<7))
      vTaskDelay(1);
    val=ads1110_read();
    //    volts=(double)val/6537.545;

    snprintf(msgbuf, sizeof(msgbuf), "Fumes: f:%d t:%.02f p:%0.4f h:%0.1f #%u\r\n",
	     val,
	     (comp_data.temperature * 9/5)+32,
	     comp_data.pressure / 3386.3886,
	     comp_data.humidity,
	     count++);

    if (count >= 9999)
      count = 0;

    ESP_LOGI("FUME", "%s", msgbuf);
    xQueueSend(xUdpSendQueue, &msgbuf, ( TickType_t ) 0);
    vTaskDelay(800/portTICK_PERIOD_MS);
  }
}
