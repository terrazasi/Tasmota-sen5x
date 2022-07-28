/*
  xsns_101_sen5x - Sensirion 5X series environmental sensor support for Tasmota.
  Based on xsns_44_sps30.ino and xsns_91_vindriktning.ino files.

  Copyright (C) 2021  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
 

#ifdef USE_I2C
#ifdef USE_SEN5X


#define XSNS_101  101
#define XI2C_69   69  // See I2CDEVICES.md

#define SEN5X_ADDR 0x69 // I2C Address from Datasheet

#include <SensirionI2CSen5x.h>
#include <SensirionCore.h>

#include <Wire.h>
#include <math.h>

#ifdef ESP8266

#include <twi.h>
#endif

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

SensirionI2CSen5x sen5x;

// Create measurement struct
struct SEN5X_VAL {
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;

    uint8_t valid = 0;
    uint8_t count = 0;
    char    name[6] = "SEN5X";
} sen5x_val;


// I2C Commands from Datasheet (https://sensirion.com/de/resource/datasheet/sen5x)
#define SEN_CMD_START_MSRMT 0x0021
#define SEN_CMD_START_MSRMT_ARG 0x0300
#define SEN_CMD_START_MSRMT_GAS_ONLY 0x0037
#define SEN_CMD_STOP_MSRMT 0x0104
#define SEN_CMD_DATA_READY 0x0202
#define SEN_CMD_READ_MSRMT 0x03C4
#define SEN_CMD_RW_TEMP_COMPENSATION 0X60B2       // "RW" = Read/Write
#define SEN_CMD_RW_WARM_START 0X60C2
#define SEN_CMD_RW_VOC_TUNING 0X60D0
#define SEN_CMD_RW_RHT_ACCELERATION 0X60F7
#define SEN_CMD_RW_VOC_STATE 0X6181
#define SEN_CMD_CLEAN 0x5607
#define SEN_CMD_RW_AUTOCLEAN_INTERVAL 0x8004
#define SEN_CMD_NAME 0xD014
#define SEN_CMD_GET_SERIAL 0xD033
#define SEN_CMD_READ_FIRMWARE 0xD100
#define SEN_CMD_READ_STATUS 0xD206
#define SEN_CMD_CLEAR_STATUS 0xD210
#define SEN_CMD_RESET 0xD304


// From the datasheet, but not sure where to implement
uint8_t sen5x_calc_CRC(uint8_t *data) {
    uint8_t crc = 0xFF;
    for (uint32_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (uint32_t bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/********************************************************************************************/

bool SEN5X_Read(void) {
    
    if (sen5x_val.valid) { sen5x_val.valid--; }
    
    SensirionI2CSen5x();

    sen5x.startMeasurement();
    Wire.beginTransmission(SEN5X_ADDR);
    Wire.requestFrom(SEN5X_ADDR, 2);        // Sensirion data transmitted in 2 bytes
    if (Wire.available() == 2) {

        if (sen5x.startMeasurement() != 0){ return false; }
      
        sen5x.readMeasuredValues(sen5x_val.massConcentrationPm1p0,
        sen5x_val.massConcentrationPm2p5,
        sen5x_val.massConcentrationPm4p0,
        sen5x_val.massConcentrationPm10p0,
        sen5x_val.ambientHumidity,
        sen5x_val.ambientTemperature, sen5x_val.vocIndex,
        sen5x_val.noxIndex);
    }
    
    sen5x_val.valid = SENSOR_MAX_MISS;

    return true;
}

/********************************************************************************************/

void SEN5X_EverySecond(void) {
  if (!(TasmotaGlobal.uptime %4)) {  // Every 4 seconds
    if (!SEN5X_Read()) {
      AddLogMissed(sen5x_val.name, sen5x_val.valid);
    }
  }
}


// DEFINE UNITS
#define D_UNIT_PM "ug/m3"
#define D_UNIT_HUM "%"
#define D_UNIT_TEMP "ºC"

#ifdef USE_WEBSERVER
const char HTTP_SEN5X_SNS_pm[] PROGMEM ="{s}SEN5X " "%s" "{m}%s " D_UNIT_PM "{e}";
const char HTTP_SEN5X_SNS_hum[] PROGMEM ="{s}SEN5X " "%s" "{m}%s " D_UNIT_HUM "{e}";
const char HTTP_SEN5X_SNS_temp[] PROGMEM ="{s}SEN5X " "%s" "{m}%s " D_UNIT_TEMP "{e}";
const char HTTP_SEN5X_SNS_index[] PROGMEM ="{s}SEN5X " "%s" "{m}%s " "{e}";

#endif  // USE_WEBSERVER

/********************************************************************************************/

void SEN5X_Detect(void) {
  if (!I2cSetDevice(SEN5X_ADDR)) {return;}
  if (I2cSetDevice(SEN5X_ADDR)) {
    I2cSetActiveFound(SEN5X_ADDR, "SEN5X");
  }
}

bool data_ready_flag;

void SEN5X_Show(bool json) {
  // Have measurements stored in sen5x struct
  SEN5X_Read();


  if (!sen5x.readDataReady(data_ready_flag)) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("Hello World!\n"));
    return;
  }

  char str[64];
  if (json) {
    ResponseAppend_P(PSTR(",\"SEN5X\":{\"" "massConcentrationPm1p0" "\":%f"), sen5x_val.massConcentrationPm1p0);
    ResponseAppend_P(PSTR(",\"" "massConcentrationPm2p5" "\":%f"), sen5x_val.massConcentrationPm2p5);
    ResponseAppend_P(PSTR(",\"" "massConcentrationPm4p0" "\":%f"), sen5x_val.massConcentrationPm4p0);
    ResponseAppend_P(PSTR(",\"" "massConcentrationPm10p0" "\":%f"), sen5x_val.massConcentrationPm10p0);
    ResponseAppend_P(PSTR(",\"" "ambientHumidity" "\":%f"), sen5x_val.ambientHumidity);
    ResponseAppend_P(PSTR(",\"" "ambientTemperature" "\":%f"), sen5x_val.ambientTemperature);
    ResponseAppend_P(PSTR(",\"" "vocIndex" "\":%f"), sen5x_val.vocIndex);
    ResponseAppend_P(PSTR(",\"" "noxIndex" "\":%f"), sen5x_val.noxIndex);
  
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_SEN5X_SNS_pm,"PM 1.0",sen5x_val.massConcentrationPm1p0);
    WSContentSend_PD(HTTP_SEN5X_SNS_pm,"PM 2.5",sen5x_val.massConcentrationPm2p5);
    WSContentSend_PD(HTTP_SEN5X_SNS_pm,"PM 4.0",sen5x_val.massConcentrationPm4p0);
    WSContentSend_PD(HTTP_SEN5X_SNS_pm,"PM 10.0",sen5x_val.massConcentrationPm10p0);
    WSContentSend_PD(HTTP_SEN5X_SNS_hum,"Humidity",sen5x_val.ambientHumidity);
    WSContentSend_PD(HTTP_SEN5X_SNS_temp,"Temperature",sen5x_val.ambientTemperature);
    WSContentSend_PD(HTTP_SEN5X_SNS_index,"VOC Index",sen5x_val.vocIndex);
    WSContentSend_PD(HTTP_SEN5X_SNS_index,"NOX Index",sen5x_val.noxIndex);
#endif

  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns101(byte function)
{

  if (!I2cEnabled(XI2C_69)) { return false; }

  AddLog(LOG_LEVEL_DEBUG, PSTR("I2C enabled"));
  bool result = false;

  if (FUNC_INIT == function) {
    SEN5X_Detect();
  }
  else if (data_ready_flag) {
    
    switch (function) {
      case FUNC_EVERY_SECOND:
        SEN5X_EverySecond();
        break;
      case FUNC_JSON_APPEND:
        SEN5X_Show(1);
        break;
  #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        SEN5X_Show(0);
        break;
  #endif
    }
  }
  return result;
}

#endif  // USE_SEN5X
#endif  // USE_I2C