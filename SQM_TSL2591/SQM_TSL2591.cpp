/**************************************************************************/
/*!
    @file     SQM_TSL2591.h
    @author   gshau

    Forked from original Adafruit TSL2591 libraries
    @author   KT0WN (adafruit.com)
                @author   wbphelps (wm@usa.net)

    This is a library for the Adafruit TSL2591 breakout board
    This library works with the Adafruit TSL2591 breakout
    ----> https://www.adafruit.com/products/1980

    Check out the links above for our tutorials and wiring diagrams
    These chips use I2C to communicate

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014 Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifdef ESP8266
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

#if defined(__AVR__)
#include <util/delay.h>
#endif
#include <stdlib.h>

#include "SQM_TSL2591.h"

SQM_TSL2591::SQM_TSL2591(int32_t sensorID) {
  _initialized = false;
  _integration = TSL2591_INTEGRATIONTIME_400MS;
  _gain = TSL2591_GAIN_LOW;
  _sensorID = sensorID;
  _calibrationOffset = 0.;

  // we cant do wire initialization till later, because we havent loaded Wire
  // yet
}

boolean SQM_TSL2591::begin(void) {
  Wire.begin();

  uint8_t id = read8(0x12);
  if (id != 0x50) {
    return false;
  }

  _initialized = true;

  // Set default integration time and gain
  setTiming(_integration);
  setGain(_gain);
  setCalibrationOffset(_calibrationOffset);

  // Note: by default, the device is in power down mode on bootup
  disable();

  verbose = true;

  return _initialized;
}

void SQM_TSL2591::enable(void) {
  if (!_initialized) {
    if (!begin()) {
      return;
    }
  }

  // Enable the device by setting the control bit to 0x01
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
         TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN);
}

void SQM_TSL2591::disable(void) {
  if (!_initialized) {
    if (!begin()) {
      return;
    }
  }

  // Disable the device by setting the control bit to 0x00
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
         TSL2591_ENABLE_POWEROFF);
}

void SQM_TSL2591::setTemperatureCalibration (const temperatureCalibration &calibrationData) {
  _temperatureCalibration = calibrationData;
}

void SQM_TSL2591::setTemperature (float temperature) {
    _hasTemperature = true;
    _temperature = temperature;
}

void SQM_TSL2591::resetTemperature () {
    _hasTemperature = false;
}

void SQM_TSL2591::setGain(tsl2591Gain_t gain) {
  if (!_initialized) {
    if (!begin()) {
      return;
    }
  }

  enable();
  _gain = gain;
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, _integration | _gain);
  disable();

  switch (_gain) {
  case TSL2591_GAIN_LOW:
    gainValue = 1.0F;
    break;
  case TSL2591_GAIN_MED:
    gainValue = 25.0F;
    break;
  case TSL2591_GAIN_HIGH:
    gainValue = 425.0F;
    break;
  case TSL2591_GAIN_MAX:
    gainValue = 9876.0F;
    break;
  default:
    if (verbose) {
      Serial.println("Gain not found!");
    }
    break;
  }
}

void SQM_TSL2591::setCalibrationOffset(float calibrationOffset) {
  _calibrationOffset = calibrationOffset;
}

tsl2591Gain_t SQM_TSL2591::getGain() { return _gain; }

void SQM_TSL2591::setTiming(tsl2591IntegrationTime_t integration) {
  if (!_initialized) {
    if (!begin()) {
      return;
    }
  }

  enable();
  _integration = integration;
  write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, _integration | _gain);
  disable();

  switch (_integration) {
  case TSL2591_INTEGRATIONTIME_100MS:
    integrationValue = 100.F;
    break;
  case TSL2591_INTEGRATIONTIME_200MS:
    integrationValue = 200.F;
    break;
  case TSL2591_INTEGRATIONTIME_300MS:
    integrationValue = 300.F;
    break;
  case TSL2591_INTEGRATIONTIME_400MS:
    integrationValue = 400.F;
    break;
  case TSL2591_INTEGRATIONTIME_500MS:
    integrationValue = 500.F;
    break;
  case TSL2591_INTEGRATIONTIME_600MS:
    integrationValue = 600.F;
    break;
  default: // 100ms
    integrationValue = 999.F;
    if (verbose) {
      Serial.println("Integration not found!");
      Serial.println(_integration);
    }
    break;
  }
}

tsl2591IntegrationTime_t SQM_TSL2591::getTiming() { return _integration; }

void SQM_TSL2591::configSensor() {
  setGain(config.gain);
  setTiming(config.time);
}

void SQM_TSL2591::showConfig() {
  Serial.print("Integration: ");
  Serial.print(integrationValue);
  Serial.println(" ms");
  Serial.print("Gain:        ");
  Serial.print(gainValue);
  Serial.println("x");
}

uint32_t SQM_TSL2591::getFullLuminosity(void) {
  if (!_initialized) {
    if (!begin()) {
      return 0;
    }
  }

  // Enable the device
  enable();

  // Wait x ms for ADC to complete
  for (uint8_t d = 0; d <= _integration; d++) {
    delay(120);
  }

  uint32_t x;
  x = read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
  x <<= 16;
  x |= read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);

  disable();

  return x;
}

void SQM_TSL2591::bumpGain(int bumpDirection) {
  switch (config.gain) {
  case TSL2591_GAIN_LOW:
    if (bumpDirection > 0) {
      config.gain = TSL2591_GAIN_MED;
    } else {
      config.gain = TSL2591_GAIN_LOW;
    }
    break;
  case TSL2591_GAIN_MED:
    if (bumpDirection > 0) {
      config.gain = TSL2591_GAIN_HIGH;
    } else {
      config.gain = TSL2591_GAIN_LOW;
    }
    break;
  case TSL2591_GAIN_HIGH:
    if (bumpDirection > 0) {
      config.gain = TSL2591_GAIN_MAX;
    } else {
      config.gain = TSL2591_GAIN_MED;
    }
    break;
  case TSL2591_GAIN_MAX:
    if (bumpDirection > 0) {
      config.gain = TSL2591_GAIN_MAX;
    } else {
      config.gain = TSL2591_GAIN_HIGH;
    }
    break;
  default:
    break;
  }
  setGain(config.gain);
}

void SQM_TSL2591::bumpTime(int bumpDirection) {
  switch (config.time) {
  case TSL2591_INTEGRATIONTIME_200MS:
    if (bumpDirection > 0) {
      config.time = TSL2591_INTEGRATIONTIME_400MS;
    } else {
      config.time = TSL2591_INTEGRATIONTIME_200MS;
    }
    break;
  case TSL2591_INTEGRATIONTIME_400MS:
    if (bumpDirection > 0) {
      config.time = TSL2591_INTEGRATIONTIME_600MS;
    } else {
      config.time = TSL2591_INTEGRATIONTIME_200MS;
    }
    break;
  case TSL2591_INTEGRATIONTIME_600MS:
    if (bumpDirection > 0) {
      config.time = TSL2591_INTEGRATIONTIME_600MS;
    } else {
      config.time = TSL2591_INTEGRATIONTIME_400MS;
    }
    break;
  default:
    break;
  }
  setTiming(config.time);
}

void SQM_TSL2591::calibrateReadingsForTemperature(uint16_t &ir, uint16_t &full) {
  if (_hasTemperature) {
    if (verbose) {
        Serial.print("Values before temperature calibration: ir=");
        Serial.print(ir);
        Serial.print(", full=");
        Serial.println(full);
    }
    float irCalibrationFactor = _temperature * _temperatureCalibration.irSlope + _temperatureCalibration.irIntercept;
    float fullCalibrationFactor = _temperature * _temperatureCalibration.fullLuminositySlope + _temperatureCalibration.fullLuminosityIntercept;
    ir   = static_cast<uint16_t>(static_cast<float>(ir) * irCalibrationFactor);
    full = static_cast<uint16_t>(static_cast<float>(full) * fullCalibrationFactor);
    if (verbose) {
        Serial.print("Values after temperature calibration: ir=");
        Serial.print(ir);
        Serial.print(", full=");
        Serial.println(full);
    }

  }
}

void SQM_TSL2591::takeReading(void) {
  uint32_t lum;
  niter = 1;
  configSensor();
  lum = getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;
  calibrateReadingsForTemperature(ir, full);
  vis = full - ir;
  if ((float)full < (float)ir) {
    if (verbose) {
      Serial.println("Odd, full less than ir!  Rechecking...");
    }
    takeReading();
  }
  // When intensity is faint at current gain setting
  if ((float)vis < 128.) {
    if (_gain == TSL2591_GAIN_MAX) {
      if (_integration != TSL2591_INTEGRATIONTIME_600MS) {
        if (verbose) {
          Serial.println("Bumping integration up");
        }
        bumpTime(1);
        if (verbose)
          showConfig();
        configSensor();
        lum = getFullLuminosity();
        delay(50);
        takeReading();
      } else {
        uint32_t fullCumulative;
        uint16_t visCumulative, irCumulative;

        fullCumulative = full;
        irCumulative = ir;
        visCumulative = vis;
        // Do iterative sampling to gain statistical certainty
        while ((float)visCumulative < 128.) {
          niter++;
          delay(50);
          lum = getFullLuminosity();
          ir = lum >> 16;
          full = lum & 0xFFFF;
          calibrateReadingsForTemperature(ir, full);
          fullCumulative += full;
          irCumulative += ir;
          visCumulative = fullCumulative - irCumulative;
          if (niter > 32) {
            break;
          }
        }
        if ((float)fullCumulative > (float)irCumulative) {
          full = fullCumulative;
          ir = irCumulative;
          vis = visCumulative;
        } else {
          if (verbose) {
            Serial.println("Odd, full less than ir!  Rechecking...");
          }
          takeReading();
        }
      }
    } else {
      if (verbose) {
        Serial.println("Bumping gain up");
      }
      bumpGain(1);
      if (verbose)
        showConfig();
      configSensor();
      lum = getFullLuminosity();
      delay(50);
      takeReading();
    }
  }
  // If saturated, bump down integration or gain
  else if (full == 0xFFFF || ir == 0xFFFF) {
    if ((_gain == TSL2591_GAIN_MAX) &
        (_integration != TSL2591_INTEGRATIONTIME_200MS)) {
      if (verbose) {
        Serial.println("Bumping integration down");
      }
      bumpTime(-1);
      if (verbose)
        showConfig();
      configSensor();
      lum = getFullLuminosity();
      delay(50);
      takeReading();
    } else {
      if (verbose) {
        Serial.println("Bumping gain down");
      }
      bumpGain(-1);
      if (verbose)
        showConfig();
      configSensor();
      lum = getFullLuminosity();
      delay(50);
      takeReading();
    }
  }

  float IR = (float)ir / (gainValue * integrationValue / 200.F * niter);
  float VIS = (float)vis / (gainValue * integrationValue / 200.F * niter);
  mpsas = 12.6 - 1.086 * log(VIS) + _calibrationOffset;
  dmpsas = 1.086 / sqrt((float)vis);
}

uint8_t SQM_TSL2591::read8(uint8_t reg) {
  Wire.beginTransmission(TSL2591_ADDR);
  Wire.write(0x80 | 0x20 | reg); // command bit, normal mode
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 1);
  while (!Wire.available())
    ;
  return Wire.read();
}

uint16_t SQM_TSL2591::read16(uint8_t reg) {
  uint16_t x;
  uint16_t t;

  Wire.beginTransmission(TSL2591_ADDR);
#if ARDUINO >= 100
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();

  Wire.requestFrom(TSL2591_ADDR, 2);
#if ARDUINO >= 100
  t = Wire.read();
  x = Wire.read();
#else
  t = Wire.receive();
  x = Wire.receive();
#endif
  x <<= 8;
  x |= t;
  return x;
}

void SQM_TSL2591::write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TSL2591_ADDR);
#if ARDUINO >= 100
  Wire.write(reg);
  Wire.write(value);
#else
  Wire.send(reg);
  Wire.send(value);
#endif
  Wire.endTransmission();
}

float SQM_TSL2591::calculateLux(uint16_t ch0, uint16_t ch1) /*wbp*/
{
  float atime, again; /*wbp*/
  float cpl, lux1, lux2, lux;
  //  uint32_t chan0, chan1; /*wbp*/

  // Check for overflow conditions first
  if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF)) {
    // Signal an overflow
    return 0.0;
  }

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (_integration) {
  case TSL2591_INTEGRATIONTIME_100MS:
    atime = 100.0F;
    break;
  case TSL2591_INTEGRATIONTIME_200MS:
    atime = 200.0F;
    break;
  case TSL2591_INTEGRATIONTIME_300MS:
    atime = 300.0F;
    break;
  case TSL2591_INTEGRATIONTIME_400MS:
    atime = 400.0F;
    break;
  case TSL2591_INTEGRATIONTIME_500MS:
    atime = 500.0F;
    break;
  case TSL2591_INTEGRATIONTIME_600MS:
    atime = 600.0F;
    break;
  default: // 200ms
    atime = 200.0F;
    break;
  }

  switch (_gain) {
  case TSL2591_GAIN_LOW:
    //      again = 1.0F;
    again = 1.03F; /*wbp*/
    break;
  case TSL2591_GAIN_MED:
    again = 25.0F;
    break;
  case TSL2591_GAIN_HIGH:
    //      again = 428.0F;
    again = 425.0F; /*wbp*/
    break;
  case TSL2591_GAIN_MAX:
    //      again = 9876.0F;
    again = 7850.0F; /*wbp*/
    break;
  default:
    again = 1.0F;
    break;
  }

  // cpl = (ATIME * AGAIN) / DF
  cpl = (atime * again) / TSL2591_LUX_DF;

  lux1 = ((float)ch0 - (TSL2591_LUX_COEFB * (float)ch1)) / cpl;
  lux2 = ((TSL2591_LUX_COEFC * (float)ch0) - (TSL2591_LUX_COEFD * (float)ch1)) /
         cpl;

  // The highest value is the approximate lux equivalent
  lux = lux1 > lux2 ? lux1 : lux2;

  // Signal I2C had no errors
  //  return (uint32_t)lux;return (uint32_t)lux;
  return lux; /*wbp*/
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool SQM_TSL2591::getEvent(sensors_event_t *event) {
  uint16_t ir, full;
  uint32_t lum = getFullLuminosity();
  /* Early silicon seems to have issues when there is a sudden jump in */
  /* light levels. :( To work around this for now sample the sensor 2x */
  lum = getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_LIGHT;
  event->timestamp = millis();

  /* Calculate the actual lux value */
  /* 0 = sensor overflow (too much light) */
  event->light = calculateLux(full, ir);
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void SQM_TSL2591::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "TSL2591", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_LIGHT;
  sensor->min_delay = 0;
  sensor->max_value = 88000.0;
  sensor->min_value = 0.001;
  sensor->resolution = 0.001;
}
