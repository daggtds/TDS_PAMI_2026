#pragma once
//  ----------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
//  ----------------------------------------------------------------------------------------------
class IMU
{
    public:
        struct ImuSample
        {
            float   gyroZ_rads; // rad/s
            float   tempC;      // internal temperature
            bool    valid;      // if new data available
        };

        IMU() = default;

        bool begin()
        {
            Wire.begin();
            Wire.setClock(400000);

            uint8_t id = readReg(REG_WHO_AM_I);
            if(id != 0x68)  return false;
            
            writeReg(REG_PWR_MGMT_1, 0x80); // Reset
            delay(100);

            writeReg(REG_PWR_MGMT_1, 0x01); // Clock source : PLL
            delay(10);

            //writeReg(REG_CONFIG, 0x03);     // DLPF : 42Hz - low bypass filter

            writeReg(REG_SMPLRT_DIV, 0x00); // 1Khz : gyro_rate / (1 + SMPLRT_DIV)

            writeReg(REG_GYRO_CONFIG, 0x08);    // Gyro : +/- 500°/s (hyp : max 360°/s)

            writeReg(REG_ACCEL_CONFIG, 0x00);   // Accel : +/- 2g (not used)

            _timer = 0;
            return true;
        };

        void calibrate(uint16_t n = 1000)
        {
            float sum = 0.0f;

            for(uint16_t i=0; i<n; i ++)
            {
                uint8_t buf[2];
                if(readBurst(REG_GYRO_ZOUT_H, buf, 2))
                {
                    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
                    sum += raw;
                }
                delay(2);
            }
            _gyroBias = (float)(sum / n) * GYRO_SCALE;
            Serial.println(String()+"Gyrobias : "+_gyroBias * RAD_TO_DEG);
        };

        bool update()
        {
            if(_timer < IMU_PERIOD_US)  return false;
            _timer -= IMU_PERIOD_US;

            // Burst read : TEMP(2) + GYRO_X(2) + GYRO_Y(2) + GYRO_Z(2) = 8 bytes
            uint8_t buf[8];
            if(!readBurst(REG_TEMP_OUT_H, buf, 8))
            {
                _last.valid = false;
                return false;
            }

            int16_t tz = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t gz = (int16_t)((buf[6] << 8) | buf[7]);

            _last.tempC = tz * TEMP_SCALE + TEMP_OFFSET;
            _last.gyroZ_rads = gz * GYRO_SCALE - _gyroBias;
            _last.valid = true;

            return true;
        };

        const ImuSample& getSample()    const {return _last;};

    private:
        // MPU6050 registers
        static constexpr uint8_t REG_PWR_MGMT_1     = 0x6B;
        static constexpr uint8_t REG_SMPLRT_DIV     = 0x19;
        static constexpr uint8_t REG_CONFIG         = 0x1A;
        static constexpr uint8_t REG_GYRO_CONFIG    = 0x1B;
        static constexpr uint8_t REG_ACCEL_CONFIG   = 0x1C;
        static constexpr uint8_t REG_ACCEL_XOUT_H   = 0x3B;
        static constexpr uint8_t REG_TEMP_OUT_H     = 0x41;
        static constexpr uint8_t REG_GYRO_ZOUT_H    = 0x47;
        static constexpr uint8_t REG_WHO_AM_I       = 0x75;

        // ±500 °/s → LSB = 65.5
        static constexpr float GYRO_SCALE = 1.0f / 65.5f * (M_PI / 180.0f);
        // ±2g → LSB = 16384
        static constexpr float ACCEL_SCALE = 1.0f / 16384.0f;
        static constexpr float TEMP_SCALE  = 1.0f / 340.0f;
        static constexpr float TEMP_OFFSET = 36.53f;

        ImuSample       _last;
        float           _gyroBias = 0.0f;
        elapsedMicros   _timer;

        void    writeReg(uint8_t reg, uint8_t val)
        {
            Wire.beginTransmission(IMU_ADDR);
            Wire.write(reg);
            Wire.write(val);
            Wire.endTransmission();
        };

        uint8_t readReg(uint8_t reg)
        {
            Wire.beginTransmission(IMU_ADDR);
            Wire.write(reg);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
            return Wire.available() ? Wire.read() : 0;
        };

        bool    readBurst(uint8_t reg, uint8_t *buf, uint8_t len)
        {
            Wire.beginTransmission(IMU_ADDR);
            Wire.write(reg);
            if(Wire.endTransmission(false) != 0)    return false;
            Wire.requestFrom((uint8_t)IMU_ADDR, len);
            for(uint8_t i=0; i<len; i++)
            {
                if(!Wire.available())   return false;
                buf[i] = Wire.read();
            }
            return true;
        };
};