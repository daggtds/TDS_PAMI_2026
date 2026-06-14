#pragma once
//  ----------------------------------------------------------------------------------------------
#include <Arduino.h>
//  ----------------------------------------------------------------------------------------------
//  ----------------------------------------------------------------------------------------------
#define LD06_BAUDRATE           230400  // LD06 baudrate
#define LD06_START_BYTE         0x54    // header
#define LD06_LENGTH_BYTE        0x2C    // data length (fixed)
//  ----------------------------------------------------------------------------------------------
static const uint8_t LD06_CRC_TABLE[256] =
{
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

struct ScanPoint
{
    uint16_t    dist;
    float       ang;
    float       rx;     // in robot reference
    float       ry;     // in robot reference
    uint32_t    t_us;   // timestamp
};

struct LidarScan
{
    ScanPoint   pts[LD06_POINT_MAX_SIZE];
    uint16_t    count;
};
//  ----------------------------------------------------------------------------------------------
class LD06
{
    public:
        LD06(HardwareSerial *_serial)
        {
            pserial = _serial;
        };

        void begin()
        {
            pserial->begin(LD06_BAUDRATE);
        };

        bool update(uint32_t timeout)
        {
            bool scan_is_ready = false;

            if(ld06_timer > timeout)
            {
                swapScans();
                scan_is_ready = true;
                ld06_timer = 0;
            }
            else
            {
                while(pserial->available())
                {
                    scan_is_ready |= readPacket();
                }
            }
            if(scan_is_ready)   ld06_timer = 0;
            return scan_is_ready;
        };

        void teleplot(Stream &serialport = Serial) 
        {          
            serialport.print(F(">lidar.cloud,loc:"));
            for(int i=0;i<previous_scan.count;i++)
            {
                serialport.print(String()+previous_scan.pts[i].rx+":"+previous_scan.pts[i].ry+";");
            }               
            serialport.println(F("|xy"));
        };

        LidarScan& getScan()  {return previous_scan;};

    private:
        HardwareSerial  *pserial;
        elapsedMillis   ld06_timer      = 0;
        uint8_t         raw_data[LD06_DATA_LEN];
        uint8_t         raw_data_idx    = 0;
        LidarScan       current_scan;
        LidarScan       previous_scan;

        static inline float normalize_angle(float ang)
        {
            while(ang < -M_PI)  ang+= 2 * M_PI;
            while(ang > M_PI)   ang-= 2 * M_PI;
            return ang;
        };

        bool readPacket()
        {
            uint8_t d = pserial->read();

            if(raw_data_idx == 0)
            {
                if(d == LD06_START_BYTE)
                {
                    raw_data[raw_data_idx++] = d;
                }
            }
            else
            {
                if(raw_data_idx == 1)
                {
                    if(d == LD06_LENGTH_BYTE)
                    {
                        raw_data[raw_data_idx++] = d;
                    }
                    else
                    {
                        raw_data_idx = 0;  
                    }                   
                }
                else
                {
                    raw_data[raw_data_idx++] = d;
                    if(raw_data_idx >= LD06_DATA_LEN)
                    {
                        raw_data_idx = 0;
                        if(checkCRC())  return decodePacket();
                        else            return false;
                    }
                }
            }
            return false;
        };

        bool decodePacket()
        {
            bool has_swaped = false;

            // angle step calculation
            float start_angle   = (float)((uint16_t)(raw_data[5] << 8 | raw_data[4])) * 0.01f;      // in °
            float end_angle     = (float)((uint16_t)(raw_data[43] << 8 | raw_data[42])) * 0.01f;    // in °
            if(end_angle < start_angle) end_angle += 360.0f;
            float angle_step    = (end_angle - start_angle) / (float)(LD06_PTS_PER_PKT - 1);
            
            // scan timestamp
            uint32_t t_us       = micros();

            // distance, intensity filter
            for(uint8_t i=0; i<LD06_PTS_PER_PKT; i++)
            {
                uint8_t base        = 6 + i * 3;
                uint16_t dist_mm    = (uint16_t)(raw_data[base+1] << 8 | raw_data[base]);
                uint8_t  intensity  = raw_data[base+2];

                if(dist_mm < LD06_MIN_DIST_MM)      continue;
                if(dist_mm > LD06_MAX_DIST_MM)      continue;
                if(intensity < LD06_MIN_INTENSITY)  continue; 

                float       point_angle_rad = (start_angle + (float)i * angle_step) * DEG_TO_RAD + M_PI;    // lidar is reversed
                point_angle_rad = normalize_angle(point_angle_rad);
                
                // REMOVE POINTS NEAR ANGLE 0,120,-120 deg
                if((point_angle_rad > -11*DEG_TO_RAD) && (point_angle_rad < 11*DEG_TO_RAD))         continue;
                if((point_angle_rad > 108*DEG_TO_RAD) && (point_angle_rad < 132*DEG_TO_RAD))        continue;
                if((point_angle_rad  < -118 *DEG_TO_RAD) && (point_angle_rad > -137*DEG_TO_RAD))    continue;

                uint32_t pt_timestamp = t_us - (LD06_PTS_PER_PKT - i - 1) * LD06_PT_PERIOD_US;
                float lx = dist_mm * cosf(point_angle_rad);
                float ly = dist_mm * sinf(point_angle_rad);
                current_scan.pts[current_scan.count++] = {dist_mm, point_angle_rad, lx, ly, pt_timestamp}; 
                
                if(current_scan.count >= LD06_POINT_MAX_SIZE)
                {
                    swapScans();
                    has_swaped = true;
                }
            }
            return has_swaped;
        };

        bool checkCRC()
        {
            uint8_t crc = 0;
            for(uint8_t i=0; i<LD06_DATA_LEN-1; i++)
            {
                crc = LD06_CRC_TABLE[(crc ^ raw_data[i]) & 0xFF];
            }
            if(raw_data[LD06_DATA_LEN-1] == crc)    return true;
            else                                    return false;
        };

        void swapScans()
        {
            LidarScan tmp_scan  = current_scan;
            current_scan        = previous_scan;
            previous_scan       = tmp_scan;
            current_scan.count  = 0;
        };
};