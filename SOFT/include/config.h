#pragma once
// ════════════════════════════════════════════════════════════════
//  INCLUDES
// ════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <math.h>
// ════════════════════════════════════════════════════════════════
//  OPTIMIZATIONS
// ════════════════════════════════════════════════════════════════
#define HOT_O3 __attribute__((optimize("O3"), hot))
// ════════════════════════════════════════════════════════════════
//  PINOUT
// ════════════════════════════════════════════════════════════════
#define PIN_LD06_RX         0   // Serial 1 - LD06
#define NC_0                1
#define PIN_LED_DATA        2   // LED
#define PIN_GENERAL_GPIO_1  3   // General GPIO - Enable Pump
#define PIN_GENERAL_GPIO_2  4   // General GPIO - IN 3
#define PIN_GENERAL_GPIO_3  5   // General GPIO - IN 2
#define PIN_PAW3805_MFIO    6   // PAW3805
#define PIN_SCS_RX          7   // Serial 2 - SCS0009
#define PIN_SCS_TX          8   // Serial 2 - SCS0009
#define PIN_PAW3805_MOTION  9   // PAW3805
#define PIN_PAW3805_CS      10  // SPI0 - PAW3805
#define NC_1                11  
#define PIN_PAW3805_SDIO    12  // SPI0 - PAW3805
#define PIN_PAW3805_SCK     13  // SPI0 - PAW3805
#define PIN_MOTOR_A_TX      14  // Serial 3 - MOTOR A
#define PIN_MOTOR_A_RX      15  // Serial 3 - MOTOR A
#define PIN_MOTOR_B_RX      16  // Serial 4 - MOTOR B
#define PIN_MOTOR_B_TX      17  // Serial 4 - MOTOR B
#define PIN_MPU6050_SDA     18  // I2C0 - MPU6050
#define PIN_MPU6050_SCL     19  // I2C0 - MPU6050
#define PIN_MOTOR_C_TX      20  // Serial 5 - MOTOR C
#define PIN_MOTOR_C_RX      21  // Serial 5 - MOTOR C
#define PIN_MOTORS_ENB      22  // MOTORS ENABLE
#define PIN_GENERAL_GPIO_4  23  // General GPIO    
#define PIN_GENERAL_COM_TX  24  // Serial 6 - Grove AI
#define PIN_GENERAL_COM_RX  25  // Serial 6 - Grove AI
#define PIN_GENERAL_GPIO_5  26  // General GPIO
#define PIN_GENERAL_GPIO_6  27  // General GPIO - IN 4
#define PIN_BW16_TX         28  // Serial 7 - BW16
#define PIN_BW16_RX         29  // Serial 7 - BW16
#define PIN_GENERAL_GPIO_7  30  // General GPIO
#define PIN_GENERAL_GPIO_8  31  // General GPIO - IN 1
#define PIN_GENERAL_GPIO_9  32  // General GPIO
#define PIN_GENERAL_GPIO_10 33  // General GPIO

#define TEAM_YELLOW 0
#define TEAM_BLUE   1
#define TEAM_UNDEFINED 2
// ════════════════════════════════════════════════════════════════
//  FAST SIN / COS
// ════════════════════════════════════════════════════════════════
#define LUT_SIZE    3600
// ════════════════════════════════════════════════════════════════
//  STRUCTURES
// ════════════════════════════════════════════════════════════════
struct Pose2D 
{
    float x, y, theta;
};

struct Point2D
{
  float x;
  float y;
};

struct Segment 
{
    float x1, y1, x2, y2;
    float nx, ny;       // normale unitaire (intérieur)
    float d;            // offset : nx*x + ny*y + d = 0
    float dx, dy;       // vecteur direction (x2-x1, y2-y1)
    float len_inv;      // 1 / (dx²+dy²) — précalculé
};

struct DeposeZone
{
    float x;
    float y;
    float length;
};
// ════════════════════════════════════════════════════════════════
//  HARDWARE UART - WIRE
// ════════════════════════════════════════════════════════════════
#define SERVO_SERIAL        Serial2

#define CAMERA_SERIAL       Serial6
#define BW16_SERIAL         Serial7
// ════════════════════════════════════════════════════════════════
//  LD06
// ════════════════════════════════════════════════════════════════
#define LIDAR_SERIAL        Serial1
static constexpr uint16_t   LD06_POINT_MAX_SIZE   = 300;              // max points per scan
static constexpr uint32_t   LD06_TIMEOUT_MS       = 150;              // full scan timeout
static constexpr uint8_t    LD06_DATA_LEN         = 47;               // packet size
static constexpr uint8_t    LD06_PTS_PER_PKT      = 12;               // points per packet
static constexpr uint16_t   LD06_MIN_DIST_MM      = 55;               // minimal distance
static constexpr uint16_t   LD06_MAX_DIST_MM      = 3600;             // maximal distance
static constexpr uint8_t    LD06_MIN_INTENSITY    = 200;              // minimal intensity
static constexpr uint32_t   LD06_PT_PERIOD_US     = 222;              // µs (4500 Hz frequency)
// ════════════════════════════════════════════════════════════════
//  IMU
// ════════════════════════════════════════════════════════════════
#define IMU_WIRE            Wire
#define IMU_ADDR            0x68
static constexpr uint32_t   IMU_PERIOD_US           = 2000;               // 500 Hz
// ════════════════════════════════════════════════════════════════
//  PAW3805
// ════════════════════════════════════════════════════════════════
static constexpr uint32_t   MOUSE_PERIOD_US         = 2000;               // 500 Hz
static constexpr float      CPI_TO_MM               = 0.0078f;            // 3250 count for 25.4mm
// ════════════════════════════════════════════════════════════════
//  ODOMETRY
// ════════════════════════════════════════════════════════════════
static constexpr uint32_t   ODOM_PERIOD_US          = 2000;                 // 500 Hz

// ════════════════════════════════════════════════════════════════
//  MAP
// ════════════════════════════════════════════════════════════════
static constexpr float      MAP_SIZE_X_MM       = 3000.f;
static constexpr float      MAP_SIZE_Y_MM       = 2000.0f;
static constexpr uint8_t    MAP_SEG_COUNT       = 8;

static const     Segment    MAP_SEGMENTS[MAP_SEG_COUNT] = 
{
    // x1      y1      x2      y2      nx    ny     d        dx      dy     len_inv
    {   0.f,    0.f,    0.f, 2000.f,  1.f,  0.f,    0.f,    0.f, 2000.f, 1.f/4000000.f },
    {   0.f, 2000.f,  600.f, 2000.f,  0.f, -1.f, 2000.f,  600.f,    0.f, 1.f/360000.f  },
    { 600.f, 2000.f,  600.f, 1550.f, -1.f,  0.f,  600.f,    0.f, -450.f, 1.f/202500.f  },
    { 600.f, 1550.f, 2400.f, 1550.f,  0.f, -1.f, 1550.f, 1800.f,    0.f, 1.f/3240000.f },
    {2400.f, 1550.f, 2400.f, 2000.f,  1.f,  0.f,-2400.f,    0.f,  450.f, 1.f/202500.f  },
    {2400.f, 2000.f, 3000.f, 2000.f,  0.f, -1.f, 2000.f,  600.f,    0.f, 1.f/360000.f  },
    {3000.f, 2000.f, 3000.f,    0.f, -1.f,  0.f, 3000.f,    0.f,-2000.f, 1.f/4000000.f },
    {3000.f,    0.f,    0.f,    0.f,  0.f,  1.f,    0.f,-3000.f,    0.f, 1.f/9000000.f },
};

// ════════════════════════════════════════════════════════════════
//  PATHFINDING
// ════════════════════════════════════════════════════════════════
static constexpr uint32_t   PATHFINDING_PERIOD_US = 20000;                // 20 ms
static constexpr float      MAP_RESOL_MM        = 25.0f;
static constexpr uint8_t    MAP_INC_X           = 120;
static constexpr uint8_t    MAP_INC_Y           = 80;
static constexpr uint8_t    CELL_COST_MAX       = 200;
static constexpr uint16_t   N_CELLS             = 9600;
static constexpr float      COST_FACTOR         = 0.02f;
static constexpr uint8_t    N_GOALS             = 32;
static constexpr uint8_t    CELL_FORBIDDEN_VALUE = 255;
static constexpr uint8_t    CELL_MAX_VALUE      =   200;
static constexpr uint8_t    N_NEIGHBORS          = 8;
static constexpr uint8_t    DECREMENT_VALUE     = 2;   // per 20ms => 250 value obstacle disappear time around 2.5s
static constexpr uint8_t    ROBOT_MASK_RADIUS   = 2;
static constexpr uint8_t    OBSTACLE_MASK_RADIUS  = 3;
static constexpr uint8_t    N_ZONES                 = 10;
static constexpr uint16_t      MAX_OBSTACLE_DIST       = 750;

static constexpr DeposeZone zone[N_ZONES] = {
            {   700.0f  , 100.0f    , 200.0f  },            // 0
            {   1500.0f , 100.0f    , 200.0f  },            // 1
            {   2300.0f , 100.0f    , 200.0f  },            // 2
            {   100.0f  , 800.0f    , 200.0f  },            // 3
            {   800.0f  , 800.0f    , 200.0f  },            // 4
            {   1500.0f , 800.0f    , 200.0f  },            // 5
            {   2200.0f , 800.0f    , 200.0f  },            // 6
            {   2900.0f , 800.0f    , 200.0f  },            // 7
            {   1250.0f , 1450.0f   , 200.0f  },            // 8
            {   1750.0f , 1450.0f   , 200.0f  }};           // 9

static constexpr uint8_t obstacleMask[7][7] = {
            { 100 , 150 , 200,  200,  200,  150,   100  },
            { 150 , 200 , 250,  250,  250,  200,   150  },
            { 150 , 200 , 250,  250,  250,  200,   150  },
            { 150 , 200 , 250,  250,  250,  200,   150  },
            { 150 , 200 , 250,  250,  250,  200,   150  },
            { 150 , 200 , 250,  250,  250,  200,   150  },
            { 100 , 150 , 200,  200,  200,  150,   100  }};

// N, NE, E, SE, S, SW, W, NW
static constexpr int8_t dx[N_NEIGHBORS] = {0,1,1,1,0,-1,-1,-1};
static constexpr int8_t dy[N_NEIGHBORS] = {1,1,0,-1,-1,-1,0,1};

// ════════════════════════════════════════════════════════════════
//  PARTICLE GENERATION
// ════════════════════════════════════════════════════════════════
static constexpr uint16_t   N_PARTICLES             = 2000;          // nb particle max
static constexpr uint8_t    N_MIN_POINTS            = 150;          // nb scan point min (50%)

static constexpr float      SIGMA_XY_GLOBAL         = 3000.0f;
static constexpr float      SIGMA_TH_GLOBAL         = 2 *M_PI;

static constexpr float      ANGLE_QUANT_STEP        = 0.15f;
static constexpr float      OUTLIER_JUMP            = 400.0f;
// ════════════════════════════════════════════════════════════════
//  PARTICLE SCORE
// ════════════════════════════════════════════════════════════════
static constexpr float    SCORE_INLIER_DIST_MM  = 200.f;
static constexpr float    SCORE_DIST_PERFECT_MM = 15.f;
static constexpr uint8_t  SCORE_WALLS_MAX       = 4;
static constexpr uint8_t  N_INLIERS_MIN         = 10;
static constexpr uint8_t  N_INLIERS_MAX         = 90;
// ════════════════════════════════════════════════════════════════
//  FUSION
// ════════════════════════════════════════════════════════════════
static constexpr float    PARTICLE_TOP_FRACTION = 0.1f;                 // 10%
static constexpr float    CLUSTER_RADIUS_MM     = 100.f; 
static constexpr float    CLUSTER_TH_RAD        = 10.0f * DEG_TO_RAD;      
static constexpr float    SCORE_PERFECT         = 200.0f;
static constexpr float    SCORE_MEDIUM          = 100.0f;
static constexpr float    SCORE_BAD             = 50.0f;
// ════════════════════════════════════════════════════════════════
//  ROBOT
// ════════════════════════════════════════════════════════════════
static constexpr float    ROBOT_RADIUS          = 65.0f;
static constexpr float    WHEEL_PERIMETER       = 25.0f * 2 * M_PI;
// ════════════════════════════════════════════════════════════════
//  PROPULSION
// ════════════════════════════════════════════════════════════════
#define MOT_T_SERIAL        Serial3
#define MOT_D_SERIAL        Serial4
#define MOT_S_SERIAL        Serial5
static constexpr uint32_t   PROP_PERIOD_US              = 20000;                // 20 ms
static constexpr float      ARRIVED_DIST                = 25.0f;                // 50mm
static constexpr float      ARRIVED_ANG                 = 5.0f * DEG_TO_RAD;    // 5°
static constexpr float      JITTER_DIST                 = 10.0f;                // 25mm
static constexpr float      JITTER_ANG                  = 2.0f * DEG_TO_RAD;    // 2°
static constexpr float      XY_SPEED_MAX                = 700.0;                // 600 mm/s
static constexpr float      XY_SPEED_MIN                = 100.0;                // 300 mm/s
static constexpr uint32_t   MOTOR_USTEP_MAX             = 4000;                 // 2.5tr/s
static constexpr int32_t    MOTOR_ACC_MAX               = 16; //8                    // µstep/ms
static constexpr int16_t    MOTOR_USTEP_PER_STEP        = 8;                    // ustep/step
static constexpr uint32_t   MOTOR_USTEP_PER_TURN        = 1600;
static constexpr uint8_t    MOTOR_CURRENT_PERCENT       = 40;                   // 1 A
static constexpr uint8_t    GOAL_HORIZON                = 5;   //5;
static constexpr float      SPEED_COST_FACTOR           = 0.01f;
// ════════════════════════════════════════════════════════════════
//  BRAS
// ════════════════════════════════════════════════════════════════
static constexpr uint8_t    SCS_ID_BRAS                 = 2;
static constexpr uint8_t    SCS_ID_MAIN                 = 1;
static constexpr uint32_t   SCS_TRAVEL_TIME             = 1000;
static constexpr uint32_t   ACTION_PERIOD_MS            = 250;
// ════════════════════════════════════════════════════════════════
//  IHM
// ════════════════════════════════════════════════════════════════
static constexpr uint8_t    MAX_MESS_SIZE               = 250;
static constexpr uint32_t   IHM_PERIOD_MS               = 100;   
// ════════════════════════════════════════════════════════════════
//  WRAP ANGLE
// ════════════════════════════════════════════════════════════════
FASTRUN inline float wrapAngle(float a) 
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}
