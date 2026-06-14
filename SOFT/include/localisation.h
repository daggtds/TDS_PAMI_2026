#pragma once
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
#include "drivers/imu.h"
#include "drivers/mouse.h"
#include "drivers/lidar.h"
//-----------------------------------------------------------------------------
struct Particle
{
    float x;
    float y;
    float theta;
    float score;
};

//-----------------------------------------------------------------------------
class Localizer 
{
    public:
        Localizer()
            : _lidar(&Serial1)
            , _mouse(PIN_PAW3805_SDIO, PIN_PAW3805_SCK,
                    PIN_PAW3805_CS, PIN_PAW3805_MOTION, PIN_PAW3805_MFIO){}
        
        bool        begin();
        bool        update();
        Pose2D      getPose()       const { return {_odomPose.x,_odomPose.y,_odomPose.theta};};
        void        setPose(float x, float y, float theta);
        float       getScore()      const { return _bestScore;};
        ScanPoint*  getScan()       { return _lidar.getScan().pts;};        // { return _correctedScan;};
        uint16_t    getScanCount()  { return _lidar.getScan().count;};    // { return _nCorrectedPt;};
        void        teleplot();              

private:
    // State machine
    enum class State : uint8_t
    {
        ERROR,
        WAIT_SCAN,
        CORRECT_SCAN,
        FUSE_POSE
    };
    State                   _state = State::ERROR;
    elapsedMicros           _loopTimer = 0;

    // Sensors
    LD06                    _lidar;
    PAW3805                 _mouse;
    IMU                     _imu;
    void                    readSensors();

    // Odometry
    struct OdomPose 
    {
        uint32_t t_us;
        float    x, y, theta;
    };
    OdomPose                _odomPose = {micros(),MAP_SIZE_X_MM*0.5f,MAP_SIZE_Y_MM*0.5f,0.0f};
    elapsedMicros           _odomTimer;
    OdomPose                _lastGoodPosition;

    // Scan undistortion
    void                    correctScan(const ScanPoint* pts, uint16_t count, const OdomPose &pt0, const OdomPose &pt1);
    ScanPoint               _correctedScan[LD06_POINT_MAX_SIZE];
    uint16_t                _nCorrectedPt = 0;
    bool                    _scanAvailable = false;
    static inline float     median3(float a, float b, float c);
    static inline float     quantizeAngle(float angle_rad);
    static inline float     lowpassGyro(float prev, float current);

    // Particle generation
    OdomPose                _poseT0;
    OdomPose                _poseT1;
    Particle                _particles[N_PARTICLES];
    uint16_t                _nGenerated     = 0;
    uint16_t                _nConverged     = 0;

    void                    generateParticle(const Pose2D &p, float spreadX, float spreadY, float spreadTh);
    void                    generateRandomParticle(const Pose2D &lastGoodPos);
    float                   evaluateParticle(uint16_t iPart);
    float                   randN();

    // Odometry & particle fusion
    Particle                _mclParticle;
    float                   _bestScore = 0.0f;
    void                    fusePose(const Particle &mclParticle, const Pose2D &odomPose);
    static Pose2D           interpPose(const Pose2D &a, const Pose2D &b, float t);  
    bool                    _isLost = false;                      

    // map
    struct SegAssoc
    {
        float   dist;
        int8_t  seg;
    };
    FASTRUN SegAssoc nearestSegment(float wx, float wy, float maxDist);
    FASTRUN inline bool insideTable(float wx, float wy); 

    float   _sin_lut[LUT_SIZE];
    float   _cos_lut[LUT_SIZE];
    void    initTrigLUT();
    void    fastSinCos(float angle, float *s, float *c);
};

    
