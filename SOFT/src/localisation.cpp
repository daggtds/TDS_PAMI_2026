#include "localisation.h"
//-----------------------------------------------------------------------------
// ════════════════════════════════════════════════════════════════
//  begin()
// ════════════════════════════════════════════════════════════════
bool Localizer::begin()
{
    _state = State::ERROR;
    
    if (!_imu.begin())      // Init MPU6050 
    {
        Serial.println("MPU6050 not detected !");
        return false;
    }
    _imu.calibrate(2000);

    if (!_mouse.begin())    // init PAW3805 
    {
        Serial.println("PAW3805 not detected !");
        return false;
    }

    _lidar.begin();         // Init LD06
    _odomTimer   = 0;

    // Initial position unkown => global localisation from center of table
    _odomPose = {micros(),MAP_SIZE_X_MM * 0.5f, MAP_SIZE_Y_MM * 0.5f, M_PI * 0.5f};
    _poseT0 = _odomPose;
    _poseT1 = _odomPose;
    srand(micros());
    _scanAvailable = false;
    _isLost = false;
    _state = State::WAIT_SCAN;
    return true;
}
// ════════════════════════════════════════════════════════════════
//  lowpassGyro()
// ════════════════════════════════════════════════════════════════
inline float Localizer::lowpassGyro(float prev, float current)
{
    const float alpha = 0.1f;

    return prev + alpha * (current - prev);
}
// ════════════════════════════════════════════════════════════════
//  readSensors()
// ════════════════════════════════════════════════════════════════
void Localizer::readSensors()
{
    if (_odomTimer >= ODOM_PERIOD_US) 
    {
        uint32_t dt_us = _odomTimer;
        _odomTimer = 0;
        float dx_mm         = 0.0f;
        float dy_mm         = 0.0f;
        float dtheta_rad    = 0.0f;

        float dt = (float)dt_us * 1e-6f;
        if(_imu.update())   dtheta_rad = -_imu.getSample().gyroZ_rads * dt; // gyro inversed
        if(_mouse.update())
        {
            dx_mm = _mouse.getMove().dx_mm;
            dy_mm = -_mouse.getMove().dy_mm;      // y inversed
        }

        float midAngle = wrapAngle(_odomPose.theta + 0.5f * dtheta_rad);
        float ct = cosf(midAngle);
        float st = sinf(midAngle);
        _odomPose.x += ct * dx_mm - st * dy_mm;
        _odomPose.y += st * dx_mm + ct * dy_mm;
        _odomPose.theta = wrapAngle(_odomPose.theta + dtheta_rad);
        _odomPose.t_us = micros();
    }
}
// ════════════════════════════════════════════════════════════════
//  update()
// ════════════════════════════════════════════════════════════════
bool Localizer::update()
{
    readSensors();
    bool newScan = _lidar.update(150);  // new scan ?
    
    switch(_state)
    {
        case State::WAIT_SCAN:
        {
            if(_scanAvailable && _nGenerated < N_PARTICLES)
            {
                if(_isLost)
                {
                    /*for(int i=0;i<3;i++) {
                        generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},1200.0f, 1200.0f, 10.0f * DEG_TO_RAD);
                    }
                    for(int i=0;i<2;i++) {
                        generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},2500.0f, 2500.0f, 10.0f * DEG_TO_RAD);
                    }*/
                    generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},6000.0f, 4000.0f, 20.0f * DEG_TO_RAD);
                }
                else
                {
                    generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},25.0f, 25.0f, 1.0f * DEG_TO_RAD);
                    for(int i=0;i<3;i++) {
                        generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},75.0f, 75.0f, 2.0f * DEG_TO_RAD);
                    }
                    for(int i=0;i<2;i++) {
                        generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},200.0f, 200.0f, 5.0f * DEG_TO_RAD);
                    }
                    generateParticle({_poseT0.x,_poseT0.y,_poseT0.theta},500.0f, 500.0f, 10.0f * DEG_TO_RAD);
                }
            }

            if(!newScan) break;
            _scanAvailable = false;
            _loopTimer = 0;
            _poseT1 = _odomPose;
            _state = State::FUSE_POSE;
            break;
        }

        case State::FUSE_POSE:
        {
            fusePose(_mclParticle,{_poseT0.x,_poseT0.y,_poseT0.theta});
            _state = State::CORRECT_SCAN;
            break;
        }

        case State::CORRECT_SCAN:
        {
            const LidarScan &s = _lidar.getScan();
            correctScan(s.pts,s.count,_poseT0,_poseT1);
            teleplot();
            _poseT0 = _poseT1;
            if(_nCorrectedPt > N_MIN_POINTS)
            {
                _scanAvailable = true;
                // prepare generation
                _nGenerated = 0;
                _nConverged = 0;
                _bestScore = 0;
            }    
            _state = State::WAIT_SCAN;
            break;
        }

        case State::ERROR: break;
    }

    return newScan;
}
// ════════════════════════════════════════════════════════════════
//  randn()
// ════════════════════════════════════════════════════════════════
float Localizer::randN()
{
    // Box-Muller : deux U(0,1) → deux N(0,1)
    static bool   has_spare = false;
    static float  spare;
    if (has_spare) { has_spare = false; return spare; }

    float u, v, s;
    do {
        u = random(-10000, 10000) * 0.0001f;  // U(-1,1)
        v = random(-10000, 10000) * 0.0001f;
        s = u*u + v*v;
    } while (s >= 1.0f || s == 0.0f);

    const float mul = sqrtf(-2.0f * logf(s) / s);
    spare     = v * mul;
    has_spare = true;
    return u * mul * 0.167f;
}
// ════════════════════════════════════════════════════════════════
//  generateParticle()
// ════════════════════════════════════════════════════════════════
void Localizer::generateParticle(const Pose2D &p, float dx, float dy, float dth)
{
    if(_nGenerated >= N_PARTICLES-1)    return;
    float px  = p.x     + 0.5f * dx * ((float)(random(-10000,10000))/10000.f); //randN();
    float py  = p.y     + 0.5f * dy * ((float)(random(-10000,10000))/10000.f); //randN();
    if(!insideTable(px,py)) return;

    float pth = wrapAngle(p.theta + 0.5f * dth * ((float)(random(-10000,10000))/10000.f)); //randN());
    _particles[_nGenerated].x       = px;
    _particles[_nGenerated].y       = py;
    _particles[_nGenerated].theta   = pth;

    float score = evaluateParticle(_nGenerated);

    if(score > _bestScore)
    {
        _mclParticle = _particles[_nGenerated];
        _bestScore = score;
    }  
    _nGenerated++;
}
// ════════════════════════════════════════════════════════════════
//  evaluateParticle()
// ════════════════════════════════════════════════════════════════ 
float Localizer::evaluateParticle(uint16_t ind)
{
    float       sumDist     = 0.0f;
    uint16_t    nInliers    = 0;
    uint32_t     wallSeen[MAP_SEG_COUNT]         =   {};

    for(uint16_t i=0; i<_nCorrectedPt; i++)
    {
        float wx = _particles[ind].x + _correctedScan[i].dist * cosf(_particles[ind].theta + _correctedScan[i].ang);
        float wy = _particles[ind].y + _correctedScan[i].dist * sinf(_particles[ind].theta + _correctedScan[i].ang);
        if(!insideTable(wx,wy))  continue;

        SegAssoc sa = nearestSegment(wx, wy, SCORE_INLIER_DIST_MM);
        if(sa.seg < 0)  continue;   // not a wall point

        float d = sa.dist;
        sumDist += d;
        nInliers ++;
        wallSeen[sa.seg]++;
    }

    if(nInliers == 0)   return 0.0f;    // no wall point

    float meanDist = sumDist / (float)nInliers;

    uint8_t nWalls = 0;
    for(uint8_t i=0;i<MAP_SEG_COUNT;i++)
    {
        if(wallSeen[i]>25) nWalls++;
    }

    if(_isLost)
    {
       _particles[ind].score = nInliers - (meanDist * 0.5) * nWalls; 
    }
    else
    {
        if(nInliers > (uint8_t)((float)_nCorrectedPt * 0.4) && nWalls >= 2)
        {
            _particles[ind].score = nInliers - (meanDist * 0.5);
        }
        else
        {
            _particles[ind].score = 0.0f;
        }
    }
   

    return _particles[ind].score;
};   
// ════════════════════════════════════════════════════════════════
//  setPose()
// ════════════════════════════════════════════════════════════════
void Localizer::setPose(float x, float y, float theta)
{
    _odomPose   =   {micros(),x, y, theta};
    _poseT0     =   _odomPose;
    _poseT1     =   _odomPose;
    _state      =   State::WAIT_SCAN;
}
// ════════════════════════════════════════════════════════════════
//  median3()
// ════════════════════════════════════════════════════════════════
inline float Localizer::median3(float a, float b, float c)
{
    float t;

    if(a > b)
    {
        t = a;
        a = b;
        b = t;
    }

    if(b > c)
    {
        t = b;
        b = c;
        c = t;
    }

    if(a > b)
    {
        t = a;
        a = b;
        b = t;
    }

    return b;
}
// ════════════════════════════════════════════════════════════════
//  quantizeAngle()
// ════════════════════════════════════════════════════════════════
inline float Localizer::quantizeAngle(float angle_rad)
{
    float step = ANGLE_QUANT_STEP * DEG_TO_RAD;

    return roundf(angle_rad / step) * step;
}
// ════════════════════════════════════════════════════════════════
//  correctScan()
// ════════════════════════════════════════════════════════════════
void    Localizer::correctScan(const ScanPoint* pts, uint16_t count, const OdomPose &pt0, const OdomPose &pt1)
{
  //const float dt = (_poseT1.t_us - _poseT0.t_us) / (float)count;
  static float filteredRange[LD06_POINT_MAX_SIZE];

  // ---- median filter
  filteredRange[0] = pts[0].dist;
  
  for(uint16_t i=1; i<count-1; i++)
  {
    filteredRange[i] = median3(pts[i-1].dist, pts[i].dist, pts[i+1].dist);
  }
  filteredRange[count-1] = pts[count-1].dist;

  _nCorrectedPt = 0;
  // ------ jump rejection
  for(uint16_t i=1; i<count; i++)
  {
    float r = filteredRange[i];
    float dr = fabsf(r - filteredRange[i-1]);
    if(dr > OUTLIER_JUMP)   continue;
    
    /*
    float ptime = (float)i * dt;
    float dtheta = _poseT1.theta * (_poseT1.t_us - _poseT0.t_us - ptime) * 1e-6f;
    float correctedAngle = pts[i].ang + dtheta;
    correctedAngle = quantizeAngle(correctedAngle);

    float s,c;
    fastSinCos(correctedAngle, &s, &c);
    */
   float s = sinf(pts[i].ang);
   float c = cosf(pts[i].ang);
    _correctedScan[_nCorrectedPt].rx = r * c;
    _correctedScan[_nCorrectedPt].ry = r * s;
    _correctedScan[_nCorrectedPt].dist = r;
    _correctedScan[_nCorrectedPt].ang = pts[i].ang;
    _nCorrectedPt++;
  }
}

// ════════════════════════════════════════════════════════════════
//  interPose()
// ════════════════════════════════════════════════════════════════ 
Pose2D Localizer::interpPose(const Pose2D &a, const Pose2D &b, float t)
{
    return {
        fmaf(t, b.x - a.x, a.x),
        fmaf(t, b.y - a.y, a.y),
        wrapAngle(fmaf(t,wrapAngle(b.theta - a.theta), a.theta))
    };
}
// ════════════════════════════════════════════════════════════════
//  fusePose()
// ════════════════════════════════════════════════════════════════ 
void Localizer::fusePose(const Particle &p, const Pose2D &PoseAtT0)
{
    float yawOffset = wrapAngle(_poseT1.theta - _poseT0.theta);
    float dxOffset = _poseT1.x - _poseT0.x;
    float dyOffset = _poseT1.y - _poseT0.y;

    if(_bestScore > SCORE_MEDIUM)
    {
        _isLost = false;
        _odomPose.x     = _mclParticle.x + dxOffset; 
        _odomPose.y     = _mclParticle.y + dyOffset; 
        _odomPose.theta = wrapAngle(_mclParticle.theta + yawOffset);
        return;
    }
    else
    {
        _isLost = true;
    }
}
// ════════════════════════════════════════════════════════════════
//  teleplot()
// ════════════════════════════════════════════════════════════════ 
void Localizer::teleplot()
{ 
    // Clustered scan
    if(_nCorrectedPt > 0)
    {
        Serial.print(F(">corrected_cloud,loc:"));
        for(uint16_t i=0;i<_nCorrectedPt;i++)
        {
            float px = _odomPose.x + _correctedScan[i].dist * cosf(_odomPose.theta + _correctedScan[i].ang);
            float py = _odomPose.y + _correctedScan[i].dist * sinf(_odomPose.theta + _correctedScan[i].ang);
            Serial.print(String()+px+":"+py+";");
        }               
        Serial.println(F("|xy"));
    }
    
    
    // particles
    Serial.print(F(">particle,loc:"));
    for(uint16_t i=0;i<_nGenerated;i++)
    {
        Serial.print(String()+_particles[i].x+":"+_particles[i].y+";");
    }               
    Serial.println(F("|xy")); 
    
    
    // estimated pose
    Serial.print(F(">odom_pose,loc:"));
    Serial.print(String()+_odomPose.x+":"+_odomPose.y+";");
    Serial.println(F("|xy"));
    
    // walls
    Serial.print(F(">walls,loc:"));
    for(uint8_t i=0; i<MAP_SEG_COUNT; i++)
    {
        Segment const &s = MAP_SEGMENTS[i];
        float incX = s.dx * 0.05f;
        float incY = s.dy * 0.05f;
        for(uint8_t j=0; j<20; j++)
        {
            float px = s.x1 + incX * j;
            float py = s.y1 + incY * j;
            Serial.print(String()+px+":"+py+";");
        }
    }              
    Serial.println(F("|xy"));

    // score
    Serial.print(F(">nparticles:"));
    Serial.println(_nGenerated);

    // score
    Serial.print(F(">score:"));
    Serial.println(_bestScore);

    // angle
    Serial.print(F(">theta_odom, odom:"));
    Serial.println(_odomPose.theta * RAD_TO_DEG);

    Serial.println(String()+"is lost ? : "+_isLost);
}
// ════════════════════════════════════════════════════════════════
//  nearestSegment()
// ════════════════════════════════════════════════════════════════ 
FASTRUN Localizer::SegAssoc Localizer::nearestSegment(float wx, float wy, float maxDist) 
{
    constexpr float T_MIN = 0.04f, T_MAX = 0.96f;
    SegAssoc best = {1.0f + maxDist, -1}; 

    for (uint8_t i = 0; i < MAP_SEG_COUNT; i++) 
    {
        const Segment& s = MAP_SEGMENTS[i];
        
        float t = fmaf(wx - s.x1, s.dx, (wy - s.y1) * s.dy) * s.len_inv;
        if (t < T_MIN || t > T_MAX) continue;
        
        float dist = fabsf(fmaf(s.nx, wx, fmaf(s.ny, wy, s.d)));
        if (dist < best.dist) 
        {
            best.seg = (int8_t)i;
            best.dist   = dist;
        }
    }
    return best;
}
// ════════════════════════════════════════════════════════════════
//  insideTable()
// ════════════════════════════════════════════════════════════════ 
FASTRUN inline bool Localizer::insideTable(float wx, float wy) 
{
    if (wx < -50.f || wx > 3050.f || wy < -50.f || wy > 2050.f) return false;
    if (wx > 650.f && wx < 2350.f && wy > 1600.f) return false;
    return true;
}
// ════════════════════════════════════════════════════════════════
//  init LUT
// ════════════════════════════════════════════════════════════════
void Localizer::initTrigLUT()
{
    for(int i=0;i<LUT_SIZE;i++)
    {
        float angle_deg = i * 0.1f;
        float angle_rad = angle_deg * DEG_TO_RAD;

        _sin_lut[i] = sinf(angle_rad);
        _cos_lut[i] = cosf(angle_rad);
    }
}
// ════════════════════════════════════════════════════════════════
//  FAST SIN / COS
// ════════════════════════════════════════════════════════════════
inline void Localizer::fastSinCos(float angle, float *s, float *c)
{
    float deg = angle * RAD_TO_DEG; 
    int idx = (int)(deg * 10.0f);
    idx %= LUT_SIZE;

    if(idx < 0) idx += LUT_SIZE;

    *s = _sin_lut[idx];
    *c = _cos_lut[idx];
}

