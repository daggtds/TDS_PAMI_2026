#pragma once
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
#include "drivers/lidar.h"
//-----------------------------------------------------------------------------
struct Cell
{
    int16_t ix;
    int16_t iy;
};

struct Node
{
    int16_t ix;
    int16_t iy;
    int16_t parentX;
    int16_t parentY;
    float   f;
    float   g;
    float   h;
    bool    is_open;
    bool    closed;
};
//-----------------------------------------------------------------------------
class PathFinder
{
    public:
        PathFinder()    {};
        
        void            begin(uint8_t teampColor, uint8_t goalZone);
        void            update();
        void            reset();
        Cell*           getPath()           {return _path;};
        uint16_t        getPathLength()     {return _pathLength;};
        Pose2D          getTarget();
        float           getNeighborsCost();
        void            teleplot();
        bool            freshPath()         {return _newPath;};                

    private:
        elapsedMicros   _pathTimer = 0;
        elapsedMicros   _loopTimer = 0;
        Cell            _path[N_CELLS];
        uint16_t        _pathLength = 0; 
        Node*           _openList[N_CELLS];
        uint16_t        _openListLength = 0;
        bool            _newPath = false;
        void            clearPath()     {_pathLength = 0;};
        void            clearOpenList() {_openListLength = 0;};
        
        // obstacles
        void            setForbiddenZone(float xMin, float xMax, float yMin, float yMax);
        void            resetCostMap();
        void            updateCostMap(const Pose2D &p);
        void            setObstacles(const ScanPoint *scan, uint16_t count, const Pose2D &p);
        void            clearMap();
        float           moveCost(const Cell &start, const Cell &end);

        // Goal
        Cell            _goals[N_GOALS];
        uint8_t         _nGoals = 0;
        void            setGoalZone(float gx, float gy);
        int8_t          chooseGoal(Cell c);

        // Conversion Grid <=> XY position
        Cell            xyToCell(const Point2D &p);
        Point2D         cellToXY(const Cell &c);
        bool            isValid(const Cell &c);
        
        // utils
        float           heuristic(const Cell &a, const Cell &b);
        inline int8_t   nodeCompare(Node *a, Node *b);
        void            nodePush(Node *a);
        Node*           nodePop();
};