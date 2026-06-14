#include "pathfinding.h"
#include "localisation.h"
extern Localizer loc;
// ════════════════════════════════════════════════════════════════
DMAMEM  uint8_t _costMap[MAP_INC_X][MAP_INC_Y];
DMAMEM  Node    _table[MAP_INC_X][MAP_INC_Y];
// ════════════════════════════════════════════════════════════════
//  begin()
// ════════════════════════════════════════════════════════════════
void    PathFinder::begin(uint8_t teamColor, uint8_t goalZone)
{
    reset();
    resetCostMap();

    setForbiddenZone(525,2475,1475,2000);                                   // scene
    if(teamColor == TEAM_YELLOW)    setForbiddenZone(2325,3000,1475,2000);  // blue nest   
    if(teamColor == TEAM_BLUE)      setForbiddenZone(0,675,1475,2000);      // yellow nest 

    for(uint8_t i=0; i<N_ZONES; i++)
    {
        float l = zone[i].length * 0.5f;
        if(i == goalZone)
        {
            l *= 0.5f;
            setGoalZone(zone[i].x,zone[i].y);
        } 
        else
        {
            l += 65.0f;
        }  
        setForbiddenZone(zone[i].x-l,zone[i].x+l,zone[i].y-l,zone[i].y+l);  
    }


}
// ════════════════════════════════════════════════════════════════
//  update()
// ════════════════════════════════════════════════════════════════
void    PathFinder::update()
{
    if(_pathTimer >= PATHFINDING_PERIOD_US)
    {
        _newPath = false;
        _pathTimer = 0;
        _loopTimer = 0;
        reset();  
        const Pose2D &p = loc.getPose();
        Serial.println(String()+"pose actuelle : {"+p.x+","+p.y+","+p.theta+"}");
        const ScanPoint *scan = loc.getScan();
        uint16_t count = loc.getScanCount();
        Serial.println(String()+"pt lidar : "+count);
        Cell start = xyToCell({p.x,p.y});

        updateCostMap(p);
        setObstacles(scan,count,p);

        int8_t indGoal = chooseGoal(start);
        if(indGoal < 0) return;
        Cell goal = _goals[indGoal];

        if(!isValid(start) || !isValid(goal))   return;

        Node *s = &_table[start.ix][start.iy];
        Node *g = &_table[goal.ix][goal.iy];

        s->g = 0;
        s->h = heuristic(start,goal);
        s->f = s->g + s->h;
        nodePush(s);

        while(_openListLength > 0)
        {
            Node *c = nodePop();
            if((c->ix == g->ix) && (c->iy == g->iy))
            {
                // path found
                Node *n = g;
                while((n->parentX != -1) && (n->parentY != -1) && _pathLength < N_CELLS)
                {
                    _path[_pathLength++] = {n->ix, n->iy};
                    n = &_table[n->parentX][n->parentY];
                }
                _path[_pathLength++] = {s->ix,s->iy};
                // reverse path
                for(uint16_t i=0; i<(_pathLength >> 1); i++)
                {
                    Cell p = _path[_pathLength - 1 -i];
                    _path[_pathLength - 1 -i] = _path[i];
                    _path[i] = p;
                }
                _newPath = true;
                teleplot();
                return;
            }

            c->closed = true;
            for(uint8_t i=0; i<N_NEIGHBORS; i++)
            {
                int8_t nx = c->ix + dx[i];
                int8_t ny = c->iy + dy[i];
                if(!isValid({nx,ny}))   continue;

                Node *n = &_table[nx][ny];
                if(n->closed)   continue;
                
                float effort = c->g + moveCost({c->ix,c->iy}, {n->ix,n->iy});
                if(effort < n->g)
                {
                    n->parentX  =   c->ix;
                    n->parentY  =   c->iy;
                    n->g        =   effort;
                    n->h        =   heuristic({n->ix,n->iy},{g->ix,g->iy});
                    n->f        =   n->g + n->h;
                    if(!n->is_open) nodePush(n);
                }
            }
        }
    }
}
// ════════════════════════════════════════════════════════════════
//  getTarget()
// ════════════════════════════════════════════════════════════════
Pose2D  PathFinder::getTarget()
{
    uint8_t len;

    if(_pathLength > GOAL_HORIZON)  len = GOAL_HORIZON;
    else                            len = _pathLength - 1;

    Point2D target = cellToXY({_path[len].ix,_path[len].iy});
    const Pose2D &p = loc.getPose();
    
    return {target.x, target.y, p.theta};
}
// ════════════════════════════════════════════════════════════════
//  getNeighborsCost()
// ════════════════════════════════════════════════════════════════
float  PathFinder::getNeighborsCost()
{
    uint16_t cost = 0;
    uint8_t nCell = 0;
    const Pose2D &p = loc.getPose();
    Cell current = xyToCell({p.x,p.y});

    // see 100 mm around (4*25mm)
    for(int16_t i=-4;i<5;i++)
    {
       for(int16_t j=-4;j<5;j++)
        {
            int16_t ix = current.ix + i;
            int16_t iy = current.iy + j;
            if(!isValid({ix,iy})) continue;
            cost += _costMap[ix][iy];
            nCell++;
        }   
    }

    return((float)((float)cost / (float)nCell));
}
// ════════════════════════════════════════════════════════════════
//  reset()
// ════════════════════════════════════════════════════════════════
void    PathFinder::reset()
{
    clearPath();
    clearOpenList();
    clearMap();
}
// ════════════════════════════════════════════════════════════════
//  setForbiddenZone()
// ════════════════════════════════════════════════════════════════
void    PathFinder::setForbiddenZone(float xMin, float xMax, float yMin, float yMax)
{
    Cell start  = xyToCell({xMin,yMin});
    Cell end    = xyToCell({xMax,yMax});

    for(int16_t i=start.ix; i<=end.ix; i++)
    {
        for(int16_t j=start.iy; j<=end.iy; j++)
        {
            if(!isValid({i,j})) continue; 
            _costMap[i][j] = CELL_FORBIDDEN_VALUE;
        }   
    }
}
// ════════════════════════════════════════════════════════════════
//  resetCostMap()
// ════════════════════════════════════════════════════════════════
void    PathFinder::resetCostMap()
{
    memset(_costMap, 0, sizeof(_costMap));
}
// ════════════════════════════════════════════════════════════════
//  updateCostMap()
// ════════════════════════════════════════════════════════════════
void    PathFinder::updateCostMap(const Pose2D &p)
{
    Cell robot = xyToCell({p.x,p.y});
    if(!isValid({robot.ix,robot.iy}))  return;

    for(uint8_t i=0; i<MAP_INC_X; i++)
    {
        for(uint8_t j=0; j<MAP_INC_Y; j++)
        {
            if(!isValid({i,j}))  continue;
            _costMap[i][j] = max(0, _costMap[i][j] - DECREMENT_VALUE);
        }
    }

    for(uint8_t i=-1;i<2;i++)
    {
        for(uint8_t j=-1;j<2;j++)
        {
            int16_t cx = robot.ix + i;
            int16_t cy = robot.iy + j;
            if(isValid({cx,cy}))    _costMap[i][j] = 0;
        }
    }
}
// ════════════════════════════════════════════════════════════════
//  setObstacles()
// ════════════════════════════════════════════════════════════════
void    PathFinder::setObstacles(const ScanPoint *scan, uint16_t count, const Pose2D &p)
{
    float ct = cosf(p.theta);
    float st = sinf(p.theta);
    
    for(uint16_t i=0; i<count; i++)
    {
        if(scan[i].dist > MAX_OBSTACLE_DIST)  continue;
        float px = p.x + ct * scan[i].rx - st * scan[i].ry;
        float py = p.y + st * scan[i].rx + ct * scan[i].ry;

        Cell c = xyToCell({px,py});

        for(int8_t k=-OBSTACLE_MASK_RADIUS; k<OBSTACLE_MASK_RADIUS; k++)
        {
            for(int8_t l=-OBSTACLE_MASK_RADIUS; l<OBSTACLE_MASK_RADIUS; l++)
            {
                int8_t cx = c.ix + k;
                int8_t cy = c.iy + l;

                if(!isValid({cx,cy}))   continue;
                _costMap[cx][cy] = max(_costMap[cx][cy],obstacleMask[k+OBSTACLE_MASK_RADIUS][l+OBSTACLE_MASK_RADIUS]);
            }
        }        
    }
}
// ════════════════════════════════════════════════════════════════
//  clearMap()
// ════════════════════════════════════════════════════════════════
void    PathFinder::clearMap()
{
    for(uint8_t i=0; i<MAP_INC_X; i++)
    {
        for(uint8_t j=0; j<MAP_INC_Y; j++)
        {
            _table[i][j].ix         = i;
            _table[i][j].iy         = j;
            _table[i][j].f          = MAXFLOAT;
            _table[i][j].g          = MAXFLOAT;
            _table[i][j].h          = 0;
            _table[i][j].parentX    = -1;
            _table[i][j].parentY    = -1;
            _table[i][j].is_open    = false;
            _table[i][j].closed     = false;
        }
    }
}
// ════════════════════════════════════════════════════════════════
//  isValid()
// ════════════════════════════════════════════════════════════════
bool    PathFinder::isValid(const Cell &c)
{
    if(c.ix < 0 || c.ix >= MAP_INC_X || c.iy < 0 || c.iy >= MAP_INC_Y)  return false;
    if(_costMap[c.ix][c.iy] == CELL_FORBIDDEN_VALUE)                    return false;
    return true;  
}
// ════════════════════════════════════════════════════════════════
//  moveCost()
// ════════════════════════════════════════════════════════════════
float   PathFinder::moveCost(const Cell &start, const Cell &end)
{
    if(!isValid(start) || !isValid(end))    return MAXFLOAT;
    float cost = ((start.ix != end.ix) && (start.iy != end.iy)) ? MAP_RESOL_MM * M_SQRT2 : MAP_RESOL_MM;
    return cost * (1.0f + _costMap[end.ix][end.iy] * COST_FACTOR);
}
// ════════════════════════════════════════════════════════════════
//  setGoalZone()
// ════════════════════════════════════════════════════════════════
void    PathFinder::setGoalZone(float gx, float gy)
{
    _nGoals = 0;
    for(uint8_t i=0; i<N_GOALS; i++)
    {
        float x = gx + 100.0f * cosf((2.0f * M_PI / (float)N_GOALS) * (float)i);
        float y = gy + 100.0f * sinf((2.0f * M_PI / (float)N_GOALS) * (float)i);
        Cell gc = xyToCell({x,y});

        if(!isValid({gc.ix,gc.iy}))   continue;
        _goals[_nGoals].ix = gc.ix;
        _goals[_nGoals].iy = gc.iy;
        _nGoals++;
    }
}
// ════════════════════════════════════════════════════════════════
//  chooseGoal()
// ════════════════════════════════════════════════════════════════
int8_t  PathFinder::chooseGoal(Cell cp)
{
    int8_t idx = -1;
    
    float minDist = MAXFLOAT;
    for(uint8_t i=0; i<_nGoals; i++)
    {
        if(_costMap[_goals[i].ix][_goals[i].iy] > CELL_MAX_VALUE)   continue;   // goal occupied
        
        float dist = heuristic(_goals[i], cp);
        if(dist < minDist)
        {
            idx = i;
            minDist = dist;
        }
    }
    return idx;
}
// ════════════════════════════════════════════════════════════════
//  xyToCell()
// ════════════════════════════════════════════════════════════════
Cell    PathFinder::xyToCell(const Point2D &p)
{
    Cell c;
    c.ix = (int16_t)(p.x / MAP_RESOL_MM);
    c.iy = (int16_t)(p.y / MAP_RESOL_MM);

    return c;
}
// ════════════════════════════════════════════════════════════════
//  cellToXY()
// ════════════════════════════════════════════════════════════════
Point2D PathFinder::cellToXY(const Cell &c)
{
    Point2D p;

    p.x = (MAP_RESOL_MM * 0.5f) + (float)c.ix * MAP_RESOL_MM;
    p.y = (MAP_RESOL_MM * 0.5f) + (float)c.iy * MAP_RESOL_MM;

    return p;
}
// ════════════════════════════════════════════════════════════════
//  heuristic()
// ════════════════════════════════════════════════════════════════
float PathFinder::heuristic(const Cell &a, const Cell &b)
{
    float dx = fabsf((float)(b.ix - a.ix));
    float dy = fabsf((float)(b.iy - a.iy));
    // Octile distance : diagonal + straight
    static constexpr float D  = MAP_RESOL_MM;
    static constexpr float D2 = MAP_RESOL_MM * 1.41421356f;
    return D * (dx + dy) + (D2 - 2.f * D) * fminf(dx, dy);
}
// ════════════════════════════════════════════════════════════════
//  nodeCompare()
// ════════════════════════════════════════════════════════════════
inline int8_t PathFinder::nodeCompare(Node *a, Node *b)
{
    if(a->f < b->f)   return -1;
    if(a->f > b->f)   return 1;
    return (a->h < b->h) ? -1 : (a->h > b->h) ? 1 : 0;
}
// ════════════════════════════════════════════════════════════════
//  nodePush()
// ════════════════════════════════════════════════════════════════
void    PathFinder::nodePush(Node *a)
{
    uint16_t i = _openListLength++;
    while(i > 0)
    {
        int16_t p = (i - 1) >> 1;
        if(nodeCompare(a, _openList[p]) >= 0)   break;
        _openList[i] = _openList[p];
        i = p;
    }
    _openList[i] = a;
    a->is_open = true;
}
// ════════════════════════════════════════════════════════════════
//  nodePop()
// ════════════════════════════════════════════════════════════════
Node*   PathFinder::nodePop()
{
    if(_openListLength == 0)    return NULL;
    Node* first = _openList[0];
    Node* last  = _openList[--_openListLength];

    uint16_t i = 0;
    while((i << 1) + 1 < _openListLength)
    {
        uint16_t a = (i << 1) + 1;
        uint16_t b = a + 1;
        uint16_t  min_i = (b < _openListLength && nodeCompare(_openList[b],_openList[a]) < 0) ? b : a;
        if(nodeCompare(last, _openList[min_i]) <= 0) break;
        _openList[i] = _openList[min_i];
        i = min_i;
    }
    _openList[i] = last;
    first->is_open = false;
    return first;
}
// ════════════════════════════════════════════════════════════════
//  teleplot()
// ════════════════════════════════════════════════════════════════
void    PathFinder::teleplot()
{
    Serial.print(F(">obstacles,loc:"));
    for(uint8_t i=0;i<MAP_INC_X;i+=2)
    {
        for(uint8_t j=0;j<MAP_INC_Y;j+=2)
        {
            if(_costMap[i][j] < 100) continue;
            Point2D p = cellToXY({i,j});
            Serial.print(String()+p.x+":"+p.y+";");
        }
    }               
    Serial.println(F("|xy"));
    
    if(_newPath)
    {
        Serial.print(F(">path,loc:"));
        for(uint8_t i=0;i<_pathLength;i++)
        {
            Point2D p = cellToXY(_path[i]);
            if(isValid(_path[i])) Serial.print(String()+p.x+":"+p.y+";");
        }               
        Serial.println(F("|xy"));
    }  

    Serial.print(F(">goals,loc:"));
    for(uint8_t i=0;i<_nGoals;i++)
    {
        Point2D p = cellToXY(_goals[i]);
        Serial.print(String()+p.x+":"+p.y+";");
    }               
    Serial.println(F("|xy"));
}
