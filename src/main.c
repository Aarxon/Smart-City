// Smart City Simulator (raylib, C)
//
// Layout:  [ LEFT: layers + live statistics ] [ MAP VIEW (2D / 3D) ] [ RIGHT: trend graphs ]
//
// Controls:
//   1-9 (or click)  Toggle layers: Buildings, Roads, Water, Sewer, Traffic, Congestion, Heat, Pollution, Noise
//   V               Switch 2D / 3D view
//   3D view         WASD / arrows = move, Q/E = rotate, mouse wheel = zoom,
//                   right-mouse drag = orbit/tilt, R = reset camera, Shift = move faster
//   Left click      Start a fire on a building / tree / grass tile
//   SPACE           Trigger a storm
//   P               Pause        + / -   Simulation speed
//   T               Switch between the NORMAL city and the SMART city (also a button, top-left)
//   F               First person: WASD walk, mouse look, Shift sprint, Space jump,
//                   E opens a car door / gets in / gets out; driving: W/S gas+brake, A/D steer, Space handbrake
//   Tab             Smart-city dashboard + digital twin (normal vs smart, measured as you run each mode)
//
// Build (Linux/macOS):   gcc main.c -o city -lraylib -lm
// Build (Windows/MinGW): gcc main.c -o city.exe -lraylib -lopengl32 -lgdi32 -lwinmm

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define TILE        36
#define MAP_W       24
#define MAP_H       16
#define PANEL_W     250
#define VIEW_W      (MAP_W * TILE)
#define VIEW_H      (MAP_H * TILE)
#define SCREEN_W    (VIEW_W + PANEL_W * 2)
#define SCREEN_H    (VIEW_H + 64)       // map view + a status strip underneath

#define SIDEWALK_W  10         // pixels (2D); scaled to tile units in 3D (the strip beside each road)
#define LANE_CAR    0.10f      // cars drive this far either side of the road centre (left-hand traffic)
#define LANE_BIKE   0.17f      // cyclists ride just inside the parked cars, nearer the kerb
#define BLD_W       0.94f     // generic building footprint (tile units; was 0.85)
#define BLD_PODIUM  0.92f     // skyrise podium
#define BLD_TOWER   0.74f     // skyrise tower
#define ROAD_INSET  0.08f     // the road surface stops this far short of a sidewalk; the sidewalk strip takes that space
#define SIDE_IN     (SIDEWALK_W / (float)TILE - ROAD_INSET)   // buildings, houses and fences stay this far in from a road-side tile edge
#define BLD_MARGIN  0.03f     // ...and this far in from the other tile edges
#define LINE_THICK  3
#define HIST        120        // graph samples (1 per second => 2 minutes)
#define DAY_LEN     120.0f     // real seconds per simulated day
#define SIG_CYCLE   24.0f      // traffic light cycle (seconds)

#define MAX_CARS    90
#define MAX_PEDS    64
#define MAX_TRUCKS  6
#define MAX_POLICE  6
#define MAX_AMB     3
#define MAX_DRONES  20
#define FARM_W      6          // width (tiles) of the renewable-energy strip east of the map (smart city)
#define MAX_STN     4
#define MAX_PART    900
#define MAX_PATH    (MAP_W * MAP_H)

#define PLANT_KW    12000.0f   // output of one power plant
#define WORKS_LPM   9000.0f    // output of one water works (litres/min)
#define SEWER_CAP   8500.0f    // sewer capacity (litres/min)

#define LAYER_Y0    84
#define LAYER_H     12
#define ROW_H       12

typedef enum {
    T_GRASS = 0, T_ROAD, T_BUILDING, T_TREE, T_RUIN,
    T_FIRE_STN, T_POLICE_STN, T_POWER, T_WATER,
    T_HOUSE, T_MALL, T_PARK, T_LANDFILL, T_HOSPITAL
} TileType;

enum { B_RES = 0, B_COM, B_IND };
enum { Z_NONE = 0, Z_GATED1, Z_GATED2, Z_GATED3, Z_INDUSTRIAL, Z_LANDFILL, Z_MALL };
enum { C_NONE = 0, C_SIGNAL, C_ROUNDABOUT, C_STOP };
enum { L_BUILDINGS = 0, L_ROADS, L_WATER, L_SEWER, L_TRAFFIC, L_CONGESTION, L_HEAT, L_POLLUTION, L_NOISE, L_COUNT };
enum { G_ENERGY = 0, G_WATER, G_SEWER, G_AQI, G_TEMP, G_RAIN, G_CARS, G_COUNT };
typedef enum { VIEW_2D = 0, VIEW_3D, VIEW_FP } ViewMode;
enum { V_SALOON = 0, V_HATCH, V_ESTATE, V_PICKUP, V_SUV, V_VAN, V_TRUCK, V_BIKE, V_COUNT };

typedef struct {
    float x, y;                 // position in tile units
    float yaw;                  // heading in degrees (local +Z forward)
    int gx, gy, tx, ty, dir;    // from-tile, to-tile, heading (0 up,1 down,2 left,3 right)
    float t, ox, oy, wait, speedMul, stopT, speedNow, reportT, lostT;
    float arcU, arcFrom, arcSweep;
    int claimX, claimY, id, reportKind, pursuer;
    int destX, destY;           // smart routing destination (road tile), -1 = none
    bool shuttle;               // autonomous shuttle on its fixed loop (smart city)
    float shDwell;              // seconds left at a shuttle stop
    int shLeg, shPax;           // next loop waypoint, passengers on board
    unsigned char type, colIdx;
    bool active, rogue, speeder, wanted, fleeing, arcing, braking;
    Color col;
} Car;

typedef struct {
    float x, y;
    float wx[3], wy[3];         // waypoints for the current move
    int wn, wi;                 // waypoint count / index
    int gx, gy, px, py;         // current node, previous node
    int cx, cy, fx, fy;         // crossing tile and far node (when crossing)
    bool crossing, started;
    bool ill, illCalled;        // medical emergency / help already on its way
    float illT, illSince;
    float robTimer, cooldown, phase, wait;
    bool active, criminal, robbing;
    unsigned char look;
    Color shirt, pants;
} Ped;

typedef struct {
    bool active, arrived, spraying;
    float x, y, angle, t, work, sx, sy, scanT, lostT;
    int path[MAX_PATH];
    int len, idx, targetX, targetY, pedTarget;
    int mode;                   // police: 0 = crime scene, 1 = search for car, 2 = pursuit
    int carId;
} Responder;

typedef struct { int x, y, rx, ry; } Station;

// Smart-city emergency drones (fly straight, ignore roads)
enum { D_POLICE = 0, D_FIRE, D_MEDIC, D_PARCEL, D_MEDSUP };
typedef struct {
    bool active, carrying, spraying;
    int kind, state;            // state: 0 outbound, 1 working, 2 carrying patient, 3 returning
    float x, y, z;              // tile units + altitude
    float tx, ty;               // current flight target
    float yaw, work, scanT, lostT, since;
    int pedTarget, carId, homeX, homeY;
    float sx, sy;               // spray / beam target
} Drone;

typedef struct {
    Vector3 p, v;
    float life, max;
    unsigned char kind;         // 0 flame, 1 smoke, 2 ember
    bool on;
} Particle;

typedef struct {
    float hour, daylight, ambient;
    int   day, population, buildings, damaged, vehicles, jammed, fires;
    float energyDemand, energyCap, energyPct, powerLoad;
    float waterDemand, waterCap, waterPct;
    float sewerLoad, sewerPct;
    float aqi, heatIsland, temp, rainMm, green, security;
} Stats;

typedef struct { Vector3 target; float yaw, pitch, dist; } OrbitCam;

// ---------------------------------------------------------------------------
// Colours
// ---------------------------------------------------------------------------
static const Color COL_GRASS       = {  86, 160,  70, 255 };
static const Color COL_SOIL        = {  70,  52,  42, 255 };
static const Color COL_ROAD        = {  98,  98, 104, 255 };
static const Color COL_ROAD_DIM    = {  60,  60,  66, 255 };
static const Color COL_SIDEWALK    = { 222, 205, 165, 255 };
static const Color COL_SIDEWALK_DIM= { 120, 110,  90, 255 };
static const Color COL_LINE        = { 255, 255, 255, 255 };
static const Color COL_DAMAGED     = { 110,  70,  60, 255 };
static const Color COL_RUIN        = {  65,  60,  60, 255 };
static const Color COL_TREE        = {  40, 130,  50, 255 };
static const Color COL_FIRE        = { 245,  90,  35, 255 };
static const Color COL_FIRE_STN    = { 200,  50,  50, 255 };
static const Color COL_POLICE_STN  = {  40,  80, 200, 255 };
static const Color COL_POWER       = {  70,  70,  78, 255 };
static const Color COL_WORKS       = {  60, 150, 170, 255 };
static const Color COL_WATER_PIPE  = {  70, 150, 255, 255 };
static const Color COL_SEWER_PIPE  = { 150, 100,  50, 255 };
static const Color COL_PANEL       = {  28,  31,  38, 255 };
static const Color COL_TRUCK       = { 235,  70,  40, 255 };
static const Color COL_POLICE_CAR  = {  60, 110, 235, 255 };
static const Color COL_CIVILIAN    = { 240, 215, 170, 255 };
static const Color COL_CRIMINAL    = { 190,  50, 190, 255 };
static const Color COL_LABEL       = { 170, 175, 185, 255 };

static const Color COL_BTYPE[3] = {
    { 185, 120,  98, 255 },   // residential
    {  96, 140, 176, 255 },   // commercial
    { 150, 150, 128, 255 }    // industrial
};
#define N_CARCOL 10
static const Color CAR_COLS[N_CARCOL] = {
    { 230, 210,  50, 255 }, { 235, 235, 235, 255 }, {  70, 170, 230, 255 }, { 150, 220, 100, 255 },
    { 220, 120, 190, 255 }, { 240, 150,  60, 255 }, {  40,  40,  48, 255 }, { 170,  30,  40, 255 },
    { 120, 125, 135, 255 }, {  30,  70, 140, 255 }
};
static const Color SHIRT_COLS[8] = {
    { 200, 60, 60, 255 }, { 60, 110, 200, 255 }, { 230, 200, 70, 255 }, { 80, 170, 90, 255 },
    { 240, 240, 240, 255 }, { 170, 90, 190, 255 }, { 240, 140, 50, 255 }, { 60, 60, 70, 255 }
};
static const char *LAYER_NAMES[L_COUNT] = {
    "Buildings", "Roads & sidewalks", "Water network", "Sewer network",
    "Traffic", "Congestion", "Heat map", "Pollution map", "Noise map"
};
static const Color LAYER_COL[L_COUNT] = {
    { 185, 120,  98, 255 }, { 150, 150, 155, 255 }, {  70, 150, 255, 255 }, { 170, 115,  55, 255 },
    { 240, 210,  50, 255 }, {  90, 220, 140, 255 }, { 235,  80,  50, 255 }, { 170,  60, 170, 255 },
    { 120, 170, 230, 255 }
};

// ---------------------------------------------------------------------------
// World state
// ---------------------------------------------------------------------------
static int           surface[MAP_H][MAP_W];
static unsigned char bType[MAP_H][MAP_W];
static float         bHeight[MAP_H][MAP_W];
static float         health[MAP_H][MAP_W];
static float         fireI[MAP_H][MAP_W];
static bool          pipeNode[MAP_H][MAP_W];
static float         heatMap[MAP_H][MAP_W];
static float         polMap[MAP_H][MAP_W];
static float         traffic[MAP_H][MAP_W];

static unsigned char zone[MAP_H][MAP_W];       // Z_* area id
static unsigned char faceDir[MAP_H][MAP_W];    // direction a house / shop faces
static unsigned char vr[MAP_H][MAP_W];         // random look bits
static unsigned char deco[MAP_H][MAP_W];       // 0 none, 1 shrubs, 2 hedge, 3 flowers
static unsigned char paved[MAP_H][MAP_W];      // private lane / yard
static bool          gateTile[MAP_H][MAP_W];   // entrance gap in a zone fence
static unsigned char drive[MAP_H][MAP_W];      // driveway car (model + 1)
static unsigned char drvCol[MAP_H][MAP_W];
static unsigned char lot[MAP_H][MAP_W][8];     // parking bays (model + 1)
static unsigned char lotCol[MAP_H][MAP_W][8];

static unsigned char ctrl[MAP_H][MAP_W];       // junction control (C_*)
static float         sigOff[MAP_H][MAP_W];
static unsigned char crossT[MAP_H][MAP_W];     // 1 = peds cross E-W (road runs N-S), 2 = peds cross N-S
static int           crossI[MAP_H][MAP_W];     // junction index the crossing belongs to (or -1)
static unsigned char nodeOcc[MAP_H][MAP_W];    // cars holding a roundabout / stop junction
static unsigned char crossBusy[MAP_H][MAP_W];  // pedestrians on a crossing right now
static bool          pedNode[MAP_H][MAP_W];    // sidewalk tile peds may stand on

static Car        cars[MAX_CARS];
static Ped        peds[MAX_PEDS];
static Responder  trucks[MAX_TRUCKS];
static Responder  police[MAX_POLICE];
static Station    fireStn[MAX_STN], policeStn[MAX_STN], hospStn[MAX_STN];
static int        nFire = 0, nPolice = 0, nHosp = 0;
static Responder  ambs[MAX_AMB];
static Drone      drones[MAX_DRONES];
static Particle   parts[MAX_PART];
static int        carSerial = 1;

static Stats st;
static float simTime = 0.0f, simSpeed = 1.0f;
static bool  paused = false;
static float rain = 0.0f, rainTimer = 0.0f, rainPeak = 1.0f, nextRain = 60.0f;
static float floodLevel = 0.0f;
static float fireTimer = 0.0f, dispatchTimer = 0.0f, mapTimer = 0.0f, statTimer = 0.0f,
             histTimer = 0.0f, birthTimer = 0.0f, parkTimer = 0.0f;
static int   crimes = 0, arrests = 0, collapses = 0, births = 0, deaths = 0, robbingNow = 0;
static int   offences = 0, reports = 0, caught = 0, escaped = 0, nBikes = 0, nPedsActive = 0;
static float landfillFill = 22.0f;      // % of landfill capacity used

// ---- smart city mode ----------------------------------------------------------
static bool  smartCity = false;
static float roofSolarKW = 0, farmSolarKW = 0, windKW = 0, gridImportKW = 0, batteryPct = 55.0f;
static float windSpeed = 0.5f, windTarget = 0.5f, windTimer = 0.0f, renewPct = 0.0f;
static int   solarRoofs = 0;
static float recycledT = 0.0f, wasteT = 0.0f;       // cumulative tonnes recycled / sent to landfill
static int   droneSorties = 0, reroutes = 0;
static int   medCalls = 0, medSaved = 0, medLost = 0, respN = 0, transN = 0;
static float respSum = 0.0f, transSum = 0.0f, medTimer = 8.0f, avgSpeed = 0.0f;
static bool  monitorTile[MAP_H][MAP_W];             // smart traffic monitor masts
static float hospTimer = 0.0f;

static float hist[G_COUNT][HIST];
static int   histIdx = 0;
static bool  histInit = false;

static bool     layerOn[L_COUNT] = { true, true, false, false, true, true, false, false, false };
static ViewMode viewMode = VIEW_2D;
static OrbitCam oc;
static bool hoverOK = false;
static int  hoverX = 0, hoverY = 0;
static float gAlpha = 1.0f;     // alpha used for structures (see-through in X-ray mode)

static const int DX[4] = { 0, 0, -1, 1 };
static const int DY[4] = { -1, 1, 0, 0 };

// vehicle dimensions in tile units (length along heading)
static const float VLEN[V_COUNT] = { 0.26f, 0.22f, 0.27f, 0.28f, 0.26f, 0.30f, 0.44f, 0.13f };
static const float VWID[V_COUNT] = { 0.115f, 0.11f, 0.115f, 0.12f, 0.125f, 0.125f, 0.14f, 0.05f };

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static float Rf(void) { return GetRandomValue(0, 10000) / 10000.0f; }
static bool InBounds(int x, int y) { return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H; }
static bool IsRoad(int x, int y) { return InBounds(x, y) && surface[y][x] == T_ROAD; }
static bool ZoneFenced(int z) { return z >= Z_GATED1 && z <= Z_LANDFILL; }
static bool IsBldg(int s) { return s == T_BUILDING || s == T_HOUSE || s == T_MALL || s == T_HOSPITAL; }
static bool Flammable(int x, int y) {
    int s = surface[y][x];
    return IsBldg(s) || s == T_TREE || s == T_GRASS;
}

static Color LerpColor(Color a, Color b, float t) {
    t = Clamp(t, 0.0f, 1.0f);
    return (Color){ (unsigned char)Lerp(a.r, b.r, t), (unsigned char)Lerp(a.g, b.g, t),
                    (unsigned char)Lerp(a.b, b.b, t), (unsigned char)Lerp(a.a, b.a, t) };
}
static Color Ramp3(Color a, Color b, Color c, float t) {
    t = Clamp(t, 0.0f, 1.0f);
    return (t < 0.5f) ? LerpColor(a, b, t * 2.0f) : LerpColor(b, c, (t - 0.5f) * 2.0f);
}
static Color BlendOver(Color base, Color over) {
    float a = over.a / 255.0f;
    return (Color){ (unsigned char)(base.r * (1 - a) + over.r * a),
                    (unsigned char)(base.g * (1 - a) + over.g * a),
                    (unsigned char)(base.b * (1 - a) + over.b * a), 255 };
}
static Color Shade(Color c, float k) {      // multiply brightness
    return (Color){ (unsigned char)Clamp(c.r * k, 0, 255), (unsigned char)Clamp(c.g * k, 0, 255),
                    (unsigned char)Clamp(c.b * k, 0, 255), c.a };
}

static float DayFactor(void) {     // energy / water demand multiplier, peaks early evening
    return 0.65f + 0.35f * sinf((st.hour - 12.0f) / 24.0f * 2.0f * PI);
}
static float TrafficFactor(void) { // morning + evening rush hours
    float h = st.hour;
    float a = expf(-(h - 8.5f) * (h - 8.5f) / 3.0f);
    float b = expf(-(h - 17.5f) * (h - 17.5f) / 4.0f);
    return 0.4f + 0.6f * fmaxf(a, b);
}
static float ShopFactor(void) {    // mall busy 10:00 - 20:00
    float h = st.hour;
    return Clamp(1.0f - fabsf(h - 15.0f) / 7.0f, 0.1f, 1.0f);
}

// 0 = none, 1 = horizontal, 2 = vertical (centre dashes only on straight sections)
static int RoadLine(int x, int y) {
    bool h = IsRoad(x - 1, y) || IsRoad(x + 1, y);
    bool v = IsRoad(x, y - 1) || IsRoad(x, y + 1);
    if (h && !v) return 1;
    if (v && !h) return 2;
    return 0;
}
static int RoadNeighbours(int x, int y) {
    return IsRoad(x - 1, y) + IsRoad(x + 1, y) + IsRoad(x, y - 1) + IsRoad(x, y + 1);
}

// Sidewalk pieces for a non-road tile, in tile units (0..1). Used by both 2D and 3D.
static int SidewalkRects(int x, int y, Rectangle *r) {
    // The strip is SIDEWALK_W wide and reaches ROAD_INSET into the road, which is narrowed to match.
    float w = SIDEWALK_W / (float)TILE, d = ROAD_INSET, in = SIDE_IN;
    int n = 0;
    bool up = IsRoad(x, y - 1), down = IsRoad(x, y + 1), left = IsRoad(x - 1, y), right = IsRoad(x + 1, y);
    if (up)    r[n++] = (Rectangle){ 0, -d, 1, w };
    if (down)  r[n++] = (Rectangle){ 0, 1 - in, 1, w };
    if (left)  r[n++] = (Rectangle){ -d, 0, w, 1 };
    if (right) r[n++] = (Rectangle){ 1 - in, 0, w, 1 };
    if (!up && !left && IsRoad(x - 1, y - 1))    r[n++] = (Rectangle){ -d, -d, w, w };
    if (!up && !right && IsRoad(x + 1, y - 1))   r[n++] = (Rectangle){ 1 - in, -d, w, w };
    if (!down && !left && IsRoad(x - 1, y + 1))  r[n++] = (Rectangle){ -d, 1 - in, w, w };
    if (!down && !right && IsRoad(x + 1, y + 1)) r[n++] = (Rectangle){ 1 - in, 1 - in, w, w };
    return n;
}

// ---- Traffic signals (phase machine, see UpdateSignals): 0 = red, 1 = green, 2 = yellow.
// axis 0 = N-S traffic, 1 = E-W traffic.  Phases: 0 A green, 1 A yellow, 2 all red, 3 B green, 4 B yellow, 5 all red.
static unsigned char sigPhase[MAP_H][MAP_W];
static float         sigT[MAP_H][MAP_W];
static float SigMinDur(int ph) { return (ph == 0 || ph == 3) ? (smartCity ? 5.0f : 9.0f) : ((ph == 1 || ph == 4) ? 2.0f : 1.0f); }

// redLeft = a lower bound on the seconds until this axis turns green (0 when it is not red)
static int SigState(int x, int y, int axis, float *redLeft) {
    int ph = sigPhase[y][x], gp = axis ? 3 : 0, yp = gp + 1;
    int s = (ph == gp) ? 1 : (ph == yp ? 2 : 0);
    if (redLeft) {
        float left = 0.0f;
        if (s == 0) {
            for (int p = ph; p != gp; p = (p + 1) % 6) {
                float d = SigMinDur(p);
                left += (p == ph) ? fmaxf(d - sigT[y][x], 0.0f) : d;
            }
        }
        *redLeft = left;
    }
    return s;
}

// ---------------------------------------------------------------------------
// Smart city extras: shared state (simulation, 2D / 3D views and the dashboard)
//   lighting, EV + V2G, smart bins, leak detection + rain harvesting, signal pre-emption,
//   air-quality sensors, flood sensors + storm drains, parking guidance, district heating,
//   autonomous shuttles, delivery drones, green roofs, digital-twin recorder
// ---------------------------------------------------------------------------
#define DRONE_RAIN    0.60f      // drones are grounded when the rain is heavier than this
#define MAX_BINS      14
#define MAX_LEAKS     5
#define N_AQ          6
#define N_FLOOD       6
#define N_STOPS       6
#define N_DROPS       8
#define DEPOT_X       18         // refuse depot (road beside the landfill gate)
#define DEPOT_Y       11
#define HUB_X         15         // drone fulfilment hub (loading yard)
#define HUB_Y         11
#define HEATPLANT_X   14         // district-heating heat exchanger (loading yard)
#define HEATPLANT_Y   11
#define EV_KWS_EACH   1500.0f    // V2G energy pool per parked EV (kW-seconds, same scale as the battery bank)
#define EV_KW_EACH    7.0f       // charge / discharge power per parked EV
#define RAIN_TANK_CAP 60.0f      // kL
#define RETENTION_CAP 120.0f     // kL

typedef struct { int x, y, side; float fill, rate, overflowT; } Bin;
typedef struct { bool active, detected; int x, y; float age, rate, detectAt, fixT; } Leak;
typedef struct { float x, y, t; int kind; } DropMark;

// ---- street lighting --------------------------------------------------------------
static float lampLvl[MAP_H][MAP_W];
static bool  lampTile[MAP_H][MAP_W];
static float lampAvg = 1.0f, lightNowKW = 0.0f, lightBaseKW = 0.0f;
static int   nLamps = 0;

// ---- EV charging / V2G -------------------------------------------------------------
static int   evCount = 0, parkedCars = 0;
static float evSoc = 70.0f, v2gKW = 0.0f, evKWhIn = 0.0f, evKWhOut = 0.0f;

// ---- smart bins + refuse truck -----------------------------------------------------
static Bin       bins[MAX_BINS];
static int       nBins = 0;
static Responder binTruck;
static int       binQ[MAX_BINS], binQn = 0, binQi = 0;
static float     binSched = 60.0f, binCool = 0.0f, binOverflowS = 0.0f, binFillSum = 0.0f;
static int       binTrips = 0, binTiles = 0, binEmptied = 0;

// ---- water: leaks, rain harvesting, storm drains, flood sensors ---------------------
static Leak  leaks[MAX_LEAKS];
static float leakTimer = 18.0f, leakLpm = 0.0f, leakLostKL = 0.0f, leakAgeSum = 0.0f;
static int   leaksTotal = 0, leaksFixed = 0;
static float rainTankKL = 25.0f, harvestLpm = 0.0f, harvestedKL = 0.0f, runoffK = 1.0f;
static float retentionKL = 0.0f, drainOpen = 0.0f;
static const int FLOOD_POS[N_FLOOD][2] = { {2,3}, {8,9}, {16,13}, {4,12}, {18,7}, {11,15} };
static int   floodState[N_FLOOD];        // 0 dry, 1 warning (gates opening), 2 flooding

// ---- emergency-vehicle signal pre-emption -----------------------------------------
static unsigned char sigPre[MAP_H][MAP_W];   // 0 none, 1 = N-S wanted green, 2 = E-W wanted green
static int   preemptWins = 0, redRuns = 0, preemptCalls = 0;

// ---- air-quality sensor network ----------------------------------------------------
static const int   AQ_POS[N_AQ][2] = { {6,3}, {11,5}, {20,3}, {4,12}, {13,13}, {21,13} };
static const char *AQ_NAME[N_AQ] = { "Hospital Qtr", "Downtown", "Mall District", "Gated Homes", "Industrial", "Recycling" };
static float aqLevel[N_AQ];
static int   aqWorst = 0;

// ---- parking guidance ---------------------------------------------------------------
static int   parkFree = 0, parkTotal = 0;
static float parkSearch = 0.0f;
static const int PSIGN[2][2] = { {19,8}, {23,8} };

// ---- district heating -------------------------------------------------------------
#define N_DH 3
static const float DH_SEG[N_DH][4] = {          // x0, y0, x1, y1 in tile units (run along the sidewalks)
    { 17.9f, 10.04f, 0.2f, 10.04f },
    { 10.97f, 10.04f, 10.97f, 4.06f },
    { 10.97f, 8.97f, 17.9f, 8.97f }
};
static bool  dhServed[MAP_H][MAP_W];
static int   dhBuildings = 0;
static float wasteHeatKW = 0.0f, dhKW = 0.0f, heatDemandKW = 0.0f, heatCover = 0.0f;

// ---- autonomous shuttles -------------------------------------------------------------
static const int SH_WP[4][2] = { {18,3}, {18,9}, {4,9}, {4,3} };
static const int SH_STOP[N_STOPS][2] = { {7,3}, {14,3}, {18,5}, {14,9}, {7,9}, {4,7} };
static int   shuttleBoard = 0, shuttleStops = 0, shuttlePax = 0, modalCars = 0;

// ---- delivery drones -----------------------------------------------------------------
static float parcelTimer = 6.0f, medSupTimer = 28.0f;
static int   parcelsDelivered = 0, medDeliveries = 0;
static DropMark drops[N_DROPS];

// ---- green roofs ------------------------------------------------------------------------
static int   greenRoofs = 0;

// ---- city budget (KPI): running balance and today's rates, in k$ --------------------------------
// Reported only. Nothing is blocked or switched off when the balance is negative.
static float budBalanceK = 0.0f;        // net since the city was generated
static float budIncomeK = 0.0f, budSpendK = 0.0f;   // income and spending, k$ per simulated day

// ---- citizen happiness (KPI, 0-100): reported only, nothing in the simulation reads it back ----
static float happyNow = 0.0f;           // current score
static int   happyLow = 0;              // index of the weakest measure (HAP_NAME in p6g_happy.c)

// ---- jobs and employment (KPI): reported only ---------------------------------------------
static float jobsNow = 0.0f, labourNow = 0.0f, employedNow = 0.0f, employRate = 0.0f;

// ---- carbon and noise (KPIs) and the noise map: reported only --------------------------------
static float carbonPowerTph = 0.0f, carbonTrafficTph = 0.0f, carbonPerPerson = 0.0f;   // t CO2 per hour, t per resident per year
static float evShare = 0.0f;                                                             // share of cars that are electric (smart city)
static float noiseMap[MAP_H][MAP_W];                                                     // dB at each tile
static float noiseAvg = 0.0f, noiseLoudPct = 0.0f;

// ---- digital-twin recorder: time-weighted averages kept separately for each mode ----------------
enum { TW_AQI = 0, TW_FOSSIL, TW_CO2, TW_DEMAND, TW_LIGHT, TW_SPEED, TW_STOP, TW_LEAK, TW_BUDGET, TW_HAPPY, TW_EMPLOY, TW_CARBON_PP, TW_NOISE, TW_POP, TW_COUNT };
static double twSum[2][TW_COUNT], twTime[2];
static double twNight[2];
static float demandBaseKW = 1.0f;
static double twRespSum[2], twRespN[2], twTransSum[2], twTransN[2], twLost[2], twRecycle[2], twLandfill[2];
static float  twLastResp = 0, twLastRespN = 0, twLastTrans = 0, twLastTransN = 0, twLastLost = 0, twLastRec = 0, twLastLand = 0;
static bool   showDash = false;

// ---------------------------------------------------------------------------
// helpers shared by sim + both views
// ---------------------------------------------------------------------------
static int LampDir(int x, int y) {                 // direction of the road a lamp / bin faces (-1 = none)
    for (int d = 0; d < 4; d++) if (IsRoad(x + DX[d], y + DY[d])) return d;
    return -1;
}

static bool IsGabledRes(int x, int y) {            // low residential block with a pitched roof (see DrawBuilding3D)
    return bType[y][x] == B_RES && (int)(bHeight[y][x] * 10.0f + 0.5f) <= 4 && (vr[y][x] & 1);
}

// smart city: some low flat roofs carry a living sedum blanket under the solar panels
static bool GreenRoof(int x, int y) {
    if (!smartCity || health[y][x] < 50.0f) return false;
    int s = surface[y][x];
    if (s == T_BUILDING) return bType[y][x] != B_IND && bHeight[y][x] < 1.0f && !IsGabledRes(x, y) && (vr[y][x] % 100) < 62;
    if (s == T_MALL)     return true;
    if (s == T_HOUSE)    return ((vr[y][x] >> 1) % 4) == 3;
    return false;
}

static Color AqColor(float aqi) {
    if (aqi < 45.0f)  return (Color){ 70, 215, 110, 255 };
    if (aqi < 75.0f)  return (Color){ 150, 220, 80, 255 };
    if (aqi < 100.0f) return (Color){ 245, 210, 60, 255 };
    if (aqi < 130.0f) return (Color){ 245, 140, 50, 255 };
    return (Color){ 230, 60, 70, 255 };
}

static float DistToSeg(float px, float py, float x0, float y0, float x1, float y1) {
    float dx = x1 - x0, dy = y1 - y0, l2 = dx * dx + dy * dy;
    float t = (l2 > 0.0f) ? Clamp(((px - x0) * dx + (py - y0) * dy) / l2, 0.0f, 1.0f) : 0.0f;
    float qx = x0 + t * dx - px, qy = y0 + t * dy - py;
    return sqrtf(qx * qx + qy * qy);
}

// ---------------------------------------------------------------------------
// First-person mode: shared state (walk the streets, open car doors, drive)
//   F = enter / leave first person.  Everything here is state + small helpers used by the
//   traffic code; the logic is in p6e_fp.c and the drawing in p9f_fp3d.c.
// ---------------------------------------------------------------------------
#define MAX_UCARS    28            // parked / player-driven cars (separate from the AI traffic)
#define FP_EYE       0.125f        // eye height above the pavement (a pedestrian is about 0.145 tall)
#define FP_RADIUS    0.032f        // body radius for walking collisions
#define GROUND_TOP   0.03f     // the one flat level: grass, roads and car parks all sit on it
#define WALK_TOP     0.032f    // sidewalks, drives and paths sit just above it; people and vehicles stand on this
#define ENT_Y        WALK_TOP

typedef struct {
    bool  active;
    int   type, colIdx;
    float x, z, yaw, v, steer, gy; // position (tile units), heading in degrees (same convention as Car), speed, steering, ground height
    float door[2];                 // door opening angle in degrees: [0] driver (right side), [1] passenger (left side)
    float doorTgt[2];
    bool  braking;
} UCar;

enum { FP_WALK = 0, FP_ENTER, FP_DRIVE, FP_EXIT };

typedef struct {
    bool  valid;
    float x, z, yaw, pitch;        // yaw in radians, forward = (sin yaw, cos yaw) on the ground plane
    float gy, hop, vy, bob, bobT, speed;
    float cx, cy, cz, cyaw, cpitch;   // camera pose actually used for rendering
    int   state, car, door;
    float st;                      // 0..1 transition clock for entering / leaving a car
    float sx, sy, sz, syaw, spitch;// camera pose at the start of a transition
    float lookYaw, lookPitch, lookIdle;
    char  prompt[56];
    float bump, hint;
} Fp;

#ifdef FP_TEST_KEYS                       // headless tests drive the player through these arrays instead of the keyboard
static bool fpTestDown[512], fpTestPress[512];
#define FP_DOWN(k)    fpTestDown[(k)]
#define FP_PRESSED(k) fpTestPress[(k)]
#else
#define FP_DOWN(k)    IsKeyDown(k)
#define FP_PRESSED(k) IsKeyPressed(k)
#endif

static UCar ucars[MAX_UCARS];
static Fp   fp;

// seat geometry per vehicle type: z centre of the cabin, cabin length, roof height
static const float UC_CABIN[V_COUNT][3] = {
    { -0.015f, 0.130f, 0.110f }, { -0.025f, 0.125f, 0.110f }, { -0.035f, 0.185f, 0.114f },
    {  0.045f, 0.085f, 0.112f }, { -0.015f, 0.170f, 0.128f }, { -0.025f, 0.250f, 0.130f },
    {  0.000f, 0.100f, 0.100f }, {  0.000f, 0.000f, 0.100f }
};

static bool FpActive(void) { return viewMode == VIEW_FP; }

// exact overlap test for two vehicles (oriented rectangles on the ground plane, yaw in degrees, forward = (sin yaw, cos yaw))
static bool VehOverlap(float x1, float z1, float yaw1, float L1, float W1, float x2, float z2, float yaw2, float L2, float W2) {
    const float m = 0.004f;                                        // small clearance
    float t1 = yaw1 * DEG2RAD, t2 = yaw2 * DEG2RAD;
    float fx[2] = { sinf(t1), sinf(t2) }, fz[2] = { cosf(t1), cosf(t2) };
    float halfL[2] = { 0.5f * L1, 0.5f * L2 }, halfW[2] = { 0.5f * W1, 0.5f * W2 };
    float dx = x2 - x1, dz = z2 - z1;
    for (int a = 0; a < 4; a++) {                                  // separating axes: forward and right of each box
        int b = a >> 1;
        float nx = (a & 1) ? fz[b] : fx[b], nz = (a & 1) ? -fx[b] : fz[b];
        float dist = fabsf(dx * nx + dz * nz);
        float r1 = halfL[0] * fabsf(fx[0] * nx + fz[0] * nz) + halfW[0] * fabsf(fz[0] * nx - fx[0] * nz);
        float r2 = halfL[1] * fabsf(fx[1] * nx + fz[1] * nz) + halfW[1] * fabsf(fz[1] * nx - fx[1] * nz);
        if (dist > r1 + r2 + m) return false;
    }
    return true;
}


// ---------------------------------------------------------------------------
// Procedural textures (no image files needed) + 3D primitive helpers
// ---------------------------------------------------------------------------
enum {
    TX_BRICK = 0, TX_SIDING, TX_STUCCO, TX_ROOF, TX_GRASS, TX_ASPHALT, TX_CONCRETE, TX_CORRUG,
    TX_BARK, TX_HEDGE, TX_TRASH, TX_WIN_GLASS, TX_WIN_GLASS_N, TX_WIN_BRICK, TX_WIN_BRICK_N,
    TX_WIN_OFFICE, TX_WIN_OFFICE_N, TX_POLICE, TX_FIRE, TX_VAN, TX_SOLAR, TX_AMBU, TX_HOSP, TX_COUNT
};
static Texture2D tex[TX_COUNT];
#define WIN_ROWS 24          // floors available in the window textures (16 px per floor)

static float Hash01(int x, int y, int s) {
    unsigned int h = (unsigned int)(x * 374761393 + y * 668265263 + s * 2246822519u);
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= (h >> 16);
    return (h & 0xFFFF) / 65535.0f;
}

static Color Jitter(Color c, float amt, int x, int y, int s) {
    float k = 1.0f + (Hash01(x, y, s) - 0.5f) * amt;
    return Shade(c, k);
}

static Texture2D Finish(Image img) {
    Texture2D t = LoadTextureFromImage(img);
    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(t, TEXTURE_WRAP_REPEAT);
    UnloadImage(img);
    return t;
}

static Texture2D MakeNoise(Color base, float amt, int seed) {
    Image img = GenImageColor(64, 64, base);
    for (int y = 0; y < 64; y++) for (int x = 0; x < 64; x++) ImageDrawPixel(&img, x, y, Jitter(base, amt, x, y, seed));
    return Finish(img);
}

static Texture2D MakeBrick(void) {
    Color mortar = { 205, 200, 190, 255 };
    Image img = GenImageColor(64, 64, mortar);
    for (int row = 0; row < 8; row++) {
        int off = (row % 2) * 8;
        for (int b = -1; b < 5; b++) {
            Color c = Jitter((Color){ 205, 140, 120, 255 }, 0.28f, b, row, 7);
            ImageDrawRectangle(&img, b * 16 + off + 1, row * 8 + 1, 14, 6, c);
        }
    }
    return Finish(img);
}

static Texture2D MakeSiding(void) {
    Image img = GenImageColor(64, 64, (Color){ 235, 235, 232, 255 });
    for (int y = 0; y < 64; y++)
        for (int x = 0; x < 64; x++) {
            int m = y % 8;
            Color c = (m == 0) ? (Color){ 175, 175, 172, 255 } : (m == 1 ? (Color){ 215, 215, 212, 255 } : Jitter((Color){ 238, 238, 235, 255 }, 0.05f, x, y, 3));
            ImageDrawPixel(&img, x, y, c);
        }
    return Finish(img);
}

static Texture2D MakeRoof(void) {
    Image img = GenImageColor(64, 64, (Color){ 150, 150, 150, 255 });
    for (int row = 0; row < 8; row++) {
        int off = (row % 2) * 4;
        for (int b = -1; b < 8; b++) {
            Color c = Jitter((Color){ 190, 190, 190, 255 }, 0.30f, b, row, 11);
            ImageDrawRectangle(&img, b * 8 + off, row * 8, 7, 6, c);
            ImageDrawRectangle(&img, b * 8 + off, row * 8 + 6, 7, 2, (Color){ 110, 110, 110, 255 });
        }
    }
    return Finish(img);
}

static Texture2D MakeCorrugated(void) {
    Image img = GenImageColor(64, 64, (Color){ 170, 175, 180, 255 });
    for (int x = 0; x < 64; x++) {
        float k = (x % 6 < 3) ? 1.08f : 0.86f;
        for (int y = 0; y < 64; y++) ImageDrawPixel(&img, x, y, Jitter(Shade((Color){ 170, 175, 180, 255 }, k), 0.06f, x, y, 5));
    }
    return Finish(img);
}

static Texture2D MakeTrash(void) {
    Image img = GenImageColor(64, 64, (Color){ 96, 82, 66, 255 });
    for (int y = 0; y < 64; y++) for (int x = 0; x < 64; x++) ImageDrawPixel(&img, x, y, Jitter((Color){ 96, 82, 66, 255 }, 0.5f, x, y, 9));
    for (int i = 0; i < 90; i++) {
        int x = GetRandomValue(0, 62), y = GetRandomValue(0, 62);
        Color c = (Color){ (unsigned char)GetRandomValue(60, 240), (unsigned char)GetRandomValue(60, 240), (unsigned char)GetRandomValue(60, 240), 255 };
        ImageDrawRectangle(&img, x, y, GetRandomValue(1, 3), GetRandomValue(1, 3), c);
    }
    return Finish(img);
}

// Window grid: 4 windows per floor, 16 px per floor, WIN_ROWS floors. night = some lit windows
static Texture2D MakeWindows(Color wall, Color glass, bool night, int seed) {
    Image img = GenImageColor(64, 16 * WIN_ROWS, wall);
    for (int y = 0; y < 16 * WIN_ROWS; y++)
        for (int x = 0; x < 64; x++) ImageDrawPixel(&img, x, y, Jitter(wall, 0.10f, x, y, seed));
    for (int f = 0; f < WIN_ROWS; f++)
        for (int c = 0; c < 4; c++) {
            bool lit = night && Hash01(c, f, seed) < 0.42f;
            Color g = lit ? (Color){ 255, 226, 120, 255 } : Jitter(glass, 0.35f, c, f, seed + 1);
            ImageDrawRectangle(&img, c * 16 + 2, f * 16 + 3, 12, 9, Shade(wall, 0.6f));      // frame
            ImageDrawRectangle(&img, c * 16 + 3, f * 16 + 4, 10, 7, g);
            if (!lit) ImageDrawRectangle(&img, c * 16 + 3, f * 16 + 4, 10, 2, Shade(g, 1.25f)); // sky glint
        }
    return Finish(img);
}

static Texture2D MakePoliceTex(void) {
    Image img = GenImageColor(64, 64, (Color){ 238, 240, 245, 255 });
    ImageDrawRectangle(&img, 0, 0, 64, 18, (Color){ 40, 80, 190, 255 });
    ImageDrawRectangle(&img, 0, 46, 64, 18, (Color){ 40, 80, 190, 255 });
    ImageDrawRectangle(&img, 0, 40, 64, 3, (Color){ 250, 210, 40, 255 });
    ImageDrawText(&img, "POLICE", 8, 22, 14, (Color){ 20, 40, 130, 255 });
    return Finish(img);
}
static Texture2D MakeFireTex(void) {
    Image img = GenImageColor(64, 64, (Color){ 205, 40, 40, 255 });
    for (int y = 0; y < 64; y++) for (int x = 0; x < 64; x++) ImageDrawPixel(&img, x, y, Jitter((Color){ 205, 40, 40, 255 }, 0.07f, x, y, 4));
    ImageDrawRectangle(&img, 0, 36, 64, 5, (Color){ 250, 250, 250, 255 });
    ImageDrawRectangle(&img, 0, 44, 64, 2, (Color){ 250, 210, 40, 255 });
    ImageDrawText(&img, "FIRE", 14, 14, 18, (Color){ 250, 250, 250, 255 });
    return Finish(img);
}
static Texture2D MakeVanTex(void) {
    Image img = GenImageColor(64, 64, (Color){ 245, 245, 245, 255 });
    ImageDrawRectangle(&img, 0, 38, 64, 10, (Color){ 215, 140, 40, 255 });
    ImageDrawText(&img, "SHIP", 14, 14, 16, (Color){ 120, 70, 20, 255 });
    return Finish(img);
}

static Texture2D MakeSolarTex(void) {
    Image img = GenImageColor(64, 64, (Color){ 22, 42, 92, 255 });
    for (int y = 0; y < 64; y++)
        for (int x = 0; x < 64; x++) {
            Color c = (Color){ 24, 48, 108, 255 };
            if (((x % 16) == 0) || ((y % 16) == 0)) c = (Color){ 150, 165, 190, 255 };       // cell gridlines
            else if ((x % 16) == 8) c = (Color){ 40, 70, 140, 255 };                          // busbar
            ImageDrawPixel(&img, x, y, Jitter(c, 0.10f, x, y, 9));
        }
    return Finish(img);
}
static Texture2D MakeAmbulanceTex(void) {
    Image img = GenImageColor(64, 64, (Color){ 244, 246, 248, 255 });
    for (int x = 0; x < 64; x += 8)
        for (int r = 0; r < 2; r++)
            ImageDrawRectangle(&img, x, 34 + r * 8, 8, 8, ((x / 8 + r) & 1) ? (Color){ 250, 215, 30, 255 } : (Color){ 30, 150, 80, 255 });
    ImageDrawText(&img, "AMBULANCE", 2, 18, 9, (Color){ 30, 120, 70, 255 });
    ImageDrawRectangle(&img, 0, 52, 64, 3, (Color){ 30, 150, 80, 255 });
    return Finish(img);
}
static Texture2D MakeHospitalTex(void) {
    Image img = GenImageColor(64, 16 * 4, (Color){ 238, 240, 244, 255 });
    for (int y = 0; y < 64; y++) for (int x = 0; x < 64; x++) ImageDrawPixel(&img, x, y, Jitter((Color){ 238, 240, 244, 255 }, 0.06f, x, y, 12));
    for (int f = 0; f < 4; f++)
        for (int c = 0; c < 4; c++) {
            ImageDrawRectangle(&img, c * 16 + 2, f * 16 + 4, 12, 8, (Color){ 120, 130, 145, 255 });
            ImageDrawRectangle(&img, c * 16 + 3, f * 16 + 5, 10, 6, (Color){ 120, 185, 220, 255 });
        }
    return Finish(img);
}

static void MakeTextures(void) {
    tex[TX_BRICK]    = MakeBrick();
    tex[TX_SIDING]   = MakeSiding();
    tex[TX_STUCCO]   = MakeNoise((Color){ 236, 226, 205, 255 }, 0.12f, 21);
    tex[TX_ROOF]     = MakeRoof();
    tex[TX_GRASS]    = MakeNoise((Color){ 86, 160, 70, 255 }, 0.30f, 31);
    tex[TX_ASPHALT]  = MakeNoise((Color){ 98, 98, 104, 255 }, 0.16f, 41);
    tex[TX_CONCRETE] = MakeNoise((Color){ 185, 185, 182, 255 }, 0.14f, 51);
    tex[TX_CORRUG]   = MakeCorrugated();
    tex[TX_BARK]     = MakeNoise((Color){ 112, 78, 50, 255 }, 0.55f, 61);
    tex[TX_HEDGE]    = MakeNoise((Color){ 38, 110, 48, 255 }, 0.70f, 71);
    tex[TX_TRASH]    = MakeTrash();
    tex[TX_WIN_GLASS]    = MakeWindows((Color){ 150, 168, 188, 255 }, (Color){ 70, 120, 170, 255 }, false, 81);
    tex[TX_WIN_GLASS_N]  = MakeWindows((Color){ 150, 168, 188, 255 }, (Color){ 40, 70, 110, 255 }, true, 81);
    tex[TX_WIN_BRICK]    = MakeWindows((Color){ 196, 132, 110, 255 }, (Color){ 95, 130, 165, 255 }, false, 91);
    tex[TX_WIN_BRICK_N]  = MakeWindows((Color){ 196, 132, 110, 255 }, (Color){ 60, 85, 115, 255 }, true, 91);
    tex[TX_WIN_OFFICE]   = MakeWindows((Color){ 200, 200, 196, 255 }, (Color){ 80, 125, 165, 255 }, false, 101);
    tex[TX_WIN_OFFICE_N] = MakeWindows((Color){ 200, 200, 196, 255 }, (Color){ 50, 80, 115, 255 }, true, 101);
    tex[TX_POLICE]   = MakePoliceTex();
    tex[TX_FIRE]     = MakeFireTex();
    tex[TX_VAN]      = MakeVanTex();
    tex[TX_SOLAR]    = MakeSolarTex();
    tex[TX_AMBU]     = MakeAmbulanceTex();
    tex[TX_HOSP]     = MakeHospitalTex();
}

static void FreeTextures(void) { for (int i = 0; i < TX_COUNT; i++) UnloadTexture(tex[i]); }

// ---- 3D geometry layer ----------------------------------------------------------------
// Every 3D primitive writes world-space triangles into a bucket per texture (opaque or transparent).
// The transform stack uses the same maths as rlgl but runs on the CPU, so the static city is baked
// into a few GPU meshes once, and the moving parts are batched each frame (see DrawScene3D).
#define GB_NONE   TX_COUNT                        // bucket for untextured geometry
#define GB_SLOTS  (TX_COUNT + 1)
#define V3(X, Y, Z) ((Vector3){ (X), (Y), (Z) })
// house and storey dimensions (tile units): one set of numbers drives walls, roof, garage and collisions
#define FLOOR_H  0.115f          // height of one storey (tile units), used by buildings and living walls
// Local frame of a house: origin = tile centre, +Z = the way the front door faces, +X = (DY,-DX) of that direction.
// Fenced compounds: the fence runs on a line FENCE_LINE from the tile centre, so everything in the yard stays inside it.
#define FENCE_LINE 0.44f         // fence stands 0.06 in from the tile edge
#define HS_CX   -0.10f          // body centre
#define HS_CZ   -0.10f
#define HS_W     0.46f          // body width (X)
#define HS_L     0.40f          // body depth (Z)
#define HS_WH    0.26f          // wall height
#define HS_RH    0.14f          // roof rise
#define HS_GX    0.27f          // garage centre (also the driveway / parked car, x)
#define HS_GZ   -0.02f
#define HS_GW    0.22f
#define HS_GL    0.28f
#define HS_GH    0.15f
#define HS_DRV_Z 0.28f          // parked car centre along the front (Z)
#define HS_GATE_X  0.22f        // gateway in the front fence: centre (local X) ...
#define HS_GATE_HW 0.18f        // ... and half width
// does this house's front door face a community fence? (then the fence gets a gateway for it)
static bool HouseGateEdge(int x, int y) {
    if (surface[y][x] != T_HOUSE) return false;
    int z = zone[y][x];
    if (z < Z_GATED1 || z > Z_GATED3) return false;
    int d = faceDir[y][x], nx = x + DX[d], ny = y + DY[d];
    return !(InBounds(nx, ny) && zone[ny][nx] == z);
}

// ---- room beside the road ---------------------------------------------------------------------------
// The sidewalk strip runs along every road edge. Buildings, houses, stations and fences keep clear of it:
// each tile has a "room" box, and what is built on the tile is scaled and slid into that box. Collisions
// use the same mapping, so what is drawn is what is solid.
#define FENCE_ROAD 0.29f           // a compound fence on a road edge stands here, just inside the sidewalk
#define FENCE_GAP  0.012f          // solid things keep this far inside a fence line
#define HNAT_X0 (HS_CX - HS_W * 0.5f)      // natural extents of a house (local frame)
#define HNAT_X1 (HS_GX + HS_GW * 0.5f)
#define HNAT_Z0 (-0.435f)                  // back hedge
#define HNAT_Z1 (HS_GZ + HS_GL * 0.5f)     // garage front
#define HNAT_Z1C 0.44f                     // a compound house also counts its drive
static int DirOfVec(int vx, int vz) { return (vz < 0) ? 0 : (vz > 0) ? 1 : (vx < 0) ? 2 : 3; }
static int RoadSide(int x, int y) {          // direction of the nearest road (for entrances)
    for (int d = 0; d < 4; d++) if (IsRoad(x + DX[d], y + DY[d])) return d;
    return 1;
}
// does side d of this tile open onto a corner square of sidewalk (a diagonal road with both sides open)?
static bool CornerSide(int x, int y, int d) {
    for (int e = 0; e < 4; e++) {
        if (DX[e] * DX[d] + DY[e] * DY[d] != 0) continue;          // e must be at right angles to d
        if (IsRoad(x + DX[d] + DX[e], y + DY[d] + DY[e]) && !IsRoad(x + DX[d], y + DY[d]) && !IsRoad(x + DX[e], y + DY[e])) return true;
    }
    return false;
}
static bool SideWalked(int x, int y, int d) { return IsRoad(x + DX[d], y + DY[d]) || CornerSide(x, y, d); }
// is side d the outside edge of a fenced compound?
static bool FencedSide(int x, int y, int d) {
    int z = zone[y][x];
    if (!ZoneFenced(z)) return false;
    int nx = x + DX[d], ny = y + DY[d];
    return !(InBounds(nx, ny) && zone[ny][nx] == z);
}
// where a compound fence on side d stands (measured from the tile centre)
static float FenceLine(int x, int y, int d) { return SideWalked(x, y, d) ? FENCE_ROAD : FENCE_LINE; }
// how far along the fence on side d (each way from the tile centre) the fence runs: a corner stops where the
// perpendicular fence stands, so the segments meet and never run through each other
static float FenceEnd(int x, int y, int z, int d, int sgn) {
    int tx = DY[d] * sgn, ty = -DX[d] * sgn;                       // tangent (local +x is (DY,-DX))
    int nx = x + tx, ny = y + ty;
    int perp = DirOfVec(tx, ty);
    if (!InBounds(nx, ny) || zone[ny][nx] != z) return FenceLine(x, y, perp);   // corner or free end
    int ox = nx + DX[d], oy = ny + DY[d];
    bool sameLine = !(InBounds(ox, oy) && zone[oy][ox] == z);      // the neighbour's own edge on this side is fenced too
    return sameLine ? 0.5f : FenceLine(x, y, perp);
}
// how far from the tile centre something on side d may reach
static float SideLimit(int x, int y, int d) {
    float lim = 0.5f - (SideWalked(x, y, d) ? SIDE_IN : BLD_MARGIN);
    if (FencedSide(x, y, d)) { float f = FenceLine(x, y, d) - FENCE_GAP; if (f < lim) lim = f; }
    return lim;
}
typedef struct { float x0, x1, z0, z1; } Room;      // x across the frame, z along it (+z = the frame direction)
// frame < 0: world axes. Otherwise the local frame of direction `frame`: +z = that direction, +x = (DY,-DX).
static Room RoomIn(int x, int y, int frame) {
    Room r;
    if (frame < 0) {
        r.x0 = -SideLimit(x, y, 2); r.x1 = SideLimit(x, y, 3);
        r.z0 = -SideLimit(x, y, 0); r.z1 = SideLimit(x, y, 1);
        return r;
    }
    r.z1 = SideLimit(x, y, frame); r.z0 = -SideLimit(x, y, frame ^ 1);
    int px = DY[frame], pz = -DX[frame];
    r.x1 = SideLimit(x, y, DirOfVec(px, pz)); r.x0 = -SideLimit(x, y, DirOfVec(-px, -pz));
    return r;
}
// a thing with natural extents n0..n1 (around its natural centre) scaled by k and slid to sit in a room
typedef struct { float n0x, n1x, n0z, n1z, ncx, ncz, ccx, ccz, kx, kz; } Fit;
static Fit FitTo(Room r, float n0x, float n1x, float n0z, float n1z, bool uniform) {
    Fit f;
    f.n0x = n0x; f.n1x = n1x; f.n0z = n0z; f.n1z = n1z;
    f.ncx = 0.5f * (n0x + n1x); f.ncz = 0.5f * (n0z + n1z);
    float kx = (r.x1 - r.x0) / (n1x - n0x), kz = (r.z1 - r.z0) / (n1z - n0z);
    if (kx > 1.0f) kx = 1.0f;
    if (kz > 1.0f) kz = 1.0f;
    if (uniform) { float k = kx < kz ? kx : kz; kx = kz = k; }
    f.kx = kx; f.kz = kz;
    // slide it as little as possible so that every part stays inside the room
    float lo = r.x0 - f.ncx - kx * (n0x - f.ncx), hi = r.x1 - f.ncx - kx * (n1x - f.ncx);
    float t = 0.0f; if (t < lo) t = lo; if (t > hi) t = hi;
    f.ccx = f.ncx + t;
    lo = r.z0 - f.ncz - kz * (n0z - f.ncz); hi = r.z1 - f.ncz - kz * (n1z - f.ncz);
    t = 0.0f; if (t < lo) t = lo; if (t > hi) t = hi;
    f.ccz = f.ncz + t;
    return f;
}
static Fit FitIn(int x, int y, int frame, float n0x, float n1x, float n0z, float n1z) {
    return FitTo(RoomIn(x, y, frame), n0x, n1x, n0z, n1z, false);
}
static Fit HouseFit(int x, int y) {
    bool comp = ZoneFenced(zone[y][x]);
    return FitTo(RoomIn(x, y, faceDir[y][x]), HNAT_X0, HNAT_X1, HNAT_Z0, comp ? HNAT_Z1C : HNAT_Z1, comp);
}
// the gateway a compound house leaves in its fence, in the house's frame (centre and half width); its piers
// stand 0.07 out from the gap, so the gap is kept well short of the fence's corners
static void HouseGate(int x, int y, float *gc, float *gh) {
    Fit f = HouseFit(x, y);
    int z = zone[y][x], d = faceDir[y][x];
    float c = f.ccx + f.kx * (HS_GATE_X - f.ncx), h = f.kx * HS_GATE_HW;
    float left = -FenceEnd(x, y, z, d, -1) + 0.08f, right = FenceEnd(x, y, z, d, 1) - 0.08f;
    if (h > 0.5f * (right - left)) h = 0.5f * (right - left);
    if (c < left + h) c = left + h;
    if (c > right - h) c = right - h;
    *gc = c; *gh = h;
}
// is a point (offset from the tile centre, world axes) inside a fitted thing's natural extents?
static bool FitHit(Fit f, int frame, float dx, float dz) {
    float lx = dx, lz = dz;
    if (frame >= 0) { lx = dx * DY[frame] - dz * DX[frame]; lz = dx * DX[frame] + dz * DY[frame]; }
    float nx = f.ncx + (lx - f.ccx) / f.kx, nz = f.ncz + (lz - f.ccz) / f.kz;
    return nx >= f.n0x && nx <= f.n1x && nz >= f.n0z && nz <= f.n1z;
}
// a mall block: neighbouring mall tiles run together; every other edge stops at the room line
static void MallRect(int x, int y, float *x0, float *x1, float *z0, float *z1) {
    bool w = x > 0 && surface[y][x - 1] == T_MALL, e = x + 1 < MAP_W && surface[y][x + 1] == T_MALL;
    bool n = y > 0 && surface[y - 1][x] == T_MALL, s = y + 1 < MAP_H && surface[y + 1][x] == T_MALL;
    *x0 = w ? (float)x       : x + 0.5f - SideLimit(x, y, 2);
    *x1 = e ? (float)(x + 1) : x + 0.5f + SideLimit(x, y, 3);
    *z0 = n ? (float)y       : y + 0.5f - SideLimit(x, y, 0);
    *z1 = s ? (float)(y + 1) : y + 0.5f + SideLimit(x, y, 1);
}

typedef struct { float x, y, z, u, v; unsigned char r, g, b, a; } GVert;
typedef struct { GVert *v; int n, cap; } GBuf;

static unsigned int gSceneGen = 0;                 // bumped whenever the city is regenerated (forces a rebake)
static GBuf     gBuf[GB_SLOTS][2];                // [texture slot][0 = opaque, 1 = transparent]
static Mesh     gMesh[GB_SLOTS][2];               // baked static scenery
static bool     gHasMesh[GB_SLOTS][2];
static Material gMat[GB_SLOTS];
static bool     gMatsReady = false;
static Matrix   gM;                               // current transform: world = gM * local
static Matrix   gMStack[48];
static int      gMSP = 0;

static void GXReset(void) { gM = MatrixIdentity(); gMSP = 0; }
static void GXPush(void)  { if (gMSP < 48) gMStack[gMSP++] = gM; }
static void GXPop(void)   { if (gMSP > 0) gM = gMStack[--gMSP]; }
static void GXTrans(float x, float y, float z) { gM = MatrixMultiply(MatrixTranslate(x, y, z), gM); }
static void GXRot(float deg, float ax, float ay, float az) { gM = MatrixMultiply(MatrixRotate((Vector3){ ax, ay, az }, deg * DEG2RAD), gM); }
static void GXScale(float x, float y, float z) { gM = MatrixMultiply(MatrixScale(x, y, z), gM); }
// put a fit on the current transform; (ox,oz) = the tile centre for world-axes drawing, (0,0) inside a frame
static void FitApplyAt(Fit f, float ox, float oz) {
    GXTrans(ox + f.ccx, 0.0f, oz + f.ccz); GXScale(f.kx, 1.0f, f.kz); GXTrans(-(ox + f.ncx), 0.0f, -(oz + f.ncz));
}

static Color ShadeF(Color c, float k) { return (Color){ (unsigned char)(c.r * k), (unsigned char)(c.g * k), (unsigned char)(c.b * k), c.a }; }

static GVert GV(Vector3 p, float u, float v, Color c) {
    Vector3 q = Vector3Transform(p, gM);
    GVert r = { q.x, q.y, q.z, u, v, c.r, c.g, c.b, c.a };
    return r;
}
static GBuf *GBufOf(int ti, Color c) { return &gBuf[ti < 0 ? GB_NONE : ti][c.a < 255 ? 1 : 0]; }
static void GAdd(GBuf *b, GVert a) {
    if (b->n == b->cap) { b->cap = b->cap ? b->cap * 2 : 8192; b->v = (GVert *)realloc(b->v, (size_t)b->cap * sizeof(GVert)); }
    b->v[b->n++] = a;
}
// quad p[0..3] in order (two triangles); uv = 8 floats or NULL
static void GQuad(int ti, const Vector3 *p, const float *uv, Color c) {
    static const float zero[8] = { 0 };
    const float *t = uv ? uv : zero;
    GBuf *g = GBufOf(ti, c);
    GVert A = GV(p[0], t[0], t[1], c), B = GV(p[1], t[2], t[3], c), C = GV(p[2], t[4], t[5], c), D = GV(p[3], t[6], t[7], c);
    GAdd(g, A); GAdd(g, B); GAdd(g, C);
    GAdd(g, A); GAdd(g, C); GAdd(g, D);
}
// triangle p[0..2]; uv = 6 floats or NULL
static void GTri(int ti, const Vector3 *p, const float *uv, Color c) {
    static const float zero[6] = { 0 };
    const float *t = uv ? uv : zero;
    GBuf *g = GBufOf(ti, c);
    GAdd(g, GV(p[0], t[0], t[1], c)); GAdd(g, GV(p[1], t[2], t[3], c)); GAdd(g, GV(p[2], t[4], t[5], c));
}

// Box with simple per-face shading (top lightest, one side darkest)
static void BoxC(float x, float y, float z, float w, float h, float l, Color c) {
    float a = w * 0.5f, b = h * 0.5f, d = l * 0.5f;
    GQuad(-1, (Vector3[4]){ V3(x - a, y - b, z + d), V3(x + a, y - b, z + d), V3(x + a, y + b, z + d), V3(x - a, y + b, z + d) }, NULL, ShadeF(c, 0.90f));   // front +z
    GQuad(-1, (Vector3[4]){ V3(x - a, y - b, z - d), V3(x - a, y + b, z - d), V3(x + a, y + b, z - d), V3(x + a, y - b, z - d) }, NULL, ShadeF(c, 0.78f));   // back -z
    GQuad(-1, (Vector3[4]){ V3(x - a, y + b, z - d), V3(x - a, y + b, z + d), V3(x + a, y + b, z + d), V3(x + a, y + b, z - d) }, NULL, c);                  // top
    GQuad(-1, (Vector3[4]){ V3(x - a, y - b, z - d), V3(x + a, y - b, z - d), V3(x + a, y - b, z + d), V3(x - a, y - b, z + d) }, NULL, ShadeF(c, 0.50f));   // bottom
    GQuad(-1, (Vector3[4]){ V3(x + a, y - b, z - d), V3(x + a, y + b, z - d), V3(x + a, y + b, z + d), V3(x + a, y - b, z + d) }, NULL, ShadeF(c, 0.84f));   // right +x
    GQuad(-1, (Vector3[4]){ V3(x - a, y - b, z - d), V3(x - a, y - b, z + d), V3(x - a, y + b, z + d), V3(x - a, y + b, z - d) }, NULL, ShadeF(c, 0.72f));   // left -x
}

// Textured cube; (u, v) are the texture-coordinate extents used on the side faces
static void CubeUV(int ti, float x, float y, float z, float w, float h, float l, Color c, float u, float v) {
    float a = w * 0.5f, b = h * 0.5f, d = l * 0.5f;
    GQuad(ti, (Vector3[4]){ V3(x - a, y - b, z + d), V3(x + a, y - b, z + d), V3(x + a, y + b, z + d), V3(x - a, y + b, z + d) },
          (float[8]){ 0, v, u, v, u, 0, 0, 0 }, ShadeF(c, 0.90f));                                   // front
    GQuad(ti, (Vector3[4]){ V3(x - a, y - b, z - d), V3(x - a, y + b, z - d), V3(x + a, y + b, z - d), V3(x + a, y - b, z - d) },
          (float[8]){ u, v, u, 0, 0, 0, 0, v }, ShadeF(c, 0.78f));                                   // back
    GQuad(ti, (Vector3[4]){ V3(x - a, y + b, z - d), V3(x - a, y + b, z + d), V3(x + a, y + b, z + d), V3(x + a, y + b, z - d) },
          (float[8]){ 0, 1, 0, 0, 1, 0, 1, 1 }, c);                                                  // top
    GQuad(ti, (Vector3[4]){ V3(x - a, y - b, z - d), V3(x + a, y - b, z - d), V3(x + a, y - b, z + d), V3(x - a, y - b, z + d) },
          (float[8]){ 1, 1, 0, 1, 0, 0, 1, 0 }, ShadeF(c, 0.50f));                                   // bottom
    GQuad(ti, (Vector3[4]){ V3(x + a, y - b, z - d), V3(x + a, y + b, z - d), V3(x + a, y + b, z + d), V3(x + a, y - b, z + d) },
          (float[8]){ u, v, u, 0, 0, 0, 0, v }, ShadeF(c, 0.84f));                                   // right
    GQuad(ti, (Vector3[4]){ V3(x - a, y - b, z - d), V3(x - a, y - b, z + d), V3(x - a, y + b, z + d), V3(x - a, y + b, z - d) },
          (float[8]){ 0, v, u, v, u, 0, 0, 0 }, ShadeF(c, 0.72f));                                   // left
}
static void BoxT(int ti, float x, float y, float z, float w, float h, float l, Color c) {
    CubeUV(ti, x, y, z, w, h, l, c, 1.0f, 1.0f);
}
// building walls with N floors of windows (crops the window texture from the top)
static void WindowBox(int ti, int floors, float x, float y, float z, float w, float h, float l, Color tint) {
    if (floors > WIN_ROWS) floors = WIN_ROWS;
    if (floors < 1) floors = 1;
    CubeUV(ti, x, y, z, w, h, l, tint, 1.0f, (float)floors / WIN_ROWS);
}

static void Quad3T(int ti, Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color col, float u, float v) {
    GQuad(ti, (Vector3[4]){ a, b, c, d }, (float[8]){ 0, 0, 0, v, u, v, u, 0 }, col);
}
static void Tri3T(int ti, Vector3 a, Vector3 b, Vector3 c, Color col, float u, float v) {
    GTri(ti, (Vector3[3]){ a, b, c }, (float[6]){ 0, 0, u, 0, u * 0.5f, v }, col);
}

// Gable roof over a w (X) by l (Z) footprint, eaves at y0, ridge h above. The roof edges are the
// footprint edges, so it sits flush on the walls. alongZ = ridge runs along Z, else along X.
static void RoofGable(float w, float l, float y0, float h, bool alongZ, Color roofCol, int wallTex, Color wallCol) {
    float a = w * 0.5f, b = l * 0.5f, yh = y0 + h;
    Color dark = ShadeF(roofCol, 0.82f);
    if (alongZ) {
        Quad3T(TX_ROOF, V3(-a, y0, -b), V3(-a, y0, b), V3(0, yh, b), V3(0, yh, -b), roofCol, 3, 2);
        Quad3T(TX_ROOF, V3(a, y0, -b), V3(a, y0, b), V3(0, yh, b), V3(0, yh, -b), dark, 3, 2);
        Tri3T(wallTex, V3(-a, y0, b), V3(a, y0, b), V3(0, yh, b), wallCol, 1, 1);
        Tri3T(wallTex, V3(-a, y0, -b), V3(a, y0, -b), V3(0, yh, -b), wallCol, 1, 1);
    } else {
        Quad3T(TX_ROOF, V3(-a, y0, -b), V3(a, y0, -b), V3(a, yh, 0), V3(-a, yh, 0), roofCol, 3, 2);
        Quad3T(TX_ROOF, V3(-a, y0, b), V3(a, y0, b), V3(a, yh, 0), V3(-a, yh, 0), dark, 3, 2);
        Tri3T(wallTex, V3(a, y0, -b), V3(a, y0, b), V3(a, yh, 0), wallCol, 1, 1);
        Tri3T(wallTex, V3(-a, y0, -b), V3(-a, y0, b), V3(-a, yh, 0), wallCol, 1, 1);
    }
}

// Hipped (pyramid) roof
static void RoofHip(float w, float l, float y0, float h, Color roofCol) {
    float a = w * 0.5f, b = l * 0.5f;
    Vector3 top = { 0, y0 + h, 0 };
    Vector3 p0 = { -a, y0, -b }, p1 = { a, y0, -b }, p2 = { a, y0, b }, p3 = { -a, y0, b };
    Tri3T(TX_ROOF, p0, p1, top, roofCol, 2, 2);
    Tri3T(TX_ROOF, p1, p2, top, ShadeF(roofCol, 0.85f), 2, 2);
    Tri3T(TX_ROOF, p2, p3, top, roofCol, 2, 2);
    Tri3T(TX_ROOF, p3, p0, top, ShadeF(roofCol, 0.85f), 2, 2);
}

// Enter / leave a local frame at tile-space (x,z) with a heading in degrees (local +Z = forward)
static void PushFrame(float x, float y, float z, float yawDeg) {
    GXPush();
    GXTrans(x, y, z);
    GXRot(yawDeg, 0, 1, 0);
}
static void PopFrame(void) { GXPop(); }
static float FaceYaw(int dir) { return atan2f((float)DX[dir], (float)DY[dir]) * RAD2DEG; }

// ---- solids (replace raylib's DrawSphere / DrawCylinder / DrawCube / DrawPlane; low-poly) ----
static void GSphereN(Vector3 c, float r, Color col, int sl, int rg) {
    for (int i = 0; i < rg; i++) {
        float p0 = PI * i / rg, p1 = PI * (i + 1) / rg;
        float s0 = sinf(p0), c0 = cosf(p0), s1 = sinf(p1), c1 = cosf(p1);
        for (int j = 0; j < sl; j++) {
            float t0 = 2.0f * PI * j / sl, t1 = 2.0f * PI * (j + 1) / sl;
            float ct0 = cosf(t0), st0 = sinf(t0), ct1 = cosf(t1), st1 = sinf(t1);
            GQuad(-1, (Vector3[4]){ V3(c.x + r * s0 * ct0, c.y + r * c0, c.z + r * s0 * st0), V3(c.x + r * s0 * ct1, c.y + r * c0, c.z + r * s0 * st1),
                                    V3(c.x + r * s1 * ct1, c.y + r * c1, c.z + r * s1 * st1), V3(c.x + r * s1 * ct0, c.y + r * c1, c.z + r * s1 * st0) }, NULL, col);
        }
    }
}
static void GSphere(Vector3 c, float r, Color col) {
    if (r < 0.02f) GSphereN(c, r, col, 6, 4);
    else if (r < 0.08f) GSphereN(c, r, col, 8, 6);
    else GSphereN(c, r, col, 12, 8);
}
// cylinder from p0 (radius r0) to p1 (radius r1), any direction
static void GCylinderEx(Vector3 p0, Vector3 p1, float r0, float r1, int sides, Color col) {
    Vector3 d = Vector3Subtract(p1, p0);
    float len = Vector3Length(d);
    if (len < 1e-6f || sides < 3) return;
    Vector3 dir = Vector3Scale(d, 1.0f / len);
    Vector3 ref = fabsf(dir.y) < 0.95f ? V3(0, 1, 0) : V3(1, 0, 0);
    Vector3 u = Vector3Normalize(Vector3CrossProduct(dir, ref));
    Vector3 v = Vector3CrossProduct(dir, u);
    for (int i = 0; i < sides; i++) {
        float a0 = 2.0f * PI * i / sides, a1 = 2.0f * PI * (i + 1) / sides;
        Vector3 k0 = Vector3Add(Vector3Scale(u, cosf(a0)), Vector3Scale(v, sinf(a0)));
        Vector3 k1 = Vector3Add(Vector3Scale(u, cosf(a1)), Vector3Scale(v, sinf(a1)));
        Vector3 s0 = Vector3Add(p0, Vector3Scale(k0, r0)), s1 = Vector3Add(p0, Vector3Scale(k1, r0));
        Vector3 e0 = Vector3Add(p1, Vector3Scale(k0, r1)), e1 = Vector3Add(p1, Vector3Scale(k1, r1));
        GQuad(-1, (Vector3[4]){ s0, s1, e1, e0 }, NULL, col);
        if (r0 > 0.0f) GTri(-1, (Vector3[3]){ p0, s1, s0 }, NULL, col);
        if (r1 > 0.0f) GTri(-1, (Vector3[3]){ p1, e0, e1 }, NULL, col);
    }
}
// upright cylinder standing on pos (base radius rb, top radius rt, height h)
static void GCylinder(Vector3 pos, float rt, float rb, float h, int sides, Color col) {
    GCylinderEx(pos, V3(pos.x, pos.y + h, pos.z), rb, rt, sides, col);
}
static void GCube(Vector3 c, float w, float h, float l, Color col) {        // single colour, no face shading
    float a = w * 0.5f, b = h * 0.5f, d = l * 0.5f;
    GQuad(-1, (Vector3[4]){ V3(c.x - a, c.y - b, c.z + d), V3(c.x + a, c.y - b, c.z + d), V3(c.x + a, c.y + b, c.z + d), V3(c.x - a, c.y + b, c.z + d) }, NULL, col);
    GQuad(-1, (Vector3[4]){ V3(c.x - a, c.y - b, c.z - d), V3(c.x - a, c.y + b, c.z - d), V3(c.x + a, c.y + b, c.z - d), V3(c.x + a, c.y - b, c.z - d) }, NULL, col);
    GQuad(-1, (Vector3[4]){ V3(c.x - a, c.y + b, c.z - d), V3(c.x - a, c.y + b, c.z + d), V3(c.x + a, c.y + b, c.z + d), V3(c.x + a, c.y + b, c.z - d) }, NULL, col);
    GQuad(-1, (Vector3[4]){ V3(c.x - a, c.y - b, c.z - d), V3(c.x + a, c.y - b, c.z - d), V3(c.x + a, c.y - b, c.z + d), V3(c.x - a, c.y - b, c.z + d) }, NULL, col);
    GQuad(-1, (Vector3[4]){ V3(c.x + a, c.y - b, c.z - d), V3(c.x + a, c.y + b, c.z - d), V3(c.x + a, c.y + b, c.z + d), V3(c.x + a, c.y - b, c.z + d) }, NULL, col);
    GQuad(-1, (Vector3[4]){ V3(c.x - a, c.y - b, c.z - d), V3(c.x - a, c.y - b, c.z + d), V3(c.x - a, c.y + b, c.z + d), V3(c.x - a, c.y + b, c.z - d) }, NULL, col);
}
static void GPlane(Vector3 c, Vector2 size, Color col) {                     // flat rectangle on the XZ plane
    float a = size.x * 0.5f, d = size.y * 0.5f;
    GQuad(-1, (Vector3[4]){ V3(c.x - a, c.y, c.z - d), V3(c.x - a, c.y, c.z + d), V3(c.x + a, c.y, c.z + d), V3(c.x + a, c.y, c.z - d) }, NULL, col);
}

// ---- drawing the buckets ----
static void GBakeStart(void) {
    for (int s = 0; s < GB_SLOTS; s++) for (int t = 0; t < 2; t++) gBuf[s][t].n = 0;
    GXReset();
}

// Upload everything collected since GBakeStart as the static scenery (replaces the previous bake)
static void GBakeFinish(void) {
    if (!gMatsReady) {
        for (int s = 0; s < GB_SLOTS; s++) {
            gMat[s] = LoadMaterialDefault();
            if (s != GB_NONE) gMat[s].maps[MATERIAL_MAP_DIFFUSE].texture = tex[s];
        }
        gMatsReady = true;
    }
    for (int s = 0; s < GB_SLOTS; s++) for (int t = 0; t < 2; t++) {
        if (gHasMesh[s][t]) { UnloadMesh(gMesh[s][t]); gHasMesh[s][t] = false; }
        GBuf *g = &gBuf[s][t];
        if (g->n > 0) {
            Mesh m = { 0 };
            m.vertexCount = g->n;
            m.triangleCount = g->n / 3;
            m.vertices = (float *)malloc((size_t)g->n * 3 * sizeof(float));
            m.texcoords = (float *)malloc((size_t)g->n * 2 * sizeof(float));
            m.colors = (unsigned char *)malloc((size_t)g->n * 4);
            for (int i = 0; i < g->n; i++) {
                const GVert *v = &g->v[i];
                m.vertices[i * 3] = v->x; m.vertices[i * 3 + 1] = v->y; m.vertices[i * 3 + 2] = v->z;
                m.texcoords[i * 2] = v->u; m.texcoords[i * 2 + 1] = v->v;
                m.colors[i * 4] = v->r; m.colors[i * 4 + 1] = v->g; m.colors[i * 4 + 2] = v->b; m.colors[i * 4 + 3] = v->a;
            }
            UploadMesh(&m, false);
            free(m.vertices); m.vertices = NULL;          // the GPU holds the data now
            free(m.texcoords); m.texcoords = NULL;
            free(m.colors); m.colors = NULL;
            gMesh[s][t] = m;
            gHasMesh[s][t] = true;
        }
        g->n = 0;
    }
}

// Draw the baked scenery of one layer (opaque = 0, transparent = 1)
static void GDrawBaked(bool trans) {
    for (int s = 0; s < GB_SLOTS; s++)
        if (gHasMesh[s][trans]) DrawMesh(gMesh[s][trans], gMat[s], MatrixIdentity());
}

// Draw the geometry collected since GBakeStart / GBeginLive, batched by texture, and clear it
static void GFlushLive(bool trans) {
    for (int s = 0; s < GB_SLOTS; s++) {
        GBuf *g = &gBuf[s][trans];
        if (g->n == 0) continue;
        rlSetTexture(s == GB_NONE ? 0 : tex[s].id);
        rlBegin(RL_TRIANGLES);
        for (int i = 0; i < g->n; i++) {
            const GVert *v = &g->v[i];
            rlColor4ub(v->r, v->g, v->b, v->a);
            rlTexCoord2f(v->u, v->v);
            rlVertex3f(v->x, v->y, v->z);
        }
        rlEnd();
        rlSetTexture(0);
        g->n = 0;
    }
}

// ---------------------------------------------------------------------------
// City generation
// ---------------------------------------------------------------------------
static void SpawnPed(Ped *p);
static void InitPlus(void);
static void InitFp(void);

static bool FindAccess(int x, int y, int *rx, int *ry) {
    for (int d = 0; d < 4; d++) {
        if (IsRoad(x + DX[d], y + DY[d])) { *rx = x + DX[d]; *ry = y + DY[d]; return true; }
    }
    return false;
}

static void PlaceFixed(int x, int y, int tile) {
    surface[y][x] = tile; health[y][x] = 100.0f; bHeight[y][x] = 0.0f; deco[y][x] = 0; zone[y][x] = 0;
}

// civilian car model for driveways / car parks
static int RandModel(void) {
    int r = GetRandomValue(0, 99);
    if (r < 28) return V_SALOON;
    if (r < 50) return V_HATCH;
    if (r < 62) return V_ESTATE;
    if (r < 72) return V_PICKUP;
    if (r < 92) return V_SUV;
    return V_VAN;
}

static bool PedSide(int x, int y) {     // true when (x,y) is a sidewalk tile
    return InBounds(x, y) && pedNode[y][x];
}

static void BuildCommunity(int id, int x0, int y0, int x1, int y1, int laneY, int gateX) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            zone[y][x] = (unsigned char)id;
            vr[y][x] = (unsigned char)GetRandomValue(0, 255);
            if (y == laneY) {
                surface[y][x] = T_GRASS; paved[y][x] = 1;
                if (x == gateX) gateTile[y][x] = true;
                continue;
            }
            surface[y][x] = T_HOUSE; bType[y][x] = B_RES; bHeight[y][x] = 0.38f;
            int face;
            if (y < laneY) face = (y == laneY - 1) ? 1 : 0;
            else           face = (y == laneY + 1) ? 0 : 1;
            faceDir[y][x] = (unsigned char)face;
            // (no car in a compound drive: the house is fitted inside its fence, too small to park one)
        }
    // a small pocket park inside each community
    int px = x1 - 1, py = (laneY > y0) ? laneY - 1 : laneY + 1;
    if (InBounds(px, py) && zone[py][px] == id) {
        surface[py][px] = T_TREE; vr[py][px] = (unsigned char)GetRandomValue(0, 255); drive[py][px] = 0; deco[py][px] = 3;
    }
}

static void GenerateCity(void) {
    gSceneGen++;
    static const int roadRows[3] = { 3, 9, 13 };
    static const int roadCols[3] = { 4, 11, 18 };
    // junction control: [row][col]
    static const unsigned char junction[3][3] = {
        { C_SIGNAL, C_ROUNDABOUT, C_SIGNAL },
        { C_SIGNAL, C_SIGNAL,     C_SIGNAL },
        { C_STOP,   C_STOP,       C_ROUNDABOUT }
    };

    memset(bType, 0, sizeof(bType));   memset(zone, 0, sizeof(zone));    memset(faceDir, 0, sizeof(faceDir));
    memset(vr, 0, sizeof(vr));         memset(deco, 0, sizeof(deco));    memset(paved, 0, sizeof(paved));
    memset(gateTile, 0, sizeof(gateTile)); memset(drive, 0, sizeof(drive)); memset(drvCol, 0, sizeof(drvCol));
    memset(lot, 0, sizeof(lot));       memset(lotCol, 0, sizeof(lotCol)); memset(ctrl, 0, sizeof(ctrl));
    memset(crossT, 0, sizeof(crossT)); memset(nodeOcc, 0, sizeof(nodeOcc)); memset(crossBusy, 0, sizeof(crossBusy));
    memset(pedNode, 0, sizeof(pedNode));

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            surface[y][x] = T_GRASS; bHeight[y][x] = 0; health[y][x] = 100.0f;
            fireI[y][x] = 0; pipeNode[y][x] = false; heatMap[y][x] = 0; polMap[y][x] = 0; traffic[y][x] = 0;
            crossI[y][x] = -1; sigOff[y][x] = 0;
        }

    for (int i = 0; i < 3; i++) {
        for (int x = 0; x < MAP_W; x++) surface[roadRows[i]][x] = T_ROAD;
        for (int y = 0; y < MAP_H; y++) surface[y][roadCols[i]] = T_ROAD;
    }

    // ---- Special zones -------------------------------------------------------
    BuildCommunity(Z_GATED1, 5, 4, 10, 8, 6, 5);
    BuildCommunity(Z_GATED2, 0, 4, 3, 8, 6, 3);
    BuildCommunity(Z_GATED3, 5, 10, 10, 12, 11, 5);

    // Industrial zone (F factory, W warehouse, T tank farm, Y loading yard)
    static const char *ind[3] = { "FFYWWT", "TYYYFF", "WWWYTF" };
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 6; c++) {
            int x = 12 + c, y = 10 + r;
            char k = ind[r][c];
            zone[y][x] = Z_INDUSTRIAL;
            vr[y][x] = (unsigned char)GetRandomValue(0, 255);
            if (k == 'Y') { paved[y][x] = 1; if (x == 14 && y == 10) gateTile[y][x] = true; continue; }
            surface[y][x] = T_BUILDING; bType[y][x] = B_IND;
            deco[y][x] = (k == 'F') ? 0 : (k == 'W' ? 1 : 2);        // building sub-type
            bHeight[y][x] = (k == 'F') ? 0.5f + Rf() * 0.2f : (k == 'W' ? 0.35f : 0.55f);
        }
    gateTile[10][14] = true;

    // Landfill
    for (int y = 10; y <= 12; y++)
        for (int x = 19; x <= 23; x++) {
            zone[y][x] = Z_LANDFILL; surface[y][x] = T_LANDFILL; vr[y][x] = (unsigned char)GetRandomValue(0, 255);
            bHeight[y][x] = 0.12f + Rf() * 0.22f;
        }
    paved[11][19] = 1; gateTile[11][19] = true;

    // Shopping centre + car park
    for (int y = 4; y <= 8; y++)
        for (int x = 19; x <= 23; x++) {
            zone[y][x] = Z_MALL;
            if (y <= 5 && x >= 20) { surface[y][x] = T_MALL; bType[y][x] = B_COM; bHeight[y][x] = 0.42f; faceDir[y][x] = 1; }
            else if (!(x == 19 && y == 4)) {
                surface[y][x] = T_PARK;
                for (int s = 0; s < 8; s++)
                    if (Rf() < 0.4f) { lot[y][x][s] = (unsigned char)(RandModel() + 1); lotCol[y][x][s] = (unsigned char)GetRandomValue(0, N_CARCOL - 1); }
            }
        }

    // ---- Everything else: apartments, offices, skyrises, parkland ------------
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] != T_GRASS || zone[y][x] != 0) continue;
            bool nearRoad = IsRoad(x - 1, y) || IsRoad(x + 1, y) || IsRoad(x, y - 1) || IsRoad(x, y + 1);
            bool downtown = (x >= 12 && x <= 17 && y <= 8);
            int r = GetRandomValue(0, 99);
            vr[y][x] = (unsigned char)GetRandomValue(0, 255);
            if (nearRoad && r < 74) {
                surface[y][x] = T_BUILDING;
                if (downtown)                    { bType[y][x] = B_COM; bHeight[y][x] = GetRandomValue(8, 22) * 0.1f; }
                else if (GetRandomValue(0, 99) < 62) { bType[y][x] = B_RES; bHeight[y][x] = GetRandomValue(2, 5) * 0.1f; }
                else                             { bType[y][x] = B_COM; bHeight[y][x] = GetRandomValue(3, 8) * 0.1f; }
            } else if (nearRoad ? (r < 86) : (r < 52)) {
                surface[y][x] = T_TREE;
            } else {
                int k = GetRandomValue(0, 99);
                deco[y][x] = (k < 22) ? 1 : (k < 40 ? 2 : (k < 55 ? 3 : 0));
            }
        }

    // ---- Fixed city services ---------------------------------------------------
    PlaceFixed(2, 2, T_FIRE_STN);
    PlaceFixed(19, 4, T_FIRE_STN);
    PlaceFixed(10, 2, T_POLICE_STN);
    PlaceFixed(3, 11, T_POLICE_STN);
    PlaceFixed(6, 2, T_HOSPITAL); bHeight[2][6] = 0.55f; bType[2][6] = B_COM;
    PlaceFixed(12, 6, T_POWER);
    PlaceFixed(17, 6, T_WATER);
    faceDir[2][6] = 1; faceDir[2][2] = 1; faceDir[4][19] = 3; faceDir[2][10] = 3; faceDir[11][3] = 3;

    // ---- Utility pipes: every road plus every built tile touching a road ------
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x], rx, ry;
            if (s == T_ROAD) pipeNode[y][x] = true;
            else if (s != T_GRASS && s != T_TREE && FindAccess(x, y, &rx, &ry)) pipeNode[y][x] = true;
        }

    // ---- Junction control ---------------------------------------------------------
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            int x = roadCols[c], y = roadRows[r];
            ctrl[y][x] = junction[r][c];
            sigOff[y][x] = Rf() * SIG_CYCLE;
            {   // start the signal phase machine at a random point of the 24 s cycle
                float ta = sigOff[y][x];
                static const float dur[6] = { 9, 2, 1, 9, 2, 1 };
                int ph = 0;
                while (ta >= dur[ph]) { ta -= dur[ph]; ph++; if (ph > 5) { ph = 5; break; } }
                sigPhase[y][x] = (unsigned char)ph; sigT[y][x] = ta;
            }
        }

    // ---- Sidewalk graph + pedestrian crossings ----------------------------------
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            pedNode[y][x] = (surface[y][x] != T_ROAD) && RoadNeighbours(x, y) > 0;

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!IsRoad(x, y) || ctrl[y][x] == C_NONE) continue;
            int ji = y * MAP_W + x;
            for (int d = 0; d < 4; d++) {
                int lx = x + DX[d], ly = y + DY[d];
                if (!IsRoad(lx, ly) || RoadNeighbours(lx, ly) != 2 || RoadLine(lx, ly) == 0) continue;
                int ct = (d < 2) ? 1 : 2;
                bool ok = (ct == 1) ? (PedSide(lx - 1, ly) && PedSide(lx + 1, ly)) : (PedSide(lx, ly - 1) && PedSide(lx, ly + 1));
                if (ok) { crossT[ly][lx] = (unsigned char)ct; crossI[ly][lx] = ji; }
            }
        }
    // Mid-block crossings at sensible places (gates, car park, shops, long straight blocks)
    static const int mid[][2] = { {4,6},{18,6},{11,6},{8,3},{15,3},{21,3},{8,9},{15,9},{21,9},{8,13},{21,13},{4,11} };
    for (unsigned i = 0; i < sizeof(mid) / sizeof(mid[0]); i++) {
        int x = mid[i][0], y = mid[i][1];
        if (!IsRoad(x, y) || RoadNeighbours(x, y) != 2 || RoadLine(x, y) == 0 || crossT[y][x]) continue;
        bool nearCross = false;
        for (int d = 0; d < 4; d++) if (InBounds(x + DX[d], y + DY[d]) && crossT[y + DY[d]][x + DX[d]]) nearCross = true;
        if (nearCross) continue;
        int ct = (RoadLine(x, y) == 2) ? 1 : 2;
        bool ok = (ct == 1) ? (PedSide(x - 1, y) && PedSide(x + 1, y)) : (PedSide(x, y - 1) && PedSide(x, y + 1));
        if (ok) { crossT[y][x] = (unsigned char)ct; crossI[y][x] = -1; }
    }

    // ---- Register stations --------------------------------------------------------
    nFire = nPolice = nHosp = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int rx, ry;
            if (surface[y][x] == T_FIRE_STN && nFire < MAX_STN && FindAccess(x, y, &rx, &ry))
                fireStn[nFire++] = (Station){ x, y, rx, ry };
            if (surface[y][x] == T_POLICE_STN && nPolice < MAX_STN && FindAccess(x, y, &rx, &ry))
                policeStn[nPolice++] = (Station){ x, y, rx, ry };
            if (surface[y][x] == T_HOSPITAL && nHosp < MAX_STN && FindAccess(x, y, &rx, &ry))
                hospStn[nHosp++] = (Station){ x, y, rx, ry };
        }

    // ---- smart traffic monitor masts: straight road tiles two steps out from every junction ----
    memset(monitorTile, 0, sizeof(monitorTile));
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!IsRoad(x, y) || ctrl[y][x] == C_NONE) continue;
            for (int d = 0; d < 4; d++) {
                int mx = x + 2 * DX[d], my = y + 2 * DY[d];
                if (IsRoad(mx, my) && RoadNeighbours(mx, my) == 2 && RoadLine(mx, my) != 0 && !crossT[my][mx]) monitorTile[my][mx] = true;
            }
        }

    memset(cars, 0, sizeof(cars));
    memset(ambs, 0, sizeof(ambs));
    memset(drones, 0, sizeof(drones));
    memset(peds, 0, sizeof(peds));
    memset(trucks, 0, sizeof(trucks));
    memset(police, 0, sizeof(police));
    memset(parts, 0, sizeof(parts));
    for (int i = 0; i < 30; i++) SpawnPed(&peds[i]);
    InitPlus();
    InitFp();
}

// ---------------------------------------------------------------------------
// Road pathfinding (BFS) + emergency responders
// ---------------------------------------------------------------------------
// Finds a road path from (sx,sy) to the first road tile within `reach` (Manhattan) of (tx,ty).
static int FindPath(int sx, int sy, int tx, int ty, int reach, int *out) {
    static int prev[MAP_W * MAP_H];
    static int queue[MAP_W * MAP_H];
    for (int i = 0; i < MAP_W * MAP_H; i++) prev[i] = -2;
    int head = 0, tail = 0, goal = -1;
    int s = sy * MAP_W + sx;
    prev[s] = -1;
    queue[tail++] = s;
    while (head < tail) {
        int c = queue[head++];
        int cx = c % MAP_W, cy = c / MAP_W;
        if (abs(cx - tx) + abs(cy - ty) <= reach) { goal = c; break; }
        for (int d = 0; d < 4; d++) {
            int nx = cx + DX[d], ny = cy + DY[d];
            if (!IsRoad(nx, ny)) continue;
            int n = ny * MAP_W + nx;
            if (prev[n] != -2) continue;
            prev[n] = c;
            queue[tail++] = n;
        }
    }
    if (goal < 0) return 0;
    int len = 0;
    for (int c = goal; c != -1; c = prev[c]) out[len++] = c;
    for (int i = 0; i < len / 2; i++) { int tmp = out[i]; out[i] = out[len - 1 - i]; out[len - 1 - i] = tmp; }
    return len;
}

static bool Dispatch(Responder *pool, int poolN, Station *stn, int nStn, int tx, int ty,
                     int pedIdx, int reach, int mode, int carId) {
    if (nStn == 0) return false;
    int best = -1, bd = 1 << 30;
    for (int i = 0; i < nStn; i++) {
        int d = abs(stn[i].x - tx) + abs(stn[i].y - ty);
        if (d < bd) { bd = d; best = i; }
    }
    int slot = -1;
    for (int i = 0; i < poolN; i++) if (!pool[i].active) { slot = i; break; }
    if (slot < 0) return false;
    Responder *r = &pool[slot];
    int len = FindPath(stn[best].rx, stn[best].ry, tx, ty, reach, r->path);
    if (len <= 0) return false;
    r->len = len; r->idx = 0; r->t = 0.0f; r->work = 0.0f; r->scanT = 0.0f; r->lostT = 0.0f;
    r->active = true; r->arrived = (len == 1); r->spraying = false;
    r->targetX = tx; r->targetY = ty; r->pedTarget = pedIdx; r->mode = mode; r->carId = carId;
    r->x = stn[best].rx + 0.5f; r->y = stn[best].ry + 0.5f; r->angle = 0.0f;
    return true;
}

static bool MoveResponder(Responder *r, float dt, float speed) {
    if (r->idx < r->len - 1) {
        // Approaching a signalled junction: on red the crew creeps through with sirens on. In the smart city the
        // junction has already been switched to green for them (see UpdateSignals), so they keep full speed.
        int nb = r->path[r->idx + 1], nbx = nb % MAP_W, nby = nb / MAP_W;
        if (ctrl[nby][nbx] == C_SIGNAL && r->t > 0.5f) {
            int axis = (r->path[r->idx] % MAP_W == nbx) ? 0 : 1;
            if (SigState(nbx, nby, axis, NULL) != 1) speed *= 0.30f;
        }
        int idx0 = r->idx;
        r->t += dt * speed;
        while (r->t >= 1.0f && r->idx < r->len - 1) { r->t -= 1.0f; r->idx++; }
        if (r->idx != idx0 && r != &binTruck) {                     // just passed a junction: green wave or red crossing?
            int j = r->path[r->idx], jx = j % MAP_W, jy = j / MAP_W;
            if (ctrl[jy][jx] == C_SIGNAL && r->idx > 0) {
                int axis = (r->path[r->idx - 1] % MAP_W == jx) ? 0 : 1;
                if (SigState(jx, jy, axis, NULL) == 1) { if (smartCity && sigPre[jy][jx]) preemptWins++; }
                else redRuns++;
            }
        }
    }
    int a = r->path[r->idx];
    float ax = a % MAP_W + 0.5f, ay = a / MAP_W + 0.5f;
    if (r->idx >= r->len - 1) { r->x = ax; r->y = ay; return true; }
    int b = r->path[r->idx + 1];
    float bx = b % MAP_W + 0.5f, by = b / MAP_W + 0.5f;
    r->x = ax + (bx - ax) * r->t;
    r->y = ay + (by - ay) * r->t;
    r->angle = atan2f(by - ay, bx - ax) * RAD2DEG;
    return false;
}

// ---------------------------------------------------------------------------
// Vehicles: spawning, rules of the road, offences and reports
// ---------------------------------------------------------------------------
static int FindCarById(int id) {
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active && cars[i].id == id) return i;
    return -1;
}

static void ReleaseClaim(Car *c) {
    if (c->claimX >= 0) {
        if (nodeOcc[c->claimY][c->claimX] > 0) nodeOcc[c->claimY][c->claimX]--;
        c->claimX = c->claimY = -1;
    }
}
static void KillCar(Car *c) { ReleaseClaim(c); c->active = false; }

static int RandTrafficType(void) {
    int r = GetRandomValue(0, 99);
    if (r < 29) return V_SALOON;
    if (r < 50) return V_HATCH;
    if (r < 62) return V_ESTATE;
    if (r < 72) return V_PICKUP;
    if (r < 88) return V_SUV;
    if (r < 97) return V_VAN;
    return V_TRUCK;
}

static int SpawnCarAt(int x, int y, int type, int colIdx, bool allowRogue) {
    if (!IsRoad(x, y) || ctrl[y][x] != C_NONE) return -1;
    for (int j = 0; j < MAX_CARS; j++)
        if (cars[j].active && fabsf(cars[j].x - (x + 0.5f)) < 0.45f && fabsf(cars[j].y - (y + 0.5f)) < 0.45f) return -1;
    int slot = -1;
    for (int i = 0; i < MAX_CARS; i++) if (!cars[i].active) { slot = i; break; }
    if (slot < 0) return -1;
    int valid[4], nv = 0;
    for (int d = 0; d < 4; d++) if (IsRoad(x + DX[d], y + DY[d])) valid[nv++] = d;
    if (nv == 0) return -1;
    int dirPick = valid[GetRandomValue(0, nv - 1)];
    float spYaw = FaceYaw(dirPick), spL = VLEN[type], spW = VWID[type], spX = x + 0.5f, spY = y + 0.5f;
    for (int j = 0; j < MAX_CARS; j++)
        if (cars[j].active && VehOverlap(spX, spY, spYaw, spL, spW, cars[j].x, cars[j].y, cars[j].yaw, VLEN[cars[j].type], VWID[cars[j].type])) return -1;
    for (int j = 0; j < MAX_UCARS; j++)
        if (ucars[j].active && VehOverlap(spX, spY, spYaw, spL, spW, ucars[j].x, ucars[j].z, ucars[j].yaw, VLEN[ucars[j].type], VWID[ucars[j].type])) return -1;
    Car *c = &cars[slot];
    memset(c, 0, sizeof(*c));
    c->active = true; c->id = carSerial++;
    c->type = (unsigned char)type; c->colIdx = (unsigned char)colIdx;
    c->col = CAR_COLS[colIdx];
    c->dir = dirPick;
    c->gx = c->tx = x; c->gy = c->ty = y; c->t = 1.0f;
    c->x = x + 0.5f; c->y = y + 0.5f; c->yaw = FaceYaw(c->dir);
    c->claimX = c->claimY = -1; c->pursuer = -1; c->destX = c->destY = -1;
    c->speedMul = 0.88f + Rf() * 0.24f;
    if (allowRogue && type != V_TRUCK) {
        c->rogue = (Rf() < 0.07f);
        c->speeder = (type != V_BIKE) && (Rf() < 0.07f);
    }
    return slot;
}

// A rule-breaking car has been seen: if a pedestrian or cyclist is near enough they may report it.
static void Violation(Car *c, int kind) {
    offences++;
    if (c->wanted || c->reportT > 0.0f || c->fleeing) return;
    bool witness = false;
    for (int i = 0; i < MAX_PEDS && !witness; i++) {
        if (!peds[i].active || peds[i].robbing) continue;
        float dx = peds[i].x - c->x, dy = peds[i].y - c->y;
        if (dx * dx + dy * dy < 3.4f * 3.4f) witness = true;
    }
    for (int i = 0; i < MAX_CARS && !witness; i++) {
        Car *o = &cars[i];
        if (!o->active || o == c || o->type != V_BIKE) continue;
        float dx = o->x - c->x, dy = o->y - c->y;
        if (dx * dx + dy * dy < 3.4f * 3.4f) witness = true;
    }
    if (witness && Rf() < 0.6f) { c->reportT = 1.2f + Rf() * 2.0f; c->reportKind = kind; }
}

static bool LaunchDrone(int kind, float tx, float ty, int pedIdx, int carId);

static void ReportCar(Car *c) {
    c->wanted = true;
    reports++;
    bool ok = smartCity && LaunchDrone(D_POLICE, c->x, c->y, -1, c->id);        // storms ground the drones: a patrol car goes instead
    if (!ok) ok = Dispatch(police, MAX_POLICE, policeStn, nPolice, c->gx, c->gy, -1, 0, 1, c->id);
    if (!ok) c->wanted = false;
}

// ---------------------------------------------------------------------------
// Smart routing: the road monitors report congestion and every car is steered along the
// quickest route to its destination (Dijkstra over the road graph, cost grows with traffic).
// ---------------------------------------------------------------------------
static int roadList[MAP_W * MAP_H], nRoadList = 0;
static unsigned char stopWhy[MAX_CARS];      // diagnostics: 1 signal, 2 roundabout, 3 stop sign, 4 pedestrians, 5 queue

static float queueMap[MAP_H][MAP_W];         // smoothed number of stopped cars per tile (from the road monitors)

static float StepCost(int x, int y, bool useCong) {
    float c = 1.0f;
    if (ctrl[y][x] == C_SIGNAL) c += 5.0f; else if (ctrl[y][x] == C_STOP) c += 2.5f; else if (ctrl[y][x] == C_ROUNDABOUT) c += 1.2f;
    if (useCong) c += 2.0f * traffic[y][x] + 5.0f * queueMap[y][x];
    return c;
}

static void RouteField(int gx, int gy, bool useCong, float *dist) {
    static bool done[MAP_W * MAP_H];
    if (nRoadList == 0) for (int i = 0; i < MAP_W * MAP_H; i++) if (IsRoad(i % MAP_W, i / MAP_W)) roadList[nRoadList++] = i;
    for (int k = 0; k < nRoadList; k++) { dist[roadList[k]] = 1e9f; done[roadList[k]] = false; }
    dist[gy * MAP_W + gx] = 0.0f;
    for (;;) {
        int b = -1; float bd = 1e9f;
        for (int k = 0; k < nRoadList; k++) { int i = roadList[k]; if (!done[i] && dist[i] < bd) { bd = dist[i]; b = i; } }
        if (b < 0) break;
        done[b] = true;
        int bx = b % MAP_W, by = b / MAP_W;
        float step = StepCost(bx, by, useCong);
        for (int d = 0; d < 4; d++) {
            int nx = bx + DX[d], ny = by + DY[d];
            if (!IsRoad(nx, ny)) continue;
            int n = ny * MAP_W + nx;
            if (!done[n] && bd + step < dist[n]) dist[n] = bd + step;
        }
    }
}

static int SmartChoose(Car *c, const int *opts, int no) {
    if (no <= 0) return -1;
    if (c->destX < 0 || (c->gx == c->destX && c->gy == c->destY)) {
        c->destX = -1;
        for (int k = 0; k < 30; k++) {
            int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1);
            if (IsRoad(x, y) && abs(x - c->gx) + abs(y - c->gy) >= 7) { c->destX = x; c->destY = y; break; }
        }
        if (c->destX < 0) return -1;
    }
    static float dC[MAP_W * MAP_H], dF[MAP_W * MAP_H];
    RouteField(c->destX, c->destY, true, dC);
    RouteField(c->destX, c->destY, false, dF);
    int bestC = -1, bestF = -1; float sC = 1e9f, sF = 1e9f;
    for (int k = 0; k < no; k++) {
        int nx = c->gx + DX[opts[k]], ny = c->gy + DY[opts[k]];
        int n = ny * MAP_W + nx;
        float jit = Rf() * 0.02f;
        float vc = StepCost(nx, ny, true) + dC[n] + jit, vf = StepCost(nx, ny, false) + dF[n] + jit;
        if (vc < sC) { sC = vc; bestC = opts[k]; }
        if (vf < sF) { sF = vf; bestF = opts[k]; }
    }
    if (bestC >= 0 && bestC != bestF) reroutes++;       // the monitors steered this car off the shortest route
    return bestC;
}

// Free space ahead of a car before it reaches the minimum following distance of the nearest vehicle or
// walking player in its lane. Negative = already inside that distance (held up). 99 = nothing ahead.
static float LaneFreeGap(const Car *c) {
    float dxv = (float)DX[c->dir], dyv = (float)DY[c->dir];
    float best = 99.0f;
    if (fp.valid && FpActive() && (fp.state == FP_WALK || fp.state == FP_ENTER)) {
        float rx = fp.x - c->x, ry = fp.z - c->y;
        float along = rx * dxv + ry * dyv, side = fabsf(-rx * dyv + ry * dxv);
        float need = 0.5f * VLEN[c->type] + 0.10f;
        if (along > 0.02f && side < 0.10f) best = fminf(best, along - need);
    }
    for (int i = 0; i < MAX_UCARS; i++) {
        const UCar *u = &ucars[i];
        if (!u->active) continue;
        float rx = u->x - c->x, ry = u->z - c->y;
        float along = rx * dxv + ry * dyv, side = fabsf(-rx * dyv + ry * dxv);
        float need = 0.5f * (VLEN[c->type] + VLEN[u->type]) + 0.07f;
        if (along > 0.02f && side < 0.10f) best = fminf(best, along - need);
    }
    for (int j = 0; j < MAX_CARS; j++) {
        const Car *o = &cars[j];
        if (o == c || !o->active || o->dir != c->dir || o->arcing) continue;
        float rx = o->x - c->x, ry = o->y - c->y;
        float along = rx * dxv + ry * dyv;
        float side = fabsf(-rx * dyv + ry * dxv);
        float need = 0.5f * (VLEN[c->type] + VLEN[o->type]) + 0.07f;
        if (along > 0.02f && side < 0.09f) best = fminf(best, along - need);
    }
    return best;
}

static int  ShuttleChoose(Car *c, const int *opts, int no);
static void ShuttleArrive(Car *c);
#define STOP_LINE 0.70f
// turning into a lane: is a vehicle already in that lane close to the entry point (so the turn would overlap it)?
static bool EntryBlocked(const Car *c, int dir) {
    float dxv = (float)DX[dir], dyv = (float)DY[dir];
    float lane = (c->type == V_BIKE) ? LANE_BIKE : LANE_CAR;
    float px = c->gx + 0.5f + DY[dir] * lane, py = c->gy + 0.5f - DX[dir] * lane;
    for (int j = 0; j < MAX_CARS; j++) {
        const Car *o = &cars[j];
        if (o == c || !o->active || o->dir != dir || o->arcing) continue;
        float rx = o->x - px, ry = o->y - py, along = rx * dxv + ry * dyv, side = fabsf(-rx * dyv + ry * dxv);
        float need = 0.5f * (VLEN[c->type] + VLEN[o->type]) + 0.07f;
        if (along > -0.05f && along < need && side < 0.12f) return true;
    }
    for (int j = 0; j < MAX_UCARS; j++) {
        const UCar *u = &ucars[j];
        if (!u->active) continue;
        float rx = u->x - px, ry = u->z - py, along = rx * dxv + ry * dyv, side = fabsf(-rx * dyv + ry * dxv);
        float need = 0.5f * (VLEN[c->type] + VLEN[u->type]) + 0.07f;
        if (along > -0.05f && along < need && side < 0.12f) return true;
    }
    return false;
}

// would a car at (x,y,yaw) newly overlap another vehicle? (it may already be touching one, then it may move on to separate)
static bool StepHitsVehicle(const Car *c, float x, float y, float yaw) {
    float L = VLEN[c->type], W = VWID[c->type];
    for (int j = 0; j < 0; j++) {   // moving traffic is handled by the lane gap
        const Car *o = &cars[j];
        if (o == c || !o->active || o->arcing || fabsf(o->x - x) > 0.6f || fabsf(o->y - y) > 0.6f) continue;
        if (VehOverlap(x, y, yaw, L, W, o->x, o->y, o->yaw, VLEN[o->type], VWID[o->type])
            && !VehOverlap(c->x, c->y, c->yaw, L, W, o->x, o->y, o->yaw, VLEN[o->type], VWID[o->type])) return true;
    }
    for (int j = 0; j < MAX_UCARS; j++) {
        const UCar *u = &ucars[j];
        if (!u->active || fabsf(u->x - x) > 0.6f || fabsf(u->z - y) > 0.6f) continue;
        if (VehOverlap(x, y, yaw, L, W, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type])
            && !VehOverlap(c->x, c->y, c->yaw, L, W, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type])) return true;
    }
    return false;
}

// roundabout: may the car move to (x,y,yaw) without overlapping a vehicle it isn't already touching?
static bool ArcStepClear(const Car *c, float x, float y, float yaw) {
    float L = VLEN[c->type], W = VWID[c->type];
    for (int j = 0; j < MAX_CARS; j++) {
        const Car *o = &cars[j];
        if (o == c || !o->active || fabsf(o->x - x) > 0.6f || fabsf(o->y - y) > 0.6f) continue;
        if (!VehOverlap(x, y, yaw, L, W, o->x, o->y, o->yaw, VLEN[o->type], VWID[o->type])) continue;
        if (!VehOverlap(c->x, c->y, c->yaw, L, W, o->x, o->y, o->yaw, VLEN[o->type], VWID[o->type])) return false;
    }
    for (int j = 0; j < MAX_UCARS; j++) {
        const UCar *u = &ucars[j];
        if (!u->active || fabsf(u->x - x) > 0.6f || fabsf(u->z - y) > 0.6f) continue;
        if (!VehOverlap(x, y, yaw, L, W, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type])) continue;
        if (!VehOverlap(c->x, c->y, c->yaw, L, W, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type])) return false;
    }
    return true;
}

static void UpdateCars(float dt) {
    static float cnt[MAP_H][MAP_W], cntStop[MAP_H][MAP_W];
    memset(cnt, 0, sizeof(cnt)); memset(cntStop, 0, sizeof(cntStop));

    int active = 0, bikes = 0;
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) { if (cars[i].type == V_BIKE) bikes++; else active++; }
    int target = (int)(62 * TrafficFactor()) - (smartCity ? modalCars : 0);      // shuttle riders leave their cars at home
    int bikeTarget = 4 + (int)(8.0f * Clamp(st.daylight, 0.0f, 1.0f)) - (int)(rain * 5.0f);

    for (int tries = 0; tries < 6; tries++) {
        bool wantBike = (bikes < bikeTarget);
        if (!wantBike && active >= target) break;
        int rx = GetRandomValue(0, MAP_W - 1), ry = GetRandomValue(0, MAP_H - 1);
        if (!IsRoad(rx, ry)) continue;
        int type = wantBike ? V_BIKE : RandTrafficType();
        int col = (type == V_VAN || type == V_TRUCK) ? 1 : GetRandomValue(0, N_CARCOL - 1);
        if (SpawnCarAt(rx, ry, type, col, true) >= 0) { if (wantBike) bikes++; else active++; }
        break;
    }
    if (active > target + 3) {
        for (int i = 0; i < MAX_CARS; i++) {
            Car *c = &cars[i];
            if (c->active && c->type != V_BIKE && !c->wanted && !c->fleeing && !c->shuttle) { KillCar(c); break; }
        }
    }

    int n = 0, jammed = 0, nb = 0;
    float spdSum = 0.0f;
    for (int i = 0; i < MAX_CARS; i++) {
        Car *c = &cars[i];
        if (!c->active) continue;
        n++;
        if (c->type == V_BIKE) nb++;

        if (c->reportT > 0.0f) { c->reportT -= dt; if (c->reportT <= 0.0f && !c->wanted) ReportCar(c); }
        if (c->lostT > 0.0f) c->lostT -= dt;

        float ox0 = c->x, oy0 = c->y;
        float base = (c->type == V_BIKE) ? 1.15f : (c->type == V_TRUCK ? 1.7f : 2.0f);
        base *= c->speedMul;
        if (c->speeder) base *= 1.75f;
        if (c->fleeing) base = 4.5f;
        float v = base;
        c->braking = false;

        if (c->arcing) {
            // ---- going round a roundabout ------------------------------------
            const float R = 0.30f;
            float nu = Clamp(c->arcU + dt * v / (c->arcSweep * R), 0.0f, 1.0f);
            float nth = c->arcFrom + c->arcSweep * nu;
            float nx = c->gx + 0.5f + R * cosf(nth), ny = c->gy + 0.5f + R * sinf(nth);
            float nyaw = atan2f(-sinf(nth), cosf(nth)) * RAD2DEG;
            if (ArcStepClear(c, nx, ny, nyaw)) c->arcU = nu;        // hold on the ring while another car is in the way
            else { c->wait += dt; c->braking = true; }
            float th = c->arcFrom + c->arcSweep * Clamp(c->arcU, 0.0f, 1.0f);
            c->x = c->gx + 0.5f + R * cosf(th);
            c->y = c->gy + 0.5f + R * sinf(th);
            c->yaw = atan2f(-sinf(th), cosf(th)) * RAD2DEG;
            if (c->arcU >= 1.0f) { c->arcing = false; c->t = 0.0f; c->wait = 0.0f; if (c->claimX >= 0) ReleaseClaim(c); }
        } else {
            // ---- rules of the road: stop lines, give way, pedestrian crossings -----------
            bool must = false; int why = 0;
            int tx = c->tx, ty = c->ty;
            bool moving = (tx != c->gx || ty != c->gy);
            if (moving && crossT[ty][tx] && c->t < 0.98f) {
                bool axisOK = (crossT[ty][tx] == 1) ? (c->dir < 2) : (c->dir >= 2);
                if (axisOK) {
                    int ji = crossI[ty][tx];
                    bool toward = (ji >= 0 && tx + DX[c->dir] == ji % MAP_W && ty + DY[c->dir] == ji / MAP_W);
                    float prevT = c->t;
                    if (toward) {
                        int ix = ji % MAP_W, iy = ji / MAP_W;
                        if (ctrl[iy][ix] == C_SIGNAL) {
                            int s = SigState(ix, iy, c->dir < 2 ? 0 : 1, NULL);
                            if (s != 1 && !c->rogue && !c->fleeing && c->t <= STOP_LINE + 0.08f) { must = true; why = 1; }
                            if (s == 0 && (c->rogue || c->fleeing) && prevT < STOP_LINE && prevT + dt * v >= STOP_LINE && c->lostT <= 0.0f) {
                                Violation(c, 1); c->lostT = 6.0f;
                            }
                        } else if (ctrl[iy][ix] == C_ROUNDABOUT) {
                            if (c->claimX < 0 && !c->fleeing) {
                                if (nodeOcc[iy][ix] > 0 && c->t >= STOP_LINE - 0.02f) { must = true; why = 2; }
                                else if (c->t >= STOP_LINE - 0.02f) { c->claimX = ix; c->claimY = iy; nodeOcc[iy][ix]++; }
                            }
                        } else if (ctrl[iy][ix] == C_STOP) {
                            if (c->claimX < 0 && !c->rogue && !c->fleeing) {
                                if (c->t >= STOP_LINE - 0.02f) {
                                    c->stopT += dt;
                                    if (c->stopT < 1.0f || nodeOcc[iy][ix] > 0) { must = true; why = 3; }
                                    else { c->claimX = ix; c->claimY = iy; nodeOcc[iy][ix]++; }
                                }
                            }
                        }
                    }
                    if (crossBusy[ty][tx] > 0 && c->t <= STOP_LINE + 0.12f) { must = true; why = 4; }     // give way to pedestrians
                }
            }
            if (must && c->t >= STOP_LINE - 0.03f) { v = 0.0f; c->braking = true; }
            else if (must) v = fminf(v, 1.2f);

            if (c->shuttle && c->shDwell > 0.0f) { c->shDwell -= dt; v = 0.0f; c->braking = true; }     // passengers boarding
            // car following: never step into the minimum gap, so queued cars can't overlap
            float free = LaneFreeGap(c);
            if (free < 0.005f) { v = 0.0f; c->braking = true; c->wait += dt; }
            else if (free < v * dt) v = free / dt;          // brake early enough to stop exactly at the gap
            if (v == 0.0f) { jammed++; stopWhy[i] = (unsigned char)(c->braking && why ? why : 5); } else stopWhy[i] = 0;

            if (c->speeder && v > 2.9f && !c->wanted && c->lostT <= 0.0f && !c->fleeing) { Violation(c, 2); c->lostT = 8.0f; }

            // last check before stepping: never move onto another vehicle, whatever lane or junction it is in
            if (v > 0.0f && c->t + dt * v < 1.0f && ctrl[c->gy][c->gx] != C_ROUNDABOUT && ctrl[c->ty][c->tx] != C_ROUNDABOUT) {
                float nt = c->t + dt * v;
                float ax = c->gx + 0.5f, ay = c->gy + 0.5f, bx = c->tx + 0.5f, by = c->ty + 0.5f;
                float nx = ax + (bx - ax) * nt + c->ox, ny = ay + (by - ay) * nt + c->oy;
                if (StepHitsVehicle(c, nx, ny, c->yaw)) { v = 0.0f; c->braking = true; c->wait += dt; }
            }
            c->t += dt * v;
            if (c->claimX >= 0 && c->gx == c->claimX && c->gy == c->claimY && c->t > 0.55f) ReleaseClaim(c);

            if (c->t >= 1.0f) {
                c->t = 0.0f;
                c->gx = c->tx; c->gy = c->ty; c->stopT = 0.0f;
                if (c->shuttle) ShuttleArrive(c);
                int valid[4], nv = 0;
                for (int d = 0; d < 4; d++) if (IsRoad(c->gx + DX[d], c->gy + DY[d])) valid[nv++] = d;
                int choice = -1;
                if (nv > 0) {
                    bool straight = false;
                    for (int k = 0; k < nv; k++) if (valid[k] == c->dir) straight = true;
                    int opts[4], no = 0;
                    for (int k = 0; k < nv; k++) if (valid[k] != (c->dir ^ 1)) opts[no++] = valid[k];
                    if (c->fleeing && c->pursuer >= 0 && police[c->pursuer].active && no > 0 && Rf() < 0.85f) {
                        float bestD = -1.0f;
                        for (int k = 0; k < no; k++) {
                            float px = c->gx + DX[opts[k]] + 0.5f - police[c->pursuer].x;
                            float py = c->gy + DY[opts[k]] + 0.5f - police[c->pursuer].y;
                            float dd = px * px + py * py;
                            if (dd > bestD) { bestD = dd; choice = opts[k]; }
                        }
                    } else if (c->shuttle && no > 0 && (choice = ShuttleChoose(c, opts, no)) >= 0) {
                        // fixed shuttle loop
                    } else if (smartCity && !c->fleeing && no > 0 && (choice = SmartChoose(c, opts, no)) >= 0) {
                        // smart-routed
                    } else if (straight && Rf() < 0.6f) choice = c->dir;
                    else choice = (no > 0) ? opts[GetRandomValue(0, no - 1)] : valid[0];
                }
                if (choice >= 0 && ctrl[c->gy][c->gx] != C_ROUNDABOUT && EntryBlocked(c, choice)) { choice = -1; c->wait += dt; c->braking = true; }   // wait at the junction until the lane is clear
                if (choice >= 0) {
                    if (ctrl[c->gy][c->gx] == C_ROUNDABOUT) {
                        float fromA = atan2f((float)-DY[c->dir], (float)-DX[c->dir]);
                        float toA = atan2f((float)DY[choice], (float)DX[choice]);
                        float sweep = fmodf(toA - fromA + 4.0f * PI, 2.0f * PI);
                        if (sweep < 0.3f) sweep = 2.0f * PI;
                        c->arcing = true; c->arcU = 0.0f; c->arcFrom = fromA; c->arcSweep = sweep;
                        if (c->claimX < 0) { c->claimX = c->gx; c->claimY = c->gy; nodeOcc[c->gy][c->gx]++; }
                    }
                    c->dir = choice;
                    c->tx = c->gx + DX[choice]; c->ty = c->gy + DY[choice];
                } else { c->tx = c->gx; c->ty = c->gy; c->t = 1.0f; }
            }

            // position along the current edge (roundabout tiles are entered / left at ring radius)
            if (!c->arcing) {
                float ax = c->gx + 0.5f, ay = c->gy + 0.5f, bx = c->tx + 0.5f, by = c->ty + 0.5f;
                float rbScale = 1.0f;
                if (ctrl[c->gy][c->gx] == C_ROUNDABOUT) { ax += DX[c->dir] * 0.30f; ay += DY[c->dir] * 0.30f; rbScale = c->t; }
                if (ctrl[c->ty][c->tx] == C_ROUNDABOUT) { bx -= DX[c->dir] * 0.30f; by -= DY[c->dir] * 0.30f; rbScale = 1.0f - c->t; }
                // left-hand traffic: drive on the left of the heading; cyclists hug the kerb
                float lane = (c->type == V_BIKE) ? LANE_BIKE : LANE_CAR;
                float tox = DY[c->dir] * lane * rbScale, toy = -DX[c->dir] * lane * rbScale;
                float k = Clamp(dt * 8.0f, 0.0f, 1.0f);
                c->ox += (tox - c->ox) * k;
                c->oy += (toy - c->oy) * k;
                c->x = ax + (bx - ax) * c->t + c->ox;
                c->y = ay + (by - ay) * c->t + c->oy;
                float mx = c->x - ox0, my = c->y - oy0;
                if (mx * mx + my * my > 1e-7f) c->yaw = atan2f(mx, my) * RAD2DEG;
            }
        }
        c->speedNow = (dt > 0.0f) ? sqrtf((c->x - ox0) * (c->x - ox0) + (c->y - oy0) * (c->y - oy0)) / dt : 0.0f;

        spdSum += c->speedNow;
        int cx = (int)c->x, cy = (int)c->y;
        if (InBounds(cx, cy)) { cnt[cy][cx] += 1.0f; if (c->speedNow < 0.15f) cntStop[cy][cx] += 1.0f; }
        if (c->speedNow > 0.05f) c->wait = 0.0f;                                        // 'wait' = time held without really moving
        if (c->wait > 20.0f && !c->wanted && !c->fleeing && !c->shuttle) KillCar(c);     // held 20 s with no way through: remove it rather than let it overlap
    }
    st.vehicles = n;
    st.jammed = jammed;
    avgSpeed += ((n ? spdSum / n : 0.0f) - avgSpeed) * Clamp(dt * 0.4f, 0.0f, 1.0f);
    nBikes = nb;

    float k = Clamp(dt * 2.0f, 0.0f, 1.0f);
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) { traffic[y][x] += (cnt[y][x] - traffic[y][x]) * k; queueMap[y][x] += (cntStop[y][x] - queueMap[y][x]) * k; }
}

// ---------------------------------------------------------------------------
// Car parks (shopping centre) and driveways (gated communities)
// ---------------------------------------------------------------------------
static bool GateRoad(int id, int *rx, int *ry) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            if (zone[y][x] == id && gateTile[y][x] && FindAccess(x, y, rx, ry)) return true;
    return false;
}

static void UpdateParking(void) {
    // ---- shopping centre ------------------------------------------------------
    int occ = 0, cap = 0;
    int ax[32], ay[32], na = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] != T_PARK) continue;
            for (int s = 0; s < 8; s++) { cap++; if (lot[y][x][s]) occ++; }
            int rx, ry;
            if (FindAccess(x, y, &rx, &ry) && na < 32) { ax[na] = rx; ay[na] = ry; na++; }
        }
    float want = (0.10f + 0.75f * ShopFactor()) * cap;
    if (occ < want) {
        for (int i = 0; i < MAX_CARS; i++) {
            Car *c = &cars[i];
            if (!c->active || c->shuttle || c->type == V_BIKE || c->type == V_TRUCK || c->rogue || c->wanted || c->fleeing || c->arcing || c->t > 0.5f) continue;
            bool done = false;
            for (int d = 0; d < 4 && !done; d++) {
                int nx = c->gx + DX[d], ny = c->gy + DY[d];
                if (!InBounds(nx, ny) || surface[ny][nx] != T_PARK) continue;
                for (int s = 0; s < 8; s++)
                    if (!lot[ny][nx][s]) { lot[ny][nx][s] = (unsigned char)(c->type + 1); lotCol[ny][nx][s] = c->colIdx; KillCar(c); done = true; break; }
            }
            if (done) break;
        }
    } else if (occ > want + 2 && na > 0) {
        for (int tries = 0; tries < 30; tries++) {
            int x = GetRandomValue(19, 23), y = GetRandomValue(4, 8);
            if (surface[y][x] != T_PARK) continue;
            int s = GetRandomValue(0, 7);
            if (!lot[y][x][s]) continue;
            int a = GetRandomValue(0, na - 1);
            if (SpawnCarAt(ax[a], ay[a], lot[y][x][s] - 1, lotCol[y][x][s], true) >= 0) lot[y][x][s] = 0;
            break;
        }
    }

    // ---- gated community driveways --------------------------------------------
    int id = GetRandomValue(Z_GATED1, Z_GATED3);
    int rx = 0, ry = 0;
    if (ZoneFenced(id) || !GateRoad(id, &rx, &ry)) return;   // gated compounds keep cars off the drives
    int houses = 0, withCar = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            if (zone[y][x] == id && surface[y][x] == T_HOUSE) { houses++; if (drive[y][x]) withCar++; }
    float homeWant = houses * Clamp(1.1f - TrafficFactor() * 0.9f, 0.2f, 0.85f);
    if (withCar < homeWant) {
        for (int i = 0; i < MAX_CARS; i++) {
            Car *c = &cars[i];
            if (!c->active || c->shuttle || c->type == V_BIKE || c->type == V_TRUCK || c->rogue || c->wanted || c->fleeing || c->arcing) continue;
            if (c->gx != rx || c->gy != ry || c->t > 0.5f) continue;
            for (int tries = 0; tries < 30; tries++) {
                int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1);
                if (zone[y][x] == id && surface[y][x] == T_HOUSE && !drive[y][x]) {
                    drive[y][x] = (unsigned char)(c->type + 1); drvCol[y][x] = c->colIdx; KillCar(c); break;
                }
            }
            break;
        }
    } else if (withCar > homeWant + 1) {
        for (int tries = 0; tries < 40; tries++) {
            int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1);
            if (zone[y][x] == id && surface[y][x] == T_HOUSE && drive[y][x]) {
                if (SpawnCarAt(rx, ry, drive[y][x] - 1, drvCol[y][x], true) >= 0) drive[y][x] = 0;
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Pedestrians: walk on sidewalks, cross only at crossings, wait for the signal
// ---------------------------------------------------------------------------
#define STRIP ((0.5f * SIDEWALK_W) / TILE)      // centre of the sidewalk strip, measured from the tile edge

// Where a walker stands when entering sidewalk tile (tx,ty) while moving (dx,dy) from tile (fx,fy)
static void PedAnchor(int tx, int ty, int dx, int dy, int fx, int fy, float *ax, float *ay) {
    float lx = 0.5f, ly = 0.5f;
    if (dx != 0) { if (IsRoad(tx, ty - 1)) ly = STRIP; else if (IsRoad(tx, ty + 1)) ly = 1.0f - STRIP; }
    else         { if (IsRoad(tx - 1, ty)) lx = STRIP; else if (IsRoad(tx + 1, ty)) lx = 1.0f - STRIP; }
    if (IsRoad(fx, fy)) {                       // just stepped off a crossing
        if (dx > 0) lx = STRIP; else if (dx < 0) lx = 1.0f - STRIP;
        if (dy > 0) ly = STRIP; else if (dy < 0) ly = 1.0f - STRIP;
    }
    *ax = tx + lx; *ay = ty + ly;
}

static void SpawnPed(Ped *p) {
    for (int tries = 0; tries < 60; tries++) {
        int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1);
        if (!pedNode[y][x]) continue;
        memset(p, 0, sizeof(*p));
        p->active = true;
        p->gx = p->px = x; p->gy = p->py = y;
        int dx = (IsRoad(x, y - 1) || IsRoad(x, y + 1)) ? 1 : 0, dy = dx ? 0 : 1;
        PedAnchor(x, y, dx, dy, -99, -99, &p->x, &p->y);
        p->wn = p->wi = 0;
        p->criminal = (GetRandomValue(0, 99) < 14);
        p->look = (unsigned char)GetRandomValue(0, 255);
        p->shirt = p->criminal ? COL_CRIMINAL : SHIRT_COLS[GetRandomValue(0, 7)];
        p->pants = (Color){ (unsigned char)GetRandomValue(30, 90), (unsigned char)GetRandomValue(40, 90), (unsigned char)GetRandomValue(60, 130), 255 };
        p->phase = Rf() * 6.0f;
        return;
    }
}

static bool PedMayCross(int cx, int cy) {
    int ji = crossI[cy][cx];
    bool roadNS = (crossT[cy][cx] == 1);
    if (ji >= 0 && ctrl[ji / MAP_W][ji % MAP_W] == C_SIGNAL) {
        float redLeft = 0.0f;
        int s = SigState(ji % MAP_W, ji / MAP_W, roadNS ? 0 : 1, &redLeft);
        return s == 0 && redLeft > 4.0f;
    }
    for (int i = 0; i < MAX_CARS; i++) {
        const Car *c = &cars[i];
        if (!c->active || c->speedNow < 0.25f) continue;
        bool along = roadNS ? (c->dir < 2) : (c->dir >= 2);
        if (!along) continue;
        float dx = c->x - (cx + 0.5f), dy = c->y - (cy + 0.5f);
        if (dx * dx + dy * dy < 1.6f * 1.6f) return false;
    }
    return true;
}

static void PedPlanMove(Ped *p) {
    int kind[8], dir[8], n = 0;
    float w[8], total = 0.0f;
    for (int d = 0; d < 4; d++) {
        int nx = p->gx + DX[d], ny = p->gy + DY[d];
        if (!InBounds(nx, ny)) continue;
        if (pedNode[ny][nx]) {
            kind[n] = 0; dir[n] = d; w[n] = (nx == p->px && ny == p->py) ? 0.12f : 1.0f; total += w[n]; n++;
        } else if (IsRoad(nx, ny) && crossT[ny][nx]) {
            bool axisOK = (crossT[ny][nx] == 1) ? (d >= 2) : (d < 2);
            int fx = nx + DX[d], fy = ny + DY[d];
            if (axisOK && InBounds(fx, fy) && pedNode[fy][fx]) {
                kind[n] = 1; dir[n] = d; w[n] = (fx == p->px && fy == p->py) ? 0.08f : 0.45f; total += w[n]; n++;
            }
        }
    }
    p->wn = p->wi = 0; p->crossing = p->started = false;
    if (n == 0) return;
    float r = Rf() * total;
    int pick = 0;
    for (int i = 0; i < n; i++) { r -= w[i]; if (r <= 0.0f) { pick = i; break; } }
    int d = dir[pick];
    int nx = p->gx + DX[d], ny = p->gy + DY[d];
    if (kind[pick] == 0) {
        PedAnchor(nx, ny, DX[d], DY[d], p->gx, p->gy, &p->wx[0], &p->wy[0]);
        p->wn = 1; p->fx = nx; p->fy = ny;
    } else {
        int fx = nx + DX[d], fy = ny + DY[d];
        p->wx[0] = p->gx + (DX[d] > 0 ? 1.0f - STRIP : (DX[d] < 0 ? STRIP : 0.5f));
        p->wy[0] = p->gy + (DY[d] > 0 ? 1.0f - STRIP : (DY[d] < 0 ? STRIP : 0.5f));
        p->wx[1] = fx + (DX[d] > 0 ? STRIP : (DX[d] < 0 ? 1.0f - STRIP : 0.5f));
        p->wy[1] = fy + (DY[d] > 0 ? STRIP : (DY[d] < 0 ? 1.0f - STRIP : 0.5f));
        PedAnchor(fx, fy, DX[d], DY[d], nx, ny, &p->wx[2], &p->wy[2]);
        p->wn = 3; p->crossing = true; p->cx = nx; p->cy = ny; p->fx = fx; p->fy = fy;
    }
}

static void UpdatePeds(float dt) {
    memset(crossBusy, 0, sizeof(crossBusy));
    robbingNow = 0;
    int act = 0;
    for (int i = 0; i < MAX_PEDS; i++) if (peds[i].active) act++;
    int target = 14 + (int)(34.0f * Clamp(st.daylight, 0.0f, 1.0f)) - (int)(rain * 18.0f);

    for (int i = 0; i < MAX_PEDS; i++) {
        Ped *p = &peds[i];
        if (!p->active) {
            if (act < target && GetRandomValue(0, 1000) < 14) { SpawnPed(p); if (p->active) act++; }
            continue;
        }
        if (p->ill) {                               // collapsed: waits for help
            p->illT += dt;
            if (p->illT > 80.0f) { p->active = false; p->ill = false; deaths++; medLost++; }
            continue;
        }
        if (p->robbing) {
            robbingNow++;
            p->robTimer += dt;
            if (p->robTimer >= 3.0f) { p->robbing = false; p->cooldown = 12.0f; crimes++; }
            continue;
        }
        if (p->cooldown > 0.0f) p->cooldown -= dt;

        if (p->wi >= p->wn) {                       // arrived (or idle): pick the next move
            if (p->wn > 0) {
                p->px = p->gx; p->py = p->gy;
                p->gx = p->fx; p->gy = p->fy;
                p->wn = p->wi = 0; p->crossing = p->started = false;
            }
            PedPlanMove(p);
            if (p->wn == 0) continue;
        }
        if (p->crossing && p->wi == 1 && !p->started) {      // waiting at the kerb
            p->wait += dt;
            p->phase += 0.0f;
            if (PedMayCross(p->cx, p->cy) || p->wait > 25.0f) { p->started = true; p->wait = 0.0f; }
            else continue;
        }
        if (p->crossing && p->wi == 1) crossBusy[p->cy][p->cx]++;

        float sp = (p->criminal ? 0.8f : 0.55f) * (p->crossing && p->wi == 1 ? 1.35f : 1.0f);
        float tx = p->wx[p->wi], ty = p->wy[p->wi];
        float dx = tx - p->x, dy = ty - p->y;
        float dist = sqrtf(dx * dx + dy * dy), step = sp * dt;
        if (dist <= step) { p->x = tx; p->y = ty; p->wi++; }
        else { p->x += dx / dist * step; p->y += dy / dist * step; }
        p->phase += dt * sp * 11.0f;

        if (p->criminal && p->cooldown <= 0.0f) {
            for (int j = 0; j < MAX_PEDS; j++) {
                if (j == i || !peds[j].active || peds[j].criminal) continue;
                float ex = peds[j].x - p->x, ey = peds[j].y - p->y;
                if (ex * ex + ey * ey < 0.5f) { p->robbing = true; p->robTimer = 0.0f; break; }
            }
        }
    }
    nPedsActive = 0;
    for (int i = 0; i < MAX_PEDS; i++) if (peds[i].active) nPedsActive++;
}

// ---------------------------------------------------------------------------
// Clock, weather (unchanged)
// ---------------------------------------------------------------------------
static void UpdateClock(void) {
    float total = 8.0f + simTime / DAY_LEN * 24.0f;          // start at 08:00
    st.day = 1 + (int)(total / 24.0f);
    st.hour = fmodf(total, 24.0f);
    float s = sinf((st.hour - 6.0f) / 24.0f * 2.0f * PI);
    st.daylight = Clamp(0.5f + s * 0.8f, 0.0f, 1.0f);
    st.ambient = 12.0f + 5.0f * sinf((st.hour - 9.0f) / 24.0f * 2.0f * PI) - rain * 2.0f;
}

static void UpdateWeather(float dt) {
    nextRain -= dt;
    if (nextRain <= 0.0f && rainTimer <= 0.0f) {
        rainTimer = 20.0f + Rf() * 25.0f;
        rainPeak = 0.5f + Rf() * 0.5f;
        nextRain = 70.0f + Rf() * 90.0f;
    }
    float target = (rainTimer > 0.0f) ? rainPeak : 0.0f;
    if (rainTimer > 0.0f) rainTimer -= dt;
    rain += (target - rain) * dt * 0.6f;
    if (target == 0.0f && rain < 0.01f) rain = 0.0f;
}

static void Diffuse(float m[MAP_H][MAP_W], float spread, float retain) {
    static float tmp[MAP_H][MAP_W];
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float sum = 0.0f; int n = 0;
            for (int d = 0; d < 4; d++) {
                int nx = x + DX[d], ny = y + DY[d];
                if (InBounds(nx, ny)) { sum += m[ny][nx]; n++; }
            }
            float avg = n ? sum / n : m[y][x];
            tmp[y][x] = (m[y][x] * (1.0f - spread) + avg * spread) * retain;
        }
    memcpy(m, tmp, sizeof(tmp));
}

static void PushHistory(void) {
    float v[G_COUNT] = { st.energyDemand, st.waterDemand, st.sewerPct, st.aqi, st.temp, st.rainMm, (float)st.vehicles };
    if (!histInit) {
        for (int g = 0; g < G_COUNT; g++) for (int i = 0; i < HIST; i++) hist[g][i] = v[g];
        histInit = true;
    }
    for (int g = 0; g < G_COUNT; g++) hist[g][histIdx] = v[g];
    histIdx = (histIdx + 1) % HIST;
}

// ---------------------------------------------------------------------------
// Particles (fire, smoke, embers)
// ---------------------------------------------------------------------------
static void EmitPart(Vector3 p, Vector3 v, float life, int kind) {
    for (int i = 0; i < MAX_PART; i++) {
        if (parts[i].on) continue;
        parts[i].on = true; parts[i].p = p; parts[i].v = v;
        parts[i].life = parts[i].max = life; parts[i].kind = (unsigned char)kind;
        return;
    }
}
static void UpdateParticles(float dt) {
    for (int i = 0; i < MAX_PART; i++) {
        Particle *q = &parts[i];
        if (!q->on) continue;
        q->life -= dt;
        if (q->life <= 0.0f) { q->on = false; continue; }
        q->p = Vector3Add(q->p, Vector3Scale(q->v, dt));
        if (q->kind == 1) { q->v.x += 0.05f * dt; q->v.y *= (1.0f - 0.1f * dt); }       // smoke drifts with the wind
        if (q->kind == 2) q->v.y -= 0.35f * dt;                                         // embers fall
    }
}
static float FireBase(int x, int y) {
    int s = surface[y][x];
    if (s == T_BUILDING || s == T_HOUSE || s == T_MALL) return bHeight[y][x] * 0.9f;
    if (s == T_TREE) return 0.42f;
    return 0.05f;
}
static void EmitFire(int x, int y, float intensity, float dt) {
    float k = intensity / 100.0f;
    float base = FireBase(x, y);
    int nf = (int)(k * 26.0f * dt + Rf());
    for (int i = 0; i < nf; i++)
        EmitPart((Vector3){ x + 0.5f + (Rf() - 0.5f) * 0.5f, base, y + 0.5f + (Rf() - 0.5f) * 0.5f },
                 (Vector3){ (Rf() - 0.5f) * 0.10f, 0.30f + Rf() * 0.30f, (Rf() - 0.5f) * 0.10f }, 0.45f + Rf() * 0.45f, 0);
    int ns = (int)(k * 9.0f * dt + Rf() * 0.8f);
    for (int i = 0; i < ns; i++)
        EmitPart((Vector3){ x + 0.5f + (Rf() - 0.5f) * 0.3f, base + 0.15f, y + 0.5f + (Rf() - 0.5f) * 0.3f },
                 (Vector3){ 0.03f, 0.22f + Rf() * 0.18f, 0.0f }, 2.2f + Rf() * 1.6f, 1);
    if (k > 0.5f && Rf() < 0.6f * dt)
        EmitPart((Vector3){ x + 0.5f, base + 0.2f, y + 0.5f },
                 (Vector3){ (Rf() - 0.5f) * 0.4f, 0.5f + Rf() * 0.3f, (Rf() - 0.5f) * 0.4f }, 1.3f, 2);
}
// chimney smoke from the industrial zone and the power plant
static void EmitChimneys(float dt) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float rate = 0.0f, hh = 0.0f, ox = 0.0f, oz = 0.0f;
            if (surface[y][x] == T_BUILDING && bType[y][x] == B_IND && deco[y][x] == 0 && fireI[y][x] <= 0.0f) {
                rate = smartCity ? 0.35f : 2.4f; hh = bHeight[y][x] + 0.40f; ox = 0.2f; oz = -0.2f;      // smoke traps scrub the flue gas
            } else if (surface[y][x] == T_POWER && !smartCity) {
                rate = 1.5f + 4.0f * st.powerLoad; hh = 1.0f; ox = -0.22f; oz = 0.0f;
            }
            if (rate <= 0.0f) continue;
            int n = (int)(rate * dt + Rf());
            for (int i = 0; i < n; i++)
                EmitPart((Vector3){ x + 0.5f + ox, hh, y + 0.5f + oz }, (Vector3){ 0.02f, 0.18f + Rf() * 0.1f, 0.0f }, 3.0f + Rf() * 2.0f, 1);
        }
}

// ---------------------------------------------------------------------------
// Fire: starts small, grows slowly, spreads slowly, rain and fire crews put it out
// ---------------------------------------------------------------------------
static float fireAge[MAP_H][MAP_W];
static void Ignite(int x, int y) {
    if (!InBounds(x, y) || fireI[y][x] > 0.0f || !Flammable(x, y)) return;
    fireI[y][x] = 18.0f; fireAge[y][x] = 0.0f;
}

static void UpdateFire(float dt) {
    static bool ign[MAP_H][MAP_W];
    memset(ign, 0, sizeof(ign));
    fireTimer += dt;
    bool spread = fireTimer >= 3.0f;               // slower spread than before
    if (spread) fireTimer = 0.0f;
    st.fires = 0;

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            if (fireI[y][x] <= 0.0f) {
                fireI[y][x] = 0.0f;
                if (IsBldg(s) && health[y][x] < 100.0f) health[y][x] = fminf(100.0f, health[y][x] + 0.8f * dt);   // slow repairs
                continue;
            }
            bool building = IsBldg(s);
            float maxI = building ? 100.0f : (s == T_TREE ? 80.0f : 45.0f);
            fireAge[y][x] += dt;
            float lifeSpan = (s == T_TREE) ? 20.0f : (s == T_RUIN ? 10.0f : 14.0f);
            if (fireI[y][x] < maxI && (building || fireAge[y][x] < lifeSpan)) fireI[y][x] += 6.0f * (1.0f - rain * 0.9f) * dt;
            float decay = rain * 30.0f;
            if (!building && fireAge[y][x] >= lifeSpan) decay += 12.0f;
            fireI[y][x] -= decay * dt;

            if (building) {
                health[y][x] -= 2.6f * (fireI[y][x] / 100.0f) * dt;       // a building burns for a good while
                if (health[y][x] <= 0.0f) {
                    health[y][x] = 0.0f;
                    surface[y][x] = T_RUIN; drive[y][x] = 0;
                    fireAge[y][x] = 0.0f; fireI[y][x] = fminf(fireI[y][x], 40.0f);
                    collapses++;
                    if (bType[y][x] == B_RES) deaths += GetRandomValue(0, 2);
                    continue;
                }
            }
            if (fireI[y][x] <= 0.0f) {
                fireI[y][x] = 0.0f;
                if (s == T_TREE) surface[y][x] = T_GRASS;   // burnt out
                continue;
            }
            st.fires++;
            EmitFire(x, y, fireI[y][x], dt);

            if (spread && fireI[y][x] > 55.0f) {
                for (int d = 0; d < 4; d++) {
                    int nx = x + DX[d], ny = y + DY[d];
                    if (!InBounds(nx, ny) || fireI[ny][nx] > 0.0f || !Flammable(nx, ny)) continue;
                    int ns = surface[ny][nx];
                    float chance = (ns == T_GRASS ? 0.05f : (ns == T_TREE ? 0.11f : 0.13f)) * (1.0f - rain);
                    if (Rf() < chance) ign[ny][nx] = true;
                }
            }
        }
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) if (ign[y][x]) Ignite(x, y);
}

// ---------------------------------------------------------------------------
// Emergency responders: fire crews, and police (crime scenes, searches, pursuits)
// ---------------------------------------------------------------------------
static void Replan(Responder *r, int gx, int gy) {
    static int tmp[MAX_PATH];
    int a = r->path[r->idx];
    int b = (r->idx + 1 < r->len) ? r->path[r->idx + 1] : a;
    int len = FindPath(b % MAP_W, b / MAP_W, gx, gy, 0, tmp);
    if (len <= 0) return;
    int n = 0;
    if (b != a) r->path[n++] = a; else r->t = 0.0f;
    for (int i = 0; i < len && n < MAX_PATH; i++) r->path[n++] = tmp[i];
    r->len = n; r->idx = 0; r->arrived = (n <= 1);
}

static void ResolveWanted(Car *c, bool wasCaught) {
    c->wanted = false; c->fleeing = false; c->pursuer = -1;
    if (wasCaught) { caught++; KillCar(c); } else escaped++;
}

static void UpdatePoliceUnit(int pi, float dt) {
    Responder *r = &police[pi];
    switch (r->mode) {
    case 0:                                                 // crime scene
        if (!r->arrived) { if (MoveResponder(r, dt, 4.0f)) r->arrived = true; return; }
        r->work += dt;
        if (r->work > 0.6f) {
            int pd = r->pedTarget;
            if (pd >= 0 && peds[pd].active && peds[pd].criminal) { peds[pd].active = false; arrests++; }
            r->active = false;
        }
        break;
    case 1: {                                               // searching for a reported car
        int ci = FindCarById(r->carId);
        if (ci < 0) { r->active = false; break; }
        if (!r->arrived) { if (MoveResponder(r, dt, 4.3f)) r->arrived = true; }
        else {                                              // patrol nearby roads
            int px = r->path[r->len - 1] % MAP_W, py = r->path[r->len - 1] / MAP_W;
            for (int tries = 0; tries < 12; tries++) {
                int nx = px + GetRandomValue(-5, 5), ny = py + GetRandomValue(-5, 5);
                if (!IsRoad(nx, ny)) continue;
                int len = FindPath(px, py, nx, ny, 0, r->path);
                if (len > 1) { r->len = len; r->idx = 0; r->t = 0.0f; r->arrived = false; }
                break;
            }
        }
        r->work += dt; r->scanT += dt;
        if (r->scanT >= 0.35f) {
            r->scanT = 0.0f;
            Car *c = &cars[ci];
            float dx = c->x - r->x, dy = c->y - r->y, d2 = dx * dx + dy * dy;
            bool sight = (d2 < 3.6f * 3.6f) && (fabsf(dx) < 0.7f || fabsf(dy) < 0.7f || d2 < 1.8f * 1.8f);
            if (sight && !c->fleeing) { r->mode = 2; r->work = 0.0f; r->lostT = 0.0f; c->fleeing = true; c->pursuer = pi; }
        }
        if (r->work > 35.0f) { ResolveWanted(&cars[ci], false); r->active = false; }
    } break;
    case 2: {                                               // pursuit
        int ci = FindCarById(r->carId);
        if (ci < 0) { r->active = false; break; }
        Car *c = &cars[ci];
        r->work += dt; r->scanT += dt;
        if (r->scanT >= 0.45f) { r->scanT = 0.0f; Replan(r, c->tx, c->ty); }
        MoveResponder(r, dt, 4.7f);
        float dx = c->x - r->x, dy = c->y - r->y, d = sqrtf(dx * dx + dy * dy);
        if (d < 0.48f) { ResolveWanted(c, true); r->mode = 3; r->work = 0.0f; break; }       // caught!
        r->lostT = (d > 4.2f) ? r->lostT + dt : fmaxf(0.0f, r->lostT - dt);
        if (r->lostT > 4.5f || r->work > 40.0f || (d > 2.5f && Rf() < 0.012f * dt * 10.0f)) {   // escaped
            ResolveWanted(c, false); r->active = false;
        }
    } break;
    case 3:                                                 // arresting driver
        r->work += dt;
        if (r->work > 1.4f) r->active = false;
        break;
    }
}

static void UpdateResponders(float dt) {
    for (int t = 0; t < MAX_TRUCKS; t++) {
        Responder *r = &trucks[t];
        if (!r->active) continue;
        r->spraying = false;
        if (!r->arrived) { if (MoveResponder(r, dt, 3.3f)) r->arrived = true; continue; }

        r->work += dt;
        float bestD = 1e9f;
        int cx = (int)r->x, cy = (int)r->y;
        for (int yy = cy - 4; yy <= cy + 4; yy++)
            for (int xx = cx - 4; xx <= cx + 4; xx++) {
                if (!InBounds(xx, yy) || fireI[yy][xx] <= 0.0f) continue;
                float dx = xx + 0.5f - r->x, dy = yy + 0.5f - r->y;
                float d = sqrtf(dx * dx + dy * dy);
                if (d < 3.6f) {
                    fireI[yy][xx] -= 48.0f * dt;
                    r->spraying = true;
                    if (d < bestD) { bestD = d; r->sx = xx + 0.5f; r->sy = yy + 0.5f; }
                }
            }
        if ((!r->spraying && r->work > 1.5f) || r->work > 30.0f) r->active = false;
    }
    for (int p = 0; p < MAX_POLICE; p++) if (police[p].active) UpdatePoliceUnit(p, dt);
}

static void UpdateDispatch(void) {
    bool flyable = !(rain > DRONE_RAIN);                         // storms ground the drones
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (fireI[y][x] <= 0.0f) continue;
            bool tCov = false, dCov = false;
            for (int t = 0; t < MAX_TRUCKS; t++)
                if (trucks[t].active && abs(trucks[t].targetX - x) <= 3 && abs(trucks[t].targetY - y) <= 3) tCov = true;
            for (int k = 0; k < MAX_DRONES; k++)
                if (drones[k].active && drones[k].kind == D_FIRE && drones[k].state < 3 &&
                    (fabsf(drones[k].tx - (x + 0.5f)) <= 3.0f && fabsf(drones[k].ty - (y + 0.5f)) <= 3.0f)) dCov = true;
            if (!smartCity) { if (!tCov) Dispatch(trucks, MAX_TRUCKS, fireStn, nFire, x, y, -1, 3, 0, 0); continue; }
            if (!tCov && !dCov && flyable) LaunchDrone(D_FIRE, x + 0.5f, y + 0.5f, -1, -1);     // fire drone scrambles from the station
            // grounded drones, or a big blaze: a fire engine rolls as well (signals are switched green for it)
            if (!tCov && (!flyable || fireI[y][x] >= 75.0f)) Dispatch(trucks, MAX_TRUCKS, fireStn, nFire, x, y, -1, 3, 0, 0);
        }
    for (int i = 0; i < MAX_PEDS; i++) {
        if (!(peds[i].active && peds[i].criminal && peds[i].robbing)) continue;
        bool assigned = false;
        for (int p = 0; p < MAX_POLICE; p++) if (police[p].active && police[p].mode == 0 && police[p].pedTarget == i) assigned = true;
        for (int k = 0; k < MAX_DRONES; k++) if (drones[k].active && drones[k].kind == D_POLICE && drones[k].pedTarget == i) assigned = true;
        if (!assigned) {
            bool ok = smartCity && flyable && LaunchDrone(D_POLICE, peds[i].x, peds[i].y, i, -1);
            if (!ok) Dispatch(police, MAX_POLICE, policeStn, nPolice, (int)peds[i].x, (int)peds[i].y, i, 3, 0, 0);
        }
    }
}
// ---------------------------------------------------------------------------
// Smart city: drones (police / fire / medical), ambulances, wind + renewable energy
// ---------------------------------------------------------------------------
#define DR_CRUISE   2.7f
#define DR_SPEED    6.4f
#define BATT_KWS    420000.0f      // battery bank size in kW-seconds

// Renewable farm strip east of the map (tile x = MAP_W .. MAP_W + FARM_W - 1)
#define FARM_ROAD_Y   9
#define FARM_SOLAR_X0 MAP_W
#define FARM_SOLAR_X1 (MAP_W + 4)          // inclusive
#define FARM_SOLAR_Y0 0
#define FARM_SOLAR_Y1 7                    // inclusive
#define N_TURB        8
static const float TURB_POS[N_TURB][2] = {
    { 29.5f, 1.5f }, { 29.5f, 4.0f }, { 29.5f, 6.5f }, { 26.5f, 11.5f },
    { 28.5f, 11.5f }, { 25.8f, 14.3f }, { 27.8f, 14.3f }, { 29.6f, 14.3f }
};
#define TURB_KW       900.0f
#define SOLAR_TILE_KW 110.0f

// part of the landfill site is converted into a recycling plant in the smart city
static bool IsRecycleTile(int x, int y) { return smartCity && surface[y][x] == T_LANDFILL && x <= 20; }

static float SunK(void) { return powf(Clamp(st.daylight, 0.0f, 1.0f), 1.3f) * (1.0f - 0.7f * rain); }
static float WindFactor(void) { return Clamp((windSpeed - 0.12f) / 0.55f, 0.0f, 1.0f); }

static void UpdateWind(float dt) {
    windTimer -= dt;
    if (windTimer <= 0.0f) { windTimer = 18.0f + Rf() * 25.0f; windTarget = 0.12f + Rf() * 0.85f; }
    float tgt = Clamp(windTarget + rain * 0.25f, 0.0f, 1.0f);
    windSpeed += (tgt - windSpeed) * Clamp(dt * 0.25f, 0.0f, 1.0f);
}

// ---------------------------------------------------------------------------
// Drones
// ---------------------------------------------------------------------------
static bool LaunchDrone(int kind, float tx, float ty, int pedIdx, int carId) {
    Station *stn; int n;
    if (kind == D_POLICE)    { stn = policeStn; n = nPolice; }
    else if (kind == D_FIRE) { stn = fireStn;   n = nFire; }
    else                     { stn = hospStn;   n = nHosp; }
    if (n == 0 || rain > DRONE_RAIN) return false;                  // drones stay on their pads in a storm
    int best = 0, bd = 1 << 30;
    for (int i = 0; i < n; i++) {
        int d = abs(stn[i].x - (int)tx) + abs(stn[i].y - (int)ty);
        if (d < bd) { bd = d; best = i; }
    }
    int slot = -1;
    for (int i = 0; i < MAX_DRONES; i++) if (!drones[i].active) { slot = i; break; }
    if (slot < 0) return false;
    Drone *d = &drones[slot];
    memset(d, 0, sizeof(*d));
    d->active = true; d->kind = kind; d->state = 0;
    d->x = stn[best].x + 0.5f; d->y = stn[best].y + 0.5f;
    d->z = (kind == D_MEDIC) ? 0.70f : 0.50f;
    d->tx = tx; d->ty = ty; d->homeX = stn[best].x; d->homeY = stn[best].y;
    d->pedTarget = pedIdx; d->carId = carId; d->since = (float)simTime;
    droneSorties++;
    return true;
}

// move horizontally towards (tx,ty); returns the distance that was left before the move
static float DroneStep(Drone *d, float tx, float ty, float speed, float dt) {
    float dx = tx - d->x, dy = ty - d->y, dist = sqrtf(dx * dx + dy * dy), step = speed * dt;
    if (dist > 0.04f) d->yaw = atan2f(dx, dy) * RAD2DEG;
    if (dist <= step) { d->x = tx; d->y = ty; } else { d->x += dx / dist * step; d->y += dy / dist * step; }
    return dist;
}
static void DroneAlt(Drone *d, float target, float dt) {
    float dz = target - d->z, m = 2.4f * dt;
    d->z += Clamp(dz, -m, m);
}
static float DroneHomeZ(const Drone *d) { return (d->kind == D_MEDIC || d->kind == D_MEDSUP) ? 0.70f : (d->kind == D_PARCEL ? 0.12f : 0.50f); }

static void DroneReturn(Drone *d, float dt) {
    float dist = DroneStep(d, d->homeX + 0.5f, d->homeY + 0.5f, DR_SPEED, dt);
    if (dist > 0.6f) DroneAlt(d, DR_CRUISE, dt);
    else             DroneAlt(d, DroneHomeZ(d), dt);
    if (dist < 0.05f && d->z <= DroneHomeZ(d) + 0.03f) d->active = false;
}

static void UpdateDrone(Drone *d, float dt) {
    d->spraying = false;
    switch (d->kind) {
    case D_FIRE: {
        if (d->state == 0) {
            float dist = DroneStep(d, d->tx, d->ty, 5.8f, dt);
            DroneAlt(d, DR_CRUISE, dt);
            if (dist < 0.1f) { d->state = 1; d->work = 0.0f; d->lostT = 0.0f; }
        } else if (d->state == 1) {
            d->work += dt;
            DroneAlt(d, 2.2f, dt);
            bool any = false; float bestI = 0.0f;
            int cx = (int)d->x, cy = (int)d->y;
            for (int yy = cy - 2; yy <= cy + 2; yy++)
                for (int xx = cx - 2; xx <= cx + 2; xx++) {
                    if (!InBounds(xx, yy) || fireI[yy][xx] <= 0.0f) continue;
                    float dx = xx + 0.5f - d->x, dy = yy + 0.5f - d->y;
                    if (dx * dx + dy * dy < 1.7f * 1.7f) {
                        fireI[yy][xx] -= 52.0f * dt; any = true;
                        if (fireI[yy][xx] > bestI) { bestI = fireI[yy][xx]; d->sx = xx + 0.5f; d->sy = yy + 0.5f; }
                    }
                }
            d->spraying = any;
            if (any) d->lostT = 0.0f;
            else {
                d->lostT += dt;
                float bd = 1e9f; float nx = 0, ny = 0; bool found = false;     // look for the next burning tile nearby
                for (int yy = cy - 4; yy <= cy + 4; yy++)
                    for (int xx = cx - 4; xx <= cx + 4; xx++) {
                        if (!InBounds(xx, yy) || fireI[yy][xx] <= 0.0f) continue;
                        float dx = xx + 0.5f - d->x, dy = yy + 0.5f - d->y, dd = dx * dx + dy * dy;
                        if (dd < bd) { bd = dd; nx = xx + 0.5f; ny = yy + 0.5f; found = true; }
                    }
                if (found) { DroneStep(d, nx, ny, 3.5f, dt); d->lostT = 0.0f; }
            }
            if (d->lostT > 1.5f || d->work > 45.0f) d->state = 3;
        } else DroneReturn(d, dt);
    } break;

    case D_POLICE: {
        if (d->state == 3) { DroneReturn(d, dt); break; }
        if (d->pedTarget >= 0) {                                   // crime in progress
            Ped *p = &peds[d->pedTarget];
            if (!p->active || !p->criminal) { d->state = 3; break; }
            float dist = DroneStep(d, p->x, p->y, DR_SPEED, dt);
            DroneAlt(d, dist < 1.0f ? 1.1f : DR_CRUISE, dt);
            if (dist < 0.35f) {
                d->work += dt;
                if (d->work > 1.6f) { p->active = false; arrests++; d->state = 3; }
            }
            d->sx = p->x; d->sy = p->y; d->spraying = (dist < 2.5f);      // spotlight beam
        } else {                                                   // reported vehicle: track it from above
            int ci = FindCarById(d->carId);
            if (ci < 0) { d->state = 3; break; }
            Car *c = &cars[ci];
            float dist = DroneStep(d, c->x, c->y, DR_SPEED, dt);
            DroneAlt(d, dist < 3.0f ? 1.3f : DR_CRUISE, dt);
            d->work += dt;
            d->sx = c->x; d->sy = c->y; d->spraying = (dist < 3.5f);
            if (dist < 3.2f && !c->fleeing) { c->fleeing = true; c->pursuer = -1; }
            d->lostT = (dist < 0.8f) ? d->lostT + dt : fmaxf(0.0f, d->lostT - dt);
            if (d->lostT > 1.6f) {                                  // locked on: car is stopped
                bool got = Rf() > (0.10f + rain * 0.30f);           // storms can break the lock
                ResolveWanted(c, got); d->state = 3;
            } else if (d->work > 45.0f) { ResolveWanted(c, false); d->state = 3; }
        }
    } break;

    case D_MEDIC: {
        Ped *p = (d->pedTarget >= 0) ? &peds[d->pedTarget] : NULL;
        if (d->state == 0) {
            if (!p || !p->active || !p->ill) { d->state = 3; break; }
            float dist = DroneStep(d, p->x, p->y, DR_SPEED, dt);
            DroneAlt(d, dist < 0.3f ? 0.12f : DR_CRUISE, dt);
            if (dist < 0.12f && d->z < 0.2f) {
                d->state = 1; d->work = 0.0f;
                respSum += (float)simTime - p->illSince; respN++;
            }
        } else if (d->state == 1) {
            d->work += dt;
            DroneAlt(d, 0.12f, dt);
            if (d->work > 2.0f) {
                if (p && p->active && p->ill) { d->since = p->illSince; p->active = false; d->carrying = true; d->state = 2; }
                else d->state = 3;
            }
        } else if (d->state == 2) {
            float hx = hospStn[0].x + 0.5f, hy = hospStn[0].y + 0.5f;
            float dist = DroneStep(d, hx, hy, DR_SPEED, dt);
            DroneAlt(d, dist < 0.6f ? 0.70f : DR_CRUISE, dt);
            if (dist < 0.05f && d->z <= 0.73f) {
                medSaved++; transSum += (float)simTime - d->since; transN++;
                d->carrying = false; d->active = false;
            }
        } else DroneReturn(d, dt);
    } break;

    case D_PARCEL: case D_MEDSUP: {                                // delivery drones: fly out, lower the package, come home
        if (d->state == 0) {
            float dist = DroneStep(d, d->tx, d->ty, 5.2f, dt);
            DroneAlt(d, dist < 0.5f ? 0.55f : 1.9f, dt);
            if (dist < 0.05f) { d->state = 1; d->work = 0.0f; }
        } else if (d->state == 1) {
            d->work += dt;
            DroneAlt(d, 0.30f, dt);                                 // lower to the doorstep, release the package
            if (d->work > 1.4f) {
                d->carrying = false; d->state = 3;
                for (int k = 0; k < N_DROPS; k++) if (drops[k].t <= 0.0f) { drops[k] = (DropMark){ d->x, d->y, 7.0f, d->kind }; break; }
                if (d->kind == D_PARCEL) parcelsDelivered++; else medDeliveries++;
            }
        } else DroneReturn(d, dt);
    } break;
    }
}

static void UpdateDrones(float dt) {
    for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active) UpdateDrone(&drones[i], dt);
}

// ---------------------------------------------------------------------------
// Ambulances (normal city) and the medical-emergency generator (both modes)
// ---------------------------------------------------------------------------
static void UpdateAmbulances(float dt) {
    for (int a = 0; a < MAX_AMB; a++) {
        Responder *r = &ambs[a];
        if (!r->active) continue;
        int pd = r->pedTarget;
        switch (r->mode) {
        case 0:                                                    // driving to the patient
            if (pd < 0 || !peds[pd].active || !peds[pd].ill) { r->active = false; break; }
            if (MoveResponder(r, dt, 3.6f)) { respSum += (float)simTime - peds[pd].illSince; respN++; r->mode = 1; r->work = 0.0f; }
            break;
        case 1:                                                    // paramedics load the patient
            r->work += dt;
            if (r->work > 2.2f) {
                if (peds[pd].active && peds[pd].ill) { r->lostT = peds[pd].illSince; peds[pd].active = false; }
                int cx = r->path[r->len - 1] % MAP_W, cy = r->path[r->len - 1] / MAP_W;
                int len = FindPath(cx, cy, hospStn[0].rx, hospStn[0].ry, 0, r->path);
                if (len <= 0) { r->active = false; break; }
                r->len = len; r->idx = 0; r->t = 0.0f; r->arrived = (len == 1); r->mode = 2; r->work = 0.0f;
            }
            break;
        case 2:                                                    // blue-light run to the hospital
            if (MoveResponder(r, dt, 3.6f)) { r->mode = 3; r->work = 0.0f; }
            break;
        case 3:
            r->work += dt;
            if (r->work > 1.6f) { medSaved++; transSum += (float)simTime - r->lostT; transN++; r->active = false; }
            break;
        }
    }
}

static void UpdateMedical(float dt) {
    medTimer -= dt;
    if (medTimer <= 0.0f) {                                        // somebody collapses
        medTimer = 13.0f + Rf() * 16.0f;
        for (int tries = 0; tries < 40; tries++) {
            int i = GetRandomValue(0, MAX_PEDS - 1);
            Ped *p = &peds[i];
            if (!p->active || p->ill || p->criminal || p->robbing) continue;
            int tx = (int)p->x, ty = (int)p->y;
            if (!InBounds(tx, ty) || !pedNode[ty][tx] || (p->wn > 0 && p->crossing)) continue;
            p->ill = true; p->illCalled = false; p->illT = 0.0f; p->illSince = (float)simTime; medCalls++;
            break;
        }
    }
    for (int i = 0; i < MAX_PEDS; i++) {                           // smoke inhalation near a blaze
        Ped *p = &peds[i];
        if (!p->active || p->ill || p->robbing) continue;
        int tx = (int)p->x, ty = (int)p->y;
        bool nearFire = false;
        for (int dy = -1; dy <= 1 && !nearFire; dy++)
            for (int dx = -1; dx <= 1; dx++)
                if (InBounds(tx + dx, ty + dy) && fireI[ty + dy][tx + dx] > 40.0f) { nearFire = true; break; }
        if (nearFire && pedNode[ty][tx] && !(p->wn > 0 && p->crossing) && Rf() < 0.35f * dt) {
            p->ill = true; p->illCalled = false; p->illT = 0.0f; p->illSince = (float)simTime; medCalls++;
        }
    }
    for (int i = 0; i < MAX_PEDS; i++) {                           // raise the alarm
        Ped *p = &peds[i];
        if (!p->active || !p->ill || p->illCalled) continue;
        bool ok = smartCity && LaunchDrone(D_MEDIC, p->x, p->y, i, -1);
        if (!ok) ok = Dispatch(ambs, MAX_AMB, hospStn, nHosp, (int)p->x, (int)p->y, i, 1, 0, 0);     // normal city, or storm / all drones busy
        if (ok) p->illCalled = true;
    }
}

// ---------------------------------------------------------------------------
// Traffic signals: fixed 24 s cycle in the normal city; in the smart city the road monitors
// count the cars approaching each junction and the lights respond (gap-out / extension).
// ---------------------------------------------------------------------------
static int ApproachCount(int jx, int jy, int axis) {
    int n = 0;
    for (int i = 0; i < MAX_CARS; i++) {
        const Car *c = &cars[i];
        if (!c->active || c->arcing || (c->dir < 2 ? 0 : 1) != axis) continue;
        float dx = c->x - (jx + 0.5f), dy = c->y - (jy + 0.5f);
        float dist, side;
        if (axis == 0) { dist = -dy * DY[c->dir]; side = fabsf(dx); }
        else           { dist = -dx * DX[c->dir]; side = fabsf(dy); }
        if (side < 0.6f && dist > 0.15f && dist < 3.5f) n++;
    }
    return n;
}

// Smart city: junctions ahead of a ground emergency vehicle are switched green on its approach
static void PreemptFrom(const Responder *r) {
    if (!r->active || r->idx >= r->len - 1) return;
    for (int k = r->idx + 1; k < r->len && k <= r->idx + 4; k++) {
        int j = r->path[k], jx = j % MAP_W, jy = j / MAP_W;
        if (ctrl[jy][jx] != C_SIGNAL) continue;
        int axis = (r->path[k - 1] % MAP_W == jx) ? 0 : 1;          // travelling N-S when the previous tile is in the same column
        sigPre[jy][jx] = (unsigned char)(axis + 1);
    }
}

static void UpdateSignals(float dt) {
    static unsigned char prevPre[MAP_H][MAP_W];
    memcpy(prevPre, sigPre, sizeof(sigPre));
    memset(sigPre, 0, sizeof(sigPre));
    if (smartCity) {
        for (int i = 0; i < MAX_AMB; i++)    PreemptFrom(&ambs[i]);
        for (int i = 0; i < MAX_TRUCKS; i++) PreemptFrom(&trucks[i]);
        for (int i = 0; i < MAX_POLICE; i++) PreemptFrom(&police[i]);
    }
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (ctrl[y][x] != C_SIGNAL) continue;
            int ph = sigPhase[y][x];
            sigT[y][x] += dt;
            float t = sigT[y][x];
            bool next = false;
            if (ph == 1 || ph == 4) next = (t >= 2.0f);
            else if (ph == 2 || ph == 5) next = (t >= 1.0f);
            else if (!smartCity) next = (t >= 9.0f);
            else {
                int axis = (ph == 0) ? 0 : 1;
                int nSame = ApproachCount(x, y, axis), nOpp = ApproachCount(x, y, 1 - axis);
                if (t >= 16.0f) next = true;
                else if (t >= 5.0f && nSame == 0 && nOpp > 0) next = true;
                else if (t >= 9.0f && (nSame == 0 || nOpp >= nSame)) next = true;
            }
            if (smartCity && sigPre[y][x]) {
                if (!prevPre[y][x]) preemptCalls++;
                int gp = (sigPre[y][x] == 1) ? 0 : 3;
                if (ph == gp) next = (t >= 12.0f);                   // hold the green for the emergency vehicle
                else if (ph == (gp + 3) % 6) next = (t >= 1.5f);     // cut the cross traffic's green short
            }
            if (next) { sigPhase[y][x] = (unsigned char)((ph + 1) % 6); sigT[y][x] = 0.0f; }
        }
}
// ---------------------------------------------------------------------------
// Smart city extras - simulation
// ---------------------------------------------------------------------------
static int leaksDetected = 0;

static void InitPlus(void) {
    memset(lampLvl, 0, sizeof(lampLvl)); memset(lampTile, 0, sizeof(lampTile));
    nLamps = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            if (pedNode[y][x] && surface[y][x] != T_ROAD && ((x * 3 + y * 5) % 4 == 0) && LampDir(x, y) >= 0) { lampTile[y][x] = true; lampLvl[y][x] = 1.0f; nLamps++; }

    // smart bins: spread evenly over the sidewalks
    static int cx[400], cy[400];
    int nc = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            if (!pedNode[y][x] || LampDir(x, y) < 0 || nc >= 400) continue;
            if (s == T_GRASS || s == T_BUILDING || s == T_HOUSE || s == T_TREE) { cx[nc] = x; cy[nc] = y; nc++; }
        }
    nBins = 0;
    for (int i = 0; i < MAX_BINS && nc > 0; i++) {
        int k = (i * nc) / MAX_BINS + nc / (MAX_BINS * 2);
        if (k >= nc) k = nc - 1;
        bins[nBins++] = (Bin){ cx[k], cy[k], LampDir(cx[k], cy[k]), Rf() * 60.0f, 0.35f + Rf() * 0.55f, 0.0f };
    }
    memset(&binTruck, 0, sizeof(binTruck));
    binQn = binQi = 0; binSched = 60.0f; binCool = 0.0f;

    // district heating: every non-industrial building within reach of a main is connected
    memset(dhServed, 0, sizeof(dhServed));
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!IsBldg(surface[y][x]) || (surface[y][x] == T_BUILDING && bType[y][x] == B_IND)) continue;
            for (int k = 0; k < N_DH; k++)
                if (DistToSeg(x + 0.5f, y + 0.5f, DH_SEG[k][0], DH_SEG[k][1], DH_SEG[k][2], DH_SEG[k][3]) <= 3.2f) { dhServed[y][x] = true; break; }
        }

    memset(leaks, 0, sizeof(leaks)); memset(drops, 0, sizeof(drops)); memset(sigPre, 0, sizeof(sigPre));
    memset(floodState, 0, sizeof(floodState)); memset(aqLevel, 0, sizeof(aqLevel));
    memset(twSum, 0, sizeof(twSum)); memset(twTime, 0, sizeof(twTime));
    memset(twRespSum, 0, sizeof(twRespSum)); memset(twRespN, 0, sizeof(twRespN));
    memset(twTransSum, 0, sizeof(twTransSum)); memset(twTransN, 0, sizeof(twTransN));
    memset(twLost, 0, sizeof(twLost)); memset(twRecycle, 0, sizeof(twRecycle)); memset(twLandfill, 0, sizeof(twLandfill));
    budBalanceK = 0.0f; budIncomeK = 0.0f; budSpendK = 0.0f;
}

// ---------------------------------------------------------------------------
// Autonomous shuttles: a loop of roads (waypoints), stops with a short dwell, passengers
// ---------------------------------------------------------------------------
static int ShuttleChoose(Car *c, const int *opts, int no) {
    if (c->gx == SH_WP[c->shLeg][0] && c->gy == SH_WP[c->shLeg][1]) c->shLeg = (c->shLeg + 1) % 4;
    static float dF[MAP_W * MAP_H];
    RouteField(SH_WP[c->shLeg][0], SH_WP[c->shLeg][1], false, dF);
    int best = -1; float bv = 1e9f;
    for (int k = 0; k < no; k++) {
        int nx = c->gx + DX[opts[k]], ny = c->gy + DY[opts[k]];
        float v = StepCost(nx, ny, false) + dF[ny * MAP_W + nx];
        if (v < bv) { bv = v; best = opts[k]; }
    }
    return best;
}

static void ShuttleArrive(Car *c) {
    for (int i = 0; i < N_STOPS; i++) {
        if (c->gx != SH_STOP[i][0] || c->gy != SH_STOP[i][1]) continue;
        c->shDwell = 4.0f; shuttleStops++;
        int alight = (c->shPax > 0) ? GetRandomValue(0, c->shPax) : 0;
        int board = GetRandomValue(0, 4);
        c->shPax = (int)fminf(8.0f, (float)(c->shPax - alight + board));
        shuttleBoard += board;
        return;
    }
}

static void SpawnShuttles(void) {
    static const int start[2][3] = { {7, 3, 3}, {14, 9, 2} };          // x, y, heading (3 = east, 2 = west)
    static const int leg[2] = { 0, 2 };
    for (int k = 0; k < 2; k++) {
        for (int j = 0; j < 8; j++) {
            int x = start[k][0] + (start[k][2] == 3 ? j : -j), y = start[k][1];
            int slot = SpawnCarAt(x, y, V_VAN, 1, false);
            if (slot < 0) continue;
            Car *c = &cars[slot];
            c->shuttle = true; c->shLeg = leg[k]; c->shPax = GetRandomValue(1, 4);
            c->dir = start[k][2]; c->yaw = FaceYaw(c->dir); c->speedMul = 1.0f;
            c->col = (Color){ 0, 190, 165, 255 };
            break;
        }
    }
}

static void KillShuttles(void) {
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active && cars[i].shuttle) KillCar(&cars[i]);
    shuttlePax = 0; modalCars = 0;
}

static void SmartPlusToggled(bool on) {
    binSched = 60.0f; binCool = 0.0f;
    if (on) { SpawnShuttles(); parcelTimer = 4.0f; medSupTimer = 18.0f; }
    else {
        KillShuttles();
        for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active && drones[i].kind >= D_PARCEL) drones[i].active = false;
        drainOpen = 0.0f; runoffK = 1.0f; harvestLpm = 0.0f; v2gKW = 0.0f;
    }
}

// ---------------------------------------------------------------------------
// Smart bins + refuse truck.  Normal city: the truck does a full round on a timetable.
// Smart city: the bins report their fill level and the truck is only called for the full ones.
// ---------------------------------------------------------------------------
static bool BinTruckLeg(int tx, int ty, int reach, int mode) {
    Responder *r = &binTruck;
    static int tmp[MAX_PATH];
    int fx = r->path[r->len - 1] % MAP_W, fy = r->path[r->len - 1] / MAP_W;
    int len = FindPath(fx, fy, tx, ty, reach, tmp);
    if (len <= 0) return false;
    memcpy(r->path, tmp, (size_t)len * sizeof(int));
    r->len = len; r->idx = 0; r->t = 0.0f; r->arrived = (len == 1); r->mode = mode; r->work = 0.0f;
    binTiles += len - 1;
    return true;
}

static void BinTruckNext(void) {
    while (binQi < binQn) {
        Bin *b = &bins[binQ[binQi]];
        if (BinTruckLeg(b->x, b->y, 1, 1)) return;
        binQi++;
    }
    if (!BinTruckLeg(DEPOT_X, DEPOT_Y, 0, 3)) binTruck.active = false;
}

// A bin's fill level once the truck could get there (sensors report fill level and fill rate)
static float BinForecast(const Bin *b, float horizon) { return b->fill + b->rate * horizon * 0.9f; }

static void StartBinRound(bool onlyFull) {
    int sel[MAX_BINS], ns = 0;
    for (int i = 0; i < nBins; i++) if (!onlyFull || BinForecast(&bins[i], 150.0f) >= 95.0f) sel[ns++] = i;
    if (ns == 0) return;
    int cx = DEPOT_X, cy = DEPOT_Y;
    binQn = 0;
    while (ns > 0) {                                              // nearest-neighbour order starting at the depot
        int best = 0, bd = 1 << 30;
        for (int k = 0; k < ns; k++) { int d = abs(bins[sel[k]].x - cx) + abs(bins[sel[k]].y - cy); if (d < bd) { bd = d; best = k; } }
        binQ[binQn++] = sel[best]; cx = bins[sel[best]].x; cy = bins[sel[best]].y;
        sel[best] = sel[--ns];
    }
    Responder *r = &binTruck;
    memset(r, 0, sizeof(*r));
    r->active = true; r->path[0] = DEPOT_Y * MAP_W + DEPOT_X; r->len = 1;
    r->x = DEPOT_X + 0.5f; r->y = DEPOT_Y + 0.5f;
    binQi = 0; binTrips++;
    BinTruckNext();
}

static void UpdateBins(float dt) {
    float worst = 0.0f;
    for (int i = 0; i < nBins; i++) {
        Bin *b = &bins[i];
        b->fill = fminf(100.0f, b->fill + b->rate * dt * (0.55f + 0.9f * ShopFactor()));
        if (b->fill >= 100.0f) { b->overflowT += dt; binOverflowS += dt; }
        worst = fmaxf(worst, BinForecast(b, 34.0f));
    }
    Responder *r = &binTruck;
    if (!r->active) {
        if (!smartCity) {
            binSched -= dt;
            if (binSched <= 0.0f) { binSched = 55.0f; StartBinRound(false); }
        } else {
            binCool -= dt;
            if (binCool <= 0.0f && worst >= 100.0f) { binCool = 6.0f; StartBinRound(true); }       // somebody will be full before the truck can get there
        }
        return;
    }
    switch (r->mode) {
    case 1: if (MoveResponder(r, dt, 2.6f)) { r->mode = 2; r->work = 0.0f; } break;
    case 2:
        r->work += dt;
        if (r->work >= 1.3f) {
            Bin *b = &bins[binQ[binQi]];
            binFillSum += b->fill; binEmptied++; b->fill = 0.0f; b->overflowT = 0.0f;
            binQi++;
            BinTruckNext();
        }
        break;
    case 3: if (MoveResponder(r, dt, 2.6f)) { r->mode = 4; r->work = 0.0f; } break;
    case 4: r->work += dt; if (r->work > 1.8f) r->active = false; break;
    default: r->active = false; break;
    }
}

// ---------------------------------------------------------------------------
// Water leaks (both cities).  Smart city: acoustic sensors find them in seconds and a crew fixes them fast.
// ---------------------------------------------------------------------------
static void UpdateLeaks(float dt) {
    leakTimer -= dt;
    if (leakTimer <= 0.0f) {
        leakTimer = 14.0f + Rf() * 22.0f;
        for (int tries = 0; tries < 30; tries++) {
            int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1);
            if (!IsRoad(x, y) || !pipeNode[y][x]) continue;
            int slot = -1; bool dup = false;
            for (int i = 0; i < MAX_LEAKS; i++) { if (!leaks[i].active) { if (slot < 0) slot = i; } else if (leaks[i].x == x && leaks[i].y == y) dup = true; }
            if (slot < 0 || dup) break;
            leaks[slot] = (Leak){ true, false, x, y, 0.0f, 250.0f + Rf() * 350.0f, 40.0f + Rf() * 40.0f, 0.0f };
            leaksTotal++;
            break;
        }
    }
    float lpm = 0.0f;
    for (int i = 0; i < MAX_LEAKS; i++) {
        Leak *l = &leaks[i];
        if (!l->active) continue;
        l->age += dt;
        if (smartCity && !l->detected && l->age > 2.5f) l->detectAt = fminf(l->detectAt, l->age);
        if (!l->detected && l->age >= l->detectAt) { l->detected = true; leaksDetected++; leakAgeSum += l->age; }
        if (l->detected) { l->fixT += dt; if (l->fixT >= (smartCity ? 9.0f : 14.0f)) { l->active = false; leaksFixed++; continue; } }
        lpm += l->rate;
    }
    leakLpm = lpm;
    leakLostKL += lpm * dt / 60.0f / 1000.0f * 60.0f;                  // sim-seconds are compressed: 1 s of play = 1 minute of flow
}

// ---------------------------------------------------------------------------
// Delivery drones: parcels from the fulfilment hub, prescriptions from the hospital
// ---------------------------------------------------------------------------
static bool LaunchDelivery(int kind) {
    int act = 0, slot = -1;
    for (int i = 0; i < MAX_DRONES; i++) { if (drones[i].active) { if (drones[i].kind >= D_PARCEL) act++; } else if (slot < 0) slot = i; }
    if (act >= 4 || slot < 0 || (kind == D_MEDSUP && nHosp == 0)) return false;
    int tx = -1, ty = -1;
    for (int tries = 0; tries < 60; tries++) {
        int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1), s = surface[y][x];
        bool ok = (s == T_HOUSE) || (kind == D_PARCEL && s == T_BUILDING && bType[y][x] == B_RES);
        if (ok && health[y][x] >= 50.0f) { tx = x; ty = y; break; }
    }
    if (tx < 0) return false;
    Drone *d = &drones[slot];
    memset(d, 0, sizeof(*d));
    d->active = true; d->kind = kind; d->state = 0; d->carrying = true;
    d->homeX = (kind == D_PARCEL) ? HUB_X : hospStn[0].x;
    d->homeY = (kind == D_PARCEL) ? HUB_Y : hospStn[0].y;
    d->x = d->homeX + 0.5f; d->y = d->homeY + 0.5f; d->z = DroneHomeZ(d);
    d->tx = tx + 0.5f + (Rf() - 0.5f) * 0.3f; d->ty = ty + 0.62f;
    return true;
}

static void UpdateDelivery(float dt) {
    for (int k = 0; k < N_DROPS; k++) if (drops[k].t > 0.0f) drops[k].t -= dt;
    if (!smartCity) return;
    bool ok = rain < 0.5f && st.daylight > 0.15f;
    parcelTimer -= dt;
    if (parcelTimer <= 0.0f) { parcelTimer = 7.0f + Rf() * 8.0f; if (ok) LaunchDelivery(D_PARCEL); }
    medSupTimer -= dt;
    if (medSupTimer <= 0.0f) { medSupTimer = 32.0f + Rf() * 24.0f; if (ok) LaunchDelivery(D_MEDSUP); }
}

// ---------------------------------------------------------------------------
// Slow tick (4 Hz): lighting, EV fleet, flood / storm-drain control, air quality, parking, heating, twin
// ---------------------------------------------------------------------------
static void UpdateLamps(void) {
    float sum = 0.0f; int n = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!lampTile[y][x]) continue;
            float target = 1.0f;
            if (smartCity) {                                                // dim until somebody walks or drives past
                target = 0.30f;
                float lx = x + 0.5f, ly = y + 0.5f;
                for (int i = 0; i < MAX_CARS && target < 1.0f; i++) {
                    if (!cars[i].active) continue;
                    float dx = cars[i].x - lx, dy = cars[i].y - ly;
                    if (dx * dx + dy * dy < 2.4f * 2.4f) target = 1.0f;
                }
                for (int i = 0; i < MAX_PEDS && target < 1.0f; i++) {
                    if (!peds[i].active) continue;
                    float dx = peds[i].x - lx, dy = peds[i].y - ly;
                    if (dx * dx + dy * dy < 2.4f * 2.4f) target = 1.0f;
                }
            }
            lampLvl[y][x] += (target - lampLvl[y][x]) * (target > lampLvl[y][x] ? 0.7f : 0.15f);
            sum += lampLvl[y][x]; n++;
        }
    lampAvg = n ? sum / n : 1.0f;
}

static void UpdateWaterPlus(float dt) {
    if (smartCity) {
        float inflow = rain * (float)solarRoofs * 0.012f * dt;          // kL falling on the roofs this tick
        float take = fminf(inflow, RAIN_TANK_CAP - rainTankKL);
        if (take > 0.0f) { rainTankKL += take; harvestedKL += take; }
        bool want = rain > 0.2f && (st.sewerPct > 65.0f || rainTankKL >= RAIN_TANK_CAP * 0.95f);
        float tgt = (want && retentionKL < RETENTION_CAP - 2.0f) ? 1.0f : 0.0f;
        drainOpen += (tgt - drainOpen) * Clamp(dt * 1.2f, 0.0f, 1.0f);
        retentionKL += drainOpen * rain * 1.6f * dt;
        if (rain < 0.05f) retentionKL -= 0.35f * dt;                    // pumped out to the treatment works after the storm
        retentionKL = Clamp(retentionKL, 0.0f, RETENTION_CAP);
        float green = Clamp(greenRoofs * 0.0045f, 0.0f, 0.2f);
        runoffK = 1.0f - green - (rainTankKL < RAIN_TANK_CAP - 1.0f ? 0.16f : 0.03f) - 0.30f * drainOpen * (retentionKL < RETENTION_CAP - 2.0f ? 1.0f : 0.0f);
    } else { runoffK = 1.0f; drainOpen = 0.0f; harvestLpm = 0.0f; }
    for (int i = 0; i < N_FLOOD; i++) {
        float local = floodLevel + ((i * 7 + 3) % 5) * 0.015f;
        floodState[i] = (floodLevel > 0.12f && local > 0.12f) ? 2 : ((smartCity && rain > 0.2f && st.sewerPct > 80.0f) ? 1 : 0);
    }
}

static void UpdateAirSensors(void) {
    float worst = -1.0f;
    for (int i = 0; i < N_AQ; i++) {
        int x0 = (i % 3) * 8, y0 = (i / 3) * 8;
        float sum = 0.0f;
        for (int y = y0; y < y0 + 8; y++) for (int x = x0; x < x0 + 8; x++) sum += polMap[y][x];
        float aqi = 12.0f + sum / 64.0f * 3.0f;
        aqLevel[i] += (aqi - aqLevel[i]) * 0.4f;
        if (aqLevel[i] > worst) { worst = aqLevel[i]; aqWorst = i; }
    }
}

static void UpdateParkPlus(void) {
    int cap = 0, occ = 0, cars_ = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] == T_PARK) for (int s = 0; s < 8; s++) { cap++; if (lot[y][x][s]) occ++; }
            else if (surface[y][x] == T_HOUSE && drive[y][x]) cars_++;
        }
    parkTotal = cap; parkFree = cap - occ;
    parkedCars = occ + cars_;
    evCount = (int)(parkedCars * 0.7f);
    float fillK = cap ? (float)occ / cap : 0.0f;
    float want = smartCity ? 0.0f : Clamp((fillK - 0.55f) / 0.40f, 0.0f, 1.0f) * 14.0f;   // drivers circling for a bay
    parkSearch += (want - parkSearch) * 0.3f;
}

static void UpdateHeating(void) {
    int fac = 0, served = 0, gr = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            if (s == T_BUILDING && bType[y][x] == B_IND && deco[y][x] == 0 && health[y][x] >= 50.0f && fireI[y][x] <= 0.0f) fac++;
            if (dhServed[y][x] && IsBldg(s) && health[y][x] >= 50.0f) served++;
            if (GreenRoof(x, y)) gr++;
        }
    greenRoofs = gr;
    dhBuildings = served;
    float df = DayFactor();
    float coldK = Clamp((16.5f - st.ambient) / 8.0f, 0.05f, 1.0f);
    wasteHeatKW = fac * 260.0f * (0.55f + 0.45f * df);
    heatDemandKW = served * 9.0f * coldK;
    if (smartCity) { dhKW = fminf(wasteHeatKW, heatDemandKW); heatCover = dhKW / (heatDemandKW + 1.0f) * 100.0f; }
    else { dhKW = 0.0f; heatCover = 0.0f; }
}

static void UpdateTwin(float dt) {
    int m = smartCity ? 1 : 0;
    float fossil = smartCity ? gridImportKW : st.energyDemand;
    twSum[m][TW_AQI]    += st.aqi * dt;
    twSum[m][TW_FOSSIL] += fossil / 1000.0f * dt;
    twSum[m][TW_CO2]    += fossil * 0.0004f * dt;                       // t CO2 per hour at 0.4 kg per kWh
    twSum[m][TW_DEMAND] += 100.0f * st.energyDemand / fmaxf(1.0f, demandBaseKW) * dt;
    if (lightBaseKW > 1.0f) { twSum[m][TW_LIGHT] += 100.0f * lightNowKW / lightBaseKW * dt; twNight[m] += dt; }
    twSum[m][TW_SPEED]  += avgSpeed * dt;
    twSum[m][TW_STOP]   += (st.vehicles ? 100.0f * st.jammed / (float)st.vehicles : 0.0f) * dt;
    twSum[m][TW_LEAK]   += leakLpm * dt;
    twTime[m] += dt;
    twRespSum[m] += respSum - twLastResp;   twRespN[m] += respN - twLastRespN;
    twTransSum[m] += transSum - twLastTrans; twTransN[m] += transN - twLastTransN;
    twLost[m] += medLost - twLastLost;
    twRecycle[m] += recycledT - twLastRec;  twLandfill[m] += wasteT - twLastLand;
    twLastResp = respSum; twLastRespN = (float)respN; twLastTrans = transSum; twLastTransN = (float)transN;
    twLastLost = (float)medLost; twLastRec = recycledT; twLastLand = wasteT;
}

static void UpdatePlus(float dt) {
    static float acc = 0.0f;
    UpdateBins(dt);
    UpdateLeaks(dt);
    UpdateDelivery(dt);
    acc += dt;
    if (acc >= 0.25f) {
        float td = acc; acc = 0.0f;
        UpdateLamps();
        UpdateWaterPlus(td);
        UpdateAirSensors();
        UpdateParkPlus();
        UpdateHeating();
        int pax = 0;
        for (int i = 0; i < MAX_CARS; i++) if (cars[i].active && cars[i].shuttle) pax += cars[i].shPax;
        shuttlePax = pax; modalCars = smartCity ? (int)(pax * 0.4f) : 0;
        UpdateTwin(td);
    }
}

// ---------------------------------------------------------------------------
// First-person mode: collisions, walking, car doors, driving
// ---------------------------------------------------------------------------
static float FpWorldMaxX(void) { return (float)MAP_W + (smartCity ? (float)FARM_W : 0.0f) + 4.0f; }

static float FpGround(float px, float pz) {                 // standing height under a point: the same flat level everywhere
    (void)px; (void)pz;
    return WALK_TOP;
}

static bool FpOnRoad(float px, float pz) {
    int tx = (int)floorf(px), ty = (int)floorf(pz);
    if (px >= MAP_W) return (smartCity && ty == FARM_ROAD_Y);
    if (!InBounds(tx, ty)) return false;
    int s = surface[ty][tx];
    return s == T_ROAD || s == T_PARK || paved[ty][tx];
}

// Is this ground point inside a building, tree, fence or other solid?
static bool FpSolid(float px, float pz) {
    if (px >= (float)MAP_W) {                                 // renewable-energy strip east of the city
        if (!smartCity) return false;
        if (px > MAP_W + FARM_W - 0.06f || pz < 0.06f || pz > MAP_H - 0.06f) return true;         // perimeter fence
        if (pz >= FARM_SOLAR_Y0 && pz < FARM_SOLAR_Y1 + 1.0f && px >= FARM_SOLAR_X0 && px < FARM_SOLAR_X1 + 1.0f) return true;   // panels
        if (px > 24.12f && px < 24.88f && pz > 11.08f && pz < 13.92f) return true;              // battery containers
        for (int i = 0; i < N_TURB; i++) { float dx = px - TURB_POS[i][0], dz = pz - TURB_POS[i][1]; if (dx * dx + dz * dz < 0.07f * 0.07f) return true; }
        return false;
    }
    int tx = (int)floorf(px), ty = (int)floorf(pz);
    if (!InBounds(tx, ty)) return false;
    float cx = tx + 0.5f, cz = ty + 0.5f, dx = px - cx, dz = pz - cz;
    int s = surface[ty][tx];
    switch (s) {
        case T_BUILDING:
            if (FitHit(FitIn(tx, ty, RoadSide(tx, ty), -0.475f, 0.475f, -0.475f, 0.475f), RoadSide(tx, ty), dx, dz)) return true;
            break;
        case T_FIRE_STN: case T_POLICE_STN:
            if (FitHit(FitIn(tx, ty, faceDir[ty][tx], -0.49f, 0.49f, -0.46f, 0.46f), faceDir[ty][tx], dx, dz)) return true;
            break;
        case T_POWER: case T_WATER:
            if (FitHit(FitIn(tx, ty, -1, -0.46f, 0.46f, -0.46f, 0.46f), -1, dx, dz)) return true;
            break;
        case T_HOSPITAL:
            if (FitHit(FitIn(tx, ty, faceDir[ty][tx], -0.46f, 0.46f, -0.37f, 0.49f), faceDir[ty][tx], dx, dz)) return true;
            break;
        case T_MALL: {
            float mx0, mx1, mz0, mz1;
            MallRect(tx, ty, &mx0, &mx1, &mz0, &mz1);
            if (px >= mx0 && px <= mx1 && pz >= mz0 && pz <= mz1) return true;
            break; }
        case T_RUIN:
            if (fabsf(dx) < 0.34f && fabsf(dz) < 0.34f) return true;
            break;
        case T_TREE:
            if (dx * dx + dz * dz < 0.07f * 0.07f) return true;
            break;
        case T_LANDFILL:
            if (IsRecycleTile(tx, ty) && !gateTile[ty][tx] && FitHit(FitIn(tx, ty, -1, -0.48f, 0.48f, -0.48f, 0.48f), -1, dx, dz)) return true;
            break;
        case T_HOUSE: {
            int hd = faceDir[ty][tx];
            Fit hf = HouseFit(tx, ty);
            float lx = dx * DY[hd] - dz * DX[hd], lz = dx * DX[hd] + dz * DY[hd];      // the house's own frame ...
            lx = hf.ncx + (lx - hf.ccx) / hf.kx; lz = hf.ncz + (lz - hf.ccz) / hf.kz;  // ... and back to its natural size
            if (lx > HS_CX - HS_W * 0.5f - 0.01f && lx < HS_GX + HS_GW * 0.5f + 0.01f && lz > HS_CZ - HS_L * 0.5f - 0.01f && lz < HS_CZ + HS_L * 0.5f + 0.01f) return true;   // body + garage
            break; }
        default:
            if (zone[ty][tx] == Z_INDUSTRIAL && s == T_GRASS && paved[ty][tx] && !gateTile[ty][tx] && fabsf(dx) < 0.30f && fabsf(dz) < 0.30f) return true;   // container yard
            break;
    }
    int z = zone[ty][tx];
    if (z >= Z_GATED1 && z <= Z_LANDFILL) {                    // fenced compounds: the fence stands FENCE_LINE from the tile centre
        for (int d = 0; d < 4; d++) {
            int nx = tx + DX[d], ny = ty + DY[d];
            if (InBounds(nx, ny) && zone[ny][nx] == z) continue;
            float along = dx * DX[d] + dz * DY[d];
            float fl = FenceLine(tx, ty, d);                           // the fence is a thin barrier on its line ...
            if (along <= fl - 0.04f || along > fl + 0.01f) continue;
            float across = dx * DY[d] - dz * DX[d];                    // ... that runs only as far as it is drawn
            if (across < -FenceEnd(tx, ty, z, d, -1) || across > FenceEnd(tx, ty, z, d, 1)) continue;
            if (HouseGateEdge(tx, ty) && d == faceDir[ty][tx]) {                       // a house's own gateway (local X = (DY,-DX))
                float lxT = dx * DY[d] - dz * DX[d];
                float gc, gh;
                HouseGate(tx, ty, &gc, &gh);
                if (fabsf(lxT - gc) < gh) continue;
            }
            bool gate = gateTile[ty][tx] && IsRoad(nx, ny);
            float perp = fabsf(dx * (float)abs(DY[d]) + dz * (float)abs(DX[d]));
            if (!(gate && perp < 0.22f)) return true;
        }
    }
    return false;
}

static bool FpSolidR(float px, float pz, float r) {
    return FpSolid(px, pz) || FpSolid(px + r, pz) || FpSolid(px - r, pz) || FpSolid(px, pz + r) || FpSolid(px, pz - r);
}

// circles that approximate a vehicle: three along its length
static bool FpCircleVsVeh(float x, float z, float r, float vx, float vz, float yawDeg, float L, float W) {
    float s = sinf(yawDeg * DEG2RAD), c = cosf(yawDeg * DEG2RAD), rr = r + W * 0.5f + 0.004f;
    for (int k = -1; k <= 1; k++) {
        float px = vx + s * L * 0.32f * k, pz = vz + c * L * 0.32f * k;
        float dx = x - px, dz = z - pz;
        if (dx * dx + dz * dz < rr * rr) return true;
    }
    return false;
}

// anything that moves or is parked: traffic, people, emergency vehicles, other user cars (skipU = the car being driven)
static bool FpHitDyn(float x, float z, float r, int skipU) {
    for (int i = 0; i < MAX_CARS; i++) {
        const Car *c = &cars[i];
        if (!c->active) continue;
        if (fabsf(c->x - x) > 0.5f || fabsf(c->y - z) > 0.5f) continue;
        if (FpCircleVsVeh(x, z, r, c->x, c->y, c->yaw, VLEN[c->type], VWID[c->type])) return true;
    }
    for (int i = 0; i < MAX_UCARS; i++) {
        const UCar *u = &ucars[i];
        if (!u->active || i == skipU) continue;
        if (fabsf(u->x - x) > 0.5f || fabsf(u->z - z) > 0.5f) continue;
        if (FpCircleVsVeh(x, z, r, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type])) return true;
    }
    for (int i = 0; i < MAX_PEDS; i++) {
        const Ped *p = &peds[i];
        if (!p->active) continue;
        float dx = x - p->x, dz = z - p->y, rr = r + 0.022f;
        if (dx * dx + dz * dz < rr * rr) return true;
    }
    const Responder *pools[4] = { trucks, police, ambs, &binTruck };
    int counts[4] = { MAX_TRUCKS, MAX_POLICE, MAX_AMB, 1 };
    for (int k = 0; k < 4; k++)
        for (int i = 0; i < counts[k]; i++) {
            const Responder *q = &pools[k][i];
            if (!q->active) continue;
            float dx = x - q->x, dz = z - q->y, rr = r + 0.10f;
            if (dx * dx + dz * dz < rr * rr) return true;
        }
    return false;
}

static float LerpAngleRad(float a, float b, float t) {
    float d = fmodf(b - a + 3.0f * PI, 2.0f * PI) - PI;
    return a + d * t;
}
static float SmoothK(float t) { t = Clamp(t, 0.0f, 1.0f); return t * t * (3.0f - 2.0f * t); }

// ---- parked cars on the streets (always present, can be entered) -------------------------------
#define PARK_SIDE 0.245f   // street-parked cars: just outside the traffic lane, clear of the sidewalk
static void InitFp(void) {
    memset(ucars, 0, sizeof(ucars));
    memset(&fp, 0, sizeof(fp));
    int n = 0;
    for (int tries = 0; tries < 800 && n < 14; tries++) {
        int x = GetRandomValue(1, MAP_W - 2), y = GetRandomValue(1, MAP_H - 2);
        if (!IsRoad(x, y) || ctrl[y][x] != C_NONE || crossT[y][x]) continue;
        bool ns = IsRoad(x, y - 1) && IsRoad(x, y + 1) && !IsRoad(x - 1, y) && !IsRoad(x + 1, y);
        bool ew = IsRoad(x - 1, y) && IsRoad(x + 1, y) && !IsRoad(x, y - 1) && !IsRoad(x, y + 1);
        if (!ns && !ew) continue;
        int ax = ns ? 0 : 1, bx = ns ? 0 : -1, ay = ns ? 1 : 0, by = ns ? -1 : 0;       // the two road neighbours along the axis
        if (ctrl[y + ay][x + ax] != C_NONE || ctrl[y + by][x + bx] != C_NONE) continue;
        bool near = false;
        for (int i = 0; i < MAX_UCARS; i++) if (ucars[i].active && fabsf(ucars[i].x - x - 0.5f) < 1.6f && fabsf(ucars[i].z - y - 0.5f) < 1.6f) near = true;
        if (near) continue;
        int type; do type = RandTrafficType(); while (type == V_TRUCK || type == V_BIKE);
        UCar *u = &ucars[n++];
        u->active = true; u->type = type; u->colIdx = GetRandomValue(0, N_CARCOL - 1);
        float side = (GetRandomValue(0, 1) ? 1.0f : -1.0f) * PARK_SIDE, along = (Rf() - 0.5f) * 0.3f;
        u->x = x + 0.5f + (ns ? side : along);
        u->z = y + 0.5f + (ns ? along : side);
        u->yaw = ns ? (GetRandomValue(0, 1) ? 0.0f : 180.0f) : (GetRandomValue(0, 1) ? 90.0f : 270.0f);
        u->gy = WALK_TOP;
    }
}

// ---- seat / door geometry ---------------------------------------------------------------------
static void UCarLocalToWorld(const UCar *u, float lx, float lz, float *wx, float *wz) {
    float th = u->yaw * DEG2RAD, c = cosf(th), s = sinf(th);
    *wx = u->x + lx * c + lz * s;
    *wz = u->z - lx * s + lz * c;
}
static void UCarSeat(const UCar *u, float *wx, float *wy, float *wz) {
    float zc = UC_CABIN[u->type][0], cl = UC_CABIN[u->type][1], roof = UC_CABIN[u->type][2];
    float sz = fmaxf(zc - cl * 0.38f, zc + cl * 0.5f - 0.09f);
    UCarLocalToWorld(u, -VWID[u->type] * 0.22f, sz, wx, wz);      // driver sits on the right (left-hand traffic)
    *wy = u->gy + roof - 0.024f;
}
static void UCarDoorPoint(const UCar *u, int door, float out, float *wx, float *wy, float *wz) {
    float side = (door == 0) ? -1.0f : 1.0f;
    UCarLocalToWorld(u, side * (VWID[u->type] * 0.5f + out), UC_CABIN[u->type][0] - UC_CABIN[u->type][1] * 0.17f, wx, wz);   // aim at the gap behind the swung-open door
    *wy = u->gy + 0.095f;
}

// ---- candidate cars the walking player could get into --------------------------------------------
typedef struct { int src, i, x, y, s; float px, pz, yaw, d; int type, col; } FpCand;     // src: 0 street car, 1 car park, 2 driveway

static float DistToVeh(float px, float pz, float vx, float vz, float yawDeg, float L, float W) {
    float th = yawDeg * DEG2RAD, c = cosf(th), s = sinf(th);
    float dx = px - vx, dz = pz - vz;
    float lx = dx * c - dz * s, lz = dx * s + dz * c;
    float ex = fmaxf(0.0f, fabsf(lx) - W * 0.5f), ez = fmaxf(0.0f, fabsf(lz) - L * 0.5f);
    return sqrtf(ex * ex + ez * ez);
}

static bool FpFindCar(FpCand *best) {
    best->d = 0.26f; best->src = -1;
    for (int i = 0; i < MAX_UCARS; i++) {
        const UCar *u = &ucars[i];
        if (!u->active) continue;
        float d = DistToVeh(fp.x, fp.z, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type]);
        if (d < best->d) { best->d = d; best->src = 0; best->i = i; }
    }
    int cx = (int)floorf(fp.x), cy = (int)floorf(fp.z);
    for (int y = cy - 1; y <= cy + 1; y++)
        for (int x = cx - 1; x <= cx + 1; x++) {
            if (!InBounds(x, y)) continue;
            if (surface[y][x] == T_PARK) {
                for (int s = 0; s < 8; s++) {
                    if (!lot[y][x][s]) continue;
                    int type = lot[y][x][s] - 1;
                    float vx = x + ((s % 4) + 0.5f) * 0.25f, vz = y + 0.30f + (s / 4) * 0.40f, yw = (s / 4) ? 180.0f : 0.0f;
                    float d = DistToVeh(fp.x, fp.z, vx, vz, yw, VLEN[type], VWID[type]);
                    if (d < best->d) { best->d = d; best->src = 1; best->x = x; best->y = y; best->s = s; best->px = vx; best->pz = vz; best->yaw = yw; best->type = type; best->col = lotCol[y][x][s]; }
                }
            } else if (surface[y][x] == T_HOUSE && drive[y][x]) {
                int type = drive[y][x] - 1;
                float th = FaceYaw(faceDir[y][x]), r = th * DEG2RAD;
                float vx = x + 0.5f + HS_GX * cosf(r) + HS_DRV_Z * sinf(r), vz = y + 0.5f - HS_GX * sinf(r) + HS_DRV_Z * cosf(r);
                float d = DistToVeh(fp.x, fp.z, vx, vz, th, VLEN[type], VWID[type]);
                if (d < best->d) { best->d = d; best->src = 2; best->x = x; best->y = y; best->px = vx; best->pz = vz; best->yaw = th; best->type = type; best->col = drvCol[y][x]; }
            }
        }
    return best->src >= 0;
}

static int UCarBlocked(const UCar *u, float x, float z, float yawDeg, int self, int *nSolid);
static int FpClaimCar(const FpCand *c) {              // make sure the car exists as a user car; returns its index
    if (c->src == 0) return c->i;
    for (int i = 0; i < MAX_UCARS; i++) if (!ucars[i].active) {
        UCar *u = &ucars[i];
        memset(u, 0, sizeof(*u));
        u->active = true; u->type = c->type; u->colIdx = c->col; u->x = c->px; u->z = c->pz; u->yaw = c->yaw;
        u->gy = FpGround(c->px, c->pz);
        int ns = 0;
        UCarBlocked(u, u->x, u->z, u->yaw, i, &ns);
        if (ns > 0) {                                      // a driveway car can overhang the garden fence: re-park it on the nearest clear spot
            float bestD = 1e9f, bx = u->x, bz = u->z, by = u->yaw;
            for (int yk = 0; yk < 4; yk++) {
                float yw = c->yaw + ((yk == 0) ? 0.0f : (yk == 1) ? 90.0f : (yk == 2) ? -90.0f : 180.0f);
                for (int ix = -9; ix <= 9; ix++)
                    for (int iz = -9; iz <= 9; iz++) {
                        float px = c->px + ix * 0.04f, pz = c->pz + iz * 0.04f, d = ix * ix + iz * iz + yk * 3.0f;
                        if (d >= bestD) continue;
                        int n2 = 0; UCarBlocked(u, px, pz, yw, i, &n2);
                        if (n2 == 0) { bestD = d; bx = px; bz = pz; by = yw; }
                    }
            }
            u->x = bx; u->z = bz; u->yaw = by;
        }
        if (c->src == 1) lot[c->y][c->x][c->s] = 0; else drive[c->y][c->x] = 0;
        return i;
    }
    return -1;
}

// ---- state changes ------------------------------------------------------------------------------
static void FpPlace(float x, float z) {                // put the walker on the nearest free ground
    for (int r = 0; r < 40; r++)
        for (int k = 0; k < 16; k++) {
            float a = k * (PI / 8.0f), px = x + cosf(a) * r * 0.1f, pz = z + sinf(a) * r * 0.1f;
            if (px < -3.0f || pz < -3.0f || px > FpWorldMaxX() - 0.3f || pz > MAP_H + 3.0f) continue;
            if (!FpSolidR(px, pz, FP_RADIUS * 1.5f) && !FpHitDyn(px, pz, FP_RADIUS, -1)) { fp.x = px; fp.z = pz; return; }
        }
    fp.x = 12.5f; fp.z = 8.5f;
}

static void FpSetCamWalk(void) {
    fp.cx = fp.x; fp.cz = fp.z; fp.cy = fp.gy + FP_EYE + fp.hop + fp.bob; fp.cyaw = fp.yaw; fp.cpitch = fp.pitch;
}

static int fpSkipLook = 0;
static void FpEnterMode(void) {
    viewMode = VIEW_FP;
    DisableCursor();
    fpSkipLook = 3;
    if (!fp.valid || fp.state == FP_WALK) {
        float tx = oc.target.x, tz = oc.target.z;
        if (!fp.valid) { fp.yaw = oc.yaw + PI; fp.pitch = 0.0f; fp.state = FP_WALK; fp.hint = 14.0f; }
        FpPlace(Clamp(tx, 0.3f, MAP_W - 0.3f), Clamp(tz, 0.3f, MAP_H - 0.3f));
        fp.valid = true; fp.gy = FpGround(fp.x, fp.z);
        FpSetCamWalk();
    }
    fp.hint = fmaxf(fp.hint, 8.0f);
}

static void FpLeaveMode(void) {
    if (fp.state == FP_DRIVE || fp.state == FP_ENTER || fp.state == FP_EXIT) {
        UCar *u = &ucars[fp.car];                      // stop the car and step out where it stands
        u->v = 0.0f; u->doorTgt[0] = u->doorTgt[1] = 0.0f; u->door[0] = u->door[1] = 0.0f;
        float ox, oz; UCarLocalToWorld(u, -(VWID[u->type] * 0.5f + 0.11f), UC_CABIN[u->type][0] - UC_CABIN[u->type][1] * 0.17f, &ox, &oz);
        fp.x = ox; fp.z = oz; fp.yaw = u->yaw * DEG2RAD; fp.pitch = 0.0f; fp.state = FP_WALK;
    }
    oc.target = (Vector3){ fp.x, 0.0f, fp.z };
    oc.yaw = fp.yaw + PI;
    viewMode = VIEW_3D;
    EnableCursor();
}

static void FpStartEnter(int ci) {
    UCar *u = &ucars[ci];
    float th = u->yaw * DEG2RAD, dx = fp.x - u->x, dz = fp.z - u->z;
    float lx = dx * cosf(th) - dz * sinf(th);
    fp.car = ci; fp.door = (lx > 0.0f) ? 1 : 0; fp.state = FP_ENTER; fp.st = 0.0f;
    fp.sx = fp.cx; fp.sy = fp.cy; fp.sz = fp.cz; fp.syaw = fp.cyaw; fp.spitch = fp.cpitch;
    fp.lookYaw = fp.lookPitch = 0.0f;
}

static bool FpStartExit(void) {
    UCar *u = &ucars[fp.car];
    float W = VWID[u->type], L = VLEN[u->type];
    float dz0 = UC_CABIN[u->type][0] - UC_CABIN[u->type][1] * 0.17f;
    float cand[4][3] = { { -(W * 0.5f + 0.11f), dz0, 0 }, { (W * 0.5f + 0.11f), dz0, 1 },
                         { 0, -(L * 0.5f + 0.11f), 0 }, { 0, (L * 0.5f + 0.11f), 0 } };
    for (int k = 0; k < 4; k++) {
        float ox, oz; UCarLocalToWorld(u, cand[k][0], cand[k][1], &ox, &oz);
        if (FpSolidR(ox, oz, FP_RADIUS * 1.4f) || FpHitDyn(ox, oz, FP_RADIUS, fp.car)) continue;
        fp.x = ox; fp.z = oz; fp.door = (int)cand[k][2];
        fp.state = FP_EXIT; fp.st = 0.0f;
        fp.sx = fp.cx; fp.sy = fp.cy; fp.sz = fp.cz; fp.syaw = fp.cyaw; fp.spitch = fp.cpitch;
        fp.gy = FpGround(ox, oz); fp.yaw = u->yaw * DEG2RAD; fp.pitch = 0.0f;
        return true;
    }
    FpPlace(u->x, u->z);                                              // no clear side: step out onto the nearest free ground
    fp.door = 0; fp.state = FP_EXIT; fp.st = 0.0f;
    fp.sx = fp.cx; fp.sy = fp.cy; fp.sz = fp.cz; fp.syaw = fp.cyaw; fp.spitch = fp.cpitch;
    fp.gy = FpGround(fp.x, fp.z); fp.yaw = u->yaw * DEG2RAD; fp.pitch = 0.0f;
    return true;
}

// ---- driving --------------------------------------------------------------------------------------
// exact vehicle-vs-vehicle test for a user car at (x,z,yaw): street traffic and the other user cars
static bool UCarOverlapsVehicles(float x, float z, float yawDeg, int type, int self) {
    float L = VLEN[type], W = VWID[type];
    for (int i = 0; i < MAX_CARS; i++) {
        const Car *c = &cars[i];
        if (!c->active || fabsf(c->x - x) > 0.6f || fabsf(c->y - z) > 0.6f) continue;
        if (VehOverlap(x, z, yawDeg, L, W, c->x, c->y, c->yaw, VLEN[c->type], VWID[c->type])) return true;
    }
    for (int i = 0; i < MAX_UCARS; i++) {
        const UCar *u = &ucars[i];
        if (!u->active || i == self || fabsf(u->x - x) > 0.6f || fabsf(u->z - z) > 0.6f) continue;
        if (VehOverlap(x, z, yawDeg, L, W, u->x, u->z, u->yaw, VLEN[u->type], VWID[u->type])) return true;
    }
    return false;
}

static int UCarBlocked(const UCar *u, float x, float z, float yawDeg, int self, int *nSolid) {      // bit 1 = solid scenery, bit 2 = other vehicle / person
    float W = VWID[u->type], L = VLEN[u->type], th = yawDeg * DEG2RAD, c = cosf(th), s = sinf(th);
    static const float P[8][2] = { { 1, 1 }, { -1, 1 }, { 1, -1 }, { -1, -1 }, { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
    bool solid = false, dyn = false;
    int ns = 0;
    for (int k = 0; k < 8; k++) {
        float lx = P[k][0] * W * 0.5f, lz = P[k][1] * L * 0.5f;
        float wx = x + lx * c + lz * s, wz = z - lx * s + lz * c;
        if (FpSolid(wx, wz)) { solid = true; ns++; }
    }
    if (nSolid) *nSolid = ns;
    for (int k = -1; k <= 1; k++) if (FpHitDyn(x + s * L * 0.30f * k, z + c * L * 0.30f * k, W * 0.5f, self)) dyn = true;   // people, emergency vehicles, rough car circles
    if (UCarOverlapsVehicles(x, z, yawDeg, u->type, self)) dyn = true;                                          // exact: no two cars share ground
    return (solid ? 1 : 0) | (dyn ? 2 : 0);
}

static void UpdateDrive(float dt) {
    UCar *u = &ucars[fp.car];
    bool fwd = FP_DOWN(KEY_W) || FP_DOWN(KEY_UP), back = FP_DOWN(KEY_S) || FP_DOWN(KEY_DOWN);
    bool left = FP_DOWN(KEY_A) || FP_DOWN(KEY_LEFT), right = FP_DOWN(KEY_D) || FP_DOWN(KEY_RIGHT);
    bool hand = FP_DOWN(KEY_SPACE);
    bool road = FpOnRoad(u->x, u->z);
    float vmax = road ? 3.3f : 1.2f;
    float acc = 0.0f;
    u->braking = false;
    if (fwd && !back) { if (u->v < -0.05f) { acc = 5.0f; u->braking = true; } else acc = 1.9f * (1.0f - fmaxf(0.0f, u->v) / vmax); }
    else if (back && !fwd) { if (u->v > 0.05f) { acc = -5.5f; u->braking = true; } else acc = -1.3f * (1.0f - fmaxf(0.0f, -u->v) / 1.1f); }
    u->v += acc * dt;
    float drag = (road ? 0.30f : 1.6f);
    if (!fwd && !back) u->v -= u->v * drag * dt + (u->v > 0 ? 1 : -1) * fminf(fabsf(u->v), 0.22f * dt);
    else u->v -= u->v * 0.10f * dt;
    if (hand) { float b = 5.0f * dt; u->v = (fabsf(u->v) <= b) ? 0.0f : u->v - (u->v > 0 ? b : -b); u->braking = true; }
    u->v = Clamp(u->v, -1.1f, vmax);
    if (!road && u->v > vmax) u->v = vmax;

    float want = (left ? 1.0f : 0.0f) - (right ? 1.0f : 0.0f);
    float maxSt = 0.55f / (1.0f + fabsf(u->v) * 0.95f);
    u->steer += (want * maxSt - u->steer) * Clamp(dt * 7.0f, 0.0f, 1.0f);
    float wb = VLEN[u->type] * 0.62f;
    float dyaw = (u->v / wb) * tanf(u->steer) * dt;                      // radians; left turn = heading rotates towards +x = yaw increases
    float nyaw = u->yaw + dyaw * RAD2DEG;
    float th = nyaw * DEG2RAD;
    float nx = u->x + sinf(th) * u->v * dt, nz = u->z + cosf(th) * u->v * dt;
    nx = Clamp(nx, -4.0f, FpWorldMaxX()); nz = Clamp(nz, -4.0f, MAP_H + 4.0f);

    int nsNow = 0, nsWas = 0;
    int now = UCarBlocked(u, nx, nz, nyaw, fp.car, &nsNow), was = UCarBlocked(u, u->x, u->z, u->yaw, fp.car, &nsWas);
    if ((nsNow > nsWas) || ((now & 2) && !(was & 2))) {                  // (a car that starts out overlapping scenery may still drive clear of it)                                 // crunch: stop and bounce back a little
        if (fabsf(u->v) > 0.5f) fp.bump = 0.35f;
        u->v = -u->v * 0.22f;
    } else { u->x = nx; u->z = nz; u->yaw = nyaw; }
    if (u->yaw > 360.0f) u->yaw -= 360.0f;
    if (u->yaw < 0.0f) u->yaw += 360.0f;
    float gTgt = FpGround(u->x, u->z);
    u->gy += (gTgt - u->gy) * Clamp(dt * 10.0f, 0.0f, 1.0f);
}

// ---- main update ----------------------------------------------------------------------------------
static void UpdateFp(float dt) {
    if (!fp.valid) return;
    Vector2 md = GetMouseDelta();
    float k = (GetScreenWidth() > 0) ? (float)SCREEN_W / (float)GetScreenWidth() : 1.0f, sens = 0.0024f;
    float mdx = md.x * k, mdy = md.y * k;
    if (fpSkipLook > 0) { fpSkipLook--; mdx = mdy = 0.0f; }
    bool looked = fabsf(mdx) + fabsf(mdy) > 0.5f;
    fp.prompt[0] = 0;
    fp.hint = fmaxf(0.0f, fp.hint - dt);
    fp.bump = fmaxf(0.0f, fp.bump - dt);
    for (int i = 0; i < MAX_UCARS; i++) {                               // doors swing towards their target
        UCar *u = &ucars[i];
        if (!u->active) continue;
        for (int d = 0; d < 2; d++) { float df = u->doorTgt[d] - u->door[d], st = 230.0f * dt; u->door[d] += Clamp(df, -st, st); }
    }
    for (int i = 0; i < MAX_UCARS; i++)                                 // parked cars settle onto the ground
        if (ucars[i].active && !(i == fp.car && fp.state != FP_WALK)) ucars[i].gy += (FpGround(ucars[i].x, ucars[i].z) - ucars[i].gy) * Clamp(dt * 10.0f, 0.0f, 1.0f);

    if (fp.state == FP_WALK) {
        if (FpSolid(fp.x, fp.z)) FpPlace(fp.x, fp.z);                    // e.g. the city was switched while standing where a fence now stands
        fp.yaw -= mdx * sens;
        fp.pitch = Clamp(fp.pitch - mdy * sens, -1.35f, 1.35f);
        float fx = sinf(fp.yaw), fz = cosf(fp.yaw), rx = -cosf(fp.yaw), rz = sinf(fp.yaw);
        float inF = ((FP_DOWN(KEY_W) || FP_DOWN(KEY_UP)) ? 1.0f : 0.0f) - ((FP_DOWN(KEY_S) || FP_DOWN(KEY_DOWN)) ? 1.0f : 0.0f);
        float inR = ((FP_DOWN(KEY_D) || FP_DOWN(KEY_RIGHT)) ? 1.0f : 0.0f) - ((FP_DOWN(KEY_A) || FP_DOWN(KEY_LEFT)) ? 1.0f : 0.0f);
        float vx = fx * inF + rx * inR, vz = fz * inF + rz * inR, l = sqrtf(vx * vx + vz * vz);
        float spd = FP_DOWN(KEY_LEFT_SHIFT) ? 1.8f : 0.8f;
        if (l > 0.001f) { vx = vx / l * spd * dt; vz = vz / l * spd * dt; } else { vx = vz = 0.0f; }
        float nx = Clamp(fp.x + vx, -5.0f, FpWorldMaxX()), nz = Clamp(fp.z + vz, -5.0f, MAP_H + 5.0f);
        bool inside = FpHitDyn(fp.x, fp.z, FP_RADIUS, -1);
        if (!FpSolidR(nx, fp.z, FP_RADIUS) && (inside || !FpHitDyn(nx, fp.z, FP_RADIUS, -1))) fp.x = nx;
        if (!FpSolidR(fp.x, nz, FP_RADIUS) && (inside || !FpHitDyn(fp.x, nz, FP_RADIUS, -1))) fp.z = nz;
        fp.speed = (l > 0.001f) ? spd : 0.0f;
        fp.bobT += fp.speed * dt * 11.0f;
        fp.bob = (fp.hop < 0.001f) ? sinf(fp.bobT) * 0.0022f * (fp.speed > 1.0f ? 1.5f : 1.0f) * (fp.speed > 0.0f ? 1.0f : 0.0f) : 0.0f;
        float gt = FpGround(fp.x, fp.z);
        fp.gy += (gt - fp.gy) * Clamp(dt * 12.0f, 0.0f, 1.0f);
        if (fp.hop <= 0.0001f && fp.vy <= 0.0f && FP_PRESSED(KEY_SPACE)) fp.vy = 0.55f;
        if (fp.vy != 0.0f || fp.hop > 0.0f) { fp.vy -= 2.6f * dt; fp.hop += fp.vy * dt; if (fp.hop <= 0.0f) { fp.hop = 0.0f; fp.vy = 0.0f; } }
        FpSetCamWalk();

        FpCand cand = { 0 };
        if (FpFindCar(&cand)) {
            snprintf(fp.prompt, sizeof(fp.prompt), "[E]  Open door and get in");
            if (FP_PRESSED(KEY_E)) { int ci = FpClaimCar(&cand); if (ci >= 0) FpStartEnter(ci); }
        }
    } else if (fp.state == FP_ENTER) {
        UCar *u = &ucars[fp.car];
        fp.st = fminf(1.0f, fp.st + dt / 1.6f);
        float t = fp.st;
        u->doorTgt[fp.door] = (t < 0.72f) ? 66.0f : 0.0f;
        float sx, sy, sz, dx, dy, dz;
        UCarSeat(u, &sx, &sy, &sz); UCarDoorPoint(u, fp.door, 0.07f, &dx, &dy, &dz);
        float m = SmoothK((t - 0.28f) / 0.50f);                          // camera travel from the pavement to the seat (via the open door)
        float a = SmoothK(m * 2.0f), b = SmoothK(m * 2.0f - 1.0f);
        float px = fp.sx + (dx - fp.sx) * a, py = fp.sy + (dy - fp.sy) * a, pz = fp.sz + (dz - fp.sz) * a;
        if (m > 0.5f) { px = dx + (sx - dx) * b; py = dy + (sy - dy) * b; pz = dz + (sz - dz) * b; }
        fp.cx = px; fp.cy = py; fp.cz = pz;
        fp.cyaw = LerpAngleRad(fp.syaw, u->yaw * DEG2RAD, m); fp.cpitch = fp.spitch * (1.0f - m);
        if (fp.st >= 1.0f) { fp.state = FP_DRIVE; fp.lookYaw = fp.lookPitch = 0.0f; u->doorTgt[0] = u->doorTgt[1] = 0.0f; fp.hint = 12.0f; }
    } else if (fp.state == FP_DRIVE) {
        UCar *u = &ucars[fp.car];
        fp.lookYaw = Clamp(fp.lookYaw - mdx * sens, -2.5f, 2.5f);
        fp.lookPitch = Clamp(fp.lookPitch - mdy * sens, -0.8f, 0.7f);
        if (looked) fp.lookIdle = 0.0f; else fp.lookIdle += dt;
        if (fp.lookIdle > 1.0f) { float r = Clamp(dt * 2.5f, 0.0f, 1.0f); fp.lookYaw -= fp.lookYaw * r; fp.lookPitch -= fp.lookPitch * r; }
        UpdateDrive(dt);
        float sx, sy, sz; UCarSeat(u, &sx, &sy, &sz);
        float rough = (FpOnRoad(u->x, u->z) ? 0.0f : 0.0016f) * fminf(1.0f, fabsf(u->v));
        fp.cx = sx; fp.cz = sz; fp.cy = sy + sinf((float)GetTime() * 31.0f) * rough + (fp.bump > 0.0f ? sinf(fp.bump * 40.0f) * 0.004f * fp.bump : 0.0f);
        fp.cyaw = u->yaw * DEG2RAD + fp.lookYaw; fp.cpitch = fp.lookPitch;
        fp.x = u->x; fp.z = u->z;
        if (fabsf(u->v) < 0.25f) snprintf(fp.prompt, sizeof(fp.prompt), "[E]  Get out");
        else if (FP_PRESSED(KEY_E)) snprintf(fp.prompt, sizeof(fp.prompt), "Slow down to get out");
        if (FP_PRESSED(KEY_E) && fabsf(u->v) < 0.25f) { u->v = 0.0f; if (!FpStartExit()) snprintf(fp.prompt, sizeof(fp.prompt), "No room to open the door"); }
    } else if (fp.state == FP_EXIT) {
        UCar *u = &ucars[fp.car];
        fp.st = fminf(1.0f, fp.st + dt / 1.6f);
        float t = fp.st;
        u->doorTgt[fp.door] = (t < 0.72f) ? 66.0f : 0.0f;
        float sx, sy, sz, dx, dy, dz;
        UCarDoorPoint(u, fp.door, 0.07f, &dx, &dy, &dz);
        (void)sx; (void)sy; (void)sz;
        float m = SmoothK((t - 0.25f) / 0.50f);
        float a = SmoothK(m * 2.0f), b = SmoothK(m * 2.0f - 1.0f);
        float ex = fp.x, ey = fp.gy + FP_EYE, ez = fp.z;
        float px = fp.sx + (dx - fp.sx) * a, py = fp.sy + (dy - fp.sy) * a, pz = fp.sz + (dz - fp.sz) * a;
        if (m > 0.5f) { px = dx + (ex - dx) * b; py = dy + (ey - dy) * b; pz = dz + (ez - dz) * b; }
        fp.cx = px; fp.cy = py; fp.cz = pz;
        fp.cyaw = u->yaw * DEG2RAD; fp.cpitch = 0.0f;
        if (fp.st >= 1.0f) { fp.state = FP_WALK; u->doorTgt[0] = u->doorTgt[1] = 0.0f; fp.hop = fp.vy = 0.0f; FpSetCamWalk(); }
    }
    if (looked && fp.hint > 0.0f && fp.hint < 6.0f) fp.hint = fmaxf(fp.hint, 6.0f);
}

// ---------------------------------------------------------------------------
// City budget (KPI).  Income comes in from taxes, business rates and electricity sold;
// spending goes out on fuel, services, road upkeep and (smart city only) the smart systems.
// Units are k$ (thousands of dollars) per simulated day.  The balance is reported on the
// stats panel and the digital-twin dashboard, and is never used to stop anything.
// ---------------------------------------------------------------------------
#define BUD_TAX_PER_RESIDENT   0.020f    // council tax per resident per day
#define BUD_RATE_PER_BUSINESS  0.50f     // business rates per shop or factory per day
#define BUD_TARIFF_PER_KW_DAY  0.0048f   // electricity sold: 24 h x $0.20/kWh, per kW of demand
#define BUD_FUEL_PER_KW_DAY    0.00144f  // fuel burned at the power plant: 24 h x $0.06/kWh
#define BUD_STATION_PER_DAY    4.0f      // running cost of each fire, police and hospital station
#define BUD_ROAD_PER_TILE      0.02f     // road and footpath upkeep per tile
#define BUD_LAMP_PER_DAY       0.05f     // smart street lamp: maintenance and data link
#define BUD_BIN_PER_DAY        0.03f     // smart bin: fill sensor and data link
#define BUD_MONITOR_PER_DAY    0.20f     // smart traffic monitor mast
#define BUD_INCOME_TAX_PER_WORKER 0.015f // income tax per employed resident per day (see p6h_jobs.c)

// Income and spending right now (k$ per simulated day)
static void BudgetRates(float *income, float *spend) {
    int shops = 0, roads = 0, monitors = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] == T_ROAD) roads++;
            if (surface[y][x] == T_BUILDING && (bType[y][x] == B_COM || bType[y][x] == B_IND)) shops++;
            if (monitorTile[y][x]) monitors++;
        }

    *income = st.population * BUD_TAX_PER_RESIDENT
            + shops * BUD_RATE_PER_BUSINESS
            + employedNow * BUD_INCOME_TAX_PER_WORKER
            + st.energyDemand * BUD_TARIFF_PER_KW_DAY;

    float fuelKW = smartCity ? gridImportKW : st.energyDemand;   // smart city burns only the backup import
    *spend = fuelKW * BUD_FUEL_PER_KW_DAY
           + (nFire + nPolice + nHosp) * BUD_STATION_PER_DAY
           + roads * BUD_ROAD_PER_TILE;
    if (smartCity)
        *spend += nLamps * BUD_LAMP_PER_DAY + nBins * BUD_BIN_PER_DAY + monitors * BUD_MONITOR_PER_DAY;
}

static void UpdateBudget(float dt) {
    float inc, sp;
    BudgetRates(&inc, &sp);
    budIncomeK = inc; budSpendK = sp;
    float net = inc - sp;
    budBalanceK += net * dt / DAY_LEN;                  // dt and DAY_LEN are both sim seconds: dt / DAY_LEN = share of a day
    twSum[smartCity ? 1 : 0][TW_BUDGET] += net * dt;    // time-weighted average net, kept per mode
}

// ---------------------------------------------------------------------------
// Citizen happiness (KPI, 0-100).  A weighted average of seven measures the simulation already
// tracks.  It is reported on the stats panel and the digital-twin dashboard; the simulation never
// reads it back, so it changes nothing by itself.  Each measure is scored 0-100 (100 = best).
// ---------------------------------------------------------------------------
#define HAP_N 7
static const char  *HAP_NAME[HAP_N] = { "air", "green", "traffic", "safety", "health", "bins", "utilities" };
static const float  HAP_W[HAP_N]    = { 0.20f, 0.15f, 0.15f, 0.15f, 0.10f, 0.10f, 0.15f };   // sums to 1.00

// 100 while the supply covers demand; loses 2 points for every 1% that demand is above supply
static float ScoreOverPct(float pct) { return Clamp(100.0f - 2.0f * fmaxf(0.0f, pct - 100.0f), 0.0f, 100.0f); }

static void UpdateHappiness(float dt) {
    int full = 0;
    for (int i = 0; i < nBins; i++) if (bins[i].fill >= 100.0f) full++;
    float stopped = st.vehicles ? 100.0f * st.jammed / (float)st.vehicles : 0.0f;

    float part[HAP_N];
    part[0] = 100.0f - (st.aqi - 10.0f) * 0.5f;                              // air: AQI 10 scores 100, AQI 210 scores 0
    part[1] = st.green / 25.0f * 100.0f;                                     // green: 25% of the map scores 100
    part[2] = 100.0f - stopped;                                              // traffic: share of vehicles not stopped
    part[3] = st.security;                                                   // safety: the security index
    part[4] = medCalls ? 100.0f * medSaved / (float)medCalls : 100.0f;      // health: share of medical calls saved
    part[5] = nBins ? 100.0f * (1.0f - full / (float)nBins) : 100.0f;       // bins: share not overflowing
    part[6] = 0.5f * (ScoreOverPct(st.energyPct) + ScoreOverPct(st.waterPct)); // utilities: power and water

    happyNow = 0.0f; happyLow = 0;
    for (int i = 0; i < HAP_N; i++) {
        part[i] = Clamp(part[i], 0.0f, 100.0f);
        happyNow += HAP_W[i] * part[i];
        if (part[i] < part[happyLow]) happyLow = i;
    }
    twSum[smartCity ? 1 : 0][TW_HAPPY] += happyNow * dt;     // time-weighted average, kept per mode
}

// ---------------------------------------------------------------------------
// Jobs and employment (KPI).  Workplaces provide jobs.  Residents of working age make up the
// labour force, and the employment rate is employed residents / labour force.  Workers pay
// income tax into the city budget.  Reported only: nothing in the simulation reads it back.
// ---------------------------------------------------------------------------
#define JOB_SHOP          10     // per shop or office building
#define JOB_FACTORY       25     // per industrial building
#define JOB_MALL_TILE      8     // per mall tile
#define JOB_RECYCLE_TILE  10     // per recycling-plant tile (smart city)
#define JOB_TURBINE        3     // per wind turbine (smart city)
#define JOB_DRONE_HUB     12     // drone fulfilment hub staff (smart city)
#define LABOUR_SHARE    0.60f    // share of residents who are of working age

static void UpdateJobs(float dt) {
    int shops = 0, factories = 0, mall = 0, recycle = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] == T_BUILDING && bType[y][x] == B_COM) shops++;
            if (surface[y][x] == T_BUILDING && bType[y][x] == B_IND) factories++;
            if (surface[y][x] == T_MALL) mall++;
            if (IsRecycleTile(x, y)) recycle++;
        }

    jobsNow = shops * JOB_SHOP + factories * JOB_FACTORY + mall * JOB_MALL_TILE;
    if (smartCity) jobsNow += recycle * JOB_RECYCLE_TILE + N_TURB * JOB_TURBINE + JOB_DRONE_HUB;

    labourNow   = st.population * LABOUR_SHARE;
    employedNow = fminf(jobsNow, labourNow);                  // more jobs than workers leaves no one unemployed
    employRate  = labourNow > 0.0f ? 100.0f * employedNow / labourNow : 0.0f;
    twSum[smartCity ? 1 : 0][TW_EMPLOY] += employRate * dt;   // time-weighted average, kept per mode
}

// ---------------------------------------------------------------------------
// Carbon and noise (KPIs and a map).
//   Carbon: CO2 from fuel burned at the power plant, plus CO2 from petrol and diesel cars on the
//           road (electric cars emit nothing on the road).  Reported as tonnes per resident per year.
//   Noise:  each tile has a source level from its traffic, factories, shops, the mall, the landfill
//           or recycling plant, and fires.  The map shows the energy average of the sources within
//           two tiles, which is roughly what someone standing on that tile hears, in dB.
// Reported only: nothing in the simulation reads these back.
// ---------------------------------------------------------------------------
#define CO2_POWER_T_PER_KWH   0.0004f   // t CO2 per kWh burned (same figure as the digital-twin table)
#define CO2_CAR_MOVING_T_PER_H 0.006f   // petrol or diesel car at about 30 km/h (about 0.2 kg per km)
#define CO2_CAR_IDLE_T_PER_H   0.002f   // the same car standing with its engine running
#define HOURS_PER_YEAR        8760.0f

#define NOISE_BASE_DB         38.0f     // quiet background: parks, houses, open ground
#define NOISE_LOUD_DB         70.0f     // tiles above this count as loud

static void UpdateCarbon(float dt) {
    float fossilKW = smartCity ? gridImportKW : st.energyDemand;   // smart city burns only the backup import
    evShare = (smartCity && parkedCars > 0) ? Clamp((float)evCount / (float)parkedCars, 0.0f, 1.0f) : 0.0f;

    int moving = 0, idle = 0;
    for (int i = 0; i < MAX_CARS; i++) {
        Car *c = &cars[i];
        if (!c->active || c->type == V_BIKE) continue;              // bikes emit nothing
        if (c->speedNow > 0.05f) moving++; else idle++;             // speed is in tiles per second
    }

    float combustion = 1.0f - evShare;
    carbonPowerTph   = fossilKW * CO2_POWER_T_PER_KWH;
    carbonTrafficTph = combustion * (moving * CO2_CAR_MOVING_T_PER_H + idle * CO2_CAR_IDLE_T_PER_H);
    carbonPerPerson  = st.population > 0 ? (carbonPowerTph + carbonTrafficTph) * HOURS_PER_YEAR / st.population : 0.0f;
    // the digital twin divides total tonnes by total resident-time, so early small-population
    // readings do not skew the average (see the dashboard)
    twSum[smartCity ? 1 : 0][TW_CARBON_PP] += (carbonPowerTph + carbonTrafficTph) * dt;   // t CO2, per mode
    twSum[smartCity ? 1 : 0][TW_POP]       += st.population * dt;                         // resident-seconds, per mode
}

static void UpdateNoise(float dt) {
    static float srcE[MAP_H][MAP_W];                                 // source levels as sound energy
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float L = NOISE_BASE_DB;
            switch (surface[y][x]) {
                case T_ROAD:     L = NOISE_BASE_DB + 12.0f * traffic[y][x] - 5.0f * evShare - 3.0f * rain; break;
                case T_BUILDING: L = (bType[y][x] == B_IND) ? 58.0f : (bType[y][x] == B_COM ? 50.0f : NOISE_BASE_DB); break;
                case T_MALL:     L = 55.0f; break;
                case T_LANDFILL: L = IsRecycleTile(x, y) ? 52.0f : 45.0f; break;
                default:         break;
            }
            if (fireI[y][x] > 0.0f) L += 12.0f * fireI[y][x] / 100.0f;
            if (L < 0.0f) L = 0.0f;
            srcE[y][x] = powf(10.0f, L / 10.0f);
        }

    float loud = 0.0f, sum = 0.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float e = 0.0f, wsum = 0.0f;
            for (int dy = -2; dy <= 2; dy++)
                for (int dx = -2; dx <= 2; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (!InBounds(nx, ny)) continue;
                    float w = 1.0f / (1.0f + (float)(dx * dx + dy * dy));   // nearer sources count for more
                    e += w * srcE[ny][nx]; wsum += w;
                }
            noiseMap[y][x] = 10.0f * log10f(e / wsum + 1e-6f);
            sum += noiseMap[y][x];
            if (noiseMap[y][x] > NOISE_LOUD_DB) loud++;
        }
    noiseAvg     = sum / (MAP_W * MAP_H);
    noiseLoudPct = 100.0f * loud / (MAP_W * MAP_H);
    twSum[smartCity ? 1 : 0][TW_NOISE] += noiseAvg * dt;             // time-weighted average, kept per mode
}

// Noise map colour: green = quiet (45 dB), yellow = 60 dB, red = loud (75 dB)
static Color NoiseColorAt(int x, int y) {
    Color c = Ramp3((Color){ 90, 210, 110, 255 }, (Color){ 235, 200, 60, 255 }, (Color){ 230, 60, 50, 255 }, (noiseMap[y][x] - 45.0f) / 30.0f);
    c.a = 150;
    return c;
}

// ---------------------------------------------------------------------------
// Heat / pollution maps, statistics, simulation driver
// ---------------------------------------------------------------------------
static void UpdateMaps(void) {
    float df = DayFactor();
    float coldK = Clamp((16.5f - st.ambient) / 8.0f, 0.05f, 1.0f);
    float recovery = (wasteHeatKW > 1.0f) ? dhKW / wasteHeatKW : 0.0f;          // share of factory waste heat that is piped away
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float heatT = 0.0f, pol = 0.0f;
            switch (surface[y][x]) {
                case T_BUILDING:
                    heatT = (bType[y][x] == B_IND ? 4.5f : 2.2f) * df + 0.8f;
                    pol = (bType[y][x] == B_IND) ? (deco[y][x] == 0 ? 7.0f : 4.0f) : (bType[y][x] == B_COM ? 0.9f : 0.5f);
                    if (smartCity) {
                        if (bType[y][x] == B_IND) pol = (deco[y][x] == 0) ? 1.5f : 3.0f;      // factory smoke traps
                        else if (bType[y][x] == B_COM && bHeight[y][x] >= 1.0f) { pol = -2.4f; heatT -= 1.0f; }   // skyrise living walls
                        if (bType[y][x] == B_IND && deco[y][x] == 0) heatT -= 1.6f * recovery;                    // waste heat goes into the heating mains
                    }
                    break;
                case T_HOUSE: heatT = 1.4f * df + 0.4f; pol = 0.3f; break;
                case T_MALL:  heatT = 3.0f * df + 0.8f; pol = 1.2f; break;
                case T_PARK:  heatT = 1.6f; pol = 0.5f; break;
                case T_LANDFILL: heatT = 1.2f; pol = IsRecycleTile(x, y) ? 1.0f : 3.5f + landfillFill * 0.03f; break;
                case T_ROAD:  heatT = 1.2f + traffic[y][x] * 2.0f; pol = 0.4f + traffic[y][x] * 4.0f; break;
                case T_TREE:  heatT = -1.8f; pol = -0.6f; break;
                case T_GRASS: heatT = (deco[y][x] ? -0.9f : -0.4f); pol = (deco[y][x] ? -0.3f : -0.15f); break;
                case T_POWER: if (smartCity) { heatT = 1.5f; pol = 0.2f; } else { heatT = 7.0f * st.powerLoad; pol = 9.0f * st.powerLoad; } break;
                case T_RUIN:  heatT = 0.5f; break;
                default:      heatT = 1.0f; pol = 0.3f; break;
            }
            if (smartCity) {
                if (GreenRoof(x, y)) { heatT -= 0.9f; pol -= 0.25f; }                                          // sedum roofs cool and filter the air
                if (dhServed[y][x] && pol > 0.0f) pol *= 1.0f - 0.35f * coldK * (heatCover > 1.0f ? 1.0f : 0.0f);   // no boilers running in connected buildings
            } else if (surface[y][x] == T_ROAD && parkSearch > 0.5f) {
                bool nearPark = false;                                                                          // drivers circling the car park
                for (int d = 0; d < 4; d++) if (InBounds(x + DX[d], y + DY[d]) && surface[y + DY[d]][x + DX[d]] == T_PARK) nearPark = true;
                if (nearPark) { pol += parkSearch * 0.10f; heatT += parkSearch * 0.05f; }
            }
            if (fireI[y][x] > 0.0f) { heatT += 12.0f * fireI[y][x] / 100.0f; pol += 7.0f * fireI[y][x] / 100.0f; }
            heatT -= rain * 2.0f;
            heatMap[y][x] += (heatT - heatMap[y][x]) * 0.25f;
            polMap[y][x] += pol;
            if (polMap[y][x] < 0.0f) polMap[y][x] = 0.0f;
        }
    for (int i = 0; i < nBins; i++) if (bins[i].fill >= 100.0f) polMap[bins[i].y][bins[i].x] += 0.15f;     // overflowing bins stink
    Diffuse(heatMap, 0.30f, 1.0f);
    Diffuse(polMap, 0.25f, 0.965f - rain * 0.08f);
}

static void UpdateStats(void) {
    static const float eBase[3] = { 40.0f, 95.0f, 230.0f };    // kW per building type
    static const float wBase[3] = { 30.0f, 55.0f, 140.0f };    // L/min per building type
    int res = 0, houses = 0, bld = 0, dmg = 0, roads = 0, green = 0, plants = 0, works = 0, stations = 0, lit = 0;
    float eDem = 0.0f, wDem = 0.0f, df = DayFactor(), roofKW = 0.0f;
    int roofs = 0;

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            switch (surface[y][x]) {
                case T_ROAD: roads++; break;
                case T_BUILDING: {
                    int t = bType[y][x];
                    bld++;
                    if (t == B_RES) res++;
                    if (health[y][x] < 60.0f) dmg++;
                    eDem += eBase[t] * df;
                    wDem += wBase[t] * df;
                    if (!(t == B_IND && deco[y][x] == 2)) {
                        roofKW += (t == B_IND) ? (deco[y][x] == 0 ? 30.0f : 45.0f) : (t == B_COM ? 24.0f : 18.0f); roofs++;
                    }
                } break;
                case T_HOUSE:
                    bld++; houses++;
                    if (health[y][x] < 60.0f) dmg++;
                    eDem += 18.0f * df; wDem += 14.0f * df;
                    roofKW += 5.5f; roofs++;
                    break;
                case T_MALL:
                    bld++;
                    if (health[y][x] < 60.0f) dmg++;
                    eDem += 160.0f * (0.5f + 0.5f * ShopFactor()); wDem += 80.0f * ShopFactor();
                    roofKW += 40.0f; roofs++;
                    break;
                case T_HOSPITAL:
                    bld++;
                    if (health[y][x] < 60.0f) dmg++;
                    eDem += 150.0f * (0.7f + 0.3f * df); wDem += 90.0f * df;
                    roofKW += 45.0f; roofs++;
                    break;
                case T_PARK: lit++; break;
                case T_LANDFILL: wDem += 6.0f; break;
                case T_TREE: case T_GRASS: green++; break;
                case T_POWER: plants++; break;
                case T_WATER: works++; roofKW += 20.0f; roofs++; break;
                case T_FIRE_STN: case T_POLICE_STN: stations++; roofKW += 12.0f; roofs++; break;
                default: break;
            }
        }
    {   // street + car park lighting at night; smart city: LED lamps that dim until somebody comes past
        float lightKW = (roads * 1.6f + lit * 4.0f) * (1.0f - st.daylight);
        lightBaseKW = lightKW;
        if (smartCity) lightKW = (roads * 1.6f * 0.55f * lampAvg + lit * 4.0f * 0.55f) * (1.0f - st.daylight);
        lightNowKW = lightKW;
        eDem += stations * 30.0f + lightKW;
    }
    if (smartCity) eDem -= dhKW * 0.18f;                                              // district heating replaces electric heat pumps
    // traffic signals draw a little power as well
    for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) if (ctrl[y][x] == C_SIGNAL) eDem += 3.0f;
    demandBaseKW = eDem + (lightBaseKW - lightNowKW) + (smartCity ? dhKW * 0.18f : 0.0f);   // demand with none of the smart savings
    wDem += stations * 20.0f;
    float wBase0 = wDem;                                                               // what actually goes down the drain
    if (smartCity && rainTankKL > 0.5f) {                                              // harvested rain flushes toilets and waters the parks
        harvestLpm = wDem * 0.30f;
        rainTankKL = fmaxf(0.0f, rainTankKL - harvestLpm * 0.00025f);
        wDem -= harvestLpm;
    } else harvestLpm = 0.0f;
    wDem += leakLpm;                                                                   // pipe leaks add to what the works must supply

    st.buildings = bld; st.damaged = dmg;
    st.energyDemand = eDem; st.energyCap = plants * PLANT_KW;
    st.energyPct = (st.energyCap > 0.0f) ? eDem / st.energyCap * 100.0f : 999.0f;
    st.powerLoad = (st.energyCap > 0.0f) ? Clamp(eDem / st.energyCap, 0.0f, 1.0f) : 0.0f;
    if (smartCity) {
        // power plants are decommissioned: roofs + solar farm + wind farm, with a battery bank as buffer
        float sun = SunK();
        solarRoofs = roofs;
        roofSolarKW = roofKW * sun;
        farmSolarKW = (FARM_SOLAR_X1 - FARM_SOLAR_X0 + 1) * (FARM_SOLAR_Y1 - FARM_SOLAR_Y0 + 1) * SOLAR_TILE_KW * sun;
        windKW = N_TURB * TURB_KW * WindFactor();
        float supply = roofSolarKW + farmSolarKW + windKW;
        gridImportKW = 0.0f;
        // parked EVs: soak up surplus power, feed the grid back when supply falls short (V2G)
        float eEff = eDem, chg = 0.0f, dis = 0.0f;
        if (evCount > 0) {
            float poolKWs = evCount * EV_KWS_EACH, maxKW = evCount * EV_KW_EACH;
            if (supply > eDem) { if (evSoc < 99.0f) chg = fminf(maxKW, (supply - eDem) * 0.5f); }
            else if (evSoc > 35.0f) dis = fminf(maxKW, eDem - supply);
            evSoc = Clamp(evSoc + (chg * 0.97f - dis) * 0.5f / poolKWs * 100.0f, 0.0f, 100.0f);
            evKWhIn += chg * 0.5f / 3600.0f; evKWhOut += dis * 0.5f / 3600.0f;
            eEff = eDem + chg - dis;
        }
        v2gKW = dis - chg;                                                             // + = feeding the grid, - = charging
        eDem = eEff;
        if (supply >= eDem) batteryPct = fminf(100.0f, batteryPct + (supply - eDem) * 0.5f / BATT_KWS * 100.0f);
        else {
            float need = eDem - supply;
            if (batteryPct > 0.5f) batteryPct = fmaxf(0.0f, batteryPct - need * 0.5f / BATT_KWS * 100.0f);
            else gridImportKW = need;                               // battery flat: small backup grid tie
        }
        st.energyCap = supply;
        st.energyPct = st.energyDemand / (supply + (batteryPct > 3.0f ? 4000.0f : 0.0f) + 1.0f) * 100.0f;
        st.powerLoad = 0.0f;
        renewPct = (eDem > 1.0f) ? 100.0f * (1.0f - gridImportKW / eDem) : 100.0f;
    } else {
        gridImportKW = eDem; renewPct = 0.0f;                       // everything is burned at the power plant
    }
    st.waterDemand = wDem; st.waterCap = works * WORKS_LPM;
    st.waterPct = (st.waterCap > 0.0f) ? wDem / st.waterCap * 100.0f : 999.0f;
    st.sewerLoad = wBase0 * 0.85f + rain * roads * 35.0f * runoffK;     // waste water + storm run-off (smart city holds some of it back)
    st.sewerPct = st.sewerLoad / SEWER_CAP * 100.0f;

    float avgPol = 0.0f, avgHeat = 0.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) { avgPol += polMap[y][x]; avgHeat += heatMap[y][x]; }
    avgPol /= (MAP_W * MAP_H); avgHeat /= (MAP_W * MAP_H);
    st.aqi = 12.0f + avgPol * 3.0f;
    st.heatIsland = avgHeat;
    st.temp = st.ambient + avgHeat;
    st.rainMm = rain * 25.0f;
    st.green = green * 100.0f / (MAP_W * MAP_H);
    st.population = res * 22 + houses * 6 + births - deaths;
    if (st.population < 0) st.population = 0;

    {   // household + industrial waste; the smart city's recycling plant diverts most of it from the landfill
        float tonnes = st.population * 0.0006f;
        float toLandfill = smartCity ? tonnes * 0.36f : tonnes;
        if (smartCity) recycledT += tonnes * 0.64f;
        wasteT += toLandfill;
        landfillFill = fminf(100.0f, landfillFill + toLandfill * 0.01f);
    }

    int pending = 0;
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active && cars[i].wanted) pending++;
    float secTarget = Clamp(50.0f + 15.0f * nPolice - 10.0f * robbingNow - 2.0f * st.fires - 3.0f * pending, 0.0f, 100.0f);
    st.security += (secTarget - st.security) * 0.2f;

    float floodTarget = (st.sewerPct > 100.0f && rain > 0.15f) ? Clamp((st.sewerPct - 100.0f) / 40.0f, 0.0f, 1.0f) : 0.0f;
    floodLevel += (floodTarget - floodLevel) * 0.3f;
}

static void UpdateSim(float dt) {
    simTime += dt;
    UpdateClock();
    UpdateWeather(dt);
    UpdateSignals(dt);
    UpdatePeds(dt);          // first, so cars can see who is on a crossing
    UpdateCars(dt);
    UpdateResponders(dt);
    UpdateAmbulances(dt);
    UpdateDrones(dt);
    UpdateMedical(dt);
    UpdateWind(dt);
    UpdatePlus(dt);
    UpdateJobs(dt);
    UpdateBudget(dt);
    UpdateHappiness(dt);
    UpdateCarbon(dt);
    UpdateNoise(dt);
    UpdateFire(dt);
    EmitChimneys(dt);
    UpdateParticles(dt);

    dispatchTimer += dt;
    if (dispatchTimer >= 1.0f) { dispatchTimer = 0.0f; UpdateDispatch(); }
    parkTimer += dt;
    if (parkTimer >= 1.3f) { parkTimer = 0.0f; UpdateParking(); }
    mapTimer += dt;
    if (mapTimer >= 0.25f) { mapTimer = 0.0f; UpdateMaps(); }
    statTimer += dt;
    if (statTimer >= 0.5f) { statTimer = 0.0f; UpdateStats(); }
    histTimer += dt;
    if (histTimer >= 1.0f) { histTimer = 0.0f; PushHistory(); }
    birthTimer += dt;
    if (birthTimer >= 6.0f) { birthTimer = 0.0f; births += GetRandomValue(1, 3); }
}

// ---------------------------------------------------------------------------
// Colour lookups used by both views
// ---------------------------------------------------------------------------
static Color RoadColorAt(int x, int y, bool xray) {
    Color c = xray ? COL_ROAD_DIM : COL_ROAD;
    if (floodLevel > 0.02f) c = BlendOver(c, (Color){ 80, 140, 255, (unsigned char)(floodLevel * 130.0f) });
    return c;
}
static Color CongColor(float d, float alpha) {
    Color c = Ramp3((Color){ 60, 210, 120, 255 }, (Color){ 250, 205, 60, 255 }, (Color){ 235, 55, 50, 255 }, d / 1.4f);
    c.a = (unsigned char)(Clamp(alpha, 0.0f, 1.0f) * 255.0f);
    return c;
}
static Color HeatColorAt(int x, int y) {
    Color c = Ramp3((Color){ 50, 90, 255, 255 }, (Color){ 250, 225, 70, 255 }, (Color){ 235, 45, 40, 255 }, (heatMap[y][x] + 3.0f) / 15.0f);
    c.a = 150;
    return c;
}
static Color PolColorAt(int x, int y) {
    Color c = Ramp3((Color){ 90, 210, 110, 255 }, (Color){ 235, 200, 60, 255 }, (Color){ 140, 40, 150, 255 }, polMap[y][x] / 60.0f);
    c.a = 150;
    return c;
}
static Color SewerColor(void) {
    if (st.sewerPct > 100.0f) return LerpColor(COL_SEWER_PIPE, RED, 0.5f + 0.5f * sinf((float)GetTime() * 6.0f));
    return COL_SEWER_PIPE;
}

// ---- pipe networks (unchanged) ----
static Vector2 NodePos(int x, int y, float off) {
    return (Vector2){ x * TILE + TILE / 2.0f + off, y * TILE + TILE / 2.0f + off };
}

static void DrawNetwork2D(float off, Color col, Color dot, float flowSpeed) {
    float t = (float)GetTime();
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!pipeNode[y][x]) continue;
            Vector2 a = NodePos(x, y, off);
            DrawCircleV(a, surface[y][x] == T_ROAD ? 3.0f : 5.0f, col);
            for (int d = 1; d <= 3; d += 2) {                     // d=1 down, d=3 right
                int nx = x + DX[d], ny = y + DY[d];
                if (!InBounds(nx, ny) || !pipeNode[ny][nx]) continue;
                if (!IsRoad(x, y) && !IsRoad(nx, ny)) continue;
                Vector2 b = NodePos(nx, ny, off);
                DrawLineEx(a, b, 4.0f, col);
                float f = fmodf(t * flowSpeed + ((x * 7 + y * 13) % 10) * 0.1f, 1.0f);
                DrawCircleV((Vector2){ a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f }, 2.0f, dot);
            }
        }
}
// ---------------------------------------------------------------------------
// 2D drawing - smart city additions
// ---------------------------------------------------------------------------
static void DrawSolar2D(float x, float y, float w, float h, float a) {
    Color line = Fade((Color){ 125, 155, 205, 255 }, a);
    DrawRectangleRec((Rectangle){ x, y, w, h }, Fade((Color){ 26, 52, 122, 255 }, a));
    int nc = (int)(w / 4.0f); if (nc < 1) nc = 1;
    for (int i = 1; i < nc; i++) DrawLineV((Vector2){ x + i * w / nc, y }, (Vector2){ x + i * w / nc, y + h }, line);
    DrawLineV((Vector2){ x, y + h * 0.5f }, (Vector2){ x + w, y + h * 0.5f }, line);
    DrawRectangleLinesEx((Rectangle){ x, y, w, h }, 1.0f, Fade((Color){ 170, 190, 220, 255 }, a));
}

static void DrawLivingWall2D(int px, int py, float a) {
    Color g = Fade((Color){ 70, 185, 80, 255 }, a), lg = Fade((Color){ 150, 230, 110, 255 }, a);
    DrawRectangleLinesEx((Rectangle){ px + 4.0f, py + 4.0f, TILE - 8.0f, TILE - 8.0f }, 3.0f, g);
    for (int k = 0; k < 5; k++) {
        float o = 6.0f + k * 6.0f;
        DrawCircle((int)(px + o), py + 4, 2.4f, lg); DrawCircle((int)(px + o), py + TILE - 4, 2.4f, lg);
        DrawCircle(px + 4, (int)(py + o), 2.4f, lg); DrawCircle(px + TILE - 4, (int)(py + o), 2.4f, lg);
    }
    DrawRectangle(px + TILE / 2 - 4, py + TILE / 2 - 4, 8, 8, Fade((Color){ 60, 160, 70, 255 }, a));       // rooftop garden
}

static void DrawHospital2D(int px, int py, float a) {
    DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade((Color){ 240, 243, 247, 255 }, a));
    DrawRectangleLines(px + 3, py + 3, TILE - 6, TILE - 6, Fade((Color){ 150, 158, 170, 255 }, a));
    DrawRectangle(px + TILE / 2 - 3, py + 8, 6, 18, Fade((Color){ 215, 35, 40, 255 }, a));
    DrawRectangle(px + TILE / 2 - 9, py + 14, 18, 6, Fade((Color){ 215, 35, 40, 255 }, a));
    if (smartCity) DrawSolar2D(px + 5, py + TILE - 10, 14, 5, a);
}

static void DrawBattery2D(int px, int py, float a) {
    DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade((Color){ 58, 74, 66, 255 }, a));
    for (int i = 0; i < 2; i++)
        for (int k = 0; k < 2; k++) {
            DrawRectangle(px + 6 + i * 14, py + 6 + k * 11, 12, 8, Fade((Color){ 205, 232, 215, 255 }, a));
            DrawRectangle(px + 8 + i * 14, py + 9 + k * 11, 8, 2, Fade((Color){ 60, 215, 120, 255 }, a));
        }
    float f = Clamp(batteryPct / 100.0f, 0.0f, 1.0f);
    DrawRectangle(px + 6, py + TILE - 10, TILE - 12, 4, Fade((Color){ 30, 34, 40, 255 }, a));
    DrawRectangle(px + 6, py + TILE - 10, (int)((TILE - 12) * f), 4, Fade(f > 0.5f ? (Color){ 70, 230, 120, 255 } : (f > 0.2f ? (Color){ 250, 205, 50, 255 } : (Color){ 240, 70, 60, 255 }), a));
}

static void DrawRecycle2D(int x, int y, float a) {
    int px = x * TILE, py = y * TILE;
    unsigned char v = vr[y][x];
    static const Color bale[5] = { {70,110,190,255}, {190,195,202,255}, {205,170,110,255}, {85,160,95,255}, {225,225,120,255} };
    if (x == 20 && y == 11) {
        DrawRectangle(px + 2, py + 3, TILE - 4, TILE - 6, Fade((Color){ 70, 150, 115, 255 }, a));
        for (int k = 0; k < 3; k++) DrawRectangle(px + 4 + k * 10, py + 5, 8, TILE - 10, Fade((Color){ 90, 175, 135, 255 }, a));
        DrawCircle(px + TILE / 2, py + TILE / 2, 9.0f, Fade((Color){ 35, 150, 85, 255 }, a));
        for (int k = 0; k < 3; k++) {
            float ang = (k * 120.0f + 90.0f) * DEG2RAD;
            Vector2 c = { px + TILE / 2 + 5.0f * cosf(ang), py + TILE / 2 - 5.0f * sinf(ang) };
            DrawPoly(c, 3, 3.2f, (float)(k * 120) + 30.0f, Fade(WHITE, a));
        }
    } else if (x == 20) {
        DrawRectangle(px + 2, py + 4, TILE - 4, TILE - 8, Fade((Color){ 160, 205, 185, 255 }, a));
        DrawRectangle(px + 14, py, 6, TILE, Fade((Color){ 55, 58, 64, 255 }, a));
        float t = (float)GetTime();
        for (int i = 0; i < 5; i++) DrawRectangle(px + 15, py + (int)(fmodf(t * 12.0f + i * 7.0f, (float)TILE)), 4, 3, Fade(bale[(v + i) % 5], a));
        for (int i = 0; i < 3; i++) DrawRectangle(px + 24, py + 4 + i * 10, 8, 7, Fade(bale[(v + i) % 5], a));
    } else {
        for (int i = 0; i < 4; i++) {
            DrawRectangle(px + 4 + (i % 2) * 16, py + 5 + (i / 2) * 15, 13, 11, Fade(bale[(v + i) % 5], a));
            DrawRectangleLines(px + 4 + (i % 2) * 16, py + 5 + (i / 2) * 15, 13, 11, Fade((Color){ 40, 50, 55, 255 }, a));
        }
    }
}

static void DrawMonitor2D(int x, int y) {
    bool horiz = (RoadLine(x, y) == 1);
    float cx = x * TILE + (horiz ? TILE * 0.55f : TILE - 3.0f), cy = y * TILE + (horiz ? TILE - 3.0f : TILE * 0.55f);
    Color c = CongColor(traffic[y][x], 1.0f);
    DrawCircleV((Vector2){ cx, cy }, 5.2f, (Color){ 20, 26, 34, 255 });
    DrawCircleV((Vector2){ cx, cy }, 3.6f, c);
    DrawCircleV((Vector2){ cx, cy }, 1.4f, (Color){ 140, 230, 255, 255 });
}

static void DrawAmbulance2D(const Responder *r, double t) {
    float px = r->x * TILE, py = r->y * TILE;
    DrawRectanglePro((Rectangle){ px, py, 17, 9 }, (Vector2){ 8.5f, 4.5f }, r->angle, (Color){ 248, 248, 250, 255 });
    DrawRectanglePro((Rectangle){ px, py, 17, 2.5f }, (Vector2){ 8.5f, 1.25f }, r->angle, (Color){ 40, 165, 90, 255 });
    bool flash = ((int)(t * 6.0)) % 2 == 0;
    DrawRectangle((int)px - 1, (int)py - 3, 2, 6, (Color){ 215, 35, 40, 255 });
    DrawRectangle((int)px - 3, (int)py - 1, 6, 2, (Color){ 215, 35, 40, 255 });
    DrawCircle((int)px, (int)py - 5, 1.8f, flash ? (Color){ 255, 60, 60, 255 } : (Color){ 70, 130, 255, 255 });
}

static void DrawDrone2D(const Drone *d, double t) {
    float px = d->x * TILE, py = d->y * TILE, lift = d->z * 5.0f;
    DrawCircleV((Vector2){ px + 1.5f, py + 2.0f }, 5.0f, Fade(BLACK, 0.28f));                      // shadow on the ground
    Vector2 c = { px, py - lift };
    Color body = (d->kind == D_FIRE) ? (Color){ 220, 65, 55, 255 } : (d->kind == D_POLICE ? (Color){ 245, 247, 250, 255 } : (d->kind == D_PARCEL ? (Color){ 250, 170, 50, 255 } : (Color){ 250, 250, 250, 255 }));
    if (d->spraying) {
        Vector2 b = { d->sx * TILE, d->sy * TILE };
        if (d->kind == D_FIRE) DrawLineEx(c, b, 2.5f, (Color){ 130, 200, 255, 200 });
        else if (d->kind == D_POLICE) DrawLineEx(c, b, 1.6f, (Color){ 255, 250, 190, 150 });
    }
    float sp = (float)t * 30.0f;
    for (int k = 0; k < 4; k++) {
        float a = k * 90.0f * DEG2RAD + 0.785f;
        Vector2 r = { c.x + 6.0f * cosf(a), c.y + 6.0f * sinf(a) };
        DrawLineV(c, r, (Color){ 50, 52, 58, 255 });
        DrawCircleV(r, 3.6f, Fade(WHITE, 0.30f));
        DrawLineV((Vector2){ r.x + 3.6f * cosf(sp + k), r.y + 3.6f * sinf(sp + k) }, (Vector2){ r.x - 3.6f * cosf(sp + k), r.y - 3.6f * sinf(sp + k) }, (Color){ 30, 30, 34, 255 });
    }
    DrawCircleV(c, 3.4f, body);
    DrawCircleLines((int)c.x, (int)c.y, 3.4f, (Color){ 40, 44, 52, 255 });
    bool flash = ((int)(t * 6.0)) & 1;
    if (d->kind == D_POLICE) DrawCircleV(c, 1.4f, flash ? RED : SKYBLUE);
    else if (d->kind == D_MEDIC) { DrawRectangle((int)c.x - 1, (int)c.y - 2, 2, 5, (Color){ 215, 35, 40, 255 }); DrawRectangle((int)c.x - 2, (int)c.y - 1, 5, 2, (Color){ 215, 35, 40, 255 }); }
    if (d->kind == D_MEDSUP) { DrawRectangle((int)c.x - 1, (int)c.y - 2, 2, 5, (Color){ 40, 165, 90, 255 }); DrawRectangle((int)c.x - 2, (int)c.y - 1, 5, 2, (Color){ 40, 165, 90, 255 }); }
    if (d->carrying && d->kind == D_PARCEL) DrawRectangleRec((Rectangle){ c.x - 3, c.y + 4, 6, 5 }, (Color){ 200, 150, 80, 255 });
    else if (d->carrying && d->kind == D_MEDSUP) DrawRectangleRec((Rectangle){ c.x - 3, c.y + 4, 6, 5 }, (Color){ 245, 245, 248, 255 });
    else if (d->carrying) DrawRectangleRec((Rectangle){ c.x - 3, c.y + 4, 6, 4 }, WHITE);
}

// the renewable-energy strip to the east of the city (world pixels, drawn with a zoomed-out camera)
static void DrawFarm2D(void) {
    float t = (float)GetTime();
    int x0 = MAP_W * TILE, w = FARM_W * TILE;
    DrawRectangle(x0, 0, w, MAP_H * TILE, (Color){ 118, 164, 86, 255 });
    for (int y = 0; y < MAP_H; y++) if (y != FARM_ROAD_Y && (y & 1)) DrawRectangle(x0, y * TILE, w, TILE, Fade((Color){ 95, 140, 70, 255 }, 0.35f));
    // access road
    DrawRectangle(x0, FARM_ROAD_Y * TILE, w, TILE, COL_ROAD);
    DrawRectangle(x0 + TILE / 4, FARM_ROAD_Y * TILE + TILE / 2 - 1, w - TILE / 2, LINE_THICK, COL_LINE);
    // solar field
    for (int y = FARM_SOLAR_Y0; y <= FARM_SOLAR_Y1; y++)
        for (int x = FARM_SOLAR_X0; x <= FARM_SOLAR_X1; x++)
            for (int i = 0; i < 2; i++)
                for (int k = 0; k < 2; k++)
                    DrawSolar2D(x * TILE + 2 + i * 17, y * TILE + 3 + k * 17, 15, 12, 1.0f);
    // grid hub (battery containers + transformer)
    DrawRectangle(24 * TILE + 3, 11 * TILE + 3, TILE - 6, 3 * TILE - 6, (Color){ 120, 126, 132, 255 });
    for (int k = 0; k < 3; k++) {
        DrawRectangle(24 * TILE + 6, (11 + k) * TILE + 6, TILE - 12, TILE - 12, (Color){ 205, 232, 215, 255 });
        for (int s = 0; s < 4; s++) DrawRectangle(24 * TILE + 9 + s * 5, (11 + k) * TILE + 14, 3, 8, (Color){ 60, 215, 120, 255 });
    }
    // wind turbines: three rotating blades
    for (int i = 0; i < N_TURB; i++) {
        Vector2 c = { TURB_POS[i][0] * TILE, TURB_POS[i][1] * TILE };
        DrawCircleV((Vector2){ c.x + 3, c.y + 3 }, 4.0f, Fade(BLACK, 0.25f));
        float ang = t * (0.5f + 5.0f * WindFactor()) + i;
        for (int b = 0; b < 3; b++) {
            float a = ang + b * 2.094f;
            DrawLineEx(c, (Vector2){ c.x + 16.0f * cosf(a), c.y + 16.0f * sinf(a) }, 2.2f, (Color){ 245, 247, 250, 255 });
        }
        DrawCircleV(c, 3.2f, (Color){ 210, 215, 222, 255 });
    }
    DrawRectangleLinesEx((Rectangle){ (float)x0, 0.0f, (float)w, (float)(MAP_H * TILE) }, 2.0f, (Color){ 235, 235, 240, 255 });
    DrawText("RENEWABLE FARM", x0 + 8, MAP_H * TILE - 14, 10, (Color){ 20, 50, 20, 255 });
    DrawText("solar + wind", x0 + 8, 10 * TILE + 6, 10, (Color){ 20, 50, 20, 255 });
}
// ---------------------------------------------------------------------------
// 2D drawing - smart city extras (lighting, bins, leaks, sensors, heating, shuttles, delivery ...)
// ---------------------------------------------------------------------------
static Vector2 Edge2D(int x, int y, int d, float out, float along) {   // point beside tile (x,y) towards direction d (0.5 = tile edge)
    float tx = (float)(DY[d] != 0 ? 1 : 0), ty = (float)(DX[d] != 0 ? 1 : 0);   // tangent
    return (Vector2){ (x + 0.5f + DX[d] * out + tx * along) * TILE, (y + 0.5f + DY[d] * out + ty * along) * TILE };
}

static void DrawBinTruck2D(const Responder *r) {
    float px = r->x * TILE, py = r->y * TILE;
    DrawRectanglePro((Rectangle){ px, py, 19, 9 }, (Vector2){ 9.5f, 4.5f }, r->angle, (Color){ 60, 150, 90, 255 });
    DrawRectanglePro((Rectangle){ px, py, 7, 7 }, (Vector2){ -4.0f, 3.5f }, r->angle, (Color){ 235, 235, 235, 255 });
    DrawCircleV((Vector2){ px, py }, 2.0f, (Color){ 250, 205, 40, 255 });
}

static void DrawDeliveryMark2D(const DropMark *m) {
    float k = Clamp(m->t / 7.0f, 0.0f, 1.0f);
    float px = m->x * TILE, py = m->y * TILE;
    Color c = (m->kind == D_PARCEL) ? (Color){ 200, 150, 80, 255 } : (Color){ 245, 245, 248, 255 };
    DrawRectangle((int)px - 3, (int)py - 3, 6, 6, Fade(c, 0.4f + 0.6f * k));
    if (m->kind == D_MEDSUP) { DrawRectangle((int)px - 1, (int)py - 2, 2, 4, Fade((Color){ 215, 35, 40, 255 }, k)); DrawRectangle((int)px - 2, (int)py - 1, 4, 2, Fade((Color){ 215, 35, 40, 255 }, k)); }
    DrawCircleLines((int)px, (int)py, 6.0f + (1.0f - k) * 8.0f, Fade((Color){ 120, 230, 160, 255 }, k));
}

// things lying on the ground: heating mains, EV chargers, bay sensors, shuttle shelters, drone hub, bins, leaks
static void DrawPlusLow2D(void) {
    double t = GetTime();
    // ---- bins (both cities); smart bins have a fill LED ----
    for (int i = 0; i < nBins; i++) {
        const Bin *b = &bins[i];
        Vector2 p = Edge2D(b->x, b->y, b->side, 0.40f, 0.20f);
        DrawRectangle((int)p.x - 3, (int)p.y - 3, 6, 6, smartCity ? (Color){ 40, 110, 80, 255 } : (Color){ 95, 100, 106, 255 });
        DrawRectangleLines((int)p.x - 3, (int)p.y - 3, 6, 6, (Color){ 20, 24, 28, 255 });
        if (smartCity) {
            Color led = (b->fill >= 95.0f) ? RED : (b->fill >= 75.0f ? (Color){ 245, 160, 40, 255 } : (b->fill >= 50.0f ? (Color){ 245, 220, 60, 255 } : (Color){ 70, 220, 110, 255 }));
            DrawCircleV(p, 1.8f, led);
        }
        if (b->fill >= 100.0f) DrawCircleLines((int)p.x, (int)p.y, 6.0f + 1.5f * sinf((float)t * 7.0f), (Color){ 245, 140, 40, 255 });
    }
    // ---- water leaks (both cities) ----
    for (int i = 0; i < MAX_LEAKS; i++) {
        const Leak *l = &leaks[i];
        if (!l->active) continue;
        float cx = (l->x + 0.5f) * TILE, cy = (l->y + 0.5f) * TILE, pl = 0.5f + 0.5f * sinf((float)t * 6.0f + i);
        DrawCircleV((Vector2){ cx, cy }, 7.0f + 3.0f * pl, Fade((Color){ 80, 150, 255, 255 }, 0.55f));
        DrawCircleV((Vector2){ cx, cy }, 3.0f, (Color){ 200, 225, 255, 255 });
        for (int k = 0; k < 3; k++) DrawCircleV((Vector2){ cx + 6.0f * cosf((float)t * 3.0f + k * 2.1f), cy - 4.0f - 5.0f * fabsf(sinf((float)t * 4.0f + k)) }, 1.3f, (Color){ 190, 220, 255, 255 });
        if (l->detected) {
            DrawCircleLines((int)cx, (int)cy, 11.0f, smartCity ? (Color){ 70, 230, 200, 255 } : (Color){ 245, 190, 50, 255 });
            DrawRectangle((int)cx - 1, (int)cy - 18, 2, 5, (Color){ 245, 190, 50, 255 }); DrawRectangle((int)cx - 1, (int)cy - 11, 2, 2, (Color){ 245, 190, 50, 255 });
        }
    }
    if (!smartCity) return;

    // ---- district heating mains + heat exchanger ----
    {
        int hx = HEATPLANT_X * TILE, hy = HEATPLANT_Y * TILE;
        DrawRectangle(hx + 6, hy + 7, TILE - 12, TILE - 14, (Color){ 120, 126, 132, 255 });
        DrawCircle(hx + TILE / 2 - 5, hy + TILE / 2, 6.0f, (Color){ 235, 120, 50, 255 });
        DrawCircle(hx + TILE / 2 + 7, hy + TILE / 2, 4.0f, (Color){ 250, 190, 70, 255 });
        DrawLineEx((Vector2){ (HEATPLANT_X + 0.5f) * TILE, HEATPLANT_Y * TILE + 4.0f }, (Vector2){ (HEATPLANT_X + 0.5f) * TILE, 10.04f * TILE }, 2.5f, (Color){ 235, 120, 50, 255 });
        for (int k = 0; k < N_DH; k++) {
            float x0 = DH_SEG[k][0], y0 = DH_SEG[k][1], x1 = DH_SEG[k][2], y1 = DH_SEG[k][3];
            float len = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            int n = (int)(len * 4.0f);
            Vector2 prev = { 0, 0 }; bool havePrev = false;
            for (int s = 0; s <= n; s++) {
                float u = (float)s / n, px = x0 + (x1 - x0) * u, py = y0 + (y1 - y0) * u;
                bool road = IsRoad((int)px, (int)py);
                Vector2 p = { px * TILE, py * TILE };
                if (!road && havePrev) DrawLineEx(prev, p, 2.6f, (Color){ 225, 110, 45, 255 });
                prev = p; havePrev = !road;
            }
            for (int d = 0; d < (int)(len / 1.4f) + 1; d++) {
                float u = fmodf((float)t * 0.9f + d * 1.4f, len) / len, px = x0 + (x1 - x0) * u, py = y0 + (y1 - y0) * u;
                if (!IsRoad((int)px, (int)py)) DrawCircleV((Vector2){ px * TILE, py * TILE }, 1.7f, (Color){ 255, 225, 120, 255 });
            }
        }
    }
    // ---- drone fulfilment hub (loading yard) ----
    {
        float cx = (HUB_X + 0.5f) * TILE, cy = (HUB_Y + 0.5f) * TILE;
        DrawCircleV((Vector2){ cx, cy }, 11.0f, (Color){ 52, 56, 64, 255 });
        DrawCircleLines((int)cx, (int)cy, 11.0f, (Color){ 250, 205, 40, 255 });
        DrawRectangle((int)cx - 5, (int)cy - 1, 10, 2, WHITE); DrawRectangle((int)cx - 1, (int)cy - 5, 2, 10, WHITE);
        DrawRectangle((int)cx + 9, (int)cy + 6, 7, 6, (Color){ 200, 150, 80, 255 });
    }
    // ---- shuttle shelters ----
    for (int i = 0; i < N_STOPS; i++) {
        int sx = SH_STOP[i][0], sy = SH_STOP[i][1], d = -1;
        for (int k = 0; k < 4; k++) if (InBounds(sx + DX[k], sy + DY[k]) && !IsRoad(sx + DX[k], sy + DY[k])) { d = k; break; }
        if (d < 0) continue;
        Vector2 p = Edge2D(sx, sy, d, 0.58f, 0.0f);
        DrawRectangle((int)p.x - 6, (int)p.y - 3, 12, 6, (Color){ 0, 190, 165, 255 });
        DrawRectangleLines((int)p.x - 6, (int)p.y - 3, 12, 6, WHITE);
    }
    // ---- car park: bay sensors, chargers, guidance signs ----
    Color chargeCol = (v2gKW > 20.0f) ? (Color){ 90, 170, 255, 255 } : (v2gKW < -20.0f ? (Color){ 70, 230, 120, 255 } : (Color){ 220, 224, 230, 255 });
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] != T_PARK) continue;
            int px = x * TILE, py = y * TILE;
            for (int s = 0; s < 8; s++) {
                int col = s % 4, row = s / 4;
                float lx = px + col * TILE / 4.0f + TILE / 8.0f, ly = py + (row ? TILE / 2 + 3.0f : TILE / 2 - 3.0f);
                DrawCircleV((Vector2){ lx, ly }, 1.2f, lot[y][x][s] ? (Color){ 235, 60, 55, 255 } : (Color){ 70, 230, 110, 255 });
            }
            if (((x + y) & 1) == 1) {
                DrawRectangle(px + TILE / 2 - 2, py + TILE / 2 - 2, 4, 4, (Color){ 40, 44, 52, 255 });
                DrawRectangle(px + TILE / 2 - 1, py + TILE / 2 - 1, 2, 2, chargeCol);
            }
        }
    for (int i = 0; i < 2; i++) {
        int x = PSIGN[i][0], y = PSIGN[i][1];
        int bx = x * TILE + 2, by = y * TILE + TILE - 12;
        DrawRectangle(bx, by, 30, 10, (Color){ 12, 14, 18, 255 });
        DrawRectangleLines(bx, by, 30, 10, (Color){ 120, 130, 145, 255 });
        DrawText(parkFree > 0 ? TextFormat("P %d", parkFree) : "FULL", bx + 3, by + 1, 8, parkFree > 8 ? (Color){ 70, 230, 110, 255 } : (Color){ 245, 90, 70, 255 });
    }
}

// things above the ground: night lighting, sensors, deliveries, refuse truck
static void DrawPlusHigh2D(void) {
    double t = GetTime();
    // ---- street lamps (both cities) ----
    if (st.daylight < 0.45f && layerOn[L_ROADS]) {
        float dark = Clamp((0.45f - st.daylight) / 0.25f, 0.0f, 1.0f);
        for (int y = 0; y < MAP_H; y++)
            for (int x = 0; x < MAP_W; x++) {
                if (!lampTile[y][x]) continue;
                int d = LampDir(x, y);
                if (d < 0) continue;
                float lv = lampLvl[y][x];
                Vector2 p = Edge2D(x, y, d, 0.40f, 0.0f);
                DrawCircleV(p, 6.0f + 13.0f * lv, Fade((Color){ 255, 225, 140, 255 }, (0.07f + 0.16f * lv) * dark));
                DrawCircleV(p, 1.4f + 1.1f * lv, Fade((Color){ 255, 244, 190, 255 }, 0.5f + 0.5f * lv));
            }
    }
    if (binTruck.active) DrawBinTruck2D(&binTruck);
    for (int k = 0; k < N_DROPS; k++) if (drops[k].t > 0.0f) DrawDeliveryMark2D(&drops[k]);
    if (!smartCity) return;

    // ---- air-quality sensors with a live AQI chip ----
    for (int i = 0; i < N_AQ; i++) {
        Vector2 p = { (AQ_POS[i][0] + 0.5f) * TILE, (AQ_POS[i][1] + 0.5f) * TILE };
        Color c = AqColor(aqLevel[i]);
        DrawCircleV(p, 5.0f, (Color){ 20, 26, 34, 255 });
        DrawCircleV(p, 3.5f, c);
        const char *txt = TextFormat("%.0f", aqLevel[i]);
        int w = MeasureText(txt, 10) + 6;
        DrawRectangle((int)p.x - w / 2, (int)p.y - 20, w, 12, Fade((Color){ 14, 18, 24, 255 }, 0.85f));
        DrawRectangleLines((int)p.x - w / 2, (int)p.y - 20, w, 12, c);
        DrawText(txt, (int)p.x - w / 2 + 3, (int)p.y - 18, 10, c);
    }
    // ---- flood sensors + storm-drain gates ----
    for (int i = 0; i < N_FLOOD; i++) {
        Vector2 p = { (FLOOD_POS[i][0] + 0.78f) * TILE, (FLOOD_POS[i][1] + 0.78f) * TILE };
        Color c = floodState[i] == 2 ? (Color){ 235, 55, 50, 255 } : (floodState[i] == 1 ? (Color){ 245, 190, 50, 255 } : (Color){ 70, 160, 255, 255 });
        DrawRectangle((int)p.x - 2, (int)p.y - 5, 4, 9, (Color){ 30, 40, 55, 255 });
        DrawCircleV((Vector2){ p.x, p.y - 6.0f }, 2.6f, c);
        if (drainOpen > 0.1f) DrawRectangle((int)p.x - 7, (int)p.y + 5, 14, 3, Fade((Color){ 90, 170, 255, 255 }, 0.4f + 0.5f * (0.5f + 0.5f * sinf((float)t * 6.0f + i))));
    }
}

// farm strip: rain tanks + stormwater retention basin (world pixels)
static void DrawPlusFarm2D(void) {
    int x0 = MAP_W * TILE, y0 = 8 * TILE;
    float f = Clamp(rainTankKL / RAIN_TANK_CAP, 0.0f, 1.0f);
    for (int i = 0; i < 2; i++) {
        Vector2 c = { x0 + (0.8f + i * 1.15f) * TILE, y0 + TILE * 0.5f };
        DrawCircleV(c, 14.0f, (Color){ 200, 208, 214, 255 });
        DrawCircleV(c, 11.0f, (Color){ 40, 70, 110, 255 });
        DrawCircleV(c, 11.0f * f, (Color){ 90, 170, 255, 255 });
    }
    Rectangle pond = { x0 + 2.7f * TILE, y0 + 5.0f, 3.1f * TILE, TILE - 10.0f };
    DrawRectangleRec(pond, (Color){ 120, 100, 70, 255 });
    DrawRectangleRec((Rectangle){ pond.x + 2, pond.y + 2, pond.width - 4, pond.height - 4 }, (Color){ 45, 95, 160, 255 });
    float rf = Clamp(retentionKL / RETENTION_CAP, 0.0f, 1.0f);
    DrawRectangleRec((Rectangle){ pond.x + 2, pond.y + 2 + (pond.height - 4) * (1.0f - rf), pond.width - 4, (pond.height - 4) * rf }, (Color){ 110, 185, 255, 255 });
    DrawText("RAIN TANKS", x0 + 6, y0 - 2, 8, (Color){ 20, 50, 20, 255 });
    DrawText("STORM BASIN", (int)pond.x + 4, (int)pond.y - 9, 8, (Color){ 20, 50, 20, 255 });
}

// ---------------------------------------------------------------------------
// 2D drawing
// ---------------------------------------------------------------------------
static Color WallColor(unsigned char v) {
    static const Color w[6] = { {226,214,190,255}, {205,150,125,255}, {236,236,230,255}, {190,170,140,255}, {170,190,205,255}, {215,190,150,255} };
    return w[v % 6];
}
static Color RoofColor(unsigned char v) {
    static const Color r[5] = { {150,70,55,255}, {85,88,98,255}, {120,85,60,255}, {70,100,80,255}, {140,60,60,255} };
    return r[(v >> 3) % 5];
}

static void DrawZebra2D(int x, int y) {
    int px = x * TILE, py = y * TILE;
    for (int i = 0; i < 5; i++) {
        float o = (0.14f + i * 0.18f) * TILE;
        if (crossT[y][x] == 1) DrawRectangle(px + (int)o - 1, py + (int)(0.34f * TILE), 3, (int)(0.32f * TILE), Fade(WHITE, 0.9f));
        else                   DrawRectangle(px + (int)(0.34f * TILE), py + (int)o - 1, (int)(0.32f * TILE), 3, Fade(WHITE, 0.9f));
    }
}

static void DrawJunction2D(int x, int y, bool xray) {
    int px = x * TILE, py = y * TILE, cx = px + TILE / 2, cy = py + TILE / 2;
    if (ctrl[y][x] == C_ROUNDABOUT) {
        DrawCircle(cx, cy, TILE * 0.40f, Fade(WHITE, 0.55f));
        DrawCircle(cx, cy, TILE * 0.37f, xray ? COL_ROAD_DIM : COL_ROAD);
        DrawCircle(cx, cy, TILE * 0.20f, (Color){ 70, 140, 65, 255 });
        DrawCircle(cx, cy, TILE * 0.10f, (Color){ 55, 115, 55, 255 });
    } else if (ctrl[y][x] == C_STOP) {
        float o[4][2] = { {0.14f,0.14f}, {0.86f,0.14f}, {0.86f,0.86f}, {0.14f,0.86f} };
        for (int i = 0; i < 4; i++) DrawPoly((Vector2){ px + o[i][0] * TILE, py + o[i][1] * TILE }, 8, 3.4f, 22.5f, RED);
    } else if (ctrl[y][x] == C_SIGNAL) {
        float o[4][2] = { {0.14f,0.14f}, {0.86f,0.14f}, {0.86f,0.86f}, {0.14f,0.86f} };
        int axisOf[4] = { 1, 0, 1, 0 };                // NW: E-W, NE: N-S, SE: E-W, SW: N-S
        for (int i = 0; i < 4; i++) {
            int s = SigState(x, y, axisOf[i], NULL);
            Color c = (s == 1) ? (Color){ 60, 230, 90, 255 } : (s == 2 ? (Color){ 250, 200, 40, 255 } : (Color){ 240, 50, 50, 255 });
            DrawCircle((int)(px + o[i][0] * TILE), (int)(py + o[i][1] * TILE), 4.4f, (Color){ 25, 25, 28, 255 });
            DrawCircle((int)(px + o[i][0] * TILE), (int)(py + o[i][1] * TILE), 2.6f, c);
        }
    }
}

static void DrawGround2D(int x, int y, bool xray) {
    int px = x * TILE, py = y * TILE;
    if (surface[y][x] == T_ROAD && layerOn[L_ROADS]) {
        DrawRectangle(px, py, TILE, TILE, RoadColorAt(x, y, xray));
        int ln = RoadLine(x, y);
        Color lc = xray ? (Color){ 150, 150, 150, 255 } : COL_LINE;
        if (ln == 1) DrawRectangle(px + TILE / 4, py + TILE / 2 - LINE_THICK / 2, TILE / 2, LINE_THICK, lc);
        else if (ln == 2) DrawRectangle(px + TILE / 2 - LINE_THICK / 2, py + TILE / 4, LINE_THICK, TILE / 2, lc);
        if (crossT[y][x]) DrawZebra2D(x, y);
        if (ctrl[y][x]) DrawJunction2D(x, y, xray);
    } else {
        Color base = xray ? COL_SOIL : COL_GRASS;
        int s = surface[y][x];
        if (s == T_PARK) base = xray ? COL_ROAD_DIM : (Color){ 88, 88, 94, 255 };
        else if (s == T_LANDFILL) base = IsRecycleTile(x, y) ? (Color){ 150, 150, 152, 255 } : (Color){ 112, 94, 72, 255 };
        else if (paved[y][x]) base = xray ? COL_ROAD_DIM : (Color){ 150, 150, 152, 255 };
        else if (s == T_HOUSE) base = xray ? COL_SOIL : (Color){ 92, 168, 76, 255 };
        DrawRectangle(px, py, TILE, TILE, base);
        if (s == T_PARK && !xray) {                      // parking bays
            for (int c = 0; c <= 4; c++) DrawRectangle(px + c * TILE / 4, py + 2, 1, TILE - 4, Fade(WHITE, 0.55f));
            DrawRectangle(px + 2, py + TILE / 2, TILE - 4, 1, Fade(WHITE, 0.25f));
        }
        if (layerOn[L_ROADS] && s != T_ROAD) {
            Rectangle r[8];
            int n = SidewalkRects(x, y, r);
            for (int i = 0; i < n; i++)
                DrawRectangle(px + (int)(r[i].x * TILE), py + (int)(r[i].y * TILE),
                              (int)(r[i].width * TILE + 0.5f), (int)(r[i].height * TILE + 0.5f),
                              xray ? COL_SIDEWALK_DIM : COL_SIDEWALK);
        }
        if (s == T_GRASS && !xray && deco[y][x] && !paved[y][x]) {
            if (deco[y][x] == 1) { DrawCircle(px + 12, py + 14, 4, (Color){ 50, 125, 55, 255 }); DrawCircle(px + 22, py + 22, 3.5f, (Color){ 60, 140, 60, 255 }); }
            else if (deco[y][x] == 2) DrawRectangle(px + 5, py + TILE / 2 - 3, TILE - 10, 6, (Color){ 36, 105, 48, 255 });
            else { for (int k = 0; k < 5; k++) DrawCircle(px + 8 + (k * 7) % 22, py + 10 + (k * 11) % 18, 1.8f, k % 2 ? (Color){ 240, 90, 120, 255 } : (Color){ 250, 220, 80, 255 }); }
        }
    }
}

// fences around gated communities, the industrial zone and the landfill
static void DrawFences2D(void) {
    if (!layerOn[L_BUILDINGS]) return;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int z = zone[y][x];
            if (!ZoneFenced(z)) continue;
            int px = x * TILE, py = y * TILE;
            Color fc = (z <= Z_GATED3) ? (Color){ 245, 245, 240, 255 } : (Color){ 70, 74, 80, 255 };
            for (int d = 0; d < 4; d++) {
                int nx = x + DX[d], ny = y + DY[d];
                if (InBounds(nx, ny) && zone[ny][nx] == z) continue;
                bool gate = gateTile[y][x] && IsRoad(nx, ny);
                int in = SIDEWALK_W + 1;
                int x0 = px, y0 = py, w = TILE, h = TILE;
                Rectangle r = { 0, 0, 0, 0 };
                if (d == 0) r = (Rectangle){ x0, y0 + in, w, 2 };
                if (d == 1) r = (Rectangle){ x0, y0 + h - in - 2, w, 2 };
                if (d == 2) r = (Rectangle){ x0 + in, y0, 2, h };
                if (d == 3) r = (Rectangle){ x0 + w - in - 2, y0, 2, h };
                if (gate) {
                    DrawRectangleRec(r, Fade(fc, 0.0f));
                    Color gc = (Color){ 235, 190, 40, 255 };
                    if (d < 2) { DrawRectangle(x0 + 4, (int)r.y - 1, 4, 4, gc); DrawRectangle(x0 + w - 8, (int)r.y - 1, 4, 4, gc); }
                    else       { DrawRectangle((int)r.x - 1, y0 + 4, 4, 4, gc); DrawRectangle((int)r.x - 1, y0 + h - 8, 4, 4, gc); }
                } else DrawRectangleRec(r, fc);
            }
        }
}

static void DrawVehicleBody2D(float x, float y, float yawDeg, int type, Color col, bool braking) {
    float px = x * TILE, py = y * TILE;
    float ang = 90.0f - yawDeg;                       // screen rotation (x axis = heading)
    float L = VLEN[type] * TILE, W = VWID[type] * TILE;
    if (type == V_BIKE) {
        DrawRectanglePro((Rectangle){ px, py, L, 2.0f }, (Vector2){ L / 2, 1.0f }, ang, (Color){ 40, 40, 44, 255 });
        DrawCircleV((Vector2){ px, py }, 2.4f, col);
        return;
    }
    DrawRectanglePro((Rectangle){ px, py, L, W }, (Vector2){ L / 2, W / 2 }, ang, col);
    if (type == V_VAN || type == V_TRUCK) {
        DrawRectanglePro((Rectangle){ px, py, L * 0.30f, W * 0.8f }, (Vector2){ -L * 0.18f, W * 0.4f }, ang, (Color){ 60, 70, 85, 255 });
    } else {
        float cl = L * 0.34f;
        DrawRectanglePro((Rectangle){ px, py, cl, W * 0.72f }, (Vector2){ cl / 2 - L * 0.04f, W * 0.36f }, ang, (Color){ 40, 50, 65, 255 });
    }
    if (braking) {
        float bx = px - cosf(ang * DEG2RAD) * L * 0.5f, by = py - sinf(ang * DEG2RAD) * L * 0.5f;
        DrawCircleV((Vector2){ bx, by }, 1.6f, RED);
    }
}

static void DrawCar2D(const Car *c) {
    DrawVehicleBody2D(c->x, c->y, c->yaw, c->type, c->col, c->braking);
    if (c->shuttle) {                                                   // autonomous shuttle: roof sensor + passenger count
        DrawCircleV((Vector2){ c->x * TILE, c->y * TILE }, 3.2f, WHITE);
        DrawCircleV((Vector2){ c->x * TILE, c->y * TILE }, 1.6f, (Color){ 0, 150, 130, 255 });
        DrawText(TextFormat("%d", c->shPax), (int)(c->x * TILE) - 3, (int)(c->y * TILE) - 15, 10, (Color){ 120, 255, 220, 255 });
    }
    if (c->wanted) {
        float f = 0.5f + 0.5f * sinf((float)GetTime() * 10.0f);
        DrawCircleLines((int)(c->x * TILE), (int)(c->y * TILE), 7.0f + 2.0f * f, c->fleeing ? RED : ORANGE);
    }
}

static void DrawUCars2D(void) {                      // parked and player-driven cars
    for (int i = 0; i < MAX_UCARS; i++) if (ucars[i].active) DrawVehicleBody2D(ucars[i].x, ucars[i].z, ucars[i].yaw, ucars[i].type, CAR_COLS[ucars[i].colIdx], ucars[i].braking);
}

static void DrawParkedCars2D(int x, int y) {
    int px = x * TILE, py = y * TILE;
    if (surface[y][x] == T_PARK) {
        for (int s = 0; s < 8; s++) {
            if (!lot[y][x][s]) continue;
            int col = s % 4, row = s / 4;
            DrawRectangle(px + col * TILE / 4 + 2, py + 3 + row * (TILE / 2), TILE / 4 - 4, TILE / 2 - 7, Fade(CAR_COLS[lotCol[y][x][s]], gAlpha));
        }
    }
}

static void DrawStructure2D(int x, int y) {
    int px = x * TILE, py = y * TILE;
    switch (surface[y][x]) {
        case T_BUILDING: {
            Color c = (health[y][x] < 50.0f) ? COL_DAMAGED : COL_BTYPE[bType[y][x]];
            if (bType[y][x] == B_IND) {
                Color ic = (health[y][x] < 50.0f) ? COL_DAMAGED : (Color){ 130, 134, 140, 255 };
                if (deco[y][x] == 2) {                        // tank farm
                    DrawCircle(px + TILE / 2, py + TILE / 2, TILE * 0.36f, Fade((Color){ 205, 208, 212, 255 }, gAlpha));
                    DrawCircleLines(px + TILE / 2, py + TILE / 2, TILE * 0.36f, Fade(DARKGRAY, gAlpha));
                } else if (deco[y][x] == 1) {                 // warehouse
                    DrawRectangle(px + 3, py + 7, TILE - 6, TILE - 14, Fade(ic, gAlpha));
                    for (int k = 0; k < 3; k++) DrawRectangle(px + 7 + k * 9, py + TILE - 13, 6, 5, Fade((Color){ 80, 90, 100, 255 }, gAlpha));
                } else {                                      // factory with chimney
                    DrawRectangle(px + 5, py + 8, TILE - 10, TILE - 13, Fade(ic, gAlpha));
                    DrawRectangle(px + TILE - 13, py + 2, 5, 11, Fade((Color){ 55, 55, 55, 255 }, gAlpha));
                    if (smartCity) {                              // smoke trap: scrubber beside the stack, green "clean" cap
                        DrawRectangle(px + TILE - 22, py + 5, 7, 10, Fade((Color){ 70, 150, 150, 255 }, gAlpha));
                        DrawRectangle(px + TILE - 13, py + 1, 5, 3, Fade((Color){ 80, 215, 130, 255 }, gAlpha));
                        DrawSolar2D((float)(px + 7), (float)(py + 20), 14.0f, 7.0f, gAlpha);
                    }
                }
                if (smartCity && deco[y][x] == 1) DrawSolar2D((float)(px + 7), (float)(py + 9), 22.0f, 8.0f, gAlpha);
                break;
            }
            DrawRectangle(px + 5, py + 5, TILE - 10, TILE - 10, Fade(c, gAlpha));
            DrawRectangleLines(px + 5, py + 5, TILE - 10, TILE - 10, Fade((Color){ 60, 40, 35, 255 }, gAlpha));
            Color wc = (st.daylight < 0.5f) ? (Color){ 255, 230, 120, 255 } : (Color){ 215, 225, 235, 255 };
            for (int wy = 0; wy < 2; wy++)
                for (int wx = 0; wx < 2; wx++)
                    DrawRectangle(px + 9 + wx * 10, py + 9 + wy * 10, 5, 5, Fade(wc, gAlpha));
            if (GreenRoof(x, y)) DrawRectangle(px + 6, py + 6, TILE - 12, TILE - 12, Fade((Color){ 96, 178, 84, 255 }, gAlpha * 0.9f));
            if (smartCity) {
                if (bType[y][x] == B_COM && bHeight[y][x] >= 1.0f) DrawLivingWall2D(px, py, gAlpha);
                else { DrawSolar2D((float)(px + 7), (float)(py + 7), 10.0f, 9.0f, gAlpha); DrawSolar2D((float)(px + 19), (float)(py + 7), 10.0f, 9.0f, gAlpha); }
            }
        } break;
        case T_HOUSE: {
            unsigned char v = vr[y][x];
            int f = faceDir[y][x];
            Color wall = (health[y][x] < 50.0f) ? COL_DAMAGED : WallColor(v);
            Color roof = RoofColor(v);
            DrawRectangle(px + 7, py + 7, 19, 17, Fade(wall, gAlpha));
            DrawRectangle(px + 8, py + 8, 17, 15, Fade(roof, gAlpha));
            DrawRectangle(px + TILE / 2 - 1, py + 8, 2, 15, Fade(Shade(roof, 0.8f), gAlpha));
            // garage + driveway on the street-facing side
            int gx = px + 20, gy = py + 14;
            if (f == 0) { gy = py + 2; } else if (f == 1) { gy = py + TILE - 8; }
            DrawRectangle(gx, gy, 10, 6, Fade((Color){ 180, 180, 184, 255 }, gAlpha));
            if (drive[y][x]) DrawRectangle(gx + 2, gy + 1, 6, 4, Fade(CAR_COLS[drvCol[y][x]], gAlpha));
            if (GreenRoof(x, y)) DrawRectangle(px + 8, py + 8, 17, 15, Fade((Color){ 96, 178, 84, 255 }, gAlpha * 0.9f));
            if (smartCity) DrawSolar2D((float)(px + 9), (float)(py + 9), 7.0f, 12.0f, gAlpha);
        } break;
        case T_MALL: {
            DrawRectangle(px, py, TILE, TILE, Fade((Color){ 206, 198, 180, 255 }, gAlpha));
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade((Color){ 190, 182, 164, 255 }, gAlpha));
            if (y == 5) DrawRectangle(px + 4, py + TILE - 6, TILE - 8, 3, Fade((Color){ 70, 120, 170, 255 }, gAlpha));
            if (x == 20 && y == 4) DrawText("MALL", px + 4, py + 6, 10, Fade((Color){ 190, 40, 40, 255 }, gAlpha));
            if (GreenRoof(x, y)) DrawRectangle(px + 4, py + 4, TILE - 8, TILE - 8, Fade((Color){ 96, 178, 84, 255 }, gAlpha * 0.85f));
            if (smartCity) { DrawSolar2D((float)(px + 5), (float)(py + TILE - 17), 12.0f, 9.0f, gAlpha); DrawSolar2D((float)(px + 19), (float)(py + TILE - 17), 12.0f, 9.0f, gAlpha); }
        } break;
        case T_PARK: DrawParkedCars2D(x, y); break;
        case T_LANDFILL: {
            if (IsRecycleTile(x, y) && !gateTile[y][x]) { DrawRecycle2D(x, y, gAlpha); break; }
            unsigned char v = vr[y][x];
            for (int k = 0; k < 3; k++) {
                float cx = px + 8 + ((v >> (k * 2)) & 3) * 7, cy = py + 9 + ((v >> (k + 2)) & 3) * 7;
                DrawCircle((int)cx, (int)cy, 6.0f + bHeight[y][x] * 14.0f, Fade((Color){ 88, 74, 56, 255 }, gAlpha));
                DrawCircle((int)cx - 2, (int)cy - 2, 2.0f, Fade(((k + v) & 1) ? (Color){ 210, 210, 215, 255 } : (Color){ 70, 150, 210, 255 }, gAlpha));
            }
        } break;
        case T_RUIN:
            DrawRectangle(px + 5, py + 5, TILE - 10, TILE - 10, Fade(COL_RUIN, gAlpha));
            DrawRectangle(px + 9, py + 12, 9, 6, Fade((Color){ 95, 90, 88, 255 }, gAlpha));
            break;
        case T_FIRE_STN:
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_FIRE_STN, gAlpha));
            DrawText("FIRE", px + 6, py + TILE / 2 - 4, 10, Fade(RAYWHITE, gAlpha));
            if (smartCity) { DrawSolar2D((float)(px + 6), (float)(py + TILE - 13), 18.0f, 7.0f, gAlpha); DrawCircle(px + TILE - 9, py + 9, 5.0f, Fade((Color){ 250, 205, 40, 255 }, gAlpha)); DrawCircle(px + TILE - 9, py + 9, 3.5f, Fade((Color){ 52, 56, 64, 255 }, gAlpha)); }
            break;
        case T_POLICE_STN:
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_POLICE_STN, gAlpha));
            DrawText("POL", px + 8, py + TILE / 2 - 4, 10, Fade(RAYWHITE, gAlpha));
            if (smartCity) { DrawSolar2D((float)(px + 6), (float)(py + TILE - 13), 18.0f, 7.0f, gAlpha); DrawCircle(px + TILE - 9, py + 9, 5.0f, Fade((Color){ 250, 205, 40, 255 }, gAlpha)); DrawCircle(px + TILE - 9, py + 9, 3.5f, Fade((Color){ 52, 56, 64, 255 }, gAlpha)); }
            break;
        case T_HOSPITAL: DrawHospital2D(px, py, gAlpha); break;
        case T_POWER:
            if (smartCity) { DrawBattery2D(px, py, gAlpha); break; }
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_POWER, gAlpha));
            DrawRectangle(px + 7, py + 5, 5, 9, Fade((Color){ 30, 30, 34, 255 }, gAlpha));
            DrawRectangle(px + 20, py + 5, 5, 9, Fade((Color){ 30, 30, 34, 255 }, gAlpha));
            DrawText("PWR", px + 7, py + TILE - 15, 10, Fade(YELLOW, gAlpha));
            break;
        case T_WATER:
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_WORKS, gAlpha));
            DrawCircle(px + TILE / 2, py + 12, 7, Fade((Color){ 200, 230, 240, 255 }, gAlpha));
            DrawText("H2O", px + 7, py + TILE - 15, 10, Fade(RAYWHITE, gAlpha));
            if (smartCity) DrawSolar2D((float)(px + TILE - 17), (float)(py + 6), 12.0f, 8.0f, gAlpha);
            break;
        default: break;
    }
}

static void DrawTree2D(int x, int y) {
    int cx = x * TILE + TILE / 2, cy = y * TILE + TILE / 2;
    int style = vr[y][x] % 5;
    if (style == 1) {          // conifer
        DrawCircle(cx, cy + 1, TILE * 0.26f, Fade((Color){ 20, 85, 50, 255 }, gAlpha));
        DrawCircle(cx, cy, TILE * 0.16f, Fade((Color){ 30, 110, 60, 255 }, gAlpha));
    } else if (style == 3) {   // birch / blossom
        DrawCircle(cx, cy + 2, TILE * 0.28f, Fade((Color){ 90, 160, 80, 255 }, gAlpha));
        DrawCircle(cx - 3, cy - 2, TILE * 0.18f, Fade((Color){ 245, 180, 200, 255 }, gAlpha));
    } else {
        DrawCircle(cx, cy + 2, TILE * 0.30f, Fade((Color){ 25, 95, 40, 255 }, gAlpha));
        DrawCircle(cx - 2, cy, TILE * 0.27f, Fade(style == 2 ? (Color){ 60, 150, 55, 255 } : COL_TREE, gAlpha));
        DrawCircle(cx + 4, cy - 3, TILE * 0.12f, Fade((Color){ 80, 170, 70, 255 }, gAlpha));
    }
}

static void DrawResponder2D(const Responder *r, Color body, double t) {
    float px = r->x * TILE, py = r->y * TILE;
    DrawRectanglePro((Rectangle){ px, py, 16, 9 }, (Vector2){ 8, 4.5f }, r->angle, body);
    DrawRectanglePro((Rectangle){ px, py, 16, 2.5f }, (Vector2){ 8, 1.25f }, r->angle, WHITE);
    bool flash = ((int)(t * 6.0)) % 2 == 0;
    DrawCircle((int)px, (int)py, 2.5f, flash ? RED : SKYBLUE);
    if (r->spraying) DrawLineEx((Vector2){ px, py }, (Vector2){ r->sx * TILE, r->sy * TILE }, 2.5f, (Color){ 130, 200, 255, 210 });
}

// sleek congestion ribbons that follow the road network
static void DrawCongestion2D(void) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!IsRoad(x, y)) continue;
            float d = traffic[y][x];
            Vector2 a = { x * TILE + TILE / 2.0f, y * TILE + TILE / 2.0f };
            for (int k = 0; k < 2; k++) {
                int nx = x + (k == 0), ny = y + (k == 1);
                if (!IsRoad(nx, ny)) continue;
                float dm = 0.5f * (d + traffic[ny][nx]);
                if (dm < 0.04f) continue;
                Vector2 b = { nx * TILE + TILE / 2.0f, ny * TILE + TILE / 2.0f };
                DrawLineEx(a, b, 6.0f, CongColor(dm, 0.20f + dm * 0.35f));
            }
            if (d >= 0.04f) DrawCircleV(a, 3.0f, CongColor(d, 0.20f + d * 0.35f));
        }
}

static void DrawParticles2D(void) {
    for (int i = 0; i < MAX_PART; i++) {
        const Particle *q = &parts[i];
        if (!q->on) continue;
        float k = q->life / q->max;
        float sx = q->p.x * TILE, sy = q->p.z * TILE - q->p.y * 16.0f;
        if (q->kind == 0) DrawCircleV((Vector2){ sx, sy }, 1.5f + 3.0f * k, Fade(LerpColor((Color){ 255, 230, 90, 255 }, (Color){ 235, 70, 30, 255 }, 1.0f - k), 0.35f + 0.5f * k));
        else if (q->kind == 1) DrawCircleV((Vector2){ sx, sy }, 3.0f + 5.0f * (1.0f - k), Fade((Color){ 70, 70, 74, 255 }, 0.30f * k));
        else DrawCircleV((Vector2){ sx, sy }, 1.0f, Fade((Color){ 255, 190, 70, 255 }, k));
    }
}

static void DrawScene2D(void) {
    bool xray = layerOn[L_WATER] || layerOn[L_SEWER];
    double t = GetTime();

    if (smartCity) { DrawFarm2D(); DrawPlusFarm2D(); }
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) DrawGround2D(x, y, xray);

    if (layerOn[L_SEWER]) DrawNetwork2D(4.0f, SewerColor(), (Color){ 230, 200, 150, 255 }, 0.7f);
    if (layerOn[L_WATER]) DrawNetwork2D(-4.0f, COL_WATER_PIPE, (Color){ 220, 240, 255, 255 }, 0.9f);

    if (layerOn[L_CONGESTION] && layerOn[L_ROADS]) DrawCongestion2D();

    gAlpha = xray ? 0.45f : 1.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            if (s == T_TREE) DrawTree2D(x, y);
            else if (s != T_GRASS && s != T_ROAD && layerOn[L_BUILDINGS]) DrawStructure2D(x, y);
        }
    if (layerOn[L_BUILDINGS]) DrawFences2D();
    gAlpha = 1.0f;
    DrawPlusLow2D();

    if (layerOn[L_HEAT])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, HeatColorAt(x, y));
    if (layerOn[L_POLLUTION])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, PolColorAt(x, y));
    if (layerOn[L_NOISE])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, NoiseColorAt(x, y));

    // burning tiles glow underneath the flame particles
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (fireI[y][x] <= 0.0f) continue;
            float k = fireI[y][x] / 100.0f;
            float fl = 0.6f + 0.4f * sinf((float)t * 11.0f + x * 5.0f + y * 3.0f);
            DrawRectangle(x * TILE + 4, y * TILE + 4, TILE - 8, TILE - 8, Fade(COL_FIRE, (0.18f + 0.35f * k) * fl));
        }

    if (layerOn[L_TRAFFIC])
        for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) DrawCar2D(&cars[i]);
    DrawUCars2D();

    for (int i = 0; i < MAX_PEDS; i++) {
        if (!peds[i].active) continue;
        Vector2 p = { peds[i].x * TILE, peds[i].y * TILE };
        DrawCircleV(p, 2.4f, peds[i].shirt);
        DrawCircleV((Vector2){ p.x, p.y - 2.2f }, 1.5f, COL_CIVILIAN);
        if (peds[i].robbing) DrawCircleLines((int)p.x, (int)p.y, 7.0f + 2.0f * sinf((float)t * 10.0f), RED);
        if (peds[i].ill) {
            DrawCircleLines((int)p.x, (int)p.y, 6.0f + 2.0f * sinf((float)t * 8.0f), (Color){ 255, 70, 70, 255 });
            DrawRectangle((int)p.x - 1, (int)p.y - 9, 2, 6, WHITE); DrawRectangle((int)p.x - 3, (int)p.y - 7, 6, 2, WHITE);
        }
    }
    for (int i = 0; i < MAX_TRUCKS; i++) if (trucks[i].active) DrawResponder2D(&trucks[i], COL_TRUCK, t);
    for (int i = 0; i < MAX_POLICE; i++) if (police[i].active) DrawResponder2D(&police[i], COL_POLICE_CAR, t);
    for (int i = 0; i < MAX_AMB; i++) if (ambs[i].active) DrawAmbulance2D(&ambs[i], t);
    if (smartCity)
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) if (monitorTile[y][x]) DrawMonitor2D(x, y);
    DrawPlusHigh2D();
    DrawParticles2D();
    for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active) DrawDrone2D(&drones[i], t);

    if (hoverOK) DrawRectangleLines(hoverX * TILE, hoverY * TILE, TILE, TILE, WHITE);
}

// ---------------------------------------------------------------------------
// 3D drawing - vehicles (drawn in a local frame: +Z is forward, ground at y = 0)
// ---------------------------------------------------------------------------
static bool NightNow(void) { return st.daylight < 0.45f; }
static Color A(Color c) { return Fade(c, gAlpha); }
static const Color GLASS = { 45, 58, 76, 255 };

// smart-city models (defined in p9d_smart3d.c)
static void SolarPanel(float x, float y, float z, float w, float d, float tilt);
static void FlatSolar(float cx, float cz, float y, int nx, int nz, float gx, float gz, float pw, float pd, float tilt);
static void GableSolar(bool alongZ, float a, float y0, float h, float s0, float s1, float L);
static void HouseSolar3D(int rtype);
static void LivingWall(float y0, int floors, float w, int startF);
static void RooftopGarden3D(float h);
static void SmokeTrap3D(float h);
static void DrawStationSmart3D(bool fire, int sx, int sy);
static void DrawHospital3D(int x, int y);
static void DrawBattery3D(int x, int y);
static void DrawRecycle3D(int x, int y);
static void GreenRoof3D(float y, float cx, float cz, float w, float d);

static void Wheels(float W, float L, int pairs) {
    Color tyre = { 22, 22, 24, 255 };
    for (int i = 0; i < pairs; i++) {
        float z = (pairs == 1) ? 0.0f : (-L * 0.30f + (L * 0.60f) * ((pairs == 2) ? i : i * 0.5f));
        if (pairs == 2) z = (i == 0) ? L * 0.30f : -L * 0.30f;
        if (pairs == 3) z = (i == 0) ? L * 0.34f : (i == 1 ? -L * 0.18f : -L * 0.36f);
        BoxC(-W * 0.5f, 0.019f, z, 0.020f, 0.038f, 0.048f, tyre);
        BoxC( W * 0.5f, 0.019f, z, 0.020f, 0.038f, 0.048f, tyre);
    }
}

static bool gHideGlass = false;       // first-person: the driver looks through the glass, so it is not drawn
static void DrawVehicleLocal(int type, Color col, bool braking, bool lights) {
    float L = VLEN[type], W = VWID[type];
    Color body = A(col), glass = A(GLASS);
    switch (type) {
    case V_SALOON:
        BoxC(0, 0.046f, 0, W, 0.040f, L, body);
        if (!gHideGlass) BoxC(0, 0.088f, -0.015f, W * 0.86f, 0.040f, 0.13f, glass);
        BoxC(0, 0.110f, -0.015f, W * 0.84f, 0.006f, 0.115f, body);
        Wheels(W, L, 2); break;
    case V_HATCH:
        BoxC(0, 0.046f, 0, W, 0.040f, L, body);
        if (!gHideGlass) BoxC(0, 0.088f, -0.025f, W * 0.86f, 0.040f, 0.125f, glass);
        BoxC(0, 0.110f, -0.030f, W * 0.84f, 0.006f, 0.110f, body);
        Wheels(W, L, 2); break;
    case V_ESTATE:
        BoxC(0, 0.046f, 0, W, 0.040f, L, body);
        if (!gHideGlass) BoxC(0, 0.090f, -0.035f, W * 0.86f, 0.044f, 0.185f, glass);
        BoxC(0, 0.114f, -0.035f, W * 0.84f, 0.006f, 0.170f, body);
        Wheels(W, L, 2); break;
    case V_PICKUP:
        BoxC(0, 0.048f, 0.060f, W, 0.044f, 0.120f, body);
        if (!gHideGlass) BoxC(0, 0.090f, 0.045f, W * 0.88f, 0.040f, 0.085f, glass);
        BoxC(0, 0.112f, 0.045f, W * 0.86f, 0.006f, 0.075f, body);
        BoxC(0, 0.036f, -0.080f, W, 0.016f, 0.130f, Shade(body, 0.55f));
        BoxC(-(W * 0.5f - 0.005f), 0.058f, -0.080f, 0.010f, 0.030f, 0.130f, body);
        BoxC( (W * 0.5f - 0.005f), 0.058f, -0.080f, 0.010f, 0.030f, 0.130f, body);
        BoxC(0, 0.058f, -0.142f, W, 0.030f, 0.010f, body);
        Wheels(W, L, 2); break;
    case V_SUV:
        BoxC(0, 0.056f, 0, W, 0.056f, L, body);
        if (!gHideGlass) BoxC(0, 0.106f, -0.015f, W * 0.90f, 0.040f, 0.170f, glass);
        BoxC(0, 0.128f, -0.015f, W * 0.88f, 0.006f, 0.160f, body);
        BoxC(-W * 0.32f, 0.134f, -0.015f, 0.006f, 0.006f, 0.15f, A((Color){ 60, 62, 68, 255 }));
        Wheels(W, L, 2); break;
    case V_VAN:
        BoxT(TX_VAN, 0, 0.078f, -0.025f, W, 0.112f, 0.25f, A(WHITE));
        BoxC(0, 0.056f, 0.130f, W, 0.068f, 0.045f, A((Color){ 235, 235, 235, 255 }));
        if (!gHideGlass) BoxC(0, 0.094f, 0.108f, W * 0.9f, 0.036f, 0.012f, glass);
        Wheels(W, L, 2); break;
    case V_TRUCK:
        BoxC(0, 0.035f, -0.03f, W * 0.8f, 0.030f, 0.40f, A((Color){ 40, 40, 44, 255 }));
        BoxC(0, 0.070f, 0.170f, W, 0.090f, 0.100f, A((Color){ 235, 235, 235, 255 }));
        if (!gHideGlass) BoxC(0, 0.086f, 0.222f, W * 0.9f, 0.036f, 0.010f, glass);
        BoxT(TX_VAN, 0, 0.100f, -0.060f, W * 1.05f, 0.130f, 0.30f, A(WHITE));
        Wheels(W, L, 3); break;
    default: break;
    }
    // lights
    Color tail = braking ? (Color){ 255, 40, 30, 255 } : (Color){ 120, 20, 20, 255 };
    Color head = lights ? (Color){ 255, 248, 200, 255 } : (Color){ 215, 215, 205, 255 };
    float hz = L * 0.5f + 0.001f;
    float ty = (type == V_TRUCK) ? 0.05f : 0.05f;
    BoxC(-W * 0.34f, ty, -hz, 0.022f, 0.012f, 0.004f, tail);
    BoxC( W * 0.34f, ty, -hz, 0.022f, 0.012f, 0.004f, tail);
    BoxC(-W * 0.32f, ty, hz, 0.022f, 0.012f, 0.004f, head);
    BoxC( W * 0.32f, ty, hz, 0.022f, 0.012f, 0.004f, head);
}

// cyclist: modelled in metres (1 m = 1/17 tile, enlarged 10% so riders stay readable)
#define M_SC (1.1f / 17.0f)
static Vector3 MV(float x, float y, float z) { return V3(x * M_SC, y * M_SC, z * M_SC); }
static void MBox(float x, float y, float z, float w, float h, float l, Color c) { BoxC(x * M_SC, y * M_SC, z * M_SC, w * M_SC, h * M_SC, l * M_SC, c); }
static void MTube(Vector3 a, Vector3 b, float r, Color c) { GCylinderEx(MV(a.x, a.y, a.z), MV(b.x, b.y, b.z), r * M_SC, r * M_SC, 6, c); }
static void DrawBike3D(const Car *c) {
    PushFrame(c->x, ENT_Y, c->y, c->yaw);
    float t = (float)GetTime();
    Color frame = A(c->col), tyre = A((Color){ 26, 26, 30, 255 }), rim = A((Color){ 188, 190, 196, 255 });
    Color skin = A((Color){ 235, 190, 160, 255 }), shirt = A(SHIRT_COLS[c->id % 8]), pants = A((Color){ 46, 52, 72, 255 });
    Color helmet = A((c->id & 1) ? (Color){ 230, 60, 60, 255 } : (Color){ 60, 160, 230, 255 });
    Color dark = A((Color){ 32, 32, 36, 255 });
    // wheels: 0.70 m, axle along X
    for (int s = 0; s < 2; s++) {
        float wz = s ? 0.525f : -0.525f;
        GCylinderEx(MV(-0.02f, 0.35f, wz), MV(0.02f, 0.35f, wz), 0.35f * M_SC, 0.35f * M_SC, 14, tyre);
        GCylinderEx(MV(-0.024f, 0.35f, wz), MV(0.024f, 0.35f, wz), 0.29f * M_SC, 0.29f * M_SC, 12, rim);
        GCylinderEx(MV(-0.03f, 0.35f, wz), MV(0.03f, 0.35f, wz), 0.035f * M_SC, 0.035f * M_SC, 6, frame);
    }
    // frame
    Vector3 bb = { 0, 0.30f, -0.05f }, seat = { 0, 0.92f, -0.18f }, head = { 0, 0.90f, 0.40f }, rearAx = { 0, 0.35f, -0.525f };
    MTube(bb, head, 0.022f, frame);                      // down tube
    MTube(seat, (Vector3){ 0, 0.96f, 0.38f }, 0.018f, frame);   // top tube
    MTube(bb, seat, 0.018f, frame);                      // seat tube
    MTube(bb, rearAx, 0.012f, frame);                    // chain stays
    MTube(seat, rearAx, 0.010f, frame);                  // seat stays
    MTube(head, (Vector3){ -0.04f, 0.35f, 0.525f }, 0.012f, frame);   // forks
    MTube(head, (Vector3){  0.04f, 0.35f, 0.525f }, 0.012f, frame);
    MTube((Vector3){ 0, 0.96f, 0.40f }, (Vector3){ 0, 1.03f, 0.40f }, 0.020f, dark);   // stem
    MTube((Vector3){ -0.20f, 1.04f, 0.42f }, (Vector3){ 0.20f, 1.04f, 0.42f }, 0.011f, dark);   // handlebar
    MBox(0, 0.97f, -0.18f, 0.12f, 0.035f, 0.26f, dark);  // saddle
    // chainring + pedals
    GCylinderEx(MV(-0.035f, 0.30f, -0.05f), MV(0.035f, 0.30f, -0.05f), 0.10f * M_SC, 0.10f * M_SC, 10, dark);
    float crank = t * 7.0f + (float)c->id;
    for (int s = 0; s < 2; s++) {
        float a = crank + s * PI;
        float py = 0.30f + 0.17f * cosf(a), pz = -0.05f + 0.17f * sinf(a);
        float px = s ? 0.10f : -0.10f;
        MTube((Vector3){ 0, 0.30f, -0.05f }, (Vector3){ px * 0.6f, py, pz }, 0.008f, dark);   // crank arm
        MBox(px, py, pz, 0.10f, 0.02f, 0.06f, dark);                                          // pedal
    }
    // rider: seated, leaning forward, hands on the bars
    Vector3 hipC = { 0, 1.00f, -0.18f }, shC = { 0, 1.42f, 0.14f };
    GCylinderEx(MV(hipC.x, hipC.y, hipC.z), MV(shC.x, shC.y, shC.z), 0.13f * M_SC, 0.12f * M_SC, 8, shirt);   // torso
    MBox(0, 0.98f, -0.18f, 0.30f, 0.10f, 0.20f, pants);                                                     // hips
    for (int s = -1; s <= 1; s += 2) {
        // arm: shoulder to grip
        MTube((Vector3){ s * 0.17f, 1.38f, 0.14f }, (Vector3){ s * 0.20f, 1.04f, 0.42f }, 0.045f, shirt);
        // leg: hip -> knee (forward) -> pedal, pedalling in step with the crank
        float a = crank + ((s > 0) ? PI : 0.0f);
        Vector3 pedal = { s * 0.10f, 0.30f + 0.17f * cosf(a), -0.05f + 0.17f * sinf(a) };
        Vector3 hip = { s * 0.09f, 1.00f, -0.18f };
        Vector3 knee = { s * 0.09f, 0.5f * (hip.y + pedal.y) + 0.02f, 0.5f * (hip.z + pedal.z) + 0.13f };
        MTube(hip, knee, 0.068f, pants);
        MTube(knee, pedal, 0.052f, pants);
        MBox(pedal.x, pedal.y - 0.02f, pedal.z + 0.02f, 0.09f, 0.05f, 0.14f, A((Color){ 30, 30, 34, 255 }));    // shoe
    }
    GSphere(MV(0, 1.58f, 0.13f), 0.11f * M_SC, skin);                                   // head
    GSphere(MV(0, 1.63f, 0.14f), 0.125f * M_SC, helmet);                                 // helmet (top and back)
    PopFrame();
}
static void DrawCar3D(const Car *c) {
    if (c->type == V_BIKE) { DrawBike3D(c); return; }
    PushFrame(c->x, ENT_Y, c->y, c->yaw);
    DrawVehicleLocal(c->type, c->col, c->braking, NightNow());
    if (c->shuttle) {                                                   // autonomous shuttle: teal livery, roof lidar, passenger glow
        BoxC(0, 0.078f, -0.025f, VWID[V_VAN] + 0.004f, 0.028f, 0.252f, A((Color){ 0, 190, 165, 255 }));
        BoxC(0, 0.139f, -0.025f, 0.080f, 0.010f, 0.180f, A((Color){ 235, 245, 245, 255 }));
        GCylinder((Vector3){ 0, 0.144f, 0.040f }, 0.020f, 0.020f, 0.022f, 12, A((Color){ 30, 36, 44, 255 }));
        GSphere((Vector3){ 0, 0.172f, 0.040f }, 0.010f, (Color){ 120, 255, 225, 255 });
        for (int k = 0; k < c->shPax && k < 8; k++) GSphere((Vector3){ ((k % 2) ? 0.026f : -0.026f), 0.108f, -0.11f + (k / 2) * 0.05f }, 0.011f, A(SHIRT_COLS[(c->id + k) % 8]));
    }
    if (c->wanted) {
        float f = 0.5f + 0.5f * sinf((float)GetTime() * 10.0f);
        GSphere((Vector3){ 0, 0.18f, 0 }, 0.025f + 0.01f * f, c->fleeing ? RED : ORANGE);
    }
    PopFrame();
}

// ---------------------------------------------------------------------------
// 3D drawing - foliage
// ---------------------------------------------------------------------------
static void DrawTree3D(int x, int y) {
    unsigned char v = vr[y][x];
    int style = v % 5;
    float sc = 0.85f + ((v >> 3) & 3) * 0.1f;
    Color g1 = Shade((Color){ 40, 130, 50, 255 }, 0.85f + ((v >> 5) & 3) * 0.1f);
    Color g2 = Shade(g1, 1.18f), g3 = Shade(g1, 0.72f);
    PushFrame(x + 0.5f, 0, y + 0.5f, (float)(v * 7 % 360));
    GXScale(sc, sc, sc);
    switch (style) {
    case 0:       // oak
        BoxT(TX_BARK, 0, 0.14f, 0, 0.07f, 0.28f, 0.07f, A(WHITE));
        GSphere((Vector3){ 0, 0.40f, 0 }, 0.20f, A(g1));
        GSphere((Vector3){ 0.11f, 0.33f, 0.07f }, 0.14f, A(g3));
        GSphere((Vector3){ -0.10f, 0.35f, -0.08f }, 0.15f, A(g2));
        GSphere((Vector3){ 0, 0.54f, 0.02f }, 0.13f, A(g2));
        break;
    case 1:       // pine
        BoxT(TX_BARK, 0, 0.06f, 0, 0.05f, 0.12f, 0.05f, A(WHITE));
        GCylinder((Vector3){ 0, 0.08f, 0 }, 0.0f, 0.19f, 0.20f, 9, A(Shade(g3, 0.8f)));
        GCylinder((Vector3){ 0, 0.20f, 0 }, 0.0f, 0.15f, 0.20f, 9, A(Shade(g3, 0.95f)));
        GCylinder((Vector3){ 0, 0.32f, 0 }, 0.0f, 0.11f, 0.20f, 9, A(g3));
        GCylinder((Vector3){ 0, 0.43f, 0 }, 0.0f, 0.065f, 0.17f, 9, A(Shade(g3, 1.1f)));
        break;
    case 2:       // tall poplar
        BoxT(TX_BARK, 0, 0.07f, 0, 0.05f, 0.14f, 0.05f, A(WHITE));
        GXPush(); GXTrans(0, 0.40f, 0); GXScale(0.55f, 1.9f, 0.55f);
        GSphere((Vector3){ 0, 0, 0 }, 0.14f, A(g2));
        GXPop();
        break;
    case 3:       // birch
        BoxC(0, 0.15f, 0, 0.045f, 0.30f, 0.045f, A((Color){ 228, 226, 214, 255 }));
        BoxC(0, 0.12f, 0.0235f, 0.02f, 0.01f, 0.004f, A((Color){ 40, 40, 40, 255 }));
        GSphere((Vector3){ 0, 0.38f, 0 }, 0.15f, A(Shade(g2, 1.1f)));
        GSphere((Vector3){ 0.09f, 0.31f, 0.04f }, 0.10f, A(g2));
        GSphere((Vector3){ -0.08f, 0.45f, -0.03f }, 0.10f, A(Shade(g1, 1.15f)));
        break;
    default:      // blossom tree
        BoxT(TX_BARK, 0, 0.12f, 0, 0.06f, 0.24f, 0.06f, A(WHITE));
        GSphere((Vector3){ 0, 0.34f, 0 }, 0.18f, A((Color){ 245, 175, 195, 255 }));
        GSphere((Vector3){ 0.10f, 0.30f, 0.05f }, 0.12f, A((Color){ 250, 200, 215, 255 }));
        GSphere((Vector3){ -0.09f, 0.40f, -0.04f }, 0.11f, A((Color){ 235, 150, 180, 255 }));
        break;
    }
    PopFrame();
}

static void DrawDeco3D(int x, int y) {
    unsigned char v = vr[y][x];
    float cx = x + 0.5f, cz = y + 0.5f;
    int d = deco[y][x];
    Color sh1 = A((Color){ 48, 125, 55, 255 }), sh2 = A((Color){ 62, 145, 62, 255 });
    if (d == 1) {          // shrubbery
        for (int k = 0; k < 4; k++) {
            float ox = ((v >> k) & 3) * 0.2f - 0.3f, oz = ((v >> (k + 2)) & 3) * 0.2f - 0.3f;
            GSphere((Vector3){ cx + ox, 0.05f, cz + oz }, 0.06f + 0.01f * k, (k & 1) ? sh1 : sh2);
        }
    } else if (d == 2) {   // hedge row
        bool alongX = (v & 1);
        BoxT(TX_HEDGE, cx, 0.06f, cz, alongX ? 0.84f : 0.16f, 0.12f, alongX ? 0.16f : 0.84f, A(WHITE));
        BoxC(cx, 0.122f, cz, alongX ? 0.80f : 0.12f, 0.012f, alongX ? 0.12f : 0.80f, sh2);
    } else if (d == 3) {   // flower bed
        BoxC(cx, 0.012f, cz, 0.55f, 0.024f, 0.38f, A((Color){ 90, 62, 44, 255 }));
        for (int k = 0; k < 9; k++) {
            float ox = (k % 3 - 1) * 0.17f, oz = (k / 3 - 1) * 0.12f;
            Color fc = (k + v) % 3 == 0 ? (Color){ 235, 70, 90, 255 } : ((k + v) % 3 == 1 ? (Color){ 250, 215, 70, 255 } : (Color){ 190, 110, 230, 255 });
            GSphere((Vector3){ cx + ox, 0.05f, cz + oz }, 0.022f, A(fc));
            BoxC(cx + ox, 0.03f, cz + oz, 0.004f, 0.03f, 0.004f, sh1);
        }
    }
}

static void DrawGrassTufts(int x, int y) {
    unsigned char v = vr[y][x];
    for (int k = 0; k < 5; k++) {
        float ox = Hash01(x, y, k) * 0.84f + 0.08f, oz = Hash01(y, x, k + 9) * 0.84f + 0.08f;
        Color g = Shade((Color){ 64, 150, 58, 255 }, 0.8f + Hash01(k, x + y, 3) * 0.5f);
        GCylinder((Vector3){ x + ox, 0.03f, y + oz }, 0.0f, 0.016f, 0.05f + 0.03f * ((v >> k) & 1), 4, A(g));
    }
}

// ---------------------------------------------------------------------------
// 3D drawing - houses in the gated communities
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 3D drawing - houses in the gated communities
// house footprint (local frame, front = +Z): one set of numbers drives walls, roof, garage and collisions
// ---------------------------------------------------------------------------
static void DrawHouse3D(int x, int y) {
    static const Color tints[5] = { {255,255,255,255}, {255,236,214,255}, {214,236,255,255}, {236,255,222,255}, {255,224,224,255} };
    static const Color roofs[5] = { {205,105,80,255}, {120,126,142,255}, {150,112,82,255}, {95,120,100,255}, {170,90,90,255} };
    unsigned char v = vr[y][x];
    int f = faceDir[y][x];
    bool dmg = health[y][x] < 50.0f;
    int wallTex = (v % 3 == 0) ? TX_BRICK : (v % 3 == 1 ? TX_SIDING : TX_STUCCO);
    Color wall = A(dmg ? Shade(tints[(v >> 2) % 5], 0.45f) : tints[(v >> 2) % 5]);
    Color roof = A(dmg ? Shade(roofs[(v >> 4) % 5], 0.5f) : roofs[(v >> 4) % 5]);
    int rtype = (v >> 1) % 4;
    bool lit = NightNow() && !dmg && (v & 4);
    PushFrame(x + 0.5f, 0, y + 0.5f, FaceYaw(f));
    FitApplyAt(HouseFit(x, y), 0.0f, 0.0f);

    // main block and roof (the roof is built on the same footprint as the walls, so it sits flush)
    BoxT(wallTex, HS_CX, HS_WH * 0.5f, HS_CZ, HS_W, HS_WH, HS_L, wall);
    GXPush(); GXTrans(HS_CX, 0.0f, HS_CZ);
    if (rtype == 0)      RoofGable(HS_W, HS_L, HS_WH, HS_RH, false, roof, wallTex, wall);
    else if (rtype == 1) RoofGable(HS_W, HS_L, HS_WH, HS_RH, true, roof, wallTex, wall);
    else if (rtype == 2) RoofHip(HS_W, HS_L, HS_WH, HS_RH, roof);
    else                 BoxC(0, HS_WH + 0.012f, 0, HS_W, 0.024f, HS_L, ShadeF(roof, 0.8f));
    if (smartCity && !dmg) HouseSolar3D(rtype);
    if (GreenRoof(x, y) && rtype == 3) GreenRoof3D(HS_WH + 0.026f, 0, 0, HS_W, HS_L);
    GXPop();
    if (v & 1) BoxT(TX_BRICK, HS_CX - 0.10f, HS_WH + 0.07f, HS_CZ - 0.09f, 0.05f, 0.14f, 0.05f, A(WHITE));   // chimney

    // garage: flat roof flush with its walls
    BoxT(TX_STUCCO, HS_GX, HS_GH * 0.5f, HS_GZ, HS_GW, HS_GH, HS_GL, wall);
    BoxC(HS_GX, HS_GH + 0.012f, HS_GZ, HS_GW, 0.024f, HS_GL, A(ShadeF(roofs[(v >> 4) % 5], 0.85f)));
    float gzf = HS_GZ + HS_GL * 0.5f + 0.0025f;
    BoxC(HS_GX, 0.055f, gzf, HS_GW - 0.03f, 0.09f, 0.006f, A((Color){ 225, 225, 228, 255 }));    // garage door
    for (int k = 0; k < 3; k++) BoxC(HS_GX, 0.025f + k * 0.03f, gzf + 0.001f, HS_GW - 0.03f, 0.003f, 0.002f, A((Color){ 170, 170, 175, 255 }));

    // front door + windows (lit at night)
    float zf = HS_CZ + HS_L * 0.5f + 0.0025f;
    BoxC(HS_CX - 0.08f, 0.06f, zf, 0.05f, 0.11f, 0.006f, A((Color){ 90 + (v & 63), 60, 45, 255 }));
    Color win = lit ? (Color){ 255, 226, 130, 255 } : (Color){ 70, 100, 130, 255 };
    BoxC(HS_CX + 0.07f, 0.13f, zf, 0.09f, 0.07f, 0.006f, A(win));
    BoxC(HS_CX + 0.07f, 0.13f, zf + 0.002f, 0.004f, 0.07f, 0.002f, A(WHITE));
    BoxC(HS_CX - 0.19f, 0.13f, zf, 0.06f, 0.06f, 0.006f, A(win));
    BoxC(HS_CX - HS_W * 0.5f - 0.0025f, 0.13f, HS_CZ - 0.04f, 0.006f, 0.06f, 0.08f, A(win));    // side window

    // driveway, garden path, garden (the parked car is drawn live, see DrawHouseDrive3D). Everything stays
    // inside FENCE_LINE, so a house on the community edge never reaches its fence.
    BoxC(HS_GX, WALK_TOP * 0.5f, 0.28f, 0.16f, WALK_TOP, 0.32f, A((Color){ 165, 165, 168, 255 }));       // drive: garage front to fence
    float pz0 = HS_CZ + HS_L * 0.5f;                                                          // front wall
    BoxC(HS_CX - 0.08f, WALK_TOP * 0.5f, pz0 + 0.10f, 0.07f, WALK_TOP, 0.20f, A((Color){ 205, 195, 175, 255 }));   // path: door to the gateway
    BoxC(-0.04f, WALK_TOP * 0.5f, 0.30f, 0.28f, WALK_TOP, 0.07f, A((Color){ 205, 195, 175, 255 }));
    GSphere(V3(-0.36f, 0.04f, 0.22f), 0.05f, A((Color){ 48, 125, 55, 255 }));
    GSphere(V3(-0.30f, 0.035f, 0.36f), 0.04f, A((Color){ 62, 145, 62, 255 }));
    for (int k = 0; k < 4; k++)
        GSphere(V3(-0.40f + k * 0.042f, 0.022f, 0.15f), 0.014f, A(k & 1 ? (Color){ 240, 90, 120, 255 } : (Color){ 250, 215, 70, 255 }));
    if (v & 2) {          // back garden tree
        BoxT(TX_BARK, 0.06f, 0.07f, -0.35f, 0.04f, 0.14f, 0.04f, A(WHITE));
        GSphere(V3(0.06f, 0.20f, -0.35f), 0.07f, A((Color){ 55, 140, 55, 255 }));
    } else if (v & 32) {  // swimming pool
        BoxC(-0.20f, 0.004f, -0.35f, 0.16f, 0.008f, 0.07f, A((Color){ 225, 225, 225, 255 }));
        BoxC(-0.20f, 0.009f, -0.35f, 0.14f, 0.006f, 0.05f, A((Color){ 70, 165, 220, 255 }));
    }
    if (v & 16) BoxT(TX_CORRUG, 0.30f, 0.04f, -0.34f, 0.10f, 0.08f, 0.09f, A(WHITE));       // shed
    BoxT(TX_HEDGE, 0, 0.035f, -0.41f, 0.80f, 0.07f, 0.05f, A(WHITE));                       // back hedge
    PopFrame();
}

// the car parked on the driveway (live: it changes when cars come and go)
static void DrawHouseDrive3D(int x, int y) {
    if (!drive[y][x] || surface[y][x] != T_HOUSE) return;
    PushFrame(x + 0.5f, 0, y + 0.5f, FaceYaw(faceDir[y][x]));
    PushFrame(HS_GX, WALK_TOP, HS_DRV_Z, 0.0f);
    DrawVehicleLocal(drive[y][x] - 1, CAR_COLS[drvCol[y][x]], false, NightNow());
    PopFrame();
    PopFrame();
}
// ---------------------------------------------------------------------------
// 3D drawing - other buildings
// ---------------------------------------------------------------------------

static int BldFloors(float h) { return (int)(h * 10.0f * 1.15f + 0.5f); }   // a little taller than the simulation's storey count
static void DrawBuilding3D(int x, int y) {
    unsigned char v = vr[y][x];
    float h = bHeight[y][x], cx = x + 0.5f, cz = y + 0.5f;
    int fl = BldFloors(h);
    if (fl < 1) fl = 1;
    bool dmg = health[y][x] < 50.0f, night = NightNow();
    float H = fl * FLOOR_H;                                       // building height (tile units)
    Color tint = A(dmg ? (Color){ 110, 90, 85, 255 } : (Color){ (unsigned char)(235 + (v & 15)), (unsigned char)(235 + ((v >> 2) & 15)), 255, 255 });
    int side = RoadSide(x, y);
    int wt = (bType[y][x] == B_COM) ? (fl >= 8 ? (night ? TX_WIN_GLASS_N : TX_WIN_GLASS) : (night ? TX_WIN_OFFICE_N : TX_WIN_OFFICE))
                                    : (night ? TX_WIN_BRICK_N : TX_WIN_BRICK);
    PushFrame(cx, 0, cz, FaceYaw(side));
    FitApplyAt(FitIn(x, y, side, -0.475f, 0.475f, -0.475f, 0.475f), 0.0f, 0.0f);
    if (fl >= 10) {                                   // skyrise: podium, tower, crown (the beacon is live)
        int f1 = fl * 35 / 100; if (f1 < 2) f1 = 2;
        int f2 = fl - f1;
        float h1 = f1 * FLOOR_H, h2 = f2 * FLOOR_H;
        WindowBox(wt, f1, 0, h1 / 2, 0, BLD_PODIUM, h1, BLD_PODIUM, tint);
        WindowBox(wt, f2, 0, h1 + h2 / 2, 0, BLD_TOWER, h2, BLD_TOWER, tint);
        BoxC(0, h1 + 0.005f, 0, BLD_PODIUM + 0.02f, 0.012f, BLD_PODIUM + 0.02f, A((Color){ 120, 122, 128, 255 }));        // podium ledge
        BoxC(0, H + 0.02f, 0, BLD_TOWER + 0.04f, 0.04f, BLD_TOWER + 0.04f, A((Color){ 105, 108, 115, 255 }));           // crown
        BoxC(0.14f, H + 0.07f, 0.12f, 0.16f, 0.06f, 0.14f, A((Color){ 150, 152, 158, 255 }));   // plant room
        BoxC(-0.12f, H + 0.18f, -0.12f, 0.012f, 0.28f, 0.012f, A((Color){ 200, 200, 205, 255 })); // mast
        BoxC(0, 0.04f, BLD_PODIUM * 0.5f + 0.03f, 0.32f, 0.08f, 0.02f, A((Color){ 40, 50, 60, 255 }));              // lobby doors
        if (smartCity && !dmg) {                                                                  // smart city: green facades + roof solar
            LivingWall(0.0f, f1, BLD_PODIUM, 3);
            LivingWall(h1, f2, BLD_TOWER, 1);
            RooftopGarden3D(H);
            SolarPanel(-0.20f, H + 0.065f, 0.18f, 0.15f, 0.11f, 12.0f);
            SolarPanel( 0.20f, H + 0.065f, 0.20f, 0.15f, 0.11f, 12.0f);
            SolarPanel( 0.00f, H + 0.065f, -0.05f, 0.15f, 0.11f, 12.0f);
        }
    } else {
        WindowBox(wt, fl, 0, H / 2, 0, BLD_W, H, BLD_W, tint);
        if (bType[y][x] == B_RES && fl <= 4 && (v & 1)) RoofGable(BLD_W + 0.02f, BLD_W + 0.02f, H, 0.14f, (v & 2) != 0, A((Color){ 150, 100, 90, 255 }), TX_BRICK, A(WHITE));
        else BoxC(0, H + 0.01f, 0, BLD_W + 0.02f, 0.03f, BLD_W + 0.02f, A((Color){ 112, 114, 120, 255 }));
        BoxC(-0.16f, H + 0.05f, 0.12f, 0.13f, 0.05f, 0.14f, A((Color){ 150, 152, 158, 255 }));
        if (GreenRoof(x, y)) GreenRoof3D(H + 0.032f, 0.0f, 0.0f, BLD_W, BLD_W);
        if (smartCity && !dmg) {
            if (bType[y][x] == B_RES && fl <= 4 && (v & 1)) GableSolar((v & 2) != 0, 0.45f, H, 0.14f, 0.08f, 0.38f, 0.32f);
            else FlatSolar(0.0f, -0.14f, H + 0.045f, 3, 2, 0.24f, 0.20f, 0.20f, 0.14f, 12.0f);
        }
        if (bType[y][x] == B_COM && fl < 8) {          // shop awning
            Color aw = (v & 1) ? (Color){ 200, 60, 60, 255 } : (Color){ 60, 120, 190, 255 };
            BoxC(0, 0.11f, BLD_W * 0.5f + 0.03f, 0.70f, 0.02f, 0.10f, A(aw));
        }
        BoxC(0, 0.04f, BLD_W * 0.5f + 0.005f, 0.20f, 0.08f, 0.012f, A((Color){ 45, 50, 60, 255 }));
    }
    PopFrame();
}

// skyrise beacon (blinks at night)
static void DrawBuildingLive3D(int x, int y) {
    int fl = BldFloors(bHeight[y][x]);
    if (fl < 10) return;
    float H = fl * FLOOR_H;
    PushFrame(x + 0.5f, 0, y + 0.5f, FaceYaw(RoadSide(x, y)));
    FitApplyAt(FitIn(x, y, RoadSide(x, y), -0.475f, 0.475f, -0.475f, 0.475f), 0.0f, 0.0f);
    if (NightNow() || (int)(GetTime() * 1.2f) % 2) GSphere(V3(-0.12f, H + 0.33f, -0.12f), 0.016f, RED);
    PopFrame();
}
static void DrawIndustrial3D(int x, int y) {
    unsigned char v = vr[y][x];
    float h = bHeight[y][x], cx = x + 0.5f, cz = y + 0.5f;
    bool dmg = health[y][x] < 50.0f;
    Color tint = A(dmg ? (Color){ 120, 100, 95, 255 } : WHITE);
    PushFrame(cx, 0, cz, FaceYaw(RoadSide(x, y)));
    FitApplyAt(FitIn(x, y, RoadSide(x, y), -0.475f, 0.475f, -0.475f, 0.475f), 0.0f, 0.0f);
    if (deco[y][x] == 0) {              // factory with chimney and vents
        BoxT(TX_CORRUG, 0, h / 2, 0, 0.84f, h, 0.72f, tint);
        for (int k = 0; k < 3; k++) { GXPush(); GXTrans(-0.28f + k * 0.28f, 0, 0); RoofGable(0.28f, 0.72f, h, 0.07f, true, A((Color){ 140, 145, 150, 255 }), TX_CORRUG, tint); GXPop(); }
        BoxC(-0.28f, h + 0.04f, 0.0f, 0.12f, 0.08f, 0.12f, A((Color){ 100, 104, 110, 255 }));
        GCylinder((Vector3){ 0.24f, h, -0.20f }, 0.065f, 0.08f, 0.22f, 12, A((Color){ 150, 70, 55, 255 }));
        GCylinder((Vector3){ 0.24f, h + 0.22f, -0.20f }, 0.055f, 0.065f, 0.16f, 12, A((Color){ 235, 235, 235, 255 }));
        GCylinder((Vector3){ 0.24f, h + 0.38f, -0.20f }, 0.050f, 0.055f, 0.06f, 12, A((Color){ 200, 50, 45, 255 }));
        BoxC(0.0f, 0.06f, 0.365f, 0.30f, 0.12f, 0.012f, A((Color){ 70, 78, 88, 255 }));
        BoxC(0.0f, 0.12f, 0.372f, 0.34f, 0.01f, 0.012f, A((Color){ 240, 190, 40, 255 }));
        if (smartCity && !dmg) {
            SmokeTrap3D(h);
            SolarPanel(-0.30f, h + 0.035f, 0.22f, 0.20f, 0.18f, 12.0f);
            SolarPanel(-0.30f, h + 0.035f, -0.22f, 0.20f, 0.18f, 12.0f);
            SolarPanel( 0.30f, h + 0.035f, 0.26f, 0.20f, 0.18f, 12.0f);
        }
    } else if (deco[y][x] == 1) {       // warehouse with roller doors
        BoxT(TX_CORRUG, 0, h / 2, 0, 0.90f, h, 0.64f, tint);
        RoofGable(0.94f, 0.68f, h, 0.07f, false, A((Color){ 120, 128, 138, 255 }), TX_CORRUG, tint);
        for (int k = 0; k < 3; k++) {
            BoxC(-0.28f + k * 0.28f, 0.09f, 0.322f, 0.20f, 0.18f, 0.008f, A((Color){ 108, 118, 130, 255 }));
            for (int j = 0; j < 4; j++) BoxC(-0.28f + k * 0.28f, 0.03f + j * 0.04f, 0.327f, 0.20f, 0.004f, 0.002f, A((Color){ 85, 92, 102, 255 }));
        }
        if (smartCity && !dmg) GableSolar(false, 0.47f, h, 0.07f, 0.08f, 0.40f, 0.30f);
    } else {                            // tank farm
        GCylinder((Vector3){ -0.22f, 0, -0.05f }, 0.19f, 0.19f, 0.56f, 20, A((Color){ 205, 208, 212, 255 }));
        GCylinder((Vector3){ -0.22f, 0.56f, -0.05f }, 0.0f, 0.19f, 0.05f, 20, A((Color){ 170, 174, 180, 255 }));
        GCylinder((Vector3){ 0.23f, 0, 0.12f }, 0.15f, 0.15f, 0.44f, 18, A((Color){ 225, 226, 230, 255 }));
        GCylinder((Vector3){ 0.23f, 0.44f, 0.12f }, 0.0f, 0.15f, 0.04f, 18, A((Color){ 185, 188, 194, 255 }));
        BoxC(0.0f, 0.10f, 0.1f, 0.5f, 0.02f, 0.02f, A((Color){ 160, 80, 60, 255 }));
        BoxC(-0.22f, 0.30f, 0.14f, 0.02f, 0.5f, 0.02f, A((Color){ 120, 124, 130, 255 }));
        if (v & 1) GCylinder((Vector3){ 0.02f, 0, -0.36f }, 0.07f, 0.07f, 0.22f, 12, A((Color){ 230, 200, 60, 255 }));
    }
    PopFrame();
}

static void DrawYard3D(int x, int y) {          // loading yard: stacked containers
    unsigned char v = vr[y][x];
    if (gateTile[y][x] && zone[y][x] == Z_INDUSTRIAL) return;
    static const Color cc[5] = { {200,70,55,255}, {60,110,170,255}, {80,150,90,255}, {220,170,50,255}, {150,150,155,255} };
    int n = 2 + (v & 1);
    for (int k = 0; k < n; k++) {
        float ox = 0.22f + (k % 2) * 0.46f - 0.5f + 0.28f, oz = -0.28f + (k / 2) * 0.5f;
        BoxT(TX_CORRUG, x + 0.5f + ox - 0.28f, 0.05f, y + 0.5f + oz, 0.34f, 0.10f, 0.14f, A(cc[(v + k) % 5]));
        if ((v >> (k + 1)) & 1) BoxT(TX_CORRUG, x + 0.5f + ox - 0.28f, 0.15f, y + 0.5f + oz, 0.34f, 0.10f, 0.14f, A(cc[(v + k + 2) % 5]));
    }
}

static void DrawMall3D(int x, int y) {
    bool n = (y > 0) && surface[y - 1][x] == T_MALL, s = (y + 1 < MAP_H) && surface[y + 1][x] == T_MALL;
    float x0, x1, z0, z1;
    MallRect(x, y, &x0, &x1, &z0, &z1);
    float hh = bHeight[y][x] + 0.04f;
    float cx = (x0 + x1) / 2, cz = (z0 + z1) / 2;
    BoxT(TX_CONCRETE, cx, hh / 2, cz, x1 - x0, hh, z1 - z0, A((Color){ 235, 225, 205, 255 }));
    BoxC(cx, hh + 0.012f, cz, x1 - x0 + 0.02f, 0.024f, z1 - z0 + 0.02f, A((Color){ 120, 118, 122, 255 }));
    if (!s) {             // glazed shop front facing the car park
        BoxC(cx, 0.17f, z1 + 0.004f, x1 - x0 - 0.1f, 0.22f, 0.008f, A((Color){ 70, 115, 155, 255 }));
        for (int k = 0; k < 4; k++) BoxC(x0 + (k + 0.5f) * (x1 - x0) / 4, 0.17f, z1 + 0.008f, 0.01f, 0.22f, 0.004f, A((Color){ 230, 230, 235, 255 }));
        BoxC(cx, 0.30f, z1 + 0.10f, x1 - x0 - 0.1f, 0.016f, 0.20f, A((Color){ 200, 60, 55, 255 }));        // canopy
        if (x == 21) BoxC(cx, 0.36f, z1 + 0.012f, 0.9f, 0.10f, 0.02f, A((Color){ 235, 190, 50, 255 }));   // sign board
    }
    if (!n && y > 0) {}
    BoxC(cx - 0.2f, hh + 0.07f, cz + 0.1f, 0.18f, 0.08f, 0.16f, A((Color){ 160, 162, 168, 255 }));       // roof plant
    BoxC(cx + 0.22f, hh + 0.05f, cz - 0.15f, 0.14f, 0.05f, 0.12f, A((Color){ 150, 152, 158, 255 }));
    if (GreenRoof(x, y)) GreenRoof3D(hh + 0.034f, cx, cz, x1 - x0 - 0.04f, z1 - z0 - 0.04f);
    if (smartCity && health[y][x] >= 50.0f) FlatSolar(cx, cz, hh + 0.045f, 3, 2, 0.30f, 0.62f, 0.24f, 0.20f, 10.0f);
}

static void DrawParkingLot3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    for (int c = 0; c <= 4; c++)
        for (int r = 0; r < 2; r++)
            BoxC(x + c * 0.25f, GROUND_TOP + 0.006f, y + 0.30f + r * 0.40f, 0.012f, 0.004f, 0.34f, A((Color){ 240, 240, 240, 255 }));
    if (((x + y) & 1) == 0) {               // lamp posts (the glow is live)
        BoxC(cx, 0.18f, cz, 0.014f, 0.36f, 0.014f, A((Color){ 70, 72, 78, 255 }));
        BoxC(cx, 0.37f, cz, 0.07f, 0.014f, 0.03f, A((Color){ 70, 72, 78, 255 }));
    }
}

// cars in the bays and the lamp glow (live)
static void DrawParkingLive3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    for (int s = 0; s < 8; s++) {
        if (!lot[y][x][s]) continue;
        int col = s % 4, row = s / 4;
        PushFrame(x + (col + 0.5f) * 0.25f, WALK_TOP, y + 0.30f + row * 0.40f, row ? 180.0f : 0.0f);
        DrawVehicleLocal(lot[y][x][s] - 1, CAR_COLS[lotCol[y][x][s]], false, NightNow());
        PopFrame();
    }
    if (((x + y) & 1) == 0 && NightNow()) GSphere(V3(cx, 0.355f, cz), 0.022f, (Color){ 255, 240, 170, 255 });
}
// the static part of the tip: weighbridge and loose refuse (the mounds are live)
static void DrawLandfill3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    if (gateTile[y][x]) {                   // weighbridge + site office
        BoxC(cx, 0.10f, cz - 0.28f, 0.34f, 0.20f, 0.22f, A((Color){ 205, 205, 195, 255 }));
        BoxC(cx, 0.21f, cz - 0.28f, 0.38f, 0.03f, 0.26f, A((Color){ 90, 100, 110, 255 }));
        BoxC(cx, 0.012f, cz + 0.15f, 0.55f, 0.024f, 0.20f, A((Color){ 100, 100, 108, 255 }));
        return;
    }
    for (int k = 0; k < 14; k++) {          // loose refuse, kept inside the fence
        float ox = (Hash01(x, y, k) - 0.5f) * 0.6f, oz = (Hash01(y, x, k + 5) - 0.5f) * 0.6f;
        Color c = (Color){ (unsigned char)(80 + Hash01(k, x, 3) * 170), (unsigned char)(80 + Hash01(k, y, 4) * 170), (unsigned char)(80 + Hash01(x, k, 5) * 170), 255 };
        BoxC(cx + ox, 0.06f + 0.14f * Hash01(k, y, x), cz + oz, 0.025f, 0.015f, 0.025f, A(c));
    }
}

// the mounds of waste (live, they grow with the fill level). Every mound is kept inside the fence.
static void DrawLandfillMounds3D(int x, int y) {
    unsigned char v = vr[y][x];
    if (gateTile[y][x] || surface[y][x] != T_LANDFILL) return;
    if (IsRecycleTile(x, y)) return;
    float fillK = 0.55f + 0.9f * landfillFill / 100.0f;
    float cx = x + 0.5f, cz = y + 0.5f;
    for (int k = 0; k < 3; k++) {
        float r = 0.12f + 0.04f * ((v >> k) & 3);                  // 0.12 .. 0.24 tile
        float lim = FENCE_ROAD - 0.07f - r;                       // fence stands at FENCE_LINE from the tile centre
        float ox = (((v >> (k * 2)) & 3) - 1.5f) * 0.20f, oz = (((v >> (k + 3)) & 3) - 1.5f) * 0.20f;
        ox = Clamp(ox, -lim, lim); oz = Clamp(oz, -lim, lim);
        float hh = bHeight[y][x] * fillK * (0.8f + 0.25f * k);
        GXPush(); GXTrans(cx + ox, 0.0f, cz + oz); GXScale(1.0f, hh / r * 0.8f, 1.0f);
        GSphere(V3(0, 0, 0), r, A(Shade((Color){ 98, 84, 66, 255 }, 0.8f + 0.1f * k)));
        GXPop();
    }
}
static void DrawRuin3D(int x, int y) {
    unsigned char v = vr[y][x];
    float cx = x + 0.5f, cz = y + 0.5f;
    BoxC(cx, 0.05f, cz, 0.7f, 0.1f, 0.7f, A(Shade(COL_RUIN, 0.8f)));
    for (int k = 0; k < 5; k++) {
        float ox = (((v >> k) & 3) - 1.5f) * 0.2f, oz = (((v >> (k + 1)) & 3) - 1.5f) * 0.2f;
        BoxC(cx + ox, 0.14f + 0.03f * k, cz + oz, 0.14f, 0.08f + 0.02f * k, 0.12f, A(Shade(COL_RUIN, 0.7f + 0.15f * k)));
    }
    BoxC(cx - 0.1f, 0.2f, cz + 0.1f, 0.02f, 0.2f, 0.02f, A((Color){ 30, 25, 22, 255 }));
}

static void DrawStation3D(int x, int y, bool fire) {
    float cx = x + 0.5f, cz = y + 0.5f;
    PushFrame(cx, 0, cz, FaceYaw(faceDir[y][x]));
    FitApplyAt(FitIn(x, y, faceDir[y][x], -0.49f, 0.49f, -0.46f, 0.46f), 0.0f, 0.0f);
    Color main = fire ? (Color){ 205, 62, 55, 255 } : (Color){ 235, 238, 245, 255 };
    BoxT(TX_BRICK, 0, 0.20f, 0, 0.96f, 0.40f, 0.92f, A(fire ? (Color){ 255, 190, 180, 255 } : (Color){ 200, 215, 245, 255 }));
    BoxC(0, 0.415f, 0, 0.98f, 0.03f, 0.96f, A(Shade(main, 0.7f)));
    if (fire) {
        for (int k = 0; k < 2; k++) {
            BoxC(-0.21f + k * 0.42f, 0.14f, 0.425f, 0.32f, 0.26f, 0.008f, A((Color){ 235, 235, 238, 255 }));
            for (int j = 0; j < 5; j++) BoxC(-0.21f + k * 0.42f, 0.04f + j * 0.05f, 0.43f, 0.32f, 0.004f, 0.002f, A((Color){ 170, 172, 176, 255 }));
        }
        BoxC(0.30f, 0.56f, -0.2f, 0.18f, 0.30f, 0.18f, A((Color){ 190, 55, 50, 255 }));               // hose tower
        BoxC(0, 0.34f, 0.43f, 0.7f, 0.04f, 0.01f, A((Color){ 250, 250, 250, 255 }));
    } else {
        BoxC(0, 0.15f, 0.425f, 0.22f, 0.24f, 0.008f, A((Color){ 60, 90, 170, 255 }));
        BoxC(0, 0.30f, 0.43f, 0.80f, 0.05f, 0.01f, A((Color){ 50, 85, 200, 255 }));
        bool fl = ((int)(GetTime() * 4.0)) % 2 == 0;
        BoxC(-0.07f, 0.46f, 0, 0.10f, 0.05f, 0.08f, fl ? A(RED) : A((Color){ 100, 20, 20, 255 }));
        BoxC( 0.07f, 0.46f, 0, 0.10f, 0.05f, 0.08f, fl ? A((Color){ 20, 40, 100, 255 }) : A(SKYBLUE));
    }
    if (smartCity) DrawStationSmart3D(fire, x, y);
    PopFrame();
}

static void DrawPlant3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    BoxT(TX_CONCRETE, cx, 0.15f, cz, 0.92f, 0.30f, 0.92f, A(WHITE));
    for (int k = 0; k < 2; k++) {
        float ox = -0.22f + k * 0.44f;
        GCylinder((Vector3){ cx + ox, 0.30f, cz - 0.1f }, 0.07f, 0.09f, 0.45f, 12, A((Color){ 215, 215, 220, 255 }));
        GCylinder((Vector3){ cx + ox, 0.75f, cz - 0.1f }, 0.065f, 0.07f, 0.08f, 12, A((Color){ 200, 55, 50, 255 }));
        GCylinder((Vector3){ cx + ox, 0.83f, cz - 0.1f }, 0.06f, 0.065f, 0.10f, 12, A((Color){ 235, 235, 238, 255 }));
    }
    GCylinder((Vector3){ cx, 0.30f, cz + 0.22f }, 0.15f, 0.20f, 0.30f, 20, A((Color){ 190, 190, 188, 255 }));   // cooling tower
    for (int k = 0; k < 3; k++) BoxC(cx - 0.3f + k * 0.18f, 0.34f, cz + 0.38f, 0.1f, 0.08f, 0.06f, A((Color){ 120, 125, 135, 255 }));
}
static void DrawWorks3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    BoxT(TX_CONCRETE, cx, 0.12f, cz, 0.92f, 0.24f, 0.92f, A((Color){ 190, 225, 235, 255 }));
    GCylinder((Vector3){ cx - 0.2f, 0.24f, cz }, 0.22f, 0.22f, 0.20f, 20, A((Color){ 70, 150, 175, 255 }));
    GCylinder((Vector3){ cx + 0.25f, 0.24f, cz - 0.2f }, 0.04f, 0.05f, 0.45f, 10, A((Color){ 180, 185, 190, 255 }));
    GSphere((Vector3){ cx + 0.25f, 0.72f, cz - 0.2f }, 0.14f, A((Color){ 70, 150, 200, 255 }));        // water tower
    BoxC(cx + 0.2f, 0.28f, cz + 0.26f, 0.3f, 0.04f, 0.3f, A((Color){ 70, 120, 200, 255 }));
    if (smartCity) FlatSolar(cx + 0.2f, cz + 0.26f, 0.315f, 1, 1, 0, 0, 0.26f, 0.22f, 10.0f);
}

// static buildings and grounds (baked)
static void DrawStructureStatic3D(int x, int y) {
    switch (surface[y][x]) {
        case T_BUILDING: if (bType[y][x] == B_IND) DrawIndustrial3D(x, y); else DrawBuilding3D(x, y); break;
        case T_HOUSE:    DrawHouse3D(x, y); break;
        case T_MALL:     DrawMall3D(x, y); break;
        case T_RUIN:     DrawRuin3D(x, y); break;
        case T_POWER:    if (!smartCity) { GXPush(); FitApplyAt(FitIn(x, y, -1, -0.46f, 0.46f, -0.46f, 0.46f), x + 0.5f, y + 0.5f); DrawPlant3D(x, y); GXPop(); } break;
        case T_WATER:    GXPush(); FitApplyAt(FitIn(x, y, -1, -0.46f, 0.46f, -0.46f, 0.46f), x + 0.5f, y + 0.5f); DrawWorks3D(x, y); GXPop(); break;
        case T_LANDFILL: if (!(IsRecycleTile(x, y) && !gateTile[y][x])) DrawLandfill3D(x, y); break;
        case T_PARK:     DrawParkingLot3D(x, y); break;
        default: break;
    }
}
// things that move, light up or change with the simulation (drawn every frame)
static void DrawStructureLive3D(int x, int y) {
    switch (surface[y][x]) {
        case T_BUILDING: if (bType[y][x] != B_IND) DrawBuildingLive3D(x, y); break;
        case T_HOUSE:    DrawHouseDrive3D(x, y); break;
        case T_PARK:     DrawParkingLive3D(x, y); break;
        case T_FIRE_STN: DrawStation3D(x, y, true); break;
        case T_POLICE_STN: DrawStation3D(x, y, false); break;
        case T_POWER:    if (smartCity) { GXPush(); FitApplyAt(FitIn(x, y, -1, -0.46f, 0.46f, -0.46f, 0.46f), x + 0.5f, y + 0.5f); DrawBattery3D(x, y); GXPop(); } break;
        case T_HOSPITAL: DrawHospital3D(x, y); break;
        case T_LANDFILL:
            if (IsRecycleTile(x, y) && !gateTile[y][x]) { GXPush(); FitApplyAt(FitIn(x, y, -1, -0.48f, 0.48f, -0.48f, 0.48f), x + 0.5f, y + 0.5f); DrawRecycle3D(x, y); GXPop(); }
            else DrawLandfillMounds3D(x, y);
            break;
        default: break;
    }
}
// ---------------------------------------------------------------------------
// 3D drawing - fences, gates, traffic lights, junction furniture
// ---------------------------------------------------------------------------
// fence segments: each edge runs on its own line (FenceLine: the sidewalk's inner edge beside a road, otherwise
// further in). An end that meets a fence on the same line runs to the tile edge; an end at a corner stops where
// the perpendicular fence stands, so the segments meet and never run through each other.
// one straight run of fence along the edge, from local x = xa to xb, on the line `inset`
static void DrawFenceRun(float xa, float xb, bool picket, float inset) {
    float len = xb - xa, xc = 0.5f * (xa + xb);
    if (len <= 0.005f) return;
    if (picket) {
        int n = (int)(len / 0.11f + 0.5f); if (n < 2) n = 2;
        BoxC(xc, 0.070f, inset, len, 0.012f, 0.010f, A(WHITE));
        BoxC(xc, 0.130f, inset, len, 0.012f, 0.010f, A(WHITE));
        for (int k = 0; k < n; k++) BoxC(xa + k * len / (n - 1), 0.080f, inset, 0.022f, 0.16f, 0.010f, A((Color){ 248, 248, 244, 255 }));
    } else {
        Color post = A((Color){ 90, 94, 100, 255 });
        BoxC(xa, 0.11f, inset, 0.016f, 0.22f, 0.016f, post);
        BoxC(xb, 0.11f, inset, 0.016f, 0.22f, 0.016f, post);
        BoxC(xc, 0.11f, inset, 0.016f, 0.22f, 0.016f, post);
        BoxC(xc, 0.115f, inset, len, 0.19f, 0.004f, A((Color){ 175, 180, 188, 150 }));           // panel
        BoxC(xc, 0.225f, inset, len, 0.008f, 0.008f, post);                                      // top rail
        BoxC(xc, 0.232f, inset, len, 0.004f, 0.004f, A((Color){ 60, 60, 64, 255 }));
    }
}
// a gateway into a house's front yard, centred at gc with half width hw: brick piers, a lintel, and the leaves folded back flat
static void DrawHouseGateway(bool picket, float inset, float gc, float hw) {
    float g0 = gc - hw, g1 = gc + hw, pz = inset - 0.02f;         // piers stand just inside the fence line
    Color cap = A((Color){ 150, 140, 125, 255 }), leaf = picket ? A((Color){ 246, 246, 240, 255 }) : A((Color){ 120, 124, 130, 255 });
    for (int s = -1; s <= 1; s += 2) {
        float px = (s < 0) ? g0 - 0.035f : g1 + 0.035f;
        BoxT(TX_BRICK, px, 0.13f, pz, 0.07f, 0.26f, 0.05f, A(WHITE));
        BoxC(px, 0.265f, pz, 0.09f, 0.02f, 0.05f, cap);
        if (NightNow()) GSphere(V3(px, 0.29f, pz), 0.018f, (Color){ 255, 235, 160, 255 });
        BoxC((s < 0) ? g0 + 0.03f : g1 - 0.03f, 0.10f, inset - 0.004f, 0.06f, 0.20f, 0.008f, leaf);     // leaf folded back against the pier
    }
    BoxC(gc, 0.25f, inset, g1 - g0 + 0.07f, 0.02f, 0.012f, cap);                                        // lintel
}
static void DrawFences3D(void) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int z = zone[y][x];
            if (!ZoneFenced(z)) continue;
            bool picket = (z <= Z_GATED3);
            for (int d = 0; d < 4; d++) {
                int nx = x + DX[d], ny = y + DY[d];
                if (InBounds(nx, ny) && zone[ny][nx] == z) continue;
                bool gate = gateTile[y][x] && IsRoad(nx, ny);
                float inset = FenceLine(x, y, d);
                PushFrame(x + 0.5f, 0, y + 0.5f, FaceYaw(d));
                if (gate) {
                    Color pil = A((Color){ 150, 140, 125, 255 });
                    for (int s = -1; s <= 1; s += 2) {
                        BoxT(TX_BRICK, s * 0.30f, 0.11f, inset - 0.02f, 0.07f, 0.22f, 0.05f, A(WHITE));
                        BoxC(s * 0.30f, 0.225f, inset - 0.02f, 0.09f, 0.012f, 0.05f, pil);
                    }
                    BoxC(-0.26f, 0.09f, inset, 0.16f, 0.012f, 0.012f, A((Color){ 235, 190, 40, 255 }));          // gate leaves (open)
                    BoxC(-0.26f, 0.05f, inset, 0.16f, 0.012f, 0.012f, A((Color){ 235, 190, 40, 255 }));
                    BoxC(0.0f, 0.03f, inset, 0.01f, 0.06f, 0.01f, A(DARKGRAY));                                    // barrier post (arm is live)
                } else {
                    float eL = FenceEnd(x, y, z, d, -1), eR = FenceEnd(x, y, z, d, 1);
                    if (HouseGateEdge(x, y) && d == faceDir[y][x]) {       // this house's front door faces the fence: leave a gateway
                        float gc, gh;
                        HouseGate(x, y, &gc, &gh);
                        DrawFenceRun(-eL, gc - gh, picket, inset);
                        DrawFenceRun(gc + gh, eR, picket, inset);
                        DrawHouseGateway(picket, inset, gc, gh);
                    } else DrawFenceRun(-eL, eR, picket, inset);
                    // where the fence steps to the next tile's line, bridge the step at the shared edge
                    int ox = x + DY[d], oy = y - DX[d];                    // the tile on this edge's +x side
                    if (InBounds(ox, oy) && zone[oy][ox] == z && FencedSide(ox, oy, d)) {
                        float other = FenceLine(ox, oy, d);
                        if (fabsf(other - inset) > 0.001f) {
                            float a = inset < other ? inset : other, b = inset < other ? other : inset;
                            BoxC(0.5f, 0.11f, 0.5f * (a + b), 0.016f, 0.22f, b - a + 0.016f, A((Color){ 90, 94, 100, 255 }));
                        }
                    }
                }
                PopFrame();
            }
        }
}

// the gate barrier and its lamps (live)
static void DrawFencesLive3D(void) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int z = zone[y][x];
            if (!ZoneFenced(z) || !gateTile[y][x]) continue;
            for (int d = 0; d < 4; d++) {
                int nx = x + DX[d], ny = y + DY[d];
                if (!(InBounds(nx, ny) && zone[ny][nx] != z && IsRoad(nx, ny))) continue;
                PushFrame(x + 0.5f, 0, y + 0.5f, FaceYaw(d));
                float inset = FenceLine(x, y, d);
                for (int s = -1; s <= 1; s += 2)
                    if (NightNow()) GSphere(V3(s * 0.30f, 0.23f, inset), 0.018f, (Color){ 255, 235, 160, 255 });
                BoxC(0.0f, 0.05f, inset, 0.50f, 0.01f, 0.01f, A(((int)(GetTime() * 1.0) % 2) ? RED : WHITE));   // barrier arm
                PopFrame();
            }
        }
}
static void DrawSignalHead(float cx, float cz, float yaw, int state) {
    PushFrame(cx, 0, cz, yaw);
    BoxC(0, 0.09f, 0, 0.014f, 0.18f, 0.014f, A((Color){ 60, 62, 68, 255 }));
    BoxC(0, 0.205f, 0.012f, 0.045f, 0.105f, 0.030f, A((Color){ 25, 25, 28, 255 }));
    Color r = (state == 0) ? (Color){ 255, 50, 40, 255 } : (Color){ 70, 15, 15, 255 };
    Color y = (state == 2) ? (Color){ 255, 205, 40, 255 } : (Color){ 70, 55, 10, 255 };
    Color g = (state == 1) ? (Color){ 60, 255, 100, 255 } : (Color){ 12, 60, 25, 255 };
    GSphere((Vector3){ 0, 0.235f, 0.030f }, 0.011f, A(r));
    GSphere((Vector3){ 0, 0.205f, 0.030f }, 0.011f, A(y));
    GSphere((Vector3){ 0, 0.175f, 0.030f }, 0.011f, A(g));
    PopFrame();
}

// roundabouts and stop signs (static); traffic-signal heads are live
static void DrawJunction3D(int x, int y, bool xray) {
    float cx = x + 0.5f, cz = y + 0.5f;
    if (ctrl[y][x] == C_ROUNDABOUT) {
        GXPush(); GXTrans(0.0f, GROUND_TOP - 0.06f, 0.0f);          // the ring was modelled on the old road top
        GCylinder(V3(cx, 0.058f, cz), 0.41f, 0.41f, 0.004f, 36, A((Color){ 190, 190, 192, 255 }));
        GCylinder(V3(cx, 0.061f, cz), 0.37f, 0.37f, 0.004f, 36, A(xray ? COL_ROAD_DIM : COL_ROAD));
        GCylinder(V3(cx, 0.06f, cz), 0.215f, 0.215f, 0.05f, 28, A((Color){ 160, 160, 162, 255 }));
        GCylinder(V3(cx, 0.11f, cz), 0.20f, 0.20f, 0.02f, 28, A((Color){ 70, 140, 65, 255 }));
        GSphere(V3(cx + 0.05f, 0.17f, cz), 0.07f, A((Color){ 48, 125, 55, 255 }));
        GSphere(V3(cx - 0.06f, 0.16f, cz + 0.05f), 0.055f, A((Color){ 62, 145, 62, 255 }));
        for (int k = 0; k < 6; k++) {
            float a = k * PI / 3.0f;
            GSphere(V3(cx + 0.13f * cosf(a), 0.15f, cz + 0.13f * sinf(a)), 0.016f, A(k & 1 ? (Color){ 240, 90, 120, 255 } : (Color){ 250, 215, 70, 255 }));
        }
        GXPop();
    } else if (ctrl[y][x] == C_STOP) {
        static const float px[4] = { -0.45f, 0.45f, 0.45f, -0.45f };
        static const float pz[4] = { -0.45f, -0.45f, 0.45f, 0.45f };
        static const float yaw[4] = { -90.0f, 180.0f, 90.0f, 0.0f };
        for (int i = 0; i < 4; i++) {
            PushFrame(cx + px[i], 0, cz + pz[i], yaw[i]);
            BoxC(0, 0.08f, 0, 0.010f, 0.16f, 0.010f, A((Color){ 150, 150, 155, 255 }));
            GXPush(); GXTrans(0, 0.18f, 0.008f); GXRot(90.0f, 1, 0, 0);
            GCylinder(V3(0, 0, 0), 0.038f, 0.038f, 0.006f, 8, A(RED));
            GXPop();
            BoxC(0, 0.18f, 0.012f, 0.040f, 0.008f, 0.002f, A(WHITE));
            PopFrame();
        }
    }
}

// traffic-signal heads (live: they change colour)
static void DrawJunctionLive3D(int x, int y) {
    if (ctrl[y][x] != C_SIGNAL) return;
    float cx = x + 0.5f, cz = y + 0.5f;
    static const float px[4] = { -0.45f, 0.45f, 0.45f, -0.45f };
    static const float pz[4] = { -0.45f, -0.45f, 0.45f, 0.45f };
    static const float yaw[4] = { -90.0f, 180.0f, 90.0f, 0.0f };
    static const int axisOf[4] = { 1, 0, 1, 0 };
    for (int i = 0; i < 4; i++) DrawSignalHead(cx + px[i], cz + pz[i], yaw[i], SigState(x, y, axisOf[i], NULL));
}
// zebra crossings, stop lines
static void DrawRoadMarkings3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    if (crossT[y][x]) {
        for (int i = 0; i < 5; i++) {
            float o = (-0.36f + i * 0.18f);
            if (crossT[y][x] == 1) BoxC(cx + o, GROUND_TOP + 0.003f, cz, 0.07f, 0.004f, 0.30f, A(WHITE));
            else                   BoxC(cx, GROUND_TOP + 0.003f, cz + o, 0.30f, 0.004f, 0.07f, A(WHITE));
        }
        int ji = crossI[y][x];
        if (ji >= 0 && ctrl[ji / MAP_W][ji % MAP_W] != C_NONE) {
            for (int d = 0; d < 4; d++) {
                if (x + DX[d] != ji % MAP_W || y + DY[d] != ji / MAP_W) continue;
                float lane = LANE_CAR;
                float lx = DY[d] * lane, lz = -DX[d] * lane;                    // lane of traffic heading toward the junction
                float bx = cx - DX[d] * 0.26f + lx, bz = cz - DY[d] * 0.26f + lz;
                if (d < 2) BoxC(bx, GROUND_TOP + 0.003f, bz, 0.30f, 0.004f, 0.025f, A(WHITE));
                else       BoxC(bx, GROUND_TOP + 0.003f, bz, 0.025f, 0.004f, 0.30f, A(WHITE));
            }
        }
    }
}
// ---------------------------------------------------------------------------
// 3D drawing - smart city: solar roofs, living walls, smoke traps, drones, ambulance,
// hospital, recycling plant, battery storage, road monitors, renewable farm
// ---------------------------------------------------------------------------
static void SolarPanel(float x, float y, float z, float w, float d, float tilt) {
    GXPush(); GXTrans(x, y, z); GXRot(tilt, 1, 0, 0);
    BoxT(TX_SOLAR, 0, 0, 0, w, 0.008f, d, A(WHITE));
    GXPop();
}

static void FlatSolar(float cx, float cz, float y, int nx, int nz, float gx, float gz, float pw, float pd, float tilt) {
    for (int i = 0; i < nx; i++)
        for (int k = 0; k < nz; k++)
            SolarPanel(cx + (i - (nx - 1) * 0.5f) * gx, y, cz + (k - (nz - 1) * 0.5f) * gz, pw, pd, tilt);
}

// panel lying on the +side slope of a gable roof (a = half span, s0..s1 = distance from the ridge, L = half length)
static void GableSolar(bool alongZ, float a, float y0, float h, float s0, float s1, float L) {
    float ya = y0 + h * (1.0f - s0 / a) + 0.004f, yb = y0 + h * (1.0f - s1 / a) + 0.004f;
    Vector3 p0, p1, p2, p3;
    if (!alongZ) { p0 = (Vector3){ -L, ya, s0 }; p1 = (Vector3){ -L, yb, s1 }; p2 = (Vector3){ L, yb, s1 }; p3 = (Vector3){ L, ya, s0 }; }
    else         { p0 = (Vector3){ s0, ya, -L }; p1 = (Vector3){ s1, yb, -L }; p2 = (Vector3){ s1, yb, L }; p3 = (Vector3){ s0, ya, L }; }
    Quad3T(TX_SOLAR, p0, p1, p2, p3, A(WHITE), L * 2.0f / 0.07f, (s1 - s0) / 0.07f);
}

// roof panels in the house's roof frame (origin = centre of the roof, see DrawHouse3D)
static void HouseSolar3D(int rtype) {
    if (rtype == 0)      GableSolar(false, HS_L * 0.5f, HS_WH, HS_RH, 0.04f, 0.16f, HS_W * 0.5f - 0.03f);
    else if (rtype == 1) GableSolar(true,  HS_W * 0.5f, HS_WH, HS_RH, 0.04f, 0.16f, HS_L * 0.5f - 0.03f);
    else if (rtype == 2) GableSolar(false, HS_L * 0.5f, HS_WH, HS_RH, 0.06f, 0.16f, 0.05f);
    else                 FlatSolar(0.0f, 0.0f, HS_WH + 0.030f, 2, 1, 0.19f, 0.0f, 0.16f, 0.14f, 12.0f);
}
// vertical gardens: a planter ledge with hedge and flowers every other floor, plus climbing plants at the corners
// vertical gardens: a planter ledge with hedge and flowers every other floor, plus climbing plants at the corners
static void LivingWall(float y0, int floors, float w, int startF) {
    float o = w * 0.5f + 0.012f;
    for (int f = startF; f < floors; f += 2) {
        float y = y0 + f * FLOOR_H;
        BoxC(0, y, 0, w + 0.05f, 0.016f, w + 0.05f, A((Color){ 150, 152, 148, 255 }));
        BoxT(TX_HEDGE, 0, y + 0.03f,  o + 0.012f, w + 0.05f, 0.05f, 0.045f, A(WHITE));
        BoxT(TX_HEDGE, 0, y + 0.03f, -o - 0.012f, w + 0.05f, 0.05f, 0.045f, A(WHITE));
        BoxT(TX_HEDGE,  o + 0.012f, y + 0.03f, 0, 0.045f, 0.05f, w + 0.05f, A(WHITE));
        BoxT(TX_HEDGE, -o - 0.012f, y + 0.03f, 0, 0.045f, 0.05f, w + 0.05f, A(WHITE));
        if ((f / 2) % 2 == 0) {
            for (int k = 0; k < 4; k++) {
                Color fc = (k + f) & 1 ? (Color){ 245, 120, 160, 255 } : (Color){ 250, 225, 90, 255 };
                GSphere(V3(-w * 0.35f + k * w * 0.23f, y + 0.062f, o + 0.03f), 0.011f, A(fc));
                GSphere(V3(o + 0.03f, y + 0.062f, -w * 0.35f + k * w * 0.23f), 0.011f, A(fc));
            }
        }
    }
    float hh = floors * FLOOR_H;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sz = -1; sz <= 1; sz += 2)
            BoxT(TX_HEDGE, sx * (w * 0.5f + 0.004f), y0 + hh * 0.5f, sz * (w * 0.5f + 0.004f), 0.032f, hh, 0.032f, A(WHITE));
}
static void RooftopGarden3D(float h) {          // on the crown of a skyrise
    BoxT(TX_HEDGE, -0.22f, h + 0.062f, -0.20f, 0.15f, 0.04f, 0.15f, A(WHITE));
    BoxT(TX_HEDGE,  0.21f, h + 0.062f, -0.20f, 0.12f, 0.04f, 0.15f, A(WHITE));
    BoxT(TX_HEDGE, -0.22f, h + 0.062f,  0.22f, 0.15f, 0.04f, 0.12f, A(WHITE));
    GSphere((Vector3){ -0.22f, h + 0.115f, -0.20f }, 0.045f, A((Color){ 60, 150, 62, 255 }));
    GSphere((Vector3){  0.21f, h + 0.105f, -0.20f }, 0.038f, A((Color){ 75, 165, 70, 255 }));
}

// flue-gas scrubber + baghouse beside a factory chimney
static void SmokeTrap3D(float h) {
    GCylinder((Vector3){ 0.24f, h, 0.06f }, 0.075f, 0.075f, 0.30f, 14, A((Color){ 70, 150, 150, 255 }));
    GCylinder((Vector3){ 0.24f, h + 0.30f, 0.06f }, 0.0f, 0.075f, 0.05f, 14, A((Color){ 55, 118, 122, 255 }));
    BoxC(0.24f, h + 0.19f, -0.07f, 0.030f, 0.030f, 0.30f, A((Color){ 110, 116, 122, 255 }));          // duct into the stack
    BoxC(0.36f, h + 0.07f, 0.06f, 0.07f, 0.14f, 0.10f, A((Color){ 185, 190, 195, 255 }));             // baghouse filter
    GCylinder((Vector3){ 0.24f, h + 0.44f, -0.20f }, 0.052f, 0.052f, 0.022f, 12, A((Color){ 80, 215, 130, 255 }));   // "clean" cap
    GSphere((Vector3){ 0.31f, h + 0.22f, 0.06f }, 0.010f, (Color){ 70, 255, 120, 255 });
}

// ---- drones ------------------------------------------------------------------
static void DrawDroneModel(int kind, float spinDeg, bool carrying) {
    Color body = (kind == D_FIRE) ? (Color){ 215, 62, 52, 255 } : (kind == D_PARCEL ? (Color){ 250, 170, 50, 255 } : (Color){ 245, 247, 250, 255 });
    Color dark = (Color){ 40, 42, 48, 255 };
    BoxC(0, 0, 0, 0.075f, 0.026f, 0.075f, A(body));
    for (int k = 0; k < 2; k++) { GXPush(); GXRot(45.0f + 90.0f * k, 0, 1, 0); BoxC(0, 0.004f, 0, 0.20f, 0.010f, 0.012f, A(dark)); GXPop(); }
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
            float px = sx * 0.07f, pz = sz * 0.07f;
            GCylinder((Vector3){ px, 0.012f, pz }, 0.040f, 0.040f, 0.004f, 10, Fade(WHITE, 0.16f * gAlpha));
            GXPush(); GXTrans(px, 0.018f, pz); GXRot(spinDeg, 0, 1, 0);
            BoxC(0, 0, 0, 0.085f, 0.003f, 0.010f, A(dark));
            GXRot(90.0f, 0, 1, 0); BoxC(0, 0, 0, 0.085f, 0.003f, 0.010f, A(dark));
            GXPop();
        }
    bool flash = ((int)(GetTime() * 6.0)) & 1;
    if (kind == D_POLICE) {
        BoxC(-0.016f, 0.020f, 0, 0.026f, 0.012f, 0.024f, flash ? (Color){ 255, 40, 40, 255 } : (Color){ 90, 15, 15, 255 });
        BoxC( 0.016f, 0.020f, 0, 0.026f, 0.012f, 0.024f, flash ? (Color){ 20, 40, 110, 255 } : (Color){ 80, 170, 255, 255 });
        GSphere((Vector3){ 0, -0.024f, 0.014f }, 0.014f, A((Color){ 25, 28, 34, 255 }));                // camera
    } else if (kind == D_FIRE) {
        GCylinder((Vector3){ 0, -0.052f, 0 }, 0.026f, 0.026f, 0.040f, 10, A((Color){ 235, 235, 238, 255 }));   // water tank
        BoxC(0, -0.060f, 0.040f, 0.010f, 0.010f, 0.045f, A(dark));                                        // nozzle
        BoxC(0, 0.018f, 0, 0.030f, 0.008f, 0.012f, flash ? (Color){ 255, 190, 40, 255 } : (Color){ 110, 80, 10, 255 });
    } else if (kind == D_PARCEL || kind == D_MEDSUP) {                                                      // delivery drones
        bool par = (kind == D_PARCEL);
        BoxC(0, 0.016f, 0, 0.040f, 0.006f, 0.012f, A(par ? (Color){ 255, 120, 20, 255 } : (Color){ 40, 165, 90, 255 }));
        BoxC(0, 0.016f, 0, 0.012f, 0.006f, 0.040f, A(par ? (Color){ 255, 120, 20, 255 } : (Color){ 40, 165, 90, 255 }));
        BoxC(0, 0.020f, 0.030f, 0.012f, 0.008f, 0.012f, flash ? (Color){ 60, 255, 120, 255 } : (Color){ 20, 90, 40, 255 });
        if (carrying) {                                                                                     // package on a tether
            BoxC(0, -0.026f, 0, 0.004f, 0.026f, 0.004f, A(dark));
            BoxC(0, -0.062f, 0, 0.056f, 0.048f, 0.056f, A(par ? (Color){ 200, 150, 80, 255 } : (Color){ 245, 245, 248, 255 }));
            if (par) BoxC(0, -0.038f, 0, 0.058f, 0.004f, 0.014f, A((Color){ 150, 105, 55, 255 }));
            else { BoxC(0, -0.062f, 0.029f, 0.028f, 0.008f, 0.004f, A((Color){ 215, 40, 40, 255 })); BoxC(0, -0.062f, 0.029f, 0.008f, 0.028f, 0.004f, A((Color){ 215, 40, 40, 255 })); }
        }
    } else {
        BoxC(0, 0.016f, 0, 0.034f, 0.006f, 0.010f, A((Color){ 215, 40, 40, 255 }));                       // red cross
        BoxC(0, 0.016f, 0, 0.010f, 0.006f, 0.034f, A((Color){ 215, 40, 40, 255 }));
        BoxC(0, 0.020f, 0.030f, 0.012f, 0.008f, 0.012f, flash ? (Color){ 60, 255, 120, 255 } : (Color){ 20, 90, 40, 255 });
        if (carrying) {                                                                                     // stretcher pod
            BoxC(0, -0.060f, 0, 0.060f, 0.040f, 0.105f, A((Color){ 245, 245, 248, 255 }));
            BoxC(0, -0.040f, 0, 0.056f, 0.004f, 0.100f, A((Color){ 90, 150, 200, 255 }));
            BoxC(0, -0.060f, 0.0535f, 0.026f, 0.008f, 0.004f, A((Color){ 215, 40, 40, 255 }));
        }
    }
}

static void DrawDrone3D(const Drone *d, double t) {
    float spin = fmodf((float)t * 1500.0f, 360.0f);
    GCylinder((Vector3){ d->x, 0.074f, d->y }, 0.07f + d->z * 0.012f, 0.07f + d->z * 0.012f, 0.002f, 12, (Color){ 0, 0, 0, (unsigned char)Clamp(95.0f - d->z * 14.0f, 25.0f, 95.0f) });
    PushFrame(d->x, d->z, d->y, d->yaw);
    GXScale(1.35f, 1.35f, 1.35f);
    DrawDroneModel(d->kind, spin, d->carrying);
    PopFrame();
    if (d->spraying) {
        Vector3 a = { d->x, d->z - 0.05f, d->y }, b = { d->sx, 0.10f, d->sy };
        if (d->kind == D_FIRE) {
            b.y += 0.06f * sinf((float)t * 12.0f);
            GCylinderEx(a, b, 0.010f, 0.030f, 6, (Color){ 140, 200, 255, 170 });
            GSphere(b, 0.07f, (Color){ 200, 230, 255, 120 });
        } else if (d->kind == D_POLICE) {
            GCylinderEx(a, b, 0.006f, 0.075f, 8, (Color){ 255, 250, 205, 55 });                          // spotlight beam
        }
    }
}

// parked drone sitting on a roof pad
static void DrawPad3D(float x, float y, float z, int kind, bool drone) {
    GCylinder((Vector3){ x, y, z }, 0.13f, 0.13f, 0.010f, 20, A((Color){ 52, 56, 64, 255 }));
    GCylinder((Vector3){ x, y + 0.010f, z }, 0.11f, 0.11f, 0.002f, 20, A((Color){ 250, 205, 40, 255 }));
    GCylinder((Vector3){ x, y + 0.012f, z }, 0.095f, 0.095f, 0.002f, 20, A((Color){ 52, 56, 64, 255 }));
    BoxC(x, y + 0.016f, z, 0.07f, 0.004f, 0.012f, A((Color){ 250, 250, 250, 255 }));
    BoxC(x, y + 0.016f, z, 0.012f, 0.004f, 0.07f, A((Color){ 250, 250, 250, 255 }));
    if (drone) { PushFrame(x, y + 0.05f, z, 0.0f); GXScale(1.2f, 1.2f, 1.2f); DrawDroneModel(kind, 0.0f, false); PopFrame(); }
}
static bool PadBusy(int hx, int hy) {
    for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active && drones[i].homeX == hx && drones[i].homeY == hy) return true;
    return false;
}

// roof of a fire / police station in the smart city (local frame of the station, roof at y = 0.43)
static void DrawStationSmart3D(bool fire, int sx, int sy) {
    DrawPad3D(fire ? -0.22f : 0.0f, 0.43f, -0.16f, fire ? D_FIRE : D_POLICE, !PadBusy(sx, sy));
    if (fire) FlatSolar(-0.02f, 0.20f, 0.455f, 3, 1, 0.26f, 0, 0.21f, 0.16f, 12.0f);
    else      FlatSolar(0.0f, 0.22f, 0.455f, 3, 1, 0.27f, 0, 0.22f, 0.16f, 12.0f);
}

// ---- ambulance (normal city) --------------------------------------------------
static void DrawAmbulance3D(const Responder *r, double t) {
    PushFrame(r->x, ENT_Y, r->y, 90.0f - r->angle);
    Color w = A(WHITE);
    BoxT(TX_AMBU, 0, 0.074f, -0.030f, 0.125f, 0.100f, 0.22f, w);
    BoxC(0, 0.062f, 0.130f, 0.118f, 0.074f, 0.085f, A((Color){ 246, 247, 250, 255 }));
    BoxC(0, 0.078f, 0.176f, 0.100f, 0.042f, 0.006f, A(GLASS));
    BoxC(0, 0.130f, -0.030f, 0.036f, 0.004f, 0.10f, A((Color){ 215, 40, 40, 255 }));
    BoxC(0, 0.130f, -0.030f, 0.100f, 0.004f, 0.034f, A((Color){ 215, 40, 40, 255 }));
    bool f = ((int)(t * 7.0)) % 2 == 0;
    BoxC(-0.032f, 0.128f, 0.135f, 0.045f, 0.012f, 0.020f, f ? (Color){ 255, 40, 40, 255 } : (Color){ 110, 20, 20, 255 });
    BoxC( 0.032f, 0.128f, 0.135f, 0.045f, 0.012f, 0.020f, f ? (Color){ 20, 60, 255, 255 } : (Color){ 20, 30, 110, 255 });
    Wheels(0.125f, 0.30f, 2);
    PopFrame();
}

// ---- hospital -----------------------------------------------------------------
static void DrawHospital3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    bool dmg = health[y][x] < 50.0f;
    Color tint = A(dmg ? (Color){ 120, 100, 95, 255 } : WHITE);
    PushFrame(cx, 0, cz, FaceYaw(faceDir[y][x]));
    FitApplyAt(FitIn(x, y, faceDir[y][x], -0.46f, 0.46f, -0.37f, 0.49f), 0.0f, 0.0f);
    BoxT(TX_HOSP, 0, 0.17f, -0.02f, 0.90f, 0.34f, 0.70f, tint);
    BoxT(TX_HOSP, -0.22f, 0.37f, -0.12f, 0.36f, 0.74f, 0.36f, tint);
    BoxC(0, 0.355f, -0.02f, 0.94f, 0.03f, 0.74f, A((Color){ 150, 152, 158, 255 }));
    BoxC(-0.22f, 0.755f, -0.12f, 0.40f, 0.03f, 0.40f, A((Color){ 150, 152, 158, 255 }));
    BoxC(-0.22f, 0.62f, 0.064f, 0.19f, 0.19f, 0.006f, A(WHITE));                                  // red cross sign
    BoxC(-0.22f, 0.62f, 0.070f, 0.13f, 0.042f, 0.006f, A((Color){ 215, 35, 40, 255 }));
    BoxC(-0.22f, 0.62f, 0.070f, 0.042f, 0.13f, 0.006f, A((Color){ 215, 35, 40, 255 }));
    BoxC(0.22f, 0.15f, 0.40f, 0.36f, 0.022f, 0.17f, A((Color){ 205, 55, 55, 255 }));            // emergency canopy
    for (int s = -1; s <= 1; s += 2) BoxC(0.22f + s * 0.16f, 0.075f, 0.45f, 0.016f, 0.15f, 0.016f, A((Color){ 235, 235, 238, 255 }));
    BoxC(0.22f, 0.012f, 0.40f, 0.30f, 0.004f, 0.14f, A((Color){ 215, 40, 40, 255 }));
    BoxC(0.22f, 0.09f, 0.352f, 0.20f, 0.12f, 0.006f, A((Color){ 70, 110, 150, 255 }));          // glass doors
    if (smartCity) {
        DrawPad3D(-0.22f, 0.77f, -0.12f, D_MEDIC, !PadBusy(x, y));
        FlatSolar(0.22f, -0.16f, 0.39f, 2, 2, 0.20f, 0.18f, 0.17f, 0.14f, 12.0f);
    }
    PopFrame();
}

// ---- battery storage (replaces the power plant in the smart city) ------------------
static void DrawBattery3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    BoxT(TX_CONCRETE, cx, 0.02f, cz, 0.94f, 0.04f, 0.94f, A(WHITE));
    for (int i = 0; i < 2; i++)
        for (int k = 0; k < 2; k++) {
            float px = cx - 0.21f + i * 0.42f, pz = cz - 0.20f + k * 0.34f;
            BoxT(TX_CORRUG, px, 0.15f, pz, 0.36f, 0.22f, 0.28f, A((Color){ 215, 240, 225, 255 }));
            BoxC(px, 0.27f, pz, 0.38f, 0.014f, 0.30f, A((Color){ 90, 120, 105, 255 }));
            for (int s = 0; s < 5; s++) BoxC(px - 0.12f + s * 0.06f, 0.15f, pz + 0.142f, 0.03f, 0.10f, 0.004f, A((Color){ 60, 215, 120, 255 }));
        }
    GCylinder((Vector3){ cx - 0.35f, 0.04f, cz + 0.36f }, 0.06f, 0.06f, 0.14f, 10, A((Color){ 170, 175, 182, 255 }));   // transformer
    BoxC(cx + 0.25f, 0.12f, cz + 0.37f, 0.30f, 0.16f, 0.10f, A((Color){ 140, 150, 160, 255 }));                          // inverters
    float fill = Clamp(batteryPct / 100.0f, 0.0f, 1.0f);                                                                  // charge gauge
    BoxC(cx + 0.42f, 0.34f, cz, 0.04f, 0.60f, 0.04f, A((Color){ 40, 44, 50, 255 }));
    Color gc = fill > 0.5f ? (Color){ 70, 230, 120, 255 } : (fill > 0.2f ? (Color){ 250, 205, 50, 255 } : (Color){ 240, 70, 60, 255 });
    BoxC(cx + 0.42f, 0.04f + 0.30f * fill, cz, 0.05f, 0.60f * fill, 0.05f, gc);
    FlatSolar(cx, cz + 0.02f, 0.30f, 2, 2, 0.42f, 0.34f, 0.30f, 0.22f, 10.0f);
}

// ---- recycling plant (part of the old landfill site) -------------------------------
static void RecycleSign3D(float cx, float y, float cz) {
    GCylinder((Vector3){ cx, y, cz }, 0.17f, 0.17f, 0.012f, 24, A((Color){ 40, 160, 90, 255 }));
    for (int k = 0; k < 3; k++) {
        PushFrame(cx, y + 0.014f, cz, k * 120.0f + (float)GetTime() * 20.0f);
        BoxC(0.08f, 0, 0, 0.09f, 0.008f, 0.026f, A((Color){ 245, 250, 245, 255 }));
        Tri3T(TX_CONCRETE, (Vector3){ 0.135f, 0.004f, 0.034f }, (Vector3){ 0.135f, 0.004f, -0.034f }, (Vector3){ 0.175f, 0.004f, 0.0f }, A((Color){ 245, 250, 245, 255 }), 0.1f, 0.1f);
        PopFrame();
    }
}

static void DrawRecycle3D(int x, int y) {
    unsigned char v = vr[y][x];
    float cx = x + 0.5f, cz = y + 0.5f, t = (float)GetTime();
    static const Color bale[5] = { {70,110,190,255}, {190,195,202,255}, {205,170,110,255}, {85,160,95,255}, {225,225,120,255} };
    if (x == 20 && y == 11) {                                         // main sorting hall
        BoxT(TX_CORRUG, cx, 0.20f, cz, 0.94f, 0.40f, 0.86f, A((Color){ 190, 235, 210, 255 }));
        for (int k = 0; k < 3; k++) { GXPush(); GXTrans(cx - 0.31f + k * 0.31f, 0, cz); RoofGable(0.30f, 0.86f, 0.40f, 0.07f, true, A((Color){ 60, 150, 110, 255 }), TX_CORRUG, A((Color){ 200, 230, 215, 255 })); GXPop(); }
        BoxC(cx, 0.30f, cz + 0.436f, 0.94f, 0.06f, 0.012f, A((Color){ 40, 160, 90, 255 }));
        BoxC(cx - 0.2f, 0.10f, cz + 0.436f, 0.28f, 0.18f, 0.012f, A((Color){ 80, 95, 100, 255 }));
        RecycleSign3D(cx, 0.49f, cz);
        GCylinder((Vector3){ cx + 0.34f, 0.40f, cz - 0.28f }, 0.035f, 0.04f, 0.20f, 10, A((Color){ 170, 175, 180, 255 }));
        FlatSolar(cx + 0.02f, cz - 0.24f, 0.495f, 3, 1, 0.28f, 0, 0.22f, 0.16f, 10.0f);
    } else if (x == 20) {                                              // conveyor + bale presses
        BoxT(TX_CORRUG, cx - 0.05f, 0.14f, cz, 0.84f, 0.28f, 0.80f, A((Color){ 200, 220, 210, 255 }));
        BoxC(cx - 0.05f, 0.285f, cz, 0.86f, 0.014f, 0.82f, A((Color){ 70, 140, 105, 255 }));
        BoxC(cx + 0.0f, 0.045f, cz, 0.10f, 0.03f, 0.96f, A((Color){ 55, 58, 64, 255 }));
        for (int i = 0; i < 6; i++) {
            float zz = cz - 0.46f + fmodf(t * 0.35f + i * 0.17f, 1.0f) * 0.92f;
            BoxC(cx, GROUND_TOP + 0.006f, zz, 0.08f, 0.012f, 0.045f, A(bale[(v + i) % 5]));
        }
        for (int i = 0; i < 3; i++) BoxT(TX_CORRUG, cx + 0.34f, 0.07f + (i % 2) * 0.0f, cz - 0.30f + i * 0.30f, 0.20f, 0.12f, 0.18f, A(bale[(v + i) % 5]));
        FlatSolar(cx - 0.05f, cz, 0.30f, 2, 3, 0.36f, 0.22f, 0.30f, 0.18f, 10.0f);
    } else {                                                           // sorting yard: skips, bales, loader
        BoxT(TX_CONCRETE, cx, 0.012f, cz, 0.98f, 0.024f, 0.98f, A(WHITE));
        for (int i = 0; i < 4; i++) {
            float px = cx - 0.33f + (i % 2) * 0.50f, pz = cz - 0.30f + (i / 2) * 0.28f;
            BoxT(TX_CORRUG, px, 0.07f, pz, 0.30f, 0.12f, 0.20f, A(bale[(v + i) % 5]));
            if ((v >> i) & 1) BoxT(TX_CORRUG, px, 0.19f, pz, 0.28f, 0.10f, 0.18f, A(bale[(v + i + 2) % 5]));
        }
        PushFrame(cx + 0.05f, 0.03f, cz + 0.30f, (y == 10) ? 90.0f : -90.0f);
        DrawVehicleLocal(V_TRUCK, (Color){ 60, 150, 90, 255 }, false, NightNow());
        PopFrame();
    }
}

// ---- smart traffic monitors -------------------------------------------------------
static void DrawMonitor3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    bool horiz = (RoadLine(x, y) == 1);
    float px = horiz ? cx + 0.05f : cx + 0.465f, pz = horiz ? cz + 0.465f : cz + 0.05f;
    float ax = horiz ? 0.0f : -1.0f, az = horiz ? -1.0f : 0.0f;          // direction towards the road
    Color mast = A((Color){ 85, 95, 108, 255 });
    BoxC(px, 0.15f, pz, 0.014f, 0.30f, 0.014f, mast);
    float hx = px + ax * 0.20f, hz = pz + az * 0.20f;
    BoxC((px + hx) * 0.5f, 0.30f, (pz + hz) * 0.5f, horiz ? 0.012f : 0.20f, 0.012f, horiz ? 0.20f : 0.012f, mast);
    BoxC(hx, 0.29f, hz, 0.055f, 0.035f, 0.055f, A((Color){ 30, 38, 52, 255 }));
    GSphere((Vector3){ hx, 0.268f, hz }, 0.014f, (Color){ 120, 220, 255, 255 });                  // lens
    Color rc = CongColor(traffic[y][x], 1.0f);
    GCylinder((Vector3){ hx, 0.31f, hz }, 0.030f, 0.030f, 0.010f, 12, rc);                         // congestion status ring
    SolarPanel(px, 0.325f, pz, 0.07f, 0.05f, 12.0f);
    if (NightNow() || st.daylight < 0.7f) GCylinderEx((Vector3){ hx, 0.265f, hz }, (Vector3){ cx + ax * 0.45f * 0.0f, 0.07f, cz }, 0.004f, 0.10f, 8, (Color){ 120, 220, 255, 26 });
}

// ---- renewable farm east of the city ----------------------------------------------
static void DrawTurbine3D(float tx, float tz, float t) {
    GCylinder((Vector3){ tx, 0, tz }, 0.016f, 0.034f, 1.10f, 10, A((Color){ 235, 238, 242, 255 }));
    BoxC(tx - 0.02f, 1.12f, tz, 0.10f, 0.05f, 0.05f, A((Color){ 225, 228, 232, 255 }));
    float ang = t * (30.0f + 300.0f * WindFactor()) + tx * 37.0f;
    GXPush(); GXTrans(tx - 0.075f, 1.12f, tz); GXRot(ang, 1, 0, 0);
    GSphere((Vector3){ 0, 0, 0 }, 0.028f, A((Color){ 240, 242, 246, 255 }));
    for (int k = 0; k < 3; k++) {
        GXPush(); GXRot(k * 120.0f, 1, 0, 0);
        BoxC(0, 0.23f, 0, 0.010f, 0.44f, 0.034f, A((Color){ 245, 247, 250, 255 }));
        GXPop();
    }
    GXPop();
}

// renewable farm, static part: ground, solar tables, grid hub, fence and power line
static void DrawFarmStatic3D(void) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = MAP_W; x < MAP_W + FARM_W; x++) {
            bool road = (y == FARM_ROAD_Y);
            if (road) {
                BoxT(TX_ASPHALT, x + 0.5f, GROUND_TOP * 0.5f, y + 0.5f, 1.0f, GROUND_TOP, 1.0f, WHITE);
                BoxC(x + 0.5f, GROUND_TOP + 0.006f, y + 0.5f, 0.5f, 0.012f, 0.045f, COL_LINE);
            } else BoxT(TX_GRASS, x + 0.5f, 0.015f, y + 0.5f, 1.0f, 0.03f, 1.0f, (Color){ 225, 240, 200, 255 });
        }
    // solar field: rows of tilted tables
    for (int y = FARM_SOLAR_Y0; y <= FARM_SOLAR_Y1; y++)
        for (int x = FARM_SOLAR_X0; x <= FARM_SOLAR_X1; x++)
            for (int i = 0; i < 2; i++)
                for (int k = 0; k < 2; k++) {
                    float px = x + 0.25f + i * 0.5f, pz = y + 0.27f + k * 0.46f;
                    BoxC(px, 0.045f, pz - 0.05f, 0.34f, 0.07f, 0.012f, A((Color){ 120, 124, 130, 255 }));        // rear support
                    SolarPanel(px, 0.085f, pz, 0.40f, 0.30f, 24.0f);
                }
    // grid hub: battery containers + transformer
    BoxT(TX_CONCRETE, 24.5f, 0.02f, 12.6f, 0.95f, 0.04f, 3.0f, WHITE);
    for (int k = 0; k < 3; k++) {
        BoxT(TX_CORRUG, 24.5f, 0.17f, 11.5f + k * 1.0f, 0.70f, 0.26f, 0.80f, A((Color){ 215, 240, 225, 255 }));
        for (int s = 0; s < 6; s++) BoxC(24.5f - 0.25f + s * 0.1f, 0.17f, 11.5f + k * 1.0f + 0.404f, 0.05f, 0.12f, 0.004f, A((Color){ 60, 215, 120, 255 }));
    }
    GCylinder(V3(25.1f, 0.04f, 12.0f), 0.07f, 0.07f, 0.22f, 10, A((Color){ 170, 175, 182, 255 }));
    // perimeter fence (north, south, east)
    Color fc = A((Color){ 150, 155, 160, 255 });
    for (int x = MAP_W; x <= MAP_W + FARM_W; x++) { BoxC((float)x, 0.06f, 0.0f, 0.012f, 0.12f, 0.012f, fc); BoxC((float)x, 0.06f, (float)MAP_H, 0.012f, 0.12f, 0.012f, fc); }
    for (int y = 0; y <= MAP_H; y++) BoxC((float)(MAP_W + FARM_W), 0.06f, (float)y, 0.012f, 0.12f, 0.012f, fc);
    BoxC(MAP_W + FARM_W * 0.5f, 0.10f, 0.0f, (float)FARM_W, 0.008f, 0.008f, fc);
    BoxC(MAP_W + FARM_W * 0.5f, 0.10f, (float)MAP_H, (float)FARM_W, 0.008f, 0.008f, fc);
    BoxC((float)(MAP_W + FARM_W), 0.10f, MAP_H * 0.5f, 0.008f, 0.008f, (float)MAP_H, fc);
    // power line along the road back to the city
    for (int x = MAP_W; x < MAP_W + FARM_W; x += 2) {
        BoxC(x + 0.5f, 0.20f, FARM_ROAD_Y + 0.05f, 0.012f, 0.40f, 0.012f, A((Color){ 105, 90, 70, 255 }));
        BoxC(x + 0.5f, 0.39f, FARM_ROAD_Y + 0.05f, 0.012f, 0.012f, 0.14f, A((Color){ 105, 90, 70, 255 }));
    }
    BoxC(MAP_W + FARM_W * 0.5f - 0.5f, 0.39f, FARM_ROAD_Y + 0.05f, (float)FARM_W, 0.004f, 0.004f, A((Color){ 30, 30, 34, 255 }));
}

// the turbines turn with the wind (live)
static void DrawFarmLive3D(void) {
    float t = (float)GetTime();
    for (int i = 0; i < N_TURB; i++) DrawTurbine3D(TURB_POS[i][0], TURB_POS[i][1], t);
}// ---------------------------------------------------------------------------
// 3D drawing - smart city extras
// ---------------------------------------------------------------------------
static void GreenRoof3D(float y, float cx, float cz, float w, float d) {       // sedum blanket (local frame of the building)
    BoxT(TX_HEDGE, cx, y, cz, w, 0.014f, d, A(WHITE));
    static const Color fl[3] = { {245,120,160,255}, {250,225,90,255}, {235,235,245,255} };
    for (int k = 0; k < 7; k++) {
        float fx = cx + ((k * 37) % 11 - 5) * w * 0.075f, fz = cz + ((k * 53) % 9 - 4) * d * 0.09f;
        GSphere((Vector3){ fx, y + 0.012f, fz }, 0.009f, A(fl[k % 3]));
    }
}

// a spot on the pavement beside road tile (x,y): returns the facing direction of the neighbouring non-road tile
static int SidePos(int x, int y, float out, float *px, float *pz) {
    for (int d = 0; d < 4; d++)
        if (InBounds(x + DX[d], y + DY[d]) && !IsRoad(x + DX[d], y + DY[d])) { *px = x + 0.5f + DX[d] * out; *pz = y + 0.5f + DY[d] * out; return d; }
    *px = x + 0.5f; *pz = y + 0.5f;
    return -1;
}

// ---- bins + refuse truck (both cities) ----------------------------------------------------
static void DrawBins3D(void) {
    float t = (float)GetTime();
    for (int i = 0; i < nBins; i++) {
        const Bin *b = &bins[i];
        float tx = (DY[b->side] != 0) ? 1.0f : 0.0f, tz = (DX[b->side] != 0) ? 1.0f : 0.0f;
        float px = b->x + 0.5f + DX[b->side] * 0.40f + tx * 0.20f, pz = b->y + 0.5f + DY[b->side] * 0.40f + tz * 0.20f;
        BoxC(px, 0.045f, pz, 0.055f, 0.09f, 0.055f, A(smartCity ? (Color){ 40, 115, 85, 255 } : (Color){ 95, 100, 106, 255 }));
        BoxC(px, 0.096f, pz, 0.066f, 0.014f, 0.066f, A((Color){ 45, 50, 56, 255 }));
        if (smartCity) {
            Color led = (b->fill >= 95.0f) ? (Color){ 255, 60, 50, 255 } : (b->fill >= 75.0f ? (Color){ 250, 165, 45, 255 } : (b->fill >= 50.0f ? (Color){ 250, 225, 70, 255 } : (Color){ 70, 235, 115, 255 }));
            GSphere((Vector3){ px, 0.118f, pz }, 0.013f, led);
            SolarPanel(px, 0.108f, pz - 0.018f, 0.04f, 0.022f, 20.0f);
        }
        if (b->fill >= 100.0f) {                                           // overflowing: litter around the bin
            for (int k = 0; k < 4; k++) BoxC(px + (k - 1.5f) * 0.03f, 0.108f + 0.02f * fabsf(sinf(t * 2.0f + k)), pz + 0.02f * (k % 2 ? 1 : -1), 0.022f, 0.016f, 0.018f, A((Color){ 215 - k * 20, 210, 190, 255 }));
        }
    }
}

static void DrawBinTruck3D(const Responder *r, double t) {
    PushFrame(r->x, ENT_Y, r->y, 90.0f - r->angle);
    DrawVehicleLocal(V_TRUCK, (Color){ 60, 150, 90, 255 }, false, NightNow());
    BoxC(-0.077f, 0.105f, -0.06f, 0.006f, 0.10f, 0.28f, A((Color){ 55, 150, 90, 255 }));
    BoxC( 0.077f, 0.105f, -0.06f, 0.006f, 0.10f, 0.28f, A((Color){ 55, 150, 90, 255 }));
    BoxC(0, 0.175f, -0.06f, 0.15f, 0.010f, 0.30f, A((Color){ 55, 150, 90, 255 }));
    bool f = ((int)(t * 4.0)) & 1;
    BoxC(0, 0.130f, 0.17f, 0.04f, 0.014f, 0.025f, f ? (Color){ 255, 190, 40, 255 } : (Color){ 110, 80, 10, 255 });
    PopFrame();
}

// ---- leaks (both cities) -------------------------------------------------------------------
static void DrawLeaks3D(void) {
    float t = (float)GetTime();
    for (int i = 0; i < MAX_LEAKS; i++) {
        const Leak *l = &leaks[i];
        if (!l->active) continue;
        float cx = l->x + 0.5f, cz = l->y + 0.5f, pl = 0.5f + 0.5f * sinf(t * 6.0f + i);
        GCylinder((Vector3){ cx, 0.072f, cz }, 0.20f + 0.05f * pl, 0.20f + 0.05f * pl, 0.004f, 18, (Color){ 70, 140, 235, 170 });
        for (int k = 0; k < 5; k++) {
            float a = t * 3.0f + k * 1.26f, h = 0.04f + 0.17f * fabsf(sinf(t * 4.0f + k * 0.9f));
            GSphere((Vector3){ cx + 0.08f * cosf(a), 0.07f + h, cz + 0.08f * sinf(a) }, 0.012f, (Color){ 190, 220, 255, 230 });
        }
        if (l->detected) {
            GCylinder((Vector3){ cx, 0.074f, cz }, 0.27f, 0.27f, 0.003f, 22, smartCity ? (Color){ 70, 230, 200, 140 } : (Color){ 245, 190, 50, 140 });
            BoxC(cx, 0.30f, cz, 0.020f, 0.07f, 0.020f, (Color){ 245, 190, 50, 255 });
            GSphere((Vector3){ cx, 0.22f, cz }, 0.014f, (Color){ 245, 190, 50, 255 });
        }
    }
}

// ---- drones in the new roles + hub pad ---------------------------------------------------------
static void DrawDeliveryMarks3D(void) {
    for (int k = 0; k < N_DROPS; k++) {
        const DropMark *m = &drops[k];
        if (m->t <= 0.0f) continue;
        float a = Clamp(m->t / 7.0f, 0.0f, 1.0f);
        BoxC(m->x, 0.035f, m->y, 0.05f, 0.04f, 0.05f, A(m->kind == D_PARCEL ? (Color){ 200, 150, 80, 255 } : (Color){ 245, 245, 248, 255 }));
        if (m->kind == D_MEDSUP) { BoxC(m->x, 0.058f, m->y, 0.03f, 0.004f, 0.010f, (Color){ 215, 35, 40, 255 }); BoxC(m->x, 0.058f, m->y, 0.010f, 0.004f, 0.03f, (Color){ 215, 35, 40, 255 }); }
        GCylinder((Vector3){ m->x, 0.02f, m->y }, 0.10f + (1.0f - a) * 0.12f, 0.10f + (1.0f - a) * 0.12f, 0.002f, 16, Fade((Color){ 120, 230, 160, 255 }, a * 0.5f));
    }
}

// ---- the smart-city world: everything that stands on the ground ---------------------------------
static void DrawPlusWorld3D(void) {
    float t = (float)GetTime();
    // ---- district heating: heat exchanger, insulated mains, flowing heat ----
    {
        float hx = HEATPLANT_X + 0.5f, hz = HEATPLANT_Y + 0.5f;
        BoxT(TX_CONCRETE, hx, 0.012f, hz, 0.96f, 0.024f, 0.96f, A(WHITE));
        BoxT(TX_CORRUG, hx - 0.10f, 0.13f, hz + 0.05f, 0.46f, 0.20f, 0.46f, A((Color){ 205, 210, 215, 255 }));
        GCylinder((Vector3){ hx + 0.30f, 0.024f, hz - 0.18f }, 0.11f, 0.11f, 0.30f, 14, A((Color){ 225, 120, 55, 255 }));
        GCylinder((Vector3){ hx + 0.30f, 0.324f, hz - 0.18f }, 0.0f, 0.11f, 0.05f, 14, A((Color){ 190, 90, 40, 255 }));
        GCylinder((Vector3){ hx + 0.30f, 0.024f, hz + 0.20f }, 0.07f, 0.07f, 0.22f, 12, A((Color){ 235, 190, 90, 255 }));
        BoxC(hx - 0.10f, 0.26f, hz + 0.05f, 0.40f, 0.012f, 0.40f, A((Color){ 120, 128, 134, 255 }));
        BoxC(hx, 0.075f, 10.04f + (hz - 10.04f) * 0.5f, 0.032f, 0.030f, hz - 10.04f, A((Color){ 225, 110, 45, 255 }));
        for (int k = 0; k < N_DH; k++) {
            float x0 = DH_SEG[k][0], y0 = DH_SEG[k][1], x1 = DH_SEG[k][2], y1 = DH_SEG[k][3];
            float len = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
            int n = (int)(len * 4.0f);
            for (int s = 0; s < n; s++) {
                float u0 = (float)s / n, u1 = (float)(s + 1) / n;
                float ax = x0 + (x1 - x0) * u0, ay = y0 + (y1 - y0) * u0, bx = x0 + (x1 - x0) * u1, by = y0 + (y1 - y0) * u1;
                float mx = (ax + bx) * 0.5f, my = (ay + by) * 0.5f;
                if (IsRoad((int)mx, (int)my)) continue;
                BoxC(mx, 0.072f, my, fabsf(bx - ax) + 0.034f, 0.028f, fabsf(by - ay) + 0.034f, A((Color){ 225, 110, 45, 255 }));
            }
            for (int d = 0; d < (int)(len / 1.4f) + 1; d++) {
                float u = fmodf(t * 0.9f + d * 1.4f, len) / len, px = x0 + (x1 - x0) * u, py = y0 + (y1 - y0) * u;
                if (!IsRoad((int)px, (int)py)) GSphere((Vector3){ px, 0.098f, py }, 0.015f, (Color){ 255, 225, 120, 255 });
            }
        }
    }
    // ---- drone fulfilment hub ----
    {
        float hx = HUB_X + 0.5f, hz = HUB_Y + 0.5f;
        BoxT(TX_CONCRETE, hx, 0.012f, hz, 0.96f, 0.024f, 0.96f, A(WHITE));
        DrawPad3D(hx, 0.024f, hz, D_PARCEL, !PadBusy(HUB_X, HUB_Y));
        for (int k = 0; k < 4; k++) BoxC(hx + 0.28f + (k % 2) * 0.09f, 0.05f + (k / 2) * 0.06f, hz + 0.26f, 0.08f, 0.055f, 0.08f, A((Color){ 200, 150, 80, 255 }));
    }
    // ---- shuttle shelters ----
    for (int i = 0; i < N_STOPS; i++) {
        float px, pz;
        int d = SidePos(SH_STOP[i][0], SH_STOP[i][1], 0.58f, &px, &pz);
        if (d < 0) continue;
        bool alongX = (DX[d] == 0);
        float w = alongX ? 0.22f : 0.08f, l = alongX ? 0.08f : 0.22f;
        BoxC(px, 0.125f, pz, w, 0.010f, l, A((Color){ 0, 185, 160, 255 }));
        for (int sx = -1; sx <= 1; sx += 2) BoxC(px + (alongX ? sx * 0.10f : 0), 0.06f, pz + (alongX ? 0 : sx * 0.10f), 0.008f, 0.12f, 0.008f, A((Color){ 70, 75, 82, 255 }));
        BoxC(px + (alongX ? 0 : DX[d] * 0.03f), 0.05f, pz + (alongX ? DY[d] * 0.03f : 0), alongX ? 0.14f : 0.012f, 0.018f, alongX ? 0.012f : 0.14f, A((Color){ 150, 110, 70, 255 }));
        BoxC(px, 0.15f, pz, 0.02f, 0.02f, 0.02f, A((Color){ 130, 255, 225, 255 }));
    }
    // ---- car park: bay sensors, EV chargers, guidance signs ----
    Color chargeCol = (v2gKW > 20.0f) ? (Color){ 90, 170, 255, 255 } : (v2gKW < -20.0f ? (Color){ 70, 230, 120, 255 } : (Color){ 225, 228, 235, 255 });
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] != T_PARK) continue;
            for (int s = 0; s < 8; s++) {
                int col = s % 4, row = s / 4;
                BoxC(x + (col + 0.5f) * 0.25f, 0.068f, y + (row ? 0.55f : 0.45f), 0.022f, 0.004f, 0.022f, lot[y][x][s] ? (Color){ 235, 60, 55, 255 } : (Color){ 70, 235, 115, 255 });
            }
            if (((x + y) & 1) == 1) {
                BoxC(x + 0.5f, 0.10f, y + 0.5f, 0.032f, 0.07f, 0.026f, A((Color){ 45, 50, 58, 255 }));
                GSphere((Vector3){ x + 0.5f, 0.155f, y + 0.5f }, 0.016f, chargeCol);
                BoxC(x + 0.5f, 0.124f, y + 0.5f + 0.014f, 0.020f, 0.012f, 0.002f, (Color){ 30, 210, 120, 255 });
            }
        }
    for (int i = 0; i < 2; i++) {
        float sx = PSIGN[i][0] + 0.14f, sz = PSIGN[i][1] + 0.86f;
        BoxC(sx, 0.11f, sz, 0.012f, 0.22f, 0.012f, A((Color){ 70, 74, 80, 255 }));
        BoxC(sx, 0.245f, sz, 0.22f, 0.085f, 0.014f, A((Color){ 12, 14, 18, 255 }));
        int g = parkTotal ? (int)(10.0f * parkFree / parkTotal + 0.5f) : 0;
        for (int k = 0; k < 10; k++) BoxC(sx - 0.095f + k * 0.021f, 0.245f, sz + 0.009f, 0.015f, 0.050f, 0.004f, k < g ? (Color){ 70, 235, 115, 255 } : (Color){ 235, 60, 55, 255 });
    }
    // ---- air-quality sensor masts ----
    for (int i = 0; i < N_AQ; i++) {
        float px, pz;
        SidePos(AQ_POS[i][0], AQ_POS[i][1], 0.40f, &px, &pz);
        Color c = AqColor(aqLevel[i]);
        BoxC(px, 0.20f, pz, 0.012f, 0.40f, 0.012f, A((Color){ 85, 95, 108, 255 }));
        BoxC(px, 0.41f, pz, 0.05f, 0.03f, 0.05f, A((Color){ 30, 38, 52, 255 }));
        GSphere((Vector3){ px, 0.46f, pz }, 0.030f, c);
        GCylinder((Vector3){ px, 0.47f, pz }, 0.06f, 0.06f, 0.002f, 16, Fade(c, 0.35f + 0.2f * sinf(t * 3.0f + i)));
    }
    // ---- flood sensors + storm-drain gates ----
    for (int i = 0; i < N_FLOOD; i++) {
        float px, pz;
        int d = SidePos(FLOOD_POS[i][0], FLOOD_POS[i][1], 0.44f, &px, &pz);
        Color c = floodState[i] == 2 ? (Color){ 235, 55, 50, 255 } : (floodState[i] == 1 ? (Color){ 245, 190, 50, 255 } : (Color){ 70, 160, 255, 255 });
        BoxC(px, 0.09f, pz, 0.010f, 0.18f, 0.010f, A((Color){ 80, 90, 110, 255 }));
        GSphere((Vector3){ px, 0.20f, pz }, 0.020f, c);
        if (d >= 0) {
            float gx = FLOOD_POS[i][0] + 0.5f + DX[d] * 0.40f, gz = FLOOD_POS[i][1] + 0.5f + DY[d] * 0.40f;
            BoxC(gx, GROUND_TOP + 0.006f, gz, DX[d] ? 0.02f : 0.26f, 0.006f, DY[d] ? 0.02f : 0.26f, A((Color){ 40, 44, 50, 255 }));
            if (drainOpen > 0.1f) BoxC(gx, GROUND_TOP + 0.011f, gz, DX[d] ? 0.02f : 0.22f, 0.004f, DY[d] ? 0.02f : 0.22f, Fade((Color){ 90, 170, 255, 255 }, 0.5f + 0.4f * sinf(t * 6.0f + i)));
        }
    }
}

// rain tanks + stormwater retention basin in the farm strip
static void DrawPlusFarm3D(void) {
    float f = Clamp(rainTankKL / RAIN_TANK_CAP, 0.0f, 1.0f);
    for (int i = 0; i < 2; i++) {
        float tx = 24.8f + i * 1.15f, tz = 8.5f;
        GCylinder((Vector3){ tx, 0.03f, tz }, 0.31f, 0.31f, 0.36f, 20, A((Color){ 205, 212, 218, 255 }));
        GCylinder((Vector3){ tx, 0.39f, tz }, 0.0f, 0.31f, 0.06f, 20, A((Color){ 150, 158, 166, 255 }));
        GCylinder((Vector3){ tx, 0.04f, tz }, 0.318f, 0.318f, 0.30f * f + 0.002f, 20, A((Color){ 70, 150, 245, 255 }));
        BoxC(tx + 0.36f, 0.20f, tz, 0.018f, 0.34f, 0.018f, A((Color){ 40, 44, 50, 255 }));
        BoxC(tx + 0.36f, 0.04f + 0.15f * f, tz, 0.026f, 0.30f * f + 0.004f, 0.026f, A((Color){ 90, 180, 255, 255 }));
    }
    float rf = Clamp(retentionKL / RETENTION_CAP, 0.0f, 1.0f);
    BoxC(28.25f, 0.016f, 8.5f, 3.10f, 0.032f, 0.82f, A((Color){ 120, 100, 70, 255 }));
    BoxC(28.25f, 0.030f + 0.02f * rf, 8.5f, 2.96f, 0.012f + 0.04f * rf, 0.68f, A((Color){ 60 + (int)(50 * rf), 125 + (int)(60 * rf), 215, 255 }));
}

// ---------------------------------------------------------------------------
// First-person mode: drawing (parked / driven cars with opening doors, cockpit, camera, HUD)
// ---------------------------------------------------------------------------
static void DrawUCarDoor(const UCar *u, int door, bool inside) {
    float W = VWID[u->type], zc = UC_CABIN[u->type][0], cl = UC_CABIN[u->type][1];
    if (u->type == V_VAN) { zc = 0.02f; cl = 0.16f; }
    float sgn = (door == 0) ? -1.0f : 1.0f, ang = u->door[door], dl = cl * 0.86f;
    Color body = A(CAR_COLS[u->colIdx]), glass = A(GLASS), seam = A((Color){ 25, 25, 28, 255 });
    float hz = zc + dl * 0.5f;
    if (ang > 1.0f) {                                                   // the opening shows the dark interior
        BoxC(sgn * (W * 0.5f + 0.0012f), 0.050f, hz - dl * 0.5f, 0.002f, 0.044f, dl, A((Color){ 34, 34, 40, 255 }));
        BoxC(sgn * (W * 0.43f + 0.0006f), 0.090f, hz - dl * 0.5f, 0.0015f, 0.036f, dl * 0.94f, A((Color){ 52, 52, 60, 255 }));
    }
    GXPush();
    GXTrans(sgn * W * 0.5f, 0.0f, hz);
    GXRot((door == 0) ? ang : -ang, 0, 1, 0);
    BoxC(sgn * 0.004f, 0.046f, -dl * 0.5f, 0.008f, 0.038f, dl, body);                                // lower door panel
    if (!inside) BoxC(sgn * (-W * 0.07f + 0.004f), 0.088f, -dl * 0.5f, 0.008f, 0.038f, dl * 0.94f, glass);   // window (the driver looks through it)
    BoxC(sgn * 0.0085f, 0.046f, -dl, 0.003f, 0.040f, 0.003f, seam);                                  // shut line
    BoxC(sgn * 0.0095f, 0.056f, -dl * 0.82f, 0.003f, 0.005f, 0.018f, A((Color){ 200, 200, 205, 255 }));   // handle
    GXPop();
}

static void DrawUCarSeats(const UCar *u, bool skipDriver) {
    float W = VWID[u->type], zc = UC_CABIN[u->type][0];
    if (u->type == V_VAN) zc = 0.02f;
    Color seat = A((Color){ 48, 48, 56, 255 });
    for (int s = -1; s <= 1; s += 2) {
        if (skipDriver && s < 0) continue;
        BoxC(s * W * 0.22f, 0.070f, zc - 0.012f, W * 0.30f, 0.014f, 0.040f, seat);
        BoxC(s * W * 0.22f, 0.088f, zc - 0.034f, W * 0.30f, 0.034f, 0.010f, seat);
    }
}

// what the driver sees: windscreen pillars, roof lining, dashboard, steering wheel, mirror
static void DrawUCarCockpit(const UCar *u) {
    float W = VWID[u->type], zc = UC_CABIN[u->type][0], cl = UC_CABIN[u->type][1], R = UC_CABIN[u->type][2];
    float zf = zc + cl * 0.5f;
    Color pil = A(Shade(CAR_COLS[u->colIdx], 0.55f)), dark = A((Color){ 26, 26, 30, 255 });
    float mid = (0.068f + R) * 0.5f, ph = R - 0.068f;
    for (int s = -1; s <= 1; s += 2) {
        BoxC(s * W * 0.43f, mid, zf - 0.005f, 0.007f, ph, 0.012f, pil);                      // A pillars
        BoxC(s * W * 0.43f, mid, zc - cl * 0.12f, 0.006f, ph, 0.010f, pil);                  // B pillars
        BoxC(s * W * 0.43f, mid, zc - cl * 0.5f + 0.008f, 0.007f, ph, 0.016f, pil);          // C pillars
    }
    BoxC(0, R - 0.003f, zf - 0.004f, W * 0.86f, 0.006f, 0.010f, pil);                         // windscreen header
    BoxC(0, R - 0.0050f, zc, W * 0.84f, 0.002f, cl, A((Color){ 70, 70, 76, 255 }));           // roof lining
    BoxC(0, 0.060f, zf - 0.020f, W * 0.90f, 0.016f, 0.034f, dark);                            // dashboard
    // steering wheel (turns with the steering angle)
    float wz = zf - 0.036f;
    GXPush();
    GXTrans(-W * 0.22f, 0.066f, wz);
    GXRot(24.0f, 1, 0, 0);
    GXRot(-u->steer * 130.0f, 0, 0, 1);
    for (int k = 0; k < 14; k++) {
        float a = k * (2.0f * PI / 14.0f);
        BoxC(cosf(a) * 0.0125f, sinf(a) * 0.0125f, 0, 0.0055f, 0.0055f, 0.0055f, dark);
    }
    BoxC(0, 0, 0, 0.025f, 0.0035f, 0.0035f, dark);
    BoxC(0, -0.006f, 0, 0.0035f, 0.012f, 0.0035f, dark);
    GXPop();
}

static void DrawUCar3D(const UCar *u, bool inside) {
    PushFrame(u->x, u->gy, u->z, u->yaw);
    gHideGlass = inside;
    DrawVehicleLocal(u->type, CAR_COLS[u->colIdx], u->braking, NightNow());
    gHideGlass = false;
    DrawUCarSeats(u, inside);
    DrawUCarDoor(u, 0, inside);
    DrawUCarDoor(u, 1, inside);
    if (inside) DrawUCarCockpit(u);
    PopFrame();
}

static void DrawUserCars3D(void) {
    for (int i = 0; i < MAX_UCARS; i++) {
        if (!ucars[i].active) continue;
        bool inside = FpActive() && fp.car == i && fp.state == FP_DRIVE;
        DrawUCar3D(&ucars[i], inside);
    }
}

static Camera3D FpCamera(void) {
    Camera3D c = { 0 };
    float cp = cosf(fp.cpitch), sp = sinf(fp.cpitch);
    c.position = (Vector3){ fp.cx, fp.cy, fp.cz };
    c.target = (Vector3){ fp.cx + sinf(fp.cyaw) * cp, fp.cy + sp, fp.cz + cosf(fp.cyaw) * cp };
    c.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    c.fovy = (fp.state == FP_DRIVE) ? 66.0f + fminf(8.0f, fabsf(ucars[fp.car].v) * 2.2f) : 66.0f;
    c.projection = CAMERA_PERSPECTIVE;
    return c;
}

static void HudPill(int cx, int y, const char *s, int size, Color c) {
    int w = MeasureText(s, size);
    DrawRectangle(cx - w / 2 - 8, y - 4, w + 16, size + 8, Fade(BLACK, 0.55f));
    DrawText(s, cx - w / 2, y, size, c);
}

static void DrawFpHud(void) {
    int cx = VIEW_W / 2, cy = VIEW_H / 2;
    if (fp.bump > 0.0f) DrawRectangle(0, 0, VIEW_W, VIEW_H, Fade((Color){ 255, 60, 40, 255 }, fp.bump * 0.45f));
    if (fp.state == FP_WALK) {
        DrawCircle(cx, cy, 2.0f, Fade(WHITE, 0.85f));
        DrawCircleLines(cx, cy, 5.0f, Fade(WHITE, 0.35f));
    }
    if (fp.prompt[0]) HudPill(cx, (int)(VIEW_H * 0.66f), fp.prompt, 14, (Color){ 255, 235, 150, 255 });

    if (fp.state == FP_DRIVE) {
        const UCar *u = &ucars[fp.car];
        float kmh = fabsf(u->v) * 36.0f;
        int bx = VIEW_W - 150, by = VIEW_H - 74;
        DrawRectangle(bx, by, 138, 62, Fade(BLACK, 0.55f));
        DrawText(TextFormat("%.0f", kmh), bx + 10, by + 6, 34, (kmh > 100.0f) ? (Color){ 255, 120, 90, 255 } : RAYWHITE);
        DrawText("km/h", bx + 76, by + 26, 10, GRAY);
        DrawText(u->v < -0.05f ? "R" : (fabsf(u->v) < 0.05f ? "P" : "D"), bx + 108, by + 8, 20, (Color){ 120, 220, 255, 255 });
        DrawRectangle(bx + 10, by + 48, 118, 5, (Color){ 45, 50, 60, 255 });
        DrawRectangle(bx + 10, by + 48, (int)(118.0f * Clamp(kmh / 120.0f, 0.0f, 1.0f)), 5, (kmh > 100.0f) ? (Color){ 240, 110, 80, 255 } : (Color){ 90, 220, 140, 255 });
        if (!FpOnRoad(u->x, u->z)) DrawText("off road", bx + 76, by + 40 - 12, 10, (Color){ 240, 190, 90, 255 });
    }
    int lines = (fp.hint > 0.0f) ? 4 : 1;
    float al = (fp.hint > 0.0f) ? Clamp(fp.hint / 2.0f, 0.25f, 1.0f) : 0.45f;
    DrawRectangle(6, 6, 300, 8 + lines * 12, Fade(BLACK, 0.45f * al + 0.1f));
    DrawText("FIRST PERSON   [F] back to city view", 12, 10, 10, Fade((Color){ 130, 240, 170, 255 }, al + 0.3f));
    if (fp.hint > 0.0f) {
        if (fp.state == FP_DRIVE || fp.state == FP_ENTER) {
            DrawText("W/S gas + brake/reverse   A/D steer", 12, 22, 10, Fade(LIGHTGRAY, al));
            DrawText("Space handbrake   Mouse look around", 12, 34, 10, Fade(LIGHTGRAY, al));
            DrawText("E get out (when stopped)", 12, 46, 10, Fade(LIGHTGRAY, al));
        } else {
            DrawText("WASD walk   Shift sprint   Space jump", 12, 22, 10, Fade(LIGHTGRAY, al));
            DrawText("Mouse look   E open a car door / get in", 12, 34, 10, Fade(LIGHTGRAY, al));
            DrawText("T smart city   Tab data   P pause", 12, 46, 10, Fade(LIGHTGRAY, al));
        }
    }
}

// ---- orbit camera + picking + pipe networks + weather overlay (unchanged) ----
static Camera3D BuildCamera(void) {
    Camera3D c = { 0 };
    float cp = cosf(oc.pitch), sp = sinf(oc.pitch);
    c.position = (Vector3){ oc.target.x + oc.dist * cp * sinf(oc.yaw),
                            oc.target.y + oc.dist * sp,
                            oc.target.z + oc.dist * cp * cosf(oc.yaw) };
    c.target = oc.target;
    c.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    c.fovy = 45.0f;
    c.projection = CAMERA_PERSPECTIVE;
    return c;
}

static void ResetCamera(void) {
    oc.target = (Vector3){ (MAP_W + (smartCity ? FARM_W : 0)) / 2.0f, 0.0f, MAP_H / 2.0f };
    oc.yaw = 0.0f; oc.pitch = 1.0f; oc.dist = smartCity ? 29.0f : 24.0f;
}

static void UpdateOrbit(float dt) {
    float spd = oc.dist * 0.7f * dt * (IsKeyDown(KEY_LEFT_SHIFT) ? 2.0f : 1.0f);
    float fx = -sinf(oc.yaw), fz = -cosf(oc.yaw);     // forward on the ground plane
    float rx = -fz, rz = fx;                          // right
    if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    { oc.target.x += fx * spd; oc.target.z += fz * spd; }
    if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  { oc.target.x -= fx * spd; oc.target.z -= fz * spd; }
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) { oc.target.x += rx * spd; oc.target.z += rz * spd; }
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  { oc.target.x -= rx * spd; oc.target.z -= rz * spd; }
    if (IsKeyDown(KEY_Q)) oc.yaw += 1.6f * dt;
    if (IsKeyDown(KEY_E)) oc.yaw -= 1.6f * dt;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 d = GetMouseDelta();
        oc.yaw -= d.x * ((float)SCREEN_W / GetScreenWidth()) * 0.005f;
        oc.pitch += d.y * ((float)SCREEN_H / GetScreenHeight()) * 0.005f;
    }
    float w = GetMouseWheelMove();
    if (w != 0.0f) oc.dist -= w * 2.0f;
    oc.pitch = Clamp(oc.pitch, 0.25f, 1.5f);
    oc.dist = Clamp(oc.dist, 4.0f, 46.0f);
    oc.target.x = Clamp(oc.target.x, -3.0f, MAP_W + FARM_W + 3.0f);
    oc.target.z = Clamp(oc.target.z, -3.0f, MAP_H + 3.0f);
}

// Cast a ray from the mouse (relative to the map view) onto the ground plane
static bool PickTile3D(Vector2 m, int *tx, int *ty) {
    Camera3D c = BuildCamera();
    Matrix proj = MatrixPerspective(c.fovy * DEG2RAD, (double)VIEW_W / (double)VIEW_H, 0.01, 1000.0);
    Matrix view = GetCameraMatrix(c);
    float nx = 2.0f * m.x / VIEW_W - 1.0f, ny = 1.0f - 2.0f * m.y / VIEW_H;
    Vector3 farPt = Vector3Unproject((Vector3){ nx, ny, 1.0f }, proj, view);
    Vector3 dir = Vector3Normalize(Vector3Subtract(farPt, c.position));
    if (fabsf(dir.y) < 1e-5f) return false;
    float t = -c.position.y / dir.y;
    if (t < 0.0f) return false;
    *tx = (int)floorf(c.position.x + dir.x * t);
    *ty = (int)floorf(c.position.z + dir.z * t);
    return true;
}

static void DrawNetwork3D(float off, float yh, Color col) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!pipeNode[y][x]) continue;
            float ax = x + 0.5f + off, az = y + 0.5f + off;
            float s = (surface[y][x] == T_ROAD) ? 0.09f : 0.17f;
            GCube((Vector3){ ax, yh, az }, s, s, s, col);
            if (InBounds(x + 1, y) && pipeNode[y][x + 1] && (IsRoad(x, y) || IsRoad(x + 1, y)))
                GCube((Vector3){ ax + 0.5f, yh, az }, 1.0f, 0.06f, 0.06f, col);
            if (InBounds(x, y + 1) && pipeNode[y + 1][x] && (IsRoad(x, y) || IsRoad(x, y + 1)))
                GCube((Vector3){ ax, yh, az + 0.5f }, 0.06f, 0.06f, 1.0f, col);
        }
}

// Night shading + rain streaks (drawn in screen space on top of either view)
static void DrawWeatherFx(void) {
    float dark = (1.0f - st.daylight) * 0.32f + rain * 0.15f;
    if (dark > 0.01f) DrawRectangle(0, 0, VIEW_W, VIEW_H, Fade((Color){ 10, 15, 40, 255 }, dark));
    int n = (int)(rain * 260.0f);
    for (int i = 0; i < n; i++) {
        int x = GetRandomValue(0, VIEW_W), y = GetRandomValue(0, VIEW_H);
        DrawLine(x, y, x - 3, y + 9, (Color){ 180, 200, 255, 140 });
    }
}

// ---------------------------------------------------------------------------
// 3D drawing - people, emergency vehicles, particles, landfill life
// ---------------------------------------------------------------------------
// pedestrian, modelled in metres (1 m = 1/17 tile; see the cyclist model for M_SC)
static void DrawPed3D(const Ped *p) {
    float yaw = 0.0f;
    bool walking = (p->wi < p->wn) && !(p->crossing && p->wi == 1 && !p->started) && !p->robbing && !p->ill;
    if (p->wi < p->wn) yaw = atan2f(p->wx[p->wi] - p->x, p->wy[p->wi] - p->y) * RAD2DEG;
    float sw = walking ? sinf(p->phase) * 28.0f : 0.0f;
    Color shirt = A(p->shirt), pants = A(p->pants), skin = A((Color){ 235, 190, 160, 255 }), shoe = A((Color){ 30, 28, 30, 255 });
    static const Color hair[4] = { {40,30,25,255}, {95,65,40,255}, {200,170,90,255}, {150,150,155,255} };
    Color hc = A(hair[p->look & 3]);
    PushFrame(p->x, ENT_Y, p->y, yaw);
    if (p->ill) { GXTrans(0.0f, 0.020f, 0.0f); GXRot(-86.0f, 1, 0, 0); }          // collapsed on the pavement
    for (int s = -1; s <= 1; s += 2) {                                             // legs swing from the hip
        GXPush(); GXTrans(s * 0.09f * M_SC, 0.90f * M_SC, 0.0f); GXRot(s * sw, 1, 0, 0);
        MBox(0, -0.42f, 0, 0.13f, 0.84f, 0.14f, pants);
        MBox(0, -0.86f, 0.03f, 0.12f, 0.08f, 0.24f, shoe);
        GXPop();
    }
    for (int s = -1; s <= 1; s += 2) {                                             // arms swing from the shoulder
        GXPush(); GXTrans(s * 0.23f * M_SC, 1.42f * M_SC, 0.0f); GXRot(-s * sw, 1, 0, 0);
        MBox(0, -0.27f, 0, 0.09f, 0.54f, 0.10f, shirt);
        MBox(0, -0.60f, 0, 0.08f, 0.12f, 0.09f, skin);
        GXPop();
    }
    MBox(0, 0.92f, 0, 0.30f, 0.12f, 0.20f, pants);                                 // hips
    MBox(0, 1.18f, 0, 0.34f, 0.54f, 0.22f, shirt);                                 // torso
    MBox(0, 1.50f, 0, 0.10f, 0.08f, 0.10f, skin);                                  // neck
    GSphere(MV(0, 1.60f, 0), 0.11f * M_SC, skin);                                  // head
    GSphere(MV(0, 1.63f, -0.02f), 0.118f * M_SC, hc);                              // hair (top and back)
    if ((p->look >> 2) & 1) { MBox(0, 1.71f, -0.005f, 0.23f, 0.04f, 0.24f, shirt); MBox(0, 1.70f, 0.13f, 0.20f, 0.02f, 0.08f, shirt); }   // cap + peak
    if (p->criminal) MBox(0, 1.60f, 0.105f, 0.13f, 0.06f, 0.02f, A((Color){ 20, 20, 20, 255 }));   // mask
    if (p->robbing) MBox(0, 1.98f + 0.03f * sinf((float)GetTime() * 10.0f), 0, 0.10f, 0.22f, 0.10f, RED);
    PopFrame();
    if (p->ill) {                                                                      // medical-emergency marker
        float by = 0.20f + 0.015f * sinf((float)GetTime() * 6.0f);
        BoxC(p->x, by, p->y, 0.050f, 0.050f, 0.012f, A(WHITE));
        BoxC(p->x, by, p->y, 0.034f, 0.010f, 0.014f, A((Color){ 215, 35, 40, 255 }));
        BoxC(p->x, by, p->y, 0.010f, 0.034f, 0.014f, A((Color){ 215, 35, 40, 255 }));
    }
}
static void DrawPoliceCar3D(const Responder *r, double t) {
    PushFrame(r->x, ENT_Y, r->y, 90.0f - r->angle);
    Color w = A(WHITE);
    BoxT(TX_POLICE, 0, 0.046f, 0, 0.115f, 0.040f, 0.27f, w);
    BoxC(0, 0.088f, -0.015f, 0.100f, 0.040f, 0.13f, A(GLASS));
    BoxT(TX_POLICE, 0, 0.110f, -0.015f, 0.098f, 0.006f, 0.115f, w);
    bool f = ((int)(t * 7.0)) % 2 == 0;
    BoxC(-0.026f, 0.121f, -0.015f, 0.040f, 0.014f, 0.024f, f ? (Color){ 255, 40, 40, 255 } : (Color){ 90, 15, 15, 255 });
    BoxC( 0.026f, 0.121f, -0.015f, 0.040f, 0.014f, 0.024f, f ? (Color){ 20, 40, 110, 255 } : (Color){ 80, 170, 255, 255 });
    Wheels(0.115f, 0.27f, 2);
    BoxC(-0.036f, 0.05f, 0.1355f, 0.022f, 0.012f, 0.004f, (Color){ 255, 248, 200, 255 });
    BoxC( 0.036f, 0.05f, 0.1355f, 0.022f, 0.012f, 0.004f, (Color){ 255, 248, 200, 255 });
    BoxC(-0.036f, 0.05f, -0.1355f, 0.022f, 0.012f, 0.004f, (Color){ 200, 30, 30, 255 });
    BoxC( 0.036f, 0.05f, -0.1355f, 0.022f, 0.012f, 0.004f, (Color){ 200, 30, 30, 255 });
    PopFrame();
}

static void DrawFireTruck3D(const Responder *r, double t) {
    PushFrame(r->x, ENT_Y, r->y, 90.0f - r->angle);
    Color w = A(WHITE);
    BoxT(TX_FIRE, 0, 0.070f, -0.040f, 0.135f, 0.100f, 0.36f, w);
    BoxT(TX_FIRE, 0, 0.078f, 0.175f, 0.135f, 0.115f, 0.10f, w);
    BoxC(0, 0.100f, 0.226f, 0.120f, 0.045f, 0.008f, A(GLASS));
    BoxC(0, 0.135f, -0.04f, 0.050f, 0.008f, 0.30f, A((Color){ 235, 235, 238, 255 }));
    for (int s = -1; s <= 1; s += 2) {                                    // ladder
        BoxC(s * 0.026f, 0.143f, -0.05f, 0.007f, 0.007f, 0.34f, A((Color){ 190, 192, 196, 255 }));
    }
    for (int k = 0; k < 7; k++) BoxC(0, 0.143f, -0.20f + k * 0.045f, 0.052f, 0.005f, 0.005f, A((Color){ 190, 192, 196, 255 }));
    bool f = ((int)(t * 7.0)) % 2 == 0;
    BoxC(-0.03f, 0.147f, 0.175f, 0.05f, 0.012f, 0.02f, f ? (Color){ 255, 40, 40, 255 } : (Color){ 110, 20, 20, 255 });
    BoxC( 0.03f, 0.147f, 0.175f, 0.05f, 0.012f, 0.02f, f ? (Color){ 110, 20, 20, 255 } : (Color){ 255, 70, 50, 255 });
    Wheels(0.135f, 0.40f, 3);
    PopFrame();
    if (r->spraying) {                                                    // water jet
        Vector3 a = { r->x, 0.15f, r->y }, b = { r->sx, 0.12f + 0.08f * sinf((float)t * 12.0f), r->sy };
        GCylinderEx(a, b, 0.012f, 0.03f, 6, (Color){ 140, 200, 255, 170 });
        GSphere(b, 0.07f, (Color){ 200, 230, 255, 120 });
    }
}

static void DrawLandfillLife3D(void) {
    float t = (float)GetTime();
    for (int k = 0; k < 2; k++) {               // bulldozers shoving rubbish
        float a = t * (0.22f + 0.05f * k) + k * 3.0f;
        float R = 0.22f - 0.07f * k;            // tight loops: the machines stay on the tip, inside the fence
        float px = 21.5f + cosf(a) * R, pz = 11.5f + sinf(a * 1.3f) * R * 0.5f;
        float dx = -sinf(a) * R, dz = cosf(a * 1.3f) * 1.3f * R * 0.5f;
        PushFrame(px, 0.05f, pz, atan2f(dx, dz) * RAD2DEG);
        BoxC(0, 0.05f, 0, 0.12f, 0.05f, 0.20f, A((Color){ 235, 190, 30, 255 }));
        BoxC(0, 0.10f, -0.03f, 0.09f, 0.05f, 0.08f, A(GLASS));
        BoxC(0, 0.09f, 0.14f, 0.15f, 0.07f, 0.015f, A((Color){ 90, 90, 95, 255 }));
        BoxC(-0.065f, 0.02f, 0, 0.025f, 0.04f, 0.22f, A((Color){ 30, 30, 32, 255 }));
        BoxC( 0.065f, 0.02f, 0, 0.025f, 0.04f, 0.22f, A((Color){ 30, 30, 32, 255 }));
        PopFrame();
    }
    for (int k = 0; k < 7; k++) {               // gulls wheeling over the tip
        float a = t * (0.5f + 0.07f * k) + k * 1.9f, r = 0.8f + 0.18f * k;
        float px = 21.5f + cosf(a) * r, pz = 11.5f + sinf(a) * r * 0.6f, py = 0.55f + 0.1f * sinf(a * 3.0f);
        float flap = sinf(t * 14.0f + k) * 25.0f;
        PushFrame(px, py, pz, atan2f(-sinf(a), cosf(a) * 0.6f) * RAD2DEG);
        BoxC(0, 0, 0, 0.025f, 0.015f, 0.05f, (Color){ 245, 245, 245, 255 });
        for (int s = -1; s <= 1; s += 2) {
            GXPush(); GXTrans(s * 0.012f, 0, 0); GXRot(s * flap, 0, 0, 1);
            BoxC(s * 0.03f, 0, 0, 0.06f, 0.004f, 0.025f, (Color){ 235, 235, 240, 255 });
            GXPop();
        }
        PopFrame();
    }
}

static void DrawParticles3D(void) {
    for (int i = 0; i < MAX_PART; i++) {
        const Particle *q = &parts[i];
        if (!q->on) continue;
        float k = q->life / q->max;
        if (q->kind == 0) {
            float s = 0.05f + 0.07f * k;
            Color c = LerpColor((Color){ 255, 235, 110, 255 }, (Color){ 235, 70, 25, 255 }, 1.0f - k);
            c.a = (unsigned char)(70 + 150 * k);
            BoxC(q->p.x, q->p.y, q->p.z, s, s * 1.3f, s, c);
        } else if (q->kind == 1) {
            float s = 0.07f + 0.20f * (1.0f - k);
            BoxC(q->p.x, q->p.y, q->p.z, s, s, s, (Color){ 62, 62, 66, (unsigned char)(120 * k) });
        } else {
            BoxC(q->p.x, q->p.y, q->p.z, 0.014f, 0.014f, 0.014f, (Color){ 255, 190, 70, (unsigned char)(255 * k) });
        }
    }
}

// ---------------------------------------------------------------------------
// 3D scene
// ---------------------------------------------------------------------------
// street-lamp posts (static); the glow is drawn by DrawStreetLampGlow (live)
static void DrawStreetLamp(int x, int y) {
    int d = RoadSide(x, y);
    if (!IsRoad(x + DX[d], y + DY[d])) return;
    float px = x + 0.5f + DX[d] * 0.40f, pz = y + 0.5f + DY[d] * 0.40f;
    BoxC(px, 0.14f, pz, 0.010f, 0.28f, 0.010f, A((Color){ 70, 72, 78, 255 }));
    BoxC(px - DX[d] * 0.03f, 0.285f, pz - DY[d] * 0.03f, DX[d] ? 0.07f : 0.012f, 0.012f, DY[d] ? 0.07f : 0.012f, A((Color){ 70, 72, 78, 255 }));
}

// the lamp glow and its light cone (live; smart lamps dim to ~30% until somebody comes past)
static void DrawStreetLampGlow(int x, int y) {
    int d = RoadSide(x, y);
    if (!IsRoad(x + DX[d], y + DY[d]) || !NightNow()) return;
    float px = x + 0.5f + DX[d] * 0.40f, pz = y + 0.5f + DY[d] * 0.40f;
    float lv = lampLvl[y][x];
    Vector3 lp = { px - DX[d] * 0.06f, 0.275f, pz - DY[d] * 0.06f };
    GSphere(lp, 0.009f + 0.009f * lv, (Color){ (unsigned char)(120 + 135 * lv), (unsigned char)(110 + 130 * lv), (unsigned char)(70 + 100 * lv), 255 });
    if (lv > 0.6f) GCylinderEx(lp, (Vector3){ lp.x, 0.07f, lp.z }, 0.005f, 0.10f, 8, (Color){ 255, 238, 170, 22 });
}
static void DrawCongestion3D(void) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (!IsRoad(x, y)) continue;
            float d = traffic[y][x];
            if (d >= 0.04f) BoxC(x + 0.5f, 0.072f, y + 0.5f, 0.15f, 0.005f, 0.15f, CongColor(d, 0.25f + d * 0.35f));
            for (int k = 0; k < 2; k++) {
                int nx = x + (k == 0), ny = y + (k == 1);
                if (!IsRoad(nx, ny)) continue;
                float dm = 0.5f * (d + traffic[ny][nx]);
                if (dm < 0.04f) continue;
                Color c = CongColor(dm, 0.25f + dm * 0.35f);
                if (k == 0) BoxC(x + 1.0f, 0.072f, y + 0.5f, 1.0f, 0.005f, 0.15f, c);
                else        BoxC(x + 0.5f, 0.072f, y + 1.0f, 0.15f, 0.005f, 1.0f, c);
            }
        }
}

static void DrawGround3D(int x, int y, bool xray) {
    float cx = x + 0.5f, cz = y + 0.5f;
    int s = surface[y][x];
    if (s == T_ROAD) {
        if (!layerOn[L_ROADS]) return;
        Color rt = xray ? (Color){ 150, 150, 150, 255 } : WHITE;
        float rx0 = (InBounds(x - 1, y) && !IsRoad(x - 1, y)) ? ROAD_INSET : 0.0f, rx1 = (InBounds(x + 1, y) && !IsRoad(x + 1, y)) ? 1.0f - ROAD_INSET : 1.0f;
        float rz0 = (InBounds(x, y - 1) && !IsRoad(x, y - 1)) ? ROAD_INSET : 0.0f, rz1 = (InBounds(x, y + 1) && !IsRoad(x, y + 1)) ? 1.0f - ROAD_INSET : 1.0f;
        BoxT(TX_ASPHALT, x + (rx0 + rx1) * 0.5f, GROUND_TOP * 0.5f, y + (rz0 + rz1) * 0.5f, rx1 - rx0, GROUND_TOP, rz1 - rz0, rt);
        int ln = RoadLine(x, y);
        Color lc = xray ? (Color){ 150, 150, 150, 255 } : COL_LINE;
        if (ln == 1) BoxC(cx, GROUND_TOP + 0.006f, cz, 0.5f, 0.012f, 0.045f, lc);
        else if (ln == 2) BoxC(cx, GROUND_TOP + 0.006f, cz, 0.045f, 0.012f, 0.5f, lc);
        DrawRoadMarkings3D(x, y);
        return;
    }
    if (xray)                BoxC(cx, GROUND_TOP * 0.5f, cz, 1.0f, GROUND_TOP, 1.0f, COL_SOIL);
    else if (s == T_PARK)    BoxT(TX_ASPHALT, cx, GROUND_TOP * 0.5f, cz, 1.0f, GROUND_TOP, 1.0f, WHITE);
    else if (s == T_LANDFILL) { if (IsRecycleTile(x, y)) BoxT(TX_CONCRETE, cx, GROUND_TOP * 0.5f, cz, 1.0f, GROUND_TOP, 1.0f, (Color){ 215, 215, 215, 255 }); else BoxT(TX_TRASH, cx, GROUND_TOP * 0.5f, cz, 1.0f, GROUND_TOP, 1.0f, WHITE); }
    else if (paved[y][x])    BoxT(TX_CONCRETE, cx, WALK_TOP * 0.5f, cz, 1.0f, WALK_TOP, 1.0f, (Color){ 215, 215, 215, 255 });
    else                     BoxT(TX_GRASS, cx, GROUND_TOP * 0.5f, cz, 1.0f, GROUND_TOP, 1.0f, (s == T_HOUSE) ? (Color){ 235, 255, 235, 255 } : WHITE);
    if (layerOn[L_ROADS]) {
        Rectangle r[8];
        int n = SidewalkRects(x, y, r);
        for (int i = 0; i < n; i++)
            BoxT(TX_CONCRETE, x + r[i].x + r[i].width / 2.0f, WALK_TOP * 0.5f, y + r[i].y + r[i].height / 2.0f,
                 r[i].width, WALK_TOP, r[i].height, xray ? COL_SIDEWALK_DIM : (Color){ 255, 238, 200, 255 });
    }
}

// ---- the scene -------------------------------------------------------------------------
// The city is split in two. Everything that does not move is baked into GPU meshes (rebuilt only
// when the layers, the night, the city or a tile's state changes). Everything that moves or
// changes every frame is batched fresh each frame.
static unsigned int GSceneKey(bool xray) {
    unsigned int h = 2166136261u;
#define HMIX(v) (h = (h ^ (unsigned int)(v)) * 16777619u)
    HMIX(gSceneGen); HMIX(smartCity); HMIX(NightNow()); HMIX(xray);
    for (int i = 0; i < L_COUNT; i++) HMIX(layerOn[i]);
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            HMIX(surface[y][x]); HMIX(bType[y][x]); HMIX(zone[y][x]); HMIX(ctrl[y][x]);
            HMIX(gateTile[y][x]); HMIX(pedNode[y][x]); HMIX(paved[y][x]); HMIX(health[y][x] < 50.0f);
        }
#undef HMIX
    return h;
}

static void DrawStaticScene(bool xray) {
    gAlpha = 1.0f;
    GPlane(V3(MAP_W / 2.0f, 0.0f, MAP_H / 2.0f), (Vector2){ MAP_W + 60, MAP_H + 60 }, (Color){ 58, 105, 52, 255 });   // the flat boundary floor
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) DrawGround3D(x, y, xray);
    if (layerOn[L_WATER]) DrawNetwork3D(-0.11f, 0.13f, COL_WATER_PIPE);     // (the sewer main is live: it can flash red)

    gAlpha = xray ? 0.45f : 1.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            if (s == T_TREE) {
                DrawTree3D(x, y);
                if (deco[y][x] == 3) DrawDeco3D(x, y);
            } else if (s == T_GRASS) {
                if (!xray) DrawGrassTufts(x, y);
                if (deco[y][x] && !paved[y][x]) DrawDeco3D(x, y);
            } else if (s == T_ROAD) {
                if (layerOn[L_ROADS]) DrawJunction3D(x, y, xray);
            } else if (layerOn[L_BUILDINGS]) {
                DrawStructureStatic3D(x, y);
            }
            if (layerOn[L_BUILDINGS] && zone[y][x] == Z_INDUSTRIAL && surface[y][x] == T_GRASS && paved[y][x] &&
                !(smartCity && ((x == HUB_X && y == HUB_Y) || (x == HEATPLANT_X && y == HEATPLANT_Y)))) DrawYard3D(x, y);
            if (layerOn[L_ROADS] && pedNode[y][x] && ((x * 3 + y * 5) % 4 == 0) && surface[y][x] != T_ROAD) DrawStreetLamp(x, y);
        }
    gAlpha = 1.0f;
    if (layerOn[L_BUILDINGS]) {
        DrawFences3D();
        if (smartCity) DrawFarmStatic3D();
    }
}

// a flood sheet over a road tile
static void DrawFloodTile(int x, int y) {
    BoxC(x + 0.5f, 0.0625f, y + 0.5f, 1.0f, 0.002f, 1.0f, (Color){ 80, 140, 255, (unsigned char)(floodLevel * 130.0f) });
}

static void DrawLiveScene(bool xray) {
    float t = (float)GetTime();
    gAlpha = 1.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            float cx = x + 0.5f, cz = y + 0.5f;
            if (s == T_ROAD && layerOn[L_ROADS]) {
                if (floodLevel > 0.02f) DrawFloodTile(x, y);
                DrawJunctionLive3D(x, y);
                if (smartCity && monitorTile[y][x]) DrawMonitor3D(x, y);
            }
            if (layerOn[L_HEAT])      BoxC(cx, 0.080f, cz, 1.0f, 0.01f, 1.0f, HeatColorAt(x, y));
            if (layerOn[L_POLLUTION]) BoxC(cx, 0.095f, cz, 1.0f, 0.01f, 1.0f, PolColorAt(x, y));
            if (layerOn[L_NOISE])     BoxC(cx, 0.0965f, cz, 1.0f, 0.01f, 1.0f, NoiseColorAt(x, y));
            if (fireI[y][x] > 0.0f)   BoxC(cx, 0.07f, cz, 0.85f, 0.01f, 0.85f, Fade(COL_FIRE, 0.15f + 0.3f * fireI[y][x] / 100.0f));
            if (layerOn[L_ROADS] && pedNode[y][x] && ((x * 3 + y * 5) % 4 == 0) && s != T_ROAD) DrawStreetLampGlow(x, y);
            if (layerOn[L_BUILDINGS]) {
                gAlpha = xray ? 0.45f : 1.0f;
                DrawStructureLive3D(x, y);
                gAlpha = 1.0f;
                if (IsBldg(s) && (layerOn[L_HEAT] || layerOn[L_POLLUTION])) {        // heat / pollution tint on tall buildings
                    float h = bHeight[y][x];
                    if (layerOn[L_HEAT])      BoxC(cx, h / 2, cz, 0.78f, h + 0.02f, 0.78f, HeatColorAt(x, y));
                    if (layerOn[L_POLLUTION]) BoxC(cx, h / 2, cz, 0.79f, h + 0.03f, 0.79f, PolColorAt(x, y));
                }
            }
        }
    if (layerOn[L_CONGESTION] && layerOn[L_ROADS]) DrawCongestion3D();
    if (layerOn[L_SEWER]) DrawNetwork3D(0.11f, 0.10f, SewerColor());
    if (layerOn[L_BUILDINGS]) {
        DrawFencesLive3D();
        DrawLandfillLife3D();
        if (smartCity) { DrawFarmLive3D(); DrawPlusFarm3D(); }
    }
    if (layerOn[L_ROADS]) { DrawBins3D(); DrawLeaks3D(); }
    if (layerOn[L_BUILDINGS] && smartCity) DrawPlusWorld3D();

    // vehicles and people
    if (layerOn[L_TRAFFIC])
        for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) DrawCar3D(&cars[i]);
    DrawUserCars3D();
    for (int i = 0; i < MAX_TRUCKS; i++) if (trucks[i].active) DrawFireTruck3D(&trucks[i], t);
    for (int i = 0; i < MAX_POLICE; i++) if (police[i].active) DrawPoliceCar3D(&police[i], t);
    for (int i = 0; i < MAX_AMB; i++) if (ambs[i].active) DrawAmbulance3D(&ambs[i], t);
    if (binTruck.active) DrawBinTruck3D(&binTruck, t);
    DrawDeliveryMarks3D();
    for (int i = 0; i < MAX_PEDS; i++) if (peds[i].active) DrawPed3D(&peds[i]);
    DrawParticles3D();
    for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active) DrawDrone3D(&drones[i], t);
    (void)xray;
}

static void DrawScene3D(void) {
    bool xray = layerOn[L_WATER] || layerOn[L_SEWER];
    static unsigned int key = 0;
    static bool baked = false;
    unsigned int k = GSceneKey(xray);
    rlDisableBackfaceCulling();
    if (!baked || k != key) {                       // the static scenery changed: bake it again
        GBakeStart();
        DrawStaticScene(xray);
        GBakeFinish();
        key = k;
        baked = true;
    }
    GDrawBaked(false);                               // static scenery, opaque
    GXReset();
    DrawLiveScene(xray);
    GFlushLive(false);                               // moving and time-varying things, opaque
    rlDisableDepthMask();
    GDrawBaked(true);                                // see-through scenery (no depth writes)
    GFlushLive(true);
    rlEnableDepthMask();
    if (hoverOK && viewMode != VIEW_FP) DrawCubeWires((Vector3){ hoverX + 0.5f, 0.05f, hoverY + 0.5f }, 1.0f, 0.1f, 1.0f, WHITE);
    rlEnableBackfaceCulling();
}
// ---------------------------------------------------------------------------
// Smart-city dashboard + digital twin (key D)
// ---------------------------------------------------------------------------
static double TwAvg(int m, int k) { return twTime[m] > 8.0 ? twSum[m][k] / twTime[m] : NAN; }

static void TwRow(int *y, int x, const char *name, double a, double b, const char *fmt, bool lowerBetter) {
    char sa[32], sb[32], sc[32];
    bool okA = !isnan(a), okB = !isnan(b);
    if (okA) snprintf(sa, sizeof(sa), fmt, a); else snprintf(sa, sizeof(sa), "--");
    if (okB) snprintf(sb, sizeof(sb), fmt, b); else snprintf(sb, sizeof(sb), "--");
    Color cc = LIGHTGRAY;
    if (okA && okB) {
        double base = fabs(a) < 1e-6 ? 1e-6 : fabs(a);
        double pct = (b - a) / base * 100.0;
        if (fabs(a) < 1e-6 && fabs(b) < 1e-6) { snprintf(sc, sizeof(sc), "0%%"); }
        else snprintf(sc, sizeof(sc), "%+.0f%%", pct);
        bool better = lowerBetter ? (b < a) : (b > a);
        if (fabs(pct) < 2.0) cc = LIGHTGRAY; else cc = better ? (Color){ 100, 230, 140, 255 } : (Color){ 245, 110, 90, 255 };
    } else sc[0] = 0;
    DrawText(name, x, *y, 10, COL_LABEL);
    DrawText(sa, x + 215, *y, 10, (Color){ 245, 215, 120, 255 });
    DrawText(sb, x + 280, *y, 10, (Color){ 120, 240, 160, 255 });
    DrawText(sc, x + 345, *y, 10, cc);
    *y += 15;
}

static void DashItem(int *y, int x, Color head, bool on, const char *title, const char *detail) {
    DrawRectangle(x, *y + 1, 3, 22, on ? head : (Color){ 70, 75, 85, 255 });
    DrawText(title, x + 9, *y, 10, on ? head : (Color){ 110, 116, 128, 255 });
    DrawText(detail, x + 9, *y + 11, 10, on ? LIGHTGRAY : (Color){ 100, 106, 118, 255 });
    *y += 27;
}

static void DrawDashboard(void) {
    int px = PANEL_W + 12, py = 10, pw = VIEW_W - 24, ph = VIEW_H - 18;
    DrawRectangle(px, py, pw, ph, (Color){ 16, 20, 27, 244 });
    DrawRectangleLinesEx((Rectangle){ (float)px, (float)py, (float)pw, (float)ph }, 1.0f, (Color){ 70, 90, 120, 255 });
    DrawText("SMART CITY DASHBOARD  -  digital twin", px + 14, py + 10, 14, (Color){ 120, 240, 160, 255 });
    DrawText("[Tab] close", px + pw - 84, py + 12, 10, GRAY);
    char b[160], b2[160];

    // ---------- left: digital twin ----------
    int x = px + 14, y = py + 38;
    DrawText("DIGITAL TWIN  - averages recorded while each mode was running", x, y, 10, (Color){ 170, 175, 185, 255 });
    y += 16;
    DrawText("metric", x, y, 10, GRAY); DrawText("NORMAL", x + 215, y, 10, (Color){ 245, 215, 120, 255 });
    DrawText("SMART", x + 280, y, 10, (Color){ 120, 240, 160, 255 }); DrawText("change", x + 345, y, 10, GRAY);
    y += 14;
    DrawLine(x, y - 2, x + 395, y - 2, (Color){ 55, 65, 80, 255 });
    TwRow(&y, x, "Air quality (AQI)",            TwAvg(0, TW_AQI),    TwAvg(1, TW_AQI),    "%.0f",   true);
    TwRow(&y, x, "Fossil power burned (MW)",     TwAvg(0, TW_FOSSIL), TwAvg(1, TW_FOSSIL), "%.2f",   true);
    TwRow(&y, x, "CO2 from power (t/h)",         TwAvg(0, TW_CO2),    TwAvg(1, TW_CO2),    "%.2f",   true);
    TwRow(&y, x, "Electricity used (% of unsaved)", TwAvg(0, TW_DEMAND), TwAvg(1, TW_DEMAND), "%.0f", true);
    TwRow(&y, x, "Lighting power at night (%)",  twNight[0] > 8 ? twSum[0][TW_LIGHT] / twNight[0] : NAN, twNight[1] > 8 ? twSum[1][TW_LIGHT] / twNight[1] : NAN, "%.0f", true);
    TwRow(&y, x, "Average road speed",           TwAvg(0, TW_SPEED),  TwAvg(1, TW_SPEED),  "%.2f",   false);
    TwRow(&y, x, "Vehicles stopped (%)",         TwAvg(0, TW_STOP),   TwAvg(1, TW_STOP),   "%.0f",   true);
    TwRow(&y, x, "Water lost to leaks (L/min)",  TwAvg(0, TW_LEAK),   TwAvg(1, TW_LEAK),   "%.0f",   true);
    double r0 = twRespN[0] > 0 ? twRespSum[0] / twRespN[0] : NAN, r1 = twRespN[1] > 0 ? twRespSum[1] / twRespN[1] : NAN;
    double t0 = twTransN[0] > 0 ? twTransSum[0] / twTransN[0] : NAN, t1 = twTransN[1] > 0 ? twTransSum[1] / twTransN[1] : NAN;
    TwRow(&y, x, "Medical response (s)",         r0, r1, "%.1f", true);
    TwRow(&y, x, "Patient to hospital (s)",      t0, t1, "%.1f", true);
    double l0 = (twLandfill[0] + twRecycle[0]) > 0.001 ? 100.0 * twLandfill[0] / (twLandfill[0] + twRecycle[0]) : NAN;
    double l1 = (twLandfill[1] + twRecycle[1]) > 0.001 ? 100.0 * twLandfill[1] / (twLandfill[1] + twRecycle[1]) : NAN;
    TwRow(&y, x, "Waste sent to landfill (%)",   l0, l1, "%.0f", true);
    TwRow(&y, x, "Patients lost (count)",        twTime[0] > 8 ? twLost[0] : NAN, twTime[1] > 8 ? twLost[1] : NAN, "%.0f", true);
    TwRow(&y, x, "City budget net (k$/day)",     TwAvg(0, TW_BUDGET), TwAvg(1, TW_BUDGET), "%.2f", false);
    TwRow(&y, x, "Citizen happiness (0-100)",    TwAvg(0, TW_HAPPY),  TwAvg(1, TW_HAPPY),  "%.0f", false);
    TwRow(&y, x, "Employment rate (%)",          TwAvg(0, TW_EMPLOY), TwAvg(1, TW_EMPLOY), "%.0f", false);
    // total tonnes divided by total resident-time, then scaled to a year
    double co0 = (twTime[0] > 8 && twSum[0][TW_POP] > 0) ? twSum[0][TW_CARBON_PP] / twSum[0][TW_POP] * HOURS_PER_YEAR : NAN;
    double co1 = (twTime[1] > 8 && twSum[1][TW_POP] > 0) ? twSum[1][TW_CARBON_PP] / twSum[1][TW_POP] * HOURS_PER_YEAR : NAN;
    TwRow(&y, x, "CO2 per resident (t/yr)",      co0, co1, "%.1f", true);
    TwRow(&y, x, "Average noise (dB)",           TwAvg(0, TW_NOISE),  TwAvg(1, TW_NOISE),  "%.1f", true);
    y += 2;
    snprintf(b, sizeof(b), "recorded: normal %.2f days   smart %.2f days  (1 day = %d s)", twTime[0] / DAY_LEN, twTime[1] / DAY_LEN, (int)DAY_LEN);
    DrawText(b, x, y, 10, GRAY); y += 12;
    DrawText("Press T to switch; run each mode for whole days for a fair", x, y, 10, DARKGRAY); y += 11;
    DrawText("comparison (weather and time of day differ). All measured.", x, y, 10, DARKGRAY); y += 20;

    // energy flow
    DrawText("ENERGY FLOW", x, y, 10, (Color){ 170, 175, 185, 255 }); y += 14;
    if (smartCity) {
        snprintf(b, sizeof(b), "roofs %.1f MW   solar farm %.1f MW   wind %.1f MW", roofSolarKW / 1000.0f, farmSolarKW / 1000.0f, windKW / 1000.0f);
        DrawText(b, x, y, 10, LIGHTGRAY); y += 12;
        snprintf(b, sizeof(b), "city demand %.1f MW   EV fleet %+.2f MW   backup %.2f MW", st.energyDemand / 1000.0f, v2gKW / 1000.0f, gridImportKW / 1000.0f);
        DrawText(b, x, y, 10, LIGHTGRAY); y += 12;
        snprintf(b, sizeof(b), "battery %.0f%%   EV pack %.0f%%   renewable share %.0f%%", batteryPct, evSoc, renewPct);
        DrawText(b, x, y, 10, (Color){ 150, 235, 180, 255 }); y += 12;
    } else {
        snprintf(b, sizeof(b), "power plant burns %.1f MW of fuel to meet demand", st.energyDemand / 1000.0f);
        DrawText(b, x, y, 10, (Color){ 245, 215, 120, 255 }); y += 12;
        DrawText("Press T for the smart city to see solar, wind, battery and V2G.", x, y, 10, DARKGRAY); y += 12;
    }

    // ---------- right: live systems ----------
    int rx = px + 440; y = py + 38;
    DrawText(smartCity ? "LIVE SYSTEMS" : "LIVE SYSTEMS  (greyed = smart city only)", rx, y, 10, (Color){ 170, 175, 185, 255 }); y += 16;
    bool s = smartCity;
    if (s) snprintf(b2, sizeof(b2), "LEDs at %.0f%% brightness, saving %.0f kW now", lampAvg * 100.0f, fmaxf(0.0f, lightBaseKW - lightNowKW));
    else   snprintf(b2, sizeof(b2), "all %d lamps at full power all night", nLamps);
    DashItem(&y, rx, (Color){ 255, 225, 140, 255 }, true, s ? "Smart street lighting (dims when empty)" : "Street lighting", b2);
    snprintf(b2, sizeof(b2), "%d EV%s parked, %s %.0f kW, %.1f kWh sold", evCount, evCount == 1 ? "" : "s", v2gKW >= 0 ? "feeding" : "charging", fabsf(v2gKW), evKWhOut);
    DashItem(&y, rx, (Color){ 120, 190, 255, 255 }, s, "EV charging + vehicle-to-grid", b2);
    snprintf(b2, sizeof(b2), "%d rounds, %d emptied at %.0f%% full, %d tiles, overflow %.0f s", binTrips, binEmptied, binEmptied ? binFillSum / binEmptied : 0.0f, binTiles, binOverflowS);
    DashItem(&y, rx, (Color){ 120, 220, 140, 255 }, true, s ? "Smart bins (sensor-called refuse truck)" : "Bins (fixed-timetable refuse truck)", b2);
    snprintf(b2, sizeof(b2), "%d leaks, found in %.0fs, %.0f kL lost, rain tank %.0f%%", leaksTotal, leaksDetected ? leakAgeSum / leaksDetected : 0.0f, leakLostKL, 100.0f * rainTankKL / RAIN_TANK_CAP);
    DashItem(&y, rx, (Color){ 90, 170, 255, 255 }, true, s ? "Leak sensors + rainwater harvesting" : "Water leaks (found when they surface)", b2);
    snprintf(b2, sizeof(b2), "%d calls, %d green passes, %d red runs%s", preemptCalls, preemptWins, redRuns, rain > DRONE_RAIN ? "  (STORM)" : "");
    DashItem(&y, rx, (Color){ 255, 130, 120, 255 }, true, s ? "Emergency signal pre-emption" : "Emergency vehicles at signals (no pre-emption)", b2);
    {
        float best = 1e9f; for (int i = 0; i < N_AQ; i++) best = fminf(best, aqLevel[i]);
        snprintf(b2, sizeof(b2), "worst %s AQI %.0f, best AQI %.0f", AQ_NAME[aqWorst], aqLevel[aqWorst], best);
    }
    DashItem(&y, rx, (Color){ 200, 130, 220, 255 }, s, "Air-quality sensor network", b2);
    { int w = 0, f = 0; for (int i = 0; i < N_FLOOD; i++) { if (floodState[i] == 1) w++; if (floodState[i] == 2) f++; }
      snprintf(b2, sizeof(b2), "gates %.0f%% open, basin %.0f%%, %d warn / %d flood", drainOpen * 100.0f, 100.0f * retentionKL / RETENTION_CAP, w, f); }
    DashItem(&y, rx, (Color){ 100, 200, 255, 255 }, s, "Flood sensors + storm-drain control", b2);
    snprintf(b2, sizeof(b2), "%d of %d bays free, signs guide drivers", parkFree, parkTotal);
    if (!s) snprintf(b2, sizeof(b2), "%d of %d bays free, ~%.0f cars circling", parkFree, parkTotal, parkSearch);
    DashItem(&y, rx, (Color){ 245, 215, 90, 255 }, true, s ? "Smart parking guidance" : "Car park (no guidance)", b2);
    snprintf(b2, sizeof(b2), "%.2f MW waste heat, %.2f piped, %d bldgs, %.0f%% covered", wasteHeatKW / 1000.0f, dhKW / 1000.0f, dhBuildings, heatCover);
    DashItem(&y, rx, (Color){ 255, 160, 80, 255 }, s, "District heating (factory waste heat)", b2);
    snprintf(b2, sizeof(b2), "%d aboard, %d boardings, %d stops, ~%d cars off road", shuttlePax, shuttleBoard, shuttleStops, modalCars);
    DashItem(&y, rx, (Color){ 0, 215, 185, 255 }, s, "Autonomous shuttles (2 on the loop)", b2);
    { int a = 0; for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active && drones[i].kind >= D_PARCEL) a++;
      snprintf(b2, sizeof(b2), "%d parcels, %d prescriptions, %d airborne, %d van trips saved", parcelsDelivered, medDeliveries, a, parcelsDelivered + medDeliveries); }
    DashItem(&y, rx, (Color){ 250, 170, 50, 255 }, s, "Drone delivery", b2);
    snprintf(b2, sizeof(b2), "%d green roofs, run-off at %.0f%% of normal", greenRoofs, runoffK * 100.0f);
    DashItem(&y, rx, (Color){ 110, 210, 110, 255 }, s, "Green roofs + stormwater", b2);
}

// ---------------------------------------------------------------------------
// UI panels
// ---------------------------------------------------------------------------
static Vector2 GetScaledMouse(void);
static int uiY;

static Rectangle LayerRect(int i) { return (Rectangle){ 10, (float)(LAYER_Y0 + i * LAYER_H), PANEL_W - 20, LAYER_H - 2 }; }
static Rectangle ViewBtnRect(void) { return (Rectangle){ 10, 26, PANEL_W - 100, 18 }; }
static Rectangle FpBtnRect(void) { return (Rectangle){ PANEL_W - 84, 26, 74, 18 }; }
static Rectangle SmartBtnRect(void) { return (Rectangle){ 10, 48, PANEL_W - 90, 22 }; }
static Rectangle DashBtnRect(void) { return (Rectangle){ PANEL_W - 74, 48, 64, 22 }; }

// 2D view: the smart city is drawn zoomed out so the renewable farm east of the map fits
static float View2DZoom(void) { return smartCity ? (float)VIEW_W / (float)((MAP_W + FARM_W) * TILE) : 1.0f; }
static float View2DOffY(void) { return ((float)VIEW_H - (float)VIEW_H * View2DZoom()) * 0.5f; }

static void ToggleSmartCity(void) {
    smartCity = !smartCity;
    SmartPlusToggled(smartCity);
    ResetCamera();
    UpdateMaps(); UpdateStats();                  // refresh the numbers straight away
}

static void StatRow(const char *label, const char *val, Color c) {
    DrawText(label, 12, uiY, 10, COL_LABEL);
    DrawText(val, PANEL_W - 12 - MeasureText(val, 10), uiY, 10, c);
    uiY += ROW_H;
}

static void StatBar(const char *label, const char *val, float pct, Color c) {
    Color fill = (pct > 100.0f) ? RED : c;
    DrawText(label, 12, uiY, 10, COL_LABEL);
    DrawText(val, PANEL_W - 12 - MeasureText(val, 10), uiY, 10, fill);
    uiY += 12;
    DrawRectangle(12, uiY, PANEL_W - 24, 5, (Color){ 45, 50, 60, 255 });
    DrawRectangle(12, uiY, (int)((PANEL_W - 24) * Clamp(pct / 100.0f, 0.0f, 1.0f)), 5, fill);
    uiY += 10;
}

static void Alert(const char *s, int *n) {
    if (*n < 3) { DrawText(s, 12, uiY, 10, (Color){ 255, 110, 90, 255 }); uiY += 12; }
    (*n)++;
}

static void DrawLeftPanel(void) {
    Vector2 m = GetScaledMouse();
    DrawRectangle(0, 0, PANEL_W, SCREEN_H, COL_PANEL);
    DrawLine(PANEL_W - 1, 0, PANEL_W - 1, SCREEN_H, (Color){ 55, 65, 80, 255 });
    DrawText("SMART CITY CONTROL", 12, 8, 14, RAYWHITE);

    Rectangle vb = ViewBtnRect();
    DrawRectangleRec(vb, CheckCollisionPointRec(m, vb) ? (Color){ 55, 65, 85, 255 } : (Color){ 42, 48, 60, 255 });
    DrawText(TextFormat("View: %s  [V]", viewMode == VIEW_2D ? "2D top-down" : (viewMode == VIEW_3D ? "3D free cam" : "first person")), (int)vb.x + 8, (int)vb.y + 4, 10, SKYBLUE);
    Rectangle fb = FpBtnRect();
    DrawRectangleRec(fb, viewMode == VIEW_FP ? (Color){ 40, 110, 70, 255 } : (CheckCollisionPointRec(m, fb) ? (Color){ 55, 65, 85, 255 } : (Color){ 42, 48, 60, 255 }));
    DrawRectangleLinesEx(fb, 1.0f, (Color){ 90, 190, 130, 255 });
    DrawText("[F] walk", (int)fb.x + 10, (int)fb.y + 4, 10, (Color){ 150, 255, 190, 255 });

    Rectangle sb = SmartBtnRect();
    DrawRectangleRec(sb, smartCity ? (CheckCollisionPointRec(m, sb) ? (Color){ 40, 120, 70, 255 } : (Color){ 30, 100, 58, 255 })
                                   : (CheckCollisionPointRec(m, sb) ? (Color){ 70, 62, 40, 255 } : (Color){ 52, 48, 36, 255 }));
    DrawRectangleLinesEx(sb, 1.0f, smartCity ? (Color){ 90, 220, 140, 255 } : (Color){ 150, 130, 70, 255 });
    DrawText(smartCity ? "SMART CITY [T]" : "NORMAL CITY [T]", (int)sb.x + 8, (int)sb.y + 6, 10, smartCity ? (Color){ 150, 255, 190, 255 } : (Color){ 245, 215, 120, 255 });
    Rectangle db = DashBtnRect();
    DrawRectangleRec(db, showDash ? (Color){ 50, 90, 140, 255 } : (CheckCollisionPointRec(m, db) ? (Color){ 55, 65, 85, 255 } : (Color){ 42, 48, 60, 255 }));
    DrawRectangleLinesEx(db, 1.0f, (Color){ 90, 130, 190, 255 });
    DrawText("[Tab] data", (int)db.x + 6, (int)db.y + 6, 10, SKYBLUE);

    DrawText("LAYERS  (click or press 1-9)", 12, LAYER_Y0 - 12, 10, GRAY);
    for (int i = 0; i < L_COUNT; i++) {
        Rectangle r = LayerRect(i);
        if (CheckCollisionPointRec(m, r)) DrawRectangleRec(r, (Color){ 40, 46, 58, 255 });
        DrawRectangleLines((int)r.x + 2, (int)r.y + 2, 11, 11, GRAY);
        if (layerOn[i]) DrawRectangle((int)r.x + 4, (int)r.y + 4, 7, 7, LAYER_COL[i]);
        DrawText(LAYER_NAMES[i], (int)r.x + 20, (int)r.y + 3, 10, layerOn[i] ? RAYWHITE : GRAY);
        DrawText(TextFormat("[%d]", i + 1), (int)(r.x + r.width) - 22, (int)r.y + 3, 10, DARKGRAY);
    }

    DrawText("CITY STATISTICS", 12, LAYER_Y0 + L_COUNT * LAYER_H + 2, 10, GRAY);
    uiY = LAYER_Y0 + L_COUNT * LAYER_H + 13;
    int mins = (int)((st.hour - (int)st.hour) * 60.0f);
    StatRow("Time", TextFormat("Day %d  %02d:%02d  x%.1f%s", st.day, (int)st.hour, mins, simSpeed, paused ? " PAUSED" : ""), RAYWHITE);
    StatRow("Population", TextFormat("%d  (%.0f%% employed)", st.population, employRate), GREEN);
    StatRow("Buildings", TextFormat("%d  (%d damaged)", st.buildings, st.damaged), RAYWHITE);
    StatRow("Green space", TextFormat("%.0f%%", st.green), (Color){ 110, 200, 100, 255 });
    StatBar(smartCity ? "Energy (renewable supply)" : "Energy (power plant)", TextFormat("%.1f / %.1f MW", st.energyDemand / 1000.0f, st.energyCap / 1000.0f), st.energyPct, ORANGE);
    StatBar("Water supply", TextFormat("%.1f / %.1f kL/min", st.waterDemand / 1000.0f, st.waterCap / 1000.0f), st.waterPct, COL_WATER_PIPE);
    StatBar("Sewer load", TextFormat("%.0f%%", st.sewerPct), st.sewerPct, (Color){ 190, 140, 80, 255 });
    StatBar("Air quality", TextFormat("AQI %.0f", st.aqi), st.aqi / 1.5f, (Color){ 190, 90, 190, 255 });
    StatBar("Landfill capacity", TextFormat("%.0f%% full", landfillFill), landfillFill, (Color){ 150, 130, 90, 255 });
    StatRow("Temperature", TextFormat("%.1f C  (+%.1f urban)", st.temp, st.heatIsland), ORANGE);
    StatRow("Rainfall", TextFormat("%.1f mm/h", st.rainMm), SKYBLUE);
    StatRow("Vehicles", TextFormat("%d  (%d stopped, %d bikes)", st.vehicles, st.jammed, nBikes), (Color){ 240, 210, 50, 255 });
    StatRow("Pedestrians", TextFormat("%d", nPedsActive), COL_CIVILIAN);
    StatRow("Fires", TextFormat("%d active / %d collapses", st.fires, collapses), COL_FIRE);
    StatRow("Crime", TextFormat("%d reported / %d arrests", crimes, arrests), COL_CRIMINAL);
    StatRow("Road offences", TextFormat("%d seen / %d reported", offences, reports), (Color){ 250, 170, 80, 255 });
    StatRow("Police chases", TextFormat("%d caught / %d escaped", caught, escaped), SKYBLUE);
    StatRow("Security index", TextFormat("%.0f%%", st.security), LIGHTGRAY);
    StatRow("Medical", TextFormat("%d calls / %d saved / %d lost", medCalls, medSaved, medLost), (Color){ 255, 120, 130, 255 });
    StatRow("City budget", TextFormat("%+.1f k$  (%+.2f k$/day)", budBalanceK, budIncomeK - budSpendK), budBalanceK >= 0.0f ? (Color){ 120, 230, 140, 255 } : (Color){ 245, 110, 90, 255 });
    StatRow("Happiness", TextFormat("%.0f / 100  (low: %s)", happyNow, HAP_NAME[happyLow]), happyNow >= 70.0f ? (Color){ 120, 230, 140, 255 } : happyNow >= 40.0f ? (Color){ 245, 215, 120, 255 } : (Color){ 245, 110, 90, 255 });
    StatRow("CO2 per resident", TextFormat("%.1f t/yr", carbonPerPerson), carbonPerPerson <= 10.0f ? (Color){ 120, 230, 140, 255 } : carbonPerPerson <= 20.0f ? (Color){ 245, 215, 120, 255 } : (Color){ 245, 110, 90, 255 });
    if (smartCity) StatRow("Recycling", TextFormat("%.0f%% diverted  (%.0f t)", recycledT + wasteT > 0 ? 100.0f * recycledT / (recycledT + wasteT) : 64.0f, recycledT), (Color){ 90, 220, 140, 255 });
    else           StatRow("Power source", "fuel-burning plant", (Color){ 200, 160, 110, 255 });

    DrawText("ALERTS", 12, uiY, 10, GRAY);
    uiY += 12;
    int n = 0, pending = 0;
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active && cars[i].wanted) pending++;
    int illNow = 0; for (int i = 0; i < MAX_PEDS; i++) if (peds[i].active && peds[i].ill) illNow++;
    if (!smartCity && st.energyPct > 100.0f) Alert("! Power shortage (brownout)", &n);
    if (smartCity && gridImportKW > 1.0f) Alert("! Battery flat - importing backup power", &n);
    if (rain > DRONE_RAIN)     Alert(smartCity ? "! Storm: drones grounded, ground crews" : "! Storm warning", &n);
    if (illNow > 0)            Alert(TextFormat("! %d medical emergenc%s", illNow, illNow == 1 ? "y" : "ies"), &n);
    { int leakN = 0, overN = 0; for (int i = 0; i < MAX_LEAKS; i++) if (leaks[i].active) leakN++; for (int i = 0; i < nBins; i++) if (bins[i].fill >= 100.0f) overN++;
      if (leakN > 0) Alert(TextFormat("! %d water leak%s", leakN, leakN == 1 ? "" : "s"), &n);
      if (overN > 0) Alert(TextFormat("! %d bin%s overflowing", overN, overN == 1 ? "" : "s"), &n); }
    if (st.waterPct > 100.0f)  Alert("! Water shortage", &n);
    if (st.sewerPct > 100.0f)  Alert("! Sewer overflow - flooding", &n);
    if (landfillFill > 90.0f)  Alert("! Landfill nearly full", &n);
    if (st.fires > 0)          Alert(TextFormat("! %d tiles on fire", st.fires), &n);
    if (robbingNow > 0)        Alert("! Crime in progress", &n);
    if (pending > 0)           Alert(TextFormat("! %d reported vehicle(s) wanted", pending), &n);
    if (n == 0) { DrawText("All systems normal", 12, uiY, 10, GREEN); }

    DrawText("F: first person - walk, open doors, drive", 12, SCREEN_H - 59, 10, DARKGRAY);
    DrawText("3D: WASD move  Q/E turn  Wheel zoom", 12, SCREEN_H - 48, 10, DARKGRAY);
    DrawText("RMB drag: tilt/turn   R: reset cam", 12, SCREEN_H - 37, 10, DARKGRAY);
    DrawText("SPACE: storm   Click: start fire", 12, SCREEN_H - 26, 10, DARKGRAY);
    DrawText("P: pause  +/-: speed  T: smart  Tab: data", 12, SCREEN_H - 15, 10, DARKGRAY);
}

static void DrawGraph(int x, int y, int w, int h, int g, const char *label, Color col,
                      const char *val, float minRange, bool zeroBase) {
    DrawRectangle(x, y, w, h, (Color){ 38, 43, 53, 255 });
    DrawRectangleLines(x, y, w, h, (Color){ 62, 72, 88, 255 });
    DrawText(label, x + 6, y + 4, 10, COL_LABEL);
    DrawText(val, x + w - MeasureText(val, 10) - 6, y + 4, 10, col);

    float mn = 1e30f, mx = -1e30f;
    for (int i = 0; i < HIST; i++) { mn = fminf(mn, hist[g][i]); mx = fmaxf(mx, hist[g][i]); }
    if (zeroBase) {
        mn = 0.0f;
        if (mx < minRange) mx = minRange;
    } else if (mx - mn < minRange) {
        float mid = (mx + mn) * 0.5f;
        mn = mid - minRange * 0.5f; mx = mid + minRange * 0.5f;
    }
    float range = mx - mn;
    int gx0 = x + 6, gw = w - 12, gy0 = y + 18, gh = h - 24;
    DrawLine(gx0, gy0 + gh / 2, gx0 + gw, gy0 + gh / 2, (Color){ 50, 58, 70, 255 });
    for (int i = 0; i < HIST - 1; i++) {
        int i1 = (histIdx + i) % HIST, i2 = (histIdx + i + 1) % HIST;
        Vector2 a = { gx0 + gw * (float)i / (HIST - 1), gy0 + gh * (1.0f - (hist[g][i1] - mn) / range) };
        Vector2 b = { gx0 + gw * (float)(i + 1) / (HIST - 1), gy0 + gh * (1.0f - (hist[g][i2] - mn) / range) };
        DrawLineEx(a, b, 1.6f, col);
    }
}

static void DrawRightPanel(void) {
    int x0 = PANEL_W + VIEW_W;
    DrawRectangle(x0, 0, PANEL_W, SCREEN_H, COL_PANEL);
    DrawLine(x0, 0, x0, SCREEN_H, (Color){ 55, 65, 80, 255 });
    DrawText("CITY TRENDS  (last 2 min)", x0 + 12, 4, 10, GRAY);

    int pitch = (SCREEN_H - 22) / 7, gx = x0 + 10, gw = PANEL_W - 20, gh = pitch - 2, y = 18;
    DrawGraph(gx, y + 0 * pitch, gw, gh, G_ENERGY, "Energy demand", ORANGE, TextFormat("%.1f MW", st.energyDemand / 1000.0f), 2.0f, false);
    DrawGraph(gx, y + 1 * pitch, gw, gh, G_WATER,  "Water demand", COL_WATER_PIPE, TextFormat("%.1f kL/min", st.waterDemand / 1000.0f), 1.0f, false);
    DrawGraph(gx, y + 2 * pitch, gw, gh, G_SEWER,  "Sewer load (% capacity)", (Color){ 190, 140, 80, 255 }, TextFormat("%.0f%%", st.sewerPct), 100.0f, true);
    DrawGraph(gx, y + 3 * pitch, gw, gh, G_AQI,    "Air pollution (AQI)", (Color){ 200, 90, 200, 255 }, TextFormat("%.0f", st.aqi), 30.0f, false);
    DrawGraph(gx, y + 4 * pitch, gw, gh, G_TEMP,   "Temperature (C)", YELLOW, TextFormat("%.1f C", st.temp), 6.0f, false);
    DrawGraph(gx, y + 5 * pitch, gw, gh, G_RAIN,   "Rainfall (mm/h)", SKYBLUE, TextFormat("%.1f", st.rainMm), 5.0f, true);
    DrawGraph(gx, y + 6 * pitch, gw, gh, G_CARS,   "Vehicles on road", (Color){ 240, 210, 50, 255 }, TextFormat("%d", st.vehicles), 20.0f, true);
}

// ---------------------------------------------------------------------------
// Status strip under the map
// ---------------------------------------------------------------------------
static void DrawStatusStrip(void) {
    int x0 = PANEL_W, y0 = VIEW_H, w = VIEW_W, h = SCREEN_H - VIEW_H;
    DrawRectangle(x0, y0, w, h, (Color){ 22, 26, 32, 255 });
    DrawLine(x0, y0, x0 + w, y0, (Color){ 55, 65, 80, 255 });
    int out = 0; for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active) out++;
    int dP = 0, dF = 0, dM = 0;
    for (int i = 0; i < MAX_DRONES; i++) if (drones[i].active) { if (drones[i].kind == D_POLICE) dP++; else if (drones[i].kind == D_FIRE) dF++; else dM++; }
    int amb = 0; for (int i = 0; i < MAX_AMB; i++) if (ambs[i].active) amb++;
    float resp = respN ? respSum / respN : 0.0f, trans = transN ? transSum / transN : 0.0f;

    if (!smartCity) {
        DrawText("NORMAL CITY", x0 + 10, y0 + 7, 14, (Color){ 245, 215, 120, 255 });
        DrawText("Power plant burns fuel. Fire engines, police cars and ambulances respond by road. Press T for the SMART CITY.", x0 + 10, y0 + 26, 10, GRAY);
        DrawText(TextFormat("Fuel burn %.1f MW   |   Ambulances out %d   |   Medical: response %.1fs, to hospital %.1fs   |   Avg road speed %.2f",
                 st.energyDemand / 1000.0f, amb, resp, trans, avgSpeed), x0 + 10, y0 + 42, 10, LIGHTGRAY);
        return;
    }
    DrawText("SMART CITY", x0 + 10, y0 + 6, 14, (Color){ 120, 240, 160, 255 });
    // energy mix bar: roofs / solar farm / wind / backup import against demand
    float supply = roofSolarKW + farmSolarKW + windKW + gridImportKW;
    float scale = fmaxf(supply, st.energyDemand) * 1.05f + 1.0f;
    int bx = x0 + 120, by = y0 + 8, bw = 330, bh = 12;
    DrawRectangle(bx, by, bw, bh, (Color){ 40, 46, 56, 255 });
    float cur = 0.0f;
    struct { float v; Color c; } seg[4] = { { roofSolarKW, { 250, 215, 70, 255 } }, { farmSolarKW, { 240, 160, 50, 255 } }, { windKW, { 110, 190, 255, 255 } }, { gridImportKW, { 235, 80, 70, 255 } } };
    for (int i = 0; i < 4; i++) { int ww = (int)(bw * seg[i].v / scale); DrawRectangle(bx + (int)(bw * cur / scale), by, ww, bh, seg[i].c); cur += seg[i].v; }
    int dx = bx + (int)(bw * st.energyDemand / scale);
    DrawRectangle(dx - 1, by - 3, 3, bh + 6, WHITE);
    DrawText("demand", dx - 16, by + bh + 4, 8, WHITE);
    DrawText(TextFormat("roofs %.1f   solar farm %.1f   wind %.1f   backup %.1f  MW", roofSolarKW / 1000.0f, farmSolarKW / 1000.0f, windKW / 1000.0f, gridImportKW / 1000.0f), bx, y0 + 38, 10, LIGHTGRAY);
    DrawText(TextFormat("Battery %.0f%%   Renewable %.0f%%   Solar roofs %d", batteryPct, renewPct, solarRoofs), bx, y0 + 50, 10, (Color){ 150, 235, 180, 255 });

    int cx = x0 + 470;
    { int dD = out - dP - dF - dM;
      DrawText(rain > DRONE_RAIN ? "STORM: emergency drones grounded - ground crews, signals pre-empted"
                                 : TextFormat("Drones aloft: %d  (police %d / fire %d / medic %d / delivery %d)   sorties %d", out, dP, dF, dM, dD, droneSorties),
               cx, y0 + 8, 10, rain > DRONE_RAIN ? (Color){ 255, 170, 90, 255 } : SKYBLUE); }
    DrawText(TextFormat("Traffic monitors: %d re-routes, avg speed %.2f", reroutes, avgSpeed), cx, y0 + 22, 10, (Color){ 120, 220, 255, 255 });
    DrawText(TextFormat("Medical: response %.1fs, to hospital %.1fs", resp, trans), cx, y0 + 36, 10, (Color){ 255, 150, 150, 255 });
    DrawText(TextFormat("Recycled %.0f t   landfilled %.0f t   AQI %.0f", recycledT, wasteT, st.aqi), cx, y0 + 50, 10, (Color){ 130, 230, 150, 255 });
}

// Convert physical fullscreen mouse coordinates to the original UI layout.
static Vector2 GetScaledMouse(void) {
    Vector2 m = GetMousePosition();
    return (Vector2){
        m.x * (float)SCREEN_W / (float)GetScreenWidth(),
        m.y * (float)SCREEN_H / (float)GetScreenHeight()
    };
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------
static void HandleInput(float dt) {
    for (int i = 0; i < L_COUNT; i++) if (IsKeyPressed(KEY_ONE + i)) layerOn[i] = !layerOn[i];
    if (IsKeyPressed(KEY_V)) { if (viewMode == VIEW_FP) FpLeaveMode(); else viewMode = (viewMode == VIEW_2D) ? VIEW_3D : VIEW_2D; }
    if (IsKeyPressed(KEY_F)) { if (viewMode == VIEW_FP) FpLeaveMode(); else FpEnterMode(); }
    if (IsKeyPressed(KEY_T)) ToggleSmartCity();
    if (IsKeyPressed(KEY_TAB)) showDash = !showDash;
    if (IsKeyPressed(KEY_SPACE) && viewMode != VIEW_FP) { rainTimer = 30.0f; rainPeak = 1.0f; }
    if (IsKeyPressed(KEY_P)) paused = !paused;
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) simSpeed = fminf(4.0f, simSpeed * 2.0f);
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) simSpeed = fmaxf(0.5f, simSpeed * 0.5f);
    if (IsKeyPressed(KEY_R) && viewMode != VIEW_FP) ResetCamera();

    if (viewMode == VIEW_FP) { UpdateFp(dt); hoverOK = false; return; }       // first person: the mouse looks around, no UI clicks

    Vector2 m = GetScaledMouse();
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (CheckCollisionPointRec(m, FpBtnRect())) { FpEnterMode(); return; }
        if (CheckCollisionPointRec(m, ViewBtnRect())) viewMode = (viewMode == VIEW_2D) ? VIEW_3D : VIEW_2D;
        if (CheckCollisionPointRec(m, SmartBtnRect())) ToggleSmartCity();
        if (CheckCollisionPointRec(m, DashBtnRect())) showDash = !showDash;
        for (int i = 0; i < L_COUNT; i++) if (CheckCollisionPointRec(m, LayerRect(i))) layerOn[i] = !layerOn[i];
    }

    if (viewMode == VIEW_3D) UpdateOrbit(dt);

    hoverOK = false;
    if (m.x >= PANEL_W && m.x < PANEL_W + VIEW_W && m.y >= 0 && m.y < VIEW_H) {
        Vector2 lm = { m.x - PANEL_W, m.y };
        int tx = 0, ty = 0;
        bool ok;
        if (viewMode == VIEW_2D) {
            float z = View2DZoom();
            float wx = lm.x / z, wy = (lm.y - View2DOffY()) / z;
            ok = (wx >= 0.0f && wy >= 0.0f);
            tx = (int)(wx / TILE); ty = (int)(wy / TILE);
        }
        else ok = PickTile3D(lm, &tx, &ty);
        if (ok && InBounds(tx, ty)) {
            hoverOK = true; hoverX = tx; hoverY = ty;
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !showDash) Ignite(tx, ty);
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void) {
    InitWindow(SCREEN_W, SCREEN_H, "Smart City Simulator");
    int monitor = GetCurrentMonitor();
    SetWindowSize(GetMonitorWidth(monitor), GetMonitorHeight(monitor));
    ToggleFullscreen();
    SetTargetFPS(60);
    SetRandomSeed((unsigned int)time(NULL));

    RenderTexture2D view = LoadRenderTexture(VIEW_W, VIEW_H);
    RenderTexture2D screen = LoadRenderTexture(SCREEN_W, SCREEN_H);
    MakeTextures();

    GenerateCity();
    ResetCamera();
    st.security = 70.0f;
    UpdateClock();
    UpdateMaps();
    UpdateStats();
    PushHistory();

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;

        HandleInput(dt);
        if (!paused) UpdateSim(dt * simSpeed);

        // Map view is rendered to a texture so 2D and 3D share the same clipped area
        BeginTextureMode(view);
            if (viewMode == VIEW_2D) {
                ClearBackground(smartCity ? (Color){ 26, 32, 28, 255 } : COL_GRASS);
                if (smartCity) {
                    Camera2D cam2 = { (Vector2){ 0.0f, View2DOffY() }, (Vector2){ 0.0f, 0.0f }, 0.0f, View2DZoom() };
                    BeginMode2D(cam2);
                        DrawScene2D();
                    EndMode2D();
                } else DrawScene2D();
            } else if (viewMode == VIEW_FP) {
                ClearBackground((Color){ 135, 190, 230, 255 });
                rlSetClipPlanes(0.012, 160.0);                                  // eye level: tighter planes keep the depth buffer precise
                BeginMode3D(FpCamera());
                    DrawScene3D();
                EndMode3D();
                rlSetClipPlanes(0.01, 1000.0);
            } else {
                ClearBackground((Color){ 135, 190, 230, 255 });
                BeginMode3D(BuildCamera());
                    DrawScene3D();
                EndMode3D();
            }
            DrawWeatherFx();
            if (viewMode == VIEW_FP) DrawFpHud();
        EndTextureMode();

        BeginTextureMode(screen);
            ClearBackground(BLACK);
            DrawTextureRec(view.texture, (Rectangle){ 0, 0, (float)VIEW_W, -(float)VIEW_H }, (Vector2){ PANEL_W, 0 }, WHITE);
            DrawStatusStrip();
            DrawLeftPanel();
            DrawRightPanel();
            if (showDash) DrawDashboard();
            if (layerOn[L_HEAT]) {
                DrawRectangle(PANEL_W + 6, VIEW_H - 36, 210, 14, Fade(BLACK, 0.6f));
                DrawText("Heat map: blue = cool, red = hot", PANEL_W + 10, VIEW_H - 33, 10, WHITE);
            }
            if (layerOn[L_POLLUTION]) {
                DrawRectangle(PANEL_W + 6, VIEW_H - 20, 240, 14, Fade(BLACK, 0.6f));
                DrawText("Pollution map: green = clean, purple = dirty", PANEL_W + 10, VIEW_H - 17, 10, WHITE);
            }
            if (layerOn[L_NOISE]) {
                DrawRectangle(PANEL_W + 6, VIEW_H - 52, 340, 14, Fade(BLACK, 0.6f));
                DrawText(TextFormat("Noise map: green = quiet (45 dB), red = loud (75 dB)  avg %.0f dB", noiseAvg), PANEL_W + 10, VIEW_H - 49, 10, WHITE);
            }
            if (paused) DrawText("PAUSED", PANEL_W + VIEW_W / 2 - 40, 10, 20, YELLOW);
        EndTextureMode();

        // Stretch the complete interface to the actual fullscreen dimensions.
        BeginDrawing();
            ClearBackground(BLACK);
            DrawTexturePro(screen.texture,
                (Rectangle){ 0, 0, (float)SCREEN_W, -(float)SCREEN_H },
                (Rectangle){ 0, 0, (float)GetScreenWidth(), (float)GetScreenHeight() },
                (Vector2){ 0, 0 }, 0.0f, WHITE);
        EndDrawing();
    }

    if (viewMode == VIEW_FP) EnableCursor();
    FreeTextures();
    UnloadRenderTexture(view);
    UnloadRenderTexture(screen);
    CloseWindow();
    return 0;
}