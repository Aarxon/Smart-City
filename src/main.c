// Smart City Simulator (raylib, C)
//
// Layout:  [ LEFT: layers + live statistics ] [ MAP VIEW (2D / 3D) ] [ RIGHT: trend graphs ]
//
// Controls:
//   1-8 (or click)  Toggle layers: Buildings, Roads, Water, Sewer, Traffic, Congestion, Heat, Pollution
//   V               Switch 2D / 3D view
//   3D view         WASD / arrows = move, Q/E = rotate, mouse wheel = zoom,
//                   right-mouse drag = orbit/tilt, R = reset camera, Shift = move faster
//   Left click      Start a fire on a building / tree / grass tile
//   SPACE           Trigger a storm
//   P               Pause        + / -   Simulation speed
//
// Build (Linux/macOS):   gcc main.c -o city -lraylib -lm
// Build (Windows/MinGW): gcc main.c -o city.exe -lraylib -lopengl32 -lgdi32 -lwinmm

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdbool.h>
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
#define SCREEN_H    VIEW_H

#define SIDEWALK_W  5          // pixels (2D); scaled to tile units in 3D
#define LINE_THICK  3
#define HIST        120        // graph samples (1 per second => 2 minutes)
#define DAY_LEN     120.0f     // real seconds per simulated day
#define SIG_CYCLE   24.0f      // traffic light cycle (seconds)

#define MAX_CARS    90
#define MAX_PEDS    64
#define MAX_TRUCKS  6
#define MAX_POLICE  6
#define MAX_STN     4
#define MAX_PART    900
#define MAX_PATH    (MAP_W * MAP_H)

#define PLANT_KW    12000.0f   // output of one power plant
#define WORKS_LPM   9000.0f    // output of one water works (litres/min)
#define SEWER_CAP   8500.0f    // sewer capacity (litres/min)

#define LAYER_Y0    62
#define LAYER_H     16
#define ROW_H       12

typedef enum {
    T_GRASS = 0, T_ROAD, T_BUILDING, T_TREE, T_RUIN,
    T_FIRE_STN, T_POLICE_STN, T_POWER, T_WATER,
    T_HOUSE, T_MALL, T_PARK, T_LANDFILL
} TileType;

enum { B_RES = 0, B_COM, B_IND };
enum { Z_NONE = 0, Z_GATED1, Z_GATED2, Z_GATED3, Z_INDUSTRIAL, Z_LANDFILL, Z_MALL };
enum { C_NONE = 0, C_SIGNAL, C_ROUNDABOUT, C_STOP };
enum { L_BUILDINGS = 0, L_ROADS, L_WATER, L_SEWER, L_TRAFFIC, L_CONGESTION, L_HEAT, L_POLLUTION, L_COUNT };
enum { G_ENERGY = 0, G_WATER, G_SEWER, G_AQI, G_TEMP, G_RAIN, G_CARS, G_COUNT };
typedef enum { VIEW_2D = 0, VIEW_3D } ViewMode;
enum { V_SALOON = 0, V_HATCH, V_ESTATE, V_PICKUP, V_SUV, V_VAN, V_TRUCK, V_BIKE, V_COUNT };

typedef struct {
    float x, y;                 // position in tile units
    float yaw;                  // heading in degrees (local +Z forward)
    int gx, gy, tx, ty, dir;    // from-tile, to-tile, heading (0 up,1 down,2 left,3 right)
    float t, ox, oy, wait, speedMul, stopT, speedNow, reportT, lostT;
    float arcU, arcFrom, arcSweep;
    int claimX, claimY, id, reportKind, pursuer;
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
    "Traffic", "Congestion", "Heat map", "Pollution map"
};
static const Color LAYER_COL[L_COUNT] = {
    { 185, 120,  98, 255 }, { 150, 150, 155, 255 }, {  70, 150, 255, 255 }, { 170, 115,  55, 255 },
    { 240, 210,  50, 255 }, {  90, 220, 140, 255 }, { 235,  80,  50, 255 }, { 170,  60, 170, 255 }
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
static Station    fireStn[MAX_STN], policeStn[MAX_STN];
static int        nFire = 0, nPolice = 0;
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

static float hist[G_COUNT][HIST];
static int   histIdx = 0;
static bool  histInit = false;

static bool     layerOn[L_COUNT] = { true, true, false, false, true, true, false, false };
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
static bool IsBldg(int s) { return s == T_BUILDING || s == T_HOUSE || s == T_MALL; }
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
    float s = SIDEWALK_W / (float)TILE;
    int n = 0;
    bool up = IsRoad(x, y - 1), down = IsRoad(x, y + 1), left = IsRoad(x - 1, y), right = IsRoad(x + 1, y);
    if (up)    r[n++] = (Rectangle){ 0, 0, 1, s };
    if (down)  r[n++] = (Rectangle){ 0, 1 - s, 1, s };
    if (left)  r[n++] = (Rectangle){ 0, 0, s, 1 };
    if (right) r[n++] = (Rectangle){ 1 - s, 0, s, 1 };
    if (!up && !left && IsRoad(x - 1, y - 1))    r[n++] = (Rectangle){ 0, 0, s, s };
    if (!up && !right && IsRoad(x + 1, y - 1))   r[n++] = (Rectangle){ 1 - s, 0, s, s };
    if (!down && !left && IsRoad(x - 1, y + 1))  r[n++] = (Rectangle){ 0, 1 - s, s, s };
    if (!down && !right && IsRoad(x + 1, y + 1)) r[n++] = (Rectangle){ 1 - s, 1 - s, s, s };
    return n;
}

// ---- Traffic signals: 0 = red, 1 = green, 2 = yellow. axis 0 = N-S traffic, 1 = E-W traffic
static int SigState(int x, int y, int axis, float *redLeft) {
    float ta = fmodf(simTime + sigOff[y][x] + (axis ? 12.0f : 0.0f), SIG_CYCLE);
    if (redLeft) *redLeft = (ta >= 11.0f) ? (SIG_CYCLE - ta) : 0.0f;
    if (ta < 9.0f) return 1;
    if (ta < 11.0f) return 2;
    return 0;
}

// ---------------------------------------------------------------------------
// Procedural textures (no image files needed) + 3D primitive helpers
// ---------------------------------------------------------------------------
enum {
    TX_BRICK = 0, TX_SIDING, TX_STUCCO, TX_ROOF, TX_GRASS, TX_ASPHALT, TX_CONCRETE, TX_CORRUG,
    TX_BARK, TX_HEDGE, TX_TRASH, TX_WIN_GLASS, TX_WIN_GLASS_N, TX_WIN_BRICK, TX_WIN_BRICK_N,
    TX_WIN_OFFICE, TX_WIN_OFFICE_N, TX_POLICE, TX_FIRE, TX_VAN, TX_COUNT
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
}

static void FreeTextures(void) { for (int i = 0; i < TX_COUNT; i++) UnloadTexture(tex[i]); }

// ---- 3D primitives (all coordinates are local to the current transform) ----
static void SetShade(Color c, float k) {
    rlColor4ub((unsigned char)(c.r * k), (unsigned char)(c.g * k), (unsigned char)(c.b * k), c.a);
}
// Untextured box with simple per-face shading (top lightest, one side darkest)
static void BoxC(float x, float y, float z, float w, float h, float l, Color c) {
    float a = w * 0.5f, b = h * 0.5f, d = l * 0.5f;
    rlSetTexture(0);
    rlBegin(RL_QUADS);
    SetShade(c, 0.90f);
    rlVertex3f(x - a, y - b, z + d); rlVertex3f(x + a, y - b, z + d); rlVertex3f(x + a, y + b, z + d); rlVertex3f(x - a, y + b, z + d);
    SetShade(c, 0.78f);
    rlVertex3f(x - a, y - b, z - d); rlVertex3f(x - a, y + b, z - d); rlVertex3f(x + a, y + b, z - d); rlVertex3f(x + a, y - b, z - d);
    SetShade(c, 1.00f);
    rlVertex3f(x - a, y + b, z - d); rlVertex3f(x - a, y + b, z + d); rlVertex3f(x + a, y + b, z + d); rlVertex3f(x + a, y + b, z - d);
    SetShade(c, 0.50f);
    rlVertex3f(x - a, y - b, z - d); rlVertex3f(x + a, y - b, z - d); rlVertex3f(x + a, y - b, z + d); rlVertex3f(x - a, y - b, z + d);
    SetShade(c, 0.84f);
    rlVertex3f(x + a, y - b, z - d); rlVertex3f(x + a, y + b, z - d); rlVertex3f(x + a, y + b, z + d); rlVertex3f(x + a, y - b, z + d);
    SetShade(c, 0.72f);
    rlVertex3f(x - a, y - b, z - d); rlVertex3f(x - a, y - b, z + d); rlVertex3f(x - a, y + b, z + d); rlVertex3f(x - a, y + b, z - d);
    rlEnd();
}

// Textured cube; (u, v) are the texture-coordinate extents used on the side faces
static void CubeUV(int ti, float x, float y, float z, float w, float h, float l, Color c, float u, float v) {
    float a = w * 0.5f, b = h * 0.5f, d = l * 0.5f;
    rlSetTexture(tex[ti].id);
    rlBegin(RL_QUADS);
    // front (+z)
    SetShade(c, 0.90f);
    rlTexCoord2f(0, v); rlVertex3f(x - a, y - b, z + d);
    rlTexCoord2f(u, v); rlVertex3f(x + a, y - b, z + d);
    rlTexCoord2f(u, 0); rlVertex3f(x + a, y + b, z + d);
    rlTexCoord2f(0, 0); rlVertex3f(x - a, y + b, z + d);
    // back (-z)
    SetShade(c, 0.78f);
    rlTexCoord2f(u, v); rlVertex3f(x - a, y - b, z - d);
    rlTexCoord2f(u, 0); rlVertex3f(x - a, y + b, z - d);
    rlTexCoord2f(0, 0); rlVertex3f(x + a, y + b, z - d);
    rlTexCoord2f(0, v); rlVertex3f(x + a, y - b, z - d);
    // top
    SetShade(c, 1.00f);
    rlTexCoord2f(0, 1); rlVertex3f(x - a, y + b, z - d);
    rlTexCoord2f(0, 0); rlVertex3f(x - a, y + b, z + d);
    rlTexCoord2f(1, 0); rlVertex3f(x + a, y + b, z + d);
    rlTexCoord2f(1, 1); rlVertex3f(x + a, y + b, z - d);
    // bottom
    SetShade(c, 0.50f);
    rlTexCoord2f(1, 1); rlVertex3f(x - a, y - b, z - d);
    rlTexCoord2f(0, 1); rlVertex3f(x + a, y - b, z - d);
    rlTexCoord2f(0, 0); rlVertex3f(x + a, y - b, z + d);
    rlTexCoord2f(1, 0); rlVertex3f(x - a, y - b, z + d);
    // right (+x)
    SetShade(c, 0.84f);
    rlTexCoord2f(u, v); rlVertex3f(x + a, y - b, z - d);
    rlTexCoord2f(u, 0); rlVertex3f(x + a, y + b, z - d);
    rlTexCoord2f(0, 0); rlVertex3f(x + a, y + b, z + d);
    rlTexCoord2f(0, v); rlVertex3f(x + a, y - b, z + d);
    // left (-x)
    SetShade(c, 0.72f);
    rlTexCoord2f(0, v); rlVertex3f(x - a, y - b, z - d);
    rlTexCoord2f(u, v); rlVertex3f(x - a, y - b, z + d);
    rlTexCoord2f(u, 0); rlVertex3f(x - a, y + b, z + d);
    rlTexCoord2f(0, 0); rlVertex3f(x - a, y + b, z - d);
    rlEnd();
    rlSetTexture(0);
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
    rlSetTexture(tex[ti].id);
    rlBegin(RL_QUADS);
    rlColor4ub(col.r, col.g, col.b, col.a);
    rlTexCoord2f(0, 0); rlVertex3f(a.x, a.y, a.z);
    rlTexCoord2f(0, v); rlVertex3f(b.x, b.y, b.z);
    rlTexCoord2f(u, v); rlVertex3f(c.x, c.y, c.z);
    rlTexCoord2f(u, 0); rlVertex3f(d.x, d.y, d.z);
    rlEnd();
    rlSetTexture(0);
}
static void Tri3T(int ti, Vector3 a, Vector3 b, Vector3 c, Color col, float u, float v) {
    rlSetTexture(tex[ti].id);
    rlBegin(RL_TRIANGLES);
    rlColor4ub(col.r, col.g, col.b, col.a);
    rlTexCoord2f(0, 0);        rlVertex3f(a.x, a.y, a.z);
    rlTexCoord2f(u, 0);        rlVertex3f(b.x, b.y, b.z);
    rlTexCoord2f(u * 0.5f, v); rlVertex3f(c.x, c.y, c.z);
    rlEnd();
    rlSetTexture(0);
}

// Gable roof: ridge runs along local Z (alongZ) or local X. Eaves at y0, ridge at y0 + h.
static void RoofGable(float w, float l, float y0, float h, bool alongZ, Color roofCol, int wallTex, Color wallCol) {
    float a = w * 0.5f, b = l * 0.5f;
    #define RV(p, q, y) (alongZ ? (Vector3){ p, y, q } : (Vector3){ q, y, p })
    Quad3T(TX_ROOF, RV(-a, -b, y0), RV(-a, b, y0), RV(0, b, y0 + h), RV(0, -b, y0 + h), roofCol, 3, 2);
    Quad3T(TX_ROOF, RV(a, -b, y0), RV(a, b, y0), RV(0, b, y0 + h), RV(0, -b, y0 + h), Shade(roofCol, 0.82f), 3, 2);
    Tri3T(wallTex, RV(-a, b, y0), RV(a, b, y0), RV(0, b, y0 + h), wallCol, 1, 1);
    Tri3T(wallTex, RV(-a, -b, y0), RV(a, -b, y0), RV(0, -b, y0 + h), wallCol, 1, 1);
    #undef RV
}

// Hipped (pyramid) roof
static void RoofHip(float w, float l, float y0, float h, Color roofCol) {
    float a = w * 0.5f, b = l * 0.5f;
    Vector3 top = { 0, y0 + h, 0 };
    Vector3 p0 = { -a, y0, -b }, p1 = { a, y0, -b }, p2 = { a, y0, b }, p3 = { -a, y0, b };
    Tri3T(TX_ROOF, p0, p1, top, roofCol, 2, 2);
    Tri3T(TX_ROOF, p1, p2, top, Shade(roofCol, 0.85f), 2, 2);
    Tri3T(TX_ROOF, p2, p3, top, roofCol, 2, 2);
    Tri3T(TX_ROOF, p3, p0, top, Shade(roofCol, 0.85f), 2, 2);
}

// Enter / leave a local frame at tile-space (x,z) with a heading in degrees (local +Z = forward)
static void PushFrame(float x, float y, float z, float yawDeg) {
    rlPushMatrix();
    rlTranslatef(x, y, z);
    rlRotatef(yawDeg, 0, 1, 0);
}
static void PopFrame(void) { rlPopMatrix(); }
static float FaceYaw(int dir) { return atan2f((float)DX[dir], (float)DY[dir]) * RAD2DEG; }

// ---------------------------------------------------------------------------
// City generation
// ---------------------------------------------------------------------------
static void SpawnPed(Ped *p);

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
            if (Rf() < 0.78f) { drive[y][x] = (unsigned char)(RandModel() + 1); drvCol[y][x] = (unsigned char)GetRandomValue(0, N_CARCOL - 1); }
        }
    // a small pocket park inside each community
    int px = x1 - 1, py = (laneY > y0) ? laneY - 1 : laneY + 1;
    if (InBounds(px, py) && zone[py][px] == id) {
        surface[py][px] = T_TREE; vr[py][px] = (unsigned char)GetRandomValue(0, 255); drive[py][px] = 0; deco[py][px] = 3;
    }
}

static void GenerateCity(void) {
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
    PlaceFixed(12, 6, T_POWER);
    PlaceFixed(17, 6, T_WATER);
    faceDir[2][2] = 1; faceDir[4][19] = 3; faceDir[2][10] = 3; faceDir[11][3] = 3;

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
    nFire = nPolice = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int rx, ry;
            if (surface[y][x] == T_FIRE_STN && nFire < MAX_STN && FindAccess(x, y, &rx, &ry))
                fireStn[nFire++] = (Station){ x, y, rx, ry };
            if (surface[y][x] == T_POLICE_STN && nPolice < MAX_STN && FindAccess(x, y, &rx, &ry))
                policeStn[nPolice++] = (Station){ x, y, rx, ry };
        }

    memset(cars, 0, sizeof(cars));
    memset(peds, 0, sizeof(peds));
    memset(trucks, 0, sizeof(trucks));
    memset(police, 0, sizeof(police));
    memset(parts, 0, sizeof(parts));
    for (int i = 0; i < 30; i++) SpawnPed(&peds[i]);
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
        r->t += dt * speed;
        while (r->t >= 1.0f && r->idx < r->len - 1) { r->t -= 1.0f; r->idx++; }
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
    if (r < 71) return V_PICKUP;
    if (r < 87) return V_SUV;
    if (r < 95) return V_VAN;
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
    Car *c = &cars[slot];
    memset(c, 0, sizeof(*c));
    c->active = true; c->id = carSerial++;
    c->type = (unsigned char)type; c->colIdx = (unsigned char)colIdx;
    c->col = CAR_COLS[colIdx];
    c->dir = valid[GetRandomValue(0, nv - 1)];
    c->gx = c->tx = x; c->gy = c->ty = y; c->t = 1.0f;
    c->x = x + 0.5f; c->y = y + 0.5f; c->yaw = FaceYaw(c->dir);
    c->claimX = c->claimY = -1; c->pursuer = -1;
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

static void ReportCar(Car *c) {
    c->wanted = true;
    reports++;
    if (!Dispatch(police, MAX_POLICE, policeStn, nPolice, c->gx, c->gy, -1, 0, 1, c->id)) c->wanted = false;
}

static bool LaneBlocked(const Car *c, float *gapOut) {
    float dxv = (float)DX[c->dir], dyv = (float)DY[c->dir];
    for (int j = 0; j < MAX_CARS; j++) {
        const Car *o = &cars[j];
        if (o == c || !o->active || o->dir != c->dir || o->arcing) continue;
        float rx = o->x - c->x, ry = o->y - c->y;
        float along = rx * dxv + ry * dyv;
        float side = fabsf(-rx * dyv + ry * dxv);
        float need = 0.5f * (VLEN[c->type] + VLEN[o->type]) + 0.07f;
        if (along > 0.02f && along < need && side < 0.09f) { if (gapOut) *gapOut = along; return true; }
    }
    return false;
}

#define STOP_LINE 0.70f
static void UpdateCars(float dt) {
    static float cnt[MAP_H][MAP_W];
    memset(cnt, 0, sizeof(cnt));

    int active = 0, bikes = 0;
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) { if (cars[i].type == V_BIKE) bikes++; else active++; }
    int target = (int)(62 * TrafficFactor());
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
            if (c->active && c->type != V_BIKE && !c->wanted && !c->fleeing) { KillCar(c); break; }
        }
    }

    int n = 0, jammed = 0, nb = 0;
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
            c->arcU += dt * v / (c->arcSweep * R);
            float u = Clamp(c->arcU, 0.0f, 1.0f);
            float th = c->arcFrom + c->arcSweep * u;
            c->x = c->gx + 0.5f + R * cosf(th);
            c->y = c->gy + 0.5f + R * sinf(th);
            c->yaw = atan2f(-sinf(th), cosf(th)) * RAD2DEG;
            if (c->arcU >= 1.0f) { c->arcing = false; c->t = 0.0f; if (c->claimX >= 0) ReleaseClaim(c); }
        } else {
            // ---- rules of the road: stop lines, give way, pedestrian crossings -----------
            bool must = false;
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
                            if (s != 1 && !c->rogue && !c->fleeing && c->t <= STOP_LINE + 0.08f) must = true;
                            if (s == 0 && (c->rogue || c->fleeing) && prevT < STOP_LINE && prevT + dt * v >= STOP_LINE && c->lostT <= 0.0f) {
                                Violation(c, 1); c->lostT = 6.0f;
                            }
                        } else if (ctrl[iy][ix] == C_ROUNDABOUT) {
                            if (c->claimX < 0 && !c->fleeing) {
                                if (nodeOcc[iy][ix] > 0 && c->t >= STOP_LINE - 0.02f) must = true;
                                else if (c->t >= STOP_LINE - 0.02f) { c->claimX = ix; c->claimY = iy; nodeOcc[iy][ix]++; }
                            }
                        } else if (ctrl[iy][ix] == C_STOP) {
                            if (c->claimX < 0 && !c->rogue && !c->fleeing) {
                                if (c->t >= STOP_LINE - 0.02f) {
                                    c->stopT += dt;
                                    if (c->stopT < 1.0f || nodeOcc[iy][ix] > 0) must = true;
                                    else { c->claimX = ix; c->claimY = iy; nodeOcc[iy][ix]++; }
                                }
                            }
                        }
                    }
                    if (crossBusy[ty][tx] > 0 && c->t <= STOP_LINE + 0.12f) must = true;     // give way to pedestrians
                }
            }
            if (must && c->t >= STOP_LINE - 0.03f) { v = 0.0f; c->braking = true; }
            else if (must) v = fminf(v, 1.2f);

            float gap = 1.0f;
            if (LaneBlocked(c, &gap)) {
                c->wait += dt;
                if (c->wait < 4.0f) { v = 0.0f; c->braking = true; }   // after 4 s creep on (gridlock breaker)
            } else c->wait = 0.0f;
            if (v == 0.0f) jammed++;

            if (c->speeder && v > 2.9f && !c->wanted && c->lostT <= 0.0f && !c->fleeing) { Violation(c, 2); c->lostT = 8.0f; }

            c->t += dt * v;
            if (c->claimX >= 0 && c->gx == c->claimX && c->gy == c->claimY && c->t > 0.55f) ReleaseClaim(c);

            if (c->t >= 1.0f) {
                c->t = 0.0f;
                c->gx = c->tx; c->gy = c->ty; c->stopT = 0.0f;
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
                    } else if (straight && Rf() < 0.6f) choice = c->dir;
                    else choice = (no > 0) ? opts[GetRandomValue(0, no - 1)] : valid[0];
                }
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
                float lane = (c->type == V_BIKE) ? 0.33f : 0.17f;
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

        int cx = (int)c->x, cy = (int)c->y;
        if (InBounds(cx, cy)) cnt[cy][cx] += 1.0f;
    }
    st.vehicles = n;
    st.jammed = jammed;
    nBikes = nb;

    float k = Clamp(dt * 2.0f, 0.0f, 1.0f);
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) traffic[y][x] += (cnt[y][x] - traffic[y][x]) * k;
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
            if (!c->active || c->type == V_BIKE || c->type == V_TRUCK || c->rogue || c->wanted || c->fleeing || c->arcing || c->t > 0.5f) continue;
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
    int rx, ry;
    if (!GateRoad(id, &rx, &ry)) return;
    int houses = 0, withCar = 0;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            if (zone[y][x] == id && surface[y][x] == T_HOUSE) { houses++; if (drive[y][x]) withCar++; }
    float homeWant = houses * Clamp(1.1f - TrafficFactor() * 0.9f, 0.2f, 0.85f);
    if (withCar < homeWant) {
        for (int i = 0; i < MAX_CARS; i++) {
            Car *c = &cars[i];
            if (!c->active || c->type == V_BIKE || c->type == V_TRUCK || c->rogue || c->wanted || c->fleeing || c->arcing) continue;
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
                rate = 2.4f; hh = bHeight[y][x] + 0.40f; ox = 0.2f; oz = -0.2f;
            } else if (surface[y][x] == T_POWER) {
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
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (fireI[y][x] <= 0.0f) continue;
            bool covered = false;
            for (int t = 0; t < MAX_TRUCKS; t++)
                if (trucks[t].active && abs(trucks[t].targetX - x) <= 3 && abs(trucks[t].targetY - y) <= 3) covered = true;
            if (!covered) Dispatch(trucks, MAX_TRUCKS, fireStn, nFire, x, y, -1, 3, 0, 0);
        }
    for (int i = 0; i < MAX_PEDS; i++) {
        if (!(peds[i].active && peds[i].criminal && peds[i].robbing)) continue;
        bool assigned = false;
        for (int p = 0; p < MAX_POLICE; p++) if (police[p].active && police[p].mode == 0 && police[p].pedTarget == i) assigned = true;
        if (!assigned) Dispatch(police, MAX_POLICE, policeStn, nPolice, (int)peds[i].x, (int)peds[i].y, i, 3, 0, 0);
    }
}

// ---------------------------------------------------------------------------
// Heat / pollution maps, statistics, simulation driver
// ---------------------------------------------------------------------------
static void UpdateMaps(void) {
    float df = DayFactor();
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float heatT = 0.0f, pol = 0.0f;
            switch (surface[y][x]) {
                case T_BUILDING:
                    heatT = (bType[y][x] == B_IND ? 4.5f : 2.2f) * df + 0.8f;
                    pol = (bType[y][x] == B_IND) ? (deco[y][x] == 0 ? 7.0f : 4.0f) : (bType[y][x] == B_COM ? 0.9f : 0.5f);
                    break;
                case T_HOUSE: heatT = 1.4f * df + 0.4f; pol = 0.3f; break;
                case T_MALL:  heatT = 3.0f * df + 0.8f; pol = 1.2f; break;
                case T_PARK:  heatT = 1.6f; pol = 0.5f; break;
                case T_LANDFILL: heatT = 1.2f; pol = 3.5f + landfillFill * 0.03f; break;
                case T_ROAD:  heatT = 1.2f + traffic[y][x] * 2.0f; pol = 0.4f + traffic[y][x] * 4.0f; break;
                case T_TREE:  heatT = -1.8f; pol = -0.6f; break;
                case T_GRASS: heatT = (deco[y][x] ? -0.9f : -0.4f); pol = (deco[y][x] ? -0.3f : -0.15f); break;
                case T_POWER: heatT = 7.0f * st.powerLoad; pol = 9.0f * st.powerLoad; break;
                case T_RUIN:  heatT = 0.5f; break;
                default:      heatT = 1.0f; pol = 0.3f; break;
            }
            if (fireI[y][x] > 0.0f) { heatT += 12.0f * fireI[y][x] / 100.0f; pol += 7.0f * fireI[y][x] / 100.0f; }
            heatT -= rain * 2.0f;
            heatMap[y][x] += (heatT - heatMap[y][x]) * 0.25f;
            polMap[y][x] += pol;
            if (polMap[y][x] < 0.0f) polMap[y][x] = 0.0f;
        }
    Diffuse(heatMap, 0.30f, 1.0f);
    Diffuse(polMap, 0.25f, 0.965f - rain * 0.08f);
}

static void UpdateStats(void) {
    static const float eBase[3] = { 40.0f, 95.0f, 230.0f };    // kW per building type
    static const float wBase[3] = { 30.0f, 55.0f, 140.0f };    // L/min per building type
    int res = 0, houses = 0, bld = 0, dmg = 0, roads = 0, green = 0, plants = 0, works = 0, stations = 0, lit = 0;
    float eDem = 0.0f, wDem = 0.0f, df = DayFactor();

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
                } break;
                case T_HOUSE:
                    bld++; houses++;
                    if (health[y][x] < 60.0f) dmg++;
                    eDem += 18.0f * df; wDem += 14.0f * df;
                    break;
                case T_MALL:
                    bld++;
                    if (health[y][x] < 60.0f) dmg++;
                    eDem += 160.0f * (0.5f + 0.5f * ShopFactor()); wDem += 80.0f * ShopFactor();
                    break;
                case T_PARK: lit++; break;
                case T_LANDFILL: wDem += 6.0f; break;
                case T_TREE: case T_GRASS: green++; break;
                case T_POWER: plants++; break;
                case T_WATER: works++; break;
                case T_FIRE_STN: case T_POLICE_STN: stations++; break;
                default: break;
            }
        }
    eDem += stations * 30.0f + (roads * 1.6f + lit * 4.0f) * (1.0f - st.daylight);   // street + car park lighting at night
    // traffic signals draw a little power as well
    for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) if (ctrl[y][x] == C_SIGNAL) eDem += 3.0f;
    wDem += stations * 20.0f;

    st.buildings = bld; st.damaged = dmg;
    st.energyDemand = eDem; st.energyCap = plants * PLANT_KW;
    st.energyPct = (st.energyCap > 0.0f) ? eDem / st.energyCap * 100.0f : 999.0f;
    st.powerLoad = (st.energyCap > 0.0f) ? Clamp(eDem / st.energyCap, 0.0f, 1.0f) : 0.0f;
    st.waterDemand = wDem; st.waterCap = works * WORKS_LPM;
    st.waterPct = (st.waterCap > 0.0f) ? wDem / st.waterCap * 100.0f : 999.0f;
    st.sewerLoad = wDem * 0.85f + rain * roads * 35.0f;     // waste water + storm run-off
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

    landfillFill = fminf(100.0f, landfillFill + st.population * 0.000012f * 0.5f);   // waste from the population

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
    UpdatePeds(dt);          // first, so cars can see who is on a crossing
    UpdateCars(dt);
    UpdateResponders(dt);
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
static bool ZoneFenced(int z) { return z >= Z_GATED1 && z <= Z_LANDFILL; }

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
        else if (s == T_LANDFILL) base = (Color){ 112, 94, 72, 255 };
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
    if (c->wanted) {
        float f = 0.5f + 0.5f * sinf((float)GetTime() * 10.0f);
        DrawCircleLines((int)(c->x * TILE), (int)(c->y * TILE), 7.0f + 2.0f * f, c->fleeing ? RED : ORANGE);
    }
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
                }
                break;
            }
            DrawRectangle(px + 5, py + 5, TILE - 10, TILE - 10, Fade(c, gAlpha));
            DrawRectangleLines(px + 5, py + 5, TILE - 10, TILE - 10, Fade((Color){ 60, 40, 35, 255 }, gAlpha));
            Color wc = (st.daylight < 0.5f) ? (Color){ 255, 230, 120, 255 } : (Color){ 215, 225, 235, 255 };
            for (int wy = 0; wy < 2; wy++)
                for (int wx = 0; wx < 2; wx++)
                    DrawRectangle(px + 9 + wx * 10, py + 9 + wy * 10, 5, 5, Fade(wc, gAlpha));
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
        } break;
        case T_MALL: {
            DrawRectangle(px, py, TILE, TILE, Fade((Color){ 206, 198, 180, 255 }, gAlpha));
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade((Color){ 190, 182, 164, 255 }, gAlpha));
            if (y == 5) DrawRectangle(px + 4, py + TILE - 6, TILE - 8, 3, Fade((Color){ 70, 120, 170, 255 }, gAlpha));
            if (x == 20 && y == 4) DrawText("MALL", px + 4, py + 6, 10, Fade((Color){ 190, 40, 40, 255 }, gAlpha));
        } break;
        case T_PARK: DrawParkedCars2D(x, y); break;
        case T_LANDFILL: {
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
            break;
        case T_POLICE_STN:
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_POLICE_STN, gAlpha));
            DrawText("POL", px + 8, py + TILE / 2 - 4, 10, Fade(RAYWHITE, gAlpha));
            break;
        case T_POWER:
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_POWER, gAlpha));
            DrawRectangle(px + 7, py + 5, 5, 9, Fade((Color){ 30, 30, 34, 255 }, gAlpha));
            DrawRectangle(px + 20, py + 5, 5, 9, Fade((Color){ 30, 30, 34, 255 }, gAlpha));
            DrawText("PWR", px + 7, py + TILE - 15, 10, Fade(YELLOW, gAlpha));
            break;
        case T_WATER:
            DrawRectangle(px + 3, py + 3, TILE - 6, TILE - 6, Fade(COL_WORKS, gAlpha));
            DrawCircle(px + TILE / 2, py + 12, 7, Fade((Color){ 200, 230, 240, 255 }, gAlpha));
            DrawText("H2O", px + 7, py + TILE - 15, 10, Fade(RAYWHITE, gAlpha));
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

    ClearBackground(COL_GRASS);
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

    if (layerOn[L_HEAT])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, HeatColorAt(x, y));
    if (layerOn[L_POLLUTION])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, PolColorAt(x, y));

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

    for (int i = 0; i < MAX_PEDS; i++) {
        if (!peds[i].active) continue;
        Vector2 p = { peds[i].x * TILE, peds[i].y * TILE };
        DrawCircleV(p, 2.4f, peds[i].shirt);
        DrawCircleV((Vector2){ p.x, p.y - 2.2f }, 1.5f, COL_CIVILIAN);
        if (peds[i].robbing) DrawCircleLines((int)p.x, (int)p.y, 7.0f + 2.0f * sinf((float)t * 10.0f), RED);
    }
    for (int i = 0; i < MAX_TRUCKS; i++) if (trucks[i].active) DrawResponder2D(&trucks[i], COL_TRUCK, t);
    for (int i = 0; i < MAX_POLICE; i++) if (police[i].active) DrawResponder2D(&police[i], COL_POLICE_CAR, t);
    DrawParticles2D();

    if (hoverOK) DrawRectangleLines(hoverX * TILE, hoverY * TILE, TILE, TILE, WHITE);
}

// ---------------------------------------------------------------------------
// 3D drawing - vehicles (drawn in a local frame: +Z is forward, ground at y = 0)
// ---------------------------------------------------------------------------
static bool NightNow(void) { return st.daylight < 0.45f; }
static Color A(Color c) { return Fade(c, gAlpha); }
static const Color GLASS = { 45, 58, 76, 255 };

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

static void DrawVehicleLocal(int type, Color col, bool braking, bool lights) {
    float L = VLEN[type], W = VWID[type];
    Color body = A(col), glass = A(GLASS);
    switch (type) {
    case V_SALOON:
        BoxC(0, 0.046f, 0, W, 0.040f, L, body);
        BoxC(0, 0.088f, -0.015f, W * 0.86f, 0.040f, 0.13f, glass);
        BoxC(0, 0.110f, -0.015f, W * 0.84f, 0.006f, 0.115f, body);
        Wheels(W, L, 2); break;
    case V_HATCH:
        BoxC(0, 0.046f, 0, W, 0.040f, L, body);
        BoxC(0, 0.088f, -0.025f, W * 0.86f, 0.040f, 0.125f, glass);
        BoxC(0, 0.110f, -0.030f, W * 0.84f, 0.006f, 0.110f, body);
        Wheels(W, L, 2); break;
    case V_ESTATE:
        BoxC(0, 0.046f, 0, W, 0.040f, L, body);
        BoxC(0, 0.090f, -0.035f, W * 0.86f, 0.044f, 0.185f, glass);
        BoxC(0, 0.114f, -0.035f, W * 0.84f, 0.006f, 0.170f, body);
        Wheels(W, L, 2); break;
    case V_PICKUP:
        BoxC(0, 0.048f, 0.060f, W, 0.044f, 0.120f, body);
        BoxC(0, 0.090f, 0.045f, W * 0.88f, 0.040f, 0.085f, glass);
        BoxC(0, 0.112f, 0.045f, W * 0.86f, 0.006f, 0.075f, body);
        BoxC(0, 0.036f, -0.080f, W, 0.016f, 0.130f, Shade(body, 0.55f));
        BoxC(-(W * 0.5f - 0.005f), 0.058f, -0.080f, 0.010f, 0.030f, 0.130f, body);
        BoxC( (W * 0.5f - 0.005f), 0.058f, -0.080f, 0.010f, 0.030f, 0.130f, body);
        BoxC(0, 0.058f, -0.142f, W, 0.030f, 0.010f, body);
        Wheels(W, L, 2); break;
    case V_SUV:
        BoxC(0, 0.056f, 0, W, 0.056f, L, body);
        BoxC(0, 0.106f, -0.015f, W * 0.90f, 0.040f, 0.170f, glass);
        BoxC(0, 0.128f, -0.015f, W * 0.88f, 0.006f, 0.160f, body);
        BoxC(-W * 0.32f, 0.134f, -0.015f, 0.006f, 0.006f, 0.15f, A((Color){ 60, 62, 68, 255 }));
        Wheels(W, L, 2); break;
    case V_VAN:
        BoxT(TX_VAN, 0, 0.078f, -0.025f, W, 0.112f, 0.25f, A(WHITE));
        BoxC(0, 0.056f, 0.130f, W, 0.068f, 0.045f, A((Color){ 235, 235, 235, 255 }));
        BoxC(0, 0.094f, 0.108f, W * 0.9f, 0.036f, 0.012f, glass);
        Wheels(W, L, 2); break;
    case V_TRUCK:
        BoxC(0, 0.035f, -0.03f, W * 0.8f, 0.030f, 0.40f, A((Color){ 40, 40, 44, 255 }));
        BoxC(0, 0.070f, 0.170f, W, 0.090f, 0.100f, A((Color){ 235, 235, 235, 255 }));
        BoxC(0, 0.086f, 0.222f, W * 0.9f, 0.036f, 0.010f, glass);
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

static void DrawBike3D(const Car *c) {
    PushFrame(c->x, 0.0f, c->y, c->yaw);
    Color frame = A(c->col), tyre = A((Color){ 25, 25, 28, 255 });
    BoxC(0, 0.025f, 0.050f, 0.012f, 0.040f, 0.040f, tyre);
    BoxC(0, 0.025f, -0.050f, 0.012f, 0.040f, 0.040f, tyre);
    BoxC(0, 0.040f, 0.0f, 0.008f, 0.008f, 0.100f, frame);
    BoxC(0, 0.052f, 0.040f, 0.008f, 0.030f, 0.008f, frame);
    float sw = sinf((float)GetTime() * 9.0f + c->id) * 0.015f;
    BoxC(0, 0.056f + sw * 0.2f, -0.030f, 0.018f, 0.006f, 0.030f, A((Color){ 30, 30, 30, 255 }));
    rlPushMatrix(); rlRotatef(-22.0f, 1, 0, 0);                           // rider leans forward
    BoxC(0, 0.092f, -0.036f, 0.034f, 0.050f, 0.022f, A(SHIRT_COLS[c->id % 8]));
    rlPopMatrix();
    DrawSphere((Vector3){ 0, 0.128f, -0.010f }, 0.017f, A((Color){ 235, 190, 160, 255 }));
    BoxC(0, 0.140f, -0.010f, 0.030f, 0.012f, 0.030f, A((c->id & 1) ? (Color){ 230, 60, 60, 255 } : (Color){ 60, 160, 230, 255 }));   // helmet
    BoxC(-0.008f, 0.040f + sw, 0.0f, 0.008f, 0.040f, 0.012f, A((Color){ 50, 60, 100, 255 }));
    BoxC( 0.008f, 0.040f - sw, 0.0f, 0.008f, 0.040f, 0.012f, A((Color){ 50, 60, 100, 255 }));
    PopFrame();
}

static void DrawCar3D(const Car *c) {
    if (c->type == V_BIKE) { DrawBike3D(c); return; }
    PushFrame(c->x, 0.0f, c->y, c->yaw);
    DrawVehicleLocal(c->type, c->col, c->braking, NightNow());
    if (c->wanted) {
        float f = 0.5f + 0.5f * sinf((float)GetTime() * 10.0f);
        DrawSphere((Vector3){ 0, 0.18f, 0 }, 0.025f + 0.01f * f, c->fleeing ? RED : ORANGE);
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
    rlScalef(sc, sc, sc);
    switch (style) {
    case 0:       // oak
        BoxT(TX_BARK, 0, 0.14f, 0, 0.07f, 0.28f, 0.07f, A(WHITE));
        DrawSphere((Vector3){ 0, 0.40f, 0 }, 0.20f, A(g1));
        DrawSphere((Vector3){ 0.11f, 0.33f, 0.07f }, 0.14f, A(g3));
        DrawSphere((Vector3){ -0.10f, 0.35f, -0.08f }, 0.15f, A(g2));
        DrawSphere((Vector3){ 0, 0.54f, 0.02f }, 0.13f, A(g2));
        break;
    case 1:       // pine
        BoxT(TX_BARK, 0, 0.06f, 0, 0.05f, 0.12f, 0.05f, A(WHITE));
        DrawCylinder((Vector3){ 0, 0.08f, 0 }, 0.0f, 0.19f, 0.20f, 9, A(Shade(g3, 0.8f)));
        DrawCylinder((Vector3){ 0, 0.20f, 0 }, 0.0f, 0.15f, 0.20f, 9, A(Shade(g3, 0.95f)));
        DrawCylinder((Vector3){ 0, 0.32f, 0 }, 0.0f, 0.11f, 0.20f, 9, A(g3));
        DrawCylinder((Vector3){ 0, 0.43f, 0 }, 0.0f, 0.065f, 0.17f, 9, A(Shade(g3, 1.1f)));
        break;
    case 2:       // tall poplar
        BoxT(TX_BARK, 0, 0.07f, 0, 0.05f, 0.14f, 0.05f, A(WHITE));
        rlPushMatrix(); rlTranslatef(0, 0.40f, 0); rlScalef(0.55f, 1.9f, 0.55f);
        DrawSphere((Vector3){ 0, 0, 0 }, 0.14f, A(g2));
        rlPopMatrix();
        break;
    case 3:       // birch
        BoxC(0, 0.15f, 0, 0.045f, 0.30f, 0.045f, A((Color){ 228, 226, 214, 255 }));
        BoxC(0, 0.12f, 0.0235f, 0.02f, 0.01f, 0.004f, A((Color){ 40, 40, 40, 255 }));
        DrawSphere((Vector3){ 0, 0.38f, 0 }, 0.15f, A(Shade(g2, 1.1f)));
        DrawSphere((Vector3){ 0.09f, 0.31f, 0.04f }, 0.10f, A(g2));
        DrawSphere((Vector3){ -0.08f, 0.45f, -0.03f }, 0.10f, A(Shade(g1, 1.15f)));
        break;
    default:      // blossom tree
        BoxT(TX_BARK, 0, 0.12f, 0, 0.06f, 0.24f, 0.06f, A(WHITE));
        DrawSphere((Vector3){ 0, 0.34f, 0 }, 0.18f, A((Color){ 245, 175, 195, 255 }));
        DrawSphere((Vector3){ 0.10f, 0.30f, 0.05f }, 0.12f, A((Color){ 250, 200, 215, 255 }));
        DrawSphere((Vector3){ -0.09f, 0.40f, -0.04f }, 0.11f, A((Color){ 235, 150, 180, 255 }));
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
            DrawSphere((Vector3){ cx + ox, 0.05f, cz + oz }, 0.06f + 0.01f * k, (k & 1) ? sh1 : sh2);
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
            DrawSphere((Vector3){ cx + ox, 0.05f, cz + oz }, 0.022f, A(fc));
            BoxC(cx + ox, 0.03f, cz + oz, 0.004f, 0.03f, 0.004f, sh1);
        }
    }
}

static void DrawGrassTufts(int x, int y) {
    unsigned char v = vr[y][x];
    for (int k = 0; k < 5; k++) {
        float ox = Hash01(x, y, k) * 0.84f + 0.08f, oz = Hash01(y, x, k + 9) * 0.84f + 0.08f;
        Color g = Shade((Color){ 64, 150, 58, 255 }, 0.8f + Hash01(k, x + y, 3) * 0.5f);
        DrawCylinder((Vector3){ x + ox, 0.03f, y + oz }, 0.0f, 0.016f, 0.05f + 0.03f * ((v >> k) & 1), 4, A(g));
    }
}

// ---------------------------------------------------------------------------
// 3D drawing - houses in the gated communities
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

    // main block, roof, garage
    BoxT(wallTex, -0.05f, 0.10f, -0.08f, 0.34f, 0.20f, 0.32f, wall);
    if (rtype == 0)      RoofGable(0.40f, 0.38f, 0.20f, 0.12f, false, roof, wallTex, wall);
    else if (rtype == 1) RoofGable(0.40f, 0.38f, 0.20f, 0.13f, true, roof, wallTex, wall);
    else if (rtype == 2) RoofHip(0.40f, 0.38f, 0.20f, 0.12f, roof);
    else { BoxC(-0.05f, 0.215f, -0.08f, 0.38f, 0.03f, 0.36f, Shade(roof, 0.8f)); }
    if (rtype != 3) { rlPushMatrix(); rlTranslatef(-0.05f, 0, -0.08f); rlPopMatrix(); }
    if (v & 1) BoxT(TX_BRICK, -0.16f, 0.30f, -0.15f, 0.05f, 0.12f, 0.05f, A(WHITE));       // chimney
    BoxT(TX_STUCCO, 0.26f, 0.07f, -0.04f, 0.16f, 0.14f, 0.26f, wall);                       // garage
    BoxC(0.26f, 0.148f, -0.04f, 0.18f, 0.02f, 0.28f, Shade(roof, 0.85f));
    BoxC(0.26f, 0.05f, 0.0925f, 0.12f, 0.09f, 0.006f, A((Color){ 225, 225, 228, 255 }));    // garage door
    for (int k = 0; k < 3; k++) BoxC(0.26f, 0.025f + k * 0.03f, 0.096f, 0.12f, 0.003f, 0.002f, A((Color){ 170, 170, 175, 255 }));

    // front door + windows (lit at night)
    float zf = 0.0825f;
    BoxC(-0.12f, 0.05f, zf, 0.05f, 0.09f, 0.006f, A((Color){ 90 + (v & 63), 60, 45, 255 }));
    Color win = lit ? (Color){ 255, 226, 130, 255 } : (Color){ 70, 100, 130, 255 };
    BoxC(-0.02f, 0.12f, zf, 0.075f, 0.065f, 0.006f, A(win));
    BoxC(-0.12f, 0.13f, zf, 0.050f, 0.050f, 0.006f, A(win));
    BoxC(-0.02f, 0.12f, zf + 0.002f, 0.004f, 0.065f, 0.002f, A(WHITE));
    BoxC(-0.20f, 0.12f, -0.08f, 0.006f, 0.060f, 0.07f, A(win));

    // driveway, parked car, garden
    BoxC(0.26f, 0.006f, 0.30f, 0.15f, 0.012f, 0.42f, A((Color){ 165, 165, 168, 255 }));
    BoxC(-0.20f, 0.004f, 0.30f, 0.07f, 0.008f, 0.38f, A((Color){ 205, 195, 175, 255 }));      // garden path
    if (drive[y][x]) {
        PushFrame(0.26f, 0.008f, 0.30f, 0.0f);
        DrawVehicleLocal(drive[y][x] - 1, CAR_COLS[drvCol[y][x]], false, NightNow());
        PopFrame();
    }
    DrawSphere((Vector3){ -0.33f, 0.04f, 0.28f }, 0.05f, A((Color){ 48, 125, 55, 255 }));
    DrawSphere((Vector3){ -0.27f, 0.035f, 0.40f }, 0.04f, A((Color){ 62, 145, 62, 255 }));
    for (int k = 0; k < 4; k++)
        DrawSphere((Vector3){ -0.40f + k * 0.045f, 0.022f, 0.12f }, 0.014f, A(k & 1 ? (Color){ 240, 90, 120, 255 } : (Color){ 250, 215, 70, 255 }));
    if (v & 2) {          // back garden tree
        BoxT(TX_BARK, -0.30f, 0.07f, -0.36f, 0.04f, 0.14f, 0.04f, A(WHITE));
        DrawSphere((Vector3){ -0.30f, 0.20f, -0.36f }, 0.11f, A((Color){ 55, 140, 55, 255 }));
    } else if (v & 32) {  // swimming pool
        BoxC(-0.12f, 0.004f, -0.36f, 0.26f, 0.008f, 0.17f, A((Color){ 225, 225, 225, 255 }));
        BoxC(-0.12f, 0.009f, -0.36f, 0.22f, 0.006f, 0.13f, A((Color){ 70, 165, 220, 255 }));
    }
    if (v & 16) BoxT(TX_CORRUG, 0.35f, 0.04f, -0.40f, 0.10f, 0.08f, 0.09f, A(WHITE));       // shed
    BoxT(TX_HEDGE, 0, 0.035f, -0.47f, 0.90f, 0.07f, 0.05f, A(WHITE));                       // back hedge
    PopFrame();
}

// ---------------------------------------------------------------------------
// 3D drawing - other buildings
// ---------------------------------------------------------------------------
static int RoadSide(int x, int y) {          // direction of the nearest road (for entrances)
    for (int d = 0; d < 4; d++) if (IsRoad(x + DX[d], y + DY[d])) return d;
    return 1;
}

static void DrawBuilding3D(int x, int y) {
    unsigned char v = vr[y][x];
    float h = bHeight[y][x], cx = x + 0.5f, cz = y + 0.5f;
    int fl = (int)(h * 10.0f + 0.5f);
    if (fl < 1) fl = 1;
    bool dmg = health[y][x] < 50.0f, night = NightNow();
    Color tint = A(dmg ? (Color){ 110, 90, 85, 255 } : (Color){ (unsigned char)(235 + (v & 15)), (unsigned char)(235 + ((v >> 2) & 15)), 255, 255 });
    int side = RoadSide(x, y);
    int wt = (bType[y][x] == B_COM) ? (fl >= 8 ? (night ? TX_WIN_GLASS_N : TX_WIN_GLASS) : (night ? TX_WIN_OFFICE_N : TX_WIN_OFFICE))
                                    : (night ? TX_WIN_BRICK_N : TX_WIN_BRICK);
    PushFrame(cx, 0, cz, FaceYaw(side));
    if (fl >= 10) {                                   // skyrise: podium, tower, crown
        int f1 = fl * 35 / 100; if (f1 < 2) f1 = 2;
        int f2 = fl - f1;
        float h1 = f1 * 0.1f, h2 = f2 * 0.1f;
        WindowBox(wt, f1, 0, h1 / 2, 0, 0.72f, h1, 0.72f, tint);
        WindowBox(wt, f2, 0, h1 + h2 / 2, 0, 0.56f, h2, 0.56f, tint);
        BoxC(0, h1 + 0.005f, 0, 0.74f, 0.012f, 0.74f, A((Color){ 120, 122, 128, 255 }));        // podium ledge
        BoxC(0, h + 0.02f, 0, 0.60f, 0.04f, 0.60f, A((Color){ 105, 108, 115, 255 }));           // crown
        BoxC(0.12f, h + 0.07f, 0.1f, 0.14f, 0.06f, 0.12f, A((Color){ 150, 152, 158, 255 }));    // plant room
        BoxC(-0.1f, h + 0.18f, -0.1f, 0.012f, 0.28f, 0.012f, A((Color){ 200, 200, 205, 255 }));  // mast
        if (night || (int)(GetTime() * 1.2f) % 2) DrawSphere((Vector3){ -0.1f, h + 0.33f, -0.1f }, 0.016f, A(RED));
        BoxC(0, 0.04f, 0.37f, 0.30f, 0.08f, 0.02f, A((Color){ 40, 50, 60, 255 }));              // lobby doors
    } else {
        WindowBox(wt, fl, 0, h / 2, 0, 0.74f, h, 0.74f, tint);
        if (bType[y][x] == B_RES && fl <= 4 && (v & 1)) RoofGable(0.78f, 0.78f, h, 0.14f, (v & 2) != 0, A((Color){ 150, 100, 90, 255 }), TX_BRICK, A(WHITE));
        else BoxC(0, h + 0.01f, 0, 0.78f, 0.03f, 0.78f, A((Color){ 112, 114, 120, 255 }));
        BoxC(-0.15f, h + 0.05f, 0.1f, 0.12f, 0.05f, 0.14f, A((Color){ 150, 152, 158, 255 }));
        if (bType[y][x] == B_COM && fl < 8) {          // shop awning
            Color aw = (v & 1) ? (Color){ 200, 60, 60, 255 } : (Color){ 60, 120, 190, 255 };
            BoxC(0, 0.11f, 0.42f, 0.66f, 0.02f, 0.10f, A(aw));
        }
        BoxC(0, 0.04f, 0.375f, 0.20f, 0.08f, 0.012f, A((Color){ 45, 50, 60, 255 }));
    }
    PopFrame();
}

static void DrawIndustrial3D(int x, int y) {
    unsigned char v = vr[y][x];
    float h = bHeight[y][x], cx = x + 0.5f, cz = y + 0.5f;
    bool dmg = health[y][x] < 50.0f;
    Color tint = A(dmg ? (Color){ 120, 100, 95, 255 } : WHITE);
    PushFrame(cx, 0, cz, FaceYaw(RoadSide(x, y)));
    if (deco[y][x] == 0) {              // factory with chimney and vents
        BoxT(TX_CORRUG, 0, h / 2, 0, 0.84f, h, 0.72f, tint);
        for (int k = 0; k < 3; k++) RoofGable(0.26f, 0.72f, h, 0.07f, true, A((Color){ 140, 145, 150, 255 }), TX_CORRUG, tint);
        BoxC(-0.28f, h + 0.04f, 0.0f, 0.12f, 0.08f, 0.12f, A((Color){ 100, 104, 110, 255 }));
        DrawCylinder((Vector3){ 0.24f, h, -0.20f }, 0.065f, 0.08f, 0.22f, 12, A((Color){ 150, 70, 55, 255 }));
        DrawCylinder((Vector3){ 0.24f, h + 0.22f, -0.20f }, 0.055f, 0.065f, 0.16f, 12, A((Color){ 235, 235, 235, 255 }));
        DrawCylinder((Vector3){ 0.24f, h + 0.38f, -0.20f }, 0.050f, 0.055f, 0.06f, 12, A((Color){ 200, 50, 45, 255 }));
        BoxC(0.0f, 0.06f, 0.365f, 0.30f, 0.12f, 0.012f, A((Color){ 70, 78, 88, 255 }));
        BoxC(0.0f, 0.12f, 0.372f, 0.34f, 0.01f, 0.012f, A((Color){ 240, 190, 40, 255 }));
    } else if (deco[y][x] == 1) {       // warehouse with roller doors
        BoxT(TX_CORRUG, 0, h / 2, 0, 0.90f, h, 0.64f, tint);
        RoofGable(0.94f, 0.68f, h, 0.07f, false, A((Color){ 120, 128, 138, 255 }), TX_CORRUG, tint);
        for (int k = 0; k < 3; k++) {
            BoxC(-0.28f + k * 0.28f, 0.09f, 0.322f, 0.20f, 0.18f, 0.008f, A((Color){ 108, 118, 130, 255 }));
            for (int j = 0; j < 4; j++) BoxC(-0.28f + k * 0.28f, 0.03f + j * 0.04f, 0.327f, 0.20f, 0.004f, 0.002f, A((Color){ 85, 92, 102, 255 }));
        }
    } else {                            // tank farm
        DrawCylinder((Vector3){ -0.22f, 0, -0.05f }, 0.19f, 0.19f, 0.56f, 20, A((Color){ 205, 208, 212, 255 }));
        DrawCylinder((Vector3){ -0.22f, 0.56f, -0.05f }, 0.0f, 0.19f, 0.05f, 20, A((Color){ 170, 174, 180, 255 }));
        DrawCylinder((Vector3){ 0.23f, 0, 0.12f }, 0.15f, 0.15f, 0.44f, 18, A((Color){ 225, 226, 230, 255 }));
        DrawCylinder((Vector3){ 0.23f, 0.44f, 0.12f }, 0.0f, 0.15f, 0.04f, 18, A((Color){ 185, 188, 194, 255 }));
        BoxC(0.0f, 0.10f, 0.1f, 0.5f, 0.02f, 0.02f, A((Color){ 160, 80, 60, 255 }));
        BoxC(-0.22f, 0.30f, 0.14f, 0.02f, 0.5f, 0.02f, A((Color){ 120, 124, 130, 255 }));
        if (v & 1) DrawCylinder((Vector3){ 0.02f, 0, -0.36f }, 0.07f, 0.07f, 0.22f, 12, A((Color){ 230, 200, 60, 255 }));
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
    float m = 0.05f;
    bool w = (surface[y][x - 1 < 0 ? 0 : x - 1] == T_MALL) && x > 0, e = (x + 1 < MAP_W) && surface[y][x + 1] == T_MALL;
    bool n = (y > 0) && surface[y - 1][x] == T_MALL, s = (y + 1 < MAP_H) && surface[y + 1][x] == T_MALL;
    float x0 = x + (w ? 0 : m), x1 = x + 1 - (e ? 0 : m), z0 = y + (n ? 0 : m), z1 = y + 1 - (s ? 0 : m);
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
}

static void DrawParkingLot3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    for (int c = 0; c <= 4; c++)
        for (int r = 0; r < 2; r++)
            BoxC(x + c * 0.25f, 0.066f, y + 0.30f + r * 0.40f, 0.012f, 0.004f, 0.34f, A((Color){ 240, 240, 240, 255 }));
    for (int s = 0; s < 8; s++) {
        if (!lot[y][x][s]) continue;
        int col = s % 4, row = s / 4;
        PushFrame(x + (col + 0.5f) * 0.25f, 0.06f, y + 0.30f + row * 0.40f, row ? 180.0f : 0.0f);
        DrawVehicleLocal(lot[y][x][s] - 1, CAR_COLS[lotCol[y][x][s]], false, NightNow());
        PopFrame();
    }
    if (((x + y) & 1) == 0) {               // lamp posts
        BoxC(cx, 0.18f, cz, 0.014f, 0.36f, 0.014f, A((Color){ 70, 72, 78, 255 }));
        BoxC(cx, 0.37f, cz, 0.07f, 0.014f, 0.03f, A((Color){ 70, 72, 78, 255 }));
        if (NightNow()) DrawSphere((Vector3){ cx, 0.355f, cz }, 0.022f, (Color){ 255, 240, 170, 255 });
    }
}

static void DrawLandfill3D(int x, int y) {
    unsigned char v = vr[y][x];
    float fillK = 0.55f + 0.9f * landfillFill / 100.0f;
    float cx = x + 0.5f, cz = y + 0.5f;
    if (gateTile[y][x]) {                   // weighbridge + site office
        BoxC(cx, 0.10f, cz - 0.28f, 0.34f, 0.20f, 0.22f, A((Color){ 205, 205, 195, 255 }));
        BoxC(cx, 0.21f, cz - 0.28f, 0.38f, 0.03f, 0.26f, A((Color){ 90, 100, 110, 255 }));
        BoxC(cx, 0.012f, cz + 0.15f, 0.55f, 0.024f, 0.20f, A((Color){ 100, 100, 108, 255 }));
        return;
    }
    for (int k = 0; k < 3; k++) {
        float ox = (((v >> (k * 2)) & 3) - 1.5f) * 0.26f, oz = (((v >> (k + 3)) & 3) - 1.5f) * 0.26f;
        float r = (0.20f + 0.05f * ((v >> k) & 3)), hh = bHeight[y][x] * fillK * (0.8f + 0.25f * k);
        rlPushMatrix(); rlTranslatef(cx + ox, 0.0f, cz + oz); rlScalef(1.0f, hh / r * 0.8f, 1.0f);
        DrawSphere((Vector3){ 0, 0, 0 }, r, A(Shade((Color){ 98, 84, 66, 255 }, 0.8f + 0.1f * k)));
        rlPopMatrix();
    }
    for (int k = 0; k < 14; k++) {          // loose refuse
        float ox = Hash01(x, y, k) - 0.5f, oz = Hash01(y, x, k + 5) - 0.5f;
        Color c = (Color){ (unsigned char)(80 + Hash01(k, x, 3) * 170), (unsigned char)(80 + Hash01(k, y, 4) * 170), (unsigned char)(80 + Hash01(x, k, 5) * 170), 255 };
        BoxC(cx + ox * 0.9f, 0.06f + 0.14f * Hash01(k, y, x), cz + oz * 0.9f, 0.025f, 0.015f, 0.025f, A(c));
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
    Color main = fire ? (Color){ 205, 62, 55, 255 } : (Color){ 235, 238, 245, 255 };
    BoxT(TX_BRICK, 0, 0.20f, 0, 0.88f, 0.40f, 0.84f, A(fire ? (Color){ 255, 190, 180, 255 } : (Color){ 200, 215, 245, 255 }));
    BoxC(0, 0.415f, 0, 0.92f, 0.03f, 0.88f, A(Shade(main, 0.7f)));
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
    PopFrame();
}

static void DrawPlant3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    BoxT(TX_CONCRETE, cx, 0.15f, cz, 0.92f, 0.30f, 0.92f, A(WHITE));
    for (int k = 0; k < 2; k++) {
        float ox = -0.22f + k * 0.44f;
        DrawCylinder((Vector3){ cx + ox, 0.30f, cz - 0.1f }, 0.07f, 0.09f, 0.45f, 12, A((Color){ 215, 215, 220, 255 }));
        DrawCylinder((Vector3){ cx + ox, 0.75f, cz - 0.1f }, 0.065f, 0.07f, 0.08f, 12, A((Color){ 200, 55, 50, 255 }));
        DrawCylinder((Vector3){ cx + ox, 0.83f, cz - 0.1f }, 0.06f, 0.065f, 0.10f, 12, A((Color){ 235, 235, 238, 255 }));
    }
    DrawCylinder((Vector3){ cx, 0.30f, cz + 0.22f }, 0.15f, 0.20f, 0.30f, 20, A((Color){ 190, 190, 188, 255 }));   // cooling tower
    for (int k = 0; k < 3; k++) BoxC(cx - 0.3f + k * 0.18f, 0.34f, cz + 0.38f, 0.1f, 0.08f, 0.06f, A((Color){ 120, 125, 135, 255 }));
}
static void DrawWorks3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    BoxT(TX_CONCRETE, cx, 0.12f, cz, 0.92f, 0.24f, 0.92f, A((Color){ 190, 225, 235, 255 }));
    DrawCylinder((Vector3){ cx - 0.2f, 0.24f, cz }, 0.22f, 0.22f, 0.20f, 20, A((Color){ 70, 150, 175, 255 }));
    DrawCylinder((Vector3){ cx + 0.25f, 0.24f, cz - 0.2f }, 0.04f, 0.05f, 0.45f, 10, A((Color){ 180, 185, 190, 255 }));
    DrawSphere((Vector3){ cx + 0.25f, 0.72f, cz - 0.2f }, 0.14f, A((Color){ 70, 150, 200, 255 }));        // water tower
    BoxC(cx + 0.2f, 0.28f, cz + 0.26f, 0.3f, 0.04f, 0.3f, A((Color){ 70, 120, 200, 255 }));
}

static void DrawStructure3D(int x, int y) {
    switch (surface[y][x]) {
        case T_BUILDING: if (bType[y][x] == B_IND) DrawIndustrial3D(x, y); else DrawBuilding3D(x, y); break;
        case T_HOUSE:    DrawHouse3D(x, y); break;
        case T_MALL:     DrawMall3D(x, y); break;
        case T_RUIN:     DrawRuin3D(x, y); break;
        case T_FIRE_STN: DrawStation3D(x, y, true); break;
        case T_POLICE_STN: DrawStation3D(x, y, false); break;
        case T_POWER:    DrawPlant3D(x, y); break;
        case T_WATER:    DrawWorks3D(x, y); break;
        case T_LANDFILL: DrawLandfill3D(x, y); break;
        case T_PARK:     DrawParkingLot3D(x, y); break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// 3D drawing - fences, gates, traffic lights, junction furniture
// ---------------------------------------------------------------------------
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
                float inset = 0.5f - 0.17f;
                PushFrame(x + 0.5f, 0, y + 0.5f, FaceYaw(d));
                if (gate) {
                    Color pil = A((Color){ 150, 140, 125, 255 });
                    for (int s = -1; s <= 1; s += 2) {
                        BoxT(TX_BRICK, s * 0.30f, 0.10f, inset, 0.07f, 0.20f, 0.07f, A(WHITE));
                        BoxC(s * 0.30f, 0.205f, inset, 0.09f, 0.012f, 0.09f, pil);
                        if (NightNow()) DrawSphere((Vector3){ s * 0.30f, 0.23f, inset }, 0.018f, (Color){ 255, 235, 160, 255 });
                    }
                    BoxC(-0.26f, 0.09f, inset, 0.16f, 0.012f, 0.012f, A((Color){ 235, 190, 40, 255 }));          // gate leaves (open)
                    BoxC(-0.26f, 0.05f, inset, 0.16f, 0.012f, 0.012f, A((Color){ 235, 190, 40, 255 }));
                    BoxC(0.0f, 0.05f, inset, 0.50f, 0.01f, 0.01f, A(((int)(GetTime() * 1.0) % 2) ? RED : WHITE));   // barrier arm
                    BoxC(0.0f, 0.03f, inset, 0.01f, 0.06f, 0.01f, A(DARKGRAY));
                } else if (picket) {
                    BoxC(0, 0.040f, inset, 1.0f, 0.012f, 0.010f, A(WHITE));
                    BoxC(0, 0.070f, inset, 1.0f, 0.012f, 0.010f, A(WHITE));
                    for (int k = 0; k < 9; k++) BoxC(-0.48f + k * 0.12f, 0.050f, inset, 0.022f, 0.10f, 0.010f, A((Color){ 248, 248, 244, 255 }));
                } else {
                    Color post = A((Color){ 90, 94, 100, 255 });
                    for (int k = 0; k < 3; k++) BoxC(-0.5f + k * 0.5f, 0.07f, inset, 0.015f, 0.14f, 0.015f, post);
                    BoxC(0, 0.075f, inset, 1.0f, 0.10f, 0.004f, A((Color){ 175, 180, 188, 130 }));
                    BoxC(0, 0.14f, inset, 1.0f, 0.008f, 0.008f, post);
                    BoxC(0, 0.155f, inset, 1.0f, 0.004f, 0.004f, A((Color){ 60, 60, 64, 255 }));
                }
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
    DrawSphere((Vector3){ 0, 0.235f, 0.030f }, 0.011f, A(r));
    DrawSphere((Vector3){ 0, 0.205f, 0.030f }, 0.011f, A(y));
    DrawSphere((Vector3){ 0, 0.175f, 0.030f }, 0.011f, A(g));
    PopFrame();
}

static void DrawJunction3D(int x, int y, bool xray) {
    float cx = x + 0.5f, cz = y + 0.5f;
    if (ctrl[y][x] == C_ROUNDABOUT) {
        DrawCylinder((Vector3){ cx, 0.058f, cz }, 0.41f, 0.41f, 0.004f, 36, A((Color){ 190, 190, 192, 255 }));
        DrawCylinder((Vector3){ cx, 0.061f, cz }, 0.37f, 0.37f, 0.004f, 36, A(xray ? COL_ROAD_DIM : COL_ROAD));
        DrawCylinder((Vector3){ cx, 0.06f, cz }, 0.215f, 0.215f, 0.05f, 28, A((Color){ 160, 160, 162, 255 }));
        DrawCylinder((Vector3){ cx, 0.11f, cz }, 0.20f, 0.20f, 0.02f, 28, A((Color){ 70, 140, 65, 255 }));
        DrawSphere((Vector3){ cx + 0.05f, 0.17f, cz }, 0.07f, A((Color){ 48, 125, 55, 255 }));
        DrawSphere((Vector3){ cx - 0.06f, 0.16f, cz + 0.05f }, 0.055f, A((Color){ 62, 145, 62, 255 }));
        for (int k = 0; k < 6; k++) {
            float a = k * PI / 3.0f;
            DrawSphere((Vector3){ cx + 0.13f * cosf(a), 0.15f, cz + 0.13f * sinf(a) }, 0.016f, A(k & 1 ? (Color){ 240, 90, 120, 255 } : (Color){ 250, 215, 70, 255 }));
        }
    } else if (ctrl[y][x] == C_SIGNAL) {
        // pole at each corner, head facing the traffic that approaches it
        static const float px[4] = { -0.45f, 0.45f, 0.45f, -0.45f };
        static const float pz[4] = { -0.45f, -0.45f, 0.45f, 0.45f };
        static const float yaw[4] = { -90.0f, 180.0f, 90.0f, 0.0f };
        static const int axisOf[4] = { 1, 0, 1, 0 };
        for (int i = 0; i < 4; i++) DrawSignalHead(cx + px[i], cz + pz[i], yaw[i], SigState(x, y, axisOf[i], NULL));
    } else if (ctrl[y][x] == C_STOP) {
        static const float px[4] = { -0.45f, 0.45f, 0.45f, -0.45f };
        static const float pz[4] = { -0.45f, -0.45f, 0.45f, 0.45f };
        static const float yaw[4] = { -90.0f, 180.0f, 90.0f, 0.0f };
        for (int i = 0; i < 4; i++) {
            PushFrame(cx + px[i], 0, cz + pz[i], yaw[i]);
            BoxC(0, 0.08f, 0, 0.010f, 0.16f, 0.010f, A((Color){ 150, 150, 155, 255 }));
            rlPushMatrix(); rlTranslatef(0, 0.18f, 0.008f); rlRotatef(90.0f, 1, 0, 0);
            DrawCylinder((Vector3){ 0, 0, 0 }, 0.038f, 0.038f, 0.006f, 8, A(RED));
            rlPopMatrix();
            BoxC(0, 0.18f, 0.012f, 0.040f, 0.008f, 0.002f, A(WHITE));
            PopFrame();
        }
    }
}

// zebra crossings, stop lines
static void DrawRoadMarkings3D(int x, int y) {
    float cx = x + 0.5f, cz = y + 0.5f;
    if (crossT[y][x]) {
        for (int i = 0; i < 5; i++) {
            float o = (-0.36f + i * 0.18f);
            if (crossT[y][x] == 1) BoxC(cx + o, 0.063f, cz, 0.07f, 0.004f, 0.30f, A(WHITE));
            else                   BoxC(cx, 0.063f, cz + o, 0.30f, 0.004f, 0.07f, A(WHITE));
        }
        int ji = crossI[y][x];
        if (ji >= 0 && ctrl[ji / MAP_W][ji % MAP_W] != C_NONE) {
            for (int d = 0; d < 4; d++) {
                if (x + DX[d] != ji % MAP_W || y + DY[d] != ji / MAP_W) continue;
                float lane = 0.17f;
                float lx = DY[d] * lane, lz = -DX[d] * lane;                    // lane of traffic heading toward the junction
                float bx = cx - DX[d] * 0.26f + lx, bz = cz - DY[d] * 0.26f + lz;
                if (d < 2) BoxC(bx, 0.063f, bz, 0.30f, 0.004f, 0.025f, A(WHITE));
                else       BoxC(bx, 0.063f, bz, 0.025f, 0.004f, 0.30f, A(WHITE));
            }
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
    oc.target = (Vector3){ MAP_W / 2.0f, 0.0f, MAP_H / 2.0f };
    oc.yaw = 0.0f; oc.pitch = 1.0f; oc.dist = 24.0f;
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
    oc.dist = Clamp(oc.dist, 4.0f, 40.0f);
    oc.target.x = Clamp(oc.target.x, -3.0f, MAP_W + 3.0f);
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
            DrawCube((Vector3){ ax, yh, az }, s, s, s, col);
            if (InBounds(x + 1, y) && pipeNode[y][x + 1] && (IsRoad(x, y) || IsRoad(x + 1, y)))
                DrawCube((Vector3){ ax + 0.5f, yh, az }, 1.0f, 0.06f, 0.06f, col);
            if (InBounds(x, y + 1) && pipeNode[y + 1][x] && (IsRoad(x, y) || IsRoad(x, y + 1)))
                DrawCube((Vector3){ ax, yh, az + 0.5f }, 0.06f, 0.06f, 1.0f, col);
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
static void DrawPed3D(const Ped *p) {
    float yaw = 0.0f;
    bool walking = (p->wi < p->wn) && !(p->crossing && p->wi == 1 && !p->started) && !p->robbing;
    if (p->wi < p->wn) yaw = atan2f(p->wx[p->wi] - p->x, p->wy[p->wi] - p->y) * RAD2DEG;
    float sw = walking ? sinf(p->phase) * 30.0f : 0.0f;
    Color shirt = A(p->shirt), pants = A(p->pants), skin = A((Color){ 235, 190, 160, 255 });
    static const Color hair[4] = { {40,30,25,255}, {95,65,40,255}, {200,170,90,255}, {150,150,155,255} };
    PushFrame(p->x, 0.0f, p->y, yaw);
    for (int s = -1; s <= 1; s += 2) {
        rlPushMatrix(); rlTranslatef(s * 0.011f, 0.052f, 0); rlRotatef(s * sw, 1, 0, 0);
        BoxC(0, -0.026f, 0, 0.016f, 0.052f, 0.018f, pants);
        BoxC(0, -0.050f, 0.004f, 0.017f, 0.008f, 0.026f, A((Color){ 30, 28, 30, 255 }));
        rlPopMatrix();
        rlPushMatrix(); rlTranslatef(s * 0.027f, 0.104f, 0); rlRotatef(-s * sw, 1, 0, 0);
        BoxC(0, -0.024f, 0, 0.011f, 0.048f, 0.013f, shirt);
        rlPopMatrix();
    }
    BoxC(0, 0.080f, 0, 0.040f, 0.058f, 0.024f, shirt);
    DrawSphere((Vector3){ 0, 0.126f, 0 }, 0.017f, skin);
    BoxC(0, 0.139f, -0.002f, 0.030f, 0.010f, 0.030f, A(hair[p->look & 3]));
    if ((p->look >> 2) & 1) BoxC(0, 0.145f, 0.004f, 0.034f, 0.008f, 0.040f, A(shirt));           // cap
    if (p->criminal) BoxC(0, 0.126f, 0.015f, 0.032f, 0.008f, 0.005f, A((Color){ 20, 20, 20, 255 }));   // mask
    if (p->robbing) BoxC(0, 0.20f + 0.02f * sinf((float)GetTime() * 10.0f), 0, 0.02f, 0.05f, 0.02f, RED);
    PopFrame();
}

static void DrawPoliceCar3D(const Responder *r, double t) {
    PushFrame(r->x, 0, r->y, 90.0f - r->angle);
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
    PushFrame(r->x, 0, r->y, 90.0f - r->angle);
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
        DrawCylinderEx(a, b, 0.012f, 0.03f, 6, (Color){ 140, 200, 255, 170 });
        DrawSphere(b, 0.07f, (Color){ 200, 230, 255, 120 });
    }
}

static void DrawLandfillLife3D(void) {
    float t = (float)GetTime();
    for (int k = 0; k < 2; k++) {               // bulldozers shoving rubbish
        float a = t * (0.22f + 0.05f * k) + k * 3.0f;
        float px = 21.5f + cosf(a) * (1.4f - 0.3f * k), pz = 11.5f + sinf(a * 1.3f) * 0.6f;
        float dx = -sinf(a) * (1.4f - 0.3f * k), dz = cosf(a * 1.3f) * 1.3f * 0.6f;
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
            rlPushMatrix(); rlTranslatef(s * 0.012f, 0, 0); rlRotatef(s * flap, 0, 0, 1);
            BoxC(s * 0.03f, 0, 0, 0.06f, 0.004f, 0.025f, (Color){ 235, 235, 240, 255 });
            rlPopMatrix();
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
static void DrawStreetLamp(int x, int y) {
    int d = RoadSide(x, y);
    if (!IsRoad(x + DX[d], y + DY[d])) return;
    float px = x + 0.5f + DX[d] * 0.40f, pz = y + 0.5f + DY[d] * 0.40f;
    BoxC(px, 0.14f, pz, 0.010f, 0.28f, 0.010f, A((Color){ 70, 72, 78, 255 }));
    BoxC(px - DX[d] * 0.03f, 0.285f, pz - DY[d] * 0.03f, DX[d] ? 0.07f : 0.012f, 0.012f, DY[d] ? 0.07f : 0.012f, A((Color){ 70, 72, 78, 255 }));
    if (NightNow()) DrawSphere((Vector3){ px - DX[d] * 0.06f, 0.275f, pz - DY[d] * 0.06f }, 0.017f, (Color){ 255, 240, 170, 255 });
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
        if (floodLevel > 0.02f) rt = BlendOver(rt, (Color){ 80, 140, 255, (unsigned char)(floodLevel * 130.0f) });
        BoxT(TX_ASPHALT, cx, 0.03f, cz, 1.0f, 0.06f, 1.0f, rt);
        int ln = RoadLine(x, y);
        Color lc = xray ? (Color){ 150, 150, 150, 255 } : COL_LINE;
        if (ln == 1) BoxC(cx, 0.066f, cz, 0.5f, 0.012f, 0.045f, lc);
        else if (ln == 2) BoxC(cx, 0.066f, cz, 0.045f, 0.012f, 0.5f, lc);
        DrawRoadMarkings3D(x, y);
        return;
    }
    if (xray)                BoxC(cx, 0.015f, cz, 1.0f, 0.03f, 1.0f, COL_SOIL);
    else if (s == T_PARK)    BoxT(TX_ASPHALT, cx, 0.03f, cz, 1.0f, 0.06f, 1.0f, WHITE);
    else if (s == T_LANDFILL) BoxT(TX_TRASH, cx, 0.015f, cz, 1.0f, 0.03f, 1.0f, WHITE);
    else if (paved[y][x])    BoxT(TX_CONCRETE, cx, 0.02f, cz, 1.0f, 0.04f, 1.0f, (Color){ 215, 215, 215, 255 });
    else                     BoxT(TX_GRASS, cx, 0.015f, cz, 1.0f, 0.03f, 1.0f, (s == T_HOUSE) ? (Color){ 235, 255, 235, 255 } : WHITE);
    if (layerOn[L_ROADS]) {
        Rectangle r[8];
        int n = SidewalkRects(x, y, r);
        for (int i = 0; i < n; i++)
            BoxT(TX_CONCRETE, x + r[i].x + r[i].width / 2.0f, 0.025f, y + r[i].y + r[i].height / 2.0f,
                 r[i].width, 0.05f, r[i].height, xray ? COL_SIDEWALK_DIM : (Color){ 255, 238, 200, 255 });
    }
}

static void DrawScene3D(void) {
    bool xray = layerOn[L_WATER] || layerOn[L_SEWER];
    float t = (float)GetTime();
    rlDisableBackfaceCulling();

    DrawPlane((Vector3){ MAP_W / 2.0f, -0.03f, MAP_H / 2.0f }, (Vector2){ MAP_W + 60, MAP_H + 60 }, (Color){ 58, 105, 52, 255 });

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            DrawGround3D(x, y, xray);
            float cx = x + 0.5f, cz = y + 0.5f;
            if (layerOn[L_HEAT])      BoxC(cx, 0.080f, cz, 1.0f, 0.01f, 1.0f, HeatColorAt(x, y));
            if (layerOn[L_POLLUTION]) BoxC(cx, 0.095f, cz, 1.0f, 0.01f, 1.0f, PolColorAt(x, y));
        }
    if (layerOn[L_CONGESTION] && layerOn[L_ROADS]) DrawCongestion3D();

    // Underground networks (same offsets as 2D so both views match)
    if (layerOn[L_SEWER]) DrawNetwork3D(0.11f, 0.10f, SewerColor());
    if (layerOn[L_WATER]) DrawNetwork3D(-0.11f, 0.13f, COL_WATER_PIPE);

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
                DrawStructure3D(x, y);
                if (s == T_BUILDING && bType[y][x] == B_IND && zone[y][x] == Z_INDUSTRIAL) {}
            }
            if (layerOn[L_BUILDINGS] && zone[y][x] == Z_INDUSTRIAL && surface[y][x] == T_GRASS && paved[y][x]) DrawYard3D(x, y);
            if (layerOn[L_ROADS] && pedNode[y][x] && ((x * 3 + y * 5) % 4 == 0) && surface[y][x] != T_ROAD) DrawStreetLamp(x, y);

            // heat / pollution tint on top of tall buildings
            if (layerOn[L_BUILDINGS] && IsBldg(s) && (layerOn[L_HEAT] || layerOn[L_POLLUTION])) {
                float h = bHeight[y][x];
                if (layerOn[L_HEAT])      BoxC(x + 0.5f, h / 2, y + 0.5f, 0.78f, h + 0.02f, 0.78f, HeatColorAt(x, y));
                if (layerOn[L_POLLUTION]) BoxC(x + 0.5f, h / 2, y + 0.5f, 0.79f, h + 0.03f, 0.79f, PolColorAt(x, y));
            }
            if (fireI[y][x] > 0.0f) BoxC(x + 0.5f, 0.07f, y + 0.5f, 0.85f, 0.01f, 0.85f, Fade(COL_FIRE, 0.15f + 0.3f * fireI[y][x] / 100.0f));
        }
    gAlpha = 1.0f;
    if (layerOn[L_BUILDINGS]) { DrawFences3D(); DrawLandfillLife3D(); }

    // Vehicles and people
    if (layerOn[L_TRAFFIC])
        for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) DrawCar3D(&cars[i]);
    for (int i = 0; i < MAX_TRUCKS; i++) if (trucks[i].active) DrawFireTruck3D(&trucks[i], t);
    for (int i = 0; i < MAX_POLICE; i++) if (police[i].active) DrawPoliceCar3D(&police[i], t);
    for (int i = 0; i < MAX_PEDS; i++) if (peds[i].active) DrawPed3D(&peds[i]);

    DrawParticles3D();
    if (hoverOK) DrawCubeWires((Vector3){ hoverX + 0.5f, 0.05f, hoverY + 0.5f }, 1.0f, 0.1f, 1.0f, WHITE);
    rlEnableBackfaceCulling();
}

// ---------------------------------------------------------------------------
// UI panels
// ---------------------------------------------------------------------------
static Vector2 GetScaledMouse(void);
static int uiY;

static Rectangle LayerRect(int i) { return (Rectangle){ 10, (float)(LAYER_Y0 + i * LAYER_H), PANEL_W - 20, LAYER_H - 2 }; }
static Rectangle ViewBtnRect(void) { return (Rectangle){ 10, 26, PANEL_W - 20, 18 }; }

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
    DrawText(TextFormat("View: %s   [V]", viewMode == VIEW_2D ? "2D top-down" : "3D free camera"), (int)vb.x + 8, (int)vb.y + 4, 10, SKYBLUE);

    DrawText("LAYERS  (click or press 1-8)", 12, 50, 10, GRAY);
    for (int i = 0; i < L_COUNT; i++) {
        Rectangle r = LayerRect(i);
        if (CheckCollisionPointRec(m, r)) DrawRectangleRec(r, (Color){ 40, 46, 58, 255 });
        DrawRectangleLines((int)r.x + 2, (int)r.y + 2, 11, 11, GRAY);
        if (layerOn[i]) DrawRectangle((int)r.x + 4, (int)r.y + 4, 7, 7, LAYER_COL[i]);
        DrawText(LAYER_NAMES[i], (int)r.x + 20, (int)r.y + 3, 10, layerOn[i] ? RAYWHITE : GRAY);
        DrawText(TextFormat("[%d]", i + 1), (int)(r.x + r.width) - 22, (int)r.y + 3, 10, DARKGRAY);
    }

    DrawText("CITY STATISTICS", 12, LAYER_Y0 + L_COUNT * LAYER_H + 2, 10, GRAY);
    uiY = LAYER_Y0 + L_COUNT * LAYER_H + 15;
    int mins = (int)((st.hour - (int)st.hour) * 60.0f);
    StatRow("Time", TextFormat("Day %d  %02d:%02d  x%.1f%s", st.day, (int)st.hour, mins, simSpeed, paused ? " PAUSED" : ""), RAYWHITE);
    StatRow("Population", TextFormat("%d", st.population), GREEN);
    StatRow("Buildings", TextFormat("%d  (%d damaged)", st.buildings, st.damaged), RAYWHITE);
    StatRow("Green space", TextFormat("%.0f%%", st.green), (Color){ 110, 200, 100, 255 });
    StatBar("Energy", TextFormat("%.1f / %.1f MW", st.energyDemand / 1000.0f, st.energyCap / 1000.0f), st.energyPct, ORANGE);
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

    uiY += 3;
    DrawText("ALERTS", 12, uiY, 10, GRAY);
    uiY += 12;
    int n = 0, pending = 0;
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active && cars[i].wanted) pending++;
    if (st.energyPct > 100.0f) Alert("! Power shortage (brownout)", &n);
    if (st.waterPct > 100.0f)  Alert("! Water shortage", &n);
    if (st.sewerPct > 100.0f)  Alert("! Sewer overflow - flooding", &n);
    if (landfillFill > 90.0f)  Alert("! Landfill nearly full", &n);
    if (st.fires > 0)          Alert(TextFormat("! %d tiles on fire", st.fires), &n);
    if (robbingNow > 0)        Alert("! Crime in progress", &n);
    if (pending > 0)           Alert(TextFormat("! %d reported vehicle(s) wanted", pending), &n);
    if (n == 0) { DrawText("All systems normal", 12, uiY, 10, GREEN); }

    DrawText("3D: WASD move  Q/E turn  Wheel zoom", 12, SCREEN_H - 48, 10, DARKGRAY);
    DrawText("RMB drag: tilt/turn   R: reset cam", 12, SCREEN_H - 37, 10, DARKGRAY);
    DrawText("SPACE: storm   Click: start fire", 12, SCREEN_H - 26, 10, DARKGRAY);
    DrawText("P: pause   +/-: sim speed", 12, SCREEN_H - 15, 10, DARKGRAY);
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

    int gx = x0 + 10, gw = PANEL_W - 20, gh = 77, y = 18, pitch = 79;
    DrawGraph(gx, y + 0 * pitch, gw, gh, G_ENERGY, "Energy demand", ORANGE, TextFormat("%.1f MW", st.energyDemand / 1000.0f), 2.0f, false);
    DrawGraph(gx, y + 1 * pitch, gw, gh, G_WATER,  "Water demand", COL_WATER_PIPE, TextFormat("%.1f kL/min", st.waterDemand / 1000.0f), 1.0f, false);
    DrawGraph(gx, y + 2 * pitch, gw, gh, G_SEWER,  "Sewer load (% capacity)", (Color){ 190, 140, 80, 255 }, TextFormat("%.0f%%", st.sewerPct), 100.0f, true);
    DrawGraph(gx, y + 3 * pitch, gw, gh, G_AQI,    "Air pollution (AQI)", (Color){ 200, 90, 200, 255 }, TextFormat("%.0f", st.aqi), 30.0f, false);
    DrawGraph(gx, y + 4 * pitch, gw, gh, G_TEMP,   "Temperature (C)", YELLOW, TextFormat("%.1f C", st.temp), 6.0f, false);
    DrawGraph(gx, y + 5 * pitch, gw, gh, G_RAIN,   "Rainfall (mm/h)", SKYBLUE, TextFormat("%.1f", st.rainMm), 5.0f, true);
    DrawGraph(gx, y + 6 * pitch, gw, gh, G_CARS,   "Vehicles on road", (Color){ 240, 210, 50, 255 }, TextFormat("%d", st.vehicles), 20.0f, true);
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
    if (IsKeyPressed(KEY_V)) viewMode = (viewMode == VIEW_2D) ? VIEW_3D : VIEW_2D;
    if (IsKeyPressed(KEY_SPACE)) { rainTimer = 30.0f; rainPeak = 1.0f; }
    if (IsKeyPressed(KEY_P)) paused = !paused;
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) simSpeed = fminf(4.0f, simSpeed * 2.0f);
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) simSpeed = fmaxf(0.5f, simSpeed * 0.5f);
    if (IsKeyPressed(KEY_R)) ResetCamera();

    Vector2 m = GetScaledMouse();
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (CheckCollisionPointRec(m, ViewBtnRect())) viewMode = (viewMode == VIEW_2D) ? VIEW_3D : VIEW_2D;
        for (int i = 0; i < L_COUNT; i++) if (CheckCollisionPointRec(m, LayerRect(i))) layerOn[i] = !layerOn[i];
    }

    if (viewMode == VIEW_3D) UpdateOrbit(dt);

    hoverOK = false;
    if (m.x >= PANEL_W && m.x < PANEL_W + VIEW_W && m.y >= 0 && m.y < VIEW_H) {
        Vector2 lm = { m.x - PANEL_W, m.y };
        int tx = 0, ty = 0;
        bool ok;
        if (viewMode == VIEW_2D) { tx = (int)(lm.x / TILE); ty = (int)(lm.y / TILE); ok = true; }
        else ok = PickTile3D(lm, &tx, &ty);
        if (ok && InBounds(tx, ty)) {
            hoverOK = true; hoverX = tx; hoverY = ty;
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) Ignite(tx, ty);
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
                DrawScene2D();
            } else {
                ClearBackground((Color){ 135, 190, 230, 255 });
                BeginMode3D(BuildCamera());
                    DrawScene3D();
                EndMode3D();
            }
            DrawWeatherFx();
        EndTextureMode();

        BeginTextureMode(screen);
            ClearBackground(BLACK);
            DrawTextureRec(view.texture, (Rectangle){ 0, 0, (float)VIEW_W, -(float)VIEW_H }, (Vector2){ PANEL_W, 0 }, WHITE);
            DrawLeftPanel();
            DrawRightPanel();
            if (layerOn[L_HEAT]) {
                DrawRectangle(PANEL_W + 6, SCREEN_H - 36, 210, 14, Fade(BLACK, 0.6f));
                DrawText("Heat map: blue = cool, red = hot", PANEL_W + 10, SCREEN_H - 33, 10, WHITE);
            }
            if (layerOn[L_POLLUTION]) {
                DrawRectangle(PANEL_W + 6, SCREEN_H - 20, 240, 14, Fade(BLACK, 0.6f));
                DrawText("Pollution map: green = clean, purple = dirty", PANEL_W + 10, SCREEN_H - 17, 10, WHITE);
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

    FreeTextures();
    UnloadRenderTexture(view);
    UnloadRenderTexture(screen);
    CloseWindow();
    return 0;
}