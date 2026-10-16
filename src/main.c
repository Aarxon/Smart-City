// Smart City Simulator (raylib, C)
//
// Layout:  [ LEFT: layers + live statistics ] [ MAP VIEW (2D / 3D) ] [ RIGHT: trend graphs ]
//
// Controls:
//   1-7 (or click)  Toggle layers: Buildings, Roads, Water, Sewer, Traffic, Heat, Pollution
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

#define SIDEWALK_W  6          // pixels (2D); scaled to tile units in 3D
#define LINE_THICK  3
#define HIST        120        // graph samples (1 per second => 2 minutes)
#define DAY_LEN     120.0f     // real seconds per simulated day

#define MAX_CARS    60
#define MAX_PEDS    28
#define MAX_TRUCKS  6
#define MAX_POLICE  5
#define MAX_STN     4
#define MAX_PATH    (MAP_W * MAP_H)

#define PLANT_KW    12000.0f   // output of one power plant
#define WORKS_LPM   9000.0f    // output of one water works (litres/min)
#define SEWER_CAP   8500.0f    // sewer capacity (litres/min)

#define LAYER_Y0    62
#define LAYER_H     17

typedef enum {
    T_GRASS = 0, T_ROAD, T_BUILDING, T_TREE, T_RUIN,
    T_FIRE_STN, T_POLICE_STN, T_POWER, T_WATER
} TileType;

enum { B_RES = 0, B_COM, B_IND };
enum { L_BUILDINGS = 0, L_ROADS, L_WATER, L_SEWER, L_TRAFFIC, L_HEAT, L_POLLUTION, L_COUNT };
enum { G_ENERGY = 0, G_WATER, G_SEWER, G_AQI, G_TEMP, G_RAIN, G_CARS, G_COUNT };
typedef enum { VIEW_2D = 0, VIEW_3D } ViewMode;

typedef struct {
    float x, y;                 // position in tile units
    int gx, gy, tx, ty, dir;    // from-tile, to-tile, heading (0 up,1 down,2 left,3 right)
    float t, ox, oy, wait, speedMul;
    Color col;
    bool active;
} Car;

typedef struct {
    float x, y;
    int gx, gy, tx, ty;
    float t, robTimer, cooldown;
    bool active, criminal, robbing;
} Ped;

typedef struct {
    bool active, arrived, spraying;
    float x, y, angle, t, work, sx, sy;
    int path[MAX_PATH];
    int len, idx, targetX, targetY, pedTarget;
} Responder;

typedef struct { int x, y, rx, ry; } Station;

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

static const Color COL_BTYPE[3] = {
    { 185, 120,  98, 255 },   // residential
    {  96, 140, 176, 255 },   // commercial
    { 150, 150, 128, 255 }    // industrial
};
static const Color CAR_COLS[6] = {
    { 230, 210,  50, 255 }, { 235, 235, 235, 255 }, {  70, 170, 230, 255 },
    { 150, 220, 100, 255 }, { 220, 120, 190, 255 }, { 240, 150,  60, 255 }
};
static const char *LAYER_NAMES[L_COUNT] = {
    "Buildings", "Roads & sidewalks", "Water network", "Sewer network",
    "Traffic", "Heat map", "Pollution map"
};
static const Color LAYER_COL[L_COUNT] = {
    { 185, 120,  98, 255 }, { 150, 150, 155, 255 }, {  70, 150, 255, 255 }, { 170, 115,  55, 255 },
    { 240, 210,  50, 255 }, { 235,  80,  50, 255 }, { 170,  60, 170, 255 }
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

static Car        cars[MAX_CARS];
static Ped        peds[MAX_PEDS];
static Responder  trucks[MAX_TRUCKS];
static Responder  police[MAX_POLICE];
static Station    fireStn[MAX_STN], policeStn[MAX_STN];
static int        nFire = 0, nPolice = 0;

static Stats st;
static float simTime = 0.0f, simSpeed = 1.0f;
static bool  paused = false;
static float rain = 0.0f, rainTimer = 0.0f, rainPeak = 1.0f, nextRain = 60.0f;
static float floodLevel = 0.0f;
static float fireTimer = 0.0f, dispatchTimer = 0.0f, mapTimer = 0.0f, statTimer = 0.0f,
             histTimer = 0.0f, birthTimer = 0.0f;
static int   crimes = 0, arrests = 0, collapses = 0, births = 0, deaths = 0, robbingNow = 0;

static float hist[G_COUNT][HIST];
static int   histIdx = 0;
static bool  histInit = false;

static bool     layerOn[L_COUNT] = { true, true, false, false, true, false, false };
static ViewMode viewMode = VIEW_2D;
static OrbitCam oc;
static bool hoverOK = false;
static int  hoverX = 0, hoverY = 0;
static float gAlpha = 1.0f;     // alpha used for structures (see-through in X-ray mode)

static const int DX[4] = { 0, 0, -1, 1 };
static const int DY[4] = { -1, 1, 0, 0 };

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static float Rf(void) { return GetRandomValue(0, 10000) / 10000.0f; }
static bool InBounds(int x, int y) { return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H; }
static bool IsRoad(int x, int y) { return InBounds(x, y) && surface[y][x] == T_ROAD; }
static bool Flammable(int x, int y) {
    int s = surface[y][x];
    return s == T_BUILDING || s == T_TREE || s == T_GRASS;
}
static bool Walkable(int x, int y) {
    if (!InBounds(x, y)) return false;
    int s = surface[y][x];
    return s == T_GRASS || s == T_TREE || s == T_ROAD;
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

static float DayFactor(void) {     // energy / water demand multiplier, peaks early evening
    return 0.65f + 0.35f * sinf((st.hour - 12.0f) / 24.0f * 2.0f * PI);
}
static float TrafficFactor(void) { // morning + evening rush hours
    float h = st.hour;
    float a = expf(-(h - 8.5f) * (h - 8.5f) / 3.0f);
    float b = expf(-(h - 17.5f) * (h - 17.5f) / 4.0f);
    return 0.4f + 0.6f * fmaxf(a, b);
}

// 0 = none, 1 = horizontal, 2 = vertical (centre dashes only on straight sections)
static int RoadLine(int x, int y) {
    bool h = IsRoad(x - 1, y) || IsRoad(x + 1, y);
    bool v = IsRoad(x, y - 1) || IsRoad(x, y + 1);
    if (h && !v) return 1;
    if (v && !h) return 2;
    return 0;
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

static bool Dispatch(Responder *pool, int poolN, Station *stn, int nStn, int tx, int ty, int pedIdx) {
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
    int len = FindPath(stn[best].rx, stn[best].ry, tx, ty, 3, r->path);
    if (len <= 0) return false;
    r->len = len; r->idx = 0; r->t = 0.0f; r->work = 0.0f;
    r->active = true; r->arrived = false; r->spraying = false;
    r->targetX = tx; r->targetY = ty; r->pedTarget = pedIdx;
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
// City generation
// ---------------------------------------------------------------------------
static bool FindAccess(int x, int y, int *rx, int *ry) {
    for (int d = 0; d < 4; d++) {
        if (IsRoad(x + DX[d], y + DY[d])) { *rx = x + DX[d]; *ry = y + DY[d]; return true; }
    }
    return false;
}

static void PlaceFixed(int x, int y, int tile) { surface[y][x] = tile; health[y][x] = 100.0f; }

static void SpawnPed(Ped *p) {
    for (int tries = 0; tries < 40; tries++) {
        int x = GetRandomValue(0, MAP_W - 1), y = GetRandomValue(0, MAP_H - 1);
        if (!Walkable(x, y) || surface[y][x] == T_ROAD) continue;
        memset(p, 0, sizeof(*p));
        p->active = true;
        p->gx = p->tx = x; p->gy = p->ty = y; p->t = 1.0f;
        p->x = x + 0.5f; p->y = y + 0.5f;
        p->criminal = (GetRandomValue(0, 99) < 16);
        return;
    }
}

static void GenerateCity(void) {
    static const int roadRows[3] = { 3, 9, 13 };
    static const int roadCols[3] = { 4, 11, 18 };

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            surface[y][x] = T_GRASS; bType[y][x] = 0; bHeight[y][x] = 0; health[y][x] = 100.0f;
            fireI[y][x] = 0; pipeNode[y][x] = false; heatMap[y][x] = 0; polMap[y][x] = 0; traffic[y][x] = 0;
        }

    for (int i = 0; i < 3; i++) {
        for (int x = 0; x < MAP_W; x++) surface[roadRows[i]][x] = T_ROAD;
        for (int y = 0; y < MAP_H; y++) surface[y][roadCols[i]] = T_ROAD;
    }

    // Buildings line the streets; block interiors become parkland
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (surface[y][x] != T_GRASS) continue;
            bool nearRoad = IsRoad(x - 1, y) || IsRoad(x + 1, y) || IsRoad(x, y - 1) || IsRoad(x, y + 1);
            int r = GetRandomValue(0, 99);
            if (nearRoad && r < 72) {
                surface[y][x] = T_BUILDING;
                int k = GetRandomValue(0, 99);
                bType[y][x] = (k < 60) ? B_RES : (k < 85 ? B_COM : B_IND);
                if (bType[y][x] == B_RES)      bHeight[y][x] = 0.6f + Rf() * 0.5f;
                else if (bType[y][x] == B_COM) bHeight[y][x] = 1.1f + Rf() * 1.2f;
                else                           bHeight[y][x] = 0.5f + Rf() * 0.3f;
            } else if (nearRoad ? (r < 86) : (r < 50)) {
                surface[y][x] = T_TREE;
            }
        }

    // Fixed city services
    PlaceFixed(2, 2, T_FIRE_STN);
    PlaceFixed(19, 4, T_FIRE_STN);
    PlaceFixed(19, 10, T_POLICE_STN);
    PlaceFixed(5, 11, T_POLICE_STN);
    PlaceFixed(12, 6, T_POWER);
    PlaceFixed(17, 6, T_WATER);   // PlaceFixed(x, y, tile)

    // Utility pipes: every road plus every service tile touching a road
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x], rx, ry;
            if (s == T_ROAD) pipeNode[y][x] = true;
            else if (s != T_GRASS && s != T_TREE && FindAccess(x, y, &rx, &ry)) pipeNode[y][x] = true;
        }

    // Register stations
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
    for (int i = 0; i < 18; i++) SpawnPed(&peds[i]);
}

// ---------------------------------------------------------------------------
// Simulation
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

static void UpdateCars(float dt) {
    static float cnt[MAP_H][MAP_W];
    memset(cnt, 0, sizeof(cnt));

    int active = 0;
    for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) active++;
    int target = (int)(MAX_CARS * TrafficFactor());

    if (active < target) {
        for (int tries = 0; tries < 30; tries++) {
            int rx = GetRandomValue(0, MAP_W - 1), ry = GetRandomValue(0, MAP_H - 1);
            if (!IsRoad(rx, ry)) continue;
            for (int i = 0; i < MAX_CARS; i++) {
                if (cars[i].active) continue;
                Car *c = &cars[i];
                memset(c, 0, sizeof(*c));
                c->active = true;
                c->gx = c->tx = rx; c->gy = c->ty = ry;
                c->t = 1.0f; c->dir = 3;
                c->x = rx + 0.5f; c->y = ry + 0.5f;
                c->speedMul = 0.85f + Rf() * 0.3f;
                c->col = CAR_COLS[GetRandomValue(0, 5)];
                break;
            }
            break;
        }
    } else if (active > target + 2) {
        for (int i = 0; i < MAX_CARS; i++) if (cars[i].active) { cars[i].active = false; break; }
    }

    int n = 0, jammed = 0;
    for (int i = 0; i < MAX_CARS; i++) {
        Car *c = &cars[i];
        if (!c->active) continue;
        n++;

        // Car following: stop behind a car ahead in the same lane
        float dxv = (float)DX[c->dir], dyv = (float)DY[c->dir];
        bool blocked = false;
        for (int j = 0; j < MAX_CARS; j++) {
            if (j == i || !cars[j].active || cars[j].dir != c->dir) continue;
            float rx = cars[j].x - c->x, ry = cars[j].y - c->y;
            float along = rx * dxv + ry * dyv;
            float side = fabsf(-rx * dyv + ry * dxv);
            if (along > 0.02f && along < 0.45f && side < 0.15f) { blocked = true; break; }
        }
        float speed = 2.0f * c->speedMul;
        if (blocked) {
            c->wait += dt;
            if (c->wait < 4.0f) { speed = 0.0f; jammed++; }   // after 4 s, creep through (gridlock breaker)
        } else c->wait = 0.0f;

        c->t += dt * speed;
        if (c->t >= 1.0f) {
            c->t = 0.0f;
            c->gx = c->tx; c->gy = c->ty;
            int valid[4], nv = 0;
            for (int d = 0; d < 4; d++) if (IsRoad(c->gx + DX[d], c->gy + DY[d])) valid[nv++] = d;
            int choice = -1;
            if (nv > 0) {
                bool straight = false;
                for (int k = 0; k < nv; k++) if (valid[k] == c->dir) straight = true;
                if (straight && Rf() < 0.6f) choice = c->dir;
                else {
                    int opts[4], no = 0;
                    for (int k = 0; k < nv; k++) if (valid[k] != (c->dir ^ 1)) opts[no++] = valid[k];
                    choice = (no > 0) ? opts[GetRandomValue(0, no - 1)] : valid[0];
                }
            }
            if (choice >= 0) {
                c->dir = choice;
                c->tx = c->gx + DX[choice]; c->ty = c->gy + DY[choice];
            } else { c->tx = c->gx; c->ty = c->gy; c->t = 1.0f; }
        }

        // Left-hand traffic: drive on the left of the heading (swap sign for right-hand traffic)
        float lane = 0.17f;
        float tox = DY[c->dir] * lane, toy = -DX[c->dir] * lane;
        float k = Clamp(dt * 8.0f, 0.0f, 1.0f);
        c->ox += (tox - c->ox) * k;
        c->oy += (toy - c->oy) * k;
        float ax = c->gx + 0.5f, ay = c->gy + 0.5f, bx = c->tx + 0.5f, by = c->ty + 0.5f;
        c->x = ax + (bx - ax) * c->t + c->ox;
        c->y = ay + (by - ay) * c->t + c->oy;

        int tx = (int)c->x, ty = (int)c->y;
        if (InBounds(tx, ty)) cnt[ty][tx] += 1.0f;
    }
    st.vehicles = n;
    st.jammed = jammed;

    float k = Clamp(dt * 2.0f, 0.0f, 1.0f);
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) traffic[y][x] += (cnt[y][x] - traffic[y][x]) * k;
}

static void UpdatePeds(float dt) {
    robbingNow = 0;
    for (int i = 0; i < MAX_PEDS; i++) {
        Ped *p = &peds[i];
        if (!p->active) {
            if (GetRandomValue(0, 1000) < 10) SpawnPed(p);
            continue;
        }
        if (p->robbing) {
            robbingNow++;
            p->robTimer += dt;
            if (p->robTimer >= 3.0f) { p->robbing = false; p->cooldown = 12.0f; crimes++; }
            continue;
        }
        if (p->cooldown > 0.0f) p->cooldown -= dt;

        p->t += dt * (p->criminal ? 1.0f : 0.7f);
        if (p->t >= 1.0f) {
            p->t = 0.0f;
            p->gx = p->tx; p->gy = p->ty;
            int valid[4], nv = 0;
            for (int d = 0; d < 4; d++) if (Walkable(p->gx + DX[d], p->gy + DY[d])) valid[nv++] = d;
            if (nv > 0) {
                int d = valid[GetRandomValue(0, nv - 1)];
                p->tx = p->gx + DX[d]; p->ty = p->gy + DY[d];
            } else { p->tx = p->gx; p->ty = p->gy; p->t = 1.0f; }
        }
        float ax = p->gx + 0.5f, ay = p->gy + 0.5f, bx = p->tx + 0.5f, by = p->ty + 0.5f;
        p->x = ax + (bx - ax) * p->t;
        p->y = ay + (by - ay) * p->t;

        if (p->criminal && p->cooldown <= 0.0f) {
            for (int j = 0; j < MAX_PEDS; j++) {
                if (j == i || !peds[j].active || peds[j].criminal) continue;
                float dx = peds[j].x - p->x, dy = peds[j].y - p->y;
                if (dx * dx + dy * dy < 0.64f) { p->robbing = true; p->robTimer = 0.0f; break; }
            }
        }
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
        if ((!r->spraying && r->work > 1.5f) || r->work > 25.0f) r->active = false;
    }

    for (int p = 0; p < MAX_POLICE; p++) {
        Responder *r = &police[p];
        if (!r->active) continue;
        if (!r->arrived) { if (MoveResponder(r, dt, 4.0f)) r->arrived = true; continue; }
        r->work += dt;
        if (r->work > 0.6f) {
            int pi = r->pedTarget;
            if (pi >= 0 && peds[pi].active && peds[pi].criminal) { peds[pi].active = false; arrests++; }
            r->active = false;
        }
    }
}

static void UpdateDispatch(void) {
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (fireI[y][x] <= 0.0f) continue;
            bool covered = false;
            for (int t = 0; t < MAX_TRUCKS; t++)
                if (trucks[t].active && abs(trucks[t].targetX - x) <= 3 && abs(trucks[t].targetY - y) <= 3) covered = true;
            if (!covered) Dispatch(trucks, MAX_TRUCKS, fireStn, nFire, x, y, -1);
        }
    for (int i = 0; i < MAX_PEDS; i++) {
        if (!(peds[i].active && peds[i].criminal && peds[i].robbing)) continue;
        bool assigned = false;
        for (int p = 0; p < MAX_POLICE; p++) if (police[p].active && police[p].pedTarget == i) assigned = true;
        if (!assigned) Dispatch(police, MAX_POLICE, policeStn, nPolice, (int)peds[i].x, (int)peds[i].y, i);
    }
}

static void UpdateFire(float dt) {
    static bool ign[MAP_H][MAP_W];
    memset(ign, 0, sizeof(ign));
    fireTimer += dt;
    bool spread = fireTimer >= 1.2f;
    if (spread) fireTimer = 0.0f;
    st.fires = 0;

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (fireI[y][x] <= 0.0f) {
                fireI[y][x] = 0.0f;
                if (surface[y][x] == T_BUILDING && health[y][x] < 100.0f)
                    health[y][x] = fminf(100.0f, health[y][x] + 0.8f * dt);   // slow repairs
                continue;
            }
            fireI[y][x] -= (3.5f + rain * 28.0f) * dt;

            if (surface[y][x] == T_BUILDING) {
                health[y][x] -= 6.0f * dt;
                if (health[y][x] <= 0.0f) {
                    health[y][x] = 0.0f;
                    surface[y][x] = T_RUIN;
                    fireI[y][x] = 0.0f;
                    collapses++;
                    if (bType[y][x] == B_RES) deaths += GetRandomValue(0, 2);
                    continue;
                }
            }
            if (fireI[y][x] <= 0.0f) {
                fireI[y][x] = 0.0f;
                if (surface[y][x] == T_TREE) surface[y][x] = T_GRASS;   // burnt out
                continue;
            }
            st.fires++;

            if (spread && fireI[y][x] > 30.0f) {
                for (int d = 0; d < 4; d++) {
                    int nx = x + DX[d], ny = y + DY[d];
                    if (!InBounds(nx, ny) || fireI[ny][nx] > 0.0f || !Flammable(nx, ny)) continue;
                    float chance = (surface[ny][nx] == T_GRASS ? 0.10f : 0.28f) * (1.0f - rain);
                    if (Rf() < chance) ign[ny][nx] = true;
                }
            }
        }
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) if (ign[y][x]) fireI[y][x] = 100.0f;
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

static void UpdateMaps(void) {
    float df = DayFactor();
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float heatT = 0.0f, pol = 0.0f;
            switch (surface[y][x]) {
                case T_BUILDING:
                    heatT = (bType[y][x] == B_IND ? 4.5f : 2.2f) * df + 0.8f;
                    pol = (bType[y][x] == B_IND) ? 5.0f : (bType[y][x] == B_COM ? 0.9f : 0.5f);
                    break;
                case T_ROAD:  heatT = 1.2f + traffic[y][x] * 2.0f; pol = 0.4f + traffic[y][x] * 4.0f; break;
                case T_TREE:  heatT = -1.8f; pol = -0.6f; break;
                case T_GRASS: heatT = -0.4f; pol = -0.15f; break;
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
    int res = 0, bld = 0, dmg = 0, roads = 0, green = 0, plants = 0, works = 0, stations = 0;
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
                case T_TREE: case T_GRASS: green++; break;
                case T_POWER: plants++; break;
                case T_WATER: works++; break;
                case T_FIRE_STN: case T_POLICE_STN: stations++; break;
                default: break;
            }
        }
    eDem += stations * 30.0f + roads * 1.6f * (1.0f - st.daylight);   // street lighting at night
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
    st.aqi = 12.0f + avgPol * 4.5f;
    st.heatIsland = avgHeat;
    st.temp = st.ambient + avgHeat;
    st.rainMm = rain * 25.0f;
    st.green = green * 100.0f / (MAP_W * MAP_H);
    st.population = res * 22 + births - deaths;
    if (st.population < 0) st.population = 0;

    float secTarget = Clamp(50.0f + 15.0f * nPolice - 10.0f * robbingNow - 2.0f * st.fires, 0.0f, 100.0f);
    st.security += (secTarget - st.security) * 0.2f;

    float floodTarget = (st.sewerPct > 100.0f && rain > 0.15f) ? Clamp((st.sewerPct - 100.0f) / 40.0f, 0.0f, 1.0f) : 0.0f;
    floodLevel += (floodTarget - floodLevel) * 0.3f;
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

static void UpdateSim(float dt) {
    simTime += dt;
    UpdateClock();
    UpdateWeather(dt);
    UpdateCars(dt);
    UpdatePeds(dt);
    UpdateResponders(dt);
    UpdateFire(dt);

    dispatchTimer += dt;
    if (dispatchTimer >= 1.0f) { dispatchTimer = 0.0f; UpdateDispatch(); }
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
    if (layerOn[L_TRAFFIC]) {
        float d = traffic[y][x];
        if (d > 0.03f) {
            Color tint = Ramp3((Color){ 60, 200, 90, 255 }, (Color){ 240, 210, 50, 255 }, (Color){ 230, 50, 40, 255 }, d / 1.3f);
            tint.a = (unsigned char)(Clamp(d * 5.0f, 0.0f, 1.0f) * 170.0f);
            c = BlendOver(c, tint);
        }
    }
    if (floodLevel > 0.02f) c = BlendOver(c, (Color){ 80, 140, 255, (unsigned char)(floodLevel * 130.0f) });
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

// ---------------------------------------------------------------------------
// 2D drawing
// ---------------------------------------------------------------------------
static void DrawGround2D(int x, int y, bool xray) {
    int px = x * TILE, py = y * TILE;
    if (surface[y][x] == T_ROAD && layerOn[L_ROADS]) {
        DrawRectangle(px, py, TILE, TILE, RoadColorAt(x, y, xray));
        int ln = RoadLine(x, y);
        Color lc = xray ? (Color){ 150, 150, 150, 255 } : COL_LINE;
        if (ln == 1) DrawRectangle(px + TILE / 4, py + TILE / 2 - LINE_THICK / 2, TILE / 2, LINE_THICK, lc);
        else if (ln == 2) DrawRectangle(px + TILE / 2 - LINE_THICK / 2, py + TILE / 4, LINE_THICK, TILE / 2, lc);
    } else {
        DrawRectangle(px, py, TILE, TILE, xray ? COL_SOIL : COL_GRASS);
        if (layerOn[L_ROADS] && surface[y][x] != T_ROAD) {
            Rectangle r[8];
            int n = SidewalkRects(x, y, r);
            for (int i = 0; i < n; i++)
                DrawRectangle(px + (int)(r[i].x * TILE), py + (int)(r[i].y * TILE),
                              (int)(r[i].width * TILE + 0.5f), (int)(r[i].height * TILE + 0.5f),
                              xray ? COL_SIDEWALK_DIM : COL_SIDEWALK);
        }
    }
}

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

static void DrawTree2D(int x, int y) {
    int cx = x * TILE + TILE / 2, cy = y * TILE + TILE / 2;
    DrawCircle(cx, cy + 2, TILE * 0.30f, Fade((Color){ 25, 95, 40, 255 }, gAlpha));
    DrawCircle(cx - 2, cy, TILE * 0.27f, Fade(COL_TREE, gAlpha));
}

static void DrawStructure2D(int x, int y) {
    int px = x * TILE, py = y * TILE;
    switch (surface[y][x]) {
        case T_BUILDING: {
            Color c = (health[y][x] < 50.0f) ? COL_DAMAGED : COL_BTYPE[bType[y][x]];
            DrawRectangle(px + 5, py + 5, TILE - 10, TILE - 10, Fade(c, gAlpha));
            DrawRectangleLines(px + 5, py + 5, TILE - 10, TILE - 10, Fade((Color){ 60, 40, 35, 255 }, gAlpha));
            Color wc = (st.daylight < 0.5f) ? (Color){ 255, 230, 120, 255 } : (Color){ 215, 225, 235, 255 };
            for (int wy = 0; wy < 2; wy++)
                for (int wx = 0; wx < 2; wx++)
                    DrawRectangle(px + 9 + wx * 10, py + 9 + wy * 10, 5, 5, Fade(wc, gAlpha));
            if (bType[y][x] == B_IND) DrawRectangle(px + TILE - 13, py + 2, 5, 9, Fade((Color){ 55, 55, 55, 255 }, gAlpha));
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

static void DrawResponder2D(const Responder *r, Color body, double t) {
    float px = r->x * TILE, py = r->y * TILE;
    DrawRectanglePro((Rectangle){ px, py, 16, 9 }, (Vector2){ 8, 4.5f }, r->angle, body);
    bool flash = ((int)(t * 6.0)) % 2 == 0;
    DrawCircle((int)px, (int)py, 2.5f, flash ? RED : SKYBLUE);
    if (r->spraying) DrawLineEx((Vector2){ px, py }, (Vector2){ r->sx * TILE, r->sy * TILE }, 2.5f, (Color){ 130, 200, 255, 210 });
}

static void DrawScene2D(void) {
    static const float carAngle[4] = { 270.0f, 90.0f, 180.0f, 0.0f };
    bool xray = layerOn[L_WATER] || layerOn[L_SEWER];
    double t = GetTime();

    ClearBackground(COL_GRASS);
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) DrawGround2D(x, y, xray);

    if (layerOn[L_SEWER]) DrawNetwork2D(4.0f, SewerColor(), (Color){ 230, 200, 150, 255 }, 0.7f);
    if (layerOn[L_WATER]) DrawNetwork2D(-4.0f, COL_WATER_PIPE, (Color){ 220, 240, 255, 255 }, 0.9f);

    gAlpha = xray ? 0.45f : 1.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            if (s == T_TREE) DrawTree2D(x, y);
            else if (s != T_GRASS && s != T_ROAD && layerOn[L_BUILDINGS]) DrawStructure2D(x, y);
        }
    gAlpha = 1.0f;

    if (layerOn[L_HEAT])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, HeatColorAt(x, y));
    if (layerOn[L_POLLUTION])
        for (int y = 0; y < MAP_H; y++) for (int x = 0; x < MAP_W; x++) DrawRectangle(x * TILE, y * TILE, TILE, TILE, PolColorAt(x, y));

    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            if (fireI[y][x] <= 0.0f) continue;
            float fl = 0.6f + 0.4f * sinf((float)t * 14.0f + x * 5.0f + y * 3.0f);
            DrawRectangle(x * TILE + 6, y * TILE + 6, TILE - 12, TILE - 12, Fade(COL_FIRE, 0.65f + 0.3f * fl));
            DrawCircle(x * TILE + TILE / 2, y * TILE + TILE / 2, 3.0f + 5.0f * fl, Fade(YELLOW, 0.9f));
        }

    if (layerOn[L_TRAFFIC])
        for (int i = 0; i < MAX_CARS; i++) {
            if (!cars[i].active) continue;
            DrawRectanglePro((Rectangle){ cars[i].x * TILE, cars[i].y * TILE, 12, 7 }, (Vector2){ 6, 3.5f }, carAngle[cars[i].dir], cars[i].col);
        }

    for (int i = 0; i < MAX_PEDS; i++) {
        if (!peds[i].active) continue;
        Vector2 p = { peds[i].x * TILE, peds[i].y * TILE };
        DrawCircleV(p, 3.0f, peds[i].criminal ? COL_CRIMINAL : COL_CIVILIAN);
        if (peds[i].robbing) DrawCircleLines((int)p.x, (int)p.y, 7.0f + 2.0f * sinf((float)t * 10.0f), RED);
    }
    for (int i = 0; i < MAX_TRUCKS; i++) if (trucks[i].active) DrawResponder2D(&trucks[i], COL_TRUCK, t);
    for (int i = 0; i < MAX_POLICE; i++) if (police[i].active) DrawResponder2D(&police[i], COL_POLICE_CAR, t);

    if (hoverOK) DrawRectangleLines(hoverX * TILE, hoverY * TILE, TILE, TILE, WHITE);
}

// ---------------------------------------------------------------------------
// 3D drawing (tile = 1 world unit; map x -> world X, map y -> world Z)
// ---------------------------------------------------------------------------
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

static void DrawScene3D(void) {
    bool xray = layerOn[L_WATER] || layerOn[L_SEWER];
    float t = (float)GetTime();

    DrawPlane((Vector3){ MAP_W / 2.0f, -0.03f, MAP_H / 2.0f }, (Vector2){ MAP_W + 60, MAP_H + 60 }, (Color){ 58, 105, 52, 255 });
    DrawPlane((Vector3){ MAP_W / 2.0f, 0.0f, MAP_H / 2.0f }, (Vector2){ MAP_W, MAP_H }, xray ? COL_SOIL : COL_GRASS);

    // Ground: roads, centre dashes, sidewalks, overlays
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            float cx = x + 0.5f, cz = y + 0.5f;
            if (layerOn[L_ROADS]) {
                if (surface[y][x] == T_ROAD) {
                    DrawCube((Vector3){ cx, 0.03f, cz }, 1.0f, 0.06f, 1.0f, RoadColorAt(x, y, xray));
                    int ln = RoadLine(x, y);
                    Color lc = xray ? (Color){ 150, 150, 150, 255 } : COL_LINE;
                    if (ln == 1) DrawCube((Vector3){ cx, 0.066f, cz }, 0.5f, 0.012f, 0.045f, lc);
                    else if (ln == 2) DrawCube((Vector3){ cx, 0.066f, cz }, 0.045f, 0.012f, 0.5f, lc);
                } else {
                    Rectangle r[8];
                    int n = SidewalkRects(x, y, r);
                    for (int i = 0; i < n; i++)
                        DrawCube((Vector3){ x + r[i].x + r[i].width / 2.0f, 0.025f, y + r[i].y + r[i].height / 2.0f },
                                 r[i].width, 0.05f, r[i].height, xray ? COL_SIDEWALK_DIM : COL_SIDEWALK);
                }
            }
            if (layerOn[L_HEAT])      DrawCube((Vector3){ cx, 0.080f, cz }, 1.0f, 0.01f, 1.0f, HeatColorAt(x, y));
            if (layerOn[L_POLLUTION]) DrawCube((Vector3){ cx, 0.095f, cz }, 1.0f, 0.01f, 1.0f, PolColorAt(x, y));
        }

    // Underground networks (same offsets as 2D so both views match)
    if (layerOn[L_SEWER]) DrawNetwork3D(0.11f, 0.10f, SewerColor());
    if (layerOn[L_WATER]) DrawNetwork3D(-0.11f, 0.13f, COL_WATER_PIPE);

    // Structures, trees, fire
    gAlpha = xray ? 0.45f : 1.0f;
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++) {
            int s = surface[y][x];
            float cx = x + 0.5f, cz = y + 0.5f;
            float top = 0.06f;

            if (s == T_TREE) {
                DrawCylinder((Vector3){ cx, 0.0f, cz }, 0.05f, 0.05f, 0.32f, 6, Fade((Color){ 110, 75, 45, 255 }, gAlpha));
                DrawSphere((Vector3){ cx, 0.50f, cz }, 0.29f, Fade(COL_TREE, gAlpha));
                top = 0.8f;
            } else if (layerOn[L_BUILDINGS]) {
                switch (s) {
                    case T_BUILDING: {
                        float h = bHeight[y][x];
                        Color c = (health[y][x] < 50.0f) ? COL_DAMAGED : COL_BTYPE[bType[y][x]];
                        Vector3 pos = { cx, h / 2.0f, cz };
                        DrawCube(pos, 0.76f, h, 0.76f, Fade(c, gAlpha));
                        DrawCubeWires(pos, 0.76f, h, 0.76f, Fade((Color){ 45, 30, 25, 255 }, gAlpha));
                        if (bType[y][x] == B_IND) DrawCylinder((Vector3){ cx + 0.2f, h, cz - 0.2f }, 0.06f, 0.06f, 0.45f, 8, Fade((Color){ 60, 60, 60, 255 }, gAlpha));
                        if (layerOn[L_HEAT])      DrawCube(pos, 0.78f, h + 0.02f, 0.78f, HeatColorAt(x, y));
                        if (layerOn[L_POLLUTION]) DrawCube(pos, 0.79f, h + 0.03f, 0.79f, PolColorAt(x, y));
                        top = h;
                    } break;
                    case T_RUIN:
                        DrawCube((Vector3){ cx, 0.1f, cz }, 0.7f, 0.2f, 0.7f, Fade(COL_RUIN, gAlpha));
                        top = 0.2f;
                        break;
                    case T_FIRE_STN: case T_POLICE_STN: {
                        Color c = (s == T_FIRE_STN) ? COL_FIRE_STN : COL_POLICE_STN;
                        DrawCube((Vector3){ cx, 0.4f, cz }, 0.88f, 0.8f, 0.88f, Fade(c, gAlpha));
                        DrawCubeWires((Vector3){ cx, 0.4f, cz }, 0.88f, 0.8f, 0.88f, Fade(BLACK, gAlpha));
                        top = 0.8f;
                    } break;
                    case T_POWER:
                        DrawCube((Vector3){ cx, 0.3f, cz }, 0.9f, 0.6f, 0.9f, Fade(COL_POWER, gAlpha));
                        DrawCylinder((Vector3){ cx - 0.22f, 0.6f, cz }, 0.07f, 0.09f, 0.7f, 8, Fade((Color){ 40, 40, 44, 255 }, gAlpha));
                        DrawCylinder((Vector3){ cx + 0.22f, 0.6f, cz }, 0.07f, 0.09f, 0.7f, 8, Fade((Color){ 40, 40, 44, 255 }, gAlpha));
                        top = 1.3f;
                        break;
                    case T_WATER:
                        DrawCube((Vector3){ cx, 0.2f, cz }, 0.9f, 0.4f, 0.9f, Fade(COL_WORKS, gAlpha));
                        DrawCylinder((Vector3){ cx, 0.4f, cz }, 0.28f, 0.28f, 0.5f, 12, Fade((Color){ 200, 230, 240, 255 }, gAlpha));
                        top = 0.9f;
                        break;
                    default: break;
                }
            }

            if (fireI[y][x] > 0.0f) {
                float fh = 0.35f + 0.25f * sinf(t * 13.0f + x * 3.1f + y * 1.7f);
                float base = (s == T_BUILDING) ? bHeight[y][x] : ((s == T_TREE) ? 0.5f : 0.06f);
                if (!layerOn[L_BUILDINGS] && s != T_TREE) base = 0.06f;
                (void)top;
                DrawCube((Vector3){ cx, base + fh / 2.0f, cz }, 0.45f, fh, 0.45f, Fade(COL_FIRE, 0.85f));
                DrawCube((Vector3){ cx, base + fh / 3.0f, cz }, 0.25f, fh * 0.7f, 0.25f, Fade(YELLOW, 0.9f));
            }
        }
    gAlpha = 1.0f;

    // Vehicles and people
    if (layerOn[L_TRAFFIC])
        for (int i = 0; i < MAX_CARS; i++) {
            if (!cars[i].active) continue;
            bool hor = cars[i].dir >= 2;
            DrawCube((Vector3){ cars[i].x, 0.10f, cars[i].y }, hor ? 0.26f : 0.14f, 0.09f, hor ? 0.14f : 0.26f, cars[i].col);
            DrawCube((Vector3){ cars[i].x, 0.16f, cars[i].y }, hor ? 0.12f : 0.11f, 0.05f, hor ? 0.11f : 0.12f, (Color){ 30, 30, 40, 255 });
        }
    for (int pass = 0; pass < 2; pass++) {
        Responder *pool = pass == 0 ? trucks : police;
        int n = pass == 0 ? MAX_TRUCKS : MAX_POLICE;
        Color body = pass == 0 ? COL_TRUCK : COL_POLICE_CAR;
        for (int i = 0; i < n; i++) {
            if (!pool[i].active) continue;
            Responder *r = &pool[i];
            bool hor = fabsf(cosf(r->angle * DEG2RAD)) > fabsf(sinf(r->angle * DEG2RAD));
            DrawCube((Vector3){ r->x, 0.12f, r->y }, hor ? 0.36f : 0.18f, 0.12f, hor ? 0.18f : 0.36f, body);
            bool flash = ((int)(t * 6.0f)) % 2 == 0;
            DrawCube((Vector3){ r->x, 0.22f, r->y }, 0.07f, 0.06f, 0.07f, flash ? RED : SKYBLUE);
            if (r->spraying) DrawLine3D((Vector3){ r->x, 0.25f, r->y }, (Vector3){ r->sx, 0.4f, r->sy }, (Color){ 130, 200, 255, 255 });
        }
    }
    for (int i = 0; i < MAX_PEDS; i++) {
        if (!peds[i].active) continue;
        DrawSphere((Vector3){ peds[i].x, 0.09f, peds[i].y }, 0.045f, peds[i].criminal ? COL_CRIMINAL : COL_CIVILIAN);
        if (peds[i].robbing) DrawCube((Vector3){ peds[i].x, 0.35f, peds[i].y }, 0.06f, 0.18f, 0.06f, RED);
    }

    if (hoverOK) DrawCubeWires((Vector3){ hoverX + 0.5f, 0.05f, hoverY + 0.5f }, 1.0f, 0.1f, 1.0f, WHITE);
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
// UI panels
// ---------------------------------------------------------------------------
static Vector2 GetScaledMouse(void);
static int uiY;
static const Color COL_LABEL = { 170, 175, 185, 255 };

static Rectangle LayerRect(int i) { return (Rectangle){ 10, (float)(LAYER_Y0 + i * LAYER_H), PANEL_W - 20, LAYER_H - 2 }; }
static Rectangle ViewBtnRect(void) { return (Rectangle){ 10, 26, PANEL_W - 20, 18 }; }

static void StatRow(const char *label, const char *val, Color c) {
    DrawText(label, 12, uiY, 10, COL_LABEL);
    DrawText(val, PANEL_W - 12 - MeasureText(val, 10), uiY, 10, c);
    uiY += 14;
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
    DrawText(s, 12, uiY, 10, (Color){ 255, 110, 90, 255 });
    uiY += 12;
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

    DrawText("LAYERS  (click or press 1-7)", 12, 50, 10, GRAY);
    for (int i = 0; i < L_COUNT; i++) {
        Rectangle r = LayerRect(i);
        if (CheckCollisionPointRec(m, r)) DrawRectangleRec(r, (Color){ 40, 46, 58, 255 });
        DrawRectangleLines((int)r.x + 2, (int)r.y + 2, 11, 11, GRAY);
        if (layerOn[i]) DrawRectangle((int)r.x + 4, (int)r.y + 4, 7, 7, LAYER_COL[i]);
        DrawText(LAYER_NAMES[i], (int)r.x + 20, (int)r.y + 3, 10, layerOn[i] ? RAYWHITE : GRAY);
        DrawText(TextFormat("[%d]", i + 1), (int)(r.x + r.width) - 22, (int)r.y + 3, 10, DARKGRAY);
    }

    DrawText("CITY STATISTICS", 12, 186, 10, GRAY);
    uiY = 200;
    int mins = (int)((st.hour - (int)st.hour) * 60.0f);
    StatRow("Time", TextFormat("Day %d  %02d:%02d  x%.1f%s", st.day, (int)st.hour, mins, simSpeed, paused ? " PAUSED" : ""), RAYWHITE);
    StatRow("Population", TextFormat("%d", st.population), GREEN);
    StatRow("Buildings", TextFormat("%d  (%d damaged)", st.buildings, st.damaged), RAYWHITE);
    StatRow("Green space", TextFormat("%.0f%%", st.green), (Color){ 110, 200, 100, 255 });
    StatBar("Energy", TextFormat("%.1f / %.1f MW", st.energyDemand / 1000.0f, st.energyCap / 1000.0f), st.energyPct, ORANGE);
    StatBar("Water supply", TextFormat("%.1f / %.1f kL/min", st.waterDemand / 1000.0f, st.waterCap / 1000.0f), st.waterPct, COL_WATER_PIPE);
    StatBar("Sewer load", TextFormat("%.0f%%", st.sewerPct), st.sewerPct, (Color){ 190, 140, 80, 255 });
    StatBar("Air quality", TextFormat("AQI %.0f", st.aqi), st.aqi / 1.5f, (Color){ 190, 90, 190, 255 });
    StatRow("Temperature", TextFormat("%.1f C  (+%.1f urban)", st.temp, st.heatIsland), ORANGE);
    StatRow("Rainfall", TextFormat("%.1f mm/h", st.rainMm), SKYBLUE);
    StatRow("Vehicles", TextFormat("%d  (%d stopped)", st.vehicles, st.jammed), (Color){ 240, 210, 50, 255 });
    StatRow("Fires", TextFormat("%d active / %d collapses", st.fires, collapses), COL_FIRE);
    StatRow("Crime", TextFormat("%d reported / %d arrests", crimes, arrests), COL_CRIMINAL);
    StatRow("Security index", TextFormat("%.0f%%", st.security), LIGHTGRAY);
    StatRow("Births / deaths", TextFormat("%d / %d", births, deaths), LIGHTGRAY);

    uiY += 4;
    DrawText("ALERTS", 12, uiY, 10, GRAY);
    uiY += 12;
    int n = 0;
    if (st.energyPct > 100.0f) Alert("! Power shortage (brownout)", &n);
    if (st.waterPct > 100.0f)  Alert("! Water shortage", &n);
    if (st.sewerPct > 100.0f)  Alert("! Sewer overflow - flooding", &n);
    if (st.fires > 0)          Alert(TextFormat("! %d tiles on fire", st.fires), &n);
    if (robbingNow > 0)        Alert("! Crime in progress", &n);
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
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && Flammable(tx, ty)) fireI[ty][tx] = 100.0f;
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

    UnloadRenderTexture(view);
    UnloadRenderTexture(screen);
    CloseWindow();
    return 0;
}
