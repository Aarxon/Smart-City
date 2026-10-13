// Smart City Sim - Autonomous City with Traffic, Pedestrians, Fire, and Police (raylib, C)
//
// Controls:
//   Left Mouse  - Ignite fire (click on buildings/trees/grass)
//   SPACE       - Trigger rain storm
//   V           - Toggle 2D / 3D Camera View
//   L           - Cycle Layers (Surface, Underground, Heat Overlay)

#include "raylib.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TILE_SIZE       48
#define MAP_W           24
#define MAP_H           16
#define UI_WIDTH        340
#define SCREEN_W        ((MAP_W * TILE_SIZE) + UI_WIDTH)
#define SCREEN_H        (MAP_H * TILE_SIZE)

#define SIDEWALK_W      8
#define LINE_THICK      3
#define HISTORY_LEN     100

#define MAX_CARS        25
#define MAX_TRUCKS      8
#define MAX_POLICE      6
#define MAX_PEDESTRIANS 30

typedef enum {
    TILE_GRASS = 0,
    TILE_ROAD,
    TILE_BUILDING,
    TILE_PIPE,
    TILE_RUIN,
    TILE_TREE,
    TILE_FIRE_STATION,
    TILE_POLICE_STATION
} TileType;

typedef enum {
    LAYER_SURFACE = 0,
    LAYER_UNDERGROUND,
    LAYER_OVERLAY
} MapLayer;

typedef enum {
    MODE_2D = 0,
    MODE_3D
} ViewMode;

typedef struct {
    Vector2 pos;
    int gridX, gridY;
    int targetX, targetY;
    float progress;
    bool active;
    int dir;
} Car;

typedef struct {
    Vector2 pos;
    int targetX, targetY;
    bool active;
} FireTruck;

typedef struct {
    Vector2 pos;
    int targetX, targetY;
    bool active;
} PoliceUnit;

typedef struct {
    Vector2 pos;
    int gridX, gridY;
    int targetX, targetY;
    float progress;
    bool isCriminal;
    bool isRobbing;
    float robTimer;
    bool active;
} Pedestrian;

// Colors
static const Color COL_GRASS          = {  86, 160,  70, 255 };
static const Color COL_ROAD           = {  95,  95, 100, 255 };
static const Color COL_BUILDING       = { 175, 115,  95, 255 };
static const Color COL_DAMAGED        = { 110,  70,  60, 255 };
static const Color COL_RUIN           = {  65,  60,  60, 255 };
static const Color COL_TREE           = {  35, 120,  45, 255 };
static const Color COL_TREE_TRUNK     = { 110,  75,  45, 255 };
static const Color COL_FIRE_STATION   = { 200,  50,  50, 255 };
static const Color COL_POLICE_STATION = {  40,  80, 200, 255 };
static const Color COL_PIPE           = {  70, 130, 180, 255 };
static const Color COL_SIDEWALK       = { 210, 195, 155, 255 };
static const Color COL_LINE           = { 255, 255, 255, 255 };
static const Color COL_PANEL          = {  30,  33,  40, 255 };
static const Color COL_FIRE           = { 245,  90,  35, 255 };
static const Color COL_CAR            = { 230, 210,  50, 255 };
static const Color COL_TRUCK          = { 255, 120,   0, 255 };
static const Color COL_POLICE_CAR     = {  50, 120, 255, 255 };
static const Color COL_CIVILIAN       = { 220, 180, 140, 255 };
static const Color COL_CRIMINAL       = { 180,  40, 180, 255 };

// Map Data
static int surfaceMap[MAP_H][MAP_W];
static int undergroundMap[MAP_H][MAP_W];
static float fireIntensity[MAP_H][MAP_W];   
static float buildingHealth[MAP_H][MAP_W]; 

// City Statistics & Demographics
static float energyUsage = 0.0f;
static float waterUsage = 0.0f;
static float pollutionLevel = 0.0f;
static float heatLevel = 22.0f;
static float rainLevel = 0.0f;
static int activeFiresCount = 0;
static float securityLevel = 100.0f;

static int statArrests = 0;
static int statDeaths = 0;
static int statNewborns = 120;
static float demoTimer = 0.0f;

static float energyHistory[HISTORY_LEN] = {0};
static float pollutionHistory[HISTORY_LEN] = {0};
static float heatHistory[HISTORY_LEN] = {0};
static int historyIndex = 0;
static float historyTimer = 0.0f;
static float fireSpreadTimer = 0.0f;

// Entities
static Car cars[MAX_CARS];
static FireTruck fireTrucks[MAX_TRUCKS];
static PoliceUnit policeUnits[MAX_POLICE];
static Pedestrian pedestrians[MAX_PEDESTRIANS];

// State
static MapLayer currentLayer = LAYER_SURFACE;
static ViewMode viewMode = MODE_2D;

// ---------------------------------------------------------------------------
// Procedural Generation
// ---------------------------------------------------------------------------
static bool IsRoad(int x, int y) {
    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return false;
    return surfaceMap[y][x] == TILE_ROAD;
}

static void AutoGenerateCity(void) {
    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            surfaceMap[y][x] = TILE_GRASS;
            undergroundMap[y][x] = TILE_GRASS;
            fireIntensity[y][x] = 0.0f;
            buildingHealth[y][x] = 100.0f;
        }
    }

    // Main Roads Grid
    for (int x = 0; x < MAP_W; x++) {
        surfaceMap[3][x] = TILE_ROAD;
        surfaceMap[9][x] = TILE_ROAD;
        surfaceMap[13][x] = TILE_ROAD;
    }
    for (int y = 0; y < MAP_H; y++) {
        surfaceMap[y][4] = TILE_ROAD;
        surfaceMap[y][11] = TILE_ROAD;
        surfaceMap[y][18] = TILE_ROAD;
    }

    // Underground Pipes under roads
    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            if (surfaceMap[y][x] == TILE_ROAD) undergroundMap[y][x] = TILE_PIPE;
        }
    }

    // Populate blocks with Buildings, Stations, and Trees
    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            if (surfaceMap[y][x] == TILE_GRASS) {
                int r = rand() % 100;
                if (r < 45) {
                    surfaceMap[y][x] = TILE_BUILDING;
                    buildingHealth[y][x] = 100.0f;
                } else if (r < 70) {
                    surfaceMap[y][x] = TILE_TREE;
                }
            }
        }
    }

    // Place Fire & Police Stations at specific blocks
    surfaceMap[2][2] = TILE_FIRE_STATION;
    surfaceMap[10][19] = TILE_POLICE_STATION;
    surfaceMap[11][2] = TILE_POLICE_STATION;
    surfaceMap[4][19] = TILE_FIRE_STATION;

    // Initialize Entities
    for (int i = 0; i < MAX_CARS; i++) cars[i].active = false;
    for (int i = 0; i < MAX_TRUCKS; i++) fireTrucks[i].active = false;
    for (int i = 0; i < MAX_POLICE; i++) policeUnits[i].active = false;
    for (int i = 0; i < MAX_PEDESTRIANS; i++) pedestrians[i].active = false;
}

// ---------------------------------------------------------------------------
// Simulation Update
// ---------------------------------------------------------------------------
static void UpdateSimulation(float dt) {
    if (rainLevel > 0.0f) {
        rainLevel -= dt * 0.15f;
        if (rainLevel < 0.0f) rainLevel = 0.0f;
    }

    // Demographics Ticker
    demoTimer += dt;
    if (demoTimer >= 5.0f) {
        demoTimer = 0.0f;
        statNewborns += (rand() % 3) + 1;
    }

    // --- Cars Traffic Wrapping & Movement ---
    for (int i = 0; i < MAX_CARS; i++) {
        if (!cars[i].active) {
            int rx = rand() % MAP_W;
            int ry = rand() % MAP_H;
            if (IsRoad(rx, ry)) {
                cars[i].gridX = rx;
                cars[i].gridY = ry;
                cars[i].pos = (Vector2){ rx * TILE_SIZE + TILE_SIZE/2, ry * TILE_SIZE + TILE_SIZE/2 };
                cars[i].targetX = rx;
                cars[i].targetY = ry;
                cars[i].progress = 1.0f;
                cars[i].active = true;
            }
        } else {
            if (cars[i].progress >= 1.0f) {
                int dx[] = { 0, 0, -1, 1 };
                int dy[] = { -1, 1, 0, 0 };
                int validDirs[4], validCount = 0;

                for (int d = 0; d < 4; d++) {
                    int nx = cars[i].gridX + dx[d];
                    int ny = cars[i].gridY + dy[d];
                    if (IsRoad(nx, ny)) {
                        validDirs[validCount++] = d;
                    }
                }

                if (validCount > 0) {
                    int dir = validDirs[rand() % validCount];
                    cars[i].gridX = cars[i].targetX;
                    cars[i].gridY = cars[i].targetY;
                    cars[i].targetX += dx[dir];
                    cars[i].targetY += dy[dir];
                    cars[i].progress = 0.0f;
                    cars[i].dir = dir;
                } else {
                    if (cars[i].targetX < 0) cars[i].targetX = MAP_W - 1;
                    else if (cars[i].targetX >= MAP_W) cars[i].targetX = 0;
                    if (cars[i].targetY < 0) cars[i].targetY = MAP_H - 1;
                    else if (cars[i].targetY >= MAP_H) cars[i].targetY = 0;
                    cars[i].gridX = cars[i].targetX;
                    cars[i].gridY = cars[i].targetY;
                    cars[i].progress = 0.0f;
                }
            } else {
                cars[i].progress += dt * 2.2f;
                if (cars[i].progress > 1.0f) cars[i].progress = 1.0f;

                float startX = cars[i].gridX * TILE_SIZE + TILE_SIZE/2;
                float startY = cars[i].gridY * TILE_SIZE + TILE_SIZE/2;
                float endX = cars[i].targetX * TILE_SIZE + TILE_SIZE/2;
                float endY = cars[i].targetY * TILE_SIZE + TILE_SIZE/2;

                cars[i].pos.x = startX + (endX - startX) * cars[i].progress;
                cars[i].pos.y = startY + (endY - startY) * cars[i].progress;
            }
        }
    }

    // --- Pedestrians & Criminals ---
    for (int i = 0; i < MAX_PEDESTRIANS; i++) {
        if (!pedestrians[i].active) {
            int px = rand() % MAP_W;
            int py = rand() % MAP_H;
            if (!IsRoad(px, py)) {
                pedestrians[i].gridX = px;
                pedestrians[i].gridY = py;
                pedestrians[i].pos = (Vector2){ px * TILE_SIZE + TILE_SIZE/2, py * TILE_SIZE + TILE_SIZE/2 };
                pedestrians[i].isCriminal = (rand() % 100 < 20);
                pedestrians[i].isRobbing = false;
                pedestrians[i].robTimer = 0.0f;
                pedestrians[i].active = true;
            }
        } else {
            if (pedestrians[i].isCriminal && !pedestrians[i].isRobbing) {
                for (int p = 0; p < MAX_PEDESTRIANS; p++) {
                    if (pedestrians[p].active && !pedestrians[p].isCriminal) {
                        float dist = hypot(pedestrians[i].pos.x - pedestrians[p].pos.x, pedestrians[i].pos.y - pedestrians[p].pos.y);
                        if (dist < 40.0f) {
                            pedestrians[i].isRobbing = true;
                            break;
                        }
                    }
                }
            }

            if (pedestrians[i].isRobbing) {
                pedestrians[i].robTimer += dt;
                if (pedestrians[i].robTimer >= 3.0f) {
                    pedestrians[i].isRobbing = false;
                    pedestrians[i].robTimer = 0.0f;
                }
            } else {
                float angle = (float)(rand() % 360) * (PI / 180.0f);
                pedestrians[i].pos.x += cos(angle) * 15.0f * dt;
                pedestrians[i].pos.y += sin(angle) * 15.0f * dt;
                if (pedestrians[i].pos.x < 0) pedestrians[i].pos.x = (float)(SCREEN_W - UI_WIDTH);
                if (pedestrians[i].pos.y < 0) pedestrians[i].pos.y = (float)SCREEN_H;
            }
        }
    }

    // --- Fire Spread & Stronger Firefighters ---
    fireSpreadTimer += dt;
    activeFiresCount = 0;

    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            if (fireIntensity[y][x] > 0.0f) {
                float suppression = (rainLevel * 20.0f) + (dt * 4.0f);

                for (int t = 0; t < MAX_TRUCKS; t++) {
                    if (fireTrucks[t].active) {
                        float distToFire = hypot(fireTrucks[t].pos.x - (x * TILE_SIZE + TILE_SIZE/2), fireTrucks[t].pos.y - (y * TILE_SIZE + TILE_SIZE/2));
                        if (distToFire < 50.0f) {
                            suppression += dt * 45.0f;
                            if (distToFire < 15.0f) {
                                fireTrucks[t].active = false;
                            }
                        }
                    }
                }

                fireIntensity[y][x] -= suppression;

                if (surfaceMap[y][x] == TILE_BUILDING) {
                    buildingHealth[y][x] -= dt * 10.0f;
                    if (buildingHealth[y][x] <= 0.0f) {
                        buildingHealth[y][x] = 0.0f;
                        surfaceMap[y][x] = TILE_RUIN;
                        statDeaths++;
                    }
                } else if (surfaceMap[y][x] == TILE_TREE) {
                    if (fireIntensity[y][x] <= 0.0f) {
                        surfaceMap[y][x] = TILE_GRASS;
                    }
                }

                if (fireIntensity[y][x] <= 0.0f) {
                    fireIntensity[y][x] = 0.0f;
                } else {
                    activeFiresCount++;
                    if (fireSpreadTimer >= 1.5f) {
                        int dx[] = { 0, 0, -1, 1 };
                        int dy[] = { -1, 1, 0, 0 };
                        for (int i = 0; i < 4; i++) {
                            int nx = x + dx[i];
                            int ny = y + dy[i];
                            if (nx >= 0 && nx < MAP_W && ny >= 0 && ny < MAP_H) {
                                if (surfaceMap[ny][nx] == TILE_BUILDING || surfaceMap[ny][nx] == TILE_TREE || surfaceMap[ny][nx] == TILE_GRASS) {
                                    if (fireIntensity[ny][nx] == 0.0f && (rand() % 100 < 25)) {
                                        fireIntensity[ny][nx] = 100.0f;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if (fireSpreadTimer >= 1.5f) fireSpreadTimer = 0.0f;

    // --- Dispatch Multiple Fire Trucks ---
    if (activeFiresCount > 0) {
        for (int y = 0; y < MAP_H; y++) {
            for (int x = 0; x < MAP_W; x++) {
                if (surfaceMap[y][x] == TILE_FIRE_STATION) {
                    for (int t = 0; t < MAX_TRUCKS; t++) {
                        if (!fireTrucks[t].active) {
                            for (int fy = 0; fy < MAP_H; fy++) {
                                for (int fx = 0; fx < MAP_W; fx++) {
                                    if (fireIntensity[fy][fx] > 0.0f) {
                                        fireTrucks[t].pos = (Vector2){ (float)(x * TILE_SIZE + TILE_SIZE/2), (float)(y * TILE_SIZE + TILE_SIZE/2) };
                                        fireTrucks[t].targetX = fx;
                                        fireTrucks[t].targetY = fy;
                                        fireTrucks[t].active = true;
                                        break;
                                    }
                                }
                                if (fireTrucks[t].active) break;
                            }
                        }
                    }
                }
            }
        }
    }

    // Move Fire Trucks
    for (int t = 0; t < MAX_TRUCKS; t++) {
        if (fireTrucks[t].active) {
            float destX = (float)(fireTrucks[t].targetX * TILE_SIZE + TILE_SIZE/2);
            float destY = (float)(fireTrucks[t].targetY * TILE_SIZE + TILE_SIZE/2);
            Vector2 dir = { destX - fireTrucks[t].pos.x, destY - fireTrucks[t].pos.y };
            float len = hypot(dir.x, dir.y);
            if (len > 2.0f) {
                dir.x /= len; dir.y /= len;
                fireTrucks[t].pos.x += dir.x * 140.0f * dt;
                fireTrucks[t].pos.y += dir.y * 140.0f * dt;
            }
        }
    }

    // --- Police Response to Crime ---
    for (int i = 0; i < MAX_PEDESTRIANS; i++) {
        if (pedestrians[i].active && pedestrians[i].isCriminal && pedestrians[i].isRobbing) {
            for (int p = 0; p < MAX_POLICE; p++) {
                if (!policeUnits[p].active) {
                    for (int py = 0; py < MAP_H; py++) {
                        for (int px = 0; px < MAP_W; px++) {
                            if (surfaceMap[py][px] == TILE_POLICE_STATION) {
                                policeUnits[p].pos = (Vector2){ (float)(px * TILE_SIZE + TILE_SIZE/2), (float)(py * TILE_SIZE + TILE_SIZE/2) };
                                policeUnits[p].targetX = (int)(pedestrians[i].pos.x / TILE_SIZE);
                                policeUnits[p].targetY = (int)(pedestrians[i].pos.y / TILE_SIZE);
                                policeUnits[p].active = true;
                                break;
                            }
                        }
                        if (policeUnits[p].active) break;
                    }
                }
            }
        }
    }

    // Move Police Units and Arrest Criminals
    for (int p = 0; p < MAX_POLICE; p++) {
        if (policeUnits[p].active) {
            float destX = (float)(policeUnits[p].targetX * TILE_SIZE + TILE_SIZE/2);
            float destY = (float)(policeUnits[p].targetY * TILE_SIZE + TILE_SIZE/2);
            Vector2 dir = { destX - policeUnits[p].pos.x, destY - policeUnits[p].pos.y };
            float len = hypot(dir.x, dir.y);
            if (len > 5.0f) {
                dir.x /= len; dir.y /= len;
                policeUnits[p].pos.x += dir.x * 150.0f * dt;
                policeUnits[p].pos.y += dir.y * 150.0f * dt;
            } else {
                policeUnits[p].active = false;
                statArrests++;
                for (int i = 0; i < MAX_PEDESTRIANS; i++) {
                    if (pedestrians[i].active && pedestrians[i].isCriminal) {
                        pedestrians[i].active = false;
                        break;
                    }
                }
            }
        }
    }

    // --- Statistics Ticker ---
    historyTimer += dt;
    if (historyTimer >= 0.5f) {
        historyTimer = 0.0f;
        int roads = 0, buildings = 0, pipes = 0, policeCount = 0;
        for (int y = 0; y < MAP_H; y++) {
            for (int x = 0; x < MAP_W; x++) {
                if (surfaceMap[y][x] == TILE_ROAD) roads++;
                if (surfaceMap[y][x] == TILE_BUILDING) buildings++;
                if (surfaceMap[y][x] == TILE_POLICE_STATION) policeCount++;
                if (undergroundMap[y][x] == TILE_PIPE) pipes++;
            }
        }

        energyUsage = (float)(buildings * 15 + roads * 2 + policeCount * 10);
        waterUsage = (float)(buildings * 10 + pipes * 1);
        pollutionLevel = (float)(roads * 1.5f + buildings * 3.0f + (activeFiresCount * 25.0f)) - (rainLevel * 10.0f);
        if (pollutionLevel < 0.0f) pollutionLevel = 0.0f;

        heatLevel = 20.0f + (buildings * 0.4f) + (roads * 0.2f) + (activeFiresCount * 4.0f) - (rainLevel * 3.0f);
        securityLevel = 60.0f + (policeCount * 20.0f) - (activeFiresCount * 5.0f);
        if (securityLevel > 100.0f) securityLevel = 100.0f;
        if (securityLevel < 0.0f) securityLevel = 0.0f;

        energyHistory[historyIndex] = energyUsage;
        pollutionHistory[historyIndex] = pollutionLevel;
        heatHistory[historyIndex] = heatLevel;
        historyIndex = (historyIndex + 1) % HISTORY_LEN;
    }
}

// ---------------------------------------------------------------------------
// Drawing Helpers
// ---------------------------------------------------------------------------
static void DrawRoadTile2D(int x, int y) {
    int px = x * TILE_SIZE;
    int py = y * TILE_SIZE;
    int cx = px + TILE_SIZE / 2;
    int cy = py + TILE_SIZE / 2;

    DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, COL_ROAD);

    bool hor = IsRoad(x - 1, y) || IsRoad(x + 1, y);
    bool ver = IsRoad(x, y - 1) || IsRoad(x, y + 1);

    int dash = TILE_SIZE / 2;
    if (hor && !ver) {
        DrawRectangle(cx - dash / 2, cy - LINE_THICK / 2, dash, LINE_THICK, COL_LINE);
    } else if (ver && !hor) {
        DrawRectangle(cx - LINE_THICK / 2, cy - dash / 2, LINE_THICK, dash, COL_LINE);
    }
}

static void DrawGrassTile2D(int x, int y) {
    int px = x * TILE_SIZE;
    int py = y * TILE_SIZE;
    DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, COL_GRASS);

    bool up    = IsRoad(x, y - 1);
    bool down  = IsRoad(x, y + 1);
    bool left  = IsRoad(x - 1, y);
    bool right = IsRoad(x + 1, y);

    if (up)    DrawRectangle(px, py, TILE_SIZE, SIDEWALK_W, COL_SIDEWALK);
    if (down)  DrawRectangle(px, py + TILE_SIZE - SIDEWALK_W, TILE_SIZE, SIDEWALK_W, COL_SIDEWALK);
    if (left)  DrawRectangle(px, py, SIDEWALK_W, TILE_SIZE, COL_SIDEWALK);
    if (right) DrawRectangle(px + TILE_SIZE - SIDEWALK_W, py, SIDEWALK_W, SIDEWALK_W, COL_SIDEWALK);
}

// ---------------------------------------------------------------------------
// UI & Dashboard
// ---------------------------------------------------------------------------
static void DrawSparkline(int x, int y, int w, int h, float *data, Color col, const char *label, float maxVal) {
    DrawRectangle(x, y, w, h, (Color){ 40, 45, 55, 255 });
    DrawRectangleLines(x, y, w, h, (Color){ 65, 75, 90, 255 });
    DrawText(label, x + 8, y + 5, 11, RAYWHITE);

    for (int i = 0; i < HISTORY_LEN - 1; i++) {
        int idx1 = (historyIndex + i) % HISTORY_LEN;
        int idx2 = (historyIndex + i + 1) % HISTORY_LEN;

        float v1 = data[idx1];
        float v2 = data[idx2];

        int x1 = x + 10 + (int)((float)i / (HISTORY_LEN - 1) * (w - 20));
        int y1 = (y + h - 8) - (int)((v1 / maxVal) * (h - 22));
        int x2 = x + 10 + (int)((float)(i + 1) / (HISTORY_LEN - 1) * (w - 20));
        int y2 = (y + h - 8) - (int)((v2 / maxVal) * (h - 22));

        if (y1 < y + 18) y1 = y + 18;
        if (y2 < y + 18) y2 = y + 18;

        DrawLine(x1, y1, x2, y2, col);
    }
}

static void DrawUI(void) {
    int uiX = MAP_W * TILE_SIZE;
    DrawRectangle(uiX, 0, UI_WIDTH, SCREEN_H, COL_PANEL);
    DrawLine(uiX, 0, uiX, SCREEN_H, (Color){ 55, 65, 80, 255 });

    int py = 12;
    DrawText("SMART CITY OBSERVER", uiX + 15, py, 17, RAYWHITE);
    py += 24;

    const char *layerName = (currentLayer == LAYER_SURFACE) ? "Surface View" :
                            (currentLayer == LAYER_UNDERGROUND) ? "Underground Pipes" : "Heat / Pollution Overlay";
    DrawText(TextFormat("Layer [L]: %s", layerName), uiX + 15, py, 10, YELLOW);
    py += 18;

    const char *viewName = (viewMode == MODE_2D) ? "2D Top-Down" : "3D Angled View";
    DrawText(TextFormat("View Mode [V]: %s", viewName), uiX + 15, py, 10, SKYBLUE);
    py += 24;

    DrawText(TextFormat("Newborns: %d", statNewborns), uiX + 15, py, 12, GREEN); py += 18;
    DrawText(TextFormat("Arrests Made: %d", statArrests), uiX + 15, py, 12, SKYBLUE); py += 18;
    DrawText(TextFormat("Deaths (Collapses): %d", statDeaths), uiX + 15, py, 12, RED); py += 18;
    DrawText(TextFormat("Active Fires: %d", activeFiresCount), uiX + 15, py, 12, COL_FIRE); py += 18;
    DrawText(TextFormat("Security Index: %.1f%%", securityLevel), uiX + 15, py, 12, LIGHTGRAY); py += 18;
    DrawText(TextFormat("City Heat: %.1f C", heatLevel), uiX + 15, py, 12, ORANGE); py += 18;
    DrawText(TextFormat("Air Pollution: %.1f AQI", pollutionLevel), uiX + 15, py, 12, LIGHTGRAY); py += 24;

    DrawSparkline(uiX + 15, py, UI_WIDTH - 30, 65, energyHistory, ORANGE, "Energy Load (kW)", 700.0f);
    py += 70;
    DrawSparkline(uiX + 15, py, UI_WIDTH - 30, 65, pollutionHistory, RED, "Pollution Index", 250.0f);
    py += 70;
    DrawSparkline(uiX + 15, py, UI_WIDTH - 30, 65, heatHistory, YELLOW, "City Temperature (C)", 60.0f);

    DrawText("[SPACE] Rain Storm  |  [Click] Fire", uiX + 15, SCREEN_H - 25, 10, DARKGRAY);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void) {
    InitWindow(SCREEN_W, SCREEN_H, "Smart City Sim - Autonomous & Detailed");
    SetTargetFPS(60);

    AutoGenerateCity();

    Camera3D camera = { 0 };
    camera.position = (Vector3){ (float)(MAP_W * TILE_SIZE) / 2.0f, 500.0f, 420.0f };
    camera.target = (Vector3){ (float)(MAP_W * TILE_SIZE) / 2.0f, 0.0f, 150.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsKeyPressed(KEY_L)) currentLayer = (MapLayer)((currentLayer + 1) % 3);
        if (IsKeyPressed(KEY_V)) viewMode = (viewMode == MODE_2D) ? MODE_3D : MODE_2D;
        if (IsKeyPressed(KEY_SPACE)) rainLevel = 7.0f;

        Vector2 m = GetMousePosition();
        int tx = (int)(m.x / TILE_SIZE);
        int ty = (int)(m.y / TILE_SIZE);
        if (tx >= 0 && tx < MAP_W && ty >= 0 && ty < MAP_H) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && currentLayer == LAYER_SURFACE) {
                if (surfaceMap[ty][tx] == TILE_BUILDING || surfaceMap[ty][tx] == TILE_TREE || surfaceMap[ty][tx] == TILE_GRASS) {
                    fireIntensity[ty][tx] = 100.0f;
                }
            }
        }

        UpdateSimulation(dt);

        BeginDrawing();
        ClearBackground(COL_GRASS);

        if (viewMode == MODE_2D) {
            for (int y = 0; y < MAP_H; y++) {
                for (int x = 0; x < MAP_W; x++) {
                    int px = x * TILE_SIZE;
                    int py = y * TILE_SIZE;

                    if (currentLayer == LAYER_SURFACE) {
                        if (surfaceMap[y][x] == TILE_ROAD) {
                            DrawRoadTile2D(x, y);
                        } else if (surfaceMap[y][x] == TILE_BUILDING) {
                            Color bCol = (buildingHealth[y][x] < 50.0f) ? COL_DAMAGED : COL_BUILDING;
                            DrawRectangle(px + 3, py + 3, TILE_SIZE - 6, TILE_SIZE - 6, bCol);
                            DrawRectangleLines(px + 3, py + 3, TILE_SIZE - 6, TILE_SIZE - 6, (Color){ 100, 60, 50, 255 });
                        } else if (surfaceMap[y][x] == TILE_FIRE_STATION) {
                            DrawRectangle(px + 2, py + 2, TILE_SIZE - 4, TILE_SIZE - 4, COL_FIRE_STATION);
                            DrawText("FIRE", px + 8, py + TILE_SIZE / 2 - 6, 10, RAYWHITE);
                        } else if (surfaceMap[y][x] == TILE_POLICE_STATION) {
                            DrawRectangle(px + 2, py + 2, TILE_SIZE - 4, TILE_SIZE - 4, COL_POLICE_STATION);
                            DrawText("POL", px + 10, py + TILE_SIZE / 2 - 6, 10, RAYWHITE);
                        } else if (surfaceMap[y][x] == TILE_TREE) {
                            DrawGrassTile2D(x, y);
                            DrawCircle(px + TILE_SIZE / 2, py + TILE_SIZE / 2, 14, COL_TREE);
                        } else if (surfaceMap[y][x] == TILE_RUIN) {
                            DrawRectangle(px + 2, py + 2, TILE_SIZE - 4, TILE_SIZE - 4, COL_RUIN);
                        } else {
                            DrawGrassTile2D(x, y);
                        }

                        if (fireIntensity[y][x] > 0.0f) {
                            DrawRectangle(px + 6, py + 6, TILE_SIZE - 12, TILE_SIZE - 12, COL_FIRE);
                        }
                    } else if (currentLayer == LAYER_UNDERGROUND) {
                        DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, (Color){ 35, 40, 48, 255 });
                        DrawRectangleLines(px, py, TILE_SIZE, TILE_SIZE, (Color){ 55, 65, 75, 255 });
                        if (undergroundMap[y][x] == TILE_PIPE) {
                            DrawCircle(px + TILE_SIZE / 2, py + TILE_SIZE / 2, 10, COL_PIPE);
                        }
                    } else if (currentLayer == LAYER_OVERLAY) {
                        Color heatCol = COL_GRASS;
                        if (fireIntensity[y][x] > 0.0f) heatCol = (Color){ 250, 50, 30, 230 };
                        else if (surfaceMap[y][x] == TILE_BUILDING) heatCol = (Color){ 210, 75, 55, 210 };
                        DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, heatCol);
                        DrawRectangleLines(px, py, TILE_SIZE, TILE_SIZE, (Color){ 25, 25, 25, 100 });
                    }
                }
            }

            if (currentLayer == LAYER_SURFACE) {
                for (int i = 0; i < MAX_PEDESTRIANS; i++) {
                    if (pedestrians[i].active) {
                        Color pCol = pedestrians[i].isCriminal ? COL_CRIMINAL : COL_CIVILIAN;
                        DrawCircleV(pedestrians[i].pos, 4.5f, pCol);
                    }
                }
                for (int i = 0; i < MAX_CARS; i++) {
                    if (cars[i].active) DrawCircleV(cars[i].pos, 5.5f, COL_CAR);
                }
                for (int t = 0; t < MAX_TRUCKS; t++) {
                    if (fireTrucks[t].active) DrawCircleV(fireTrucks[t].pos, 7.0f, COL_TRUCK);
                }
                for (int p = 0; p < MAX_POLICE; p++) {
                    if (policeUnits[p].active) DrawCircleV(policeUnits[p].pos, 7.0f, COL_POLICE_CAR);
                }
            }

        } else {
            BeginMode3D(camera);
            
            DrawPlane((Vector3){ (float)(MAP_W * TILE_SIZE) / 2.0f - TILE_SIZE / 2.0f, 0, (float)(MAP_H * TILE_SIZE) / 2.0f - TILE_SIZE / 2.0f },
                      (Vector2){ (float)(MAP_W * TILE_SIZE), (float)(MAP_H * TILE_SIZE) }, COL_GRASS);

            for (int y = 0; y < MAP_H; y++) {
                for (int x = 0; x < MAP_W; x++) {
                    float posX = (float)(x * TILE_SIZE);
                    float posZ = (float)(y * TILE_SIZE);

                    if (surfaceMap[y][x] == TILE_ROAD) {
                        DrawCube((Vector3){ posX, 0.5f, posZ }, (float)TILE_SIZE, 1.0f, (float)TILE_SIZE, COL_ROAD);
                    } else if (surfaceMap[y][x] == TILE_BUILDING) {
                        float height = 48.0f;
                        Color bCol = (buildingHealth[y][x] < 50.0f) ? COL_DAMAGED : COL_BUILDING;
                        DrawCube((Vector3){ posX, height / 2.0f, posZ }, (float)TILE_SIZE - 6, height, (float)TILE_SIZE - 6, bCol);
                        DrawCubeWires((Vector3){ posX, height / 2.0f, posZ }, (float)TILE_SIZE - 6, height, (float)TILE_SIZE - 6, (Color){ 50, 30, 20, 255 });
                    } else if (surfaceMap[y][x] == TILE_FIRE_STATION) {
                        DrawCube((Vector3){ posX, 22.0f, posZ }, (float)TILE_SIZE - 4, 44.0f, (float)TILE_SIZE - 4, COL_FIRE_STATION);
                    } else if (surfaceMap[y][x] == TILE_POLICE_STATION) {
                        DrawCube((Vector3){ posX, 22.0f, posZ }, (float)TILE_SIZE - 4, 44.0f, (float)TILE_SIZE - 4, COL_POLICE_STATION);
                    } else if (surfaceMap[y][x] == TILE_TREE) {
                        DrawCylinder((Vector3){ posX, 8.0f, posZ }, 3.0f, 3.0f, 16.0f, 6, COL_TREE_TRUNK);
                        DrawSphere((Vector3){ posX, 22.0f, posZ }, 14.0f, COL_TREE);
                    } else if (surfaceMap[y][x] == TILE_RUIN) {
                        DrawCube((Vector3){ posX, 6.0f, posZ }, (float)TILE_SIZE - 4, 12.0f, (float)TILE_SIZE - 4, COL_RUIN);
                    }

                    if (fireIntensity[y][x] > 0.0f) {
                        DrawCube((Vector3){ posX, 28.0f, posZ }, 22.0f, 36.0f, 22.0f, COL_FIRE);
                    }
                }
            }

            for (int i = 0; i < MAX_CARS; i++) {
                if (cars[i].active) {
                    DrawCube((Vector3){ cars[i].pos.x - TILE_SIZE / 2, 4.0f, cars[i].pos.y - TILE_SIZE / 2 }, 14.0f, 6.0f, 10.0f, COL_CAR);
                }
            }
            for (int t = 0; t < MAX_TRUCKS; t++) {
                if (fireTrucks[t].active) {
                    DrawCube((Vector3){ fireTrucks[t].pos.x - TILE_SIZE / 2, 6.0f, fireTrucks[t].pos.y - TILE_SIZE / 2 }, 18.0f, 8.0f, 12.0f, COL_TRUCK);
                }
            }
            for (int p = 0; p < MAX_POLICE; p++) {
                if (policeUnits[p].active) {
                    DrawCube((Vector3){ policeUnits[p].pos.x - TILE_SIZE / 2, 6.0f, policeUnits[p].pos.y - TILE_SIZE / 2 }, 16.0f, 8.0f, 12.0f, COL_POLICE_CAR);
                }
            }

            EndMode3D();
        }

        DrawUI();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}