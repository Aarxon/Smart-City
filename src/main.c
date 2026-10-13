// Smart City Sim - Advanced 2D/3D Layered City Simulator (raylib, C)
//
// Controls:
//   Left Mouse   - Place selected tile
//   Right Mouse  - Remove tile (revert to grass)
//   1, 2, 3, 4   - Select Tool (Grass, Road, Building, Pipe)
//   L            - Cycle Layers (Surface -> Underground -> Overlay)
//   V            - Toggle 2D / 3D Camera View
//   SPACE        - Trigger rain storm

#include "raylib.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TILE_SIZE    48
#define MAP_W        20
#define MAP_H        14
#define UI_WIDTH     320
#define SCREEN_W     ((MAP_W * TILE_SIZE) + UI_WIDTH)
#define SCREEN_H     (MAP_H * TILE_SIZE)

#define SIDEWALK_W   8
#define LINE_THICK   3
#define HISTORY_LEN  100

typedef enum {
    TILE_GRASS = 0,
    TILE_ROAD,
    TILE_BUILDING,
    TILE_PIPE
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

// Colors
static const Color COL_GRASS     = {  86, 160,  70, 255 };
static const Color COL_ROAD      = { 100, 100, 105, 255 };
static const Color COL_BUILDING  = { 170, 110,  90, 255 };
static const Color COL_PIPE      = {  70, 130, 180, 255 };
static const Color COL_SIDEWALK  = { 222, 205, 165, 255 };
static const Color COL_LINE      = { 255, 255, 255, 255 };
static const Color COL_PANEL     = {  35,  38,  45, 255 };

// Map Data
static int surfaceMap[MAP_H][MAP_W];
static int undergroundMap[MAP_H][MAP_W];

// City Statistics & History
static float energyUsage = 0.0f;
static float waterUsage = 0.0f;
static float pollutionLevel = 0.0f;
static float heatLevel = 22.0f; // Celsius
static float rainLevel = 0.0f;

static float energyHistory[HISTORY_LEN] = {0};
static float pollutionHistory[HISTORY_LEN] = {0};
static float heatHistory[HISTORY_LEN] = {0};
static int historyIndex = 0;
static float historyTimer = 0.0f;

// State
static TileType currentTool = TILE_ROAD;
static MapLayer currentLayer = LAYER_SURFACE;
static ViewMode viewMode = MODE_2D;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static bool IsRoad(int x, int y) {
    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return false;
    return surfaceMap[y][x] == TILE_ROAD;
}

static void InitMap(void) {
    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            surfaceMap[y][x] = TILE_GRASS;
            undergroundMap[y][x] = TILE_GRASS;
        }
    }

    // Default Roads
    for (int x = 0; x < MAP_W; x++) {
        surfaceMap[2][x] = TILE_ROAD;
        surfaceMap[9][x] = TILE_ROAD;
    }
    for (int y = 0; y < MAP_H; y++) {
        surfaceMap[y][4] = TILE_ROAD;
        surfaceMap[y][14] = TILE_ROAD;
    }

    // Default Buildings & Pipes
    surfaceMap[3][5] = TILE_BUILDING;
    surfaceMap[3][6] = TILE_BUILDING;
    surfaceMap[8][5] = TILE_BUILDING;
    surfaceMap[8][6] = TILE_BUILDING;

    for (int x = 4; x <= 14; x++) undergroundMap[2][x] = TILE_PIPE;
}

// ---------------------------------------------------------------------------
// Simulation Update
// ---------------------------------------------------------------------------
static void UpdateSimulation(float dt) {
    // Dynamic rain dissipation
    if (rainLevel > 0.0f) {
        rainLevel -= dt * 0.1f;
        if (rainLevel < 0.0f) rainLevel = 0.0f;
    }

    // Ticker for graph history
    historyTimer += dt;
    if (historyTimer >= 0.5f) {
        historyTimer = 0.0f;
        
        // Calculate current metrics
        int roads = 0, buildings = 0, pipes = 0;
        for (int y = 0; y < MAP_H; y++) {
            for (int x = 0; x < MAP_W; x++) {
                if (surfaceMap[y][x] == TILE_ROAD) roads++;
                if (surfaceMap[y][x] == TILE_BUILDING) buildings++;
                if (undergroundMap[y][x] == TILE_PIPE) pipes++;
            }
        }

        energyUsage = (float)(buildings * 15 + roads * 2);
        waterUsage = (float)(buildings * 10 + pipes * 1);
        pollutionLevel = (float)(roads * 1.5f + buildings * 3.0f) - (rainLevel * 10.0f);
        if (pollutionLevel < 0.0f) pollutionLevel = 0.0f;

        heatLevel = 20.0f + (buildings * 0.4f) + (roads * 0.2f) - (rainLevel * 3.0f);

        // Shift history buffers
        energyHistory[historyIndex] = energyUsage;
        pollutionHistory[historyIndex] = pollutionLevel;
        heatHistory[historyIndex] = heatLevel;
        historyIndex = (historyIndex + 1) % HISTORY_LEN;
    }
}

// ---------------------------------------------------------------------------
// 2D Drawing helpers
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
    if (right) DrawRectangle(px + TILE_SIZE - SIDEWALK_W, py, SIDEWALK_W, TILE_SIZE, COL_SIDEWALK);
}

// ---------------------------------------------------------------------------
// UI & Graphs Rendering
// ---------------------------------------------------------------------------
static void DrawSparkline(int x, int y, int w, int h, float *data, Color col, const char *label, float maxVal) {
    DrawRectangle(x, y, w, h, (Color){ 45, 50, 60, 255 });
    DrawRectangleLines(x, y, w, h, (Color){ 70, 80, 95, 255 });
    DrawText(label, x + 8, y + 6, 12, RAYWHITE);

    // Draw line graph
    for (int i = 0; i < HISTORY_LEN - 1; i++) {
        int idx1 = (historyIndex + i) % HISTORY_LEN;
        int idx2 = (historyIndex + i + 1) % HISTORY_LEN;

        float v1 = data[idx1];
        float v2 = data[idx2];

        int x1 = x + 10 + (int)((float)i / (HISTORY_LEN - 1) * (w - 20));
        int y1 = (y + h - 10) - (int)((v1 / maxVal) * (h - 25));
        int x2 = x + 10 + (int)((float)(i + 1) / (HISTORY_LEN - 1) * (w - 20));
        int y2 = (y + h - 10) - (int)((v2 / maxVal) * (h - 25));

        if (y1 < y + 20) y1 = y + 20;
        if (y2 < y + 20) y2 = y + 20;

        DrawLine(x1, y1, x2, y2, col);
    }
}

static void DrawUI(void) {
    int uiX = MAP_W * TILE_SIZE;
    DrawRectangle(uiX, 0, UI_WIDTH, SCREEN_H, COL_PANEL);
    DrawLine(uiX, 0, uiX, SCREEN_H, (Color){ 60, 70, 85, 255 });

    int py = 15;
    DrawText("SMART CITY DASHBOARD", uiX + 15, py, 18, RAYWHITE);
    py += 30;

    // Current Layer & View info
    const char *layerName = (currentLayer == LAYER_SURFACE) ? "Surface (Roads/Buildings)" :
                            (currentLayer == LAYER_UNDERGROUND) ? "Underground (Pipes)" : "Heat/Pollution Overlay";
    DrawText(TextFormat("Layer [L]: %s", layerName), uiX + 15, py, 11, YELLOW);
    py += 20;

    const char *toolName = (currentTool == TILE_GRASS) ? "Grass / Erase" :
                           (currentTool == TILE_ROAD) ? "Road" :
                           (currentTool == TILE_BUILDING) ? "Building" : "Water Pipe";
    DrawText(TextFormat("Active Tool [1-4]: %s", toolName), uiX + 15, py, 11, LIGHTGRAY);
    py += 20;

    const char *viewName = (viewMode == MODE_2D) ? "2D Top-Down" : "3D Angled View";
    DrawText(TextFormat("View Mode [V]: %s", viewName), uiX + 15, py, 11, SKYBLUE);
    py += 35;

    // Numerical Stats
    DrawText(TextFormat("Energy Consumption: %.1f kW", energyUsage), uiX + 15, py, 13, ORANGE); py += 22;
    DrawText(TextFormat("Water Usage: %.1f L/s", waterUsage), uiX + 15, py, 13, SKYBLUE); py += 22;
    DrawText(TextFormat("Avg City Heat: %.1f C", heatLevel), uiX + 15, py, 13, RED); py += 22;
    DrawText(TextFormat("Air Pollution: %.1f AQI", pollutionLevel), uiX + 15, py, 13, LIGHTGRAY); py += 35;

    // Graphs
    DrawSparkline(uiX + 15, py, UI_WIDTH - 30, 85, energyHistory, ORANGE, "Energy Load (kW)", 500.0f);
    py += 95;
    DrawSparkline(uiX + 15, py, UI_WIDTH - 30, 85, pollutionHistory, RED, "Pollution Index", 200.0f);
    py += 95;
    DrawSparkline(uiX + 15, py, UI_WIDTH - 30, 85, heatHistory, YELLOW, "City Temperature (C)", 50.0f);

    // Controls footer
    DrawText("[SPACE] Rain  |  [C] Toggle View", uiX + 15, SCREEN_H - 30, 11, DARKGRAY);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void) {
    InitWindow(SCREEN_W, SCREEN_H, "Smart City Sim - Advanced");
    SetTargetFPS(60);

    InitMap();

    // Setup 3D Camera for angled perspective
    Camera3D camera = { 0 };
    camera.position = (Vector3){ (float)(MAP_W * TILE_SIZE) / 2.0f, 450.0f, 400.0f };
    camera.target = (Vector3){ (float)(MAP_W * TILE_SIZE) / 2.0f, 0.0f, 150.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // --- Input Handling ---
        if (IsKeyPressed(KEY_ONE)) currentTool = TILE_GRASS;
        if (IsKeyPressed(KEY_TWO)) currentTool = TILE_ROAD;
        if (IsKeyPressed(KEY_THREE)) currentTool = TILE_BUILDING;
        if (IsKeyPressed(KEY_FOUR)) currentTool = TILE_PIPE;
        if (IsKeyPressed(KEY_L)) currentLayer = (MapLayer)((currentLayer + 1) % 3);
        if (IsKeyPressed(KEY_V)) viewMode = (viewMode == MODE_2D) ? MODE_3D : MODE_2D;
        if (IsKeyPressed(KEY_SPACE)) rainLevel = 5.0f;

        // Mouse interaction (only in 2D or simplified raycast)
        Vector2 m = GetMousePosition();
        int tx = (int)(m.x / TILE_SIZE);
        int ty = (int)(m.y / TILE_SIZE);

        if (tx >= 0 && tx < MAP_W && ty >= 0 && ty < MAP_H) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                if (currentLayer == LAYER_UNDERGROUND) {
                    undergroundMap[ty][tx] = currentTool;
                } else {
                    surfaceMap[ty][tx] = currentTool;
                }
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                if (currentLayer == LAYER_UNDERGROUND) {
                    undergroundMap[ty][tx] = TILE_GRASS;
                } else {
                    surfaceMap[ty][tx] = TILE_GRASS;
                }
            }
        }

        // --- Update Simulation ---
        UpdateSimulation(dt);

        // --- Drawing ---
        BeginDrawing();
        ClearBackground(COL_GRASS);

        if (viewMode == MODE_2D) {
            // --- 2D Top-Down Mode ---
            for (int y = 0; y < MAP_H; y++) {
                for (int x = 0; x < MAP_W; x++) {
                    int px = x * TILE_SIZE;
                    int py = y * TILE_SIZE;

                    if (currentLayer == LAYER_SURFACE) {
                        if (surfaceMap[y][x] == TILE_ROAD) DrawRoadTile2D(x, y);
                        else if (surfaceMap[y][x] == TILE_BUILDING) {
                            DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, COL_BUILDING);
                            DrawRectangleLines(px, py, TILE_SIZE, TILE_SIZE, (Color){ 130, 80, 60, 255 });
                        } else {
                            DrawGrassTile2D(x, y);
                        }
                    } else if (currentLayer == LAYER_UNDERGROUND) {
                        DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, (Color){ 40, 45, 50, 255 });
                        DrawRectangleLines(px, py, TILE_SIZE, TILE_SIZE, (Color){ 60, 70, 80, 255 });
                        if (undergroundMap[y][x] == TILE_PIPE) {
                            DrawCircle(px + TILE_SIZE/2, py + TILE_SIZE/2, 12, COL_PIPE);
                        }
                    } else if (currentLayer == LAYER_OVERLAY) {
                        // Heat / Pollution heatmap view
                        Color heatCol = COL_GRASS;
                        if (surfaceMap[y][x] == TILE_BUILDING) heatCol = (Color){ 220, 80, 60, 200 };
                        else if (surfaceMap[y][x] == TILE_ROAD) heatCol = (Color){ 200, 160, 60, 200 };
                        DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, heatCol);
                        DrawRectangleLines(px, py, TILE_SIZE, TILE_SIZE, (Color){ 30, 30, 30, 100 });
                    }
                }
            }
        } else {
            // --- 3D Angled Perspective Mode ---
            BeginMode3D(camera);
            
            // Draw ground plane
            DrawPlane((Vector3){ (float)(MAP_W * TILE_SIZE)/2.0f - TILE_SIZE/2.0f, 0, (float)(MAP_H * TILE_SIZE)/2.0f - TILE_SIZE/2.0f },
                      (Vector2){ (float)(MAP_W * TILE_SIZE), (float)(MAP_H * TILE_SIZE) }, COL_GRASS);

            for (int y = 0; y < MAP_H; y++) {
                for (int x = 0; x < MAP_W; x++) {
                    float posX = x * TILE_SIZE;
                    float posZ = y * TILE_SIZE;

                    if (surfaceMap[y][x] == TILE_ROAD) {
                        DrawCube((Vector3){ posX, 1.0f, posZ }, (float)TILE_SIZE, 2.0f, (float)TILE_SIZE, COL_ROAD);
                    } else if (surfaceMap[y][x] == TILE_BUILDING) {
                        float height = 45.0f;
                        DrawCube((Vector3){ posX, height / 2.0f, posZ }, (float)TILE_SIZE - 4, height, (float)TILE_SIZE - 4, COL_BUILDING);
                        DrawCubeWires((Vector3){ posX, height / 2.0f, posZ }, (float)TILE_SIZE - 4, height, (float)TILE_SIZE - 4, BLACK);
                    }
                }
            }
            EndMode3D();
        }

        // Draw UI dashboard on the right
        DrawUI();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}