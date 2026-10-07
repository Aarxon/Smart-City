// Smart City Sim - basic 2D street layout (raylib, C)
//
// Controls:
//   Left mouse   - place road
//   Right mouse  - remove road (turns back into grass)

#include "raylib.h"
#include <stdbool.h>

#define TILE_SIZE   48
#define MAP_W       20
#define MAP_H       12
#define SCREEN_W    (MAP_W * TILE_SIZE)
#define SCREEN_H    (MAP_H * TILE_SIZE)

#define SIDEWALK_W  8   // sidewalk strip width in pixels
#define LINE_THICK  3   // centre line thickness in pixels

typedef enum { TILE_GRASS = 0, TILE_ROAD = 1 } TileType;

static const Color COL_GRASS    = {  86, 160,  70, 255 };
static const Color COL_ROAD     = { 100, 100, 105, 255 };
static const Color COL_SIDEWALK = { 222, 205, 165, 255 };
static const Color COL_LINE     = { 255, 255, 255, 255 };

static int map[MAP_H][MAP_W];

// ---------------------------------------------------------------------------
// Map helpers
// ---------------------------------------------------------------------------
static bool IsRoad(int x, int y)
{
    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return false;
    return map[y][x] == TILE_ROAD;
}

static void InitMap(void)
{
    // Everything starts as grass
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            map[y][x] = TILE_GRASS;

    // Two horizontal roads
    for (int x = 0; x < MAP_W; x++) {
        map[2][x] = TILE_ROAD;
        map[8][x] = TILE_ROAD;
    }

    // Three vertical roads
    for (int y = 0; y < MAP_H; y++) {
        map[y][3]  = TILE_ROAD;
        map[y][10] = TILE_ROAD;
        map[y][16] = TILE_ROAD;
    }

    // A short side street
    for (int x = 10; x <= 14; x++) map[5][x] = TILE_ROAD;
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
static void DrawRoadTile(int x, int y)
{
    int px = x * TILE_SIZE;
    int py = y * TILE_SIZE;
    int cx = px + TILE_SIZE / 2;
    int cy = py + TILE_SIZE / 2;

    DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, COL_ROAD);

    bool hor = IsRoad(x - 1, y) || IsRoad(x + 1, y);
    bool ver = IsRoad(x, y - 1) || IsRoad(x, y + 1);

    // Centre dashes only on straight sections (none on corners / junctions)
    int dash = TILE_SIZE / 2;
    if (hor && !ver) {
        DrawRectangle(cx - dash / 2, cy - LINE_THICK / 2, dash, LINE_THICK, COL_LINE);
    } else if (ver && !hor) {
        DrawRectangle(cx - LINE_THICK / 2, cy - dash / 2, LINE_THICK, dash, COL_LINE);
    }
}

static void DrawGrassTile(int x, int y)
{
    int px = x * TILE_SIZE;
    int py = y * TILE_SIZE;

    DrawRectangle(px, py, TILE_SIZE, TILE_SIZE, COL_GRASS);

    bool up    = IsRoad(x, y - 1);
    bool down  = IsRoad(x, y + 1);
    bool left  = IsRoad(x - 1, y);
    bool right = IsRoad(x + 1, y);

    // Sidewalk strips on any edge that touches a road
    if (up)    DrawRectangle(px, py, TILE_SIZE, SIDEWALK_W, COL_SIDEWALK);
    if (down)  DrawRectangle(px, py + TILE_SIZE - SIDEWALK_W, TILE_SIZE, SIDEWALK_W, COL_SIDEWALK);
    if (left)  DrawRectangle(px, py, SIDEWALK_W, TILE_SIZE, COL_SIDEWALK);
    if (right) DrawRectangle(px + TILE_SIZE - SIDEWALK_W, py, SIDEWALK_W, TILE_SIZE, COL_SIDEWALK);

    // Corner pieces where only a diagonal neighbour is a road
    if (!up   && !left  && IsRoad(x - 1, y - 1))
        DrawRectangle(px, py, SIDEWALK_W, SIDEWALK_W, COL_SIDEWALK);
    if (!up   && !right && IsRoad(x + 1, y - 1))
        DrawRectangle(px + TILE_SIZE - SIDEWALK_W, py, SIDEWALK_W, SIDEWALK_W, COL_SIDEWALK);
    if (!down && !left  && IsRoad(x - 1, y + 1))
        DrawRectangle(px, py + TILE_SIZE - SIDEWALK_W, SIDEWALK_W, SIDEWALK_W, COL_SIDEWALK);
    if (!down && !right && IsRoad(x + 1, y + 1))
        DrawRectangle(px + TILE_SIZE - SIDEWALK_W, py + TILE_SIZE - SIDEWALK_W, SIDEWALK_W, SIDEWALK_W, COL_SIDEWALK);
}

static void DrawMap(void)
{
    for (int y = 0; y < MAP_H; y++) {
        for (int x = 0; x < MAP_W; x++) {
            if (map[y][x] == TILE_ROAD) DrawRoadTile(x, y);
            else                        DrawGrassTile(x, y);
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    InitWindow(SCREEN_W, SCREEN_H, "Smart City Sim");
    SetTargetFPS(60);

    InitMap();

    while (!WindowShouldClose()) {
        // --- Update: paint / erase roads with the mouse ---
        Vector2 m = GetMousePosition();
        int tx = (int)(m.x / TILE_SIZE);
        int ty = (int)(m.y / TILE_SIZE);

        if (tx >= 0 && tx < MAP_W && ty >= 0 && ty < MAP_H) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))  map[ty][tx] = TILE_ROAD;
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) map[ty][tx] = TILE_GRASS;
        }

        // --- Draw ---
        BeginDrawing();
            ClearBackground(COL_GRASS);
            DrawMap();
            DrawText("LMB: add road   RMB: remove road", 10, 10, 18, BLACK);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}