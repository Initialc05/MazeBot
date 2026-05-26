/**
 * @file autonav.c
 * @brief Embedded automatic navigation: OGM + topology + planner + local policy.
 */
#include "autonav.h"
#include "FreeRTOS.h"
#include "bt_cmd.h"
#include "cmsis_os.h"
#include "encoder.h"
#include "im948.h"
#include "robot_state.h"
#include "task.h"
#include "uart_device.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265
#endif

/* Geometry tuned for the coursework 5x5 orthogonal maze demo. */
#define AUTONAV_CELL_SIZE_M          0.30f
#define AUTONAV_GOAL_CENTER_TOL_M    0.20f
#define AUTONAV_FRONT_CLEAR_M        0.35f
#define AUTONAV_FRONT_DANGER_M       0.18f
#define AUTONAV_SIDE_SAFE_M          0.22f
#define AUTONAV_SIDE_ADJUST_M        0.20f
#define AUTONAV_HEADING_TOL_DEG      8.0f
#define AUTONAV_WALL_ALIGN_TOL_DEG   12.0f
#define AUTONAV_MATCH_ACCEPT_SCORE   55.0f
#define AUTONAV_SETTLE_MS            300U
#define AUTONAV_DECISION_PERIOD_MS   50U
#define AUTONAV_DRY_TURN_MS          250U
#define AUTONAV_DRY_MOVE_MS          450U
#define AUTONAV_DRY_ADJUST_MS        250U
#define AUTONAV_SCAN_MIN_POINTS      10U

#if AUTONAV_COMMAND_OUTPUT
#define AUTONAV_DRY_RUN              0
#else
#define AUTONAV_DRY_RUN              1
#endif

/* RPLIDAR mounting offset: 0 deg is treated as the robot forward direction. */
#define AUTONAV_LIDAR_FORWARD_DEG    0.0f

/* OGM is small enough for F446 SRAM and large enough for a 5x5 demo maze. */
#define OGM_W                        64
#define OGM_H                        64
#define OGM_RES_M                    0.05f
#define OGM_OCC_INC                  6
#define OGM_FREE_DEC                 2
#define OGM_OCC_MAX                  80
#define OGM_FREE_MIN                 (-20)
#define OGM_OCC_THRESHOLD            12

#define SECTOR_COUNT                 36
#define SECTOR_WIDTH_DEG             10.0f
#define SECTOR_RANGE_MAX_M           2.00f
#define SCAN_POINT_MAX               72
#define PATH_MAX_CELLS               (AUTONAV_CELL_COUNT * AUTONAV_CELL_COUNT)

typedef enum {
    DIR_EAST = 0,
    DIR_NORTH,
    DIR_WEST,
    DIR_SOUTH,
    DIR_COUNT
} NavDir_t;

typedef enum {
    EDGE_UNKNOWN = 0,
    EDGE_OPEN,
    EDGE_WALL
} EdgeState_t;

typedef enum {
    LOCAL_STOP = 0,
    LOCAL_STRAIGHT,
    LOCAL_ADJUST_LEFT,
    LOCAL_ADJUST_RIGHT
} LocalAction_t;

typedef struct {
    int8_t x;
    int8_t y;
} Cell_t;

typedef struct {
    uint8_t edge[DIR_COUNT];
    uint8_t confidence[DIR_COUNT];
    uint8_t visited;
} TopoCell_t;

typedef struct {
    float x_body_m;
    float y_body_m;
    uint8_t quality;
} ScanPoint_t;

typedef struct {
    float distance_m;
    uint16_t hits;
    bool observed;
} Clearance_t;

static const int8_t dir_dx[DIR_COUNT] = { 1, 0, -1, 0 };
static const int8_t dir_dy[DIR_COUNT] = { 0, 1, 0, -1 };
static const float dir_yaw_deg[DIR_COUNT] = { 0.0f, 90.0f, 180.0f, -90.0f };
static const char *dir_name[DIR_COUNT] = { "E", "N", "W", "S" };

static volatile AutoNavMode_t g_mode = AUTONAV_OFF;
static volatile AutoNavState_t g_state = AUTONAV_STATE_OFF;
static AutoNavMetrics_t g_last_metrics;
static char g_status[AUTONAV_STATUS_LEN];

static int8_t g_ogm[OGM_H][OGM_W];
static TopoCell_t g_topo[AUTONAV_CELL_COUNT][AUTONAV_CELL_COUNT];

static float g_sector_min[SECTOR_COUNT];
static float g_stable_sector_min[SECTOR_COUNT];
static uint16_t g_sector_count[SECTOR_COUNT];
static uint16_t g_stable_sector_count[SECTOR_COUNT];
static uint16_t g_scan_points_in_rev;

static ScanPoint_t g_scan_points[SCAN_POINT_MAX];
static uint8_t g_scan_head;
static uint8_t g_scan_count;
static uint8_t g_lidar_decimator;

static ScanPoint_t g_match_points[SCAN_POINT_MAX];
static uint8_t g_match_point_count;
static bool g_match_points_integrated;
static uint32_t g_match_points_ms;

static Cell_t g_goal;
static Cell_t g_next_cell;
static Cell_t g_path[PATH_MAX_CELLS];
static uint8_t g_path_len;
static NavDir_t g_target_dir = DIR_EAST;

static uint32_t g_state_enter_ms;
static uint32_t g_last_status_ms;
static uint8_t g_stable_goal_count;
static bool g_action_started;
static uint8_t g_wall_adjust_phase;

static float g_origin_x_m;
static float g_origin_y_m;
static float g_origin_yaw_deg;
static float g_origin_nav_x_m;
static float g_origin_nav_y_m;
static bool g_origin_valid;
static Cell_t g_dry_cell;
static float g_dry_yaw_deg;

static float g_matched_x_m;
static float g_matched_y_m;
static float g_matched_yaw_deg;
static float g_match_score;
static bool g_match_accepted;

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int clampi_local(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float norm180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float deg_to_rad(float deg)
{
    return deg * (float)(M_PI / 180.0);
}

static float yaw_error_deg(float current, float target)
{
    return norm180(target - current);
}

static void autonav_lock(void)
{
    taskENTER_CRITICAL();
}

static void autonav_unlock(void)
{
    taskEXIT_CRITICAL();
}

static uint8_t opposite_dir(uint8_t dir)
{
    return (uint8_t)((dir + 2U) & 3U);
}

static bool cell_valid(Cell_t c)
{
    return c.x >= 0 && c.x < AUTONAV_CELL_COUNT &&
           c.y >= 0 && c.y < AUTONAV_CELL_COUNT;
}

static int manhattan(Cell_t a, Cell_t b)
{
    int dx = (int)a.x - (int)b.x;
    int dy = (int)a.y - (int)b.y;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    return dx + dy;
}

static Cell_t current_cell_from_pose(float x_m, float y_m)
{
    Cell_t c;
    c.x = (int8_t)clampi_local((int)lroundf(x_m / AUTONAV_CELL_SIZE_M), 0, AUTONAV_CELL_COUNT - 1);
    c.y = (int8_t)clampi_local((int)lroundf(y_m / AUTONAV_CELL_SIZE_M), 0, AUTONAV_CELL_COUNT - 1);
    return c;
}

static void capture_nav_origin_at(Cell_t cell, float nav_yaw_deg)
{
    g_origin_x_m = odom_x;
    g_origin_y_m = odom_y;
    g_origin_yaw_deg = norm180(AngleZ - nav_yaw_deg);
    g_origin_nav_x_m = (float)cell.x * AUTONAV_CELL_SIZE_M;
    g_origin_nav_y_m = (float)cell.y * AUTONAV_CELL_SIZE_M;
    g_origin_valid = true;
}

static void capture_nav_origin(void)
{
    Cell_t start = { 0, 0 };
    capture_nav_origin_at(start, 0.0f);
}

static void odom_to_nav_pose(float *x_m, float *y_m, float *yaw_deg)
{
    if (!g_origin_valid) {
        capture_nav_origin();
    }

    float dx = odom_x - g_origin_x_m;
    float dy = odom_y - g_origin_y_m;
    float yaw0 = deg_to_rad(g_origin_yaw_deg);
    float c = cosf(yaw0);
    float s = sinf(yaw0);

    *x_m = g_origin_nav_x_m + dx * c + dy * s;
    *y_m = g_origin_nav_y_m - dx * s + dy * c;
    *yaw_deg = norm180(AngleZ - g_origin_yaw_deg);
}

static void dry_pose(float *x_m, float *y_m, float *yaw_deg)
{
    *x_m = (float)g_dry_cell.x * AUTONAV_CELL_SIZE_M;
    *y_m = (float)g_dry_cell.y * AUTONAV_CELL_SIZE_M;
    *yaw_deg = g_dry_yaw_deg;
}

static float current_nav_yaw(void)
{
#if AUTONAV_DRY_RUN
    return g_dry_yaw_deg;
#else
    float x;
    float y;
    float yaw;
    odom_to_nav_pose(&x, &y, &yaw);
    (void)x;
    (void)y;
    return yaw;
#endif
}

static NavDir_t quantize_heading(float yaw_deg)
{
    float best_err = 999.0f;
    NavDir_t best = DIR_EAST;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        float err = fabsf(norm180(yaw_deg - dir_yaw_deg[d]));
        if (err < best_err) {
            best_err = err;
            best = (NavDir_t)d;
        }
    }
    return best;
}

static NavDir_t dir_between(Cell_t from, Cell_t to)
{
    int dx = (int)to.x - (int)from.x;
    int dy = (int)to.y - (int)from.y;
    if (dx > 0) return DIR_EAST;
    if (dx < 0) return DIR_WEST;
    if (dy > 0) return DIR_NORTH;
    return DIR_SOUTH;
}

static int turn_cost(NavDir_t from, NavDir_t to)
{
    int diff = ((int)to - (int)from) & 3;
    if (diff == 0) return 0;
    if (diff == 2) return 3;
    return 1;
}

static int sector_index_from_rel_deg(float rel_deg)
{
    float norm = norm180(rel_deg);
    int idx = (int)floorf((norm + 180.0f) / SECTOR_WIDTH_DEG);
    if (idx < 0) idx = 0;
    if (idx >= SECTOR_COUNT) idx = SECTOR_COUNT - 1;
    return idx;
}

static void reset_sector_buffers(void)
{
    for (uint8_t i = 0; i < SECTOR_COUNT; i++) {
        g_sector_min[i] = SECTOR_RANGE_MAX_M;
        g_stable_sector_min[i] = SECTOR_RANGE_MAX_M;
        g_sector_count[i] = 0;
        g_stable_sector_count[i] = 0;
    }
    g_scan_points_in_rev = 0;
}

static bool world_to_grid(float x_m, float y_m, int *gx, int *gy)
{
    int ix = (int)lroundf(x_m / OGM_RES_M) + (OGM_W / 2);
    int iy = (int)lroundf(y_m / OGM_RES_M) + (OGM_H / 2);
    if (ix < 0 || ix >= OGM_W || iy < 0 || iy >= OGM_H) {
        return false;
    }
    *gx = ix;
    *gy = iy;
    return true;
}

static void ogm_add(int gx, int gy, int delta)
{
    if (gx < 0 || gx >= OGM_W || gy < 0 || gy >= OGM_H) return;
    int v = (int)g_ogm[gy][gx] + delta;
    if (v > OGM_OCC_MAX) v = OGM_OCC_MAX;
    if (v < OGM_FREE_MIN) v = OGM_FREE_MIN;
    g_ogm[gy][gx] = (int8_t)v;
}

static void ogm_trace_free(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int x = x0;
    int y = y0;

    for (uint8_t step = 0; step < 80; step++) {
        if (x == x1 && y == y1) break;
        ogm_add(x, y, -OGM_FREE_DEC);
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
}

static int ogm_score_at(float x_m, float y_m)
{
    int gx, gy;
    if (!world_to_grid(x_m, y_m, &gx, &gy)) return -2;

    int best = -20;
    for (int yy = gy - 1; yy <= gy + 1; yy++) {
        for (int xx = gx - 1; xx <= gx + 1; xx++) {
            if (xx < 0 || xx >= OGM_W || yy < 0 || yy >= OGM_H) continue;
            if ((int)g_ogm[yy][xx] > best) best = (int)g_ogm[yy][xx];
        }
    }

    if (best >= OGM_OCC_THRESHOLD) return 2;
    if (best > 0) return 1;
    if (best <= OGM_FREE_MIN / 2) return -1;
    return 0;
}

static void update_ogm_body_point(float pose_x_m, float pose_y_m, float pose_yaw_deg,
                                  float x_body_m, float y_body_m)
{
    float yaw = deg_to_rad(pose_yaw_deg);
    float c = cosf(yaw);
    float s = sinf(yaw);
    float sx = pose_x_m;
    float sy = pose_y_m;
    float hx = sx + x_body_m * c - y_body_m * s;
    float hy = sy + x_body_m * s + y_body_m * c;

    int sxg, syg, hxg, hyg;
    if (!world_to_grid(sx, sy, &sxg, &syg)) return;
    if (!world_to_grid(hx, hy, &hxg, &hyg)) return;

    ogm_trace_free(sxg, syg, hxg, hyg);
    ogm_add(hxg, hyg, OGM_OCC_INC);
}

static void integrate_scan_points(const ScanPoint_t *points, uint8_t count,
                                  float pose_x_m, float pose_y_m, float pose_yaw_deg)
{
    for (uint8_t i = 0; i < count; i++) {
        update_ogm_body_point(pose_x_m, pose_y_m, pose_yaw_deg,
                              points[i].x_body_m, points[i].y_body_m);
    }
}

static void store_scan_point(float angle_deg, float dist_m, uint8_t quality)
{
    float rel = deg_to_rad(angle_deg - AUTONAV_LIDAR_FORWARD_DEG);
    g_scan_points[g_scan_head].x_body_m = dist_m * cosf(rel);
    g_scan_points[g_scan_head].y_body_m = dist_m * sinf(rel);
    g_scan_points[g_scan_head].quality = quality;
    g_scan_head = (uint8_t)((g_scan_head + 1U) % SCAN_POINT_MAX);
    if (g_scan_count < SCAN_POINT_MAX) g_scan_count++;
}

static void freeze_match_points_locked(uint32_t now_ms)
{
    uint8_t count = g_scan_count;
    if (count > SCAN_POINT_MAX) count = SCAN_POINT_MAX;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (uint8_t)((g_scan_head + SCAN_POINT_MAX - count + i) % SCAN_POINT_MAX);
        g_match_points[i] = g_scan_points[idx];
    }

    g_match_point_count = count;
    g_match_points_integrated = false;
    g_match_points_ms = now_ms;
}

static float scan_match_candidate_score(const ScanPoint_t *points, uint8_t count,
                                        float px, float py, float yaw_deg,
                                        uint8_t *used_out)
{
    float yaw = deg_to_rad(yaw_deg);
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    int score = 0;
    uint8_t used = 0;

    for (uint8_t i = 0; i < count; i++) {
        float bx = points[i].x_body_m;
        float by = points[i].y_body_m;
        float wx = px + bx * cy - by * sy;
        float wy = py + bx * sy + by * cy;
        score += ogm_score_at(wx, wy);
        used++;
    }

    if (used_out != NULL) *used_out = used;
    if (used == 0U) return 0.0f;
    return (float)score / (float)used;
}

static void run_scan_match(float odom_nav_x, float odom_nav_y, float odom_nav_yaw)
{
    static const float pos_offsets[] = { -0.02f, 0.0f, 0.02f };
    static const float yaw_offsets[] = { -3.0f, 0.0f, 3.0f };

    ScanPoint_t points[SCAN_POINT_MAX];
    uint8_t point_count;
    bool should_integrate = false;

    autonav_lock();
    point_count = g_match_point_count;
    for (uint8_t i = 0; i < point_count; i++) {
        points[i] = g_match_points[i];
    }
    if (point_count >= AUTONAV_SCAN_MIN_POINTS && !g_match_points_integrated) {
        g_match_points_integrated = true;
        should_integrate = true;
    }
    autonav_unlock();

    float best_raw = -999.0f;
    float best_x = odom_nav_x;
    float best_y = odom_nav_y;
    float best_yaw = odom_nav_yaw;
    uint8_t best_used = 0;

    if (point_count < AUTONAV_SCAN_MIN_POINTS) {
        g_matched_x_m = odom_nav_x;
        g_matched_y_m = odom_nav_y;
        g_matched_yaw_deg = odom_nav_yaw;
        g_match_score = 0.0f;
        g_match_accepted = false;
        return;
    }

    for (uint8_t ix = 0; ix < 3U; ix++) {
        for (uint8_t iy = 0; iy < 3U; iy++) {
            for (uint8_t it = 0; it < 3U; it++) {
                uint8_t used = 0;
                float cx = odom_nav_x + pos_offsets[ix];
                float cy = odom_nav_y + pos_offsets[iy];
                float ct = odom_nav_yaw + yaw_offsets[it];
                float raw = scan_match_candidate_score(points, point_count, cx, cy, ct, &used);
                if (used > 0U && raw > best_raw) {
                    best_raw = raw;
                    best_x = cx;
                    best_y = cy;
                    best_yaw = ct;
                    best_used = used;
                }
            }
        }
    }

    g_matched_x_m = best_x;
    g_matched_y_m = best_y;
    g_matched_yaw_deg = best_yaw;

    if (best_used < AUTONAV_SCAN_MIN_POINTS) {
        g_match_score = 0.0f;
        g_match_accepted = false;
        return;
    }

    g_match_score = clampf_local(50.0f + best_raw * 25.0f, 0.0f, 100.0f);
    g_match_accepted = (g_match_score >= AUTONAV_MATCH_ACCEPT_SCORE);

    if (should_integrate) {
        integrate_scan_points(points, point_count, best_x, best_y, best_yaw);
    }
}

static Clearance_t sector_clearance_ex(float rel_center_deg, float half_width_deg)
{
    Clearance_t result;
    result.distance_m = SECTOR_RANGE_MAX_M;
    result.hits = 0U;
    result.observed = false;

    autonav_lock();
    for (uint8_t i = 0; i < SECTOR_COUNT; i++) {
        float sector_center = -180.0f + ((float)i + 0.5f) * SECTOR_WIDTH_DEG;
        float err = fabsf(norm180(sector_center - rel_center_deg));
        if (err <= half_width_deg && g_stable_sector_count[i] > 0U) {
            result.observed = true;
            result.hits = (uint16_t)(result.hits + g_stable_sector_count[i]);
            if (g_stable_sector_min[i] < result.distance_m) {
                result.distance_m = g_stable_sector_min[i];
            }
        }
    }
    autonav_unlock();

    return result;
}

static Clearance_t dir_clearance_ex(NavDir_t dir, float yaw_deg)
{
    float rel = norm180(dir_yaw_deg[dir] - yaw_deg);
    return sector_clearance_ex(rel, 22.0f);
}

static float estimate_wall_angle_error(void)
{
    Clearance_t lf = sector_clearance_ex(55.0f, 12.0f);
    Clearance_t lb = sector_clearance_ex(125.0f, 12.0f);
    Clearance_t rf = sector_clearance_ex(-55.0f, 12.0f);
    Clearance_t rb = sector_clearance_ex(-125.0f, 12.0f);
    float accum = 0.0f;
    uint8_t n = 0;

    if (lf.observed && lb.observed) {
        accum += atan2f(lf.distance_m - lb.distance_m, 0.22f) * (float)(180.0 / M_PI);
        n++;
    }
    if (rf.observed && rb.observed) {
        accum += atan2f(rb.distance_m - rf.distance_m, 0.22f) * (float)(180.0 / M_PI);
        n++;
    }

    if (n == 0U) return 0.0f;
    return accum / (float)n;
}

static uint8_t update_edge(Cell_t cell, NavDir_t dir, EdgeState_t state)
{
    if (!cell_valid(cell)) return 0U;
    TopoCell_t *tc = &g_topo[cell.y][cell.x];
    tc->edge[dir] = (uint8_t)state;
    if (tc->confidence[dir] < 10U) tc->confidence[dir]++;

    Cell_t nb = { (int8_t)(cell.x + dir_dx[dir]), (int8_t)(cell.y + dir_dy[dir]) };
    if (cell_valid(nb)) {
        uint8_t od = opposite_dir((uint8_t)dir);
        g_topo[nb.y][nb.x].edge[od] = (uint8_t)state;
        if (g_topo[nb.y][nb.x].confidence[od] < 10U) {
            g_topo[nb.y][nb.x].confidence[od]++;
        }
    }
    return 1U;
}

static void collect_metrics(AutoNavMetrics_t *m)
{
    memset(m, 0, sizeof(*m));

    float odom_nav_x;
    float odom_nav_y;
    float odom_nav_yaw;
    odom_to_nav_pose(&odom_nav_x, &odom_nav_y, &odom_nav_yaw);

    run_scan_match(odom_nav_x, odom_nav_y, odom_nav_yaw);

    float pose_x = g_match_accepted ? g_matched_x_m : odom_nav_x;
    float pose_y = g_match_accepted ? g_matched_y_m : odom_nav_y;
    float pose_yaw = g_match_accepted ? g_matched_yaw_deg : odom_nav_yaw;

#if AUTONAV_DRY_RUN
    dry_pose(&pose_x, &pose_y, &pose_yaw);
    m->virtual_pose = true;
#endif

    Cell_t c = current_cell_from_pose(pose_x, pose_y);
    NavDir_t heading = quantize_heading(pose_yaw);

    m->cell_x = c.x;
    m->cell_y = c.y;

    Clearance_t forward = sector_clearance_ex(0.0f, 20.0f);
    Clearance_t left = sector_clearance_ex(90.0f, 18.0f);
    Clearance_t right = sector_clearance_ex(-90.0f, 18.0f);

    m->forward_observed = forward.observed;
    m->left_wall_observed = left.observed;
    m->right_wall_observed = right.observed;
    m->forward_clearance_m = forward.observed ? forward.distance_m : 0.0f;
    m->left_wall_dist_m = left.observed ? left.distance_m : SECTOR_RANGE_MAX_M;
    m->right_wall_dist_m = right.observed ? right.distance_m : SECTOR_RANGE_MAX_M;
    if (left.observed && right.observed) {
        m->nearest_wall_dist_m = fminf(m->left_wall_dist_m, m->right_wall_dist_m);
    } else if (left.observed) {
        m->nearest_wall_dist_m = m->left_wall_dist_m;
    } else if (right.observed) {
        m->nearest_wall_dist_m = m->right_wall_dist_m;
    } else {
        m->nearest_wall_dist_m = SECTOR_RANGE_MAX_M;
    }

    m->wall_angle_error_deg = estimate_wall_angle_error();
    m->heading_error_deg = yaw_error_deg(pose_yaw, dir_yaw_deg[g_target_dir]);

    float cx = (float)c.x * AUTONAV_CELL_SIZE_M;
    float cy = (float)c.y * AUTONAV_CELL_SIZE_M;
    float ex = pose_x - cx;
    float ey = pose_y - cy;
    m->center_error_m = sqrtf(ex * ex + ey * ey);
    m->match_score = g_match_score;
    m->match_accepted = g_match_accepted;

    uint8_t open_mask = 0;
    uint8_t open_count = 0;
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Clearance_t clear = dir_clearance_ex((NavDir_t)d, pose_yaw);
        if (clear.observed && clear.distance_m > 0.34f) {
            open_mask |= (uint8_t)(1U << d);
            open_count++;
        }
    }
    m->open_dirs_mask = open_mask;
    m->junction_score = open_count;

    (void)heading;
}

static void update_topology_from_metrics(const AutoNavMetrics_t *m)
{
    Cell_t c = { m->cell_x, m->cell_y };
    if (!cell_valid(c)) return;

    g_topo[c.y][c.x].visited = 1U;

    float yaw = g_match_accepted ? g_matched_yaw_deg : current_nav_yaw();
    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Clearance_t clear = dir_clearance_ex((NavDir_t)d, yaw);
        if (!clear.observed) {
            continue;
        }
        if (clear.distance_m < 0.26f) {
            update_edge(c, (NavDir_t)d, EDGE_WALL);
        } else if (clear.distance_m > 0.38f) {
            update_edge(c, (NavDir_t)d, EDGE_OPEN);
        }
    }
}

static bool edge_passable(Cell_t c, NavDir_t dir, float *cost_out)
{
    if (!cell_valid(c)) return false;
    Cell_t nb = { (int8_t)(c.x + dir_dx[dir]), (int8_t)(c.y + dir_dy[dir]) };
    if (!cell_valid(nb)) return false;

    uint8_t edge = g_topo[c.y][c.x].edge[dir];
    uint8_t conf = g_topo[c.y][c.x].confidence[dir];
    if (edge == EDGE_WALL) return false;

    float cost = 1.0f;
    if (edge == EDGE_UNKNOWN) cost += 1.2f;
    if (conf < 2U) cost += 0.3f;
    if (!g_topo[nb.y][nb.x].visited) cost += 0.2f;

    if (cost_out != NULL) *cost_out = cost;
    return true;
}

static bool astar_plan(Cell_t start, Cell_t goal, Cell_t *path, uint8_t *path_len)
{
    const uint8_t total = PATH_MAX_CELLS;
    float gscore[PATH_MAX_CELLS];
    float fscore[PATH_MAX_CELLS];
    int8_t parent[PATH_MAX_CELLS];
    uint8_t open[PATH_MAX_CELLS];
    uint8_t closed[PATH_MAX_CELLS];

    for (uint8_t i = 0; i < total; i++) {
        gscore[i] = 10000.0f;
        fscore[i] = 10000.0f;
        parent[i] = -1;
        open[i] = 0U;
        closed[i] = 0U;
    }

    uint8_t start_idx = (uint8_t)(start.y * AUTONAV_CELL_COUNT + start.x);
    uint8_t goal_idx = (uint8_t)(goal.y * AUTONAV_CELL_COUNT + goal.x);
    gscore[start_idx] = 0.0f;
    fscore[start_idx] = (float)manhattan(start, goal);
    open[start_idx] = 1U;

    for (;;) {
        int current_idx = -1;
        float best_f = 10000.0f;
        for (uint8_t i = 0; i < total; i++) {
            if (open[i] && !closed[i] && fscore[i] < best_f) {
                best_f = fscore[i];
                current_idx = (int)i;
            }
        }

        if (current_idx < 0) return false;
        if ((uint8_t)current_idx == goal_idx) break;

        open[current_idx] = 0U;
        closed[current_idx] = 1U;

        Cell_t cur = {
            (int8_t)(current_idx % AUTONAV_CELL_COUNT),
            (int8_t)(current_idx / AUTONAV_CELL_COUNT)
        };

        for (uint8_t d = 0; d < DIR_COUNT; d++) {
            float step_cost = 1.0f;
            if (!edge_passable(cur, (NavDir_t)d, &step_cost)) continue;
            Cell_t nb = { (int8_t)(cur.x + dir_dx[d]), (int8_t)(cur.y + dir_dy[d]) };
            uint8_t ni = (uint8_t)(nb.y * AUTONAV_CELL_COUNT + nb.x);
            if (closed[ni]) continue;

            float tentative = gscore[current_idx] + step_cost;
            if (!open[ni] || tentative < gscore[ni]) {
                parent[ni] = (int8_t)current_idx;
                gscore[ni] = tentative;
                fscore[ni] = tentative + (float)manhattan(nb, goal);
                open[ni] = 1U;
            }
        }
    }

    Cell_t reverse[PATH_MAX_CELLS];
    uint8_t len = 0U;
    int idx = goal_idx;
    while (idx >= 0 && len < PATH_MAX_CELLS) {
        reverse[len].x = (int8_t)(idx % AUTONAV_CELL_COUNT);
        reverse[len].y = (int8_t)(idx / AUTONAV_CELL_COUNT);
        len++;
        if ((uint8_t)idx == start_idx) break;
        idx = parent[idx];
    }

    if (len == 0U || reverse[len - 1U].x != start.x || reverse[len - 1U].y != start.y) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        path[i] = reverse[len - 1U - i];
    }
    *path_len = len;
    return true;
}

static bool choose_goal_directed_step(const AutoNavMetrics_t *m, Cell_t *next, NavDir_t *dir)
{
    Cell_t cur = { m->cell_x, m->cell_y };
    NavDir_t heading = quantize_heading(current_nav_yaw());
    int best_score = 32000;
    bool found = false;

    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Cell_t nb = { (int8_t)(cur.x + dir_dx[d]), (int8_t)(cur.y + dir_dy[d]) };
        if (!cell_valid(nb)) continue;
        uint8_t edge = g_topo[cur.y][cur.x].edge[d];
        if (edge == EDGE_WALL) continue;

        int score = manhattan(nb, g_goal) * 10;
        score += turn_cost(heading, (NavDir_t)d) * 3;
        if (edge == EDGE_UNKNOWN) score += 8;
        if (g_topo[cur.y][cur.x].confidence[d] < 2U) score += 4;
        if (!g_topo[nb.y][nb.x].visited) score -= 1;

        if (score < best_score) {
            best_score = score;
            *next = nb;
            *dir = (NavDir_t)d;
            found = true;
        }
    }

    return found;
}

static bool choose_frontier_step(Cell_t cur, Cell_t *next, NavDir_t *dir)
{
    NavDir_t heading = quantize_heading(current_nav_yaw());
    int best_score = 32000;
    bool found = false;

    for (uint8_t d = 0; d < DIR_COUNT; d++) {
        Cell_t nb = { (int8_t)(cur.x + dir_dx[d]), (int8_t)(cur.y + dir_dy[d]) };
        if (!cell_valid(nb)) continue;
        uint8_t edge = g_topo[cur.y][cur.x].edge[d];
        if (edge == EDGE_WALL) continue;
        if (edge != EDGE_UNKNOWN && g_topo[nb.y][nb.x].visited) continue;

        int score = manhattan(nb, g_goal) * 10 + turn_cost(heading, (NavDir_t)d) * 3;
        if (edge == EDGE_OPEN) score -= 3;

        if (score < best_score) {
            best_score = score;
            *next = nb;
            *dir = (NavDir_t)d;
            found = true;
        }
    }
    return found;
}

static bool is_goal_reached(const AutoNavMetrics_t *m)
{
    bool same_cell = (m->cell_x == g_goal.x && m->cell_y == g_goal.y);
    bool near_center = (m->center_error_m < AUTONAV_GOAL_CENTER_TOL_M);
    bool match_ok = m->match_accepted || m->match_score > AUTONAV_MATCH_ACCEPT_SCORE;

    if (same_cell && near_center) {
        if (match_ok || g_stable_goal_count >= 2U) return true;
        g_stable_goal_count++;
    } else {
        g_stable_goal_count = 0U;
    }
    return false;
}

static bool needs_wall_adjust(const AutoNavMetrics_t *m)
{
    bool near_left = m->left_wall_observed && (m->left_wall_dist_m < AUTONAV_SIDE_ADJUST_M);
    bool near_right = m->right_wall_observed && (m->right_wall_dist_m < AUTONAV_SIDE_ADJUST_M);

    return near_left || near_right ||
           (fabsf(m->wall_angle_error_deg) > AUTONAV_WALL_ALIGN_TOL_DEG) ||
           (m->center_error_m > 0.12f);
}

static bool is_straight_safe(const AutoNavMetrics_t *m)
{
    return m->forward_observed &&
           (m->forward_clearance_m > AUTONAV_FRONT_CLEAR_M) &&
           (m->nearest_wall_dist_m > AUTONAV_SIDE_SAFE_M) &&
           (fabsf(m->wall_angle_error_deg) < 10.0f) &&
           (fabsf(m->heading_error_deg) < AUTONAV_HEADING_TOL_DEG) &&
           (m->virtual_pose || m->match_accepted || m->match_score > 45.0f ||
            g_match_point_count > 20U);
}

static LocalAction_t evaluate_local_window(const AutoNavMetrics_t *m)
{
    if (!m->forward_observed || m->forward_clearance_m < AUTONAV_FRONT_DANGER_M) {
        return LOCAL_STOP;
    }

    float straight_score = m->forward_clearance_m * 10.0f
                         - fabsf(m->heading_error_deg) * 0.20f
                         - fabsf(m->wall_angle_error_deg) * 0.12f;
    float heading_err = yaw_error_deg(current_nav_yaw(), dir_yaw_deg[g_target_dir]);
    float left_dist = m->left_wall_observed ? m->left_wall_dist_m : 0.0f;
    float right_dist = m->right_wall_observed ? m->right_wall_dist_m : 0.0f;
    float left_score = left_dist * 8.0f + heading_err * 0.02f;
    float right_score = right_dist * 8.0f - heading_err * 0.02f;

    if (is_straight_safe(m) && straight_score >= left_score && straight_score >= right_score) {
        return LOCAL_STRAIGHT;
    }
    if (m->left_wall_observed && (!m->right_wall_observed || m->left_wall_dist_m < m->right_wall_dist_m)) {
        return LOCAL_ADJUST_RIGHT;
    }
    return LOCAL_ADJUST_LEFT;
}

static float compute_micro_adjust_deg(const AutoNavMetrics_t *m)
{
    float lateral_error = 0.0f;
    if (m->left_wall_observed && m->right_wall_observed) {
        lateral_error = (m->right_wall_dist_m - m->left_wall_dist_m) * 0.5f;
    } else if (m->left_wall_observed && m->left_wall_dist_m < AUTONAV_SIDE_ADJUST_M) {
        lateral_error = AUTONAV_SIDE_ADJUST_M - m->left_wall_dist_m;
    } else if (m->right_wall_observed && m->right_wall_dist_m < AUTONAV_SIDE_ADJUST_M) {
        lateral_error = m->right_wall_dist_m - AUTONAV_SIDE_ADJUST_M;
    }

    float corr = 0.65f * m->wall_angle_error_deg + 38.0f * lateral_error;
    return clampf_local(corr, -12.0f, 12.0f);
}

static void publish_status(const char *msg, bool force)
{
    uint32_t now = HAL_GetTick();
    if (msg != NULL) {
        strncpy(g_status, msg, sizeof(g_status) - 1U);
        g_status[sizeof(g_status) - 1U] = '\0';
    }

    if (force || now - g_last_status_ms > 700U) {
        printf("%s\r\n", g_status);
        BT_SendString(g_status);
        BT_SendString("\r\n");
        g_last_status_ms = now;
    }
}

static void format_metrics_status(char *out, uint16_t len, const char *tag, const AutoNavMetrics_t *m)
{
    int clear_cm = (int)lroundf(m->forward_clearance_m * 100.0f);
    int wall_cm = (int)lroundf(m->nearest_wall_dist_m * 100.0f);
    int err_d = (int)lroundf(m->wall_angle_error_deg);
    int match = (int)lroundf(m->match_score);
    snprintf(out, len, "NAV:%s clear=%d wall=%d err=%d match=%d",
             tag, clear_cm, wall_cm, err_d, match);
}

static void enter_state(AutoNavState_t state)
{
    g_state = state;
    g_state_enter_ms = HAL_GetTick();
    g_action_started = false;
    if (state != AUTONAV_WALL_ADJUST) {
        g_wall_adjust_phase = 0U;
    }
}

static void issue_turn_to_dir(NavDir_t dir)
{
    float delta = yaw_error_deg(current_nav_yaw(), dir_yaw_deg[dir]);
#if AUTONAV_COMMAND_OUTPUT
    BtCmd_ExecutePreciseTurn(delta);
#else
    (void)delta;
#endif
}

static void issue_move_cell(void)
{
#if AUTONAV_COMMAND_OUTPUT
    BtCmd_ExecutePreciseMove(AUTONAV_CELL_SIZE_M * 100.0f, 1);
#endif
}

static void issue_micro_adjust(float correction_deg)
{
#if AUTONAV_COMMAND_OUTPUT
    BtCmd_ExecutePreciseTurn(correction_deg);
#else
    (void)correction_deg;
#endif
}

#if AUTONAV_COMMAND_OUTPUT
static void issue_micro_forward(void)
{
    BtCmd_ExecutePreciseMove(12.0f, 1);
}
#endif

static void select_goal_for_mode(AutoNavMode_t mode)
{
    if (mode == AUTONAV_EXIT_TO_START) {
        g_goal.x = 0;
        g_goal.y = 0;
    } else {
        g_goal.x = AUTONAV_CELL_COUNT - 1;
        g_goal.y = AUTONAV_CELL_COUNT - 1;
    }
}

static Cell_t start_cell_for_mode(AutoNavMode_t mode)
{
    Cell_t c;
    if (mode == AUTONAV_EXIT_TO_START) {
        c.x = AUTONAV_CELL_COUNT - 1;
        c.y = AUTONAV_CELL_COUNT - 1;
    } else {
        c.x = 0;
        c.y = 0;
    }
    return c;
}

static float start_yaw_for_mode(AutoNavMode_t mode)
{
    return (mode == AUTONAV_EXIT_TO_START) ? 180.0f : 0.0f;
}

static void start_mode(AutoNavMode_t mode)
{
    g_mode = mode;
    select_goal_for_mode(mode);
    Cell_t start = start_cell_for_mode(mode);
    float start_yaw = start_yaw_for_mode(mode);
    capture_nav_origin_at(start, start_yaw);
    g_dry_cell = start;
    g_dry_yaw_deg = start_yaw;
    g_stable_goal_count = 0U;
    g_path_len = 0U;
    g_target_dir = quantize_heading(start_yaw);
    enter_state(AUTONAV_LOCALIZE_START);

    if (mode == AUTONAV_START_TO_EXIT) {
        RobotState_Set(ROBOT_NAVIGATING);
        publish_status("NAV:START_TO_EXIT", true);
    } else if (mode == AUTONAV_EXIT_TO_START) {
        RobotState_Set(ROBOT_RETURNING);
        publish_status("NAV:EXIT_TO_START", true);
    }
}

static void switch_to_return_mode(void)
{
    g_mode = AUTONAV_EXIT_TO_START;
    select_goal_for_mode(g_mode);
    g_stable_goal_count = 0U;
    g_path_len = 0U;
    g_target_dir = quantize_heading(current_nav_yaw());
    RobotState_Set(ROBOT_RETURNING);
    publish_status("NAV:EXIT_REACHED return_mode", true);
    enter_state(AUTONAV_UPDATE_TOPOLOGY);
}

static void handle_plan(const AutoNavMetrics_t *m)
{
    char line[AUTONAV_STATUS_LEN];
    Cell_t cur = { m->cell_x, m->cell_y };
    NavDir_t planned_dir = g_target_dir;
    Cell_t next = cur;
    bool found = false;
    bool junction = (m->junction_score >= 2U);

    if (junction) {
        found = choose_goal_directed_step(m, &next, &planned_dir);
        if (found) {
            int score = manhattan(next, g_goal);
            snprintf(line, sizeof(line), "NAV:JUNCTION turn=%s score=%d",
                     dir_name[planned_dir], score);
            publish_status(line, true);
        }
    }

    if (!found && astar_plan(cur, g_goal, g_path, &g_path_len) && g_path_len >= 2U) {
        next = g_path[1];
        planned_dir = dir_between(cur, next);
        snprintf(line, sizeof(line), "NAV:ASTAR next=(%d,%d) dir=%s len=%u",
                 next.x, next.y, dir_name[planned_dir], (unsigned)g_path_len);
        publish_status(line, true);
        found = true;
    }

    if (!found && choose_frontier_step(cur, &next, &planned_dir)) {
        snprintf(line, sizeof(line), "NAV:FRONTIER next=(%d,%d) dir=%s",
                 next.x, next.y, dir_name[planned_dir]);
        publish_status(line, true);
        found = true;
    }

    if (!found) {
        publish_status("NAV:RECOVERY no_path", true);
        enter_state(AUTONAV_RECOVERY);
        return;
    }

    g_next_cell = next;
    g_target_dir = planned_dir;
    enter_state(AUTONAV_ALIGN_TO_EDGE);
}

static void step_state_machine(const AutoNavMetrics_t *m)
{
    char line[AUTONAV_STATUS_LEN];

    switch (g_state) {
    case AUTONAV_LOCALIZE_START:
        if (m->match_accepted || HAL_GetTick() - g_state_enter_ms > 500U) {
            snprintf(line, sizeof(line), "NAV:LOCALIZED cell=(%d,%d) match=%d",
                     m->cell_x, m->cell_y, (int)lroundf(m->match_score));
            publish_status(line, true);
            enter_state(AUTONAV_UPDATE_TOPOLOGY);
        } else {
            publish_status("NAV:LOCALIZE waiting_match", false);
        }
        break;

    case AUTONAV_UPDATE_TOPOLOGY:
        update_topology_from_metrics(m);
        enter_state(AUTONAV_CHECK_GOAL);
        break;

    case AUTONAV_CHECK_GOAL:
        if (is_goal_reached(m)) {
            snprintf(line, sizeof(line), "NAV:GOAL cell=(%d,%d) center=%d match=%s",
                     m->cell_x, m->cell_y,
                     (int)lroundf(m->center_error_m * 100.0f),
                     m->match_accepted ? "ok" : "odom");
            publish_status(line, true);
            enter_state(AUTONAV_GOAL_REACHED);
        } else {
            enter_state(AUTONAV_PLAN);
        }
        break;

    case AUTONAV_PLAN:
        handle_plan(m);
        break;

    case AUTONAV_ALIGN_TO_EDGE:
        if (fabsf(m->heading_error_deg) <= AUTONAV_HEADING_TOL_DEG) {
            enter_state(AUTONAV_ADVANCE_ONE_CELL);
            break;
        }

        snprintf(line, sizeof(line), "NAV:ALIGN dir=%s herr=%d",
                 dir_name[g_target_dir], (int)lroundf(m->heading_error_deg));
        publish_status(line, !g_action_started);
        if (!g_action_started) {
            issue_turn_to_dir(g_target_dir);
            g_action_started = true;
        }
#if AUTONAV_DRY_RUN
        if (HAL_GetTick() - g_state_enter_ms > AUTONAV_DRY_TURN_MS) {
            g_dry_yaw_deg = dir_yaw_deg[g_target_dir];
            enter_state(AUTONAV_SETTLE_AND_MATCH);
        }
#elif AUTONAV_COMMAND_OUTPUT
        if (turnState == TURN_REACHED || HAL_GetTick() - g_state_enter_ms > 5500U) {
            enter_state(AUTONAV_SETTLE_AND_MATCH);
        }
#endif
        break;

    case AUTONAV_ADVANCE_ONE_CELL:
        if (m->forward_clearance_m < AUTONAV_FRONT_DANGER_M) {
            publish_status("NAV:RECOVERY front_blocked", true);
            enter_state(AUTONAV_RECOVERY);
            break;
        }
        if (needs_wall_adjust(m)) {
            enter_state(AUTONAV_WALL_ADJUST);
            break;
        }

        if (evaluate_local_window(m) == LOCAL_STRAIGHT && is_straight_safe(m)) {
            format_metrics_status(line, sizeof(line), "STRAIGHT", m);
            publish_status(line, !g_action_started);
            if (!g_action_started) {
                issue_move_cell();
                g_action_started = true;
            }
#if AUTONAV_DRY_RUN
            if (HAL_GetTick() - g_state_enter_ms > AUTONAV_DRY_MOVE_MS) {
                g_dry_cell = g_next_cell;
                enter_state(AUTONAV_SETTLE_AND_MATCH);
            }
#elif AUTONAV_COMMAND_OUTPUT
            if (moveState == MOVE_REACHED || HAL_GetTick() - g_state_enter_ms > 11000U) {
                enter_state(AUTONAV_SETTLE_AND_MATCH);
            }
#endif
        } else {
            publish_status("NAV:RECOVERY uncertain_local", true);
            enter_state(AUTONAV_RECOVERY);
        }
        break;

    case AUTONAV_WALL_ADJUST: {
        float corr = compute_micro_adjust_deg(m);
        snprintf(line, sizeof(line), "NAV:ADJUST wall=%d angle=%d corr=%d",
                 (int)lroundf(m->nearest_wall_dist_m * 100.0f),
                 (int)lroundf(m->wall_angle_error_deg),
                 (int)lroundf(corr));
        publish_status(line, !g_action_started);

        if (!g_action_started) {
            issue_micro_adjust(corr);
            g_action_started = true;
            g_wall_adjust_phase = 0U;
        }

#if AUTONAV_DRY_RUN
        if (HAL_GetTick() - g_state_enter_ms > AUTONAV_DRY_ADJUST_MS) {
            g_dry_yaw_deg = norm180(g_dry_yaw_deg + corr);
            enter_state(AUTONAV_SETTLE_AND_MATCH);
        }
#elif AUTONAV_COMMAND_OUTPUT
        if (g_wall_adjust_phase == 0U &&
            (turnState == TURN_REACHED || HAL_GetTick() - g_state_enter_ms > 3000U)) {
            issue_micro_forward();
            g_wall_adjust_phase = 1U;
            g_state_enter_ms = HAL_GetTick();
        } else if (g_wall_adjust_phase == 1U &&
                   (moveState == MOVE_REACHED || HAL_GetTick() - g_state_enter_ms > 2500U)) {
            enter_state(AUTONAV_SETTLE_AND_MATCH);
        }
#endif
        break;
    }

    case AUTONAV_SETTLE_AND_MATCH:
        if (HAL_GetTick() - g_state_enter_ms >= AUTONAV_SETTLE_MS) {
            enter_state(AUTONAV_UPDATE_TOPOLOGY);
        } else {
            publish_status("NAV:SETTLE match_verify", false);
        }
        break;

    case AUTONAV_RECOVERY:
        if (!g_action_started) {
            format_metrics_status(line, sizeof(line), "RECOVERY", m);
            publish_status(line, true);
#if AUTONAV_COMMAND_OUTPUT
            BtCmd_ProcessByte('x');
#endif
            g_action_started = true;
        }
        if (HAL_GetTick() - g_state_enter_ms > 600U) {
            enter_state(AUTONAV_UPDATE_TOPOLOGY);
        }
        break;

    case AUTONAV_GOAL_REACHED:
#if AUTONAV_COMMAND_OUTPUT
        if (!g_action_started) {
            BtCmd_ProcessByte('x');
            g_action_started = true;
        }
#endif
        if (g_mode == AUTONAV_START_TO_EXIT) {
            switch_to_return_mode();
        } else {
            RobotState_Set(ROBOT_IDLE);
            g_mode = AUTONAV_OFF;
            enter_state(AUTONAV_STATE_OFF);
            publish_status("NAV:DONE", true);
        }
        break;

    case AUTONAV_STATE_OFF:
    default:
        break;
    }
}

void AutoNav_Init(void)
{
    memset(g_ogm, 0, sizeof(g_ogm));
    memset(g_topo, 0, sizeof(g_topo));
    memset(&g_last_metrics, 0, sizeof(g_last_metrics));
    reset_sector_buffers();

    g_mode = AUTONAV_OFF;
    g_state = AUTONAV_STATE_OFF;
    g_scan_head = 0U;
    g_scan_count = 0U;
    g_match_point_count = 0U;
    g_match_points_integrated = true;
    g_match_points_ms = 0U;
    g_lidar_decimator = 0U;
    g_origin_valid = false;
    g_origin_nav_x_m = 0.0f;
    g_origin_nav_y_m = 0.0f;
    g_dry_cell.x = 0;
    g_dry_cell.y = 0;
    g_dry_yaw_deg = 0.0f;
    g_matched_x_m = 0.0f;
    g_matched_y_m = 0.0f;
    g_matched_yaw_deg = 0.0f;
    g_match_score = 0.0f;
    g_match_accepted = false;
    strncpy(g_status, "NAV:OFF", sizeof(g_status) - 1U);
    g_status[sizeof(g_status) - 1U] = '\0';
}

void AutoNav_StartToExit(void)
{
    start_mode(AUTONAV_START_TO_EXIT);
}

void AutoNav_ReturnToStart(void)
{
    start_mode(AUTONAV_EXIT_TO_START);
}

void AutoNav_Stop(void)
{
    g_mode = AUTONAV_OFF;
    enter_state(AUTONAV_STATE_OFF);
    publish_status("NAV:OFF", true);
}

void AutoNav_ObserveLidar(float angle_deg, uint16_t distance_mm, uint8_t quality)
{
    if (distance_mm == 0U || quality == 0U) return;

    float dist_m = (float)distance_mm * 0.001f;
    if (dist_m <= 0.0f || dist_m > SECTOR_RANGE_MAX_M) return;

    float rel = norm180(angle_deg - AUTONAV_LIDAR_FORWARD_DEG);
    int sidx = sector_index_from_rel_deg(rel);

    autonav_lock();
    if (dist_m < g_sector_min[sidx]) {
        g_sector_min[sidx] = dist_m;
    }
    if (g_sector_count[sidx] < 0xFFFFU) g_sector_count[sidx]++;
    g_scan_points_in_rev++;

    g_lidar_decimator++;
    if ((g_lidar_decimator % 3U) == 0U) {
        store_scan_point(angle_deg, dist_m, quality);
    }
    autonav_unlock();
}

void AutoNav_NotifyScanStart(void)
{
    uint32_t now = HAL_GetTick();

    autonav_lock();
    if (g_scan_points_in_rev >= 8U) {
        for (uint8_t i = 0; i < SECTOR_COUNT; i++) {
            g_stable_sector_min[i] = g_sector_min[i];
            g_stable_sector_count[i] = g_sector_count[i];
        }
    }

    for (uint8_t i = 0; i < SECTOR_COUNT; i++) {
        g_sector_min[i] = SECTOR_RANGE_MAX_M;
        g_sector_count[i] = 0U;
    }

    if (g_scan_count >= AUTONAV_SCAN_MIN_POINTS) {
        freeze_match_points_locked(now);
    }

    g_scan_points_in_rev = 0U;
    autonav_unlock();
}

void AutoNav_Tick(void)
{
    RobotState_t rs = RobotState_Get();

    if (g_estop_latched || rs == ROBOT_FAULT) {
        if (g_mode != AUTONAV_OFF) AutoNav_Stop();
        return;
    }

    if (rs == ROBOT_EXPLORING || rs == ROBOT_NAVIGATING) {
        if (g_mode != AUTONAV_START_TO_EXIT) {
            start_mode(AUTONAV_START_TO_EXIT);
        }
    } else if (rs == ROBOT_RETURNING) {
        if (g_mode != AUTONAV_EXIT_TO_START) {
            start_mode(AUTONAV_EXIT_TO_START);
        }
    } else if (rs == ROBOT_IDLE && g_mode != AUTONAV_OFF) {
        AutoNav_Stop();
        return;
    }

    if (g_mode == AUTONAV_OFF) return;

    AutoNavMetrics_t m;
    collect_metrics(&m);
    g_last_metrics = m;
    step_state_machine(&m);
}

void AutoNavTask(void const *argument)
{
    (void)argument;
    for (;;) {
        AutoNav_Tick();
        osDelay(AUTONAV_DECISION_PERIOD_MS);
    }
}

bool AutoNav_IsActive(void)
{
    return g_mode != AUTONAV_OFF;
}

AutoNavMode_t AutoNav_GetMode(void)
{
    return g_mode;
}

AutoNavState_t AutoNav_GetState(void)
{
    return g_state;
}

void AutoNav_GetMetrics(AutoNavMetrics_t *out)
{
    if (out != NULL) {
        *out = g_last_metrics;
    }
}

void AutoNav_GetStatusLine(char *out, uint16_t len)
{
    if (out == NULL || len == 0U) return;
    strncpy(out, g_status, len - 1U);
    out[len - 1U] = '\0';
}

const char *AutoNav_StateName(AutoNavState_t state)
{
    switch (state) {
    case AUTONAV_STATE_OFF: return "OFF";
    case AUTONAV_LOCALIZE_START: return "LOCALIZE";
    case AUTONAV_UPDATE_TOPOLOGY: return "TOPO";
    case AUTONAV_CHECK_GOAL: return "GOAL?";
    case AUTONAV_PLAN: return "PLAN";
    case AUTONAV_ALIGN_TO_EDGE: return "ALIGN";
    case AUTONAV_ADVANCE_ONE_CELL: return "ADVANCE";
    case AUTONAV_WALL_ADJUST: return "ADJUST";
    case AUTONAV_SETTLE_AND_MATCH: return "SETTLE";
    case AUTONAV_RECOVERY: return "RECOVERY";
    case AUTONAV_GOAL_REACHED: return "DONE";
    default: return "?";
    }
}
