/**
 * @file autonav.h
 * @brief Fully embedded MazeBot Sense-Think-Act navigation demo.
 *
 * Runtime navigation decisions are computed on the STM32F446RE.  The host may
 * display telemetry, but LiDAR processing, OGM update, topology update, A* /
 * frontier selection, local obstacle checks, goal validation and command
 * selection are all performed by this module.
 */
#ifndef AUTONAV_H
#define AUTONAV_H

#include <stdbool.h>
#include <stdint.h>

#ifndef AUTONAV_COMMAND_OUTPUT
#define AUTONAV_COMMAND_OUTPUT 0
#endif

#define AUTONAV_CELL_COUNT      5
#define AUTONAV_STATUS_LEN      96

typedef enum {
    AUTONAV_OFF = 0,
    AUTONAV_START_TO_EXIT,
    AUTONAV_EXIT_TO_START
} AutoNavMode_t;

typedef enum {
    AUTONAV_STATE_OFF = 0,
    AUTONAV_LOCALIZE_START,
    AUTONAV_UPDATE_TOPOLOGY,
    AUTONAV_CHECK_GOAL,
    AUTONAV_PLAN,
    AUTONAV_ALIGN_TO_EDGE,
    AUTONAV_ADVANCE_ONE_CELL,
    AUTONAV_WALL_ADJUST,
    AUTONAV_SETTLE_AND_MATCH,
    AUTONAV_RECOVERY,
    AUTONAV_GOAL_REACHED
} AutoNavState_t;

typedef struct {
    float forward_clearance_m;
    float left_wall_dist_m;
    float right_wall_dist_m;
    float nearest_wall_dist_m;
    float wall_angle_error_deg;
    float heading_error_deg;
    float center_error_m;
    float match_score;
    bool  match_accepted;
    bool  forward_observed;
    bool  left_wall_observed;
    bool  right_wall_observed;
    bool  virtual_pose;
    uint8_t junction_score;
    uint8_t open_dirs_mask;
    int8_t cell_x;
    int8_t cell_y;
} AutoNavMetrics_t;

void AutoNav_Init(void);
void AutoNav_StartToExit(void);
void AutoNav_ReturnToStart(void);
void AutoNav_Stop(void);
void AutoNav_Tick(void);
void AutoNavTask(void const *argument);

void AutoNav_ObserveLidar(float angle_deg, uint16_t distance_mm, uint8_t quality);
void AutoNav_NotifyScanStart(void);

bool AutoNav_IsActive(void);
AutoNavMode_t AutoNav_GetMode(void);
AutoNavState_t AutoNav_GetState(void);
void AutoNav_GetMetrics(AutoNavMetrics_t *out);
void AutoNav_GetStatusLine(char *out, uint16_t len);
const char *AutoNav_StateName(AutoNavState_t state);

#endif /* AUTONAV_H */
