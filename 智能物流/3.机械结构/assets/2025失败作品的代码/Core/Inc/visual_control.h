#ifndef __VISUAL_CONTROL_H
#define __VISUAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motor_pid.h"
#include <stdbool.h>

/* Public variables ----------------------------------------------------------*/
extern char target_color;
extern int target_x;
extern int target_y;
extern int target_angle;
extern volatile bool openmv_data_ready;

extern int center_x;
extern int center_y;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  【“一键式”阻塞型】通用视觉对准函数 (已修正).
 * @note   此函数将阻塞程序执行，直到对准完成或超时。
 *         它会对OpenMV发送的任何有效目标进行对准，不区分颜色。
 * @param  move_speed_rpm: 小车在对准过程中的移动速度 (RPM).
 * @param  dead_zone_pixels: 目标死区范围（像素）.
 * @param  timeout_ms: 最大执行时间（毫秒）. 0表示永不超时。
 * @return bool:
 *         - true: 在超时时间内成功完成对准。
 *         - false: 因超时或找不到目标而失败。
 */
// bool visual_align_blocking(float move_speed_rpm, int dead_zone_pixels, uint32_t timeout_ms);
bool visual_align_blocking_V2(int angle_dead_zone, float rotate_speed_rpm, float move_speed_rpm, int dead_zone_pixels, uint32_t timeout_ms);

/**
 * @brief  【V3 - 终极版】带比例控制和陀螺仪锁定的阻塞型视觉对准函数.
 * @note   此函数将阻塞程序执行，直到对准完成或超时。
 *         它能平滑地接近目标，并在平移时保持车身姿态稳定。
 *
 * @param  target_angle_deg   目标角度 (例如 90 度).
 * @param  angle_dead_zone    角度死区 (例如 3 度, 表示 87-93 度都算对准).
 * @param  rotate_speed_rpm   旋转时的最大电机速度.
 *
 * @param  pos_dead_zone_pixels 平移时的中心点死区 (像素).
 * @param  pos_max_speed_rpm    平移时的最大速度 (RPM).
 * @param  pos_min_speed_rpm    平移时的最小启动速度 (RPM), 用于克服静摩擦.
 * @param  pos_kp               平移比例控制的Kp系数.
 *
 * @param  yaw_lock_kp          平移时陀螺仪偏航锁定的Kp系数.
 * @param  timeout_ms           总超时时间 (毫秒).
 *
 * @retval bool: true 表示对准成功, false 表示超时或失败.
 */
bool visual_align_blocking_V3(int target_angle_deg, int angle_dead_zone, float rotate_speed_rpm, uint32_t timeout_ms);



void visual_align_blocking_111(void);
void visual_align_blocking_V3_111(void);
void final_destination_blocking(void);
void visual_align_blocking_V3_111_2(void);
#ifdef __cplusplus
}
#endif

#endif /* __VISUAL_CONTROL_H */