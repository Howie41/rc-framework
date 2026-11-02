#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 在 RTOS 资源创建完成后、调度器启动前初始化机器人。 */
void robot_init(void);

#ifdef __cplusplus
}
#endif
