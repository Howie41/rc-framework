/**
 * 把 `rtos_resource_init()` 放在 freertos.c 中的 MX_FREERTOS_Init() 中调用
 * 确保在内核启动之前就创建所有的资源
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
void rtos_resource_init(void);
#ifdef __cplusplus
}
#endif
