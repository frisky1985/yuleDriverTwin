/*
 * FreeRTOSConfig.h — dtwin FreeRTOS 镜像（Cortex-M4F / ARM_CM4F port）
 *
 * 时钟对齐 QEMU mps2-an386（SYSCLK=25MHz）：configCPU_CLOCK_HZ=25MHz、
 * configSYSTICK_CLOCK_HZ=25MHz（与 CPU 同频 → port 置 CLKSOURCE=1）、
 * configTICK_RATE_HZ=1000 → SysTick LOAD = 24999。
 *
 * 关键配置（FRT-FW-01）：
 *   - configUSE_PREEMPTION=1 + configUSE_TIME_SLICING=1：抢占 + 同优先级时间片
 *   - configMAX_PRIORITIES=8、configUSE_PORT_OPTIMISED_TASK_SELECTION=1（依赖 CLZ）
 *   - configMAX_SYSCALL_INTERRUPT_PRIORITY=5：临界区 BASEPRI 值（屏蔽数值 ≥5 的异常）
 *   - configUSE_TICKLESS_IDLE=0：本阶段关闭（避开 cpsid/tickless 复杂路径）
 *   - configTOTAL_HEAP_SIZE=32KB、configSUPPORT_STATIC_ALLOCATION=1（idle 任务静态栈）
 *
 * 诚实标注：
 *   - 不定义 configASSERT（configCHECK_HANDLER_INSTALLATION=0）：ARM_CM4F port 在
 *     configASSERT_DEFINED 时会探测 NVIC IP 优先级寄存器（0xE000E400 读回），
 *     本模拟器未建模 NVIC IP 寄存器（本阶段 MAY）；固件正确性由任务创建返回值检查
 *     + E2E 输出序列与 QEMU 黄金对照共同保证。
 *   - configUSE_TIMERS/队列/信号量等内核对象全部关闭（FRT-AC-11 边界）。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* 时钟（与 QEMU mps2-an386 SYSCLK 对齐） */
#define configCPU_CLOCK_HZ                      ( 25000000UL )
#define configSYSTICK_CLOCK_HZ                  ( 25000000UL )
#define configTICK_RATE_HZ                      ( 1000 )

/* 调度 */
#define configUSE_PREEMPTION                    1
/* 时间片：默认开启；时间片变体固件（main_freertos_timeslice.c）的对照实验
 * 通过 -DconfigUSE_TIME_SLICING=0 覆盖构建（FRT-AC-02 证据：同固件仅此一处
 * 配置差异 → 输出从交替退化为 A 全量先跑完） */
#ifndef configUSE_TIME_SLICING
    #define configUSE_TIME_SLICING              1
#endif
#define configMAX_PRIORITIES                    ( 8 )
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TICKLESS_IDLE                 0
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS

/* 内存 */
#define configMINIMAL_STACK_SIZE                ( 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 32 * 1024 ) )
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configSTACK_DEPTH_TYPE                  uint32_t

/* 中断/BASEPRI（4 位优先级；临界区 BASEPRI=5） */
#define configPRIO_BITS                         4
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

/* 本阶段关闭的内核对象（FRT-AC-11 边界） */
#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configUSE_EVENT_GROUPS                  0
#define configUSE_TIMERS                        0
#define configUSE_TASK_NOTIFICATIONS            0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* 钩子（失败可观测，打印标记后挂起） */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1

/* 内存保护/调试（本阶段关闭） */
#define configENABLE_MPU                        0
#define configENABLE_FPU                        0
#define configENABLE_TRUSTZONE                  0
#define configENABLE_ACCESS_CONTROL_LIST        0
#define configENABLE_BTI                        0
#define configENABLE_PAC                        0
#define configENABLE_MVE                        0
#define configENABLE_DSP                        1
#define configNUMBER_OF_CORES                   1
#define configRUN_FREERTOS_SECURE_ONLY          1
#define configCHECK_HANDLER_INSTALLATION        0

#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskSuspend                    0
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          0

/* 由启动代码提供（向量表直接引用，避免链接器警告） */
extern void vPortSVCHandler( void );
extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );

#endif /* FREERTOS_CONFIG_H */
