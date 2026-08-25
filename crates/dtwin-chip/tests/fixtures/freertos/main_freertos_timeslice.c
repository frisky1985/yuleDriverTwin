/*
 * main_freertos_timeslice.c — FreeRTOS 时间片轮转专用变体固件（FRT-AC-02）
 *
 * 与主固件（main_freertos.c，vTaskDelay 驱动）不同，本变体只为验证
 * configUSE_TIME_SLICING=1 下**真实的时间片抢占**：
 *
 *   - 仅 2 个同优先级（pri2）任务 TS_A / TS_B，各打印 N=40 行 [TS]；
 *   - 循环内**不阻塞**（无 vTaskDelay/vTaskSuspend/yield）：每次迭代 =
 *     忙等自己的轮次 + 打印一行 + 翻转轮次标志（纯忙循环，任务全程保持
 *     Ready，唯一能抢走 CPU 的机制是 configUSE_TIME_SLICING 的每 tick 轮转）。
 *
 * 时间片触发证据（双跑逐行一致）：
 *   1. 轮次标志 g_turn 初始 = TS_B：调度器启动后第一个运行的任务恒为
 *      pxCurrentTCB（FreeRTOS 语义：最高优先级任务中**最后创建**者，即
 *      TS_B——两个模拟器运行同一固件，此语义确定）→ 首行恒为 [TS] B 0，
 *      与首个 tick 的相位无关（QEMU 首 tick 落点 / dtwin 25000 周期处均一致）；
 *   2. 每 tick 时间片旋转一次 → 轮次交替 → 输出严格交替 B0 A0 B1 A1 …，
 *      每任务恰 N 行；旋转调度（B↔A）由 FreeRTOS 时间片旋转 + PendSV 决定，
 *      与 SysTick 寄存器可观测性无关（不依赖 COUNTFLAG/周期相位，双跑可复现）；
 *   3. 对照构建（-DconfigUSE_TIME_SLICING=0）：轮次翻转后无人切换 →
 *      输出恒为 [TS] B 0 后不再推进（第二任务永不运行）——唯一配置差异即
 *      时间片开关，证明交替由时间片旋转触发而非其他机制。
 *
 * 忙循环确定性：轮次忙等为内存旋转（volatile 读），不依赖 tick 观察；
 * 迭代次数固定（N=40）；打印受临界区保护（BASEPRI 屏蔽 SysTick），行不交错。
 *
 * 输出行前缀：[PASS]/[TS]（FRT-FW-02 统一前缀）。
 */
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ---------------- CMSDK APB UART @ 0x40004000 ---------------- */
#define UART_BASE 0x40004000UL
#define REG_DATA  0x000UL
#define REG_STATE 0x004UL
#define REG_CTRL  0x008UL
#define REG_BAUD  0x010UL
#define STATE_TXFULL (1U << 0)

static void uart_init(void)
{
    volatile uint32_t *uart = (volatile uint32_t *)UART_BASE;
    uart[REG_CTRL / 4] = 0;
    uart[REG_BAUD / 4] = 115200;
    uart[REG_CTRL / 4] = 0x3; /* TXEN | RXEN */
}

static void uart_putchar(char c)
{
    volatile uint32_t *uart = (volatile uint32_t *)UART_BASE;
    while (uart[REG_STATE / 4] & STATE_TXFULL) { }
    uart[REG_DATA / 4] = (uint32_t)(uint8_t)c;
}

/* 打印一行（临界区保护，原子输出；\n 展开为 \r\n 兼容 QEMU 终端） */
static void print_line(const char *s)
{
    taskENTER_CRITICAL();
    while (*s) {
        if (*s == '\n') {
            uart_putchar('\r');
        }
        uart_putchar(*s);
        s++;
    }
    taskEXIT_CRITICAL();
}

/* 十进制小整数拼接后打印 */
static void print_num(uint32_t v)
{
    char buf[12];
    int i = 11;
    buf[i--] = 0;
    do {
        buf[i--] = (char)('0' + (v % 10));
        v /= 10;
    } while (v);
    print_line(&buf[i + 1]);
}

/* ---------------- 时间片轮转核心机制（FRT-AC-02） ---------------- */

/* 轮次标志：1 = TS_A 的轮次，0 = TS_B 的轮次。
 * 初始 = TS_B：调度器启动后第一个运行的任务 = pxCurrentTCB = 最高优先级
 * 任务中最后创建者（FreeRTOS xTaskCreate 语义）→ 首行恒为 [TS] B 0，
 * 与首个 SysTick 相对启动时刻的位置无关（跨模拟器一致的关键）。 */
static volatile uint32_t g_turn_is_a = 0;

#define TS_ITERATIONS 40

/* 忙等自己的轮次：纯忙循环（不阻塞、不放弃 CPU），任务全程保持 Ready——
 * 从打印完翻转轮次到被下个 tick 抢占期间，唯一能推进轮次的是另一任务
 * （时间片旋转保证其必定运行）。 */
static void spin_until_turn(uint32_t mine_is_a)
{
    while (g_turn_is_a != mine_is_a) { }
}

/* 同优先级忙循环任务：忙等轮次 → 打印 → 翻转轮次（不阻塞）。
 * 每 slice 恰输出一行；两任务经时间片轮转严格交替（B0 A0 B1 A1 …）。 */
static void vTsTaskA(void *p)
{
    uint32_t seq;
    (void)p;
    for (seq = 0; seq < TS_ITERATIONS; seq++) {
        spin_until_turn(1);
        print_line("[TS] A ");
        print_num(seq);
        print_line("\r\n");
        g_turn_is_a = 0; /* 交还轮次给 TS_B（单字写，天然原子） */
    }
    /* 固定迭代次数跑完 → 永久阻塞（离开就绪列表），输出结束 */
    vTaskDelay(portMAX_DELAY);
}

static void vTsTaskB(void *p)
{
    uint32_t seq;
    (void)p;
    for (seq = 0; seq < TS_ITERATIONS; seq++) {
        spin_until_turn(0);
        print_line("[TS] B ");
        print_num(seq);
        print_line("\r\n");
        g_turn_is_a = 1; /* 交还轮次给 TS_A */
    }
    vTaskDelay(portMAX_DELAY);
}

/* ---------------- SVC 分发桩（startup_freertos.S 引用；本变体无自定义 SVC 用例，
 * 仅调度器启动 svc 0 走 vPortSVCHandler 路径，此函数不会被调用） ---------------- */
void vApplicationSvcHandler(uint32_t ulSvCallNumber)
{
    (void)ulSvCallNumber;
}

/* ---------------- 静态分配（idle 任务） ---------------- */
static StackType_t ucIdleStack[configMINIMAL_STACK_SIZE];
static StaticTask_t xIdleTaskTCB;
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = ucIdleStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* ---------------- 失败钩子（可观测，不静默） ---------------- */
void vApplicationMallocFailedHook(void)
{
    print_line("[FAIL] malloc failed\r\n");
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask;
    print_line("[FAIL] stack overflow: ");
    print_line(pcTaskName);
    print_line("\r\n");
    for (;;) { }
}

/* ---------------- main ---------------- */
int main(void)
{
    BaseType_t ok = pdTRUE;

    uart_init();
    print_line("\r\n[PASS] freertos timeslice variant start\r\n");

    /* 创建顺序固定：TS_A 先建、TS_B 后建 → pxCurrentTCB 在调度器启动时指向
     * TS_B（最高优先级中最后创建者）→ 首行恒为 [TS] B 0（跨模拟器一致）。 */
    ok &= (xTaskCreate(vTsTaskA, "TS_A", 128, NULL, 2, NULL) == pdPASS);
    ok &= (xTaskCreate(vTsTaskB, "TS_B", 128, NULL, 2, NULL) == pdPASS);

    if (ok != pdTRUE) {
        print_line("[FAIL] task create failed\r\n");
        for (;;) { }
    }

    vTaskStartScheduler();

    /* 调度器永不返回 */
    print_line("[FAIL] scheduler returned\r\n");
    for (;;) { }
}
