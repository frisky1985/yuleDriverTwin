/*
 * main_freertos.c — dtwin FreeRTOS 镜像演示固件（FRT-FW-01~04）
 *
 * 任务集（FRT-FW-02）：
 *   [SVC]   pri4：svc #0x2A（自定义 SVC 处理器打印 [SVC] 42），vTaskDelay(4)
 *   [CRIT]  pri3：taskENTER_CRITICAL 保护共享计数 +1000，打印 [CRIT] n=…，delay(10)
 *   [TASK]  pri2：HIGH（delay 2）/ pri1：MID（delay 3）/ pri0：LOW（delay 5）
 *   [TS]    pri2：TS_A/TS_B 同优先级每 tick 轮换打印（vTaskDelay(1)）
 *
 * 确定性设计（QEMU mps2-an386 实测：SysTick 由宿主时间驱动，每 tick 指令数
 * 不可复现 48~60 万且逐次不同；dtwin 为 1 指令=1 周期 → 每 tick 恰好 25000）：
 *   - 所有任务「每次唤醒打印一行 + vTaskDelay」，唤醒 tick 由延迟计数决定
 *     （两侧一致）→ 输出序列 tick 计数驱动，跨模拟器可复现。
 *   - 打印受 taskENTER_CRITICAL/taskEXIT_CRITICAL 保护（UART 共享资源，
 *     避免 tick 抢占导致行交错——BASEPRI=5 屏蔽 SysTick(255)），顺带验证临界区。
 *
 * 输出行前缀：[TASK]/[TS]/[SVC]/[CRIT]/[PASS]（FRT-FW-02 统一前缀）。
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

/* ---------------- 任务 ---------------- */
static void vHighTask(void *p)
{
    uint32_t seq = 0;
    (void)p;
    for (;;) {
        print_line("[TASK] HIGH ");
        print_num(seq++);
        print_line("\r\n");
        vTaskDelay(2);
    }
}

static void vMidTask(void *p)
{
    uint32_t seq = 0;
    (void)p;
    for (;;) {
        print_line("[TASK] MID ");
        print_num(seq++);
        print_line("\r\n");
        vTaskDelay(3);
    }
}

static void vLowTask(void *p)
{
    uint32_t seq = 0;
    (void)p;
    for (;;) {
        print_line("[TASK] LOW ");
        print_num(seq++);
        print_line("\r\n");
        vTaskDelay(5);
    }
}

/* 时间片对：同优先级（pri2）每 tick 轮换（FRT-AC-02）。
 * 说明：迭代 = 打印 + vTaskDelay(1)——「每次唤醒打印一行」在 QEMU（宿主时间
 * 时钟）与 dtwin（1 指令=1 周期）之间可复现；A/B 就绪顺序由 ready 列表轮转
 * 决定，输出交替出现（A,B,A,B…）。 */
static void vTsTaskA(void *p)
{
    uint32_t seq = 0;
    (void)p;
    for (;;) {
        print_line("[TS] A ");
        print_num(seq++);
        print_line("\r\n");
        vTaskDelay(1);
    }
}

static void vTsTaskB(void *p)
{
    uint32_t seq = 0;
    (void)p;
    for (;;) {
        print_line("[TS] B ");
        print_num(seq++);
        print_line("\r\n");
        vTaskDelay(1);
    }
}

/* 自定义 SVC 用例（FRT-FW-03）：任务执行 svc #0x2A，处理器打印 [SVC] 42 后返回 */
static void vSvcTask(void *p)
{
    (void)p;
    for (;;) {
        __asm volatile ("svc #0x2A" ::: "memory");
        vTaskDelay(4);
    }
}

/* 临界区用例（FRT-FW-04）：BASEPRI 屏蔽下共享计数 +1000 无丢失/无重入 */
static volatile uint32_t ulSharedCounter;
static void vCritTask(void *p)
{
    uint32_t i;
    (void)p;
    for (;;) {
        taskENTER_CRITICAL();
        for (i = 0; i < 1000; i++) {
            ulSharedCounter++;
        }
        taskEXIT_CRITICAL();
        print_line("[CRIT] n=");
        print_num(ulSharedCounter);
        print_line("\r\n");
        vTaskDelay(10);
    }
}

/* ---------------- 自定义 SVC 处理器（startup_freertos.S 调用） ---------------- */
void vApplicationSvcHandler(uint32_t ulSvCallNumber)
{
    print_line("[SVC] ");
    print_num(ulSvCallNumber);
    print_line("\r\n");
}

/* ---------------- 静态分配（idle 任务；timers 关闭无需 timer 任务） ---------------- */
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
    print_line("\r\n[PASS] freertos demo start\r\n");

    /* 创建顺序固定（ready 列表轮转顺序依赖创建序，保证跨模拟器一致） */
    ok &= (xTaskCreate(vSvcTask, "SVC", 256, NULL, 4, NULL) == pdPASS);
    ok &= (xTaskCreate(vCritTask, "CRIT", 256, NULL, 3, NULL) == pdPASS);
    ok &= (xTaskCreate(vHighTask, "HIGH", 128, NULL, 2, NULL) == pdPASS);
    ok &= (xTaskCreate(vMidTask, "MID", 128, NULL, 1, NULL) == pdPASS);
    ok &= (xTaskCreate(vLowTask, "LOW", 128, NULL, 0, NULL) == pdPASS);
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
