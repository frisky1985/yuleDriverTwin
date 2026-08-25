/*
 * main_freertos_fpu.c — FreeRTOS FPU 场景 B 专用变体固件（FRT-AC-07）
 *
 * 验证任务使用浮点（VADD/VMUL/VDIV/VCVT 工作负载）时 FPU 上下文切换正确：
 *
 *   - vFpuTask（pri2）：浮点累计跨 vTaskDelay 存活——float 局部变量在
 *     hard-float AAPCS 下分配至 callee-saved s16-s31（编译器跨调用保存），
 *     每次睡眠切换：引擎压 S0-S15+FPSCR 扩展帧（EXC_RETURN=ED 变体）+
 *     port 压 s16-s31；唤醒切换反向恢复——累计值错任一寄存器即与黄金发散。
 *   - vIntTask（pri1）：纯整数任务（场景 A，无浮点指令）→ PendSV 走非
 *     FPU 变体（FD），双向切换覆盖 FPU 帧保存与恢复两条路径。
 *   - 浮点累计：acc = (acc+1)*2/2 + 0.25，取 (uint32_t)acc 打印——
 *     精确二进制分数（n+0.25 步进），IEEE-754 两侧（QEMU / dtwin）逐位一致；
 *     舍入语义由 C 强制转换（朝零截断）定义，QEMU 黄金为参照。
 *
 * 输出行前缀：[PASS]/[FPU]/[INT]（FRT-FW-02 统一前缀）。
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

/* 浮点任务（场景 B）：VADD/VMUL/VDIV/VCVT 工作负载。float 局部（acc/k1/k2/k3）
 * 跨 vTaskDelay 存活 → hard-float AAPCS callee-saved s16-s31（objdump 验证），
 * 每次上下文切换都须完整保存/恢复 S0-S31 + FPSCR。 */
static void vFpuTask(void *p)
{
    float acc = 0.0f;   /* 累计值：跨切换存活（s16-s31 域） */
    float k1 = 1.0f;    /* VADD 步长 */
    float k2 = 2.0f;    /* VMUL/VDIV 因子（不可折叠：无 -ffast-math） */
    float k3 = 0.25f;   /* 分数步进 → (uint32_t) 朝零截断可观测 */
    uint32_t iv;
    (void)p;
    for (;;) {
        acc = acc + k1; /* VADD.F32 */
        acc = acc * k2; /* VMUL.F32 */
        acc = acc / k2; /* VDIV.F32 */
        acc = acc + k3; /* VADD.F32 */
        iv = (uint32_t)acc; /* VCVT.U32.F32（C 强制转换 = 朝零截断） */
        print_line("[FPU] n=");
        print_num(iv);
        print_line("\r\n");
        vTaskDelay(1);
    }
}

/* 纯整数任务（场景 A）：无浮点指令 → 切换走非 FPU 变体（FD），
 * 与 vFpuTask 双向切换覆盖扩展帧保存/恢复两方向。 */
static void vIntTask(void *p)
{
    uint32_t seq = 0;
    (void)p;
    for (;;) {
        print_line("[INT] n=");
        print_num(seq++);
        print_line("\r\n");
        vTaskDelay(2);
    }
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
    print_line("\r\n[PASS] freertos fpu variant start\r\n");

    ok &= (xTaskCreate(vFpuTask, "FPU", 256, NULL, 2, NULL) == pdPASS);
    ok &= (xTaskCreate(vIntTask, "INT", 128, NULL, 1, NULL) == pdPASS);

    if (ok != pdTRUE) {
        print_line("[FAIL] task create failed\r\n");
        for (;;) { }
    }

    vTaskStartScheduler();

    /* 调度器永不返回 */
    print_line("[FAIL] scheduler returned\r\n");
    for (;;) { }
}
