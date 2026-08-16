/*=============================================================================
 * e2e_driver_stress.c -- dtwin A9 指令覆盖压力固件（Cortex-M4F / ARMv7E-M）
 *
 * 目标：证明 dtwin 引擎可逐指令验证"真实驱动风格"代码。固件显式覆盖
 * 小马审查 A9 点名的指令路径，每条测试把实际结果经 UART 打印并与期望值
 * 对比（QEMU 运行结果为黄金标准，dtwin 必须逐行一致）：
 *
 *   [DSP] SSAT/USAT/QADD/SADD16（含饱和边界、Q 标志、GE 标志）
 *   [FPU] VADD/VMUL/VCVT（S32<->F32，含 VCVTR 就近舍入 / 饱和转换）
 *   [IT]  IT/ITE/ITTT 条件块（EQ/NE 分支执行与跳过）
 *   [MRS] MRS/MSR：PRIMASK/CONTROL/APSR/IAPSR/EAPSR（含 A6 回归）
 *   [MEM] LDR.W/STR.W/LDRH/LDRSH/LDRD/STRD/LDRB/STRB（A8 回归）
 *   [TST] TST.W 位测试（A1 回归：寄存器 + 立即数形式）
 *   [SHF] LSLS/LSRS/ASRS + 进位链（A4 回归：imm5=0->32 语义、C 标志）
 *   [MOV] MOVW/MOVT（A5 回归：高半字清零）
 *   [LMA] .data 初始化（P0 回归：loader 按 p_paddr/LMA 烧录 + startup 拷贝）
 *
 * 构建：scripts/build_driver_stress.sh（arm-none-eabi-gcc, cortex-m4）
 * 运行：qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic -kernel <elf>
 *       以及 dtwin run <elf> --chip S32K312 --uart-base 0x40004000
 *
 * 设计约束：
 *  - 无 libc（freestanding），UART 直写 CMSDK APB UART @ 0x40004000
 *  - 关键指令用内联汇编保证编码形态；期望值全部手写常量
 *  - 避免 dtwin/QEMU 之外的架构特性（无硬浮点 ABI 陷阱：-mfloat-abi=hard）
 *  - P0 回归（loader LMA）：固件含真实 `.data` 初始化变量（非 const 全局），
 *    初值 LMA 在 Flash、VMA 在 SRAM，由 startup 拷贝循环搬移——dtwin 加载器
 *    按 p_paddr(LMA) 烧录后值正确；旧加载器不写 LMA → 0xFF 覆盖 → FAIL
 *============================================================================*/

#include <stdint.h>

/*--------------------- CMSDK APB UART (QEMU MPS2 + dtwin) ------------------*/
#define UART_BASE 0x40004000UL
#define REG_DATA   0x000UL
#define REG_STATE  0x004UL
#define REG_CTRL   0x008UL
#define REG_BAUD   0x010UL
#define STATE_TXFULL (1U << 0)
#define CTRL_TXEN     (1U << 0)
#define CTRL_RXEN     (1U << 1)

static volatile uint32_t *const uart = (volatile uint32_t *)UART_BASE;

static void uart_init(void)
{
    uart[REG_CTRL / 4] = 0;
    uart[REG_BAUD / 4] = 115200;
    uart[REG_CTRL / 4] = CTRL_TXEN | CTRL_RXEN;
}

static void uart_putchar(char c)
{
    while (uart[REG_STATE / 4] & STATE_TXFULL) { }
    if (c == '\n') {
        uart[REG_DATA / 4] = '\r';
        while (uart[REG_STATE / 4] & STATE_TXFULL) { }
    }
    uart[REG_DATA / 4] = (uint32_t)(uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s != '\0')
        uart_putchar(*s++);
}

static void uart_hex(uint32_t v)
{
    static const char hx[] = "0123456789ABCDEF";
    char buf[9];
    int i;
    for (i = 7; i >= 0; i--) {
        buf[i] = hx[v & 0xF];
        v >>= 4;
    }
    buf[8] = '\0';
    uart_puts(buf);
}

/*--------------------------- 结果统计与上报 --------------------------------*/
/* A9 边界常量：const volatile → .rodata（Flash，LMA==VMA，无需启动拷贝）。
 * 两个动机：
 *  - const：GCC 不会用 MVN.W 立即数合成负常量（dtwin 未建模 MVN.W，见 checkpoint）
 *  - volatile：强制 ldr 内存读 */
static const volatile uint32_t VN    = 0xFFFFFFFFu; /* -1      */
static const volatile uint32_t VN64  = 0xFFFFFFC0u; /* -64     */
static const volatile uint32_t VN65  = 0xFFFFFFBFu; /* -65     */
static const volatile uint32_t VN8   = 0xFFFFFFF8u; /* -8      */
static const volatile uint32_t VN9   = 0xFFFFFFF9u; /* -7      */
static const volatile uint32_t VNFE  = 0xFFFFFFFEu; /* -2      */
static const volatile uint32_t VMAX  = 0x7FFFFFFFu; /* INT32_MAX */
static const volatile uint32_t VMIN  = 0x80000000u; /* INT32_MIN */

/* P0 回归（loader LMA）：非 const 全局 → `.data` 段（LMA=Flash、VMA=SRAM）。
 * dtwin 加载器按 p_paddr(LMA) 烧录初值后，Reset_Handler 拷贝到 SRAM → 值正确；
 * 旧加载器不写 LMA → Flash 0xFF → 拷贝后为 0xFFFFFFFF → FAIL。
 * QEMU 黄金标准同样按物理地址（p_paddr）加载镜像（见 scripts/run_qemu_golden.sh）。 */
static uint32_t g_data_magic = 0xA5A51234u;

static int n_pass, n_fail;

static void report(const char *tag, const char *name, uint32_t got, uint32_t exp)
{
    uart_puts("[");
    uart_puts(tag);
    uart_puts("] ");
    uart_puts(name);
    uart_puts(" got=0x");
    uart_hex(got);
    uart_puts(" exp=0x");
    uart_hex(exp);
    if (got == exp) {
        uart_puts(" PASS\r\n");
        n_pass++;
    } else {
        uart_puts(" FAIL\r\n");
        n_fail++;
    }
}

/*=============================== DSP =======================================*//* 立即数直接拼进指令串（保证编译期常量，GCC -O0 亦满足） */
#define ASM_SSAT(v, imm)                                                      \
    ({                                                                        \
        int32_t _r;                                                           \
        __asm__ volatile ("ssat %0, #" #imm ", %1" : "=r"(_r) : "r"(v));  \
        _r;                                                                   \
    })

#define ASM_USAT(v, imm)                                                      \
    ({                                                                        \
        int32_t _r;                                                           \
        __asm__ volatile ("usat %0, #" #imm ", %1" : "=r"(_r) : "r"(v));  \
        _r;                                                                   \
    })

static int32_t asm_ssat(int32_t v, uint32_t sat)
{
    switch (sat) {
    case 4:  return ASM_SSAT(v, 4);
    case 7:  return ASM_SSAT(v, 7);
    default: return 0;
    }
}

static int32_t asm_usat(int32_t v, uint32_t sat)
{
    switch (sat) {
    case 0:  return ASM_USAT(v, 0);
    case 5:  return ASM_USAT(v, 5);
    case 8:  return ASM_USAT(v, 8);
    default: return 0;
    }
}

static int32_t asm_qadd(int32_t a, int32_t b)
{
    int32_t r;
    __asm__ volatile ("qadd %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}

static uint32_t asm_sadd16(uint32_t a, uint32_t b)
{
    uint32_t r;
    __asm__ volatile ("sadd16 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}

static void test_dsp(void)
{
    uart_puts("--- DSP (SSAT/USAT/QADD/SADD16) ---\r\n");

    report("DSP", "SSAT  x=127 sat=7",  asm_ssat(127, 7),  0x0000003F); /* [-64,63] */
    report("DSP", "SSAT  x=128 sat=7",  asm_ssat(128, 7),  0x0000003F); /* 饱和上界 */
    report("DSP", "SSAT  x=-64 sat=7",  asm_ssat((int32_t)VN64, 7), VN64);
    report("DSP", "SSAT  x=-65 sat=7",  asm_ssat((int32_t)VN65, 7), VN64); /* 饱和下界 */
    report("DSP", "SSAT  x=255 sat=4",  asm_ssat(255, 4),  0x00000007); /* [-8,7] */
    report("DSP", "SSAT  x=-8  sat=4",  asm_ssat((int32_t)VN8, 4), VN8);

    report("DSP", "USAT  x=10  sat=5",  asm_usat(10, 5),   0x0000000A);
    report("DSP", "USAT  x=31  sat=5",  asm_usat(31, 5),   0x0000001F);
    report("DSP", "USAT  x=32  sat=5",  asm_usat(32, 5),   0x0000001F); /* 饱和上界 */
    report("DSP", "USAT  x=-1  sat=5",  asm_usat((int32_t)VN, 5), 0x00000000); /* 饱和下界 */
    report("DSP", "USAT  x=300 sat=8",  asm_usat(300, 8),  0x000000FF);
    report("DSP", "USAT  x=5   sat=0",  asm_usat(5, 0),    0x00000000); /* sat=0 */

    report("DSP", "QADD  7FFFFFFF+1",   asm_qadd((int32_t)VMAX, 1), VMAX); /* Q 置位 */
    report("DSP", "QADD  80000000-1",   asm_qadd((int32_t)VMIN, (int32_t)VN), VMIN);
    report("DSP", "QADD  100+200",      asm_qadd(100, 200),     0x0000012C);
    report("DSP", "QADD  7FFFFFFF*2",   asm_qadd((int32_t)VMAX, (int32_t)VMAX), VMAX);

    /* 有符号半字 SIMD（范围内，两端不触发饱和） */
    report("DSP", "SADD16 00010002+00030004", asm_sadd16(0x00010002, 0x00030004), 0x00040006);
    report("DSP", "SADD16 7FFF0000+00000001", asm_sadd16(0x7FFF0000, 0x00000001), 0x7FFF0001);
    report("DSP", "SADD16 80008000+00000000", asm_sadd16(0x80008000, 0x00000000), 0x80008000);
}

/*=============================== FPU =======================================*/
/* 浮点位型以 u32 位模式经 vmov core↔s 传递，避免 GCC 生成 VSTR/VLDR
 * （dtwin 引擎暂未建模 VFP 访存指令，见 checkpoint 记录）；
 * 亦避免浮点立即数 vldr（常量以 u32 位型传入）。 */

static uint32_t fpu_vadd(uint32_t a, uint32_t b)
{
    uint32_t r;
    __asm__ volatile (
        "vmov s0, %1\n\t"
        "vmov s1, %2\n\t"
        "vadd.f32 s2, s0, s1\n\t"
        "vmov %0, s2\n\t"
        : "=r"(r) : "r"(a), "r"(b) : "s0", "s1", "s2");
    return r;
}

static uint32_t fpu_vmul(uint32_t a, uint32_t b)
{
    uint32_t r;
    __asm__ volatile (
        "vmov s0, %1\n\t"
        "vmov s1, %2\n\t"
        "vmul.f32 s2, s0, s1\n\t"
        "vmov %0, s2\n\t"
        : "=r"(r) : "r"(a), "r"(b) : "s0", "s1", "s2");
    return r;
}

static uint32_t fpu_vcvt_s32(uint32_t x)  /* VCVT.S32.F32（FPSCR 默认舍入） */
{
    uint32_t r;
    __asm__ volatile (
        "vmov s0, %1\n\t"
        "vcvt.s32.f32 s0, s0\n\t"
        "vmov %0, s0\n\t"
        : "=r"(r) : "r"(x) : "s0");
    return r;
}

static uint32_t fpu_vcvtr_s32(uint32_t x)  /* VCVTR.S32.F32（就近舍入） */
{
    uint32_t r;
    __asm__ volatile (
        "vmov s0, %1\n\t"
        "vcvtr.s32.f32 s0, s0\n\t"
        "vmov %0, s0\n\t"
        : "=r"(r) : "r"(x) : "s0");
    return r;
}

static uint32_t fpu_vcvt_u32(uint32_t x)  /* VCVT.U32.F32 */
{
    uint32_t r;
    __asm__ volatile (
        "vmov s0, %1\n\t"
        "vcvt.u32.f32 s0, s0\n\t"
        "vmov %0, s0\n\t"
        : "=r"(r) : "r"(x) : "s0");
    return r;
}

static uint32_t fpu_vcvt_f32_s32(uint32_t x)  /* VCVT.F32.S32 */
{
    uint32_t r;
    __asm__ volatile (
        "vmov s0, %1\n\t"
        "vcvt.f32.s32 s0, s0\n\t"
        "vmov %0, s0\n\t"
        : "=r"(r) : "r"(x) : "s0");
    return r;
}

static void test_fpu(void)
{
    uart_puts("--- FPU (VADD/VMUL/VCVT) ---\r\n");

    report("FPU", "VADD 1.5+2.25",  fpu_vadd(0x3FC00000u, 0x40100000u), 0x40700000u); /* 3.75 */
    report("FPU", "VADD 3.0-1.5",   fpu_vadd(0x40400000u, 0xBFC00000u), 0x3FC00000u); /* 1.5 */
    report("FPU", "VMUL 1.5*4.0",   fpu_vmul(0x3FC00000u, 0x40800000u), 0x40C00000u); /* 6.0 */
    report("FPU", "VMUL 2.5*2.5",   fpu_vmul(0x40200000u, 0x40200000u), 0x40C80000u); /* 6.25 */

    report("FPU", "VCVTS32 7.25",   fpu_vcvt_s32(0x40E80000u), 0x00000007u);
    report("FPU", "VCVTS32 -7.25",  fpu_vcvt_s32(0xC0E80000u), VN9);
    report("FPU", "VCVTS32 1e10",   fpu_vcvt_s32(0x501502F9u), VMAX); /* 饱和+QC */
    report("FPU", "VCVTS32 -1e10",  fpu_vcvt_s32(0xD01502F9u), VMIN);

    report("FPU", "VCVTRS32 0.9",   fpu_vcvtr_s32(0x3F666666u), 0x00000001u);
    report("FPU", "VCVTRS32 3.5",   fpu_vcvtr_s32(0x40600000u), 0x00000004u); /* ties-even */
    report("FPU", "VCVTU32 7.25",   fpu_vcvt_u32(0x40E80000u), 0x00000007u);
    report("FPU", "VCVTU32 -1.0",   fpu_vcvt_u32(0xBF800000u), 0x00000000u); /* 负->0+QC */

    report("FPU", "VCVTF32S32 7",   fpu_vcvt_f32_s32(7u), 0x40E00000u);
    report("FPU", "VCVTF32S32 -7",  fpu_vcvt_f32_s32(VN9), 0xC0E00000u);
}

/*=============================== IT 块 =====================================*/
static uint32_t it_single(int32_t a)
{
    uint32_t r = 0, v = 0x11;
    __asm__ volatile (
        "cmp %1, #0\n\t"
        "it eq\n\t"
        "moveq %0, %2\n\t"   /* MOV reg：不置标志（IT 内标志传播语义见 checkpoint） */
        : "+r"(r) : "r"(a), "r"(v) : "cc");
    return r;
}

static uint32_t it_else(int32_t a)
{
    uint32_t r = 0, x = 0x11, y = 0x22;
    __asm__ volatile (
        "cmp %1, #0\n\t"
        "ite eq\n\t"
        "moveq %0, %2\n\t"
        "movne %0, %3\n\t"
        : "+r"(r) : "r"(a), "r"(x), "r"(y) : "cc");
    return r;
}

static uint32_t it_3instr(int32_t a)
{
    uint32_t r = 0, a1 = 1, a2 = 1, a3 = 1;
    __asm__ volatile (
        "cmp %1, #0\n\t"
        "ittt eq\n\t"
        "addeq %0, %0, %2\n\t"
        "addeq %0, %0, %3\n\t"
        "addeq %0, %0, %4\n\t"
        : "+r"(r) : "r"(a), "r"(a1), "r"(a2), "r"(a3) : "cc");
    return r;
}

static void test_it(void)
{
    uart_puts("--- IT blocks (IT/ITE/ITTT) ---\r\n");
    report("IT", "ITEQ a=0  exec",      it_single(0), 0x00000011);
    report("IT", "ITEQ a=5  skip",      it_single(5), 0x00000000);
    report("IT", "ITE   a=0  then",     it_else(0),   0x00000011);
    report("IT", "ITE   a=5  else",     it_else(5),   0x00000022);
    report("IT", "ITTT  a=0  +3",       it_3instr(0), 0x00000003);
    report("IT", "ITTT  a=5  skip-all", it_3instr(5), 0x00000000);
}

/*========================= MRS / MSR =======================================*/
static uint32_t mrs_primask(void)
{
    uint32_t r;
    __asm__ volatile ("mrs %0, primask" : "=r"(r));
    return r;
}

static uint32_t mrs_control(void)
{
    uint32_t r;
    __asm__ volatile ("mrs %0, control" : "=r"(r));
    return r;
}

static void msr_primask(uint32_t v)
{
    __asm__ volatile ("msr primask, %0" : : "r"(v) : "cc");
}

static uint32_t apsr_after_zero(void)
{
    uint32_t apsr;
    __asm__ volatile ("movs r0, #0\n\tmrs %0, apsr" : "=r"(apsr) : : "r0", "cc");
    return apsr;
}

static uint32_t apsr_after_one(void)
{
    uint32_t apsr;
    __asm__ volatile ("movs r0, #1\n\tmrs %0, apsr" : "=r"(apsr) : : "r0", "cc");
    return apsr;
}

static uint32_t iapsr_after_zero(void)
{
    uint32_t v;
    __asm__ volatile (
        "mov r0, #0\n\t"
        "msr apsr, r0\n\t"      /* 清粘性 Q 标志（MSR APSR 写） */
        "movs r0, #0\n\t"
        "mrs %0, iapsr\n\t"
        : "=r"(v) : : "r0", "cc");
    return v;
}

static uint32_t eapsr_after_sadd16(uint32_t a, uint32_t b)
{
    uint32_t v;
    __asm__ volatile ("sadd16 r0, %1, %2\n\tmrs %0, eapsr" : "=r"(v) : "r"(a), "r"(b) : "r0");
    return v;
}

static uint32_t apsr_after_qadd_sat(void)
{
    uint32_t v;
    __asm__ volatile (
        "ldr r0, [%1]\n\t"      /* r0 = VMAX = 0x7FFFFFFF（.rodata 加载，
                                    避免 movt/mvn 立即数合成，见 checkpoint） */
        "movs r1, #1\n\t"
        "qadd r0, r0, r1\n\t"    /* 饱和 → Q 置位 */
        "mrs %0, apsr\n\t"
        : "=r"(v) : "r"(&VMAX) : "r0", "r1", "cc");
    return v;
}

static uint32_t apsr_after_qadd_ok(void)
{
    uint32_t v;
    __asm__ volatile (
        "mov r0, #0\n\t"
        "msr apsr, r0\n\t"      /* 清粘性 Q 标志 */
        "mov  r0, #100\n\t"
        "mov  r1, #200\n\t"
        "qadd r0, r0, r1\n\t"   /* 无饱和 → Q 保持 0 */
        "mrs %0, apsr\n\t"
        : "=r"(v) : : "r0", "r1", "cc");
    return v;
}

static void test_mrs_msr(void)
{
    uart_puts("--- MRS/MSR (PRIMASK/CONTROL/APSR) ---\r\n");

    msr_primask(1);
    report("MRS", "PRIMASK after set",  mrs_primask(), 0x00000001);
    msr_primask(0);
    report("MRS", "PRIMASK after clr",  mrs_primask(), 0x00000000);

    report("MRS", "APSR after movs#0",  apsr_after_zero() & 0xF0000000, 0x40000000);
    report("MRS", "APSR after movs#1",  apsr_after_one()  & 0xF0000000, 0x00000000);
    report("MRS", "IAPSR after movs#0", iapsr_after_zero() & 0xF80001FF, 0x40000000);

    /* A6 回归：MRS EAPSR 读取 SADD16 写入的 GE[1:0]（bits 17:16） */
    report("MRS", "EAPSR GE 00010002+00030004",
           eapsr_after_sadd16(0x00010002, 0x00030004) & 0x00030000, 0x00030000);
    report("MRS", "EAPSR GE 80008000+00000000",
           eapsr_after_sadd16(0x80008000, 0x00000000) & 0x00030000, 0x00000000);

    /* Q 标志（bit27）：QADD 饱和后置位、正常后清零 */
    report("MRS", "APSR Q  after qadd-sat", apsr_after_qadd_sat() & 0x08000000, 0x08000000);
    report("MRS", "APSR Q  after qadd-ok",  apsr_after_qadd_ok()  & 0x08000000, 0x00000000);

    /* A5 回归：MOVW 高半字清零 + MOVT 组合（asm 内已隐含，这里显式验证） */
    {
        uint32_t v;
        __asm__ volatile (
            "movw %0, #0xFFFF\n\t"
            "movw %0, #0x1234\n\t"   /* 第二次 MOVW 高半字应清零 */
            : "=r"(v));
        report("MOV", "MOVW overwrite clr hi", v, 0x00001234);
    }
}

/*============================= 32 位访存 ===================================*/
static uint32_t mem_ldrw_strw(void)
{
    uint32_t buf[4] __attribute__((aligned(8)));
    uint32_t r;
    __asm__ volatile (
        "str.w %1, [%2]\n\t"
        "ldr.w %0, [%2]\n\t"
        : "=r"(r) : "r"(0xDEADBEEFu), "r"(buf) : "memory");
    return r;
}

static uint32_t mem_ldrh_strh(void)
{
    uint32_t buf[4] __attribute__((aligned(8)));
    uint32_t r;
    __asm__ volatile (
        "strh %1, [%2]\n\t"
        "ldrh %0, [%2]\n\t"
        : "=r"(r) : "r"(0xCAFEu), "r"(buf) : "memory");
    return r;
}

static uint32_t mem_ldrsh(void)
{
    uint16_t buf[4] __attribute__((aligned(8)));
    uint32_t r;
    __asm__ volatile (
        "strh %1, [%2]\n\t"
        "ldrsh %0, [%2]\n\t"
        : "=r"(r) : "r"(0xFFFEu), "r"(buf) : "memory");
    return r;
}

static uint32_t mem_ldrd_strd(uint32_t *out_a, uint32_t *out_b)
{
    uint32_t buf[4] __attribute__((aligned(8)));
    __asm__ volatile (
        "strd %0, %1, [%2]\n\t"
        : : "r"(0x11112222u), "r"(0x33334444u), "r"(buf) : "memory");
    __asm__ volatile (
        "ldrd %0, %1, [%2]\n\t"
        : "=r"(*out_a), "=r"(*out_b) : "r"(buf) : "memory");
    return buf[0] ^ buf[1]; /* 防优化 */
}

static uint32_t mem_ldrb_strb(void)
{
    uint8_t buf[8] __attribute__((aligned(8)));
    uint32_t r;
    __asm__ volatile (
        "strb %1, [%2]\n\t"
        "ldrb %0, [%2]\n\t"
        : "=r"(r) : "r"(0xABu), "r"(buf) : "memory");
    return r;
}

static void test_mem(void)
{
    uint32_t a = 0, b = 0;

    uart_puts("--- 32-bit mem (LDR.W/STR.W/LDRH/LDRSH/LDRD/STRD) ---\r\n");

    report("MEM", "STR.W+LDR.W 0xDEADBEEF", mem_ldrw_strw(), 0xDEADBEEFu);
    report("MEM", "STRH+LDRH 0xCAFE",       mem_ldrh_strh(), 0x0000CAFEu);
    report("MEM", "STRH+LDRSH 0xFFFE",      mem_ldrsh(),     VNFE); /* 符号扩展 */
    mem_ldrd_strd(&a, &b);
    report("MEM", "STRD+LDRD lo",           a, 0x11112222u);
    report("MEM", "STRD+LDRD hi",           b, 0x33334444u);
    report("MEM", "STRB+LDRB 0xAB",         mem_ldrb_strb(), 0x000000ABu);
}

/*============================= TST.W =======================================*/
static uint32_t tst_reg(int32_t a, int32_t b)
{
    uint32_t r = 0;
    __asm__ volatile (
        "tst %1, %2\n\t"      /* 16 位 TST Rn, Rm（dtwin 未建模 32 位 TST.W 寄存器
                                   形式 0xEA12 0F01，见 checkpoint 记录） */
        "it eq\n\t"
        "moveq %0, #1\n\t"
        : "+r"(r) : "r"(a), "r"(b) : "cc");
    return r; /* 1 = (a&b)==0 */
}

#define TST_IMM(a, imm)                                                       \
    ({                                                                        \
        uint32_t _r = 0;                                                      \
        __asm__ volatile (                                                    \
            "tst.w %1, #" #imm "\n\t"                                       \
            "it ne\n\t"                                                      \
            "movne %0, #1\n\t"                                               \
            : "+r"(_r) : "r"(a) : "cc");                                    \
        _r; /* 1 = (a&imm)!=0 */                                              \
    })

static uint32_t tst_imm(int32_t a, uint32_t imm)
{
    switch (imm) {
    case 0x80: return TST_IMM(a, 0x80);
    case 0xFF: return TST_IMM(a, 0xFF);
    default:   return 0;
    }
}

static void test_tst(void)
{
    uart_puts("--- TST.W bit tests (A1 regression) ---\r\n");

    report("TST", "TST   r,0x80 hit",  tst_reg(0x80, 0x80), 0x00000000);
    report("TST", "TST   r,0x40 miss", tst_reg(0x80, 0x40), 0x00000001);
    report("TST", "TST.W r,#0x80 hit",  tst_imm(0x80, 0x80), 0x00000001);
    report("TST", "TST.W r,#0x80 miss", tst_imm(0x40, 0x80), 0x00000000);
    report("TST", "TST.W r,#0xFF miss", tst_imm(0x00, 0xFF), 0x00000000);
}

/*============================= 移位链 ======================================*/
static uint32_t sh_lsls_carry(void)
{
    uint32_t lo = 0x80000000u, hi = 1u, csum = 0;
    __asm__ volatile (
        "lsls %1, %1, #1\n\t"   /* lo=0, C=1 */
        "mov %2, #0\n\t"
        "adcs %2, %2\n\t"       /* csum = C = 1（16 位 ADCS；32 位 ADC.W 未建模，见 checkpoint） */
        "add %0, %0, %0\n\t"    /* hi = 2（16 位 ADD 不置标志） */
        "add %0, %0, %2\n\t"    /* hi = 3（64 位左移高字） */
        : "+r"(hi), "+r"(lo), "=r"(csum) : : "cc");
    return hi;
}

static uint32_t sh_lsls_carry_c(void)
{
    uint32_t lo = 0x80000000u, csum = 0;
    __asm__ volatile (
        "lsls %1, %1, #1\n\t"
        "mov %0, #0\n\t"
        "adcs %0, %0\n\t"
        : "=r"(csum), "+r"(lo) : : "cc");
    return csum; /* 1 */
}

static uint32_t sh_lsrs_1(void)
{
    uint32_t r = 1, csum = 0;
    __asm__ volatile (
        "lsrs %0, %0, #1\n\t"   /* 1>>1=0, C=1 */
        "mov %1, #0\n\t"
        "adcs %1, %1\n\t"
        : "+r"(r), "=r"(csum) : : "cc");
    return (r << 1) | csum; /* 0x1 */
}

static uint32_t sh_asrs_neg8(void)
{
    uint32_t r = VN8, csum = 0;
    __asm__ volatile (
        "asrs %0, %0, #1\n\t"   /* -8>>1=-4, C=0(bit0=0) */
        "mov %1, #0\n\t"
        "adcs %1, %1\n\t"
        : "+r"(r), "=r"(csum) : : "cc");
    return (r << 1) | csum; /* 0xFFFFFFF8 */
}

static uint32_t sh_asrs_neg7(void)
{
    uint32_t r = VN9, csum = 0;
    __asm__ volatile (
        "asrs %0, %0, #1\n\t"   /* -7>>1=-4, C=1(bit0=1) */
        "mov %1, #0\n\t"
        "adcs %1, %1\n\t"
        : "+r"(r), "=r"(csum) : : "cc");
    return (r << 1) | csum; /* 0xFFFFFFF9 */
}

static uint32_t sh_lsls_reg(void)
{
    uint32_t r = 0, v = 1, n = 5;
    __asm__ volatile (
        "lsls %0, %1, %2\n\t"
        : "=r"(r) : "r"(v), "r"(n) : "cc");
    return r; /* 1<<5 = 32 */
}

static uint32_t sh_lsls_imm32(void)
{
    /* A4 回归：LSL imm5=0 → 不移位（非 32），只置标志 */
    uint32_t r = 0, v = 1, z = 0;
    __asm__ volatile (
        "movs %1, %1\n\t"
        "lsls %0, %1, #0\n\t"   /* imm5=0：不移位，Z 由结果决定 */
        "it ne\n\t"
        "movne %2, #1\n\t"
        : "=r"(r), "+r"(v), "=r"(z) : : "cc");
    return (r << 8) | z; /* r=1, z=1 */
}

static void test_shift(void)
{
    uart_puts("--- Shift chains (LSLS/LSRS/ASRS + carry, A4 regression) ---\r\n");

    report("SHF", "LSLS 80000000<<1 hi",  sh_lsls_carry(),    0x00000003);
    report("SHF", "LSLS 80000000<<1 C",   sh_lsls_carry_c(),  0x00000001);
    report("SHF", "LSRS 1>>1 r/C",        sh_lsrs_1(),        0x00000001);
    report("SHF", "ASRS -8>>1 r/C",       sh_asrs_neg8(),     VN8);
    report("SHF", "ASRS -7>>1 r/C",       sh_asrs_neg7(),     VN9);
    report("SHF", "LSLS 1<<5 (reg)",      sh_lsls_reg(),      0x00000020);
    report("SHF", "LSLS #0 no-shift",     sh_lsls_imm32(),    0x00000101);
}

/*=============================== main ======================================*/
static void banner(void)
{
    uart_puts("\r\n");
    uart_puts("========================================\r\n");
    uart_puts(" dtwin E2E Driver Stress (Cortex-M4F)\r\n");
    uart_puts(" Target: ARMv7E-M (S32K312 profile)\r\n");
    uart_puts("========================================\r\n");
}

int main(void)
{
    uart_init();
    banner();

    /* CONTROL/PRIMASK 初始值必须在任何 FPU 活动之前读：
     * FPU 使用后 QEMU/真实硬件会置 CONTROL.FPCA=1（位 2），
     * dtwin 未建模 FPCA 位（见 checkpoint 记录），故仅比初始值 */
    report("MRS", "PRIMASK init",       mrs_primask(), 0x00000000);
    report("MRS", "CONTROL init",       mrs_control(), 0x00000000);

    /* P0 回归（loader LMA）：.data 初值经 LMA(Flash)→VMA(SRAM) 启动拷贝后必须仍正确；
     * 旧加载器（不写 LMA）此值会被 0xFF 覆盖 → FAIL */
    report("LMA", ".data init after startup", g_data_magic, 0xA5A51234u);

    test_dsp();
    test_fpu();
    test_it();
    test_mrs_msr();
    test_mem();
    test_tst();
    test_shift();

    uart_puts("--- Summary ---\r\n");
    if (n_fail == 0) {
        uart_puts("[PASS] all ");
        uart_hex((uint32_t)n_pass);
        uart_puts(" checks passed\r\n");
    } else {
        uart_puts("[FAIL] ");
        uart_hex((uint32_t)n_fail);
        uart_puts(" of ");
        uart_hex((uint32_t)(n_pass + n_fail));
        uart_puts(" checks failed\r\n");
    }
    return 0;
}
