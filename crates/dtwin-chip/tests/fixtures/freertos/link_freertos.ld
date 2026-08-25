/*=============================================================================
 * link_freertos.ld — FreeRTOS 镜像链接脚本（与 mps2-an386 及 S32K312 profile 同布局）
 *
 *   FLASH 0x0000_0000（代码 + 向量表 + .rodata）
 *   SRAM  0x2000_0000（.data + .bss + 栈）
 *
 * 布局对齐 QEMU mps2-an386（M4）与 dtwin S32K312 profile 内存映射
 * （0x0 代码 / 0x20000000 SRAM / 0x40004000 UART）。
 *============================================================================*/

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00080000  /* 512KB */
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000  /* 256KB */
}

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
        *(.rodata.*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        _sdata = .;
        *(.data*)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT> FLASH

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM

    /* 栈顶（向量表 [0] 初始 MSP；FreeRTOS 启动后任务用 PSP 于 SRAM 堆内） */
    _estack = ORIGIN(SRAM) + LENGTH(SRAM);
}
