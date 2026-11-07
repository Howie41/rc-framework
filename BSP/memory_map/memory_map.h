#pragma once

// DTCMRAM 不可由 DMA1/DMA2 访问；D2 SRAM 用于 DMA 缓冲区。
#if defined(__GNUC__)

// D2 SRAM：DATA 从 Flash 复制初值，BSS 在启动时清零；DMA 缓冲区按 32 字节对齐。
#define DMA_BUFFER_DATA __attribute__((section(".dma_buffer_data"), aligned(32)))
#define DMA_BUFFER_BSS __attribute__((section(".dma_buffer_bss"), aligned(32)))

// 用于头文件中的 inline 模板变量。GCC 在 -fdata-sections 下生成独立的
// .noinit.* 输入段，链接脚本将其并入启动时清零的 D2 SRAM 区域。
#if defined(__clang__)
#define DMA_BUFFER_BSS_UNIQUE DMA_BUFFER_BSS
#else
#define DMA_BUFFER_BSS_UNIQUE __attribute__((noinit, aligned(32)))
#endif

// D1 AXI SRAM：DATA 从 Flash 复制初值，BSS 在启动时清零。
#define RAM_D1_DATA __attribute__((section(".ram_d1_data"), aligned(8)))
#define RAM_D1_BSS __attribute__((section(".ram_d1_bss"), aligned(8)))

// 保留旧名称供现有的零初始化对象使用。
#define DMA_BUFFER DMA_BUFFER_BSS
#define RAM_D1 RAM_D1_BSS

#define ITCM_FUNCTION __attribute__((section(".itcm_text"), noinline))

#else
#error "Unsupported compiler"
#endif
