#pragma once

// dtcmram为dma不可达区域，因此需要额外定义dma缓冲区
#if defined(__GNUC__)
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#define RAM_D1 __attribute__((section(".ram_d1"), aligned(8)))
#else
#error "Unsupported compiler"
#endif