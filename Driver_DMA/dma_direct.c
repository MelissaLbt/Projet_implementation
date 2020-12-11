#include <stdio.h>

#define MM2S_DMACR_OFFSET 0x00
#define MM2S_DMASR_OFFSET 0x04
#define MM2S_DA_OFFSET 0x18
#define MM2S_LENGTH_OFFSET 0x28
#define S2MM_DMACR_OFFSET 0x30
#define S2MM_DA_OFFSET 0x48
#define S2MM_LENGTH_OFFSET 0x58
#define S2MM_DMASR_OFFSET 0x34

struct axi_dma {
  volatile char* register_space;
  volatile int rx_done;
  volatile int tx_done;
};

struct axi_dma_buffer {
  volatile void *data;
  size_t size;
};

void read_dma_simple(struct axi_dma *dma, struct axi_dma_buffer *buffer){
  iowrite32(dma->register_space + S2MM_DMACR_OFFSET, 0x1001);
  iowrite32(buffer->size, dma->register_space + S2MM_LENGTH_OFFSET);
  int reg = ioread32(dma->register_space + S2MM_DA_OFFSET);
  iowrite32(buffer->data, reg);
  iowrite32(dma->register_space + S2MM_DMASR_OFFSET, 0x1000);
}

void write_dma_simple(struct axi_dma *dma, struct axi_dma_buffer *buffer){
  iowrite32(dma->register_space + MM2S_DMACR_OFFSET, 0x1001);
  iowrite32(dma->register_space + MM2S_LENGTH_OFFSET, buffer->size);
  int reg = ioread32(buffer->data);
  iowrite32(dma->register_space + MM2S_DA_OFFSET, reg);
  iowrite32(dma->register_space + MM2S_DMASR_OFFSET, 0x1000);
}

void wait_rx_completion(struct axi_dma *dma) {
  while(!dma->rx_done){
    if(dma->register_space + S2MM_DMASR_OFFSET == 0x1000){
      dma_irq_rx(dma);
    }
  }
}

void wait_tx_completion(struct axi_dma *dma) {
  while(!dma->tx_done){
    if(dma->register_space + MM2S_DMASR_OFFSET == 0x1000){
      dma_irq_tx(dma);
    }
  }
}

void dma_irq_rx(struct axi_dma *dma){
  dma->rx_done = 1;
}
void dma_irq_tx(struct axi_dma *dma){
  dma->tx_done = 1;
}
