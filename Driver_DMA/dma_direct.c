#include <stdio.h>

#DEFINE S2MM_DMACR_OFFSET 0x30
#DEFINE S2MM_LENGTH_OFFSET 0x34
#DEFINE S2MM_DMASR_OFFSET 0x58
#DEFINE MM2S_DMACR_OFFSET 0x00
#DEFINE MM2S_LENGTH_OFFSET 0x04
#DEFINE MM2S_DMASR_OFFSET 0x28

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
  // ioread32(dma->register_space);
}

void write_dma_simple(struct axi_dma *dma, struct axi_dma_buffer *buffer){
  // iowrite32(dma->register_space, buffer->data);
}

void wait_rx_completion(struct axi_dma *dma) {
  // struct axi_dma_buffer buffer;
  // while(!dma->rx_done){
  //   read_dma_simple(dma, buffer);
  // }
  // buffer->data = dma->register_space;
}

void wait_tx_completion(struct axi_dma *dma) {
  // struct axi_dma_buffer buffer;
  // while(!dma->tx_done){
  //   write_dma_simple(dma, buffer);
  // }
}

void dma_irq_rx(struct axi_dma *dma){
  // if(S2MM_DMASR(12) == 1){
  //   dma->rx_done = 1;
  //   S2MM_DMASR(12) = 0;
  }
}
void dma_irq_tx(struct axi_dma *dma){
  // if(MM2S_DMASR(12) == 1){
  //   dma->tx_done = 1;
  //   MM2S_DMASR(12) = 0;
  }
}
