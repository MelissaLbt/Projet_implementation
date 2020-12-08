#include <stdio.h>

//Register space en mode scather-gather
#DEFINE MM2S_DMACR_OFFSET 0x00
#DEFINE MM2S_DMASR_OFFSET 0x04
#DEFINE MM2S_CURDESC 0x08
#DEFINE MM2S_TAILDESC_OFFSET 0x10
#DEFINE S2MM_DMACR_OFFSET 0x30
#DEFINE S2MM_DMASR_OFFSET 0x34
#DEFINE S2MM_CURDESC 0x38
#DEFINE S2MM_TAILDESC_OFFSET 0x40

//Descripteur en mode scather-gather
#DEFINE NEXTDESC_OFFSET 0x00
#DEFINE BUFFER_ADRRESS_OFFSET 0x08
#DEFINE CONTROL 0x18
#DEFINE STATUS 1C

struct axi_dma {
  volatile char* register_space;
  volatile int rx_done;
  volatile int tx_done;
};

struct axi_dma_buffer {
  volatile void *data;
  size_t size;
};

struct axi_dma_sg {
  u32 nextdesc;
  u32 nextdesc_msb;
  u32 buffer_address
  u32 buffer_address_msb;
  u32 reserved[2];
  u32 control;
  u32 status;
  u32 app[5];
  u32 padding[3];
} __attribute__((aligned(64)));


void dma_sg_init(struct axi_dma_sg *sg, struct axi_dma_buffer *buffer, size_t pkt_size){
  // size_t buffer_length = buffer->size;
  // while(buffer_length >= pkt_size){
  //   sg-> buffer_address = buffer->data(buffer_length to buffer_length - 64);
  //   buffer_length -= 64;
  //   sg = sg->nextdesc;
  // }
  // if(buffer_length != 0){
  //   sg-> buffer_address = buffer->data(buffer_length to 0);
  // }
}

void dma_sg_init_sparse(struct axi_dma_sg *sg, struct axi_dma_buffer **buffers, size_t num_buffers, size_t pkt_size){
    // for(int i = 0; i < num_buffers; i++){
    //   dma_sg_init(sg, buffers[i], pkt_size);
    //   sg = sg->nextdesc;
    // }
}

void read_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg);
void write_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg);
