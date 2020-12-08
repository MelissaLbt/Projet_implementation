#include <stdio.h>

//Register space en mode scather-gather
#DEFINE MM2S_DMACR_OFFSET 0x00
#DEFINE MM2S_DMASR_OFFSET 0x04
#DEFINE MM2S_CURDESC_OFFSET 0x08
#DEFINE MM2S_CURDESC_MSB_OFFSET 0x0C
#DEFINE MM2S_TAILDESC_OFFSET 0x10
#DEFINE S2MM_DMACR_OFFSET 0x30
#DEFINE S2MM_DMASR_OFFSET 0x34
#DEFINE S2MM_CURDESC_OFFSET 0x38
#DEFINE S2MM_CURDESC_MSB_OFFSET 0x3C
#DEFINE S2MM_TAILDESC_OFFSET 0x40

//Descripteur en mode scather-gather
#DEFINE NEXTDESC_OFFSET 0x00
#DEFINE NEXTDESC_MSB_OFFSET 0x04
#DEFINE BUFFER_ADDRESS_OFFSET 0x08
#DEFINE BUFFER_ADDRESS_MSB_OFFSET 0x0C
#DEFINE RESERVED_0_OFFSET 0x10
#DEFINE RESERVED_1_OFFSET 0x14
#DEFINE CONTROL_OFFSET 0x18
#DEFINE STATUS_OFFSET 0x1C
#DEFINE APP_0_OFFSET 0x20
#DEFINE APP_1_OFFSET 0x24
#DEFINE APP_2_OFFSET 0x28
#DEFINE APP_3_OFFSET 0x2C
#DEFINE APP_4_OFFSET 0x30
#DEFINE OFFSET_64 0x100 //Décalage de 64 octets

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
  size_t buffer_length = buffer->size;
  void *data = buffer->data;
  while(buffer_length >= pkt_size){
    iowrite32(sg->nextdesc, data + NEXTDESC_OFFSET);
    iowrite32(sg->nextdesc_msb, data + NEXTDESC_MSB_OFFSET);
    iowrite32(sg->buffer_address, data + BUFFER_ADDRESS_OFFSET);
    iowrite32(sg->buffer_address_msb, data + BUFFER_ADDRESS_MSB_OFFSET);
    iowrite32(sg->reserved[0], data + RESERVED_0_OFFSET);
    iowrite32(sg->reserved[1], data + RESERVED_1_OFFSET);
    iowrite32(sg->control, data + CONTROL_OFFSET);
    iowrite32(sg->status, data + STATUS_OFFSET);
    iowrite32(sg->app[0], data + APP_0_OFFSET);
    iowrite32(sg->app[1], data + APP_1_OFFSET);
    iowrite32(sg->app[2], data + APP_2_OFFSET);
    iowrite32(sg->app[3], data + APP_3_OFFSET);
    iowrite32(sg->app[4], data + APP_4_OFFSET);
    data = data + OFFSET_64;
  }
}

void dma_sg_init_sparse(struct axi_dma_sg *sg, struct axi_dma_buffer **buffers, size_t num_buffers, size_t pkt_size){
  for(int i = 0; i < num_buffers; i++){
    dma_sg_init(sg, buffer[i], pkt_size);
  }
}

void read_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg){

}

void write_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg){

}
