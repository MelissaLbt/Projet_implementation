#include <stdio.h>

//Register space en mode scather-gather
#DEFINE MM2S_DMACR_OFFSET 0x00
#DEFINE MM2S_DMASR_OFFSET 0x04
#DEFINE MM2S_CURDESC_OFFSET 0x08
// #DEFINE MM2S_CURDESC_MSB_OFFSET 0x0C
#DEFINE MM2S_TAILDESC_OFFSET 0x10
#DEFINE S2MM_DMACR_OFFSET 0x30
#DEFINE S2MM_DMASR_OFFSET 0x34
#DEFINE S2MM_CURDESC_OFFSET 0x38
// #DEFINE S2MM_CURDESC_MSB_OFFSET 0x3C
#DEFINE S2MM_TAILDESC_OFFSET 0x40

//Descripteur en mode scather-gather
#DEFINE NEXTDESC_OFFSET 0x00
// #DEFINE NEXTDESC_MSB_OFFSET 0x04
#DEFINE BUFFER_ADDRESS_OFFSET 0x08
// #DEFINE BUFFER_ADDRESS_MSB_OFFSET 0x0C
// #DEFINE RESERVED_0_OFFSET 0x10
// #DEFINE RESERVED_1_OFFSET 0x14
// #DEFINE CONTROL_OFFSET 0x18
// #DEFINE STATUS_OFFSET 0x1C
// #DEFINE APP_0_OFFSET 0x20
// #DEFINE APP_1_OFFSET 0x24
// #DEFINE APP_2_OFFSET 0x28
// #DEFINE APP_3_OFFSET 0x2C
// #DEFINE APP_4_OFFSET 0x30

//Décalage de 64 octets
#DEFINE OFFSET_64 0x40

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
  int i = 0;
  while(buffer_length >= pkt_size){
    sg[i].nextdesc =  &s[i + 1];
    sg[i].buffer_address = &data;
    i++;
    data = data + OFFSET_64;
    buffer_length -= pkt_size;
  }
}

void dma_sg_init_sparse(struct axi_dma_sg *sg, struct axi_dma_buffer **buffers, size_t num_buffers, size_t pkt_size){
  for(int i = 0; i < num_buffers; i++){
    dma_sg_init(sg, buffer[i], pkt_size);
  }
}

void read_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg){
  int i = 0, length = sg.length; // Nombre de descripteurs
  iowrite32(dma->register_space + S2MM_DMACR_OFFSET, (length << 16) | 0x1001); //IRQ Threshold + IOC_IRQ + RS
  iowrite32(dma->register_space + S2MM_TAILDESC_OFFSET, sg[length-1].nextdesc); // Adresse du dernier descripteur
  while(i != length ){
    int reg = ioread32(sg[0]->buffer_address);
    iowrite32(, reg);
  }
}

void write_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg){
  int length = sg.length; // Nombre de descripteurs
  iowrite32(dma->register_space + MM2S_DMACR_OFFSET, (length << 16) | 0x1001); //IRQ Threshold + IOC_IRQ + RS
  iowrite32(dma->register_space + MM2S_TAILDESC_OFFSET, sg[length-1].nextdesc); // Adresse du dernier descripteur
}
