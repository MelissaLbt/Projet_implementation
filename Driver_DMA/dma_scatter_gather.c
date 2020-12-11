#include <stdio.h>
#include <stdlib.h>

//Register space en mode scather-gather
#define MM2S_DMACR_OFFSET 0x00
#define MM2S_DMASR_OFFSET 0x04
#define MM2S_CURDESC_OFFSET 0x08
// #DEFINE MM2S_CURDESC_MSB_OFFSET 0x0C
#define MM2S_TAILDESC_OFFSET 0x10
#define S2MM_DMACR_OFFSET 0x30
#define S2MM_DMASR_OFFSET 0x34
#define S2MM_CURDESC_OFFSET 0x38
// #DEFINE S2MM_CURDESC_MSB_OFFSET 0x3C
#define S2MM_TAILDESC_OFFSET 0x40

//Descripteur en mode scather-gather
#define NEXTDESC_OFFSET 0x00
#define BUFFER_ADDRESS_OFFSET 0x08

//Décalage de 64 octets
#define OFFSET_64 0x40

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
  u32 buffer_address;
  u32 buffer_address_msb;
  u32 reserved[2];
  u32 control;
  u32 status;
  u32 app[5];
  u32 padding[3];
} __attribute__((aligned(64)));

void dma_sg_init(struct axi_dma_sg *sg, struct axi_dma_buffer *buffer, size_t pkt_size){
  int taille_max = (int) buffer->size / pkt_size;
  void *data = buffer->data;
  for(int i = 0; i < taille_max; i++){
    sg[i].nextdesc =  &sg[i + 1];
    sg[i].buffer_address = &data;
    iowrite32(sg->control, pkt_size);
    data += pkt_size;
  }
}

void dma_sg_init_sparse(struct axi_dma_sg *sg, struct axi_dma_buffer **buffers, size_t num_buffers, size_t pkt_size){
  for(int i = 0; i < num_buffers; i++){
    dma_sg_init(sg, buffers[i], pkt_size);
  }
}

void read_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg){
  int i = 0, length = sizeof(sg) / sizeof(sg[0]); // Nombre de descripteurs
  iowrite32(dma->register_space + S2MM_DMACR_OFFSET, (length << 16) | 0x1001); //IRQ Threshold + IOC_IRQ + RS
  iowrite32(dma->register_space + S2MM_TAILDESC_OFFSET, sg[length-1].nextdesc); // Adresse du dernier descripteur
  while(i != length ){
    dma->register_space + S2MM_CURDESC_OFFSET = &sg[i];
    i++;
  }
}

void write_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg){
  int i = 0, length = sizeof(sg) / sizeof(sg[0]); // Nombre de descripteurs
  iowrite32(dma->register_space + MM2S_DMACR_OFFSET, (length << 16) | 0x1001); //IRQ Threshold + IOC_IRQ + RS
  iowrite32(dma->register_space + MM2S_TAILDESC_OFFSET, sg[length-1].nextdesc); // Adresse du dernier descripteur
  while(i != length ){
     sg[i] = dma->register_space + S2MM_CURDESC_OFFSET;
    i++;
  }
}
