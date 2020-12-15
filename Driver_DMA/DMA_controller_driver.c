MODULE_AUTHOR("Mélissa LEBRETON, Meriem LAGHA, Diego MORENO, Lisa HENNEBELLE");
MODULE_DESCRIPTION("DMA controller driver");
MODULE_LICENSE("GPL");

//Librairies
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/device.h> // pour accéder au device
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <sys/ioctl.h>
#include <linux/mm.h>
#include <linux/highmem.h>

/* ##################################################  MACROS ################################################## */

// Définition des commandes IOCTL
#define DRIVER_NAME   "DMA-controller-driver"
#define MODULE_MAJOR  100
#define DMA_START     _IO(MODULE_MAJOR, 0)
#define DMA_WAIT      _IO(MODULE_MAJOR, 1)
#define DMA_MMAP      _IO(MODULE_MAJOR, 2)
#define DMA_STOP      _IO(MODULE_MAJOR, 3)

#define IOC_BIT 12 // bit de l'interruption IOC des registres de contrôle

// Définition pour SG
#define MM2S_CR 0x00 // offset du registre MM2S_DMACR
#define MM2S_SR 0x04 // offset du registre MM2S_DMACR
#define MM2S_CD 0x08 // offset du registre MM2S_CURDESC
#define MM2S_TD 0x10 // offset du registre MM2S_TAILDESC

#define S2MM_CR 0x30 // offset du registre S2MM_DMACR
#define S2MM_SR 0x34 // offset du registre S2MM_DMASR
#define S2MM_CD 0x38 // offset du registre S2MM_CURDESC
#define S2MM_TD 0x40 // offset du registre S2MM_TAILDESC

#define SIZE_MASK ((1 << 26) - 1) // masque pour la taille d'un paquet (champ control de la structure sg)
#define EOF_BIT 26 // bit EOF (end of frame) (champ control de la structure sg)
#define SOF_BIT 27 // bit SOF (start of frame) (champ control de la structure sg)

/* ##################################################  TYPE DEFINI ################################################## */
typedef unsigned int u32;
typedef u32 size_t;

/* ##################################################  EXTERN ################################################## */

extern u32 ioread32(volatile void *addr);
extern void iowrite32(u32 value, volatile void *addr); // j'ai inversé l'ordre des arguments pour avoir un prototype similaire à linux

/* ##################################################  STRUCTURE ################################################## */


// class de chardev
static struct class *class = NULL;

// structure représentant le device
struct dma_controller{
  struct platform_device *pdev;      // pointeur vers l'instance platform_device crée par le platform_driver avant l'appel à "probe()"
  struct cdev             cdev;      // device de type caractère
  dev_t                   dt;        // région du device de type caratère
  void __iomem           *registers; // mémoire physique du dma_controller remappée en espace virtuel kernel
  int irq;
};

struct kdata {
  struct device dev;
  void* addr;
  size_t size;
};

// Structure de l'AXI DMA
struct axi_dma
{
  volatile char *register_space; // adresse de base des registres
  volatile int   rx_done; // booléen pour savor si une lecture est terminée
  volatile int   tx_done; // booléen pour savoir si une écriture est terminée
};

// Structure du buffer
struct axi_dma_buffer
{
  volatile void *data; // pointeur vers l'adresse de base du buffer
  size_t size; // taille du buffer
};

//Structure des descripteurs SG
struct axi_dma_sg
{
  u32 nextdesc; // adresse [31:0] du descripteur suivant
  u32 nextdesc_msb; // adresse [63:32] du descripteur
  u32 buffer_address; // adresse [31:0] de base du paquet
  u32 buffer_address_msb; // adresse [63:32] de base du paquet
  u32 reserved[2];
  u32 control; // taille [25:0] + EOF [26] + SOF [27]
  u32 status; // champ de status
  u32 app[5];
  u32 padding[3];
} __attribute__((aligned(64)));

/* ##################################################  FONCTIONS DE TRAITEMENT ################################################## */

// Remapper la mémoire pour un driver
static int simple_kernel_mmap(struct file *f,struct vm_area_struct *vma){
  int ret;
  struct kdata *kdata;
  dma_addr_t phys;
  kdata = f->private_data;
  phys = virt_to_phys(kdata->addr);
  vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
  if(kdata->size < (vma->vm_end - vma->vm_start))
  {
    dev_err( &kdata->dev,"unable to allocate more than available memory\n");
    return -EINVAL;
  }
    ret = remap_pfm_range( vma,vm_start,phys >> PAGE_SHIFT,vma_>vm_end - vma->vm_start,vma->vm_page_prot);
    if(ret < 0)
    {
      dev_err(&kdata->dev,"unable to remap memory\n");
      return -EI0
    }
    return 0;
}

static ssize_t simple_user_read(struct file *f, char __user *udata, size_t size, loff_t *off){
  ssize_t ret;
  struct kdata *kdata;
  unsigned long nr_pages, addr;
  addr = (unsigned long) udata;
  addr &= ~(PAGE_SIZE - 1); // Définir PAGE_SIZE - 1 , Ajout à udata  ~(PAGE_SIZE - 1)
  nr_pages = DIV_ROUND_UP(size, PAGE_SIZE); // <-> (((size) + (PAGE_SIZE) - 1) / (size))
  kdata = f->private_data;
  ret = get_user_pages_fast(addr, nr_pages, FOLL_WRITE, kdata->addr); //retourne le nbre de page utilisateur épingler en mémoire
  if(ret < 0){
    dev_err(&kdata->dev, "Unable to get user page\n");
    return ret;
  }
  nr_pages = ret;
  kdata->size = nr_pages * PAGE_SIZE;
  kmap(kdata->addr) // Créer un espace mémoire virtuel pour une page
  /*###########################*/

  //do something with kdata

  /*##########################*/
  kunmap(kdata->addr); // Libère la mémoire d'une page
  put_page(kdata->addr);// Libère la copie d'un map d'un espace user en espace kernel
  return kdata->size; // Renvoie la taille de kdata (= 0 ?)
}

//Initialisation des descripteurs à partir d'un buffer
void dma_sg_init( struct axi_dma_sg *sg, struct axi_dma_buffer *buffer, size_t pkt_size)
{
  u32 i, j, size;
  size = buffer->size; // on sauvegarde la taille totale du buffer
  for(j = 0, i = 0 ; i < buffer->size ; i += pkt_size, j++) // on boucle sur les données du buffer
  {
    sg[j].buffer_address = (u32) buffer->data + i; // on écrit l'adresse de début de paquet
    sg[j].control = (pkt_size < size ? pkt_size : size) & SIZE_MASK; // on écrit la taille du paquet
    sg[j].nextdesc = (u32)&sg[j+1]; // on écrit l'adresse du descripteur suivant
    size -= pkt_size; // on décrémente la taille non assignée à un descripteur
  }
  sg[j-1].control |= (1 << EOF_BIT); // on écrit '1' sur le EOF bit du champ control du dernier descripteur
  sg[0].control |= (1 << SOF_BIT); // on écrit '1' sur le SOF bit du champ control du premier descripteur
}

//Initialisation des descripteurs à partir de plusieurs buffers
void dma_sg_init_sparse( struct axi_dma_sg *sg, struct axi_dma_buffer **buffers
                       , size_t num_buffers, size_t pkt_size)
{
  u32 i, j,k, size;
  j = 0;
  for(k=0; k < num_buffers ; k++) // on boucle sur les buffers
  {
    size = buffers[k]->size; // on sauvegarde la taille totale du buffer k
    for(i = 0 ; i < buffers[k]->size ; i += pkt_size, j++) // on boucle sur le buffer k
    {
      sg[j].buffer_address = (u32) buffers[k]->data + i; // on écrit l'adresse de début de paquet
      sg[j].control = (pkt_size < size ? pkt_size : size) & SIZE_MASK; // on écrit la taille du paquet
      sg[j].nextdesc = (u32)&sg[j+1]; // on écrit l'adresse du descripteur suivant
      size -= pkt_size; // on décrémente la taille non assignée à un descripteur
    }
  }
  sg[j-1].control |= (1 << EOF_BIT); // on écrit '1' sur le EOF bit du champ control du dernier descripteur
  sg[0].control |= (1 << SOF_BIT); // on écrit '1' sur le SOF bit du champ control du premier descripteur
}

//Ecrire les données pointées par la mémoire scatter-gather sur un stream
void write_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg)
{
  int i;
  dma->rx_done = 0; // on remet le rx_done à 0 pour pouvoir détecter une nouvelle interruption
  iowrite32(1 | (1 << IOC_BIT), dma->register_space + MM2S_CR); // on active le DMA et l'interruption IOC
  iowrite32((u32)sg, dma->register_space + MM2S_CD); // on écrit l'adresse du premier descripteur dans CURDESC
  for(i = 0 ; (sg[i].control & (1 << EOF_BIT)) == 0 ; i++); // on cherche le dernier descripteur
  iowrite32((u32)&sg[i-1], dma->register_space + MM2S_TD); // on écrit l'adresse du dernier descripteur dans TAILDESC
}

//Lire les données d’ un stream et les stocker dans les adresses pointées dans la mémoire scatter-gather
void read_dma_sg(struct axi_dma *dma, struct axi_dma_sg *sg)
{
  int i;
  dma->tx_done = 0; // on remet le tx_done à 0 pour pouvoir détecter une nouvelle interruption
  iowrite32(1 | (1 << IOC_BIT), dma->register_space + S2MM_CR); // on active le DMA et l'interruption IOC
  iowrite32((u32)sg, dma->register_space + S2MM_CD); // on écrit l'adresse du premier descripteur dans CURDESC
  for(i = 0 ; (sg[i].control & (1 << EOF_BIT)) == 0 ; i++); // on cherche le dernier descripteur
  iowrite32((u32)&sg[i], dma->register_space + S2MM_TD); // on écrit l'adresse du dernier descripteur dans TAILDESC
}



/* ##################################################  FONCTIONS DRIVER ################################################## */

// définition des fonctions pour la structure "file_operations"
static int dma_controller_open(struct inode *i, struct file *f){ // --> Chercher utilité struct inode
  struct dma_controller *mdev = container_of(i->i_cdev, struct dma_controller, cdev); // on récupère l'adresse de structure mdev grâce à la macro container_of
  f->private_data = mdev; // on écrit cette adresse dans le membre private_data
  return 0;
}

//Libére la structure du DMA
static int dma_controller_release(struct inode *i, struct file *f){
  f->private_data = NULL; // le fichier est fermé: on efface le membre private_data
  return 0;
}

//Gère le comportement du dma_controller
static long dma_controller_ioctl(struct file *f, unsigned int cmd, unsigned long arg){ //Page 14-15 de la doc
 struct dma_controller *mdev;
 int reg, regr, ret;
 void *userptr;
 mdev = f->private_data; // on récupère un pointeur vers la structure dma_controller
 userptr = (void*)arg;

  switch(cmd){
/*    ##### A REVOIR  #####
      case DMA_START:
      //iowrite32(0b1, mdev->registers);
      break;
    case DMA_WAIT:
      //iowrite32(0x1001, mdev->registers);
      break;
    case DMA_MMAP:
      ret = simple_kernel_mmap(f, &(mdev->registers));
      if(ret < 0){
        dev_err(&mdev->pdev->dev, "Impossible de mapper la mémoire\n");
        return -EIO;
      }
      break;
    case DMA_STOP:
      break;
*/
    default:
      return -EINVAL;
  }
  return 0;
}

// définition de la structure "file_operations"
static const struct file_operations dma_controller_fops =
{.open = dma_controller_open
, .release = dma_controller_release // release = close
, .unlocked_ioctl = dma_controller_ioctl
};

// définitions des fonctions pour le "platform_driver", fonction appelée pour chaque périphérique compatible au chargement du module
static int dma_controller_probe(struct platform_device *pdev){
  int ret;
  struct resource *res;
  struct dma_controller *mdev;
  struct device *dev;
  static int instance_num = 0;

  // allocation de la structure représentant le device
  mdev = kzalloc(sizeof(struct dma_controller), GFP_KERNEL);
	if(!mdev)
  {
    dev_err(&mdev->pdev->dev, "Unable to allocate dma_controller structure\n");
    return -ENOMEM;
  }
  mdev->pdev = pdev;
  platform_set_drvdata(pdev, mdev);

  // remappage de l'espace des registres
  res = platform_get_resource(pdev, IORESOURCE_MEM, 0); // on récupère la première ressource (et la seule dans ce cas)
  mdev->registers = devm_ioremap_resource(&pdev->dev, res); // on la remappe en espace virtuel kernel (le prefixe devm fait que le démappage sera fait automatiquement à la destruction du struct device associé
                                                            // , ici il s'agit de pdev->dev qui est alloué et désalloué automatiquement par le platform_driver)
  if(mdev->registers)
  {
    ret = -ENOMEM;
    dev_err(&mdev->pdev->dev, "Unable to remap resource\n");
    goto mdev_free;
  }
  // allocation d'une region de device de type caractère
  ret = alloc_chrdev_region(&mdev->dt, 0, 1, DRIVER_NAME);
  if(ret < 0)
  {
    dev_err(&mdev->pdev->dev, "Unable to request chrdev region\n");
    goto mdev_free;
  }
  // creation d'un device pour le cdev (la chaine de caractere "dma_controller<num>" est le nom du fichier inode)
  dev = device_create( class
                     , NULL, mdev->dt
                     , NULL, "dma_controller%d", instance_num++);
  if(!dev)
  {
    dev_err(&mdev->pdev->dev, "Unable to create device\n");
    ret = -ENOMEM;
    goto region_free;
  }
  // on initialise et ajoute le nouveau device de type caractere
  cdev_init(&mdev->cdev, &dma_controller_fops);
  ret = cdev_add(&mdev->cdev, mdev->dt, 1);
  if(ret < 0)
  {
    dev_err(&mdev->pdev->dev, "Unable to add char device\n");
    goto device_free;
  }
  mdev->irq = platform_get_irq(pdev, 0);
	if (mdev->irq < 0)
		return mdev->irq;

  return 0; // le driver a été chargé pour ce périphérique, on retourne 0

  device_free:
    device_destroy(class, mdev->dt);
  region_free:
    unregister_chrdev_region(mdev->dt, 1);
  mdev_free:
    kfree(mdev);
    return ret;

}

// fonction appelée pour chaque périphérique compatible lors du déchargement du module
static int dma_controller_remove(struct platform_device *pdev){
  // A Vérifier
  struct dma_controller *mdev;
  mdev = platform_get_drvdata(pdev);
  cdev_del(&mdev->cdev);
  device_destroy(class, mdev->dt);
  unregister_chrdev_region(mdev->dt, 1);
  kfree(mdev);
  return 0;
}

// définition de la structure d'id (à partir du device-tree) pour le platform-driver
static const struct of_device_id dma_controller_ids[] = { { .compatible = "eise,my_gpio" }, {} };
MODULE_DEVICE_TABLE(of, dma_controller_ids);

static struct platform_driver dma_controller_pdrv =
{ .driver =
  { .name = DRIVER_NAME
  , .owner = THIS_MODULE
  , .of_match_table = dma_controller_ids }
, .probe = dma_controller_probe
, .remove = dma_controller_remove
};


/* ##################################################  GESTION DMA ################################################## */

//Initialisation
static int _init dma_init(void){
  pr_info("Initialisation DMA controller\n");
  platform_driver_register(&dma_controller_pdrv); // cette fonction crée un platform_device et appelle la fonction probe du driver pour chaque périphérique compatible
  return 0;
}
//Module à décharger
static void _exit dma_exit(void){
  pr_info("Déchargement DMA controller\n");
  platform_driver_unregister(&dma_controller_pdrv); // cette fonction appelle la fonction remove du driver pour chaque périphérique compatible
}

//Chargement module
module_init(dma_init);
//Décharchargement module
module_exit(dma_exit);
