/*Méthodes :
- open associée à l'ouverture du périphérique. Cette méthode permet le plus souvent la détection et l'initialisation du hardware lorsque cela est nécessaire
- read associée à la lecture. Cette méthode permet de lire les données sur le périphérique (dans l'espace noyau) puis de faire passer ces données dans l'espace utilisateur appelant
- write associée à l'écriture. Cette méthode permet de passer les données de l'espace utilisateur à l'espace noyau puis d'envoyer les données au périphérique
- close associée à la fermeture d'accès. Cette méthode pourra exécuter des actions matérielles nécessaires à la fermeture du périphérique.
- ioctl, cette entrée permettra d'envoyer des séquences de paramétrage au périphérique en utilisant l'appel système ioctl depuis un programme utilisateur
- select, l'appel système select permet à un utilisateur de se mettre en attente d'évènements (read/write/signal) sur un ensemble de descripteurs de fichiers
*/

MODULE_AUTHOR("Mélissa LEBRETON, Meriem LAGHA");
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


// Définition des commandes IOCTL
#DEFINE DRIVER_NAME "DMA-controller-driver"
#DEFINE MODULE_MAJOR 100
#DEFINE DMA_START _IO(MODULE_MAJOR, 0)
#DEFINE DMA_WAIT  _IO(MODULE_MAJOR, 1)
#DEFINE DMA_MMAP _IO(MODULE_MAJOR, 2)
#DEFINE DMA_STOP _IO(MODULE_MAJOR, 3)

// class de chardev
static struct class *class = NULL;

// structure représentant le device
struct dma_controller{
  struct platform_device *pdev;      // pointeur vers l'instance platform_device crée par le platform_driver avant l'appel à "probe()"
  struct cdev             cdev;      // device de type caractère
  dev_t                   dt;        // région du device de type caratère
  void __iomem           *registers; // mémoire physique du dma_controller remappée en espace virtuel kernel
}

// définition des fonctions pour la structure "file_operations"
static int dma_controller_open(struct inode *i, struct file *f){ // --> Chercher utilité struct inode
  struct dma_controller *mdev = container_of(i->i_cdev, struct dma_controller, cdev); // on récupère l'adresse de structure mdev grâce à la macro container_of
  f->private_data = mdev; // on écrit cette adresse dans le membre private_data
  return 0;
}

static int dma_controller_release(struct inode *i, struct file *f){
  f->private_data = NULL; // le fichier est fermé: on efface le membre private_data
  return 0;
}

struct kdata
{
  struct device dev;
  void* addr;
  size_t size;
};

// Remapper la mémoire pour un driver
static int simple_kernel_mmap(struct file *f,struct vm_area_struct *vma)
{
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


static long dma_controller_ioctl(struct file *f, unsigned int cmd, unsigned long arg){ //Page 14-15 de la doc
 //Gère le comportement du dma_controller
 struct dma_controller *mdev;
 int reg, regr, ret;
 void *userptr;
 mdev = f->private_data; // on récupère un pointeur vers la structure dma_controller
 userptr = (void*)arg;

  switch(cmd){
    case DMA_START:
      iowrite32(0b1, mdev->registers);
      break;
    case DMA_WAIT:
      iowrite32(0x1001, mdev->registers);
      break;
    case DMA_MMAP:
      ret = simple_kernel_mmap(f, &(mdev->registers));
      if(ret < 0){
        dev_err(&mdev->pdev->dev, "Impossible de mapper la mémoire\n");
        return -EIO;
      }
      break;
    case DMA_STOP:
      iowrite32(0b0, mdev->registers);
      break;
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

// définitions des fonctions pour le "platform_driver"
// fonction appelée pour chaque périphérique compatible au chargement du module
static int dma_controller_probe(struct platform_device *pdev){
  
  int ret;
  struct resource *res;
  struct dma_controller *mdev; // crée la structure de device
  struct cdev chardev; // crée les chardev
  struct device *dev;
<<<<<<< HEAD
  static int instance_num = 0;
=======
  struct descripteur descr; //prépare les descripteurs
  struct file *f;
  struct vm_area_struct *vma;
>>>>>>> 5063e01a5604e525a119bdbe0dd993167cc66473

  mdev = kzalloc(sizeof(struct dma_controller), GFP_KERNEL);
  if(!mdev)
  {
    dev_err(&mdev->pdev->dev, "Unable to allocate dma_controller structure\n");
    return -ENOMEM;
  }
  mdev->pdev = pdev;
  platform_set_drvdata(pdev, mdev);
  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  *f=res;
  int map= simple_kernel_mmap(*f,mdev->registers);
  if(map)
  {
    ret = -ENOMEM;
    dev_err(&mdev->pdev->dev, "Unable to remap resource\n");
    goto mdev_free;
  }
<<<<<<< HEAD
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
  return 0; // le driver a été chargé pour ce périphérique, on retourne 0
  // gérer les interruptions

  mdev->irq = platform_get_irq(pdev, 0);
	if (mdev->irq < 0)
		return mdev->irq;

  device_free:
    device_destroy(class, mdev->dt);
  region_free:
    unregister_chrdev_region(mdev->dt, 1);
  mdev_free:
    kfree(mdev);
    return ret;
=======
>>>>>>> 5063e01a5604e525a119bdbe0dd993167cc66473
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
