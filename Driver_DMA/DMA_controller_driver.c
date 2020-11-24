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
// ICI on utilise la macro _IOW car le kernel doit écrire vers l'utilisateur
#DEFINE DMA_WRITE _IOW(MODULE_MAJOR , 0, int)
// ICI on utilise la macro _IOR car l'utilisateur doit écrire vers le kernel
#DEFINE DMA_READ _IOR(MODULE_MAJOR , 1, int)

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


static long dma_controller_ioctl(struct file *f, unsigned int cmd, unsigned long arg){
 //Gère le comportement du dma_controller
 struct dma_controller *mdev;
 int reg, regr, ret;
 void *userptr;
 mdev = f->private_data; // on récupère un pointeur vers la structure dma_controller
 userptr = (void*)arg;

  switch(cmd){

    // case DMA_WRITE :
    //   ret = simple_kernel_mmap(f, )//struct file *f,struct vm_area_struct *vma
    //   if(ret < 0){
    //     dev_err(&mdev->pdev->dev, "Unable to copy value to user\n");
    //     return -EIO;
    //   }
    //
    // case DMA_READ :
    //   //Char udata représente les données à lire
    //   ret = simple_user_read(f, char __user *udata, size_t size, loff_t *off); //*udata =  , size = sizeof(data), *off =
    //   if(ret < 0){
    //     dev_err(&mdev->pdev->dev, "Unable to copy value to user\n");
    //     return -EIO;
    //   }
    //   break;
    // default:
    //   return -EINVAL;
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
