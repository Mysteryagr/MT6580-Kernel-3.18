#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>



/*create apk debug channel*/
#define PROC_UPGRADE					0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG				8
#define FTS_DEBUG_DIR_NAME	"fts_debug"
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		1016
#define READ_BUF_SIZE		1016
struct i2c_client *fts_i2c_client;
 static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
//extern int fts_ctpm_auto_clb(void);

/*dma declare, allocate and release*/
extern  unsigned char *tpDMABuf_va ;
extern  dma_addr_t tpDMABuf_pa ;
