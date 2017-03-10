/*
 * The proc filesystem constants/structures
 */
#ifndef _LINUX_PROC_FS_H
#define _LINUX_PROC_FS_H

#include <linux/types.h>
#include <linux/fs.h>

struct proc_dir_entry;

#ifdef CONFIG_PROC_FS

/// wanglj, DATE20151210, NOTE, Bug FCFAMFRA-170 START {

//#ifdef CONFIG_WIKO_UNIFY
#define MAX_DEVINFO_STR_LEN  255
//add by tinno dev info
#define DEF_TINNO_DEV_INFO(name)  \
extern struct proc_dir_entry * creat_devinfo_file(const char *name,struct file_operations * fp); \
static size_t name##_devinfo_size=0; \
static char name##_des_buf[MAX_DEVINFO_STR_LEN]; \
static int name##_dev_info_open(struct inode *inode, struct file *file)  \
{  \
       for(name##_devinfo_size=0;name##_devinfo_size<MAX_DEVINFO_STR_LEN;name##_devinfo_size++) \
       { \
		if(name##_des_buf[name##_devinfo_size]==0) \
		{ \
			break; \
		} \
	 } \
	return 0; \
}  \
static int name##_dev_info_release(struct inode *inode, struct file *file)  \
{ \
	return 0; \
} \
static ssize_t name##_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)  \
{   ssize_t name##_ret=name##_devinfo_size; \
	if(name##_devinfo_size==0) \
	{ \
		return name##_devinfo_size; \
	} \
       name##_ret=copy_to_user(buf,name##_des_buf,name##_ret); \
       name##_ret=name##_devinfo_size; \
       name##_devinfo_size=0; \
	 return name##_ret;  \
}  \
static  struct file_operations  name##_info_fops = { \
	.owner		= THIS_MODULE, \
	.open		= name##_dev_info_open,  \
	.read		= name##_read,  \
	.release	= name##_dev_info_release,  \
};

#define CAREAT_TINNO_DEV_INFO(name)  creat_devinfo_file(#name,&name##_info_fops)

#define SET_DEVINFO_STR(name,str)  \
do{ \
	memset(name##_des_buf,0,MAX_DEVINFO_STR_LEN); \
	sprintf(name##_des_buf,"%s",str); \
}while(0)
//#endif
/// wanglj, Bug FCFAMFRA-170 END }


extern void proc_root_init(void);
extern void proc_flush_task(struct task_struct *);

extern struct proc_dir_entry *proc_symlink(const char *,
		struct proc_dir_entry *, const char *);
extern struct proc_dir_entry *proc_mkdir(const char *, struct proc_dir_entry *);
extern struct proc_dir_entry *proc_mkdir_data(const char *, umode_t,
					      struct proc_dir_entry *, void *);
extern struct proc_dir_entry *proc_mkdir_mode(const char *, umode_t,
					      struct proc_dir_entry *);
 
extern struct proc_dir_entry *proc_create_data(const char *, umode_t,
					       struct proc_dir_entry *,
					       const struct file_operations *,
					       void *);

static inline struct proc_dir_entry *proc_create(
	const char *name, umode_t mode, struct proc_dir_entry *parent,
	const struct file_operations *proc_fops)
{
	return proc_create_data(name, mode, parent, proc_fops, NULL);
}

extern void proc_set_size(struct proc_dir_entry *, loff_t);
extern void proc_set_user(struct proc_dir_entry *, kuid_t, kgid_t);
extern void *PDE_DATA(const struct inode *);
extern void *proc_get_parent_data(const struct inode *);
extern void proc_remove(struct proc_dir_entry *);
extern void remove_proc_entry(const char *, struct proc_dir_entry *);
extern int remove_proc_subtree(const char *, struct proc_dir_entry *);

#else /* CONFIG_PROC_FS */

static inline void proc_root_init(void)
{
}

static inline void proc_flush_task(struct task_struct *task)
{
}

static inline struct proc_dir_entry *proc_symlink(const char *name,
		struct proc_dir_entry *parent,const char *dest) { return NULL;}
static inline struct proc_dir_entry *proc_mkdir(const char *name,
	struct proc_dir_entry *parent) {return NULL;}
static inline struct proc_dir_entry *proc_mkdir_data(const char *name,
	umode_t mode, struct proc_dir_entry *parent, void *data) { return NULL; }
static inline struct proc_dir_entry *proc_mkdir_mode(const char *name,
	umode_t mode, struct proc_dir_entry *parent) { return NULL; }
#define proc_create(name, mode, parent, proc_fops) ({NULL;})
#define proc_create_data(name, mode, parent, proc_fops, data) ({NULL;})

static inline void proc_set_size(struct proc_dir_entry *de, loff_t size) {}
static inline void proc_set_user(struct proc_dir_entry *de, kuid_t uid, kgid_t gid) {}
static inline void *PDE_DATA(const struct inode *inode) {BUG(); return NULL;}
static inline void *proc_get_parent_data(const struct inode *inode) { BUG(); return NULL; }

static inline void proc_remove(struct proc_dir_entry *de) {}
#define remove_proc_entry(name, parent) do {} while (0)
static inline int remove_proc_subtree(const char *name, struct proc_dir_entry *parent) { return 0; }

#endif /* CONFIG_PROC_FS */

struct net;

static inline struct proc_dir_entry *proc_net_mkdir(
	struct net *net, const char *name, struct proc_dir_entry *parent)
{
	return proc_mkdir_data(name, 0, parent, net);
}

#endif /* _LINUX_PROC_FS_H */
