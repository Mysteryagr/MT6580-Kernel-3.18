#ifndef __AAL_CONTROL_SUB_H__
#define __AAL_CONTROL_SUB_H__

#define AAL_SUB_TAG                  "[ALS_SUB/AAL]"
#define AAL_SUB_LOG(fmt, args...)	 pr_debug(AAL_SUB_TAG fmt, ##args)
#define AAL_SUB_ERR(fmt, args...)    pr_err(AAL_SUB_TAG fmt, ##args)
extern int aal_use_sub;
#endif

