#ifndef __BCMPH_LINUX_JIFFIES_H__
#define __BCMPH_LINUX_JIFFIES_H__

#include <linux/jiffies.h>

static inline unsigned long get_jiffies(void)
{
   return (jiffies);
}

#endif // __BCMPH_LINUX_JIFFIES_H__
