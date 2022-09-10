#include <linux/ioctl.h>

#define MAGIC_NUMBER					'H'

#define	SET_TRIGGER						_IOW(MAGIC_NUMBER, 0, unsigned int)
#define	SET_ECHO						_IOW(MAGIC_NUMBER, 1, unsigned int)