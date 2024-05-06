#include <linux/types.h>

#define CHARDRIVER_IOCTL_RETURN_STATS  _IO('C', 0x10)
#define CHARDRIVER_IOCTL_CHANGE_BUFFERSIZE  _IO('C', 0x11)
#define CHARDRIVER_IOCTL_GET_BUFFERSIZE  _IO('C', 0x12)

typedef struct chardriver_stats{
     uint64_t opencount;
     uint64_t closecount;
     uint64_t readcount;
     uint64_t writecount;
     uint64_t mmapcount;
     uint64_t munmapcount;
     uint64_t openfail;
     uint64_t closefail;
     uint64_t readfail;
     uint64_t writefail;
     uint64_t mmapfail;

} stats_t;


