#ifndef _KERNEL_VERSION_H_
#define _KERNEL_VERSION_H_

/*  values come from cmake/version.cmake */

#define ZEPHYR_VERSION_CODE 69219
#define ZEPHYR_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))

#define KERNELVERSION          0x10E6300
#define KERNEL_VERSION_NUMBER  0x10E63
#define KERNEL_VERSION_MAJOR   1
#define KERNEL_VERSION_MINOR   14
#define KERNEL_PATCHLEVEL      99
#define KERNEL_VERSION_STRING  "1.14.99"

#endif /* _KERNEL_VERSION_H_ */
