
/* auto-generated by gen_syscalls.py, don't edit */

#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall_macros.h>

#ifdef __cplusplus
extern "C" {
#endif

K_SYSCALL_DECLARE1(K_SYSCALL_COUNTER_START, counter_start, int, struct device *, dev)

K_SYSCALL_DECLARE1(K_SYSCALL_COUNTER_STOP, counter_stop, int, struct device *, dev)

K_SYSCALL_DECLARE1(K_SYSCALL_COUNTER_READ, counter_read, u32_t, struct device *, dev)

K_SYSCALL_DECLARE1(K_SYSCALL_COUNTER_GET_PENDING_INT, counter_get_pending_int, int, struct device *, dev)

K_SYSCALL_DECLARE1(K_SYSCALL_COUNTER_GET_TOP_VALUE, counter_get_top_value, u32_t, struct device *, dev)

K_SYSCALL_DECLARE1(K_SYSCALL_COUNTER_GET_MAX_RELATIVE_ALARM, counter_get_max_relative_alarm, u32_t, struct device *, dev)

K_SYSCALL_DECLARE3(K_SYSCALL_COUNTER_SET_GUARD_PERIOD, counter_set_guard_period, int, struct device *, dev, u32_t, ticks, u32_t, flags)

K_SYSCALL_DECLARE2(K_SYSCALL_COUNTER_GET_GUARD_PERIOD, counter_get_guard_period, u32_t, struct device *, dev, u32_t, flags)

#ifdef __cplusplus
}
#endif

#endif