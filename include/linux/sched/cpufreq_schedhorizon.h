/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_CPUFREQ_SCHEDHORIZON_H
#define _LINUX_SCHED_CPUFREQ_SCHEDHORIZON_H

#ifdef CONFIG_CPU_FREQ_GOV_SCHEDHORIZON
void restrict_escape_kick(u64 duration_ms);
#else/* CONFIG_CPU_FREQ_GOV_SCHEDHORIZON */
static inline
void restrict_escape_kick(u64 duration_ms)
{
}
#endif /* CONFIG_CPU_FREQ_GOV_SCHEDHORIZON */

#endif /* _LINUX_SCHED_CPUFREQ_H */