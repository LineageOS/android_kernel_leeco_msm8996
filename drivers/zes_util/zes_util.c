#include <linux/io.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/pm_wakeup.h>
#include <linux/kernel_stat.h>
#include <linux/irq.h>
#include <linux/zes_util.h>

static int zes_util_delay_time = 60000;
module_param_named(pm_delay, zes_util_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP);

static struct workqueue_struct *zes_pm_monitor_wq = NULL;

#define MAX_PID 32768
#define NUM_BUSY_THREAD_CHECK 5
struct _zes_kernel_power_monitor {
	struct delayed_work dwork;
	unsigned int *prev_proc_time;
	int *curr_proc_delta;
	int *curr_proc_pid;
	struct task_struct **task_ptr_array;
	struct kernel_cpustat curr_cpustat;
	struct kernel_cpustat prev_cpustat;
	unsigned long cpustat_time;
	int top_loading_pid[NUM_BUSY_THREAD_CHECK];
	spinlock_t lock;
};

static void sort_based_on_cpu_loading(int *curr_proc_delta, int *curr_pid, int pid_cnt, int top_loading_pid[NUM_BUSY_THREAD_CHECK])
{
	int i = 0;
	int j = 0;
	int k = 0;
	int temp_delta;

	for(i = 0; i < NUM_BUSY_THREAD_CHECK; i++)
	{
		top_loading_pid[i] = 0;
		temp_delta = 0;
 
		for(j = 0; j < pid_cnt; j++)
		{
			//printk("Rilcy Current PID %d\n", curr_pid[j]);
			if(curr_proc_delta[curr_pid[j]] >= temp_delta)	
			{
				for(k = 0; k < i; k++)
				{
					if(top_loading_pid[k] == curr_pid[j])
					{
						break;
					}
				}
				if(k == i)
				{
					temp_delta = curr_proc_delta[curr_pid[j]];
					top_loading_pid[i] = curr_pid[j];
				}	
			}
		}
	}
	
}

#ifdef arch_idle_time
static cputime64_t get_idle_time(int cpu)
{
        cputime64_t idle;

        idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
        if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
                idle += arch_idle_time(cpu);
        return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
        cputime64_t iowait;

        iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
        if (cpu_online(cpu) && nr_iowait_cpu(cpu))
                iowait += arch_idle_time(cpu);
        return iowait;
}

#else
static u64 get_idle_time(int cpu)
{
        u64 idle, idle_time = get_cpu_idle_time_us(cpu, NULL);

        if (idle_time == -1ULL)
                /* !NO_HZ so we can rely on cpustat.idle */
                idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
        else
                idle = usecs_to_cputime64(idle_time);

        return idle;
}

static u64 get_iowait_time(int cpu)
{
        u64 iowait, iowait_time = get_cpu_iowait_time_us(cpu, NULL);

        if (iowait_time == -1ULL)
                /* !NO_HZ so we can rely on cpustat.iowait */
                iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
        else
                iowait = usecs_to_cputime64(iowait_time);

        return iowait;
}
#endif

static void get_all_cpustat(struct kernel_cpustat *cpu_stat)
{
	int cpu;

	if (!cpu_stat)
		return;

	memset(cpu_stat, 0, sizeof(struct kernel_cpustat));

	for_each_possible_cpu(cpu) {
		cpu_stat->cpustat[CPUTIME_USER] += kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
		cpu_stat->cpustat[CPUTIME_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];
		cpu_stat->cpustat[CPUTIME_SYSTEM] += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
		cpu_stat->cpustat[CPUTIME_IDLE] += get_idle_time(cpu);
		cpu_stat->cpustat[CPUTIME_IOWAIT] += get_iowait_time(cpu);
		cpu_stat->cpustat[CPUTIME_IRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		cpu_stat->cpustat[CPUTIME_SOFTIRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
		cpu_stat->cpustat[CPUTIME_STEAL] += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
		cpu_stat->cpustat[CPUTIME_GUEST] += kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST];
		cpu_stat->cpustat[CPUTIME_GUEST_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST_NICE];
	}
}

static unsigned long zs_calculate_cpustat_time(struct kernel_cpustat curr_cpustat,
						struct kernel_cpustat prev_cpustat)
{
	unsigned long user_time = 0, system_time = 0, io_time = 0;
	unsigned long irq_time = 0, idle_time = 0;

	user_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_USER] +
					curr_cpustat.cpustat[CPUTIME_NICE]) -
					(prev_cpustat.cpustat[CPUTIME_USER] +
					prev_cpustat.cpustat[CPUTIME_NICE]));
	system_time = (unsigned long) (curr_cpustat.cpustat[CPUTIME_SYSTEM] -
					prev_cpustat.cpustat[CPUTIME_SYSTEM]);
	io_time = (unsigned long) (curr_cpustat.cpustat[CPUTIME_IOWAIT] -
					prev_cpustat.cpustat[CPUTIME_IOWAIT]);
	irq_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_IRQ] +
					curr_cpustat.cpustat[CPUTIME_SOFTIRQ]) -
					(prev_cpustat.cpustat[CPUTIME_IRQ] +
					 prev_cpustat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_IDLE] > prev_cpustat.cpustat[CPUTIME_IDLE]) ?
					curr_cpustat.cpustat[CPUTIME_IDLE] - prev_cpustat.cpustat[CPUTIME_IDLE] : 0);
	idle_time += (unsigned long) ((curr_cpustat.cpustat[CPUTIME_STEAL] +
					curr_cpustat.cpustat[CPUTIME_GUEST]) -
					(prev_cpustat.cpustat[CPUTIME_STEAL] +
					prev_cpustat.cpustat[CPUTIME_GUEST]));

	return (user_time + system_time + io_time + irq_time + idle_time);
}

static void kernel_top_cal(struct _zes_kernel_power_monitor *power_monitor)
{
	struct task_struct *process;
	struct task_cputime cputime;
	int pid_cnt = 0;

	spin_lock(&power_monitor->lock);
	for_each_process(process){
		thread_group_cputime(process, &cputime);
		if(process->pid < MAX_PID)
		{
			power_monitor->curr_proc_delta[process->pid] = (cputime.utime + cputime.stime) - 
									power_monitor->prev_proc_time[process->pid];
			power_monitor->task_ptr_array[process->pid] = process;
			if(power_monitor->curr_proc_delta[process->pid] > 0)
			{
				//printk("Ktop cal: Current PID: %d: %d\n", process->pid, power_monitor->curr_proc_delta[process->pid]);
				power_monitor->curr_proc_pid[pid_cnt] = process->pid;
				pid_cnt += 1; 
			}
		}
	}

	sort_based_on_cpu_loading(power_monitor->curr_proc_delta, power_monitor->curr_proc_pid, pid_cnt, power_monitor->top_loading_pid);
	
	get_all_cpustat(&power_monitor->curr_cpustat);
	power_monitor->cpustat_time = zs_calculate_cpustat_time(power_monitor->curr_cpustat, power_monitor->prev_cpustat);

	//printk("current cpustate time: %ld\n", power_monitor->cpustat_time);
	for_each_process(process){
		if(process->pid < MAX_PID)	
		{
			thread_group_cputime(process, &cputime);
			power_monitor->prev_proc_time[process->pid] = cputime.stime + cputime.utime;
		}
	}

	memcpy(&power_monitor->prev_cpustat, &power_monitor->curr_cpustat, sizeof(struct kernel_cpustat));
	spin_unlock(&power_monitor->lock);
}


static void kernel_top_show(struct _zes_kernel_power_monitor *power_monitor)
{
	int top_n_pid = 0, i;

	/* Print most time consuming processes */
	pr_info("[K]  CPU Usage\t\tPID\t\tName\n");
	for (i = 0; i < NUM_BUSY_THREAD_CHECK; i++) {
		top_n_pid = power_monitor->top_loading_pid[i];
		pr_info("[K]%8lu%%\t\t%d\t\t%s\t\t%d\n",
			power_monitor->curr_proc_delta[top_n_pid] * 100 / power_monitor->cpustat_time,
			top_n_pid,
			power_monitor->task_ptr_array[top_n_pid]->comm,
			power_monitor->curr_proc_delta[top_n_pid]);
	}
	memset(power_monitor->curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(power_monitor->task_ptr_array, 0, sizeof(int) * MAX_PID);
	memset(power_monitor->curr_proc_pid, 0, sizeof(int) * MAX_PID);	
}

static void zes_pm_monitor_work_func(struct work_struct *work)
{
	struct timespec ts;
	struct rtc_time tm;
	struct _zes_kernel_power_monitor *power_monitor = container_of(work, struct _zes_kernel_power_monitor, dwork.work);
	if (!zes_pm_monitor_wq) {
		printk("[ZES] zes_pm_monitor_wq is unavaliable.\n");
		return;
	}

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	printk("[ZES][PM] ZES PM Statistic start (%02d-%02d %02d:%02d:%02d)\n",
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	zs_rpm_master_stats_dump(0);
	queue_delayed_work(zes_pm_monitor_wq, &power_monitor->dwork, msecs_to_jiffies(zes_util_delay_time));
	kernel_top_cal(power_monitor);
	kernel_top_show(power_monitor);
	printk("[ZES][PM] ZES PM Statistic done\n");
}


void init_zes_power_monitor(void)
{
	struct _zes_kernel_power_monitor *zes_power_monitor;
	zes_pm_monitor_wq = create_workqueue("zes_pm_monitor_wq");
	if(!zes_pm_monitor_wq)
	{
		return;	
	}
	printk("[ZES] Success to create zes_pm_monitor_wq\n");
	zes_power_monitor = vmalloc(sizeof(*zes_power_monitor));
	spin_lock_init(&zes_power_monitor->lock);

	zes_power_monitor->prev_proc_time = vmalloc(sizeof(int) * MAX_PID);
	zes_power_monitor->curr_proc_delta = vmalloc(sizeof(int) * MAX_PID); 
	zes_power_monitor->curr_proc_pid = vmalloc(sizeof(int) * MAX_PID);
	zes_power_monitor->task_ptr_array = vmalloc(sizeof(int) * MAX_PID);

	memset(zes_power_monitor->curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(zes_power_monitor->task_ptr_array, 0, sizeof(int) * MAX_PID);
	memset(zes_power_monitor->curr_proc_pid, 0, sizeof(int) * MAX_PID);
	memset(zes_power_monitor->prev_proc_time, 0, sizeof(int) * MAX_PID);

	get_all_cpustat(&zes_power_monitor->curr_cpustat);
	get_all_cpustat(&zes_power_monitor->prev_cpustat); 

	INIT_DELAYED_WORK(&zes_power_monitor->dwork, zes_pm_monitor_work_func);
	queue_delayed_work(zes_pm_monitor_wq, &zes_power_monitor->dwork, msecs_to_jiffies(zes_util_delay_time));	
}

//module_init(init_zes_power_monitor);
//MODULE_DESCRIPTION("Power Debug Feature for ZEUSIS");
//MODULE_VERSION("1.2");
//MODULE_LICENSE("GPL v2");
