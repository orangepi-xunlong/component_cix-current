// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 Cix Technology Group Co., Ltd. All Rights Reserved.
 */

#include <mali_kbase.h>
#include <mali_kbase_config.h>
#include <mali_kbase_config_defaults.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <linux/pm_domain.h>
#include <linux/scmi_protocol.h>
#include <linux/reset.h>
#include <linux/mali_hw_access.h>

#include "backend/gpu/mali_kbase_clk_rate_trace_mgr.h"
#include "mali_kbase_config_platform.h"
#include <linux/acpi.h>

static struct kbase_platform_config dummy_platform_config;

struct kbase_platform_config *kbase_get_platform_config(void)
{
	return &dummy_platform_config;
}

#ifndef CONFIG_OF
int kbase_platform_register(void)
{
	return 0;
}

void kbase_platform_unregister(void)
{
}
#endif

#ifdef CONFIG_MALI_MIDGARD_DVFS
#if MALI_USE_CSF
int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation)
#else
int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation, u32 util_gl_share,
			      u32 util_cl_share[2])
#endif
{
	return 1;
}
#endif /* CONFIG_MALI_MIDGARD_DVFS */

static unsigned int get_rcsu_reg_offset(enum gpu_rcsu_hwreg rcsu_reg)
{
	switch (rcsu_reg) {
	case GPU_RCSU_HWREG_PGCTRL:
		return 0x218;
	case GPU_RCSU_HWREG_STRAP_PIN1:
		return 0x304;
	default:
		return 0;
	}
}

uint32_t sky1_rcsu_reg_read32(struct kbase_device *kbdev,
            enum gpu_rcsu_hwreg rcsu_reg)
{
	u32 val;
	unsigned int offset = get_rcsu_reg_offset(rcsu_reg);

	val = mali_readl(kbdev->rcsu_reg + offset);
	dev_dbg(kbdev->dev, "read rcsu reg offset 0x%x val 0x%x", offset, val);
	return val;
}

void sky1_rcsu_reg_write32(struct kbase_device *kbdev,
			enum gpu_rcsu_hwreg rcsu_reg, uint32_t value)
{
	unsigned int offset = get_rcsu_reg_offset(rcsu_reg);

	mali_writel(value, kbdev->rcsu_reg + offset);
}

#ifdef CONFIG_MALI_DEVFREQ

#if IS_ENABLED(CONFIG_ARM_SCMI_PERF_DOMAIN)
int sky1_gpu_clk_rate_change_notifier(struct notifier_block *nb, unsigned long event, void *data)
{
	struct devfreq_freqs *freqs = data;
	struct kbase_device *kbdev =
		container_of(nb, struct kbase_device, sky1_nb_call);
	unsigned long flags;
	unsigned int i;
	unsigned long new_freq;
	unsigned int new_freqMHz;

	dev_dbg(kbdev->dev, "%s new freq %ld with event %ld\n", __func__, freqs->new, event);
	spin_lock_irqsave(&kbdev->pm.clk_rtm.lock, flags);
	if (event == DEVFREQ_POSTCHANGE) {
		new_freq = freqs->new;
		new_freqMHz = freqs->new / 1000000;
		if (new_freqMHz % 10 != 0) {
			for (i = 0; i < kbdev->num_opps; i++) {
				if (kbdev->devfreq_table[i].opp_freq == freqs->new) {
					new_freq = kbdev->devfreq_table[i].real_freqs[0];
					break;
				}
			}
		}

		for (i = 0; i < BASE_MAX_NR_CLOCKS_REGULATORS; i++) {
			if (!kbdev->pm.clk_rtm.gpu_idle &&
				(kbdev->pm.clk_rtm.clks[i]->clock_val != new_freq)) {
				kbase_clk_rate_trace_manager_notify_all(&kbdev->pm.clk_rtm,
									kbdev->pm.clk_rtm.clks[i]->index, new_freq);
			}
			kbdev->pm.clk_rtm.clks[i]->clock_val = new_freq;
		}
	}
	spin_unlock_irqrestore(&kbdev->pm.clk_rtm.lock, flags);

	return NOTIFY_DONE;
}
#endif /*CONFIG_ARM_SCMI_PERF_DOMAIN */
#endif

static int sky1_gpu_attach_pd(struct kbase_device *kbdev)
{
	struct device_link *link;
	dev_info(kbdev->dev, "%s\n", __func__);

#ifdef CONFIG_MALI_DEVFREQ

#ifdef CONFIG_ARM_SCMI_SUPPORT_DT_ACPI
	kbdev->sky1_perf_dev = fwnode_dev_pm_domain_attach_by_name(kbdev->dev, "perf");
#else
  	kbdev->sky1_perf_dev = dev_pm_domain_attach_by_name(kbdev->dev, "perf");
#endif

	link = device_link_add(kbdev->dev, kbdev->sky1_perf_dev,
				DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (!link) {
		dev_err(kbdev->dev, "Failed to add device_link to gpu perf pd.\n");
		return -EINVAL;
	}
#endif

	if (!has_acpi_companion(kbdev->dev)) {
		kbdev->sky1_power_dev = dev_pm_domain_attach_by_name(kbdev->dev, "pd_gpu");
		link = device_link_add(kbdev->dev, kbdev->sky1_power_dev,
					DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (!link) {
			dev_err(kbdev->dev, "Failed to add device_link to gpu power pd.\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int sky1_gpu_detach_pd(struct kbase_device *kbdev)
{
	dev_info(kbdev->dev, "%s\n", __func__);

#ifdef CONFIG_MALI_DEVFREQ
	dev_pm_domain_detach(kbdev->sky1_perf_dev, "perf");
#endif
	if (!has_acpi_companion(kbdev->dev)) {
		dev_pm_domain_detach(kbdev->sky1_power_dev, "pd_gpu");
	}

	return 0;
}

static int kbase_platform_sky1_init(struct kbase_device *kbdev)
{
	int err = 0;
	u32 harvesting_reg_val;

	err = sky1_gpu_attach_pd(kbdev);

	if (err)
		return err;

	kbdev->gpu_reset = devm_reset_control_get(kbdev->dev, "gpu_reset");
	if (IS_ERR(kbdev->gpu_reset)) {
		dev_err(kbdev->dev, "failed to get gpu_reset\n");
		return PTR_ERR(kbdev->gpu_reset);
	}

	kbdev->clk_response_addr = ioremap(PM_CLK_RESPONSE_ADDR, SZ_4K);
	if (!kbdev->clk_response_addr) {
		dev_err(kbdev->dev, "failed to ioremap clk reponse addr\n");
		err = -EIO;
		goto init_failed;
	}

	kbdev->power_share_addr = ioremap(PM_POWER_SHARE_ADDR, SZ_4K);
	if (!kbdev->power_share_addr) {
		dev_err(kbdev->dev, "failed to ioremap power share addr\n");
		err = -EIO;
		goto init_failed;
	}

#if IS_ENABLED(CONFIG_ARCH_CIX)
	harvesting_reg_val = sky1_rcsu_reg_read32(kbdev, GPU_RCSU_HWREG_STRAP_PIN1);
	/* A bit value of harvesting reg is 1 indicates that the shader core is unavailable. */
	kbdev->harvesting_core_mask = ~((harvesting_reg_val & 0xFFFFFF0) >> 4) & MALI_TITAN_MC10_CORE_MASK;
	if(!__arch_hweight32(kbdev->harvesting_core_mask)) {
		dev_err(kbdev->dev, "available core nums after harvesting is NULL\n");
		err = -EINVAL;
		goto init_failed;
	}
	dev_info(kbdev->dev, "core mask after harvesting 0x%llx\n", kbdev->harvesting_core_mask);
#endif
	return 0;

init_failed:
	if(kbdev->clk_response_addr)
		iounmap(kbdev->clk_response_addr);
	if(kbdev->power_share_addr)
		iounmap(kbdev->power_share_addr);
	sky1_gpu_detach_pd(kbdev);
	return err;
}

static void kbase_platform_sky1_term(struct kbase_device *kbdev)
{
	if (!has_acpi_companion(kbdev->dev)) {
		if(kbdev->clk_response_addr)
			iounmap(kbdev->clk_response_addr);
		if(kbdev->power_share_addr)
			iounmap(kbdev->power_share_addr);
		sky1_gpu_detach_pd(kbdev);
	}
	return ;
}

/* -0 as default, 0 when manually set as off and 1 when manually set as on */
bool sky1_power_timer_enabled = false;

/* Read the current GPU power value and write to the shared address with PM */
static enum hrtimer_restart sky1_power_timer_callback(struct hrtimer *timer)
{
	struct kbase_device *kbdev = container_of(timer, struct kbase_device, sky1_power_timer);
	u32 power;

	if(sky1_power_timer_enabled) {
		mutex_lock(&kbdev->ipa.lock);
		kbase_get_real_power_locked(kbdev, &power, kbdev->current_nominal_freq,
							(kbdev->current_voltages[0] / 1000));
		mutex_unlock(&kbdev->ipa.lock);

		iowrite32(power, kbdev->power_share_addr);
		hrtimer_forward_now(timer, HR_TIMER_DELAY_MSEC(PM_POWER_MODEL_SAMPLE_INTERVAL_MS));
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}

static int kbase_platform_sky1_late_init(struct kbase_device *kbdev)
{
	hrtimer_init(&kbdev->sky1_power_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kbdev->sky1_power_timer.function = sky1_power_timer_callback;

	return 0;
}

static void kbase_platform_sky1_late_term(struct kbase_device *kbdev)
{
	hrtimer_cancel(&kbdev->sky1_power_timer);
}

struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = &kbase_platform_sky1_init,
	.platform_term_func = &kbase_platform_sky1_term,
	.platform_late_init_func = &kbase_platform_sky1_late_init,
	.platform_late_term_func = &kbase_platform_sky1_late_term,
};

/**
 * According to the logic in the hardware RTL, these registers need to be
 * set individually.
 * Otherwise, the LS_MEM_READ_SHORT counter value will remain fixed at 0.
 */
void kbase_enable_ls_mem_counter(struct kbase_device *kbdev)
{
	kbase_reg_write32(kbdev, GPU_RESERVED_ENUM(CSHW), 0xFFFFFFFF);
	kbase_reg_write32(kbdev, GPU_RESERVED_ENUM(MEMSYS), 0xFFFFFFFF);
	kbase_reg_write32(kbdev, GPU_RESERVED_ENUM(TILE), 0xFFFFFFFF);
	kbase_reg_write32(kbdev, GPU_RESERVED_ENUM(SHADER), 0xFFFFFFFF);

	kbase_reg_write32(kbdev, GPU_RESERVED_ENUM(CONFIG_INIT), 0x1);
}

#ifdef CONFIG_MALI_DEVFREQ
static void sky1_gpu_set_devfreq_table(struct kbase_device *kbdev, int opp_index,
				unsigned long freq, unsigned long volt) {
	int j = 0;
	unsigned int core_count;
	unsigned int freqMHz;
	u64 core_mask = 0, core_bit = 0;
	u64 max_available_core_mask;
	unsigned int max_available_core_count = 0;

	max_available_core_mask = kbdev->gpu_props.shader_present & kbdev->harvesting_core_mask;
	max_available_core_count = __arch_hweight32(max_available_core_mask);

	/* After converting the frequency to MHz, if the unit digit is 0,
	 * it indicates that the core_mask is 10.
	 * If the unit digit is not 0, it is used as the value for actual core_mask.
	 */
	freqMHz = freq / 1000000;
	if (freqMHz % 10 == 0) {
		for (j = 0; j < BASE_MAX_NR_CLOCKS_REGULATORS; j++) {
			kbdev->devfreq_table[opp_index].real_freqs[j] = freq;
			kbdev->devfreq_table[opp_index].opp_volts[j] = volt;
		}
		kbdev->devfreq_table[opp_index].core_mask = max_available_core_mask;
	} else {
		core_count = freqMHz % 10;
		for (j = 0; j < BASE_MAX_NR_CLOCKS_REGULATORS; j++) {
			/* Convert the frequency expressed in MHz to the real frequency expressed in Hz.
			* freq = (real_freq * core_count) / 10
			*/
			kbdev->devfreq_table[opp_index].real_freqs[j] = ((freqMHz - core_count) / core_count) * 10000000 ;
			kbdev->devfreq_table[opp_index].opp_volts[j] = volt;
		}

		/* If any shader core is powered up, the shader stack is also powered up.
		 * Therefore, in order to reduce the power consumption after shader core power up,
		 * prefer shader cores in the same shader stack.
		 */
		if(max_available_core_mask == MALI_TITAN_MC10_CORE_MASK) {
			switch (core_count) {
			case 1:
				core_mask = MALI_TITAN_MC01_CORE_MASK;
				break;
			case 2:
				core_mask = MALI_TITAN_MC02_CORE_MASK;
				break;
			case 3:
				core_mask = MALI_TITAN_MC03_CORE_MASK;
				break;
			case 4:
				core_mask = MALI_TITAN_MC04_CORE_MASK;
				break;
			case 5:
				core_mask = MALI_TITAN_MC05_CORE_MASK;
				break;
			case 6:
				core_mask = MALI_TITAN_MC06_CORE_MASK;
				break;
			case 7:
				core_mask = MALI_TITAN_MC07_CORE_MASK;
				break;
			case 8:
				core_mask = MALI_TITAN_MC08_CORE_MASK;
				break;
			case 9:
				core_mask = MALI_TITAN_MC09_CORE_MASK;
				break;
			default:
				WARN(true, "Unknown core count value: %d", core_count);
				break;
			}
		/* Configure the core mask based on the maximum available core count
		 * when enable harvesting.
		 */
		} else if(core_count > max_available_core_count) {
			core_mask = max_available_core_mask;
		} else {
			for (; core_count > 0; core_count--) {
				core_bit = ffs(max_available_core_mask);
				if (!core_bit) {
					dev_err(kbdev->dev, "OPP has more cores than GPU\n");
					return;
				}
				core_mask |= (1ull << (core_bit - 1));
				max_available_core_mask &= ~(1ull << (core_bit - 1));
			}
		}
		kbdev->devfreq_table[opp_index].core_mask = core_mask;
	}

	kbdev->devfreq_table[opp_index].opp_freq = freq;
	return;
}

int sky1_gpu_init_perf_opp_table(struct kbase_device *kbdev, struct devfreq_dev_profile *dp)
{
	int err = 0;
	int count;
	int i = 0;
	unsigned long freq, volt;
	struct dev_pm_opp *opp;
	unsigned long lowest_freq_khz = DEFAULT_REF_TIMEOUT_FREQ_KHZ;
	unsigned long found_lowest_freq = 0;

	err = scmi_device_opp_table_parse(kbdev->sky1_perf_dev);
	if (err) {
		dev_err(kbdev->dev, "Failed to parse opp table from scmi, err = %d.\n", err);
		return err;
	}

	count = dev_pm_opp_get_opp_count(kbdev->sky1_perf_dev);
	if (count <= 0) {
		dev_err(kbdev->dev, "failed to get opps count\n");
		return -ENODEV;
	}

	dp->freq_table = kmalloc_array(count, sizeof(dp->freq_table[0]), GFP_KERNEL);
	if (!dp->freq_table)
		return -ENOMEM;

	kbdev->devfreq_table = kmalloc_array(count, sizeof(struct kbase_devfreq_opp), GFP_KERNEL);
	if (!kbdev->devfreq_table)
		return -ENOMEM;

	for (i = 0, freq = 0; i < count; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(kbdev->sky1_perf_dev, &freq);
		if (IS_ERR(opp))
			break;

		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		sky1_gpu_set_devfreq_table(kbdev, i, freq, volt);
		dp->freq_table[i] = freq;

		err = dev_pm_opp_add(kbdev->dev, freq, volt);
		if (err) {
			dev_err(kbdev->dev, "failed to add opp %luHz\n", freq);
			while (i-- > 0) {
				dev_pm_opp_remove(kbdev->dev, dp->freq_table[i]);
			}
			return err;
		}
		dev_info(kbdev->dev, "init perf opp[%d] freq = %ld real_freq =%llu core_mask = 0x%llx\n",
					i, freq, kbdev->devfreq_table[i].real_freqs[0], kbdev->devfreq_table[i].core_mask);
	}

	/* init lowest frequency, which are used to compute the timeouts*/
	if (count > 0) {
		/* convert found frequency to KHz */
		found_lowest_freq = dp->freq_table[0] / 1000;
		if (found_lowest_freq < lowest_freq_khz)
			lowest_freq_khz = found_lowest_freq;
		kbdev->lowest_gpu_freq_khz = lowest_freq_khz;
	}

	if (count != i)
		dev_err(kbdev->dev, "Unable to enumerate all scmi OPPs (%d!=%d\n", count, i);

	dp->max_state = i;
	if (dp->max_state > 0) {
		/* Record the maximum frequency possible */
		kbdev->gpu_props.gpu_freq_khz_max = dp->freq_table[count - 1] / 1000;
	}

	kbdev->num_opps = count;

	return 0;
}

void sky1_remove_opp_table(struct kbase_device *kbdev, struct devfreq_dev_profile *dp)
{
	unsigned int i;

	for(i = 0; i < kbdev->num_opps; i++) {
		dev_pm_opp_remove(kbdev->dev, dp->freq_table[i]);
	}
}

#endif
