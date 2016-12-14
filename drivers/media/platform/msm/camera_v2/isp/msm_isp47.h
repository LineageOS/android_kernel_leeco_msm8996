/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MSM_ISP47_H__
#define __MSM_ISP47_H__

#define VFE47_NUM_STATS_COMP 2
#define VFE47_NUM_STATS_TYPE 9
/* composite mask order */
enum msm_vfe47_stats_comp_idx {
	STATS_COMP_IDX_HDR_BE = 0,
	STATS_COMP_IDX_BG,
	STATS_COMP_IDX_BF,
	STATS_COMP_IDX_HDR_BHIST,
	STATS_COMP_IDX_RS,
	STATS_COMP_IDX_CS,
	STATS_COMP_IDX_IHIST,
	STATS_COMP_IDX_BHIST,
	STATS_COMP_IDX_AEC_BG,
};

extern struct msm_vfe_hardware_info vfe47_hw_info;
#endif /* __MSM_ISP47_H__ */
