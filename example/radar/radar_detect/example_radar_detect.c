/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
#include "example_radar_detect.h"
#include "wifi_intf_drv_to_app_internal.h"
#include "wifi_radar_api.h"

/* Buffer size definitions must match driver-side values (hal_radar.h) */
#define STATIC_REMOVE_SAMPLE_NUM  13      /* samples per frame after static removal */
#define RADAR_DEFAULT_FRAME_NUM   32      /* default frame count */
#define RADAR_IQ_DATA_BYTES       4       /* sizeof(struct rtw_radar_iq_data) = 2 * sizeof(s16) */
#define RADAR_SUSTAINED_TIME_S    10      /* s, hold time after target lost before dis_trk is cleared */

struct radar_report_ring_priv g_radar_rpt_priv = {0};
rtos_sema_t radar_ready_sema = NULL;

/* buffer in order and handle in order */
const u32 radar_report_max_size[6] = {
	1800,      /* RTW_RADAR_TYPE_STATIC_REMOVE_FAR */
	11400,      /* RTW_RADAR_TYPE_CFAR_AI_S_FAR */
	1800,      /* RTW_RADAR_TYPE_STATIC_REMOVE_NEAR */
	11400,      /* RTW_RADAR_TYPE_CFAR_AI_S_NEAR */
	3200,      /* RTW_RADAR_TYPE_STATIC_REMOVE_L_NEAR */
	1696         /* RTW_RADAR_TYPE_AI_L_NEAR */
};

#ifndef CONFIG_RADAR_M_FRIDGE_SCENE
/* Split v into sign / integer / 3-digit fraction for RTK_LOGx (no %f) */
static void radar_split_milli(float v, const char **sign, int *ip, int *fp)
{
	int m = (int)(v * 1000.0f);

	*sign = (m < 0) ? "-" : "";
	if (m < 0) {
		m = -m;
	}
	*ip = m / 1000;
	*fp = m % 1000;
}
#endif

static void radar_buffer_deinit(void)
{
	u8 i = 0;
	u8 *radar_buffer = NULL;

	if (g_radar_rpt_priv.radar_rpt_mutex) {
		rtos_mutex_take(g_radar_rpt_priv.radar_rpt_mutex, MUTEX_WAIT_TIMEOUT);

		for (i = 0; i < RADAR_REPORT_BUF_NUM; i++) {
			radar_buffer = g_radar_rpt_priv.radar_rpt_pkt[i].radar_buffer;
			if (radar_buffer) {
				rtos_mem_free(radar_buffer);
				g_radar_rpt_priv.radar_rpt_pkt[i].radar_buffer = NULL;
				radar_buffer = NULL;
			}
		}

		g_radar_rpt_priv.per_radar_rpt_done = FALSE;
		g_radar_rpt_priv.qlen = 0;
		for (i = 0; i < RADAR_REPORT_GROUP; i++) {
			g_radar_rpt_priv.radar_buf_write_idx[i] = 0;
		}
		g_radar_rpt_priv.radar_buf_read_idx = 0;
		g_radar_rpt_priv.curr_wbuf_idx = -1;

		rtos_mutex_give(g_radar_rpt_priv.radar_rpt_mutex);

		rtos_mutex_delete_static(g_radar_rpt_priv.radar_rpt_mutex);
	}
}

static s32 radar_buffer_init(void)
{
	u8 i = 0;
	s32 ret = RTK_SUCCESS;
	struct radar_report_data *radar_rpt_pkt = NULL;

	rtos_mutex_create_static(&g_radar_rpt_priv.radar_rpt_mutex);

	for (i = 0; i < RADAR_REPORT_BUF_NUM; i++) {
		radar_rpt_pkt = &g_radar_rpt_priv.radar_rpt_pkt[i];
		radar_rpt_pkt->radar_buffer = rtos_mem_zmalloc(radar_report_max_size[i % RADAR_REPORT_GROUP]);
		if (radar_rpt_pkt->radar_buffer) {
			radar_rpt_pkt->type = i % RADAR_REPORT_GROUP;
			radar_rpt_pkt->buf_size = radar_report_max_size[i % RADAR_REPORT_GROUP];
			radar_rpt_pkt->rpt_len = 0;
			radar_rpt_pkt->rpt_ready = 0;
		} else {
			RTK_LOGE(NOTAG, "ERR: radar init malloc fail\n");
			ret = RTK_FAIL;
			break;
		}
	}

	if (ret != RTK_SUCCESS) {
		radar_buffer_deinit();
	} else {
		g_radar_rpt_priv.per_radar_rpt_done = TRUE;
		g_radar_rpt_priv.qlen = 0;
		for (i = 0; i < RADAR_REPORT_GROUP; i++) {
			g_radar_rpt_priv.radar_buf_write_idx[i] = i % RADAR_REPORT_GROUP;  /* point to the expected location during initialization */
		}
		g_radar_rpt_priv.radar_buf_read_idx = 0;
		g_radar_rpt_priv.curr_wbuf_idx = -1;
	}

	return ret;
}

/* segments to report */
static u8 radar_assemble_report(u8 curr_wbuf_idx, u8 *evt_info)
{
	struct rtw_event_radar_proc_rpt_info *radar_rpt_seg_info = (struct rtw_event_radar_proc_rpt_info *)evt_info;
	struct radar_report_data *radar_rpt_pkt = &g_radar_rpt_priv.radar_rpt_pkt[curr_wbuf_idx];
	u32 rpt_len = 0;

	/* first segments: rpt_seg_start = 1 */
	if (radar_rpt_seg_info->rpt_seg_start) {
		rpt_len = sizeof(struct rtw_event_radar_proc_rpt_info) + radar_rpt_seg_info->radar_data_length;
		if (rpt_len <= radar_rpt_pkt->buf_size) {
			memcpy(radar_rpt_pkt->radar_buffer, evt_info, rpt_len);  //maintain header info
			radar_rpt_pkt->rpt_len = rpt_len;
			radar_rpt_pkt->rpt_ready |= BIT0;
		}
	} else {
		rpt_len = radar_rpt_seg_info->radar_data_length;
		if ((radar_rpt_pkt->rpt_len + rpt_len) <= radar_rpt_pkt->buf_size) {
			memcpy(radar_rpt_pkt->radar_buffer + radar_rpt_pkt->rpt_len, &radar_rpt_seg_info->radar_data, rpt_len);  //remove header info
			radar_rpt_pkt->rpt_len += rpt_len;
		}
	}
	if (radar_rpt_seg_info->rpt_seg_end) {
		radar_rpt_pkt->rpt_ready |= BIT1;

		/* update partial header info */
		struct rtw_event_radar_proc_rpt_info *radar_rpt_info = (struct rtw_event_radar_proc_rpt_info *)radar_rpt_pkt->radar_buffer;
		radar_rpt_info->rpt_seg_start = 0;
		radar_rpt_info->rpt_seg_end = 0;
		radar_rpt_info->radar_data_length = radar_rpt_pkt->rpt_len - sizeof(struct rtw_event_radar_proc_rpt_info);
	}

	return radar_rpt_seg_info->rpt_seg_end;
}

static u8 radar_buf_is_idle(struct radar_report_data *pkt)
{
	return pkt->rpt_ready == 0 && pkt->rpt_len == 0;
}

static s8 radar_fetch_next_wbuf(u8 rpt_type)
{
	struct radar_report_data *radar_rpt_pkt = NULL;
	struct radar_report_data *radar_rpt_pkt_tmp = NULL;
	s8 ret = RTK_FAIL;
	u8 write_idx = 0, tmp_idx = 0;

	if (!g_radar_rpt_priv.radar_rpt_mutex) {
		return ret;
	}

	rtos_mutex_take(g_radar_rpt_priv.radar_rpt_mutex, MUTEX_WAIT_TIMEOUT);

	write_idx = g_radar_rpt_priv.radar_buf_write_idx[rpt_type];
	radar_rpt_pkt = &g_radar_rpt_priv.radar_rpt_pkt[write_idx];

	if (radar_rpt_pkt->type != rpt_type) {
		RTK_LOGE(NOTAG, "ERR: slot type mismatch\n");
		goto out;
	}

	if (rpt_type == RTW_RADAR_TYPE_STATIC_REMOVE_FAR) {
		/* FAR starts a new group: all slots must be free */
		for (int i = 0; i < RADAR_REPORT_GROUP; i++) {
			tmp_idx = g_radar_rpt_priv.radar_buf_write_idx[i];
			radar_rpt_pkt_tmp = &g_radar_rpt_priv.radar_rpt_pkt[tmp_idx];
			if (!radar_buf_is_idle(radar_rpt_pkt_tmp)) {
				RTK_LOGA(NOTAG, "+++DBG_INFO: type %d buf busy, drop group\n", i);
				g_radar_rpt_priv.b_drop_group = 1;
				goto out;
			}
		}
	} else {
		if (!radar_buf_is_idle(radar_rpt_pkt)) {
			RTK_LOGA(NOTAG, "+++DBG_INFO: no free buffer for type %d\n", rpt_type);
			g_radar_rpt_priv.b_drop_group = 1;
			goto out;
		}
	}

	ret = write_idx;
	g_radar_rpt_priv.radar_buf_write_idx[rpt_type] = (write_idx + RADAR_REPORT_GROUP) % RADAR_REPORT_BUF_NUM;

out:
	rtos_mutex_give(g_radar_rpt_priv.radar_rpt_mutex);
	return ret;
}

static struct radar_report_data *radar_fetch_next_rbuf(u8 *rpt_valid)
{
	struct radar_report_data *radar_rpt_pkt = NULL;

	*rpt_valid = FALSE;
	if (g_radar_rpt_priv.radar_rpt_mutex) {
		rtos_mutex_take(g_radar_rpt_priv.radar_rpt_mutex, MUTEX_WAIT_TIMEOUT);
		if (g_radar_rpt_priv.qlen) {
			radar_rpt_pkt = &g_radar_rpt_priv.radar_rpt_pkt[g_radar_rpt_priv.radar_buf_read_idx];
			*rpt_valid = radar_rpt_pkt->rpt_ready & (BIT0 | BIT1) ? TRUE : FALSE;
		}
		rtos_mutex_give(g_radar_rpt_priv.radar_rpt_mutex);
	}

	return radar_rpt_pkt;
}

/* radar report callback */
void example_radar_detect_report_cb(u8 *evt_info)
{
	struct rtw_event_radar_proc_rpt_info *radar_rpt_seg_info = (struct rtw_event_radar_proc_rpt_info *)evt_info;

	static u16 print_cnt = 0;    // for Debug, delete in the future

	/* Drop the entire group when buffers were unavailable at group start.*/
	if (g_radar_rpt_priv.b_drop_group) {
		if (radar_rpt_seg_info->rpt_type == RADAR_REPORT_GROUP - 1 && radar_rpt_seg_info->rpt_seg_end) {
			g_radar_rpt_priv.b_drop_group = 0;
			g_radar_rpt_priv.per_radar_rpt_done = 1;
		}
		RTK_LOGD(NOTAG, "+++DBG_INFO: drop next group [%d] seg_end=%d\n", radar_rpt_seg_info->rpt_type, radar_rpt_seg_info->rpt_seg_end);
		return;
	}

	if (radar_rpt_seg_info->rpt_seg_start) {
		// RTK_LOGA(NOTAG, "+++DBG_INFO: new report [Type = %d  (print_cnt=%d)] =====================\n", print_cnt % RADAR_REPORT_GROUP, print_cnt);
		print_cnt += 1;
	}

	/**
	 * per radar pkt will be transmitted in segments, different radar report
	 * can be distinguished by per_radar_rpt_ongoing maintained by sw.
	 * per segments: radar header + raw data
	 */
	if (g_radar_rpt_priv.per_radar_rpt_done) {
		g_radar_rpt_priv.per_radar_rpt_done = 0;
		g_radar_rpt_priv.curr_wbuf_idx = radar_fetch_next_wbuf(radar_rpt_seg_info->rpt_type);

		if (g_radar_rpt_priv.curr_wbuf_idx < 0) {
			g_radar_rpt_priv.per_radar_rpt_done = 1;  /* an report may be missing the first few segments */
			return;
		}
	}

	g_radar_rpt_priv.per_radar_rpt_done = radar_assemble_report(g_radar_rpt_priv.curr_wbuf_idx, evt_info);

	if (g_radar_rpt_priv.per_radar_rpt_done) {
		rtos_mutex_take(g_radar_rpt_priv.radar_rpt_mutex, MUTEX_WAIT_TIMEOUT);
		g_radar_rpt_priv.qlen++;
		rtos_mutex_give(g_radar_rpt_priv.radar_rpt_mutex);

		rtos_sema_give(radar_ready_sema);
	}

}

static void radar_detect_thread(void *param)
{
	(void)param;
	struct rtw_radar_action_parm act_param = {0};
	struct radar_report_data *radar_rpt_pkt = NULL;
	u8 radar_report_valid = FALSE;
	struct wifi_radar_cfg radar_cfg = {.sustained_time = RADAR_SUSTAINED_TIME_S};
	struct wifi_radar_result radar_res;

	while (!wifi_is_running(STA_WLAN_INDEX)) {
		rtos_time_delay_ms(2000);  /* 2s */
	}

	/* init radar report buffer pool */
	if (radar_buffer_init() != RTK_SUCCESS) {
		goto done;
	}

	/**
	 * should use semaphore to wait radar report happen
	 * the following example shows that we wait for semaphore: radar_ready_sema
	 */
	rtos_sema_create(&radar_ready_sema, 0, RADAR_REPORT_BUF_NUM);
	if (!radar_ready_sema) {
		RTK_LOGE(NOTAG, "radar_sema init failed\r\n");
		goto done;
	}

	wifi_radar_register_report_cb(example_radar_detect_report_cb);

	/* config radar parameters and enable radar */
	act_param.mode = RTW_RADAR_NORMAL_MODE;
	act_param.chirp_bw = 1;        /* 40M: valid channels 5~9 */
	act_param.trig_period = 15;    /* unit ms */
	act_param.channel = 7;         /* recommended default center frequency */

	if (wifi_radar_set_cfg(&radar_cfg) != RTK_SUCCESS) {
		RTK_LOGE(NOTAG, "[radar] set cfg fail\n");
	}
	if (wifi_radar_algo_init() != RTK_SUCCESS) {
		RTK_LOGE(NOTAG, "[radar] algo init fail\n");
	}

	/* cis cfg and csi en */
	act_param.act = RTW_RADAR_ACT_CFG;  /* radar cfg */
	wifi_radar_config(&act_param);

	act_param.act = RTW_RADAR_ACT_EN;  /* radar en */
	act_param.enable = 1;
	wifi_radar_config(&act_param);

	while (1) {
		/* example: when radar rx done, dequeue radar report and do some process. */
		if (rtos_sema_take(radar_ready_sema, 0xFFFFFFFF) != RTK_SUCCESS) {
			rtos_sema_delete(radar_ready_sema);
			RTK_LOGE(NOTAG,  "Get radar_sema failed\r\n");

			/* radar disable */
			act_param.act = RTW_RADAR_ACT_EN;  /* radar dis */
			act_param.enable = 0;
			wifi_radar_config(&act_param);
			break;
		}

		radar_rpt_pkt = radar_fetch_next_rbuf(&radar_report_valid);
		if (radar_rpt_pkt) {
			if (radar_report_valid) {
				/*do something for handing radar info: like show radar data */
				// example_radar_detect_show(radar_rpt_pkt);
				{
					/* Use memcpy to safely extract float values from potentially unaligned struct */
					float aagc_tmp;
					float dagc_tmp[4];
					struct rtw_event_radar_proc_rpt_info *radar_rpt_n = (struct rtw_event_radar_proc_rpt_info *)radar_rpt_pkt->radar_buffer;
					memcpy(&aagc_tmp, &radar_rpt_n->aagc_gain, sizeof(float));
					memcpy(dagc_tmp, radar_rpt_n->dagc_gain_normal_mode, sizeof(dagc_tmp));
					wifi_radar_rpt_hdl(radar_rpt_pkt->radar_buffer, aagc_tmp, dagc_tmp);
				}
				/* one result per window */
				if (wifi_radar_get_result(&radar_res) == RTK_SUCCESS) {
#ifndef CONFIG_RADAR_M_FRIDGE_SCENE
					const char *ds, *rs;
					int di, df, ri, rf;
					radar_split_milli(radar_res.dis_trk, &ds, &di, &df);
					radar_split_milli(radar_res.rssi, &rs, &ri, &rf);
					RTK_LOGA(NOTAG, "[radar] dis_trk = %s%d.%03d m, is_human = %d, rssi = %s%d.%03d dBm\r\n", ds, di, df, radar_res.is_human, rs, ri, rf);
#endif
				}
			} else {
				RTK_LOGA(NOTAG, "+++DBG_INFO: handle invalid radar rpt: %d\n", radar_report_valid);
			}

			//handle done
			if (g_radar_rpt_priv.radar_rpt_mutex) {
				rtos_mutex_take(g_radar_rpt_priv.radar_rpt_mutex, MUTEX_WAIT_TIMEOUT);
				g_radar_rpt_priv.qlen--;
				radar_rpt_pkt->rpt_len = 0;
				radar_rpt_pkt->rpt_ready = FALSE;
				g_radar_rpt_priv.radar_buf_read_idx = (g_radar_rpt_priv.radar_buf_read_idx + 1) % RADAR_REPORT_BUF_NUM;
				rtos_mutex_give(g_radar_rpt_priv.radar_rpt_mutex);
				radar_rpt_pkt = NULL;
			}
		} else {
			RTK_LOGE(NOTAG, "ERR: Null pkt!\r\n");
		}
	}

done:
	wifi_radar_register_report_cb(NULL);
	/* free radar report buffer */
	wifi_radar_algo_deinit();
	radar_buffer_deinit();

	if (radar_ready_sema) {
		rtos_sema_delete(radar_ready_sema);
	}

	rtos_task_delete(NULL);
}

void example_radar_detect_show(struct radar_report_data *radar_rpt_pkt)
{
	struct rtw_event_radar_proc_rpt_info *radar_rpt_info = (struct rtw_event_radar_proc_rpt_info *)radar_rpt_pkt->radar_buffer;
	u16 *buff_tmp = NULL; /* for printf radar data*/
	u32 print_len = 0, i = 0;

	RTK_LOGA(NOTAG, "--------------Radar Rpt Type %d-------------------\n", radar_rpt_pkt->type);
	RTK_LOGA(NOTAG, "[RADAR] radar header info:\r\n");
	RTK_LOGA(NOTAG, "# rpt_type              = %d\r\n", radar_rpt_info->rpt_type);
	RTK_LOGA(NOTAG, "# rpt_seg_start         = %d\r\n", radar_rpt_info->rpt_seg_start);
	RTK_LOGA(NOTAG, "# rpt_seg_end           = %d\r\n", radar_rpt_info->rpt_seg_end);
	RTK_LOGA(NOTAG, "# bw_idx                = %d[0-70M;1-40M;2-20M]\r\n", radar_rpt_info->bw_idx);
	RTK_LOGA(NOTAG, "# chirp_width           = %d\r\n", radar_rpt_info->chirp_width);
	RTK_LOGA(NOTAG, "# chirp_num             = %d\r\n", radar_rpt_info->chirp_num);
	RTK_LOGA(NOTAG, "# frame_num             = %d\r\n", radar_rpt_info->frame_num);
	RTK_LOGA(NOTAG, "# frame_interval        = %d\r\n", radar_rpt_info->frame_interval);
	RTK_LOGA(NOTAG, "# fft_strt_idx          = %d\r\n", radar_rpt_info->fft_strt_idx);
	RTK_LOGA(NOTAG, "# fft_num_sub           = %d\r\n", radar_rpt_info->fft_num_sub);
	RTK_LOGA(NOTAG, "# channel               = %d\r\n", radar_rpt_info->channel);
	RTK_LOGA(NOTAG, "# doppler_sample_num    = %d\r\n", radar_rpt_info->doppler_sample_num);
	RTK_LOGA(NOTAG, "# isolation             = %d\r\n", radar_rpt_info->isolation);
	RTK_LOGA(NOTAG, "# range_leakage_dBx10   = %d\r\n", radar_rpt_info->range_leakage_dBx10);
	RTK_LOGA(NOTAG, "# doppler_t2f_strt_idx  = %d / %d / %d\r\n", radar_rpt_info->doppler_t2f_strt_idx[0],
			 radar_rpt_info->doppler_t2f_strt_idx[1], radar_rpt_info->doppler_t2f_strt_idx[2]);
	RTK_LOGA(NOTAG, "# doppler_t2f_end_idx   = %d / %d / %d\r\n", radar_rpt_info->doppler_t2f_end_idx[0],
			 radar_rpt_info->doppler_t2f_end_idx[1], radar_rpt_info->doppler_t2f_end_idx[2]);
	RTK_LOGA(NOTAG, "# doppler_t2f_step      = %d / %d / %d\r\n", radar_rpt_info->doppler_t2f_step[0],
			 radar_rpt_info->doppler_t2f_step[1], radar_rpt_info->doppler_t2f_step[2]);

	RTK_LOGA(NOTAG, "[RADAR] radar raw data: len = %d [rpt_len=%d]\r\n", radar_rpt_info->radar_data_length, radar_rpt_pkt->rpt_len);

	buff_tmp = (u16 *)(radar_rpt_info->radar_data);
	print_len = radar_rpt_info->radar_data_length / 2;

	for (i = 0; i < (print_len > 16 ? 16 : print_len); i++) {
		if (i % 8 == 0) {
			RTK_LOGA(NOTAG, "\r\n");
			RTK_LOGA(NOTAG, "[%04u]", i);
		}
		RTK_LOGA(NOTAG, "%04x ", buff_tmp[i]);
	}

	RTK_LOGA(NOTAG, "\n[RADAR] raw data done!\r\n\n");
}

void example_radar_detect(void)
{
	if (rtos_task_create(NULL, "radar_detect_thread", radar_detect_thread, NULL, 1024 * 16, 1) != RTK_SUCCESS) {
		RTK_LOGE(NOTAG, "\n\rERR: %s create radar_detect_thread failed", __FUNCTION__);
	}

	return;
}
