#include "hideep3d.h"
#include "hideep_isp.h"

int hideep3d_pgm_r_reg(struct i2c_client *client, u32 addr, u32 *val)
{
	int ret = 0;
	u32 packet[3];
	u8* bulk = (u8*)packet + 3;

	packet[0] = htonl(0x00);
	packet[1] = htonl(addr);

	ret = i2c_master_send(client, bulk, 5);
	if (ret < 0) {
		goto err;
	}

	mdelay(1);
	ret = i2c_master_recv(client, (u8*)&(packet[2]), 4);

	if (ret < 0) {
		goto err;
	}

	*val = ntohl(packet[2]);

err:
	return ret;
}

int hideep3d_pgm_w_reg(struct i2c_client *client, u32 addr, u32 data)
{
	int ret = 0;
	u32 packet[3];
	u8* bulk = (u8*)packet + 3;

	packet[0] = htonl(0x80);
	packet[1] = htonl(addr);
	packet[2] = htonl(data);

    /* i2c_master_send */
	ret = i2c_master_send(client, bulk+0, 5);

	if (ret < 0) {
		goto err;
	}

	ret = i2c_master_send(client, bulk+5, 4);

	if (ret < 0) {
		goto err;
	}

err:
	return ret;
}

#ifdef PGM_BURST_WR
int hideep3d_pgm_w_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
	int ret = 0;
	int i;

	if ((len % 4) != 0)
		return -1;

	packet->header.w[0] = htonl((0x80 | (len / 4-1)));
	packet->header.w[1] = htonl(addr);

	for (i = 0; i < NVM_PAGE_SIZE / sizeof(u32); i++)
		packet->payload[i] = htonl(packet->payload[i]);

	ret = i2c_master_send(client, (u8*)&packet->header.b[3], (len+5));

	if (ret < 0) {
		goto err;
	}

err:
	return ret;
}

int hideep3d_pgm_r_mem(struct i2c_client *client, u32 addr, struct pgm_packet *packet, u32 len)
{
	int ret = 0;
	int i;

	if ((len % 4) != 0)
		return -1;

	packet->header.w[0] = htonl((0x00 | (len / 4-1)));
	packet->header.w[1] = htonl(addr);

	ret = i2c_master_send(client, (u8*)&packet->header.b[3], 5);

	if (ret < 0) {
		goto err;
	}

	ret = i2c_master_recv(client, (u8*)packet->payload, len);

	if (ret < 0) {
		goto err;
	}

	for(i = 0; i < NVM_PAGE_SIZE / sizeof(u32); i++)
		packet->payload[i] = htonl(packet->payload[i]);

err:
	return ret;
}
#endif

void hideep3d_sw_reset(struct i2c_client *client, u32 food)
{
	hideep3d_pgm_w_reg(client, SYSCON_WDT_CNT, food);
	hideep3d_pgm_w_reg(client, SYSCON_WDT_CON, 0x03);
	hideep3d_pgm_w_reg(client, SYSCON_WDT_CON, 0x01);

	HIDEEP3D_DBG("sw reset");
}

int hideep3d_load_dwz(struct hideep3d_t *h3d)
{
	int ret = 0;
	int retry = 4;
	struct pgm_packet packet_r;

	while (retry--) {
		HIDEEP3D_DBG("enter_pgm : %d", retry);
		ret = hideep3d_enter_pgm(h3d->client);
		if (ret >= 0)
			break;
	}

	if (retry <= 0) {
		HIDEEP3D_ERR("dwz enter_pgm : failed");
		return -1;
	}

	mdelay(50);

	ret = hideep3d_pgm_r_mem(h3d->client, HIDEEP_DWZ_ADDR, &packet_r, sizeof(struct dwz_3dinfo_t));
	if (ret < 0) {
		HIDEEP3D_ERR("i2c failed");
		goto i2c_err;
	}

	memcpy((u8*)h3d->dwz_info, packet_r.payload, sizeof(struct dwz_3dinfo_t));
	hideep3d_sw_reset(h3d->client, 10);

	HIDEEP3D_INFO("firmware boot version   : %04x", h3d->dwz_info->ver_b);
	HIDEEP3D_INFO("firmware core version   : %04x", h3d->dwz_info->ver_c);
	HIDEEP3D_INFO("firmware custom version : %04x", h3d->dwz_info->ver_d);
	HIDEEP3D_INFO("firmware vr version     : %04x", h3d->dwz_info->ver_v);

	mdelay(50);

	return 0;
i2c_err:
	return -1;
}

unsigned int hideep3d_chipid(struct hideep3d_t *h3d)
{
	int ret = 0;
	int retry = 4;
	unsigned int status;

	while (retry--) {
		HIDEEP3D_DBG("enter_pgm : %d", retry);
		ret = hideep3d_enter_pgm(h3d->client);
		if (ret >= 0)
			break;
	}

	if (retry <= 0) {
		HIDEEP3D_ERR("chip id enter_pgm : failed");
		return -1;
	}
	mdelay(50);

	hideep3d_pgm_r_reg(h3d->client, SYSCON_CHIP_ID, &status);
	hideep3d_sw_reset(h3d->client, 10);

	return status;
}

