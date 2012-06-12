/*
 * Copyright (c) 2012 Ahmed Ammar, Genesi, Inc. <aammar@genesi-usa.com>
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#define DEBUG 1

#include <drm/drmP.h>
#include <drm/imx-ipu-v3.h>
#include <drm/imx-ipu-v3-ioctls.h>
#include <drm/drm_gem_cma_helper.h>

#include <linux/kthread.h>
#include <linux/export.h>
#include <linux/kernel.h>

#include "ipu-prv.h"

struct ipu_ic {
	struct device		*dev;
	void __iomem		*base;
	struct ipu_soc		*ipu;
	bool			in_use;
};

struct ipu_ic *ipu_ic_get(struct ipu_soc *ipu)
{
	struct ipu_ic *ic = ipu->ic_priv;

	if (!ic)
		return ERR_PTR(-EINVAL);

	if (ic->in_use)
		return ERR_PTR(-EBUSY);

	ic->in_use = true;

	return ipu->ic_priv;
}
EXPORT_SYMBOL_GPL(ipu_ic_get);

void ipu_ic_put(struct ipu_ic *ic)
{
	ic->in_use = false;
}
EXPORT_SYMBOL_GPL(ipu_ic_put);

#define IPU_IC_REG(offset)	(offset)
#define IC_CONF			IPU_IC_REG(0x0000 << 2)
#define IC_PRP_ENC_RSC		IPU_IC_REG(0x0001 << 2)
#define IC_PRP_VF_RSC		IPU_IC_REG(0x0002 << 2)
#define IC_PP_RSC		IPU_IC_REG(0x0003 << 2)
#define IC_CMBP_1		IPU_IC_REG(0x0004 << 2)
#define IC_CMBP_2		IPU_IC_REG(0x0005 << 2)
#define IC_IDMAC_1		IPU_IC_REG(0x0006 << 2)
#define IC_IDMAC_2		IPU_IC_REG(0x0007 << 2)
#define IC_IDMAC_3		IPU_IC_REG(0x0008 << 2)
#define IC_IDMAC_4		IPU_IC_REG(0x0009 << 2)

enum ipu_ic_conf_registers {
	IC_CONF_PRPENC_EN	= (1 << 0),
	IC_CONF_PRPENC_CSC1	= (1 << 1),
	IC_CONF_PRPENC_ROT_EN	= (1 << 2),

	IC_CONF_PRPVF_EN	= (1 << 8),
	IC_CONF_PRPVF_CSC1	= (1 << 9),
	IC_CONF_PRPVF_CSC2	= (1 << 10),
	IC_CONF_PRPVF_CMB	= (1 << 11),
	IC_CONF_PRPVF_ROT_EN	= (1 << 12),

	IC_CONF_PP_EN		= (1 << 16),
	IC_CONF_PP_CSC1		= (1 << 17),
	IC_CONF_PP_CSC2		= (1 << 18),
	IC_CONF_PP_CMB		= (1 << 19),
	IC_CONF_PP_ROT_EN	= (1 << 20),

	IC_CONF_IC_GLB_LOC_A	= (1 << 28),
	IC_CONF_KEY_COLOR_EN	= (1 << 29),
	IC_CONF_RWS_EN		= (1 << 30),
	IC_CONF_CSI_MEM_WR_EN	= (1 << 31),

	IC_IDMAC_1_CB0_BURST_16		= (1 << 0),
	IC_IDMAC_1_CB1_BURST_16		= (1 << 1),
	IC_IDMAC_1_CB2_BURST_16		= (1 << 2),
	IC_IDMAC_1_CB3_BURST_16		= (1 << 3),
	IC_IDMAC_1_CB4_BURST_16		= (1 << 4),
	IC_IDMAC_1_CB5_BURST_16		= (1 << 5),
	IC_IDMAC_1_CB6_BURST_16		= (1 << 6),
	IC_IDMAC_1_CB7_BURST_16		= (1 << 7),

	IC_IDMAC_1_PRPENC_ROT_OFFSET	= 11,
	IC_IDMAC_1_PRPENC_ROT_MASK	= (3 << IC_IDMAC_1_PRPENC_ROT_OFFSET),
	IC_IDMAC_1_PRPVF_ROT_OFFSET	= 14,
	IC_IDMAC_1_PRPVF_ROT_MASK	= (3 << IC_IDMAC_1_PRPVF_ROT_OFFSET),
	IC_IDMAC_1_PP_ROT_OFFSET	= 17,
	IC_IDMAC_1_PP_ROT_MASK		= (3 << IC_IDMAC_1_PP_ROT_OFFSET),
	IC_IDMAC_1_PRPENC_FLIP_RS	= (1 << 20),
	IC_IDMAC_1_PRPVF_FLIP_RS	= (1 << 21),
	IC_IDMAC_1_PP_FLIP_RS		= (1 << 22),

	IC_IDMAC_2_PRPENC_HEIGHT_OFFSET	= 0,
	IC_IDMAC_2_PRPENC_HEIGHT_MASK	= (0x3FF << IC_IDMAC_2_PRPENC_HEIGHT_OFFSET),
	IC_IDMAC_2_PRPVF_HEIGHT_OFFSET	= 10,
	IC_IDMAC_2_PRPVF_HEIGHT_MASK	= (0x3FF << IC_IDMAC_2_PRPVF_HEIGHT_OFFSET),
	IC_IDMAC_2_PP_HEIGHT_OFFSET	= 20,
	IC_IDMAC_2_PP_HEIGHT_MASK	= (0x3FF << IC_IDMAC_2_PP_HEIGHT_OFFSET),

	IC_IDMAC_3_PRPENC_WIDTH_OFFSET	= 0,
	IC_IDMAC_3_PRPENC_WIDTH_MASK	= (0x3FF << IC_IDMAC_3_PRPENC_WIDTH_OFFSET),
	IC_IDMAC_3_PRPVF_WIDTH_OFFSET	= 10,
	IC_IDMAC_3_PRPVF_WIDTH_MASK	= (0x3FF << IC_IDMAC_3_PRPVF_WIDTH_OFFSET),
	IC_IDMAC_3_PP_WIDTH_OFFSET	= 20,
	IC_IDMAC_3_PP_WIDTH_MASK	= (0x3FF << IC_IDMAC_3_PP_WIDTH_OFFSET),
};

struct ipu_ic_resource {
	int ipu_channel_in;
	int ipu_channel_out;
};

static struct ipu_ic_resource ipu_ic_resources[] = {
	{	// MEM_CSC_MEM
		.ipu_channel_in = 11,
		.ipu_channel_out = 22,
	},
};

u32 ipu_ic_read(struct ipu_soc *ipu, unsigned offset)
{
	return readl(ipu->ic_reg + offset);
}

void ipu_ic_write(struct ipu_soc *ipu, u32 value, unsigned offset)
{
	writel(value, ipu->ic_reg + offset);
}

static void init_csc(struct ipu_ic *ic, uint32_t **csc_coeff)
{
	uint32_t param;
	uint32_t *base = NULL;

	base = ic->base + 0x6060; /* FIXME */

	param = (csc_coeff[3][0] << 27) |
		(csc_coeff[0][0] << 18) |
		(csc_coeff[1][1] << 9) | csc_coeff[2][2];
	writel(param, base++);

	/* scale = 2, sat = 0 */
	param = (csc_coeff[3][0] >> 5) | (2L << (40 - 32));
	writel(param, base++);

	param = (csc_coeff[3][1] << 27) |
		(csc_coeff[0][1] << 18) |
		(csc_coeff[1][0] << 9) | csc_coeff[2][0];
	writel(param, base++);
	param = (csc_coeff[3][1] >> 5);
	writel(param, base++);

	param = (csc_coeff[3][2] << 27) |
		(csc_coeff[0][2] << 18) |
		(csc_coeff[1][2] << 9) | csc_coeff[2][1];
	writel(param, base++);
	param = (csc_coeff[3][2] >> 5);
	writel(param, base++);
}

static irqreturn_t task_irq_handler(int irq, void *dcond)
{
	struct completion *comp = dcond;

	complete(comp);
	return IRQ_HANDLED;
}

extern struct drm_gem_cma_object *cma_objs[16]; /* FIXME */

int ipu_ic_init(struct ipu_soc *ipu, struct device *dev, unsigned long base)
{
	struct ipu_ic *ic;
	int ret;

	ic = devm_kzalloc(dev, sizeof(*ic), GFP_KERNEL);
	if (!ic)
		return -ENOMEM;

	ic->dev = dev;
	ic->ipu = ipu;
	ic->base = devm_ioremap(dev, base, SZ_256K);
	if (!ic->base)
		return -ENOMEM;

	ipu->ic_priv = ic;

	dev_info(dev, "CSC base: 0x%08lx remapped to %p\n",
			base, ic->base);

	return 0;
}

void ipu_ic_exit(struct ipu_soc *ipu)
{
}

int  colorspace_conversion_task(struct drm_device *drm, struct ipu_soc *ipu, struct drm_imx_ipu_queue *args)
{
	struct ipu_ic_resource *res;
	struct ipuv3_channel *channel_in, *channel_out;
	struct ipu_ch_param *cpmem_in, *cpmem_out;
	uint32_t ipu_ic_conf, ipu_ic_idmac_1, ipu_ic_idmac_2, ipu_ic_idmac_3;
	enum ipu_color_space format_in, format_out;
	struct completion comp;
	int ret = 0, i;

	struct ipu_ic *ic = ipu_ic_get(ipu);
	if(IS_ERR(ic)) {
		dev_err(ipu->dev, "ipu_ic_get failed.\n");
		return -1;
	}

	init_completion(&comp);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	res = &ipu_ic_resources[0];
	channel_in = ipu_idmac_get(ipu, res->ipu_channel_in);
	channel_out = ipu_idmac_get(ipu, res->ipu_channel_out);
	cpmem_in = ipu_get_cpmem(channel_in);
	cpmem_out = ipu_get_cpmem(channel_out);

	uint32_t **coeffs = kcalloc(4, sizeof(uint32_t *), GFP_KERNEL);
	for(i = 0; i < 4; i++) {
		coeffs[i] = kcalloc(3, sizeof(uint32_t), GFP_KERNEL);
		ret = copy_from_user(coeffs[i], args->csc_coeffs[i], 3 * sizeof(uint32_t));
		dev_dbg(ipu->dev, "ic: to:%p form:%p\n", coeffs[i], args->csc_coeffs[i]);
		if (ret) {
			dev_err(ipu->dev, "copy_from_user failed. (%d/%d)\n", ret, 3*sizeof(uint32_t));
			return -ret;
		}
	}

	init_csc(ic, coeffs);

	dev_dbg(ipu->dev, "colorspace_conversion_task:\n");
	dev_dbg(ipu->dev, "\tinput:\n");
	dev_dbg(ipu->dev, "\t\tphys: %x\n", args->input.phys);
	dev_dbg(ipu->dev, "\t\tpix.pixelformat: %x\n", args->input.pix.pixelformat);
	dev_dbg(ipu->dev, "\t\tpix.bytesperline: %d\n", args->input.pix.bytesperline);
	dev_dbg(ipu->dev, "\t\tpix.width: %d\n", args->input.pix.width);
	dev_dbg(ipu->dev, "\t\tpix.height: %d\n", args->input.pix.height);
	dev_dbg(ipu->dev, "\toutput:\n");
	dev_dbg(ipu->dev, "\t\tphys: %x\n", args->output.phys);
	dev_dbg(ipu->dev, "\t\tpix.pixelformat: %x\n", args->output.pix.pixelformat);
	dev_dbg(ipu->dev, "\t\tpix.bytesperline: %d\n", args->output.pix.bytesperline);
	dev_dbg(ipu->dev, "\t\tpix.width: %d\n", args->output.pix.width);
	dev_dbg(ipu->dev, "\t\tpix.height: %d\n", args->output.pix.height);

	args->input.phys = cma_objs[args->input.phys]->paddr;
	args->output.phys = cma_objs[args->output.phys]->paddr;

	ipu_cpmem_set_image(cpmem_in, &args->input);
	ipu_cpmem_set_image(cpmem_out, &args->output);

	ipu_cpmem_set_burstsize(cpmem_in,16);
	ipu_cpmem_set_burstsize(cpmem_out,16);

	ipu_idmac_set_double_buffer(channel_in, false);
	ipu_idmac_set_double_buffer(channel_out, false);

	ipu_ic_idmac_1 = ipu_ic_read(ipu, IC_IDMAC_1);
	ipu_ic_idmac_2 = ipu_ic_read(ipu, IC_IDMAC_2);
	ipu_ic_idmac_3 = ipu_ic_read(ipu, IC_IDMAC_3);

	ipu_ic_idmac_2 &= ~IC_IDMAC_2_PP_HEIGHT_MASK;
	ipu_ic_idmac_2 |= (args->input.pix.height - 1) << IC_IDMAC_2_PP_HEIGHT_OFFSET;

	ipu_ic_idmac_3 &= ~IC_IDMAC_3_PP_WIDTH_MASK;
	ipu_ic_idmac_3 |= (args->input.pix.width - 1)  << IC_IDMAC_3_PP_WIDTH_OFFSET;

//	ipu_ic_idmac_1 &= ~IC_IDMAC_1_CB2_BURST_16;
//	ipu_ic_idmac_1 &= ~IC_IDMAC_1_CB5_BURST_16;

	ipu_ic_idmac_1 |= IC_IDMAC_1_CB2_BURST_16;
	ipu_ic_idmac_1 |= IC_IDMAC_1_CB5_BURST_16;

	ipu_ic_write(ipu, ipu_ic_idmac_1, IC_IDMAC_1);
	ipu_ic_write(ipu, ipu_ic_idmac_2, IC_IDMAC_2);
	ipu_ic_write(ipu, ipu_ic_idmac_3, IC_IDMAC_3);

	ipu_cpmem_set_high_priority(channel_in);
	ipu_cpmem_set_high_priority(channel_out);

	ipu_ic_conf = ipu_ic_read(ipu, IC_CONF);
	ipu_ic_conf |= IC_CONF_PP_EN;
	ipu_ic_conf |= IC_CONF_PP_CSC1;
	ipu_ic_conf &= ~IC_CONF_PP_CMB;
	ipu_ic_write(ipu, ipu_ic_conf, IC_CONF);

	ipu_idmac_enable_channel(channel_out);
	ipu_idmac_enable_channel(channel_in);

	ipu_module_enable(ipu, IPU_CONF_IC_EN);

	int irq = platform_get_irq(to_platform_device(drm->dev), 0);
	ret = devm_request_irq(ipu->dev, ipu->irq_start + channel_out->num, task_irq_handler, 0, "task_irq", &comp);
	if (ret)
	    goto out;

	dev_dbg(ipu->dev, "ipu_idmac_select_buffer()\n");
	ipu_idmac_select_buffer(channel_out, 0);
	ipu_idmac_select_buffer(channel_in, 0);

	wait_for_completion_timeout(&comp, msecs_to_jiffies(100));

	dev_dbg(ipu->dev, "disable everything\n");

	free_irq(ipu->irq_start + channel_out->num, &comp);

	ipu_ic_conf = ipu_ic_read(ipu, IC_CONF);
	ipu_ic_conf &= ~(IC_CONF_PP_EN | IC_CONF_PP_CSC1 | IC_CONF_PP_CSC2 |
		 IC_CONF_PP_CMB);
	ipu_ic_write(ipu, ipu_ic_conf, IC_CONF);

	ipu_module_disable(ipu, IPU_CONF_IC_EN);

	ipu_idmac_disable_channel(channel_in);
	ipu_idmac_disable_channel(channel_out);
out:
	ipu_idmac_put(channel_in);
	ipu_idmac_put(channel_out);

	ipu_ic_put(ic);

	for(i = 0; i < 4; i++)
		kfree(coeffs[i]);
	kfree(coeffs);

	return ret;
}

int ipu_task_queue_ioctl(struct drm_device *drm, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;
	struct drm_imx_ipu_queue *req = data;
	struct ipu_soc *ipu = dev_get_drvdata(drm->dev->parent);

	switch (req->task) {
		case IPU_TASK_CSC:
			ret = colorspace_conversion_task(drm, ipu, req);
			break;
		default:
			ret = -ENOIOCTLCMD;
			break;
	}

	return ret;
}
