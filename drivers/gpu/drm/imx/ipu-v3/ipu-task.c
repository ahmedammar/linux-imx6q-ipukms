#include <drm/drmP.h>
#include <drm/imx-ipu-v3.h>
#include <drm/imx-ipu-v3-ioctls.h>

#include "ipu-prv.h"

#define IPU_IC_REG(offset)	(offset)
#define IC_CONF			IPU_IC_REG(0x0000)
#define IC_PRP_ENC_RSC		IPU_IC_REG(0x0001*4)
#define IC_PRP_VF_RSC		IPU_IC_REG(0x0002*4)
#define IC_PP_RSC		IPU_IC_REG(0x0003*4)
#define IC_CMBP_1		IPU_IC_REG(0x0004*4)
#define IC_CMBP_2		IPU_IC_REG(0x0005*4)
#define IC_IDMAC_1		IPU_IC_REG(0x0006*4)
#define IC_IDMAC_2		IPU_IC_REG(0x0007*4)
#define IC_IDMAC_3		IPU_IC_REG(0x0008*4)
#define IC_IDMAC_4		IPU_IC_REG(0x0009*4)

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

ipu_color_space_t format_to_colorspace(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_RGB565:
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_GBR24:
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_ABGR32:
	case IPU_PIX_FMT_LVDS666:
	case IPU_PIX_FMT_LVDS888:
		return IPU_COLORSPACE_RGB;
		break;

	default:
		return IPU_COLORSPACE_YCBCR;
		break;
	}
	return IPU_COLORSPACE_RGB;
}

uint32_t bytes_per_pixel(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_GENERIC:	/*generic data */
	case IPU_PIX_FMT_RGB332:
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YVU420P:
	case IPU_PIX_FMT_YUV422P:
		return 1;
		break;
	case IPU_PIX_FMT_RGB565:
	case IPU_PIX_FMT_YUYV:
	case IPU_PIX_FMT_UYVY:
		return 2;
		break;
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		return 3;
		break;
	case IPU_PIX_FMT_GENERIC_32:	/*generic data */
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_ABGR32:
		return 4;
		break;
	default:
		return 1;
		break;
	}
	return 0;
}

u32 ipu_ic_read(struct ipu_soc *ipu, unsigned offset)
{
	return readl(ipu->ic_reg + offset);
}

void ipu_ic_write(struct ipu_soc *ipu, u32 value, unsigned offset)
{
	writel(value, ipu->ic_reg + offset);
}

static void _init_csc(struct ipu_soc *ipu, uint8_t ic_task, ipu_color_space_t in_format,
		      ipu_color_space_t out_format, int csc_index)
{
	uint32_t param;
	uint32_t *base = NULL;

	/*
	 * Y = R *  .299 + G *  .587 + B *  .114;
	 * U = R * -.169 + G * -.332 + B *  .500 + 128.;
	 * V = R *  .500 + G * -.419 + B * -.0813 + 128.;
	 */
	static const uint32_t rgb2ycbcr_coeff[4][3] = {
		{0x004D, 0x0096, 0x001D},
		{0x01D5, 0x01AB, 0x0080},
		{0x0080, 0x0195, 0x01EB},
		{0x0000, 0x0200, 0x0200},	/* A0, A1, A2 */
	};

	/*
	 * transparent RGB->RGB matrix for combining
	 */
	static const uint32_t rgb2rgb_coeff[4][3] = {
		{0x0080, 0x0000, 0x0000},
		{0x0000, 0x0080, 0x0000},
		{0x0000, 0x0000, 0x0080},
		{0x0000, 0x0000, 0x0000},	/* A0, A1, A2 */
	};

	/*
	 * R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128));
	 * G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128));
	 * B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128); 
	 */
	static const uint32_t ycbcr2rgb_coeff[4][3] = {
		{149, 0, 204},
		{149, 462, 408},
		{149, 255, 0},
		{8192 - 446, 266, 8192 - 554},	/* A0, A1, A2 */
	};

	/*if (ic_task == IC_TASK_ENCODER) {
		base = ipu->tpmem_base + 0x2008 / 4;
	} else if (ic_task == IC_TASK_VIEWFINDER) {
		if (csc_index == 1)
			base = ipu->tpmem_base + 0x4028 / 4;
		else
			base = ipu->tpmem_base + 0x4040 / 4;
	} else if (ic_task == IC_TASK_POST_PROCESSOR) {*/
		if (csc_index == 1)
			base = ipu->tpmem_base + 0x6060;
		else
			base = ipu->tpmem_base + 0x6078;
	/*} else {
		BUG();
	}*/

	if ((in_format == IPU_COLORSPACE_YCBCR) && (out_format == IPU_COLORSPACE_RGB)) {
		/* Init CSC (YCbCr->RGB) */
		param = (ycbcr2rgb_coeff[3][0] << 27) |
			(ycbcr2rgb_coeff[0][0] << 18) |
			(ycbcr2rgb_coeff[1][1] << 9) | ycbcr2rgb_coeff[2][2];
		writel(param, base++);
		/* scale = 2, sat = 0 */
		param = (ycbcr2rgb_coeff[3][0] >> 5) | (2L << (40 - 32));
		writel(param, base++);

		param = (ycbcr2rgb_coeff[3][1] << 27) |
			(ycbcr2rgb_coeff[0][1] << 18) |
			(ycbcr2rgb_coeff[1][0] << 9) | ycbcr2rgb_coeff[2][0];
		writel(param, base++);
		param = (ycbcr2rgb_coeff[3][1] >> 5);
		writel(param, base++);

		param = (ycbcr2rgb_coeff[3][2] << 27) |
			(ycbcr2rgb_coeff[0][2] << 18) |
			(ycbcr2rgb_coeff[1][2] << 9) | ycbcr2rgb_coeff[2][1];
		writel(param, base++);
		param = (ycbcr2rgb_coeff[3][2] >> 5);
		writel(param, base++);
	} else if ((in_format == IPU_COLORSPACE_RGB) && (out_format == IPU_COLORSPACE_YCBCR)) {
		/* Init CSC (RGB->YCbCr) */
		param = (rgb2ycbcr_coeff[3][0] << 27) |
			(rgb2ycbcr_coeff[0][0] << 18) |
			(rgb2ycbcr_coeff[1][1] << 9) | rgb2ycbcr_coeff[2][2];
		writel(param, base++);
		/* scale = 1, sat = 0 */
		param = (rgb2ycbcr_coeff[3][0] >> 5) | (1UL << 8);
		writel(param, base++);

		param = (rgb2ycbcr_coeff[3][1] << 27) |
			(rgb2ycbcr_coeff[0][1] << 18) |
			(rgb2ycbcr_coeff[1][0] << 9) | rgb2ycbcr_coeff[2][0];
		writel(param, base++);
		param = (rgb2ycbcr_coeff[3][1] >> 5);
		writel(param, base++);

		param = (rgb2ycbcr_coeff[3][2] << 27) |
			(rgb2ycbcr_coeff[0][2] << 18) |
			(rgb2ycbcr_coeff[1][2] << 9) | rgb2ycbcr_coeff[2][1];
		writel(param, base++);
		param = (rgb2ycbcr_coeff[3][2] >> 5);
		writel(param, base++);
	} else if ((in_format == IPU_COLORSPACE_RGB) && (out_format == IPU_COLORSPACE_RGB)) {
		/* Init CSC */
		param =
		    (rgb2rgb_coeff[3][0] << 27) | (rgb2rgb_coeff[0][0] << 18) |
		    (rgb2rgb_coeff[1][1] << 9) | rgb2rgb_coeff[2][2];
		writel(param, base++);
		/* scale = 2, sat = 0 */
		param = (rgb2rgb_coeff[3][0] >> 5) | (2UL << 8);
		writel(param, base++);

		param =
		    (rgb2rgb_coeff[3][1] << 27) | (rgb2rgb_coeff[0][1] << 18) |
		    (rgb2rgb_coeff[1][0] << 9) | rgb2rgb_coeff[2][0];
		writel(param, base++);
		param = (rgb2rgb_coeff[3][1] >> 5);
		writel(param, base++);

		param =
		    (rgb2rgb_coeff[3][2] << 27) | (rgb2rgb_coeff[0][2] << 18) |
		    (rgb2rgb_coeff[1][2] << 9) | rgb2rgb_coeff[2][1];
		writel(param, base++);
		param = (rgb2rgb_coeff[3][2] >> 5);
		writel(param, base++);
	} else {
		dev_err(ipu->dev, "Unsupported color space conversion\n");
	}
}

static struct ipu_rgb def_rgb_32 = {
	.red	= { .offset = 16, .length = 8, },
	.green	= { .offset =  8, .length = 8, },
	.blue	= { .offset =  0, .length = 8, },
	.transp = { .offset = 24, .length = 8, },
	.bits_per_pixel = 32,
};

static irqreturn_t task_irq_handler(int irq, void *dcond)
{
	struct completion *comp = dcond;

	printk("%s\n", __func__);
	complete(comp);
	return IRQ_HANDLED;
}

int colorspace_conversion_task(struct ipu_soc *ipu, struct drm_imx_ipu_queue *args)
{
	struct ipu_ic_resource *res;
	struct ipu_channel *channel_in, *channel_out;
	uint32_t ipu_ic_conf, ipu_ic_idmac_1, ipu_ic_idmac_2, ipu_ic_idmac_3;
	ipu_color_space_t format_in, format_out;
	struct completion comp;
	int ret = 0;

	init_completion(&comp);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	res = &ipu_ic_resources[0];
	channel_in = ipu_idmac_get(ipu, res->ipu_channel_in);
	channel_out = ipu_idmac_get(ipu, res->ipu_channel_out);

	ret = request_threaded_irq(ipu->irq_start + channel_out->num, NULL, \
		task_irq_handler, 0, "task_irq", &comp);
	if (ret)
	    goto out;

	format_in = format_to_colorspace(args->format_in);
	format_out = format_to_colorspace(args->format_out);

	_init_csc(ipu, 0/*unused*/, format_in, format_out, 1);

	ipu_channel_set_resolution(channel_in, args->width, args->height);
	ipu_channel_set_stride(channel_in, args->width * bytes_per_pixel(args->format_in));
	ipu_channel_set_yuv_planar(channel_in, args->format_in,
		args->width * bytes_per_pixel(args->format_in),
		args->width, args->height);
	ipu_channel_set_buffer(channel_in, 0, args->phys_in);

	ipu_channel_set_resolution(channel_out, args->width, args->height);
	ipu_channel_set_stride(channel_out, args->width * bytes_per_pixel(args->format_out));
	ipu_channel_set_format_rgb(channel_out, &def_rgb_32);
	ipu_channel_set_buffer(channel_out, 0, args->phys_out);

	ipu_channel_set_burstsize(channel_in,16);
	ipu_channel_set_burstsize(channel_out,16);

        ipu_idmac_set_double_buffer(channel_in, false);
        ipu_idmac_set_double_buffer(channel_out, false);
	
	ipu_ic_idmac_1 = ipu_ic_read(ipu, IC_IDMAC_1);
	ipu_ic_idmac_2 = ipu_ic_read(ipu, IC_IDMAC_2);
	ipu_ic_idmac_3 = ipu_ic_read(ipu, IC_IDMAC_3);

	ipu_ic_idmac_2 &= ~IC_IDMAC_2_PP_HEIGHT_MASK;
	ipu_ic_idmac_2 |= (args->height - 1) << IC_IDMAC_2_PP_HEIGHT_OFFSET;

	ipu_ic_idmac_3 &= ~IC_IDMAC_3_PP_WIDTH_MASK;
	ipu_ic_idmac_3 |= (args->width- 1)  << IC_IDMAC_3_PP_WIDTH_OFFSET;

//	ipu_ic_idmac_1 &= ~IC_IDMAC_1_CB2_BURST_16;
//	ipu_ic_idmac_1 &= ~IC_IDMAC_1_CB5_BURST_16;

	ipu_ic_idmac_1 |= IC_IDMAC_1_CB2_BURST_16;
	ipu_ic_idmac_1 |= IC_IDMAC_1_CB5_BURST_16;

	ipu_ic_write(ipu, ipu_ic_idmac_1, IC_IDMAC_1);
	ipu_ic_write(ipu, ipu_ic_idmac_2, IC_IDMAC_2);
	ipu_ic_write(ipu, ipu_ic_idmac_3, IC_IDMAC_3);

	ipu_channel_set_high_priority(channel_in);
	ipu_channel_set_high_priority(channel_out);

	ipu_ic_conf = ipu_ic_read(ipu, IC_CONF);
	ipu_ic_conf |= IC_CONF_PP_EN;
	ipu_ic_conf |= IC_CONF_PP_CSC1;
	ipu_ic_conf &= ~IC_CONF_PP_CMB;
	ipu_ic_write(ipu, ipu_ic_conf, IC_CONF);

	ipu_idmac_enable_channel(channel_out);
	ipu_idmac_enable_channel(channel_in);

	ipu_module_enable(ipu, IPU_CONF_IC_EN);

	printk("ipu_idmac_select_buffer()\n");
	ipu_idmac_select_buffer(channel_out, 0);
	ipu_idmac_select_buffer(channel_in, 0);

	wait_for_completion_timeout(&comp, msecs_to_jiffies(100));

	printk("disable everything\n");

	free_irq(ipu->irq_start + channel_out->num, &comp);

	ipu_module_disable(ipu, IPU_CONF_IC_EN);

	ipu_idmac_disable_channel(channel_in);
	ipu_idmac_disable_channel(channel_out);

	ipu_ic_conf = ipu_ic_read(ipu, IC_CONF);
	ipu_ic_conf &= ~(IC_CONF_PP_EN | IC_CONF_PP_CSC1 | IC_CONF_PP_CSC2 |
		 IC_CONF_PP_CMB);
	ipu_ic_write(ipu, ipu_ic_conf, IC_CONF);
out:
	ipu_idmac_put(channel_in);
	ipu_idmac_put(channel_out);

	return ret;
}

int ipu_task_queue_ioctl(struct drm_device *drm, void *data,
		struct drm_file *file_priv)
{
	int ret = 0;

	struct drm_imx_ipu_queue *args = data;
	struct ipu_soc *ipu = dev_get_drvdata(drm->dev->parent);

	switch (args->task) {
		case IPU_TASK_CSC:
			ret = colorspace_conversion_task(ipu, args);
			break;
		default:
			ret = -ENOIOCTLCMD;
			break;
	}

	return ret;
}
