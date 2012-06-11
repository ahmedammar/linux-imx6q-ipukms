/*
 * Freescale i.MX drm driver
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/fb.h>
#include <asm/fb.h>
#include <linux/module.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/imx-ipu-v3-ioctls.h>

#include "imx-drm.h"
#include "ipu-v3/ipu-prv.h"

#define MAX_CRTC	4

struct imx_drm_device {
	struct drm_device			*drm;
	struct device				*dev;
	struct list_head			crtc_list;
	struct list_head			encoder_list;
	struct list_head			connector_list;
	struct mutex				mutex;
	int					references;
	int					pipes;
	struct drm_fbdev_cma			*fbhelper;
};

struct imx_drm_crtc {
	struct drm_crtc				*crtc;
	struct list_head			list;
	struct imx_drm_device			*imxdrm;
	int					pipe;
	struct drm_crtc_helper_funcs		crtc_helper_funcs;
	struct drm_crtc_funcs			crtc_funcs;
	struct imx_drm_crtc_helper_funcs	imx_drm_helper_funcs;
	struct module				*owner;
};

struct imx_drm_encoder {
	struct drm_encoder			*encoder;
	struct list_head			list;
	struct module				*owner;
};

struct imx_drm_connector {
	struct drm_connector			*connector;
	struct list_head			list;
	struct module				*owner;
};

static int imx_drm_driver_firstopen(struct drm_device *drm)
{
	if (!imx_drm_device_get())
		return -EINVAL;

	return 0;
}

static void imx_drm_driver_lastclose(struct drm_device *drm)
{
	struct imx_drm_device *imxdrm = drm->dev_private;

	if (imxdrm->fbhelper)
		drm_fbdev_cma_restore_mode(imxdrm->fbhelper);

	imx_drm_device_put();
}

static int imx_drm_driver_unload(struct drm_device *drm)
{
	struct imx_drm_device *imxdrm = drm->dev_private;

	drm_mode_config_cleanup(imxdrm->drm);
	drm_kms_helper_poll_fini(imxdrm->drm);

	return 0;
}

/*
 * We don't care at all for crtc numbers, but the core expects the
 * crtcs to be numbered
 */
static struct imx_drm_crtc *imx_drm_crtc_by_num(struct imx_drm_device *imxdrm,
		int num)
{
	struct imx_drm_crtc *imx_drm_crtc;

	list_for_each_entry(imx_drm_crtc, &imxdrm->crtc_list, list)
		if (imx_drm_crtc->pipe == num)
			return imx_drm_crtc;
	return NULL;
}

int imx_drm_crtc_panel_format(struct drm_crtc *crtc, u32 encoder_type,
		u32 interface_pix_fmt)
{
	struct imx_drm_device *imxdrm = crtc->dev->dev_private;
	struct imx_drm_crtc *imx_crtc;

	mutex_lock(&imxdrm->mutex);

	list_for_each_entry(imx_crtc, &imxdrm->crtc_list, list)
		if (imx_crtc->crtc == crtc)
			goto found;

	mutex_unlock(&imxdrm->mutex);

	return -EINVAL;
found:
	mutex_unlock(&imxdrm->mutex);

	if (imx_crtc->imx_drm_helper_funcs.set_interface_pix_fmt)
		return imx_crtc->imx_drm_helper_funcs.set_interface_pix_fmt(crtc,
				encoder_type, interface_pix_fmt);
	return 0;
}

int imx_drm_crtc_vblank_get(struct imx_drm_crtc *imx_drm_crtc)
{
	return drm_vblank_get(imx_drm_crtc->imxdrm->drm, imx_drm_crtc->pipe);
}
EXPORT_SYMBOL_GPL(imx_drm_crtc_vblank_get);

void imx_drm_crtc_vblank_put(struct imx_drm_crtc *imx_drm_crtc)
{
	drm_vblank_put(imx_drm_crtc->imxdrm->drm, imx_drm_crtc->pipe);
}
EXPORT_SYMBOL_GPL(imx_drm_crtc_vblank_put);

void imx_drm_handle_vblank(struct imx_drm_crtc *imx_drm_crtc)
{
	drm_handle_vblank(imx_drm_crtc->imxdrm->drm, imx_drm_crtc->pipe);
}
EXPORT_SYMBOL_GPL(imx_drm_handle_vblank);

static int imx_drm_enable_vblank(struct drm_device *drm, int crtc)
{
	struct imx_drm_device *imxdrm = drm->dev_private;
	struct imx_drm_crtc *imx_drm_crtc;
	int ret;

	imx_drm_crtc = imx_drm_crtc_by_num(imxdrm, crtc);
	if (!imx_drm_crtc)
		return -EINVAL;

	if (!imx_drm_crtc->imx_drm_helper_funcs.enable_vblank)
		return -ENOSYS;

	ret = imx_drm_crtc->imx_drm_helper_funcs.enable_vblank(imx_drm_crtc->crtc);
	return ret;
}

static void imx_drm_disable_vblank(struct drm_device *drm, int crtc)
{
	struct imx_drm_device *imxdrm = drm->dev_private;
	struct imx_drm_crtc *imx_drm_crtc;

	imx_drm_crtc = imx_drm_crtc_by_num(imxdrm, crtc);
	if (!imx_drm_crtc)
		return;

	if (!imx_drm_crtc->imx_drm_helper_funcs.disable_vblank)
		return;

	imx_drm_crtc->imx_drm_helper_funcs.disable_vblank(imx_drm_crtc->crtc);
}

static const struct file_operations imx_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_cma_mmap,
	.poll = drm_poll,
	.fasync = drm_fasync,
	.read = drm_read,
	.llseek = noop_llseek,
};

static struct imx_drm_device *imx_drm_device;

static struct imx_drm_device *__imx_drm_device(void)
{
	return imx_drm_device;
}

struct drm_device *imx_drm_device_get(void)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	struct imx_drm_encoder *enc;
	struct imx_drm_connector *con;
	struct imx_drm_crtc *crtc;

	mutex_lock(&imxdrm->mutex);

	list_for_each_entry(enc, &imxdrm->encoder_list, list) {
		if (!try_module_get(enc->owner)) {
			dev_err(imxdrm->dev, "could not get module %s\n",
					module_name(enc->owner));
			goto unwind_enc;
		}
	}

	list_for_each_entry(con, &imxdrm->connector_list, list) {
		if (!try_module_get(con->owner)) {
			dev_err(imxdrm->dev, "could not get module %s\n",
					module_name(con->owner));
			goto unwind_con;
		}
	}

	list_for_each_entry(crtc, &imxdrm->crtc_list, list) {
		if (!try_module_get(crtc->owner)) {
			dev_err(imxdrm->dev, "could not get module %s\n",
					module_name(crtc->owner));
			goto unwind_crtc;
		}
	}

	imxdrm->references++;

	mutex_unlock(&imxdrm->mutex);

	return imx_drm_device->drm;

unwind_crtc:
	list_for_each_entry_continue_reverse(crtc, &imxdrm->crtc_list, list)
		module_put(crtc->owner);
unwind_con:
	list_for_each_entry_continue_reverse(con, &imxdrm->connector_list, list)
		module_put(con->owner);
unwind_enc:
	list_for_each_entry_continue_reverse(enc, &imxdrm->encoder_list, list)
		module_put(enc->owner);

	mutex_unlock(&imxdrm->mutex);

	return NULL;

}
EXPORT_SYMBOL_GPL(imx_drm_device_get);

void imx_drm_device_put(void)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	struct imx_drm_encoder *enc;
	struct imx_drm_connector *con;
	struct imx_drm_crtc *crtc;

	mutex_lock(&imxdrm->mutex);

	list_for_each_entry(crtc, &imxdrm->crtc_list, list)
		module_put(crtc->owner);

	list_for_each_entry(con, &imxdrm->connector_list, list)
		module_put(con->owner);

	list_for_each_entry(enc, &imxdrm->encoder_list, list)
		module_put(enc->owner);

	imxdrm->references--;

	mutex_unlock(&imxdrm->mutex);
}
EXPORT_SYMBOL_GPL(imx_drm_device_put);

static int drm_mode_group_reinit(struct drm_device *dev)
{
	struct drm_mode_group *group = &dev->primary->mode_group;
	uint32_t *id_list = group->id_list;
	int ret;

	ret = drm_mode_group_init_legacy_group(dev, group);
	if (ret < 0)
		return ret;

	kfree(id_list);
	return 0;
}

/*
 * register an encoder to the drm core
 */
static int imx_drm_encoder_register(struct imx_drm_encoder *imx_drm_encoder)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();

	drm_encoder_init(imxdrm->drm, imx_drm_encoder->encoder,
			imx_drm_encoder->encoder->funcs,
			imx_drm_encoder->encoder->encoder_type);

	drm_mode_group_reinit(imxdrm->drm);

	return 0;
}

/*
 * unregister an encoder from the drm core
 */
static void imx_drm_encoder_unregister(struct imx_drm_encoder *imx_drm_encoder)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();

	drm_encoder_cleanup(imx_drm_encoder->encoder);

	drm_mode_group_reinit(imxdrm->drm);
}

/*
 * register a connector to the drm core
 */
static int imx_drm_connector_register(struct imx_drm_connector *imx_drm_connector)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	int ret;

	drm_connector_init(imxdrm->drm, imx_drm_connector->connector,
			imx_drm_connector->connector->funcs,
			imx_drm_connector->connector->connector_type);
	drm_mode_group_reinit(imxdrm->drm);
	ret = drm_sysfs_connector_add(imx_drm_connector->connector);
	if (ret)
		goto err;

	return 0;
err:

	return ret;
}

/*
 * unregister a connector from the drm core
 */
static void imx_drm_connector_unregister(struct imx_drm_connector *imx_drm_connector)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();

	drm_sysfs_connector_remove(imx_drm_connector->connector);
	drm_connector_cleanup(imx_drm_connector->connector);

	drm_mode_group_reinit(imxdrm->drm);
}

/*
 * register a crtc to the drm core
 */
static int imx_drm_crtc_register(struct imx_drm_crtc *imx_drm_crtc)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	int ret;

	drm_crtc_init(imxdrm->drm, imx_drm_crtc->crtc, &imx_drm_crtc->crtc_funcs);
	ret = drm_mode_crtc_set_gamma_size(imx_drm_crtc->crtc, 256);
	if (ret)
		return ret;

	drm_crtc_helper_add(imx_drm_crtc->crtc, &imx_drm_crtc->crtc_helper_funcs);

	drm_mode_group_reinit(imxdrm->drm);

	return 0;
}

/*
 * Called by the CRTC driver when all CRTCs are registered. This
 * puts all the pieces together and initializes the driver.
 * Once this is called no more CRTCs can be registered since
 * the drm core has hardcoded the number of crtcs in several
 * places.
 */
static int imx_drm_driver_load(struct drm_device *drm, unsigned long flags)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	int ret;

	imxdrm->drm = drm;

	drm->dev_private = imxdrm;

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = 1, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *      just specific driver own one instead because
	 *      drm framework supports only one irq handler and
	 *      drivers can well take care of their interrupts
	 */
	drm->irq_enabled = 1;

	drm_mode_config_init(drm);
	imx_drm_mode_config_init(drm);

	mutex_lock(&imxdrm->mutex);

	drm_kms_helper_poll_init(imxdrm->drm);

	/* setup the grouping for the legacy output */
	ret = drm_mode_group_init_legacy_group(imxdrm->drm,
			&imxdrm->drm->primary->mode_group);
	if (ret)
		goto err_init;

	ret = drm_vblank_init(imxdrm->drm, MAX_CRTC);
	if (ret)
		goto err_init;

	/*
	 * with vblank_disable_allowed = 1, vblank interrupt will be disabled
	 * by drm timer once a current process gives up ownership of
	 * vblank event.(after drm_vblank_put function is called)
	 */
	imxdrm->drm->vblank_disable_allowed = 1;

	ret = 0;

err_init:
	mutex_unlock(&imxdrm->mutex);

	return ret;
}

/*
 * imx_drm_add_crtc - add a new crtc
 *
 * The return value if !NULL is a cookie for the caller to pass to
 * imx_drm_remove_crtc later.
 */
int imx_drm_add_crtc(struct drm_crtc *crtc,
		struct imx_drm_crtc **new_crtc,
		const struct drm_crtc_funcs *crtc_funcs,
		const struct drm_crtc_helper_funcs *crtc_helper_funcs,
		const struct imx_drm_crtc_helper_funcs *imx_drm_helper_funcs,
		struct module *owner)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	struct imx_drm_crtc *imx_drm_crtc;
	int ret;

	mutex_lock(&imxdrm->mutex);

	if (imxdrm->references) {
		ret = -EBUSY;
		goto err_busy;
	}

	imx_drm_crtc = kzalloc(sizeof(*imx_drm_crtc), GFP_KERNEL);
	if (!imx_drm_crtc) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	imx_drm_crtc->crtc_funcs = *crtc_funcs;
	imx_drm_crtc->crtc_helper_funcs = *crtc_helper_funcs;
	imx_drm_crtc->imx_drm_helper_funcs = *imx_drm_helper_funcs;
	imx_drm_crtc->pipe = imxdrm->pipes++;

	WARN_ON(crtc_funcs->set_config);
	WARN_ON(crtc_funcs->destroy);

	imx_drm_crtc->crtc_funcs.set_config = drm_crtc_helper_set_config;
	imx_drm_crtc->crtc_funcs.destroy = drm_crtc_cleanup;

	imx_drm_crtc->crtc = crtc;
	imx_drm_crtc->imxdrm = imxdrm;

	imx_drm_crtc->owner = owner;

	list_add_tail(&imx_drm_crtc->list, &imxdrm->crtc_list);

	*new_crtc = imx_drm_crtc;

	ret = imx_drm_crtc_register(imx_drm_crtc);
	if (ret)
		goto err_register;

	mutex_unlock(&imxdrm->mutex);

	return 0;

err_register:
	kfree(imx_drm_crtc);
err_alloc:
err_busy:
	mutex_unlock(&imxdrm->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(imx_drm_add_crtc);

/*
 * imx_drm_remove_crtc - remove a crtc
 */
int imx_drm_remove_crtc(struct imx_drm_crtc *imx_drm_crtc)
{
	struct imx_drm_device *imxdrm = imx_drm_crtc->imxdrm;

	mutex_lock(&imxdrm->mutex);

	drm_crtc_cleanup(imx_drm_crtc->crtc);

	list_del(&imx_drm_crtc->list);

	drm_mode_group_reinit(imxdrm->drm);

	mutex_unlock(&imxdrm->mutex);

	kfree(imx_drm_crtc);

	return 0;
}
EXPORT_SYMBOL_GPL(imx_drm_remove_crtc);

/*
 * imx_drm_add_encoder - add a new encoder
 */
int imx_drm_add_encoder(struct drm_encoder *encoder,
		struct imx_drm_encoder **newenc, struct module *owner)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	struct imx_drm_encoder *imx_drm_encoder;
	int ret;

	mutex_lock(&imxdrm->mutex);

	if (imxdrm->references) {
		ret = -EBUSY;
		goto err_busy;
	}

	imx_drm_encoder = kzalloc(sizeof(struct imx_drm_encoder), GFP_KERNEL);
	if (!imx_drm_encoder) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	imx_drm_encoder->encoder = encoder;
	imx_drm_encoder->owner = owner;

	ret = imx_drm_encoder_register(imx_drm_encoder);
	if (ret) {
		kfree(imx_drm_encoder);
		ret = -ENOMEM;
		goto err_register;
	}

	list_add_tail(&imx_drm_encoder->list, &imxdrm->encoder_list);

	*newenc = imx_drm_encoder;

	mutex_unlock(&imxdrm->mutex);

	return 0;

err_register:
	kfree(imx_drm_encoder);
err_alloc:
err_busy:
	mutex_unlock(&imxdrm->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(imx_drm_add_encoder);

/*
 * imx_drm_remove_encoder - remove an encoder
 */
int imx_drm_remove_encoder(struct imx_drm_encoder *imx_drm_encoder)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();

	mutex_lock(&imxdrm->mutex);

	imx_drm_encoder_unregister(imx_drm_encoder);

	list_del(&imx_drm_encoder->list);

	mutex_unlock(&imxdrm->mutex);

	kfree(imx_drm_encoder);

	return 0;
}
EXPORT_SYMBOL_GPL(imx_drm_remove_encoder);

/*
 * imx_drm_add_connector - add a connector
 */
int imx_drm_add_connector(struct drm_connector *connector,
		struct imx_drm_connector **new_con,
		struct module *owner)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();
	struct imx_drm_connector *imx_drm_connector;
	int ret;

	mutex_lock(&imxdrm->mutex);

	if (imxdrm->references) {
		ret = -EBUSY;
		goto err_busy;
	}

	imx_drm_connector = kzalloc(sizeof(struct imx_drm_connector), GFP_KERNEL);
	if (!imx_drm_connector) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	imx_drm_connector->connector = connector;
	imx_drm_connector->owner = owner;

	ret = imx_drm_connector_register(imx_drm_connector);
	if (ret)
		goto err_register;

	list_add_tail(&imx_drm_connector->list, &imxdrm->connector_list);

	*new_con = imx_drm_connector;

	mutex_unlock(&imxdrm->mutex);

	return 0;

err_register:
	kfree(imx_drm_connector);
err_alloc:
err_busy:
	mutex_unlock(&imxdrm->mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(imx_drm_add_connector);

void imx_drm_fb_helper_set(struct drm_fbdev_cma *fbdev_helper)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();

	imxdrm->fbhelper = fbdev_helper;
}
EXPORT_SYMBOL_GPL(imx_drm_fb_helper_set);

/*
 * imx_drm_remove_connector - remove a connector
 */
int imx_drm_remove_connector(struct imx_drm_connector *imx_drm_connector)
{
	struct imx_drm_device *imxdrm = __imx_drm_device();

	mutex_lock(&imxdrm->mutex);

	imx_drm_connector_unregister(imx_drm_connector);

	list_del(&imx_drm_connector->list);

	mutex_unlock(&imxdrm->mutex);

	kfree(imx_drm_connector);

	return 0;
}
EXPORT_SYMBOL_GPL(imx_drm_remove_connector);

static struct drm_ioctl_desc imx_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(IMX_IPU_QUEUE, ipu_task_queue_ioctl, DRM_AUTH),
};

static struct drm_driver imx_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM,
	.load			= imx_drm_driver_load,
	.unload			= imx_drm_driver_unload,
	.firstopen		= imx_drm_driver_firstopen,
	.lastclose		= imx_drm_driver_lastclose,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_cma_dumb_destroy,

	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= imx_drm_enable_vblank,
	.disable_vblank		= imx_drm_disable_vblank,
	.reclaim_buffers	= drm_core_reclaim_buffers,
	.ioctls			= imx_drm_ioctls,
	.num_ioctls		= ARRAY_SIZE(imx_drm_ioctls),
	.fops			= &imx_drm_driver_fops,
	.name			= "imx-drm",
	.desc			= "i.MX DRM graphics",
	.date			= "20120507",
	.major			= 1,
	.minor			= 0,
	.patchlevel		= 0,
};

static int imx_drm_platform_probe(struct platform_device *pdev)
{
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	imx_drm_device->dev = &pdev->dev;

	return drm_platform_init(&imx_drm_driver, pdev);
}

static int imx_drm_platform_remove(struct platform_device *pdev)
{
	drm_platform_exit(&imx_drm_driver, pdev);

	return 0;
}

static struct platform_driver imx_drm_pdrv = {
	.probe		= imx_drm_platform_probe,
	.remove		= __devexit_p(imx_drm_platform_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "imx-drm",
	},
};


static int __init imx_drm_init(void)
{
	int ret;

	imx_drm_device = kzalloc(sizeof(*imx_drm_device), GFP_KERNEL);
	if (!imx_drm_device)
		return -ENOMEM;

	mutex_init(&imx_drm_device->mutex);
	INIT_LIST_HEAD(&imx_drm_device->crtc_list);
	INIT_LIST_HEAD(&imx_drm_device->connector_list);
	INIT_LIST_HEAD(&imx_drm_device->encoder_list);

	ret = platform_driver_register(&imx_drm_pdrv);
	if (ret)
		return ret;

	return 0;

}

static void __exit imx_drm_exit(void)
{
	DRM_DEBUG_DRIVER("%s\n", __FILE__);

	platform_driver_unregister(&imx_drm_pdrv);

	kfree(imx_drm_device);
}

module_init(imx_drm_init);
module_exit(imx_drm_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("i.MX drm driver core");
MODULE_LICENSE("GPL");
