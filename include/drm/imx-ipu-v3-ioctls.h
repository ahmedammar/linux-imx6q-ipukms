/*
 * Userspace IPU access ioctls
 */

#include <drm/imx-ipu-v3.h>

enum drm_imx_task_type {
	IPU_TASK_CSC,
};

struct drm_imx_ipu_queue {
	enum drm_imx_task_type task;
	struct ipu_image input;
	struct ipu_image output;
	uint32_t *csc_coeffs[4];
};

#define DRM_IMX_IPU_QUEUE		0x00

#define DRM_IOCTL_IMX_IPU_QUEUE		DRM_IOW(DRM_COMMAND_BASE + \
		DRM_IMX_IPU_QUEUE, struct drm_imx_ipu_queue)
