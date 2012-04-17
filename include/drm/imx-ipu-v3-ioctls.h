/*
 * Userspace IPU access ioctls
 */

enum drm_imx_task_type {
	IPU_TASK_CSC,
};

struct drm_imx_ipu_queue {
	enum drm_imx_task_type task;
	unsigned int phys_in;
	unsigned int phys_out;
	unsigned int format_in;
	unsigned int format_out;
	unsigned int width;
	unsigned int height;
};

#define DRM_IMX_IPU_QUEUE		0x00

#define DRM_IOCTL_IMX_IPU_QUEUE		DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_IMX_IPU_QUEUE, struct drm_imx_ipu_queue)
