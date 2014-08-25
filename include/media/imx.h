struct v4l2_subdev;
struct device_node;
struct ipu_media_link;
struct v4l2_device;
struct media_device;
struct device;

struct ipu_media_link *ipu_media_entity_create_link(struct v4l2_subdev *sd,
		int padno, struct device_node *remote_node,
		u32 media_link_flags);

void ipu_media_entity_remove_link(struct ipu_media_link *link);

struct v4l2_device *ipu_media_get_v4l2_dev(void);

struct media_device *ipu_find_media_device(void);

#ifdef CONFIG_MEDIA_IMX
int ipu_media_device_register(struct device *dev);
int ipu_media_device_unregister(struct device *dev);
#else
static inline int ipu_media_device_register(struct device *dev)
{
	return -EINVAL;
}

static inline int ipu_media_device_unregister(struct device *dev)
{
	return -EINVAL;
}
#endif
