
#ifndef __AP_HAL_LINUX_IIO_H__
#define __AP_HAL_LINUX_IIO_H__

#include <stdint.h>
#include <sys/types.h>
#include <stddef.h>

namespace IIO {

    class Context {
    public:

    private:

    };

}

struct iio_data_format {
    /** @brief Total length of the sample, in bits */
    unsigned int length;

    /** @brief Length of valuable data in the sample, in bits */
    unsigned int bits;

    /** @brief Right-shift to apply when converting sample */
    unsigned int shift;

    /** @brief Contains True if the sample is signed */
    bool is_signed;

    /** @brief Contains True if the sample is fully defined, sign extended, etc. */
    bool is_fully_defined;

    /** @brief Contains True if the sample is in big-endian format */
    bool is_be;

    /** @brief Contains True if the sample should be scaled when converted */
    bool with_scale;

    /** @brief Contains the scale to apply if with_scale is set */
    double scale;
};
struct iio_device_pdata {
    int fd;
    bool blocking;
    unsigned int samples_count, nb_blocks;

    struct block *blocks;
    void **addrs;
    int last_dequeued;
    bool is_high_speed, cyclic, cyclic_buffer_enqueued, buffer_enabled;
};

struct iio_channel_attr {
    char *name;
    char *filename;
};


struct iio_channel {
    struct iio_device *dev;
    struct iio_channel_pdata *pdata;
    void *userdata;

    bool is_output;
    bool is_scan_element;
    struct iio_data_format format;
    char *name, *id;
    long index;

    struct iio_channel_attr *attrs;
    unsigned int nb_attrs;
};


struct iio_device {
    const struct iio_context *ctx;
    struct iio_device_pdata *pdata;
    void *userdata;

    char *name, *id;

    char **attrs;
    unsigned int nb_attrs;

    char **debug_attrs;
    unsigned int nb_debug_attrs;

    struct iio_channel **channels;
    unsigned int nb_channels;

    uint32_t *mask;
    size_t words;
};

struct iio_context {
    struct iio_context_pdata *pdata;
    const char *name;
    char *description;

    struct iio_device **devices;
    unsigned int nb_devices;

    unsigned int rw_timeout_ms;
};

struct iio_buffer {
    const struct iio_device *dev;
    void *buffer, *userdata;
    size_t length, data_length;

    uint32_t *mask;
    unsigned int dev_sample_size;
    unsigned int sample_size;
    bool is_output, dev_is_high_speed;
};

struct block_alloc_req {
    uint32_t type,
         size,
         count,
         id;
};

struct block {
    uint32_t id,
         size,
         bytes_used,
         type,
         flags,
         offset;
    uint64_t timestamp;
};

struct iio_context* iio_create_local_context(void);
struct iio_channel * iio_device_find_channel(const struct iio_device *dev,
        const char *name, bool output);
void iio_channel_enable(struct iio_channel *chn);
int iio_device_set_kernel_buffers_count(const struct iio_device *dev,
        unsigned int nb_buffers);
void iio_buffer_destroy(struct iio_buffer *buffer);
ssize_t iio_device_attr_write(const struct iio_device *dev,
        const char *attr, const char *src);
struct iio_device * iio_context_find_device(const struct iio_context *ctx,
        const char *name);
struct iio_buffer * iio_device_create_buffer(const struct iio_device *dev,
        size_t samples_count, bool cyclic);
void iio_context_destroy(struct iio_context *ctx);
ssize_t iio_buffer_refill(struct iio_buffer *buffer);
ptrdiff_t iio_buffer_step(const struct iio_buffer *buffer);
void * iio_buffer_end(const struct iio_buffer *buffer);
void * iio_buffer_first(const struct iio_buffer *buffer,
        const struct iio_channel *chn);
void iio_channel_convert(const struct iio_channel *chn,
        void *dst, const void *src);


#endif /* __AP_HAL_LINUX_IIO_H__ */
