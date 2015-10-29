#include "IIO.h"

//#include "iio-private.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#define DEFAULT_TIMEOUT_MS 1000
#define NB_BLOCKS 4

#define ARRAY_SIZE(x) (sizeof(x) ? sizeof(x) / sizeof((x)[0]) : 0)
#define BIT(x) (1 << (x))
#define BIT_MASK(bit) BIT((bit) % 32)
#define BIT_WORD(bit) ((bit) / 32)
#define SET_BIT(addr, bit) \
    *(((uint32_t *) addr) + BIT_WORD(bit)) |= BIT_MASK(bit)
#define TEST_BIT(addr, bit) (!!(*(((uint32_t *) addr) + BIT_WORD(bit)) \
        & BIT_MASK(bit)))

#  define ERROR(...) \
    fprintf(stderr, "ERROR: " __VA_ARGS__)
#  define WARNING(...) \
    fprintf(stderr, "WARNING: " __VA_ARGS__)
#  define DEBUG(...) \
    fprintf(stdout, "DEBUG: " __VA_ARGS__)


#define BLOCK_ALLOC_IOCTL   _IOWR('i', 0xa0, struct block_alloc_req)
#define BLOCK_FREE_IOCTL      _IO('i', 0xa1)
#define BLOCK_QUERY_IOCTL   _IOWR('i', 0xa2, struct block)
#define BLOCK_ENQUEUE_IOCTL _IOWR('i', 0xa3, struct block)
#define BLOCK_DEQUEUE_IOCTL _IOWR('i', 0xa4, struct block)

#define BLOCK_FLAG_CYCLIC BIT(1)


static const char * const device_attrs_blacklist[] = {
    "dev",
    "uevent",
};

enum iio_modifier {
    IIO_NO_MOD,
    IIO_MOD_X,
    IIO_MOD_Y,
    IIO_MOD_Z,
    IIO_MOD_LIGHT_BOTH,
    IIO_MOD_LIGHT_IR,
    IIO_MOD_ROOT_SUM_SQUARED_X_Y,
    IIO_MOD_SUM_SQUARED_X_Y_Z,
    IIO_MOD_LIGHT_CLEAR,
    IIO_MOD_LIGHT_RED,
    IIO_MOD_LIGHT_GREEN,
    IIO_MOD_LIGHT_BLUE,
    IIO_MOD_I,
    IIO_MOD_Q,
};

static const char * const modifier_names[] = {
        /* See if it should be initialized */
};

static ssize_t local_write_dev_attr(const struct iio_device *dev,
        const char *attr, const char *src, size_t len, bool is_debug);
static ssize_t iio_device_attr_read(const struct iio_device *dev,
        const char *attr, char *dst, size_t len, bool is_debug);

static int set_timeout(struct iio_context *ctx, unsigned int timeout)
{
    ctx->rw_timeout_ms = timeout;
    return 0;
}

void iio_strerror(int err, char *buf, size_t len)
{
    int ret = (int) strerror_r(err, buf, len);
    if (ret != 0)
        snprintf(buf, len, "Unknown error %i", err);
}

static int foreach_in_dir(void *d, const char *path, bool is_dir,
        int (*callback)(void *, const char *));


static void local_free_pdata(struct iio_device *device)
{
    if (device && device->pdata) {
        free(device->pdata->blocks);
        free(device->pdata->addrs);
        free(device->pdata);
    }
}


static ssize_t iio_read_all_dev_attrs(const struct iio_device *dev,
        char *dst, size_t len, bool is_debug)
{
    unsigned int i, nb = is_debug ? dev->nb_debug_attrs : dev->nb_attrs;
    char **attrs = is_debug ? dev->debug_attrs : dev->attrs;
    char *ptr = dst;

    for (i = 0; len >= 4 && i < nb; i++) {
        /* Recursive! */
        ssize_t ret = iio_device_attr_read(dev, attrs[i],
                ptr + 4, len - 4, is_debug);
        *(uint32_t *) ptr = htonl(ret);

        /* Align the length to 4 bytes */
        if (ret > 0 && ret & 3)
            ret = ((ret >> 2) + 1) << 2;
        ptr += 4 + (ret < 0 ? 0 : ret);
        len -= 4 + (ret < 0 ? 0 : ret);
    }

    return ptr - dst;
}
static ssize_t iio_device_attr_read(const struct iio_device *dev,
        const char *attr, char *dst, size_t len, bool is_debug)
{
    FILE *f;
    char buf[1024];
    ssize_t ret;

    if (!attr)
        return iio_read_all_dev_attrs(dev, dst, len, is_debug);

    if (is_debug)
        snprintf(buf, sizeof(buf), "/sys/kernel/debug/iio/%s/%s",
                dev->id, attr);
    else
        snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/%s/%s",
                dev->id, attr);
    f = fopen(buf, "r");
    if (!f)
        return -errno;

    ret = fread(dst, 1, len, f);
    if (ret > 0)
        dst[ret - 1] = '\0';
    fflush(f);
    if (ferror(f))
        ret = -errno;
    fclose(f);
    return ret ? ret : -EIO;
}

static int read_device_name(struct iio_device *dev)
{
    char buf[1024];
    ssize_t ret = iio_device_attr_read(dev, "name", buf, sizeof(buf), false);
    if (ret < 0)
        return ret;
    else if (ret == 0)
        return -EIO;

    dev->name = strdup(buf);
    if (!dev->name)
        return -ENOMEM;
    else
        return 0;
}

static int add_attr_to_device(struct iio_device *dev, const char *attr)
{
    char **attrs, *name;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(device_attrs_blacklist); i++)
        if (!strcmp(device_attrs_blacklist[i], attr))
            return 0;

    if (!strcmp(attr, "name"))
        return read_device_name(dev);

    name = strdup(attr);
    if (!name)
        return -ENOMEM;

    attrs = (char**) realloc(dev->attrs, (1 + dev->nb_attrs) * sizeof(char *));
    if (!attrs) {
        free(name);
        return -ENOMEM;
    }

    attrs[dev->nb_attrs++] = name;
    dev->attrs = attrs;
    DEBUG("Added attr \'%s\' to device \'%s\'\n", attr, dev->id);
    return 0;
}


/*
 * Looks for a IIO channel modifier at the beginning of the string s. If a
 * modifier was found the symbolic constant (IIO_MOD_*) is returned, otherwise
 * IIO_NO_MOD is returned. If a modifier was found len_p will be update with the
 * length of the modifier.
 */
static unsigned int find_modifier(const char *s, size_t *len_p)
{
    unsigned int i;
    size_t len;

    for (i = 0; i < ARRAY_SIZE(modifier_names); i++) {
        if (!modifier_names[i])
            continue;
        len = strlen(modifier_names[i]);
        if (strncmp(s, modifier_names[i], len) == 0 && s[len] == '_') {
            if (len_p)
                *len_p = len;
            return i;
        }
    }

    return IIO_NO_MOD;
}


static bool is_channel(const char *attr)
{
    char *ptr = NULL;
    if (!strncmp(attr, "in_timestamp_", sizeof("in_timestamp_") - 1))
        return true;
    if (!strncmp(attr, "in_", 3))
        ptr = (char*) strchr(attr + 3, '_');
    else if (!strncmp(attr, "out_", 4))
        ptr = (char*) strchr(attr + 4, '_');
    if (!ptr)
        return false;
    if (*(ptr - 1) >= '0' && *(ptr - 1) <= '9')
        return true;

    if (find_modifier(ptr + 1, NULL) != IIO_NO_MOD)
        return true;
    return false;
}

static char * get_short_attr_name(struct iio_channel *chn, const char *attr)
{
    char *ptr = (char*) strchr(attr, '_') + 1;
    size_t len;

    ptr = (char*) strchr(ptr, '_') + 1;
    if (find_modifier(ptr, &len) != IIO_NO_MOD)
        ptr += len + 1;

    if (chn->name) {
        size_t len = strlen(chn->name);
        if  (strncmp(chn->name, ptr, len) == 0 && ptr[len] == '_')
            ptr += len + 1;
    }

    return strdup(ptr);
}

static char * get_channel_id(const char *attr)
{
    char *res, *ptr;
    size_t len;

    attr = strchr(attr, '_') + 1;
    ptr = (char*) strchr(attr, '_');
    if (find_modifier(ptr + 1, &len) != IIO_NO_MOD)
        ptr += len + 1;

    res = (char*) malloc(ptr - attr + 1);
    if (!res)
        return NULL;

    memcpy(res, attr, ptr - attr);
    res[ptr - attr] = 0;
    return res;
}

static int add_attr_to_channel(struct iio_channel *chn,
        const char *attr, const char *path)
{
    struct iio_channel_attr *attrs;
    char *fn, *name = get_short_attr_name(chn, attr);
    if (!name)
        return -ENOMEM;

    fn = strdup(path);
    if (!fn)
        goto err_free_name;

    attrs = (struct iio_channel_attr*) realloc(chn->attrs, (1 + chn->nb_attrs) *
            sizeof(struct iio_channel_attr));
    if (!attrs)
        goto err_free_fn;

    attrs[chn->nb_attrs].filename = fn;
    attrs[chn->nb_attrs++].name = name;
    chn->attrs = attrs;
    DEBUG("Added attr \'%s\' to channel \'%s\'\n", name, chn->id);
    return 0;

err_free_fn:
    free(fn);
err_free_name:
    free(name);
    return -ENOMEM;
}

static struct iio_channel *create_channel(struct iio_device *dev,
        char *id, const char *attr, const char *path)
{
    struct iio_channel *chn = (struct iio_channel*) calloc(1, sizeof(*chn));
    if (!chn)
        return NULL;

    if (!strncmp(attr, "out_", 4))
        chn->is_output = true;
    else if (strncmp(attr, "in_", 3))
        goto err_free_chn;

    chn->dev = dev;
    chn->id = id;

    if (!add_attr_to_channel(chn, attr, path))
        return chn;

err_free_chn:
    free(chn);
    return NULL;
}

static int add_channel_to_device(struct iio_device *dev,
        struct iio_channel *chn)
{
    struct iio_channel **channels = (struct iio_channel **) realloc(dev->channels,
            (dev->nb_channels + 1) * sizeof(struct iio_channel *));
    if (!channels)
        return -ENOMEM;

    channels[dev->nb_channels++] = chn;
    dev->channels = channels;
    DEBUG("Added channel \'%s\' to device \'%s\'\n", chn->id, dev->id);
    return 0;
}

void free_channel(struct iio_channel *chn)
{
    size_t i;
    for (i = 0; i < chn->nb_attrs; i++) {
        free(chn->attrs[i].name);
        free(chn->attrs[i].filename);
    }
    if (chn->nb_attrs)
        free(chn->attrs);
    if (chn->name)
        free(chn->name);
    if (chn->id)
        free(chn->id);
    free(chn);
}

static int add_device_to_context(struct iio_context *ctx,
        struct iio_device *dev)
{
    struct iio_device **devices = (struct iio_device **) realloc(ctx->devices,
            (ctx->nb_devices + 1) * sizeof(struct iio_device *));
    if (!devices)
        return -ENOMEM;

    devices[ctx->nb_devices++] = dev;
    ctx->devices = devices;
    DEBUG("Added device \'%s\' to context \'%s\'\n", dev->id, ctx->name);
    return 0;
}

static int add_attr_or_channel_helper(struct iio_device *dev,
        const char *path, bool dir_is_scan_elements)
{
    int ret;
    unsigned int i;
    struct iio_channel *chn;
    char buf[1024], *channel_id;
    const char *name = strrchr(path, '/') + 1;

    if (dir_is_scan_elements) {
        snprintf(buf, sizeof(buf), "scan_elements/%s", name);
        path = buf;
    } else {
        path = name;
    }

    if (!is_channel(name))
        return add_attr_to_device(dev, name);

    channel_id = get_channel_id(name);
    if (!channel_id)
        return -ENOMEM;

    for (i = 0; i < dev->nb_channels; i++) {
        chn = dev->channels[i];
        if (!strcmp(chn->id, channel_id)
                && chn->is_output == (name[0] == 'o')) {
            free(channel_id);
            ret = add_attr_to_channel(chn, name, path);
            chn->is_scan_element = dir_is_scan_elements && !ret;
            return ret;
        }
    }

    chn = create_channel(dev, channel_id, name, path);
    if (!chn) {
        free(channel_id);
        return -ENXIO;
    }
    ret = add_channel_to_device(dev, chn);
    if (ret)
        free_channel(chn);
    else
        chn->is_scan_element = dir_is_scan_elements;
    return ret;
}

static int add_attr_or_channel(void *d, const char *path)
{
    return add_attr_or_channel_helper((struct iio_device *) d, path, false);
}

static int add_scan_element(void *d, const char *path)
{
    return add_attr_or_channel_helper((struct iio_device *) d, path, true);
}

static int add_scan_elements(struct iio_device *dev, const char *devpath)
{
    struct stat st;
    char buf[1024];
    snprintf(buf, sizeof(buf), "%s/scan_elements", devpath);

    if (!stat(buf, &st) && S_ISDIR(st.st_mode)) {
        int ret = foreach_in_dir(dev, buf, false, add_scan_element);
        if (ret < 0)
            return ret;
    }

    return 0;
}

void free_device(struct iio_device *dev)
{
    unsigned int i;
    for (i = 0; i < dev->nb_attrs; i++)
        free(dev->attrs[i]);
    if (dev->nb_attrs)
        free(dev->attrs);
    for (i = 0; i < dev->nb_debug_attrs; i++)
        free(dev->debug_attrs[i]);
    if (dev->nb_debug_attrs)
        free(dev->debug_attrs);
    for (i = 0; i < dev->nb_channels; i++)
        free_channel(dev->channels[i]);
    if (dev->nb_channels)
        free(dev->channels);
    if (dev->mask)
        free(dev->mask);
    if (dev->name)
        free(dev->name);
    if (dev->id)
        free(dev->id);
    free(dev);
}

/** Shrinks the first nb characters of a string
 * e.g. strcut("foobar", 4) replaces the content with "ar". */
static void strcut(char *str, int nb)
{
    char *ptr = str + nb;
    while (*ptr)
        *str++ = *ptr++;
    *str = 0;
}

static int set_channel_name(struct iio_channel *chn)
{
    size_t prefix_len = 0;
    const char *attr0;
    const char *ptr;
    unsigned int i;

    if (chn->nb_attrs < 2)
        return 0;

    attr0 = ptr = chn->attrs[0].name;

    while (true) {
        bool can_fix = true;
        size_t len;

        ptr = strchr(ptr, '_');
        if (!ptr)
            break;

        len = ptr - attr0;
        for (i = 1; can_fix && i < chn->nb_attrs; i++)
            can_fix = !strncmp(attr0, chn->attrs[i].name, len);

        if (!can_fix)
            break;

        prefix_len = len;
        ptr = ptr + 1;
    }

    if (prefix_len) {
        char *name;

        name = (char*) malloc(prefix_len + 1);
        if (!name)
            return -ENOMEM;
        strncpy(name, attr0, prefix_len);
        name[prefix_len] = '\0';
        DEBUG("Setting name of channel %s to %s\n", chn->id, name);
        chn->name = name;

        /* Shrink the attribute name */
        for (i = 0; i < chn->nb_attrs; i++)
            strcut(chn->attrs[i].name, prefix_len + 1);
    }

    return 0;
}


/*
 * Possible return values:
 * 0 = Attribute should not be moved to the channel
 * 1 = Attribute should be moved to the channel and it is a shared attribute
 * 2 = Attribute should be moved to the channel and it is a private attribute
 */
static unsigned int is_global_attr(struct iio_channel *chn, const char *attr)
{
    unsigned int len;
    char *ptr;

    if (!chn->is_output && !strncmp(attr, "in_", 3))
        attr += 3;
    else if (chn->is_output && !strncmp(attr, "out_", 4))
        attr += 4;
    else
        return 0;

    ptr = (char*) strchr(attr, '_');
    if (!ptr)
        return 0;

    len = ptr - attr;

    if (strncmp(chn->id, attr, len))
        return 0;

    DEBUG("Found match: %s and %s\n", chn->id, attr);
    if (chn->id[len] >= '0' && chn->id[len] <= '9') {
        if (chn->name) {
            size_t name_len = strlen(chn->name);
            if (strncmp(chn->name, attr + len + 1, name_len) == 0 &&
                attr[len + 1 + name_len] == '_')
                return 2;
        }
        return 1;
    } else if (chn->id[len] != '_') {
        return 0;
    }

    if (find_modifier(chn->id + len + 1, NULL) != IIO_NO_MOD)
        return 1;

    return 0;
}

static int detect_global_attr(struct iio_device *dev, const char *attr,
    unsigned int level, bool *match)
{
    unsigned int i;

    *match = false;
    for (i = 0; i < dev->nb_channels; i++) {
        struct iio_channel *chn = dev->channels[i];
        if (is_global_attr(chn, attr) == level) {
            int ret;
            *match = true;
            ret = add_attr_to_channel(chn, attr, attr);
            if (ret)
                return ret;
        }
    }

    return 0;
}

static int detect_and_move_global_attrs(struct iio_device *dev)
{
    unsigned int i;
    char **ptr = dev->attrs;

    for (i = 0; i < dev->nb_attrs; i++) {
        const char *attr = dev->attrs[i];
        bool match;
        int ret;

        ret = detect_global_attr(dev, attr, 2, &match);
        if (ret)
            return ret;

        if (!match) {
            ret = detect_global_attr(dev, attr, 1, &match);
            if (ret)
                return ret;
        }

        if (match) {
            free(dev->attrs[i]);
            dev->attrs[i] = NULL;
        }
    }

    for (i = 0; i < dev->nb_attrs; i++) {
        if (dev->attrs[i])
            *ptr++ = dev->attrs[i];
    }

    dev->nb_attrs = ptr - dev->attrs;
    return 0;
}

static int create_device(void *d, const char *path)
{
    uint32_t *mask = NULL;
    unsigned int i;
    int ret;
    struct iio_context *ctx = (struct iio_context *) d;
    struct iio_device *dev = (struct iio_device *) calloc(1, sizeof(*dev));
    if (!dev)
        return -ENOMEM;

    dev->pdata = (struct iio_device_pdata*) calloc(1, sizeof(*dev->pdata));
    if (!dev->pdata) {
        free(dev);
        return -ENOMEM;
    }

    dev->pdata->fd = -1;
    dev->pdata->blocking = true;
    dev->pdata->nb_blocks = NB_BLOCKS;

    dev->ctx = ctx;
    dev->id = strdup(strrchr(path, '/') + 1);
    if (!dev->id) {
        local_free_pdata(dev);
        free(dev);
        return -ENOMEM;
    }

    ret = foreach_in_dir(dev, path, false, add_attr_or_channel);
    if (ret < 0) {
        free_device(dev);
        return ret;
    }

    ret = add_scan_elements(dev, path);
    if (ret < 0) {
        free_device(dev);
        return ret;
    }

    for (i = 0; i < dev->nb_channels; i++)
        set_channel_name(dev->channels[i]);

    ret = detect_and_move_global_attrs(dev);
    if (ret < 0) {
        free_device(dev);
        return ret;
    }

    dev->words = (dev->nb_channels + 31) / 32;
    if (dev->words) {
        mask = (uint32_t*) calloc(dev->words, sizeof(*mask));
        if (!mask) {
            free_device(dev);
            return ret;
        }
    }

    dev->mask = mask;

    ret = add_device_to_context(ctx, dev);
    if (ret < 0)
        free_device(dev);
    return ret;
}

static int foreach_in_dir(void *d, const char *path, bool is_dir,
        int (*callback)(void *, const char *))
{
    long name_max;
    struct dirent *entry, *result;
    DIR *dir = opendir(path);
    if (!dir)
        return -errno;

    name_max = pathconf(path, _PC_NAME_MAX);
    if (name_max == -1)
        name_max = 255;
    entry = (struct dirent *) malloc(offsetof(struct dirent, d_name) + name_max + 1);
    if (!entry) {
        closedir(dir);
        return -ENOMEM;
    }

    while (true) {
        struct stat st;
        char buf[1024];
        int ret = readdir_r(dir, entry, &result);
        if (ret) {
            iio_strerror(ret, buf, sizeof(buf));
            ERROR("Unable to open directory %s: %s\n", path, buf);
            free(entry);
            closedir(dir);
            return -ret;
        }
        if (!result)
            break;

        snprintf(buf, sizeof(buf), "%s/%s", path, entry->d_name);
        if (stat(buf, &st) < 0) {
            ret = -errno;
            iio_strerror(errno, buf, sizeof(buf));
            ERROR("Unable to stat file: %s\n", buf);
            free(entry);
            closedir(dir);
            return ret;
        }

        if (is_dir && S_ISDIR(st.st_mode) && entry->d_name[0] != '.')
            ret = callback(d, buf);
        else if (!is_dir && S_ISREG(st.st_mode))
            ret = callback(d, buf);
        else
            continue;

        if (ret < 0) {
            free(entry);
            closedir(dir);
            return ret;
        }
    }

    free(entry);
    closedir(dir);
    return 0;
}

void iio_context_destroy(struct iio_context *ctx)
{
    unsigned int i;

    for (i = 0; i < ctx->nb_devices; i++)
        free_device(ctx->devices[i]);
    if (ctx->nb_devices)
        free(ctx->devices);
    if (ctx->description)
        free(ctx->description);
    free(ctx);
}

static int add_debug_attr(void *d, const char *path)
{
    struct iio_device *dev = (struct iio_device *) d;
    const char *attr = strrchr(path, '/') + 1;
    char **attrs, *name = strdup(attr);
    if (!name)
        return -ENOMEM;

    attrs = (char**) realloc(dev->debug_attrs,
            (1 + dev->nb_debug_attrs) * sizeof(char *));
    if (!attrs) {
        free(name);
        return -ENOMEM;
    }

    attrs[dev->nb_debug_attrs++] = name;
    dev->debug_attrs = attrs;
    DEBUG("Added debug attr \'%s\' to device \'%s\'\n", name, dev->id);
    return 0;
}

struct iio_device * iio_context_find_device(const struct iio_context *ctx,
        const char *name)
{
    unsigned int i;
    for (i = 0; i < ctx->nb_devices; i++) {
        struct iio_device *dev = ctx->devices[i];
        if (!strcmp(dev->id, name) ||
                (dev->name && !strcmp(dev->name, name)))
            return dev;
    }
    return NULL;
}

static int add_debug(void *d, const char *path)
{
    struct iio_context *ctx = (struct iio_context *) d;
    const char *name = strrchr(path, '/') + 1;
    struct iio_device *dev = iio_context_find_device(ctx, name);
    if (!dev)
        return -ENODEV;
    else
        return foreach_in_dir(dev, path, false, add_debug_attr);
}

static void reorder_channels(struct iio_device *dev)
{
    bool found;
    unsigned int i;

    /* Reorder channels by index */
    do {
        found = false;
        for (i = 1; i < dev->nb_channels; i++) {
            struct iio_channel **channels = dev->channels;
            long ch1 = channels[i - 1]->index;
            long ch2 = channels[i]->index;

            if (ch2 >= 0 && ((ch1 > ch2) || ch1 < 0)) {
                struct iio_channel *bak = channels[i];
                channels[i] = channels[i - 1];
                channels[i - 1] = bak;
                found = true;
            }
        }
    } while (found);
}

void iio_context_init(struct iio_context *ctx)
{
    unsigned int i;
    for (i = 0; i < ctx->nb_devices; i++)
        reorder_channels(ctx->devices[i]);
}

static ssize_t local_read_chn_attr(const struct iio_channel *chn,
        const char *attr, char *dst, size_t len);

static ssize_t local_read_all_chn_attrs(const struct iio_channel *chn,
        char *dst, size_t len)
{
    unsigned int i;
    char *ptr = dst;

    for (i = 0; len >= 4 && i < chn->nb_attrs; i++) {
        /* Recursive! */
        ssize_t ret = local_read_chn_attr(chn,
                chn->attrs[i].name, ptr + 4, len - 4);
        *(uint32_t *) ptr = htonl(ret);

        /* Align the length to 4 bytes */
        if (ret > 0 && ret & 3)
            ret = ((ret >> 2) + 1) << 2;
        ptr += 4 + (ret < 0 ? 0 : ret);
        len -= 4 + (ret < 0 ? 0 : ret);
    }

    return ptr - dst;
}

static const char * get_filename(const struct iio_channel *chn,
        const char *attr)
{
    unsigned int i;
    for (i = 0; i < chn->nb_attrs; i++)
        if (!strcmp(attr, chn->attrs[i].name))
            return chn->attrs[i].filename;
    return attr;
}

static ssize_t local_read_chn_attr(const struct iio_channel *chn,
        const char *attr, char *dst, size_t len)
{
    if (!attr)
        return local_read_all_chn_attrs(chn, dst, len);

    attr = get_filename(chn, attr);
    return iio_device_attr_read(chn->dev, attr, dst, len, false);
}
static void init_index(struct iio_channel *chn)
{
    char buf[1024];
    long id = -ENOENT;

    if (chn->is_scan_element) {
        id = (long) local_read_chn_attr(chn, "index", buf, sizeof(buf));
        if (id > 0)
            id = atol(buf);
    }
    chn->index = id;
}

ssize_t iio_channel_attr_read(const struct iio_channel *chn,
        const char *attr, char *dst, size_t len)
{
    if (!attr)
        return local_read_all_chn_attrs(chn, dst, len);

    attr = get_filename(chn, attr);
    return iio_device_attr_read(chn->dev, attr, dst, len, false);
}

static void init_data_format(struct iio_channel *chn)
{
    char buf[1024];
    ssize_t ret;

    if (chn->is_scan_element) {
        ret = local_read_chn_attr(chn, "type", buf, sizeof(buf));
        if (ret < 0) {
            chn->format.length = 0;
        } else {
            char endian, sign;

            sscanf(buf, "%ce:%c%u/%u>>%u", &endian, &sign,
                    &chn->format.bits, &chn->format.length,
                    &chn->format.shift);
            chn->format.is_signed = (sign == 's' || sign == 'S');
            chn->format.is_fully_defined =
                    (sign == 'S' || sign == 'U'||
                    chn->format.bits == chn->format.length);
            chn->format.is_be = endian == 'b';
        }
    }

    ret = iio_channel_attr_read(chn, "scale", buf, sizeof(buf));
    if (ret < 0) {
        chn->format.with_scale = false;
    } else {
        chn->format.with_scale = true;
        chn->format.scale = atof(buf);
    }
}

static void init_scan_elements(struct iio_context *ctx)
{
    unsigned int i, j;

    for (i = 0; i < ctx->nb_devices; i++) {
        struct iio_device *dev = ctx->devices[i];

        for (j = 0; j < dev->nb_channels; j++) {
            struct iio_channel *chn = dev->channels[j];
            init_index(chn);
            init_data_format(chn);
        }
    }
}
struct iio_context* iio_create_local_context(void)
{
    int ret;
    unsigned int len;
    struct utsname uts;
    struct iio_context *ctx = (struct iio_context*) calloc(1, sizeof(*ctx));
    if (!ctx)
        goto err_set_errno_enomem;

    ctx->name = "local";
    set_timeout(ctx, DEFAULT_TIMEOUT_MS);

    uname(&uts);
    len = strlen(uts.sysname) + strlen(uts.nodename) + strlen(uts.release)
        + strlen(uts.version) + strlen(uts.machine);
    ctx->description = (char*) malloc(len + 5); /* 4 spaces + EOF */
    if (!ctx->description) {
        free(ctx);
        goto err_set_errno_enomem;
    }

    snprintf(ctx->description, len + 5, "%s %s %s %s %s", uts.sysname,
            uts.nodename, uts.release, uts.version, uts.machine);

    ret = foreach_in_dir(ctx, "/sys/bus/iio/devices", true, create_device);
    if (ret < 0) {
        iio_context_destroy(ctx);
        errno = -ret;
        return NULL;
    }

    foreach_in_dir(ctx, "/sys/kernel/debug/iio", true, add_debug);

    init_scan_elements(ctx);
    iio_context_init(ctx);

    return ctx;

err_set_errno_enomem:
    errno = ENOMEM;
    return NULL;
}

bool iio_channel_is_output(const struct iio_channel *chn)
{
    return chn->is_output;
}

struct iio_channel * iio_device_find_channel(const struct iio_device *dev,
        const char *name, bool output)
{
    unsigned int i;
    for (i = 0; i < dev->nb_channels; i++) {
        struct iio_channel *chn = dev->channels[i];
        if (iio_channel_is_output(chn) != output)
            continue;

        if (!strcmp(chn->id, name) ||
                (chn->name && !strcmp(chn->name, name)))
            return chn;
    }
    return NULL;
}


void iio_channel_enable(struct iio_channel *chn)
{
    if (chn->is_scan_element && chn->index >= 0 && chn->dev->mask)
        SET_BIT(chn->dev->mask, chn->index);
}

int iio_device_set_kernel_buffers_count(const struct iio_device *dev,
        unsigned int nb_buffers)
{
    struct iio_device_pdata *pdata = dev->pdata;

    if (nb_buffers == 0)
        return -EINVAL;
    if (pdata->fd != -1)
        return -EBUSY;

    pdata->nb_blocks = nb_buffers;

    return 0;
}

ssize_t iio_device_get_sample_size_mask(const struct iio_device *dev,
        const uint32_t *mask, size_t words)
{
    ssize_t size = 0;
    unsigned int i;

    if (words != (dev->nb_channels + 31) / 32)
        return -EINVAL;

    for (i = 0; i < dev->nb_channels; i++) {
        const struct iio_channel *chn = dev->channels[i];
        unsigned int length = chn->format.length / 8;

        if (chn->index < 0)
            break;
        if (!TEST_BIT(mask, chn->index))
            continue;

        if (size % length)
            size += 2 * length - (size % length);
        else
            size += length;
    }
    return size;
}

ssize_t iio_device_get_sample_size(const struct iio_device *dev)
{
    return iio_device_get_sample_size_mask(dev, dev->mask, dev->words);
}

static int local_buffer_analyze(unsigned int nb, const char *src, size_t len)
{
    while (nb--) {
        int32_t val;

        if (len < 4)
            return -EINVAL;

        val = (int32_t) ntohl(*(uint32_t *) src);
        src += 4;
        len -= 4;

        if (val > 0) {
            if ((uint32_t) val > len)
                return -EINVAL;

            /* Align the length to 4 bytes */
            if (val & 3)
                val = ((val >> 2) + 1) << 2;
            len -= val;
            src += val;
        }
    }

    /* We should have analyzed the whole buffer by now */
    return !len ? 0 : -EINVAL;
}

static ssize_t local_write_all_dev_attrs(const struct iio_device *dev,
        const char *src, size_t len, bool is_debug)
{
    unsigned int i, nb = is_debug ? dev->nb_debug_attrs : dev->nb_attrs;
    char **attrs = is_debug ? dev->debug_attrs : dev->attrs;
    const char *ptr = src;

    /* First step: Verify that the buffer is in the correct format */
    if (local_buffer_analyze(nb, src, len))
        return -EINVAL;

    /* Second step: write the attributes */
    for (i = 0; i < nb; i++) {
        int32_t val = (int32_t) ntohl(*(uint32_t *) ptr);
        ptr += 4;

        if (val > 0) {
            local_write_dev_attr(dev, attrs[i], ptr, val, is_debug);

            /* Align the length to 4 bytes */
            if (val & 3)
                val = ((val >> 2) + 1) << 2;
            ptr += val;
        }
    }

    return ptr - src;
}

static ssize_t local_write_dev_attr(const struct iio_device *dev,
        const char *attr, const char *src, size_t len, bool is_debug)
{
    FILE *f;
    char buf[1024];
    ssize_t ret;

    if (!attr)
        return local_write_all_dev_attrs(dev, src, len, is_debug);

    if (is_debug)
        snprintf(buf, sizeof(buf), "/sys/kernel/debug/iio/%s/%s",
                dev->id, attr);
    else
        snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/%s/%s",
                dev->id, attr);
    f = fopen(buf, "w");
    if (!f)
        return -errno;

    ret = fwrite(src, 1, len, f);
    fflush(f);
    if (ferror(f))
        ret = -errno;
    fclose(f);
    return ret ? ret : -EIO;
}

bool iio_channel_is_enabled(const struct iio_channel *chn)
{
    return chn->index >= 0 && chn->dev->mask &&
        TEST_BIT(chn->dev->mask, chn->index);
}

static ssize_t local_write_chn_attr(const struct iio_channel *chn,
        const char *attr, const char *src, size_t len);

static ssize_t local_write_all_chn_attrs(const struct iio_channel *chn,
        const char *src, size_t len)
{
    unsigned int i, nb = chn->nb_attrs;
    const char *ptr = src;

    /* First step: Verify that the buffer is in the correct format */
    if (local_buffer_analyze(nb, src, len))
        return -EINVAL;

    /* Second step: write the attributes */
    for (i = 0; i < nb; i++) {
        int32_t val = (int32_t) ntohl(*(uint32_t *) ptr);
        ptr += 4;

        if (val > 0) {
            local_write_chn_attr(chn, chn->attrs[i].name, ptr, val);

            /* Align the length to 4 bytes */
            if (val & 3)
                val = ((val >> 2) + 1) << 2;
            ptr += val;
        }
    }

    return ptr - src;
}

static ssize_t local_write_chn_attr(const struct iio_channel *chn,
        const char *attr, const char *src, size_t len)
{
    if (!attr)
        return local_write_all_chn_attrs(chn, src, len);

    attr = get_filename(chn, attr);
    return local_write_dev_attr(chn->dev, attr, src, len, false);
}

static int channel_write_state(const struct iio_channel *chn)
{
    const char *en = iio_channel_is_enabled(chn) ? "1" : "0";
    ssize_t ret = local_write_chn_attr(chn, "en", en, 2);
    if (ret < 0)
        return (int) ret;
    else
        return 0;
}


static int enable_high_speed(const struct iio_device *dev)
{
    struct block_alloc_req req;
    struct iio_device_pdata *pdata = dev->pdata;
    unsigned int i;
    int ret, fd = pdata->fd;

    if (pdata->cyclic) {
        pdata->nb_blocks = 1;
        DEBUG("Enabling cyclic mode\n");
    } else {
        DEBUG("Cyclic mode not enabled\n");
    }

    pdata->blocks = (struct block*) calloc(pdata->nb_blocks, sizeof(*pdata->blocks));
    if (!pdata->blocks) {
        pdata->nb_blocks = 0;
        return -ENOMEM;
    }

    pdata->addrs = (void**) calloc(pdata->nb_blocks, sizeof(*pdata->addrs));
    if (!pdata->addrs) {
        free(pdata->blocks);
        pdata->blocks = NULL;
        return -ENOMEM;
    }

    req.id = 0;
    req.type = 0;
    req.size = pdata->samples_count *
        iio_device_get_sample_size_mask(dev, dev->mask, dev->words);
    req.count = pdata->nb_blocks;

    ret = ioctl(fd, BLOCK_ALLOC_IOCTL, &req);
    if (ret < 0) {
        ret = -errno;
        goto err_freemem;
    }

    /* We might get less blocks than what we asked for */
    pdata->nb_blocks = req.count;

    /* mmap all the blocks */
    for (i = 0; i < pdata->nb_blocks; i++) {
        pdata->blocks[i].id = i;
        ret = ioctl(fd, BLOCK_QUERY_IOCTL, &pdata->blocks[i]);
        if (ret) {
            ret = -errno;
            goto err_munmap;
        }

        ret = ioctl(fd, BLOCK_ENQUEUE_IOCTL, &pdata->blocks[i]);
        if (ret) {
            ret = -errno;
            goto err_munmap;
        }

        pdata->addrs[i] = mmap(0, pdata->blocks[i].size,
                PROT_READ | PROT_WRITE,
                MAP_SHARED, fd, pdata->blocks[i].offset);
        if (pdata->addrs[i] == MAP_FAILED) {
            ret = -errno;
            goto err_munmap;
        }
    }

    pdata->last_dequeued = -1;
    return 0;

err_munmap:
    for (; i > 0; i--)
        munmap(pdata->addrs[i - 1], pdata->blocks[i - 1].size);
    ioctl(fd, BLOCK_FREE_IOCTL, 0);
err_freemem:
    free(pdata->addrs);
    pdata->addrs = NULL;
    free(pdata->blocks);
    pdata->blocks = NULL;
    return ret;
}

static ssize_t local_enable_buffer(const struct iio_device *dev)
{
    struct iio_device_pdata *pdata = dev->pdata;
    ssize_t ret = 0;

    if (!pdata->buffer_enabled) {
        ret = local_write_dev_attr(dev,
                "buffer/enable", "1", 2, false);
        if (ret >= 0)
            pdata->buffer_enabled = true;
    }

    return 0;
}


static int local_open(const struct iio_device *dev,
        size_t samples_count, bool cyclic)
{
    unsigned int i;
    int ret;
    char buf[1024];
    struct iio_device_pdata *pdata = dev->pdata;

    if (pdata->fd != -1)
        return -EBUSY;

    ret = local_write_dev_attr(dev, "buffer/enable", "0", 2, false);
    if (ret < 0)
        return ret;

    snprintf(buf, sizeof(buf), "%lu", (unsigned long) samples_count);
    ret = local_write_dev_attr(dev, "buffer/length",
            buf, strlen(buf) + 1, false);
    if (ret < 0)
        return ret;

    snprintf(buf, sizeof(buf), "/dev/%s", dev->id);
    pdata->fd = open(buf, O_RDWR);
    if (pdata->fd == -1)
        return -errno;

    /* Disable channels */
    for (i = 0; i < dev->nb_channels; i++) {
        struct iio_channel *chn = dev->channels[i];
        if (chn->index >= 0 && !iio_channel_is_enabled(chn)) {
            ret = channel_write_state(chn);
            if (ret < 0)
                goto err_close;
        }
    }
    /* Enable channels */
    for (i = 0; i < dev->nb_channels; i++) {
        struct iio_channel *chn = dev->channels[i];
        if (chn->index >= 0 && iio_channel_is_enabled(chn)) {
            ret = channel_write_state(chn);
            if (ret < 0)
                goto err_close;
        }
    }

    pdata->cyclic = cyclic;
    pdata->cyclic_buffer_enqueued = false;
    pdata->buffer_enabled = false;
    pdata->samples_count = samples_count;
    pdata->is_high_speed = !enable_high_speed(dev);

    if (!pdata->is_high_speed) {
        unsigned long size = samples_count * pdata->nb_blocks;
        WARNING("High-speed mode not enabled\n");

        /* Increase the size of the kernel buffer, when using the
         * low-speed interface. This avoids losing samples when
         * refilling the iio_buffer. */
        snprintf(buf, sizeof(buf), "%lu", size);
        ret = local_write_dev_attr(dev, "buffer/length",
                buf, strlen(buf) + 1, false);
        if (ret < 0)
            goto err_close;

        /* NOTE: The low-speed interface will enable the buffer after
         * the first samples are written, or if the device is set
         * to non blocking-mode */
    } else {
        ret = local_enable_buffer(dev);
        if (ret < 0)
            goto err_close;
    }

    return 0;
err_close:
    close(pdata->fd);
    pdata->fd = -1;
    return ret;
}


int iio_device_open(const struct iio_device *dev,
        size_t samples_count, bool cyclic)
{
    unsigned int i;
    bool has_channels = false;

    for (i = 0; !has_channels && i < dev->words; i++)
        has_channels = !!dev->mask[i];
    if (!has_channels)
        return -EINVAL;

    return local_open(dev, samples_count, cyclic);
}
static ssize_t local_get_buffer(const struct iio_device *dev,
        void **addr_ptr, size_t bytes_used,
        uint32_t *mask, size_t words)
{
    struct block block;
    struct iio_device_pdata *pdata = dev->pdata;
    int f = pdata->fd;
    ssize_t ret;

    if (!pdata->is_high_speed)
        return -ENOSYS;
    if (f == -1)
        return -EBADF;
    if (!addr_ptr)
        return -EINVAL;

    if (pdata->last_dequeued >= 0) {
        struct block *last_block = &pdata->blocks[pdata->last_dequeued];

        if (pdata->cyclic) {
            if (pdata->cyclic_buffer_enqueued)
                return -EBUSY;
            pdata->blocks[0].flags |= BLOCK_FLAG_CYCLIC;
            pdata->cyclic_buffer_enqueued = true;
        }

        last_block->bytes_used = bytes_used;
        ret = (ssize_t) ioctl(f,
                BLOCK_ENQUEUE_IOCTL, last_block);
        if (ret) {
            ret = (ssize_t) -errno;
            ERROR("Unable to enqueue block: %s\n", strerror(errno));
            return ret;
        }

        if (pdata->cyclic) {
            *addr_ptr = pdata->addrs[pdata->last_dequeued];
            return (ssize_t) last_block->bytes_used;
        }
    }

    memset(&block, 0, sizeof(block));
    ret = (ssize_t) ioctl(f, BLOCK_DEQUEUE_IOCTL, &block);
    if (ret) {
        ret = (ssize_t) -errno;
        ERROR("Unable to dequeue block: %s\n", strerror(errno));
        return ret;
    }

    /* Requested buffer size is too big! */
    if (pdata->last_dequeued < 0 && bytes_used != block.size)
        return -EFBIG;

    pdata->last_dequeued = block.id;
    *addr_ptr = pdata->addrs[block.id];
    return (ssize_t) block.bytes_used;
}

static bool device_is_high_speed(const struct iio_device *dev)
{
    return local_get_buffer(dev, NULL, 0, NULL, 0) != -ENOSYS;
}

bool iio_device_is_tx(const struct iio_device *dev)
{
    unsigned int i;

    for (i = 0; i < dev->nb_channels; i++) {
        struct iio_channel *ch = dev->channels[i];
        if (iio_channel_is_output(ch) && iio_channel_is_enabled(ch))
            return true;
    }

    return false;
}


static int iio_device_close(const struct iio_device *dev)
{
    struct iio_device_pdata *pdata = dev->pdata;
    int ret;

    if (pdata->fd == -1)
        return -EBADF;

    if (pdata->is_high_speed) {
        unsigned int i;
        for (i = 0; i < pdata->nb_blocks; i++)
            munmap(pdata->addrs[i], pdata->blocks[i].size);
        ioctl(pdata->fd, BLOCK_FREE_IOCTL, 0);
        free(pdata->addrs);
        pdata->addrs = NULL;
        free(pdata->blocks);
        pdata->blocks = NULL;
    }

    ret = close(pdata->fd);
    if (ret)
        return ret;

    pdata->fd = -1;
    ret = local_write_dev_attr(dev, "buffer/enable", "0", 2, false);
    return (ret < 0) ? ret : 0;
}

struct iio_buffer * iio_device_create_buffer(const struct iio_device *dev,
        size_t samples_count, bool cyclic)
{
    int ret = -EINVAL;
    struct iio_buffer *buf;
    unsigned int sample_size = iio_device_get_sample_size(dev);
    if (!sample_size)
        goto err_set_errno;

    buf = (struct iio_buffer *) malloc(sizeof(*buf));
    if (!buf) {
        ret = -ENOMEM;
        goto err_set_errno;
    }

    buf->dev_sample_size = sample_size;
    buf->length = sample_size * samples_count;
    buf->dev = dev;
    buf->mask = (uint32_t*) calloc(dev->words, sizeof(*buf->mask));
    if (!buf->mask) {
        ret = -ENOMEM;
        goto err_free_buf;
    }

    /* Set the default channel mask to the one used by the device.
     * While input buffers will erase this as soon as the refill function
     * is used, it is useful for output buffers, as it permits
     * iio_buffer_foreach_sample to be used. */
    memcpy(buf->mask, dev->mask, dev->words * sizeof(*buf->mask));

    ret = iio_device_open(dev, samples_count, cyclic);
    if (ret < 0)
        goto err_free_mask;

    buf->dev_is_high_speed = device_is_high_speed(dev);
    if (buf->dev_is_high_speed) {
        /* Dequeue the first buffer, so that buf->buffer is correctly
         * initialized */
        buf->buffer = NULL;
        if (iio_device_is_tx(dev)) {
            ret = local_get_buffer(dev, &buf->buffer,
                    buf->length, buf->mask, dev->words);
            if (ret < 0)
                goto err_close_device;
        }
    } else {
        buf->buffer = malloc(buf->length);
        if (!buf->buffer) {
            ret = -ENOMEM;
            goto err_close_device;
        }
    }

    buf->sample_size = iio_device_get_sample_size_mask(dev,
            buf->mask, dev->words);
    buf->data_length = buf->length;
    return buf;

err_close_device:
    iio_device_close(dev);
err_free_mask:
    free(buf->mask);
err_free_buf:
    free(buf);
err_set_errno:
    errno = -ret;
    return NULL;
}


void iio_buffer_destroy(struct iio_buffer *buffer)
{
    iio_device_close(buffer->dev);
    if (!buffer->dev_is_high_speed)
        free(buffer->buffer);
    free(buffer->mask);
    free(buffer);
}

ssize_t iio_device_attr_write(const struct iio_device *dev,
        const char *attr, const char *src)
{
    return local_write_dev_attr(dev,
            attr, (const char*) src, strlen(src) + 1, false);
}


static int device_check_ready(const struct iio_device *dev, bool do_write)
{
    struct pollfd pollfd = {
        dev->pdata->fd,
        do_write ? POLLOUT : POLLIN,
    };
    int ret;

    if (!dev->pdata->blocking)
        return 0;

    ret = poll(&pollfd, 1, dev->ctx->rw_timeout_ms);
    if (ret < 0)
        return -errno;
    if (!ret)
        return -ETIMEDOUT;
    if (pollfd.revents & POLLNVAL)
        return -EBADF;
    if (!(pollfd.revents & (do_write ? POLLOUT : POLLIN)))
        return -EIO;
    return 0;
}

static ssize_t read_all(void *dst, size_t len, int fd)
{
    uintptr_t ptr = (uintptr_t) dst;
    ssize_t readsize;
    int ret;

    while (len > 0) {
        do {
            ret = read(fd, (void *) ptr, len);
        } while (ret == -1 && errno == EINTR);

        if (ret == -1) {
            ret = -errno;
            break;
        } else if (ret == 0) {
            ret = -EIO;
            break;
        }

        ptr += ret;
        len -= ret;
    }

    readsize = (ssize_t)(ptr - (uintptr_t) dst);
    if ((ret > 0 || ret == -EAGAIN) && (readsize > 0))
        return readsize;
    else
        return ret;
}

static ssize_t local_read(const struct iio_device *dev,
        void *dst, size_t len, uint32_t *mask, size_t words)
{
    ssize_t ret;
    struct iio_device_pdata *pdata = dev->pdata;
    if (pdata->fd == -1)
        return -EBADF;
    if (words != dev->words)
        return -EINVAL;

    ret = local_enable_buffer(dev);
    if (ret < 0)
        return ret;

    ret = device_check_ready(dev, false);
    if (ret < 0)
        return ret;

    memcpy(mask, dev->mask, words);
    ret = read_all(dst, len, pdata->fd);

    return ret ? ret : -EIO;
}

ssize_t iio_buffer_refill(struct iio_buffer *buffer)
{
    ssize_t read;
    const struct iio_device *dev = buffer->dev;

    if (buffer->dev_is_high_speed) {
        read = local_get_buffer(dev, &buffer->buffer,
                buffer->length, buffer->mask, dev->words);
    } else {
        read = local_read(dev, buffer->buffer, buffer->length,
                buffer->mask, dev->words);
    }

    if (read >= 0) {
        buffer->data_length = read;
        buffer->sample_size = iio_device_get_sample_size_mask(dev,
                buffer->mask, dev->words);
    }
    return read;
}

ptrdiff_t iio_buffer_step(const struct iio_buffer *buffer)
{
    return (ptrdiff_t) buffer->sample_size;
}

void * iio_buffer_end(const struct iio_buffer *buffer)
{
    return (void *) ((uintptr_t) buffer->buffer + buffer->data_length);
}

void * iio_buffer_first(const struct iio_buffer *buffer,
        const struct iio_channel *chn)
{
    size_t len;
    unsigned int i;
    uintptr_t ptr = (uintptr_t) buffer->buffer;

    if (!iio_channel_is_enabled(chn))
        return iio_buffer_end(buffer);

    for (i = 0; i < buffer->dev->nb_channels; i++) {
        struct iio_channel *cur = buffer->dev->channels[i];
        len = cur->format.length / 8;

        /* NOTE: dev->channels are ordered by index */
        if (cur->index < 0 || cur->index == chn->index)
            break;

        /* Test if the buffer has samples for this channel */
        if (!TEST_BIT(buffer->mask, cur->index))
            continue;

        if (ptr % len)
            ptr += len - (ptr % len);
        ptr += len;
    }

    len = chn->format.length / 8;
    if (ptr % len)
        ptr += len - (ptr % len);
    return (void *) ptr;
}

static void byte_swap(uint8_t *dst, const uint8_t *src, size_t len)
{
    size_t i;
    for (i = 0; i < len; i++)
        dst[i] = src[len - i - 1];
}


static void shift_bits(uint8_t *dst, size_t shift, size_t len, bool left)
{
    size_t i, shift_bytes = shift / 8;
    shift %= 8;

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if (!left)
#else
    if (left)
#endif
    {
        if (shift_bytes) {
            memmove(dst, dst + shift_bytes, len - shift_bytes);
            memset(dst + len - shift_bytes, 0, shift_bytes);
        }
        if (shift) {
            for (i = 0; i < len; i++) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
                dst[i] >>= shift;
                if (i < len - 1)
                    dst[i] |= dst[i + 1] << (8 - shift);
#else
                dst[i] <<= shift;
                if (i < len - 1)
                    dst[i] |= dst[i + 1] >> (8 - shift);
#endif
            }
        }
    } else {
        if (shift_bytes) {
            memmove(dst + shift_bytes, dst, len - shift_bytes);
            memset(dst, 0, shift_bytes);
        }
        if (shift) {
            for (i = len; i > 0; i--) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
                dst[i - 1] <<= shift;
                if (i > 1)
                    dst[i - 1] |= dst[i - 2] >> (8 - shift);
#else
                dst[i - 1] >>= shift;
                if (i > 1)
                    dst[i - 1] |= dst[i - 2] << (8 - shift);
#endif
            }
        }
    }
}

static void sign_extend(uint8_t *dst, size_t bits, size_t len)
{
    size_t upper_bytes = ((len * 8 - bits) / 8);
    uint8_t msb, msb_bit = 1 << ((bits - 1) % 8);

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    msb = dst[len - 1 - upper_bytes] & msb_bit;
    if (upper_bytes)
        memset(dst + len - upper_bytes, msb ? 0xff : 0x00, upper_bytes);
    if (msb)
        dst[len - 1 - upper_bytes] |= ~(msb_bit - 1);
    else
        dst[len - 1 - upper_bytes] &= (msb_bit - 1);
#else
    /* XXX: untested */
    msb = dst[upper_bytes] & msb_bit;
    if (upper_bytes)
        memset(dst, msb ? 0xff : 0x00, upper_bytes);
    if (msb)
        dst[upper_bytes] |= ~(msb_bit - 1);
#endif
}

static void mask_upper_bits(uint8_t *dst, size_t bits, size_t len)
{
    size_t i;

    /* Clear upper bits */
    if (bits % 8)
        dst[bits / 8] &= (1 << (bits % 8)) - 1;

    /* Clear upper bytes */
    for (i = (bits + 7) / 8; i < len; i++)
        dst[i] = 0;
}

void iio_channel_convert(const struct iio_channel *chn,
        void *dst, const void *src)
{
    unsigned int len = chn->format.length / 8;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    bool swap = chn->format.is_be;
#else
    bool swap = !chn->format.is_be;
#endif

    if (len == 1 || !swap)
        memcpy(dst, src, len);
    else
        byte_swap((unsigned char*) dst, (const unsigned char*) src, len);

    if (chn->format.shift)
        shift_bits((unsigned char*) dst, chn->format.shift, len, false);

    if (!chn->format.is_fully_defined) {
        if (chn->format.is_signed)
            sign_extend((unsigned char*) dst, chn->format.bits, len);
        else
            mask_upper_bits((unsigned char*) dst, chn->format.bits, len);
    }
}
