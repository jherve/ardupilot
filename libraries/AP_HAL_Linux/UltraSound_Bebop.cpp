

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <linux/types.h>
#include <errno.h>
#include <linux/spi/spidev.h>
#include <AP_HAL_Linux/IIO.h>
#include <sys/ioctl.h>
#include <float.h>
#include <math.h>
#include <time.h>
#include "UltraSound_Bebop.h"
#include "IIO.h"

#define ULOG(_fmt, ...)   fprintf(stderr, _fmt "\n", ##__VA_ARGS__)
/** Log as debug */
#define ULOGD(_fmt, ...)  ULOG("[D]" _fmt, ##__VA_ARGS__)
/** Log as info */
#define ULOGI(_fmt, ...)  ULOG("[I]" _fmt, ##__VA_ARGS__)
/** Log as warning */
#define ULOGW(_fmt, ...)  ULOG("[W]" _fmt, ##__VA_ARGS__)
/** Log as error */
#define ULOGE(_fmt, ...)  ULOG("[E]" _fmt, ##__VA_ARGS__)

/* flags to mark echoes */
#define ECHO_REJECTED 0x20
#define ECHO_PREVIOUS_BETTER 0x40
#define ECHO_FOLLOWING_BETTER 0x80


/* codecheck_ignore[COMPLEX_MACRO] */
#define P2_1200 1200, 1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P4_1200 P2_1200, P2_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P8_1200 P4_1200, P4_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P16_1200 P8_1200, P8_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P32_1200 P16_1200, P16_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P64_1200 P32_1200, P32_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P128_1200 P64_1200, P64_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P256_1200 P128_1200, P128_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P512_1200 P256_1200, P256_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P1024_1200 P512_1200, P512_1200
/* codecheck_ignore[COMPLEX_MACRO] */
#define P2048_1200 P1024_1200, P1024_1200

/* codecheck_ignore[COMPLEX_MACRO] */
#define P2_4195 4195, 4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P4_4195 P2_4195, P2_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P8_4195 P4_4195, P4_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P16_4195 P8_4195, P8_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P32_4195 P16_4195, P16_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P64_4195 P32_4195, P32_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P128_4195 P64_4195, P64_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P256_4195 P128_4195, P128_4195
/* codecheck_ignore[COMPLEX_MACRO] */
#define P512_4195 P256_4195, P256_4195

/*
 * threshold are values to compare with adc datas
 */
static unsigned short thresholds[2][2048] = {
    {
        P128_4195,
        P8_4195,
        P2_4195,
        4195,
        4000, 3800, 3600, 3400, 3200, 3000, 2800,
        2600, 2400, 2200, 2000, 1800, 1600, 1400,
        P1024_1200,
        P512_1200,
        P256_1200,
        P64_1200,
        P16_1200,
        P16_1200,
        P4_1200,
        P2_1200,
        1200,
    },
    {
        P64_4195,
        P8_4195,
        4195,
        4190, 4158, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
        4095, 4090, 4058, 3943, 3924, 3841, 3679, 3588, 3403,
        3201, 3020, 2816, 2636, 2448, 2227, 2111, 1955, 1819,
        1675, 1540, 1492, 1374, 1292,
        P512_1200,
        P512_4195,
        P512_4195,
        P256_4195,
        P128_4195,
        P16_4195,
        P4_4195,
        P2_4195,
        4195,
    },
};

/*
 * purge is used when changing mode
 */
int UltraSound_Bebop::launch_purge()
{
    iio_device_attr_write(_adc.device, "buffer/enable", "1");
    return ioctl(_spi.fd, SPI_IOC_MESSAGE(1), &_spi.tr_purge);
}

void UltraSound_Bebop::configure_gpio(int value)
{
    int ret = 0;
    int fd;
    fd = open(P7_US_PULSE_LEVEL_GPIO, O_RDWR);
    if (fd == -1) {
        ULOGE("could not configure gpio");
        return;
    }
    switch (value) {
    case 1: // high voltage
        ret = write(fd, "1", 2);
        break;
    case 0: // low voltage
        ret = write(fd, "0", 2);
        break;
    default:
        ULOGE("bad gpio value (%d)", value);
        break;
    }
    if (ret != 2)
        ULOGE("error gpio");
}

/*
 * reconfigure the pulse that will be sent over spi
 * first send a purge then configure the new pulse
 */
void UltraSound_Bebop::reconfigure_wave()
{
    /* configure the output buffer for a purge */
    /* perform a purge */
    if (launch_purge() < 0)
        ULOGE("purge could not send data overspi");
    if (capture() < 0)
        ULOGE("purge could not capture data");
    /* configure the output buffer with a new mode */
    _spi.tr.tx_buf = (unsigned long)_spi.tx[_mode];
    _adc.thresholds = thresholds[_mode];
    switch (_mode) {
    case 1: /* low voltage */
        configure_gpio(0);
        break;
    case 0: /* high voltage */
        configure_gpio(1);
        break;
    default:
        ULOGE("p7us: WARNING, invalid value to configure gpio\n");
        break;
    }
}

/*
 * First configuration of the the pulse that will be send over spi
 */
int UltraSound_Bebop::configure_wave()
{
    const char *spiname = "/dev/spidev1.0";
    int spi_mode = 0;
    /* output configuration */
    /* low level mode by default */
    configure_gpio(0);

    /* configure spi device */
    _spi.fd = open(spiname, O_RDWR);
    if (_spi.fd < 0) {
        ULOGE("spidev_init: ERROR[%d], failed to open", _spi.fd);
        return -1;
    }

    memset(_spi.tx[0], 0xF0, 16);
    memset(_spi.tx[1], 0xF0, 4);
    memset(_spi.purge, 0xFF, P7_US_NB_PULSES_PURGE);
    _spi.tr.tx_buf = (unsigned long)_spi.tx[_mode];
    _spi.tr.len = P7_US_NB_PULSES_MAX;
    _spi.tr.rx_buf = (unsigned long)NULL;
    _spi.tr.delay_usecs = 0;
    _spi.tr.speed_hz = P7_US_SPI_SPEED;
    _spi.tr.bits_per_word = 8;
    _spi.tr.cs_change = 1;
    _adc.thresholds = thresholds[_mode];

    _spi.tr_purge.tx_buf = (unsigned long)_spi.purge;
    _spi.tr_purge.rx_buf = (unsigned long)NULL;
    _spi.tr_purge.delay_usecs = 0;
    _spi.tr_purge.speed_hz = P7_US_SPI_SPEED;
    _spi.tr_purge.bits_per_word = 8;
    _spi.tr_purge.cs_change = 1;
    _spi.tr_purge.len = P7_US_NB_PULSES_PURGE;

    if (ioctl(_spi.fd, SPI_IOC_WR_MODE, &spi_mode)
            || ioctl(_spi.fd, SPI_IOC_WR_BITS_PER_WORD,
                &_spi.tr.bits_per_word)
            || ioctl(_spi.fd, SPI_IOC_WR_MAX_SPEED_HZ,
                &_spi.tr.speed_hz)) {
        ULOGE("spidev_init: ERROR, failed to configure %s", spiname);
        close(_spi.fd);
        return -1;
    }
    return 0;
}

/*
 * Configure the adc to get the samples
 */
int UltraSound_Bebop::configure_capture()
{
    const char *adcname = "p7mu-adc_2";
    char *adcchannel = "voltage2";
    /* configure adc interface using libiio */
    _iio = iio_create_local_context();
    if (!_iio)
        return -1;
    _adc.device = iio_context_find_device(_iio, adcname);
    if (!_adc.device) {
        ULOGE("Unable to find %s", adcname);
        goto error_destroy_context;
    }
    _adc.channel = iio_device_find_channel(_adc.device, adcchannel,
            false);
    if (!_adc.channel) {
        ULOGE("Fail to init adc channel %s", adcchannel);
        goto error_destroy_context;
    }

    iio_channel_enable(_adc.channel);

    _adc.freq = P7_US_DEFAULT_ADC_FREQ >> P7_US_FILTER_POWER;
    _adc.threshold_time_rejection = 2.0 / P7_US_SOUND_SPEED *
        _adc.freq;

    /* Create input buffer */
    _adc.buffer_size = P7_US_P7_COUNT;
    if (iio_device_set_kernel_buffers_count(_adc.device, 1)) {
        ULOGE("cannot set buffer count");
        goto error_destroy_context;
    }
    _adc.buffer = iio_device_create_buffer(_adc.device,
            _adc.buffer_size, false);
    if (!_adc.buffer) {
        ULOGE("Fail to create buffer : %s", strerror(errno));
        goto error_destroy_context;
    }

    _adc.filter_buffer_size =
        _adc.buffer_size >> P7_US_FILTER_POWER;
    _adc.filter_buffer = (unsigned short *) calloc(1,
            sizeof(_adc.filter_buffer[0])
            * _adc.filter_buffer_size);
    if (!_adc.filter_buffer) {
        ULOGE("Fail to create filter buffer : %s", strerror(errno));
        goto error_buffer_destroy;
    }
    return 0;

error_buffer_destroy:
    iio_buffer_destroy(_adc.buffer);
    _adc.buffer = NULL;
error_destroy_context:
    iio_context_destroy(_iio);
    _iio = NULL;
    return -1;
}

/*
 * Initialize the us
 */
UltraSound_Bebop::UltraSound_Bebop()
{
    _mode = P7_US_DEFAULT_MODE;
    _freq = P7_US_DEFAULT_FREQ;
    if (configure_capture() < 0)
        goto error_free_us;

    if (configure_wave() < 0)
        goto error_buffer_destroy;

error_buffer_destroy:
    iio_buffer_destroy(_adc.buffer);
    _adc.buffer = NULL;
    iio_context_destroy(_iio);
    _iio = NULL;
error_free_us:
    ;
}

/*
 * enable the capture buffer
 * send a pulse over spi
 */
int UltraSound_Bebop::launch()
{
    iio_device_attr_write(_adc.device, "buffer/enable", "1");
    return ioctl(_spi.fd, SPI_IOC_MESSAGE(1), &_spi.tr);
}

/*
 * read the iio buffer
 * disable the capture buffer
 */
int UltraSound_Bebop::capture()
{
    int ret;

    ret = iio_buffer_refill(_adc.buffer);
    iio_device_attr_write(_adc.device, "buffer/enable", "0");
    return ret;
}

UltraSound_Bebop::~UltraSound_Bebop()
{
    close(_spi.fd);
    iio_buffer_destroy(_adc.buffer);
    _adc.buffer = NULL;
    free(_adc.filter_buffer);
    _adc.filter_buffer = NULL;
    iio_context_destroy(_iio);
    _iio = NULL;
}
