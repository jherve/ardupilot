
#include <AP_HAL/AP_HAL.h>

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

/*
 * GPIO used to configure ultrasound level
 */
#define P7_US_PULSE_LEVEL_GPIO "/sys/class/gpio/gpio200/value"
/*
 * this mode is used at low altitude
 * send 4 wave patterns
 * gpio in low mode
 */
#define P7_US_DEFAULT_MODE 1

/*
 * the number of p7s in the iio buffer
 */
#define P7_US_P7_COUNT 8192

extern const AP_HAL::HAL& hal;

#define ULOG(_fmt, ...)   fprintf(stdout, _fmt "\n", ##__VA_ARGS__)
/** Log as debug */
#define ULOGD(_fmt, ...)  ULOG("[D]" _fmt, ##__VA_ARGS__)
/** Log as info */
#define ULOGI(_fmt, ...)  ULOG("[I]" _fmt, ##__VA_ARGS__)
/** Log as warning */
#define ULOGW(_fmt, ...)  ULOG("[W]" _fmt, ##__VA_ARGS__)
/** Log as error */
#define ULOGE(_fmt, ...)  ULOG("[E]" _fmt, ##__VA_ARGS__)

/*
 * purge is used when changing mode
 */
int UltraSound_Bebop::launch_purge()
{
    iio_device_attr_write(_adc.device, "buffer/enable", "1");
    _spi->transfer(_purge, P7_US_NB_PULSES_PURGE);
    return 0;
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
    configure_gpio(0);
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
    _hysteresis_counter = 0;
    /* SPI and IIO can not be initialized just yet */
    _spi = nullptr;
    _iio = nullptr;
    memset(_tx[0], 0xF0, 16);
    memset(_tx[1], 0xF0, 4);
    memset(_purge, 0xFF, P7_US_NB_PULSES_PURGE);
    _tx_buf = _tx[_mode];
}

void UltraSound_Bebop::init()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_BebopUltraSound);
    if (_spi == NULL) {
        hal.scheduler->panic("Could not find SPI device for Bebop ultrasound");

        return; /* never reached */
    }
    _spi->init();

    if (configure_capture() < 0)
        goto error_free_us;

    if (configure_wave() < 0)
        goto error_buffer_destroy;

    return;

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
    _spi->transfer(_tx_buf, P7_US_NB_PULSES_MAX);
    return 0;
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

static int f_is_zero(const float f)
{
    return fabsf(f) < FLT_EPSILON;
}

int UltraSound_Bebop::update_mode(float altitude)
{
    switch (_mode) {
    case 0:
        if (altitude < P7_US_TRANSITION_HIGH_TO_LOW
                && !f_is_zero(altitude)) {
            if (_hysteresis_counter > P7_US_TRANSITION_COUNT) {
                _mode = 1;
                _hysteresis_counter = 0;
                reconfigure_wave();
            } else {
                _hysteresis_counter++;
            }
        } else {
            _hysteresis_counter = 0;
        }
        break;

    default:
    case 1:
        if (altitude > P7_US_TRANSITION_LOW_TO_HIGH
                || f_is_zero(altitude)) {
            if (_hysteresis_counter > P7_US_TRANSITION_COUNT) {
                _mode = 0;
                _hysteresis_counter = 0;
                reconfigure_wave();
            } else {
                _hysteresis_counter++;
            }
        } else {
            _hysteresis_counter = 0;
        }
        break;
    }
    return _mode;
}

UltraSound_Bebop::~UltraSound_Bebop()
{
    iio_buffer_destroy(_adc.buffer);
    _adc.buffer = NULL;
    iio_context_destroy(_iio);
    _iio = NULL;
}
