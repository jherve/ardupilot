
#ifndef __AP_HAL_LINUX_ULTRASOUND_BEBOP_H__
#define __AP_HAL_LINUX_ULTRASOUND_BEBOP_H__

#include <stdint.h>
#include <linux/spi/spidev.h>

/*
 * the number of p7s in the iio buffer
 */
#define P7_US_P7_COUNT 8192

/*
 * the size of the buffer sent over spi
 */
#define P7_US_NB_PULSES_MAX 32

/*
 * the size of the purge buffer sent over spi
 */
#define P7_US_NB_PULSES_PURGE 64

/*
 * value for echo processing
 */
#define P7_US_START_ECHO_FINDER 25

/*
 * value for echo processing
 */
#define P7_US_THRESHOLD_ECHO_FINDER 15

/*
 * value for echo processing
 */
#define P7_US_THRESHOLD_ECHO_INIT 1500

/*
 * the spi speed to create a wave
 */
#define P7_US_SPI_SPEED 320000

/*
 * the number of echoes we will keep at most
 */
#define P7_US_MAX_ECHOES 30

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
 * default us frequency
 * 17 times by seconds
 */
#define P7_US_DEFAULT_FREQ 17

/*
 * default adc frequency
 */
#define P7_US_DEFAULT_ADC_FREQ 160000

/*
 * to filter data we make the average of (1 << this_value) datas
 */
#define P7_US_FILTER_POWER 2

/*
 * Speed of sound
 */
#define P7_US_SOUND_SPEED 340

/* above this altitude we should use mode 0 */
#define P7_US_TRANSITION_HIGH_TO_LOW 0.75

/* below this altitude we should use mode 1 */
#define P7_US_TRANSITION_LOW_TO_HIGH 1.5

/* count this times before switching mode */
#define P7_US_TRANSITION_COUNT 5


/*
 * struct related to ultrasoud echo
 */
struct echo {
    uint16_t start_idx;
    uint16_t stop_idx;
    int32_t max_value;
    uint16_t max_idx;
    uint16_t previous;
    int16_t d_echo;
};

/*
 * struct related to adc
 * data to receive and process adc datas
 */
struct adc_info {
    /* step 0 cpature raw adc buffer */
    struct iio_device *device;
    struct iio_buffer *buffer;
    unsigned int buffer_size;
    struct iio_channel *channel;
    unsigned short *thresholds;
    unsigned int freq;

    /* step 1 create buffer with filtered data */
    unsigned short *filter_buffer;
    unsigned int filter_buffer_size;

    /* step 2 search echoes */
    struct echo echoes[P7_US_MAX_ECHOES];
    int nb_echoes;

    /* step 3 match with previous capture */
    struct echo echoes_old[P7_US_MAX_ECHOES];
    int nb_echoes_old;

    /* step 4 select an echo and an altitude */
    struct echo *echo_selected;
    float altitude;

    /* step 5 check if we should change the mode high or low mode */
    int count_us_transit;
     /* Used in order to match two echoes of two ADC acquisitions */
    unsigned short threshold_time_rejection;
};

/*
 * struct related to spi
 * data to transmit over ultrasound
 */
struct spi_info {
    int fd;
    unsigned char tx[2][P7_US_NB_PULSES_MAX];
    char purge[P7_US_NB_PULSES_PURGE];
    struct spi_ioc_transfer tr;
    struct spi_ioc_transfer tr_purge;
};

class UltraSound_Bebop {
public:
    UltraSound_Bebop();
    ~UltraSound_Bebop();

    int launch(void);
    int capture(void);
    void wave_test(unsigned int);

private:
    struct adc_info _adc;
    struct spi_info _spi;
    struct iio_context *_iio;
    int _mode;
    int _freq;

    void configure_gpio(int value);
    int configure_wave();
    void reconfigure_wave();
    int configure_capture();
    int launch_purge();

};

#endif /* __AP_HAL_LINUX_ULTRASOUND_BEBOP_H__ */
