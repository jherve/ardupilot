
#ifndef __AP_HAL_LINUX_ULTRASOUND_BEBOP_H__
#define __AP_HAL_LINUX_ULTRASOUND_BEBOP_H__

#include <stdint.h>
#include <linux/spi/spidev.h>

/*
 * the size of the buffer sent over spi
 */
#define P7_US_NB_PULSES_MAX 32

/*
 * the size of the purge buffer sent over spi
 */
#define P7_US_NB_PULSES_PURGE 64

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
 * struct related to adc
 * data to receive and process adc datas
 */
struct adcCapture_t {
    struct iio_device *device;
    struct iio_buffer *buffer;
    unsigned int buffer_size;
    struct iio_channel *channel;
    unsigned int freq;

     /* Used in order to match two echoes of two ADC acquisitions */
    unsigned short threshold_time_rejection;
};

class UltraSound_Bebop {
public:
    UltraSound_Bebop();
    ~UltraSound_Bebop();

    void init(void);
    int launch(void);
    int capture(void);
    int update_mode(float altitude);
    void wave_test(unsigned int);
    unsigned int get_buffer_size() { return _adc.buffer_size; };
    struct adcCapture_t* get_capture() { return &_adc; };

private:
    struct adcCapture_t _adc;
    AP_HAL::SPIDeviceDriver *_spi;
    struct iio_context *_iio;
    int _mode;
    unsigned char _tx[2][P7_US_NB_PULSES_MAX];
    unsigned char _purge[P7_US_NB_PULSES_PURGE];
    unsigned char* _tx_buf;
    int _hysteresis_counter;

    void configure_gpio(int value);
    int configure_wave();
    void reconfigure_wave();
    int configure_capture();
    int launch_purge();

};

#endif /* __AP_HAL_LINUX_ULTRASOUND_BEBOP_H__ */
