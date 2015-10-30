// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>

#include "AP_RangeFinder_AnalogSonar.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>
#include "AP_HAL_Linux/UltraSound_Bebop.h"

#include "AP_HAL_Linux/IIO.h"

extern const AP_HAL::HAL& hal;

#define ULOG(_fmt, ...)   fprintf(stdout, _fmt "\n", ##__VA_ARGS__)
/** Log as debug */
#define ULOGD(_fmt, ...)  ULOG("**** [D]" _fmt, ##__VA_ARGS__)
/** Log as info */
#define ULOGI(_fmt, ...)  ULOG("**** [I]" _fmt, ##__VA_ARGS__)
/** Log as warning */
#define ULOGW(_fmt, ...)  ULOG("**** [W]" _fmt, ##__VA_ARGS__)
/** Log as error */
#define ULOGE(_fmt, ...)  ULOG("**** [E]" _fmt, ##__VA_ARGS__)

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
static unsigned short sThresholds[2][2048] = {
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



AP_RangeFinder_AnalogSonar::AP_RangeFinder_AnalogSonar(RangeFinder &_ranger,
                                        uint8_t instance,
                                        RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    _last_max_distance_cm(-1),
    _last_min_distance_cm(-1),
    _last_timestamp(0),
    _log(*this),
    _ultrasound(AP_HAL_Linux.ultraSound),
    _fd(-1),
    _mode(0),
    _nb_echoes(0),
    _nb_echoes_old(0),
    _altitude(0.0f),
    _echo_selected(NULL)
{
    _filter_buffer_size = _ultrasound->get_buffer_size() >> P7_US_FILTER_POWER;
    _filter_buffer = (unsigned short*) calloc(1, sizeof(_filter_buffer[0]) * _filter_buffer_size);
    _freq = P7_US_DEFAULT_FREQ;
}

/*
   close the file descriptor
*/
AP_RangeFinder_AnalogSonar::~AP_RangeFinder_AnalogSonar()
{
}

/**
 * Test if an echo is an rebound of an other echo of the same acquisition adc
 * @param echoA : test echoA with echo B
 * @param echoB : test echoB with echo A
 * @return : 1 => echo A and echoB are the "same" echo 0
 * => echoA and echoB are two different echoes
 */
uint8_t AP_RangeFinder_AnalogSonar::echo_linear_test(struct echo *echoA, struct echo *echoB)
{
    uint8_t return_value;
    float duty = abs((float)echoA->start_idx /
            echoB->start_idx);

    if (duty > 1.25 || duty < 0.75)
        return_value = 0;
    else
        return_value = 1;

    return return_value;
}

/*
 * p7_us_apply_filter
 * as input take raw adc buffer given by iio
 * as ouput give a filtered buffer
 * legacy of HAL get_raw_adc_data_tab
 */
int AP_RangeFinder_AnalogSonar::apply_filter(void)
{
    uint8_t *end;
    uint8_t *data_ptr;
    unsigned short data;
    ptrdiff_t step;

    unsigned int filter_idx = 0;
    unsigned int filter_power;
    unsigned int filter_counter;
    int first_echo_detected = 0;
    int echo_detected = 0;
    int filter_max;

    /* initialise datas */
    memset(_filter_buffer, 0, _filter_buffer_size);
    step = iio_buffer_step(_adcCapture->buffer);
    end = (unsigned char *) iio_buffer_end(_adcCapture->buffer);
    filter_counter = 0;
    filter_power = P7_US_FILTER_POWER;
    filter_max = 1 << filter_power;
    data_ptr = (unsigned char *) iio_buffer_first(_adcCapture->buffer,
                                                    _adcCapture->channel);

    /* search first echo */
    while (data_ptr < end) {
        iio_channel_convert(_adcCapture->channel, &data, data_ptr);
        data_ptr += step;

        if (data >= P7_US_THRESHOLD_ECHO_INIT) {
            /* first echo found */
            ULOGD("first echo detected");
            first_echo_detected = 1;
            break;
        }
    }

    if (!first_echo_detected) {
        ULOGD("no first echo detected");
        return 0;
    }
    _filter_buffer[0] = data;
    filter_counter++;

    /* search other echo by filtering datas
     * average is done on data gathered according to filter_max */
    while (data_ptr < end) {
        iio_channel_convert(_adcCapture->channel, &data, data_ptr);
        data_ptr += step;

        _filter_buffer[filter_idx] += data;
        filter_counter = (filter_counter + 1) % filter_max;

        if (!filter_counter) {
            _filter_buffer[filter_idx] >>= filter_power;
            if (!echo_detected && _filter_buffer[filter_idx]
                    > sThresholds[_mode][filter_idx]) {
                /* echo found process raw distance */
                echo_detected++;
            }
            filter_idx++;
        }
    }
    return !(echo_detected && first_echo_detected);
}

/*
 * p7_us_search_echoes
 * from filter_buffer get a list of echoes
 * and put them in us->adc.echoes
 * inherited from HAL_ultrasound_research_echoes
 */
int AP_RangeFinder_AnalogSonar::search_echoes(void)
{
    unsigned short *thresholds = sThresholds[_mode];
    unsigned int fb_idx;
    unsigned short *fb_ptr = _filter_buffer;
    short delta;
    uint16_t min = 0;
    struct echo *e = _echoes;
    unsigned int n = 0, c = 0;

    enum search_state {
        CROSS_SEARCH,
        MAX_SEARCH,
        MIN_SEARCH,
    } search_state = CROSS_SEARCH;

    for (fb_idx = 0; fb_idx < _filter_buffer_size; fb_idx++) {
        delta = *fb_ptr - *thresholds;
        switch (search_state) {
        case CROSS_SEARCH:
        default:
            /* no echo if below thresholds */
            if (delta < 0)
                break;
            /* new search a new max of echo */
            search_state = MAX_SEARCH;
            c = n;
            e[c].start_idx = fb_idx;
            e[c].previous = 0x8000;
            e[c].d_echo = 0xFFF;
            e[c].max_value = *fb_ptr;
            e[c].max_idx = fb_idx;
            n++;
            ULOGD("CROSS_SEARCH found echo at idx %d"
                       " delta is %d = %d - %d",
                fb_idx,
                delta,
                *fb_ptr,
                *thresholds);
            if (n >= P7_US_MAX_ECHOES) {
                _nb_echoes = n;
                return 0;
            }

            break;
        case MAX_SEARCH:
            if (delta < 0) {
                /* value is lower than thresholds no echo */
                search_state = CROSS_SEARCH;
                e[c].stop_idx = fb_idx;
            } else if (*fb_ptr > e[c].max_value) {
                /* value is growing go on searching max */
                e[c].max_value = *fb_ptr;
                e[c].max_idx = fb_idx;
            } else if (*fb_ptr < e[c].max_value) {
                /* new value is lower search min of echo */
                search_state = MIN_SEARCH;
                e[c].stop_idx = fb_idx;
                min = *fb_ptr;
            }
            break;
        case MIN_SEARCH:
            if (*fb_ptr < min)
                min = *fb_ptr;

            if (delta < 0) {
                /* below thresholds no echo */
                search_state = CROSS_SEARCH;
                e[c].stop_idx = fb_idx;
            } else if (*fb_ptr > min) {
                /* more than min search max */
                search_state = MAX_SEARCH;
                c = n;
                e[c].start_idx = fb_idx;
                e[c].previous = 0x8000;
                e[c].d_echo = 0xFFF;
                e[c].max_value = *fb_ptr;
                e[c].max_idx = fb_idx;
                n++;
                ULOGD("MIN_SEARCH found echo at idx %d"
                        "delta is %d = %d - %d",
                        fb_idx,
                        delta,
                        *fb_ptr,
                        *thresholds);

                if (n >= P7_US_MAX_ECHOES) {
                    _nb_echoes = n;
                    return 0;
                }
            }
            break;
        }
        fb_ptr++;
        thresholds++;
    }
    _nb_echoes = n;
    return 0;
}

/*
 * p7_us_mach_previous_echoes
 * from us->adc.echoes and us->adc.echoes_old
 * try to find matching echoes
 * inherited from HAL_ultrasound_matching_echoes
 */
int AP_RangeFinder_AnalogSonar::match_echoes(void)
{
    int16_t d_echo, d_test_echo;
    int16_t thres_delta_idx = 4 * 2 * (_adcCapture->freq)
                                / (P7_US_SOUND_SPEED * _freq);
    uint8_t nb_echoes = _nb_echoes;
    uint8_t nb_echoes_old = _nb_echoes_old;

    uint8_t previous_echo_idx = 0;
    struct echo *p_detect = NULL;
    struct echo *p_echo_used, *p_echo_old_used;
    struct echo *p_end_echo_used, *p_end_echo_old_used;

    struct echo *p_previous_echo = NULL;
    struct echo *p_test_echo = NULL;

    p_echo_used = &_echoes[0];
    p_echo_old_used = &_echoes_old[0];

    p_end_echo_used = &_echoes[nb_echoes-1];
    p_end_echo_old_used = &_echoes_old[nb_echoes_old-1];

    /* To succeed in matching we need to have a list of current echoes and the
     * previous list of echoes, both non empty */
    ULOGD("nb_echoes %d nb_echoes_old %d", nb_echoes, nb_echoes_old);
    if (!nb_echoes || !nb_echoes_old) {
        while (p_end_echo_used >= p_echo_used) {
            p_echo_used->previous = 0xFFB;
            p_echo_used++;
        }
        return 0;
    }

    p_test_echo = p_echo_old_used + 1;

    /* we try to match each echo of the current list... */
    while (p_end_echo_used >= p_echo_used) {

        /* distance to the echo of the previous list. */
        d_echo = p_echo_old_used->max_idx - p_echo_used->max_idx;
        /* .. with the echoes of the previous list. */
        while (previous_echo_idx < nb_echoes_old) {
            /* We stop trying to match this current echo because all other
             * echoes of the previous list will be further (echoes are stored
             * in the lists from close to far.) */
            if (d_echo >= 0)
                break;

            /* distance to next echo of the previous list */
            d_test_echo = p_test_echo->max_idx -
                p_echo_used->max_idx;

            if (d_test_echo >= 0) {
                /* We find two echoes which surrounded p_echo_used */
                if (d_test_echo <= -d_echo) {
                    /*The second echo of the two is closer */
                    p_echo_old_used = p_test_echo++;
                    previous_echo_idx++;
                    d_echo = d_test_echo;

                    /* we have found the best match for the current echo so we
                     * don't need to keep this reference.  */
                    p_previous_echo = NULL;
                } else {
                    /* the first echo of the two is closer maybe next echo of
                     * the current list will be closer to the reference so we
                     * keep it. */
                    d_echo = -d_echo;
                }
                /* Matching is done for the current echo */
                break;
            } else {
                /* we have to go on looking for a closer echo in the previous
                 * list. */
                p_echo_old_used = p_test_echo++;
                previous_echo_idx++;
                d_echo = d_test_echo;
                /* we don't keep the reference on this echo of previous list. */
                p_previous_echo = NULL;
            }
        }

        /* We store the best match for the current echo and the delta */
        p_echo_used->d_echo = d_echo;
        p_echo_used->previous = previous_echo_idx;

        /* We check if the match is valid */
        if (p_echo_used->d_echo < thres_delta_idx) {
            if (p_detect == NULL)
                p_detect = p_echo_used;
            if (p_previous_echo == NULL) {
                p_previous_echo = p_echo_used;
                p_echo_used++;
                continue;
            }
            /* we compare new match with the reference */
            if (p_previous_echo->d_echo > p_echo_used->d_echo) {
                /* the previous echo is tagged as not so good */
                p_previous_echo->previous |=
                    ECHO_FOLLOWING_BETTER;
                ULOGD("FOLLOWING BETTER");
                /* if we are improving the match that was already recorded we
                 * replace it with the improved one. */
                if (p_detect == p_previous_echo)
                    p_detect = p_echo_used;
            } else {
                /* the current echo is tagged as not so good */
                p_echo_used->previous |= ECHO_PREVIOUS_BETTER;
                ULOGD("PREVIOUS BETTER");
            }
        } else {
            p_echo_used->previous |= ECHO_REJECTED;
            ULOGD("REJECTED");
        }

        /* before proceeding with next echo of the current list we record this
         * one as the new reference */
        p_previous_echo = p_echo_used;
        p_echo_used++;
    }
    if (p_detect)
        ULOGD("ECHO USED bei %d, eoi %d, mv %d, p 0x%x, de %d, pmi %d",
                p_detect->start_idx,
                p_detect->stop_idx,
                p_detect->max_value,
                p_detect->previous,
                p_detect->d_echo,
                p_detect->max_idx);
    return 0;
}

/*
 * inherited from HAL_ultrasound_recup_number_echoes
 * Calculate the real number of echoes
 */
void AP_RangeFinder_AnalogSonar::get_echoes(void)
{
    struct echo *echoes = _echoes;
    int i;
    uint8_t nb_peaks, nb_echoes;
    uint8_t nb_echoes_tempA, nb_echoes_tempB, nb_echoes_temp;
    struct echo *first_echo, *second_echo;
    int min = 2;
    first_echo = echoes;

    uint16_t threshold_time = echoes->start_idx
            + _adcCapture->threshold_time_rejection;


    nb_peaks  = nb_echoes_temp = _nb_echoes;
    if (nb_peaks > 0)
        nb_echoes = 1;

    for (i = 0; i < nb_peaks; i++) {
        if (echoes[i].start_idx >= threshold_time)
            nb_echoes_temp--;
    }

    if (nb_echoes_temp < min) {
        _nb_echoes = nb_echoes;
        return;
    }

    second_echo = echoes + 1;

    if (echo_linear_test(first_echo, second_echo)) {

        nb_echoes_tempA = nb_echoes_temp;
        nb_echoes_tempA--;

        for (i = min; i < nb_echoes_temp; i++) {
            if (echo_linear_test(first_echo, echoes+i))
                nb_echoes_tempA--;
        }
        nb_echoes = nb_echoes_tempA;
    } else {
        nb_echoes_tempA = nb_echoes_temp;

        for (i = min; i < nb_echoes_temp; i++) {
            if (echo_linear_test(first_echo, echoes+i))
                nb_echoes_tempA--;
        }

        nb_echoes_tempB = nb_echoes_temp;

        for (i = min; i < nb_echoes_temp; i++) {
            if (echo_linear_test(second_echo, echoes+i))
                nb_echoes_tempB--;
        }

        if (nb_echoes_tempA < nb_echoes_tempB)
            nb_echoes = nb_echoes_tempA;
        else
            nb_echoes = nb_echoes_tempB;
    }
    _nb_echoes = nb_echoes;
}

/*
 * p7_us_process_echoes
 * this processing was initially done in Colibry compute_alt_measure
 * It takes the list of echoes
 * search the best echo and process altitude
 */
int AP_RangeFinder_AnalogSonar::process_echoes(void)
{
    uint8_t i;
    uint8_t echo_max_idx = 0;
    uint8_t echo_tracked_idx = 0;
    uint8_t echo_selected_idx = 0;
    int32_t max_value = 0;
    int dist_min_tracked = 1000;
    int nb_echo_matched = 0;
    int max_value_integration = 0;
    struct echo *echo_max = &_echoes[0];
    struct echo *echo_tracked = echo_max;
    struct echo *echo_selected = echo_max;
    int dist;
    for (i = 0; i < _nb_echoes; i++) {
        struct echo *echo = &_echoes[i];
        if ((echo->previous & 0x20) != 0)
            continue;
        /* Test matching */
        nb_echo_matched++;
        if (echo->max_value > max_value) {
            /* Research echo max */
            max_value = echo->max_value;
            echo_max = echo;
            echo_max_idx = i;
        }
        if ((echo->previous & 0x1F) == echo_selected_idx) {
            /* Research previous selected echo in current list */
            dist = abs(echo->max_idx
                    - echo_selected->max_idx);
            if (dist < dist_min_tracked) {
                dist_min_tracked = dist;
                echo_tracked = echo;
                echo_tracked_idx = i;
            }
        }
    }

    if (nb_echo_matched <= 0) {
        ULOGI("No match");
        return 0;
    }
    if (dist_min_tracked == 1000) {
        /* We lost the previous echo, we select the max */
        echo_selected = echo_max;
        echo_selected_idx = echo_max_idx;
        max_value_integration = 0;
    } else if (echo_tracked->max_value == max_value) {
        /* Tracked echo is the max, we keep it */
        echo_selected = echo_tracked;
        echo_selected_idx = echo_tracked_idx;
        max_value_integration = 0;
    } else {
        /* Tracked echo is the not the max */
        max_value_integration +=
            max_value - echo_tracked->max_value;
        if (max_value_integration < 500) {
            /* We still keep it */
            echo_selected = echo_tracked;
            echo_selected_idx = echo_tracked_idx;
        } else {
            /* Too far, we select the max */
            echo_selected = echo_max;
            echo_selected_idx = echo_max_idx;
            max_value_integration = 0;
        }
    }
    _altitude =
        (float)(echo_selected->max_idx * P7_US_SOUND_SPEED)
        / (2 * (P7_US_DEFAULT_ADC_FREQ >> P7_US_FILTER_POWER));
    ULOGD("final alt %f", _altitude);
    _echo_selected = echo_selected;
    return 0;
}

bool AP_RangeFinder_AnalogSonar::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}

void AP_RangeFinder_AnalogSonar::update(void)
{
    printf("%s\n", __func__);
    _ultrasound->launch();
    _ultrasound->capture();

    _adcCapture = _ultrasound->get_capture();

    apply_filter();
    search_echoes();
    match_echoes();
    get_echoes();
    process_echoes();

    memcpy(_echoes_old, _echoes, P7_US_MAX_ECHOES * sizeof(struct echo));
    _nb_echoes_old = _nb_echoes;
    state.distance_cm = (uint16_t) (_altitude * 100);
#ifdef RANGEFINDER_LOG
    _log.step();
#endif
    update_status();
    ULOGI("distance_cm : %u", ranger.distance_cm());
    _mode = _ultrasound->update_mode(ranger.distance_cm() * 100);
}

RangeFinder_Log::RangeFinder_Log(const AP_RangeFinder_AnalogSonar &ranger):
        _rangefinder(ranger)
{
    const char *res_path = "/data/ftp/internal_000/log/us.log";
    _fd = open(res_path, O_RDWR | O_CREAT | O_TRUNC,
            S_IRWXU | S_IRWXG | S_IRWXO);
    _cpt = 0;
}
RangeFinder_Log::~RangeFinder_Log()
{
    close(_fd);
}
void RangeFinder_Log::step()
{
    unsigned int i;
    int size;
    unsigned int tot_size = 0;
    unsigned int nb_filter_sample = 8192 >> P7_US_FILTER_POWER;
    unsigned int sample_size = 25;
    char buffer[nb_filter_sample * sample_size];

    memset(buffer, 0, sizeof(buffer));
    for (i = 0; i < nb_filter_sample; i++) {
        if (_rangefinder._filter_buffer[i] > 0 || sThresholds[_rangefinder._mode][i] > 1200) {
            if (_rangefinder._echo_selected
                && i == _rangefinder._echo_selected->max_idx) {
                tot_size += snprintf(&buffer[tot_size],
                    sizeof(buffer),
                    "%d %d %d %d %f\n",
                    _cpt,
                    sThresholds[_rangefinder._mode][i],
                    _rangefinder._filter_buffer[i],
                    _rangefinder._echo_selected->max_value,
                    _rangefinder._altitude * 10000);
            } else {
                tot_size += snprintf(&buffer[tot_size],
                        sizeof(buffer),
                        "%d %d %d 0 0.0\n",
                        _cpt,
                        sThresholds[_rangefinder._mode][i],
                        _rangefinder._filter_buffer[i]);
            }
            _cpt++;
        }
    }

    i = 0;
    while (tot_size > 0) {
        size = write(_fd, &buffer[i], tot_size);
        if (size < 0)
            break;
        else {
            tot_size -= size;
            i += size;
        }
    }
}
