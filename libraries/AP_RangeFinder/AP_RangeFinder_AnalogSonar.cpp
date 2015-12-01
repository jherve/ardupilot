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
#include <pthread.h>

#include <stdio.h>
#include <errno.h>
#include "AP_HAL_Linux/UltraSound_Bebop.h"

#include "AP_HAL_Linux/IIO.h"

extern const AP_HAL::HAL& hal;

#define LOG(_fmt, ...)   fprintf(stdout, _fmt "\n", ##__VA_ARGS__)
/** Log as debug */
#define LOGD(_fmt, ...)  //LOG("**** [D]" _fmt, ##__VA_ARGS__)
/** Log as info */
#define LOGI(_fmt, ...)  LOG("**** [I]" _fmt, ##__VA_ARGS__)
/** Log as warning */
#define LOGW(_fmt, ...)  LOG("**** [W]" _fmt, ##__VA_ARGS__)
/** Log as error */
#define LOGE(_fmt, ...)  LOG("**** [E]" _fmt, ##__VA_ARGS__)

unsigned short AP_RangeFinder_AnalogSonar::sWaveformMode0[14] = {
    4000, 3800, 3600, 3400, 3200, 3000, 2800,
    2600, 2400, 2200, 2000, 1800, 1600, 1400,
};
unsigned short AP_RangeFinder_AnalogSonar::sWaveformMode1[32] = {
    4190, 4158, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4090, 4058, 3943, 3924, 3841, 3679, 3588, 3403,
    3201, 3020, 2816, 2636, 2448, 2227, 2111, 1955, 1819,
    1675, 1540, 1492, 1374, 1292
};

AP_RangeFinder_AnalogSonar::AP_RangeFinder_AnalogSonar(RangeFinder &_ranger,
                                        uint8_t instance,
                                        RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    _last_max_distance_cm(850),
    _last_min_distance_cm(32),
    _last_timestamp(0),
#ifdef RANGEFINDER_LOG
    _log(*this),
#endif
    _ultrasound(get_HAL_Linux().ultraSound),
    _adcCapture(NULL),
    _mode(0),
    _echoesNb(0),
    _altitude(0.0f),
    _thread(NULL),
    _filterAverage(4)
{
    LOGI("AP_RangeFinder_AnalogSonar::AP_RangeFinder_AnalogSonar");
    _freq = P7_US_DEFAULT_FREQ;
    _filteredCaptureSize = _ultrasound->get_buffer_size() / _filterAverage;
    _filteredCapture = (unsigned int*) calloc(1, sizeof(_filteredCapture[0]) * _filteredCaptureSize);
    LOGI("_ultraSound init : %x", &_ultrasound);

}

AP_RangeFinder_AnalogSonar::~AP_RangeFinder_AnalogSonar()
{
}

bool AP_RangeFinder_AnalogSonar::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}

unsigned short AP_RangeFinder_AnalogSonar::getThresholdAt(int iCapture)
{
    unsigned short thresholdValue = 0;

    /*
     * We define several kinds of thresholds signals ; for an echo to be
     * recorded, its maximum amplitude has to be ABOVE that threshold.
     * There is one kind of threshold per mode (mode 0 is "low" and mode 1 is
     * "high")
     * Basically they look like this :
     *
     *            on this part
     *            of the capture
     *  amplitude  we use
     *      ^     the waveform
     *      |     <---------->
     * 4195 +-----+
     *      |
     *      |
     *      |
     *      |
     *  1200|                 +----------------+
     *    +-------------------------------------->
     *      +    low         high               time
     *           limit       limit
     *
     *  */
    switch (_mode) {
    case 0:
        if (iCapture < 139)
            thresholdValue = 4195;
        else if (iCapture < 153)
            thresholdValue = sWaveformMode0[iCapture - 139];
        else
            thresholdValue = 1200;
        break;

    case 1:
        if (iCapture < 73)
            thresholdValue = 4195;
        else if (iCapture < 105)
            thresholdValue = sWaveformMode1[iCapture - 73];
        else if (iCapture < 617)
            thresholdValue = 1200;
        else
            thresholdValue = 4195;
        break;

    default:
        break;
    }

    return thresholdValue;
}

int AP_RangeFinder_AnalogSonar::applyAveragingFilter(void)
{

    int iFilter = 0; /* index in the filtered buffer */
    int iCapture = 0; /* index in the capture buffer : starts incrementing when
                            the captured data first exceeds
                            P7_US_THRESHOLD_ECHO_INIT */
    unsigned int filteredValue = 0;
    bool firstEcho = false;
    unsigned char *data;
    unsigned char *start;
    unsigned char *end;
    ptrdiff_t step;

    step = iio_buffer_step(_adcCapture->buffer);
    end = (unsigned char *) iio_buffer_end(_adcCapture->buffer);
    start = (unsigned char *) iio_buffer_first(_adcCapture->buffer,
                                                    _adcCapture->channel);

    for (data = start; data < end; data += step) {
        unsigned int currentValue = 0;
        iio_channel_convert(_adcCapture->channel, &currentValue, data);

        /* We keep on advancing in the captured buffer without registering the
         * filtered data until the signal first exceeds a given value */
        if (!firstEcho && currentValue < thresholdEchoInit)
            continue;
        else
            firstEcho = true;

        filteredValue += currentValue;
        if (iCapture % _filterAverage == 0) {
            _filteredCapture[iFilter] = filteredValue / _filterAverage;
            filteredValue = 0;
            iFilter++;
        }
        iCapture++;
    }
    return 0;
}

int AP_RangeFinder_AnalogSonar::searchLocalMaxima(void)
{
    int iEcho = 0; /* index in echo array */

    for (int iCapture = 1; iCapture < (int)_filteredCaptureSize - 1; iCapture++) {
        if (_filteredCapture[iCapture] > getThresholdAt(iCapture)) {
            unsigned short curr = _filteredCapture[iCapture];
            unsigned short prev = _filteredCapture[iCapture - 1];
            unsigned short next = _filteredCapture[iCapture + 1];

            if (curr > prev && curr > next) {
                _echoes[iEcho].maxIndex = iCapture;
                LOGD("found local max at index %d : %d", iCapture, curr);
                iEcho++;
                if (iEcho >= P7_US_MAX_ECHOES)
                    break;
            }
        }
    }

    _echoesNb = iEcho;
    return 0;
}

int AP_RangeFinder_AnalogSonar::searchMaximaDistance(void)
{
    int threshold = 50;

    for (int iEcho = 0; iEcho < _echoesNb ; iEcho++) {
        int iCapture = _echoes[iEcho].maxIndex; /* index in capture buffer */
        unsigned short c_value;
        unsigned short target = _filteredCapture[iCapture] - threshold;
        do {
            iCapture--;
            c_value = _filteredCapture[iCapture];
        } while(c_value >= target);
        LOGD("Found crossing at %d : %d", iCapture, _filteredCapture[iCapture]);
        _echoes[iEcho].distanceIndex = iCapture;
    }

    return 0;
}

int AP_RangeFinder_AnalogSonar::searchMaximumWithMaxAmplitude(void)
{
    unsigned short max = 0;
    int maxIdx = -1;

    for (int iEcho = 0; iEcho < _echoesNb ; iEcho++) {
        unsigned short curr = _filteredCapture[_echoes[iEcho].maxIndex];
        if (curr >= max) {
            max = curr;
            maxIdx = iEcho;
        }
    }

    if (maxIdx >= 0)
        return _echoes[maxIdx].distanceIndex;
    else
        return -1;
}

void AP_RangeFinder_AnalogSonar::loop(void)
{
    static bool outOfFirstLoop = false;
    int maxIndex;

    while(1) {
        /* Wait for the semaphore that will be given when the "update" function
         * is over */
        _semCapture.take(0);
        int32_t start = AP_HAL::get_HAL().scheduler->micros();
        int32_t launch, capture, average, localmax, maxdist, maxwithmax, update, end;
        _ultrasound->launch();
        launch = AP_HAL::get_HAL().scheduler->micros() - start;
        _ultrasound->capture();
        _adcCapture = _ultrasound->get_capture();

        capture = AP_HAL::get_HAL().scheduler->micros() - start;

        if (applyAveragingFilter() < 0) {
            LOGW("Could not apply averaging filter");
            goto endloop;
        }
        average = AP_HAL::get_HAL().scheduler->micros() - start;
        if (searchLocalMaxima() < 0) {
            LOGW("Did not find any local maximum");
            goto endloop;
        }
        localmax = AP_HAL::get_HAL().scheduler->micros() - start;
        if (searchMaximaDistance() < 0) {
            LOGW("Maxima distance did not find anything");
            goto endloop;
        }
        maxdist = AP_HAL::get_HAL().scheduler->micros() - start;

        maxIndex = searchMaximumWithMaxAmplitude();
        maxwithmax = AP_HAL::get_HAL().scheduler->micros() - start;
        LOGD("Index of max : %d", maxIndex);
        if (maxIndex >= 0) {
            _altitude = (float)(maxIndex * P7_US_SOUND_SPEED) / (2 * (P7_US_DEFAULT_ADC_FREQ / _filterAverage));
            _mode = _ultrasound->update_mode(_altitude);
        }
        update = AP_HAL::get_HAL().scheduler->micros() - start;

endloop:

#ifdef RANGEFINDER_LOG
        _log.step();
#endif
        end = AP_HAL::get_HAL().scheduler->micros() - start;
        //_semAltitude.give();
        LOGI("final alt %f, mode = %d", _altitude, _mode);
        LOGI("launch %d, capture %d, average %d, localmax %d, maxdist %d, maxwithmax %d, update %d, end %d", launch, capture, average, localmax, maxdist, maxwithmax, update, end);
    }
}

void* AP_RangeFinder_AnalogSonar::thread(void* arg)
{
    AP_RangeFinder_AnalogSonar *sonar = (AP_RangeFinder_AnalogSonar*) arg;

    sonar->loop();

    return NULL;
}

void AP_RangeFinder_AnalogSonar::update(void)
{
    static bool firstCall = true;
    int32_t start = AP_HAL::get_HAL().scheduler->micros();

    if (firstCall) {
        int ret;
        pthread_attr_t attr;
        struct sched_param param = { 14 };

        ret = pthread_attr_init(&attr);
        if (ret != 0) {
            perror("RCOut_Bebop: failed to init attr\n");
            return;
        }
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &param);
        ret = pthread_create(&_thread, &attr, AP_RangeFinder_AnalogSonar::thread, this);
        LOGI("AP_RangeFinder_AnalogSonar::AP_RangeFinder_AnalogSonar : %d", ret);
        firstCall = false;
    }

    //_semAltitude.take(0);
    state.distance_cm = (uint16_t) (_altitude * 100);
    update_status();
    /* Let the secondary thread launch the ultrasound capture */
    _semCapture.give();
    int32_t end = AP_HAL::get_HAL().scheduler->micros() - start;
    LOGI("******************************************** update took %d", end);
}

#ifdef RANGEFINDER_LOG
RangeFinder_Log::RangeFinder_Log(AP_RangeFinder_AnalogSonar &ranger):
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
    int i;
    int size;
    unsigned int tot_size = 0;
    unsigned int nb_filter_sample = 8192 >> P7_US_FILTER_POWER;
    unsigned int sample_size = 25;
    char buffer[nb_filter_sample * sample_size];

    memset(buffer, 0, sizeof(buffer));
    for (i = 0; i < nb_filter_sample; i++) {
        if (_rangefinder._filteredCapture[i] > 0 || (unsigned short)_rangefinder.getThresholdAt(i) > 1200) {
            if (i == _rangefinder.searchMaximumWithMaxAmplitude()) {
                tot_size += snprintf(&buffer[tot_size],
                    sizeof(buffer),
                    "%d %d %d %d %f\n",
                    _cpt,
                    _rangefinder.getThresholdAt(i),
                    _rangefinder._filteredCapture[i],
                    i,
                    _rangefinder._altitude * 10000);
            } else {
                tot_size += snprintf(&buffer[tot_size],
                        sizeof(buffer),
                        "%d %d %d 0 0.0\n",
                        _cpt,
                        _rangefinder.getThresholdAt(i),
                        _rangefinder._filteredCapture[i]);
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
#endif
