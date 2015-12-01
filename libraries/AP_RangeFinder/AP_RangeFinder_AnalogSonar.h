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

#ifndef AP_RangeFinder_ANALOG_SONAR_H
#define AP_RangeFinder_ANALOG_SONAR_H

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Linux/Semaphores.h>
#include <pthread.h>

#define RANGEFINDER_LOG

/*
 * the number of echoes we will keep at most
 */
#define P7_US_MAX_ECHOES 30


class AP_RangeFinder_AnalogSonar;

struct echo {
    int maxIndex; /* index in the capture buffer at which the maximum is reached */
    int distanceIndex; /* index in the capture buffer at which the signal is for
                            the first time above a fixed threshold below the
                            maximum => this corresponds to the real distance
                            that should be attributed to this echo */
};

#ifdef RANGEFINDER_LOG
class RangeFinder_Log {
private:
    int _fd;
    int _cpt;
    AP_RangeFinder_AnalogSonar &_rangefinder;
public:
    RangeFinder_Log(AP_RangeFinder_AnalogSonar &ranger);
    ~RangeFinder_Log();
    void step();
};
#endif

class AP_RangeFinder_AnalogSonar : public AP_RangeFinder_Backend
{
#ifdef RANGEFINDER_LOG
    friend class RangeFinder_Log;
#endif
public:
    // constructor
    AP_RangeFinder_AnalogSonar(RangeFinder &ranger,
                                    uint8_t instance,
                                    RangeFinder::RangeFinder_State &_state);

    // destructor
    ~AP_RangeFinder_AnalogSonar(void);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:
    const unsigned int thresholdEchoInit = 1500;
    static unsigned short sWaveformMode0[14];
    static unsigned short sWaveformMode1[32];

#ifdef RANGEFINDER_LOG
    RangeFinder_Log _log;
#endif
    UltraSound_Bebop *_ultrasound;
    struct adcCapture_t *_adcCapture;
    uint64_t _last_timestamp;

    int _mode;
    int _echoesNb;
    int _freq;
    float _altitude;
    Linux::Semaphore _semCapture;
    Linux::Semaphore _semMeasure;
    pthread_t _thread;

    unsigned int *_filteredCapture;
    unsigned int _filteredCaptureSize;
    struct echo _echoes[P7_US_MAX_ECHOES];
    unsigned int _filterAverage;

    unsigned short getThresholdAt(int iCapture);
    int applyAveragingFilter(void);
    int searchLocalMaxima(void);
    int searchMaximaDistance(void);
    int searchMaximumWithMaxAmplitude(void);
    void loop(void);
    static void* thread(void* arg);

    int16_t _last_max_distance_cm;
    int16_t _last_min_distance_cm;
};

#endif // AP_RangeFinder_ANALOG_SONAR_H
