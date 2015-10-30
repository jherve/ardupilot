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

/*
 * the number of echoes we will keep at most
 */
#define P7_US_MAX_ECHOES 30

/*
 * value for echo processing
 */
#define P7_US_THRESHOLD_ECHO_INIT 1500

/*
 * struct related to ultrasound echo
 */
struct echo {
    uint16_t start_idx;
    uint16_t stop_idx;
    int32_t max_value;
    uint16_t max_idx;
    uint16_t previous;
    int16_t d_echo;
};


class AP_RangeFinder_AnalogSonar : public AP_RangeFinder_Backend
{
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
    UltraSound_Bebop *_ultrasound;
    struct adcCapture_t *_adcCapture;
    int _fd;
    uint64_t _last_timestamp;
    unsigned short *_filter_buffer;
    unsigned int _filter_buffer_size;
    int _mode;
    int _nb_echoes;
    int _nb_echoes_old;
    int _freq;
    float _altitude;

    struct echo _echoes[P7_US_MAX_ECHOES];
    struct echo _echoes_old[P7_US_MAX_ECHOES];
    struct echo *_echo_selected;

    int apply_filter(void);
    int search_echoes(void);
    int match_echoes(void);
    void get_echoes(void);
    int process_echoes(void);
    uint8_t echo_linear_test(struct echo *echoA, struct echo *echoB);

    int16_t _last_max_distance_cm;
    int16_t _last_min_distance_cm;
};

#endif // AP_RangeFinder_ANALOG_SONAR_H
