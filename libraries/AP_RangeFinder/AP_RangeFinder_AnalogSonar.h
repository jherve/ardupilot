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
    int _fd;
    uint64_t _last_timestamp;

    int16_t _last_max_distance_cm;
    int16_t _last_min_distance_cm;
};

#endif // AP_RangeFinder_ANALOG_SONAR_H
