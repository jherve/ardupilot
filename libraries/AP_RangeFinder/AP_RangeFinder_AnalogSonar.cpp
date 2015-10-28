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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>
#include "AP_HAL_Linux/UltraSound_Bebop.h"

extern const AP_HAL::HAL& hal;


AP_RangeFinder_AnalogSonar::AP_RangeFinder_AnalogSonar(RangeFinder &_ranger,
                                        uint8_t instance,
                                        RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    _last_max_distance_cm(-1),
    _last_min_distance_cm(-1),
    _last_timestamp(0),
    _fd(-1)
{

}

/*
   close the file descriptor
*/
AP_RangeFinder_AnalogSonar::~AP_RangeFinder_AnalogSonar()
{
}

/*
   see if the PX4 driver is available
*/
bool AP_RangeFinder_AnalogSonar::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}

void AP_RangeFinder_AnalogSonar::update(void)
{
    AP_HAL_Linux.ultraSound->launch();
}


