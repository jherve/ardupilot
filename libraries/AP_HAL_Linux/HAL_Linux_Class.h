
#ifndef __AP_HAL_LINUX_CLASS_H__
#define __AP_HAL_LINUX_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux_Namespace.h"
#include "UltraSound_Bebop.h"

class HAL_Linux : public AP_HAL::HAL {
public:
    UltraSound_Bebop *ultraSound;
    HAL_Linux();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

HAL_Linux& get_HAL_Linux();

extern const HAL_Linux AP_HAL_Linux;

#endif // __AP_HAL_LINUX_CLASS_H__

