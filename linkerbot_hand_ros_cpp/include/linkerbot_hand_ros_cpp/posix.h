#pragma once

// POSIX extensions have no corresponding std:: declarations. Keep this C API
// boundary separate from C++ standard-library include recommendations.
#include <stdlib.h>

namespace linkerbot_hand_ros_cpp {
    inline int make_temporary_file(char *pattern) { return ::mkstemp(pattern); }
    inline int set_environment(const char *name, const char *value, int overwrite) {
        return ::setenv(name, value, overwrite);
    }
}
