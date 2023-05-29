#include "misc.h"

#include <cmath>

namespace Misc {

double pos_fmod(const double num, const double mod) {
    return std::fmod(std::fmod(num, mod) + mod, mod);
}

} // namespace Misc
