#include "file-path.h"

namespace path
{
    std::string resource(std::string str)
    {
        return RESOURCE_DIR + str;
    }
}