#pragma once
#include <cstdlib>
#include <string>
namespace ament_index_cpp
{
    // 离线仿真: 由环境变量 SIM_SHARE_DIR 指向 src/planning
    inline std::string get_package_share_directory(const std::string &)
    {
        const char *d = std::getenv("SIM_SHARE_DIR");
        return d ? d : "src/planning";
    }
}
