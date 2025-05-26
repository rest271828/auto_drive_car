#ifndef PNC_MAP_CERATOR_STRAIGHT_H_
#define PNC_MAP_CERATOR_STRAIGHT_H_

#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorStraight : public PNCMapCreatorBase
    {
    public:
        PNCMapCreatorStraight();
        PNCMap create_pnc_map() override;

    private:

    };
}  // namespace /* namespace_name */
#endif  // PNC_MAP_CERATOR_STRAIGHT_H_