#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorSTurn : public PNCMapCreatorBase // S弯道地图
    {
    public:
        PNCMapCreatorSTurn();
        PNCMap create_pnc_map() override;                                              // 创建pnc_map的纯虚函数

    private:
        void init_pnc_map(); // 初始化pnc_map
        void draw_straight_x(const double &length, const double &plus_flag, const double &ratio = 1.0); // 
        void draw_rac(const double &angle, const double &plus_flag, const double &ratio = 1.0); // 画弧线,angle为弧线的总转角,plus_flag为弧线的转向(1为左转,-1为右转),ratio为弧线的半径与道路宽度的比值
    };
} // namespace Planning
#endif // PNC_MAP_CREATOR_STURN_H_