#include "pnc_map_server.h"

namespace Planning
{
    PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "pnc_map_server_node init");

        //地图发布器    
        map_pub_ = this->create_publisher<PNCMap>("pnc_map", 10);
        map_rviz_pub_ = this->create_publisher<MarkerArray>("pnc_map_markerarray", 10);


        //地图创建器
        map_server_ = this->create_service<PNCMapService>(
            "pnc_map_service",
            std::bind(&PNCMapServer::response_pnc_map_callback, this,
                     _1 , _2));
    }

    void PNCMapServer::response_pnc_map_callback(
        const std::shared_ptr<PNCMapService::Request> request,
        const std::shared_ptr<PNCMapService::Response> response)
    {
        //接收请求
        switch (request->map_type)
        {
        case static_cast<int>(PNCMapType::STRAIGHT):
            map_creator_ = std::make_shared<PNCMapCreatorStraight>();
            break;
        case static_cast<int>(PNCMapType::STURN):
            map_creator_ = std::make_shared<PNCMapCreatorSTurn>();
            break;
        
        default:
            break;
        }

        //创建地图
        const auto pnc_map = map_creator_->create_pnc_map();

        //发布地图

        //发布rviz可视化

    }

} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
