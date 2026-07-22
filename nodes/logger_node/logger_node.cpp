#include <fstream>
#include <filesystem>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>

#include "camera_tracking.pb.h"

int main(int argc, char** argv)
{
    eCAL::Initialize(argc, argv, "logger_node");

    std::ofstream csv("validation.csv");

    csv << "timestamp,valid,fanuc_is_stub,error_mm,"
           "error_x_mm,error_y_mm,error_z_mm,"
           "fanuc_x,fanuc_y,fanuc_z,"
           "camera_x,camera_y,camera_z\n";

    eCAL::protobuf::CSubscriber<camera_tracking::ValidationPacket>
        sub("hand/validation");

    sub.AddReceiveCallback(
        [&](const char*,
            const camera_tracking::ValidationPacket& msg,
            long long,
            longg
        {
            csv
                << msg.timestamp() << ","
                << msg.valid() << ","
                << msg.fanuc_is_stub() << ","
                << msg.error_mm() << ","
                << msg.error_x_mm() << ","
                << msg.error_y_mm() << ","
                << msg.error_z_mm() << ","
                << msg.pose_fanuc().pos_x() << ","
                << msg.pose_fanuc().pos_y() << ","
                << msg.pose_fanuc().pos_z() << ","
                << msg.pose_camera().pos_x() << ","
                << msg.pose_camera().pos_y() << ","
                << msg.pose_camera().pos_z()
                << "\n";
        });

    while (eCAL::Ok())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }

    eCAL::Finalize();
}