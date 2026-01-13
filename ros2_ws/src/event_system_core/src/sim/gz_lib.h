#pragma once

#include <gz/transport.hh>
#include <gz/msgs.hh>

namespace sim {
    inline void getServices() {
        gz::transport::Node node;
        std::vector<std::string> services;
        node.ServiceList(services);
        std::cout << "Available Services:" << std::endl;
        for (const auto &service: services) {
            std::cout << "  " << service << std::endl;
        }
    }

    inline void moveRobot(const double x, const double y) {
        gz::transport::Node node;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        gz::msgs::Pose req;
        req.set_name("dingo");
        req.mutable_position()->set_x(x);
        req.mutable_position()->set_y(y);
        req.mutable_position()->set_z(0.1);
        req.mutable_orientation()->set_w(1.0);


        gz::msgs::Boolean rep;
        bool result;
        bool executed = node.Request("/world/fhnw/set_pose", req, 5000, rep, result);
        if (executed) {
            if (!result)
                std::cout << "Service call failed" << std::endl;
        } else
            std::cerr << "Service call timed out" << std::endl;
    }


    inline void addMarker(const std::string id, const double x, const double y, const double z, const double r = 0.2) {
        gz::transport::Node node;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::ostringstream ss;

        ss << "<?xml version=\"1.0\"?>\n";
        ss << "<sdf version='1.7'>\n";
        ss << "  <model name='" << id << "'>\n";
        ss << "    <pose>" << x << " " << y << " " << z << " 0 0 " << -M_PI_2 << " </pose>\n";
        ss << "    <link name='link'>\n";
        ss << "      <visual name='visual'>\n";
        ss << "        <geometry>\n";
        ss << "          <sphere><radius>" << r << "</radius></sphere>\n";
        ss << "        </geometry>\n";
        ss << "        <material>\n";
        ss << "          <ambient>1 0 0 1</ambient>\n";
        ss << "          <diffuse>1 0 0 1</diffuse>\n";
        ss << "        </material>\n";
        ss << "      </visual>\n";
        ss << "      <collision name='collision'>\n";
        ss << "        <geometry>\n";
        ss << "          <sphere><radius>" << r << "</radius></sphere>\n";
        ss << "        </geometry>\n";
        ss << "      </collision>\n";
        ss << "    </link>\n";
        ss << "  </model>\n";
        ss << "</sdf>\n";

        bool result;
        gz::msgs::EntityFactory req;
        gz::msgs::Boolean res;
        req.set_sdf(ss.str());


        bool executed = node.Request("/world/fhnw/create", req, 1000, res, result);
        if (executed)
        {
            if (result)
                std::cout << "Entity was created : [" << res.data() << "]" << std::endl;
            else
            {
                std::cout << "Service call failed" << std::endl;
                return;
            }
        }
        else
            std::cerr << "Service call timed out" << std::endl;
        }
}




