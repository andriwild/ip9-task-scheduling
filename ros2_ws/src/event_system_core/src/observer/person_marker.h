#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "observer.h"
#include "engine/contracts/i_sim_context.h"

class PersonMarkerObserver final : public IObserver {
public:
    PersonMarkerObserver(const rclcpp::Node::SharedPtr& node, des::RoomMap rooms)
        : m_rooms(std::move(rooms))
    {
        auto qos = rclcpp::QoS(1).transient_local();
        m_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", qos);
    }

    void attach(const IPersonRegistry* ctx) {
        m_ctx = ctx;

        visualization_msgs::msg::MarkerArray wipe;
        visualization_msgs::msg::Marker clearAll;
        clearAll.header.frame_id = "map";
        clearAll.ns = "persons";
        clearAll.action = visualization_msgs::msg::Marker::DELETEALL;
        wipe.markers.push_back(clearAll);
        m_publisher->publish(wipe);
    }

    std::string getName() override {
        return "PersonMarker";
    }

    void onEvent(const int /*time*/, des::EventType type, const std::string& /*message*/, bool /*isDriving*/, bool /*isCharging*/, const std::string& /*color*/ = "", int /*missionId*/ = -1) override {
        switch (type) {
            case des::EventType::SIMULATION_START:
            case des::EventType::RESET:
            case des::EventType::PERSON_TRANSITION:
            case des::EventType::PERSON_ARRIVED:
            case des::EventType::PERSON_DEPARTURE:
            case des::EventType::PERSON_ROOM_ARRIVED:
            case des::EventType::PERSON_ACCOMPANY_DEPARTURE:
            case des::EventType::PERSON_ACCOMPANY_ARRIVED:
                publishMarkers();
                break;
            default:
                break;
        }
    }

private:
    void publishMarkers() {
        if (m_ctx == nullptr) {
            return;
        }

        visualization_msgs::msg::MarkerArray markers;

        for (const auto& person : m_ctx->getAllPersons()) {
            const std::string& name = person->firstName;
            auto marker = baseMarker(person->id);

            const auto pos = m_ctx->getPersonPosition(name);
            if (!pos) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
                markers.markers.push_back(marker);
                continue;
            }

            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pos->m_x;
            marker.pose.position.y = pos->m_y;
            marker.pose.position.z = 0.2;
            marker.pose.orientation.w = 1.0;
            marker.color = colorFromHex(person->color);
            markers.markers.push_back(marker);
        }

        m_publisher->publish(markers);
    }

    static visualization_msgs::msg::Marker baseMarker(const int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "persons";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        return marker;
    }

    static std_msgs::msg::ColorRGBA colorFromHex(const std::string& hex) {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.5f;
        color.g = 0.5f;
        color.b = 0.5f;
        color.a = 1.0f;

        const bool wellFormed = hex.size() == 7 && hex.front() == '#'
            && std::all_of(hex.begin() + 1, hex.end(), [](const unsigned char c) { return std::isxdigit(c); });
        if (wellFormed) {
            color.r = static_cast<float>(std::stoi(hex.substr(1, 2), nullptr, 16)) / 255.0f;
            color.g = static_cast<float>(std::stoi(hex.substr(3, 2), nullptr, 16)) / 255.0f;
            color.b = static_cast<float>(std::stoi(hex.substr(5, 2), nullptr, 16)) / 255.0f;
        }
        return color;
    }

    const IPersonRegistry* m_ctx = nullptr;
    des::RoomMap m_rooms;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_publisher;
};
