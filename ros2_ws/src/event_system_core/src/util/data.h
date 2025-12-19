#include <map>
#include <string>
#include <vector>
#include "types.h"


inline std::map<std::string, std::vector<std::string>> employeeLocations = {
    {"Max", {"5.2B03","5.2B10" , "Kitchen", "Open Zone"}},
    {"Leo", {"5.2B15", "5.2B10", "5.2B33", "Kitchen"}},
    {"Fred", {"Open Zone", "Kitchen"}},
    {"Luici", {"5.2B18", "Kitchen", "Open Zone", "5.2B33"}},
    {"Anna",  {"5.2B01", "5.2B03", "Open Zone"}},
    {"Tom",   {"5.2B03", "5.2B05", "Printer"}},
    {"Sarah", {"5.2B13", "5.2B15", "Kitchen"}},
    {"Ben",   {"5.2B01", "5.2B18", "Floor"}},
    {"Lisa",  {"5.2B04", "5.2B18", "Open Zone"}},
    {"Kevin", {"5.2B10", "Printer", "Kitchen"}},
    {"Julia", {"5.2B01", "5.2B05", "5.2B31"}}
};


inline std::map<std::string, des::Point> locationMap = {
    // ROS coordinates
    {"0/0", {0.0, 0.0}},

    // imvs left side
    {"5.2B01", {-17.0, -8.0}},
    {"5.2B03", {-14.0, -2.2}},
    {"5.2B04", {-12.4, 0.8}},
    {"5.2B05", {-10.6, 4.0}},

    // imvs front side
    {"5.2B10", { -4.0, 4.8}},
    {"5.2B13", { 1.75, 3.3}},
    {"5.2B15", { 7.6 , 2.0}},
    {"5.2B16", { 14.3, 0.3}},
    {"5.2B18", { 19.0, -0.7}},

    // imvs meeting rooms 
    {"5.2B31", {-8.2, -1.2}},
    {"5.2B33", {-4.8, -1.2}},
    {"5.2B34", {-1.3, -1.2}},

    // Special Locations
    {"Dock",      {-4.66, 2.75}},
    {"Printer",   { 6.0 , -3.0}},
    {"Open Zone", {-9.2 , 7.6,}},
    {"Kitchen",   {-11.6, -8.0}},
    {"Floor",     {-7.8 , 1.6}}
};
