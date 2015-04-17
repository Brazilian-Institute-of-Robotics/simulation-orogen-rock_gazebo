#ifndef rock_gazebo_TYPES_HPP
#define rock_gazebo_TYPES_HPP

#include <iostream>

namespace rock_gazebo
{
    struct LinkExport {
        // The port name
        std::string port_name;
        // The RBS sourceFrame, leave empty to use source_link
        std::string source_frame;
        // The RBS targetFrame, leave empty to use target_link
        std::string target_frame;
        // The source gazebo link, leave empty for "world"
        std::string source_link;
        // The target gazebo link, leave empty for "world"
        std::string target_link;
    };
}

#endif
