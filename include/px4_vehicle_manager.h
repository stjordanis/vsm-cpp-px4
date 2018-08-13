// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file px4_vehicle_manager.h
 */
#ifndef _PX4_VEHICLE_MANAGER_H_
#define _PX4_VEHICLE_MANAGER_H_

#include <mavlink_vehicle_manager.h>

class Px4_vehicle_manager: public Mavlink_vehicle_manager {
    DEFINE_COMMON_CLASS(Px4_vehicle_manager, Mavlink_vehicle_manager)

public:
    /** Constructor. */
    Px4_vehicle_manager();

private:
    virtual Mavlink_vehicle::Ptr
    Create_mavlink_vehicle(
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Socket_address::Ptr,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            const std::string& serial_number,
            const std::string& model_name) override;

    virtual void
    Register_detectors() override;
};

#endif /* _PX4_VEHICLE_MANAGER_H_ */
