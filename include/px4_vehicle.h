// Copyright (c) 2017, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file px4_vehicle.h
 */
#ifndef _PX4_VEHICLE_H_
#define _PX4_VEHICLE_H_

#define DIRECT_PAYLOAD_CONTROLLING_COEF 10

#define MODEL_TYPHOON_H520 6021

#include <mavlink_vehicle.h>

#define PX4_VERSION(maj, min, patch) ((maj << 24) + (min << 16) + (patch << 8))

/** Vehicle supporting PX4 specific flavor of Mavlink. */
class Px4_vehicle: public Mavlink_vehicle {
    DEFINE_COMMON_CLASS(Px4_vehicle, Mavlink_vehicle)

public:
    static constexpr int RC_FS_DISABLED = 0;
    static constexpr int RC_FS_LOITER = 1;
    static constexpr int RC_FS_RTH = 2;
    static constexpr int RC_FS_LAND = 3;

    static constexpr int BATT_FS_WARNING = 0;
    static constexpr int BATT_FS_RTH = 1;
    static constexpr int BATT_FS_LAND = 2;

    template<typename... Args>
    Px4_vehicle(
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            Args &&... args) :
            Mavlink_vehicle(
                    system_id, component_id, type,
                    ugcs::vsm::mavlink::MAV_AUTOPILOT::MAV_AUTOPILOT_PX4,
                    Vehicle::Capabilities(),
                    stream, mission_dump_path, std::forward<Args>(args)...),
            vehicle_command(*this),
            task_upload(*this)
    {
        autopilot_type = "px4";
        /* Consider this as uptime start. */
        recent_connect = std::chrono::steady_clock::now();
        Configure();
    }

    virtual void
    On_enable();

    virtual void
    On_disable();

    /** Distinguishable type of PX4 vehicle. This is mainly driven by
     * PX4 firmware flavors each having some minor differences. */
    enum Type {
        /** Copter (quad, octa, hexa etc). */
        COPTER,
        /** Fixed wing plane. */
        PLANE,
        /** Rovers, cars. */
        ROVER,
        /** Other unsupported/unknown vehicle type, depending on the context.*/
        OTHER
    };

    /** UCS has sent a task for a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request) override;

    /**
     * UCS requesting command execution on a vehicle.
     */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request) override;

    bool
    Default_mavlink_handler(
        ugcs::vsm::Io_buffer::Ptr,
        ugcs::vsm::mavlink::MESSAGE_ID_TYPE message_id,
        uint8_t system_id,
        uint8_t component_id,
        uint8_t);

    /** Get the Type of the vehicle. */
    Type
    Get_type() const;

    /** Get the Type of the vehicle based on Mavlink type. */
    static Type
    Get_type(ugcs::vsm::mavlink::MAV_TYPE);

    /** PX4 specific activity. */
    class Px4_activity : public Activity {
    public:
        /** Constructor based on PX4 vehicle class. */
        Px4_activity(Px4_vehicle& px4_vehicle) :
            Activity(px4_vehicle),
            px4_vehicle(px4_vehicle) {}

        /** Managed PX4 vehicle. */
        Px4_vehicle& px4_vehicle;
    };

    void
    On_home_position(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HOME_POSITION>::Ptr message);

    void
    On_autopilot_version(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::AUTOPILOT_VERSION>::Ptr);

    void
    On_camera_information(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::CAMERA_INFORMATION>::Ptr);

    void
    On_image_captured(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::CAMERA_IMAGE_CAPTURED>::Ptr);

    void
    On_parameter(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

    void
    Download_mission();

    void
    On_mission_item(ugcs::vsm::mavlink::Pld_mission_item mi);

    void
    On_mission_downloaded(bool, std::string);

    // Calculates route id from command map and HL and puts in current_route_id.
    void
    Calculate_current_route_id();

    /** Vehicle specific telemetry initialization. */
    virtual void
    Initialize_telemetry();

    /** Data related to vehicle command processing. */
    class Vehicle_command_act : public Px4_activity {
    public:
        using Px4_activity::Px4_activity;

        /** Try execute command a vehicle. */
        bool
        Try();

        /** Command ack received. */
        void
        On_command_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr);

        void
        On_mission_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        void
        On_param_value(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        void
        On_mission_current(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_CURRENT>::Ptr);

        void
        Send_next_command();

        /** Status text recieved. */
        void
        On_status_text(
                ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr);

        /** Enable class and start command execution. */
        void
        Enable(ugcs::vsm::Vehicle_command_request::Handle vehicle_command_request);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Schedule timer for retry operation. */
        void
        Schedule_timer();

        /** Register status text handler. */
        void
        Register_status_text();

        /** Unregister status text handler. */
        void
        Unregister_status_text();

        /** Get the value of custom mode corresponding to AUTO mode of the
         * current vehicle type. */
        uint32_t
        Get_custom_auto_mode();

        /** Get the value of custom mode corresponding to reasonable manual mode
         * of the current vehicle type. */
        uint32_t
        Get_custom_manual_mode();

        /** Get the value of custom mode corresponding to guided mode
         * of the current vehicle type. */
        uint32_t
        Get_custom_guided_mode();

        /** Add MAVLink "Set mode" command */
        void
        Set_mode(uint8_t main_mode = 0, uint8_t sub_mode = 0);

        /** Add MAVLink "Do reposition" command */
        void
        Do_reposition(
            float latitude = NAN,
            float longitude = NAN,
            float altitude = NAN,
            float heading = NAN,
            float speed = 1.0f);

        /** Current command request. */
        ugcs::vsm::Vehicle_command_request::Handle vehicle_command_request;

        /** Mavlink messages to be sent to execute current command. */
        std::list<ugcs::vsm::mavlink::Payload_base::Ptr> cmd_messages;

        /** Remaining attempts towards vehicle. */
        size_t remaining_attempts = 0;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Current timeout to use when scheduling timer. */
        std::chrono::milliseconds current_timeout;
    } vehicle_command;

    /** Data related to task upload processing. */
    class Task_upload: public Px4_activity {
    public:
        using Px4_activity::Px4_activity;

        /** Calls appropriate prepare action based on type. */
        void
        Prepare_action(ugcs::vsm::Action::Ptr);

        /** Add mission item to prepared actions. Common mission item
         * initialization are made, like sequence number generation.
         */
        void
        Add_mission_item(ugcs::vsm::mavlink::Pld_mission_item::Ptr);

        //@{
        /** Prepare methods for different types of actions. These methods
         * create an item in the prepared actions list.
         * @return Created mission item. */

        void
        Prepare_move(ugcs::vsm::Action::Ptr&);

        void
        Prepare_wait(ugcs::vsm::Action::Ptr&);

        void
        Prepare_payload_steering(ugcs::vsm::Action::Ptr&);

        void
        Prepare_takeoff(ugcs::vsm::Action::Ptr&);

        void
        Prepare_landing(ugcs::vsm::Action::Ptr&);

        void
        Prepare_vtol_transition(ugcs::vsm::Action::Ptr&);

        void
        Prepare_change_speed(ugcs::vsm::Action::Ptr&);

        void
        Prepare_POI(ugcs::vsm::Action::Ptr&);

        void
        Prepare_heading(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_control(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_series_by_distance(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_series_by_time(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_trigger(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_trigger_impl(bool multiply_photos, float interval_in_seconds = 0);

        void
        Prepare_camera_recording_impl(bool start_recording);

        void Prepare_camera_mode(ugcs::vsm::mavlink::CAMERA_MODE mode);
        //@}

        /** Build waypoint mission item based on move action. */
        ugcs::vsm::mavlink::Pld_mission_item::Ptr
        Build_wp_mission_item(ugcs::vsm::Action::Ptr&);

        /** Build ROI mission item based on given coordinates */
        ugcs::vsm::mavlink::Pld_mission_item::Ptr
        Build_roi_mission_item(const ugcs::vsm::Geodetic_tuple& coords);

        /** Previous activity is completed, enable class and start task upload. */
        void
        Enable(ugcs::vsm::Vehicle_task_request::Handle);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Filter unsupported actions. */
        void
        Filter_actions();

        /** Filter actions unsupported by copters. */
        void
        Filter_copter_actions();

        /** Filter actions unsupported by planes. */
        void
        Filter_plane_actions();

        /** Filter actions unsupported by rovers. */
        void
        Filter_rover_actions();

        /** Filter actions unsupported by other types of vehicles. */
        void
        Filter_other_actions();

        /** Prepare the task for uploading to the vehicle. */
        void
        Prepare_task();

        /** Prepare task attributes depending on the vehicle type. */
        void
        Prepare_task_attributes();

        /** Prepare copter task attributes. */
        void
        Prepare_copter_task_attributes();

        /** Prepare plane task attributes. */
        void
        Prepare_plane_task_attributes();

        /** Prepare rover task attributes. */
        void
        Prepare_rover_task_attributes();

        /** Prepare task attributes of other vehicles. */
        void
        Prepare_other_task_attributes();

        /** Task attributes upload handler. */
        void
        Task_atributes_uploaded(bool success, std::string error_msg);

        /** Mission upload handler. */
        void
        Mission_uploaded(bool success, std::string error_msg);

        /**
         * Fill coordinates into Mavlink message based on ugcs::vsm::Geodetic_tuple and
         * some other common mission item data structures.
         * @param msg Mavlink message.
         * @param tuple Geodetic tuple.
         * @param heading Vehicle heading.
         */
        void
        Fill_mavlink_mission_item_coords(
            ugcs::vsm::mavlink::Pld_mission_item& msg,
            const ugcs::vsm::Geodetic_tuple& tuple,
            double heading);

        /**
         * Fill Mavlink mission item common parameters.
         * @param msg Mavlink message.
         */
        void
        Fill_mavlink_mission_item_common(ugcs::vsm::mavlink::Pld_mission_item& msg);


        /** Current task for uploading, if any. */
        ugcs::vsm::Vehicle_task_request::Handle request;

        /** Prepared Mavlink actions to be uploaded to the vehicle and built based
         * on the actions from the original request. Original actions could be
         * extended/removed/updated to meet the Mavlink mission protocol
         * requirements. Example is adding of magical "dummy waypoints" and
         * special processing of waypoint zero.
         */
        std::vector<ugcs::vsm::mavlink::Payload_base::Ptr> prepared_actions;

        /** Task attributes to be written to the vehicle. */
        Write_parameters::List task_attributes;

        /** Previous move action, if any. */
        ugcs::vsm::Action::Ptr last_move_action;

        /** takeoff action, if any. */
        ugcs::vsm::Action::Ptr takeoff_action;

        /** Active POI from mission. */
        ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> current_mission_poi;

        /** Current heading from mission. */
        ugcs::vsm::Optional<float> current_mission_heading;

        /** Does current WP have POI action defined in mission*/
        bool first_mission_poi_set = false;

        /** Mission POI action must be added as it was cancelled by previous actions.*/
        bool restart_mission_poi = false;

        float current_heading = 0.0;

        float heading_to_this_wp = 0.0;

        ugcs::vsm::Optional<int> current_camera_mode;

        /** CAMERA_SERIES_BY_DISTANCE was activated. */
        bool camera_series_by_dist_active = false,
        /** CAMERA_SERIES_BY_DISTANCE was activated in current waypoint. */
             camera_series_by_dist_active_in_wp = false,
        /** CAMERA_SERIES_BY_TIME was activated. */
            camera_series_by_time_active = false,
        /** CAMERA_SERIES_BY_DISTANCE was activated in current waypoint. */
            camera_series_by_time_active_in_wp = false;

        float max_mission_speed = 0;
    } task_upload;

private:
    /** Flight main modes of the PX4. */
    enum class Px4_main_mode {
        UNKNOWN = 0,
        /** Manual flight mode. */
        MANUAL = 1,
        /** Altitude control flight mode. */
        ALTCTL,
        /** Position control flight mode. */
        POSCTL,
        /**  Auto flight mode (see sub modes). */
        AUTO,
        /** Acro flight mode. */
        ACRO,
        /** Offboard flight mode. */
        OFFBOARD,
        /** Stabilized flight mode. */
        STABILIZED,
        /** Rattitude flight mode. */
        RATTITUDE
    };

    /** Flight auto sub modes of the PX4. */
    enum class Px4_auto_sub_mode {
        UNKNOWN = 0,
        /** Ready flight mode (unused). */
        AUTO_READY = 1,
        /** Takeoff flight mode. */
        AUTO_TAKEOFF,
        /** Hold flight mode. */
        AUTO_LOITER,
        /** Mission flight mode. */
        AUTO_MISSION,
        /** Return to Land flight mode. */
        AUTO_RTL,
        /** Land flight mode. */
        AUTO_LAND,
        /** Return to Groundstation flight mode. */
        AUTO_RTGS,
        /** Follow me flight mode */
        AUTO_FOLLOW_TARGET
    };

    /** PX4 vendor list. */
    enum class Px4_vendor {
        UNKNOWN = 0,
        YUNEEC = 1
    };

    union Px4_custom_mode {
        struct {
            uint16_t reserved;
            uint8_t main_mode;
            uint8_t sub_mode;
        };
        uint32_t data = 0;
    };

    const char*
    Get_native_flight_mode_name(Px4_custom_mode);

    /** Process heartbeat message by setting system status according to it. */
    virtual void
    Process_heartbeat(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr) override;

    void
    On_extended_sys_state(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::EXTENDED_SYS_STATE>::Ptr message);

    void
    Update_capability_states();

    /** Load parameters from configuration. */
    void
    Configure();

    bool
    Is_home_position_valid();

    void
    Start_rc_override();

    void
    Send_rc_override();

    bool
    Send_rc_override_timer();

    bool
    Is_rc_override_active();

    void
    Stop_rc_override();

    void
    Set_rc_override(int p, int r, int t, int y);

    /**
     * Minimal waypoint acceptance radius to use.
     */
    constexpr static double ACCEPTANCE_RADIUS_MIN = 1;

    /**
     * Max speed px4 copter can reach in mission. (max value for MPC_XY_VEL_MAX).
     */
    constexpr static float MAX_COPTER_SPEED = 20;

    /** Recent connect time of the vehicle. */
    std::chrono::steady_clock::time_point recent_connect;

    /** camera trigger type (0=high level commands, 1=set_servo
     *  Note: Yuneec overrides this parameters to 0 */
    int camera_trigger_type = 0;

    /** Index of servo to use for camera trigger. */
    int camera_servo_idx;
    /** PWM value to set for camera trigger. */
    int camera_servo_pwm;
    /** Time to hold camera servo at the specified PWM when triggering. */
    float camera_servo_time;

    Px4_vendor vendor = Px4_vendor::UNKNOWN;

    /** By default joystick mode is disabled for planes.
     * Turn on via an entry in conf file. */
    bool enable_joystick_control_for_fixed_wing = false;

    /** If vehicle does not support ROI for multiple WPts then VSM must
     * generate POI commands for each WP until POI(none) received.
     * Leave it true for now until VSM is able to detect PX4 FW version.
     * Pre 3.2 needs POI for each WP
     * 3.2+ will keep pointing to current poi POI until POI(0,0,0) received.*/
    bool auto_generate_mission_poi = true;

    ugcs::vsm::Geodetic_tuple home_location {0, 0, 0};

    /** Joystick mode support */

    // Timer instance for sending rc_override messages.
    ugcs::vsm::Timer_processor::Timer::Ptr rc_override_timer = nullptr;

    // RC Override message which holds the latest joystick values.
    // Existence of this messages means that vehicle is in joystick mode.
    ugcs::vsm::mavlink::Pld_rc_channels_override::Ptr rc_override = nullptr;

    // Last time we sent the rc_overrride message. Used to limit rate at which
    // the joystick commands are sent to vehicle.
    std::chrono::time_point<std::chrono::steady_clock> rc_override_last_sent;

    // Last time we received joystick message from ucs. Used to fail over to
    // manual mode automatically if no messages received for RC_OVERRIDE_TIMEOUT.
    std::chrono::time_point<std::chrono::steady_clock> direct_vehicle_control_last_received;

    // Timeout when to revert back to manual mode if no joystick messages
    // received from ucs.
    constexpr static std::chrono::milliseconds RC_OVERRIDE_TIMEOUT {3000};

    // Counter which governs the end of Joystick mode. To ensure PX4
    // exits rc_override we need to spam 0,0,0,0 messages.
    size_t rc_override_end_counter = 0;

    // How frequently to send the rc_overrride messages to vehicle.
    constexpr static std::chrono::milliseconds RC_OVERRIDE_PERIOD {200};

    // How many 0,0,0,0 rc_override messages to send to exit joystick mode.
    constexpr static size_t RC_OVERRIDE_END_COUNT = 15;

    bool is_airborne = false;

    // camera pitch and yaw starting positions
    float payload_pitch = 0;
    float payload_yaw = 0;

    // default camera component id
    int camera_component_id = 100;

    // Keep the current configured rates for each message type.
    std::map<int, float> telemetry_rates;

    Px4_custom_mode native_flight_mode;

    // Value read from MPC_XY_VEL_MAX on vehicle connect.
    float max_ground_speed = 0;

    // true when VSM has understood which mavlink version the vehicle supports.
    bool protocol_version_detected = false;

    // Current mission hash.
    uint32_t current_route_id;
};

#endif /* _PX4_VEHICLE_H_ */
