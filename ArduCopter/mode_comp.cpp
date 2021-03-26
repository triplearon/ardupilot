#include "Copter.h"

bool ModeComp::init(bool ignore_checks){
    pos_control_start();

    init_wp();

    fly_mode = loiter;

    //do_takeoff_start(500.0f);
    return true;
}

void ModeComp::run(){
    switch (fly_mode){
        case takeoff:
            takeoff_run();
            break;

        case loiter:
            auto_loiter();
            break;

        case vision:
            break;
    }
}

void ModeComp::pos_control_start()
{

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

void ModeComp::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && ModeComp::use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeComp::use_pilot_yaw(void) const
{
    return (copter.g2.guided_options.get() & uint32_t(ModeGuided::Options::IgnorePilotYaw)) == 0;
}

bool ModeComp::do_takeoff_start(float takeoff_alt_cm){
    fly_mode = takeoff;

    // initialise wpnav destination
    Location target_loc = copter.current_loc;
    Location::AltFrame frame = Location::AltFrame::ABOVE_HOME;

    if (wp_nav->rangefinder_used_and_healthy() &&
            wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
            takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        frame = Location::AltFrame::ABOVE_TERRAIN;
    }
    target_loc.set_alt_cm(takeoff_alt_cm, frame);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed");
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

    return true;
}

void ModeComp::takeoff_run()
{
    auto_takeoff_run();
    if (wp_nav->reached_wp_destination()) {
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();

        // switch to position control mode but maintain current target
        const Vector3f target = wp_nav->get_wp_destination();
        set_destination(target, false, 0, false, 0, false, wp_nav->origin_and_destination_are_terrain_alt());

        fly_mode = loiter;
    }
}

bool ModeComp::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool terrain_alt)
{
#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // ensure we are in position control mode
    if (fly_mode != loiter) {
        pos_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, terrain_alt);

    // log target
    copter.Log_Write_GuidedTarget(fly_mode, destination, Vector3f());
    return true;
}

void ModeComp::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

void ModeComp::init_wp(){
    wp_nav->get_wp_stopping_point(wp_list[0]);
    wp_count = 0;

    float distance_a = g.distance_a;
    float distance_b = g.distance_b;
    float disantce_c = g.distance_c;
    float avoidance_degreed = g.avoidance_angle;

    wp_list[1] = wp_list[0] + Vector3f(1.0f, 0, 0) * distance_a;
    wp_list[2] = wp_list[1] + Vector3f(0, 1.0f, 0) * distance_b;
    wp_list[3] = wp_list[2] + Vector3f(-1.0f, (-1.0f / (cosf(radians(avoidance_degreed)))), 0) * (disantce_c / 2.0f);
    wp_list[4] = wp_list[3] + Vector3f(-1.0f, (1.0f / (cosf(radians(avoidance_degreed)))), 0) * (disantce_c / 2.0f);

    wp_list[5] = wp_list[0];
}

// Loiter process, running at 400Hz
void ModeComp::auto_loiter(){
    static int timer;

    if(wp_nav->reached_wp_destination()){
        timer++;

        if(timer < 5 * 400 && wp_count != 3){
            goto outside;
        }

        timer = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Got Point:%d", wp_count);
        if(wp_count == 5){
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Mission accomplished!");
            copter.set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
        } else {
            wp_nav->set_wp_destination(wp_list[++wp_count]);
        }
    }
    outside:
    pos_control_run();
}
