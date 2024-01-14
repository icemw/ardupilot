#include "Copter.h"

#if MODE_TRACK_ENABLED == ENABLED

/*
 * 五角星航线模式初始化
 */
bool ModeTrack::init(bool ignore_checks)
{
    finish = false;     //初始化是否抵达目标
    generate_dest( track_dest );  // 路径生成

    pos_control_start();  // 开始位置控制

    return true;
}

// 开始位置控制
void ModeTrack::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
//    wp_nav->set_wp_destination(current_dest, false);
    wp_nav->set_wp_destination(track_dest, false);    

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

void ModeTrack::generate_dest(Vector3f target_loc){
    //track_dest = ;
}
// 此模式的周期调用
void ModeTrack::run()
{
    if (wp_nav->      current_dest == track_dest) {  // 如果已经到达最终位置
        if (wp_nav->reached_wp_destination()) {  // 到达某个端点
            finish = true;  //已到达目标位置
            gcs().send_text(MAV_SEVERITY_INFO, "Track finished, now go into loiter mode");
            copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
            //wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    } else {  // 未到达最终位置
        if (wp_nav->reached_wp_destination()) {  // 到达当前目标点
            current_dest = next_dest;
            //wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    }

    pos_control_run();  // 位置控制器
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeTrack::pos_control_run()  // 注意，此函数直接从mode_guided.cpp中复制过来，不需要该其中的内容
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
    // if (auto_yaw.mode() == AutoYaw::Mode::HOLD) {
    //     // roll & pitch from waypoint controller, yaw rate from pilot
    //     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    // } else if (auto_yaw.mode() == AutoYaw::Mode::RATE) {
    //     // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
    //     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    // } else {
    //     // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
    //     attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    // }
}

#endif
