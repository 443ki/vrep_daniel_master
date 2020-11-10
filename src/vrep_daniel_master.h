/**
 * @file vrep_daniel_master.h
 * @brief DANIELをvrep上で動かすためのクラス
 * @author Yusuke Yoshizaki
 * @date 2019/09/19
 * @detail
 *  シミュレータV-REP上のロボットを操作する．
 *  V-REPのremoteAPIを使用し，DANIELのプログラムを
 *  使用できるようにシステムを構築する．
 */

#ifndef VREP_DANIEL_MASTER_SRC_VREP_DANIEL_MASTER_H_
#define VREP_DANIEL_MASTER_SRC_VREP_DANIEL_MASTER_H_

#include <ros/ros.h>
#include <math.h>
#include "vrep.h"

// Twist
#include <geometry_msgs/Twist.h>

// Dynamixel
#include <std_msgs/Float32.h>

// Axis
#include <axis_camera/Axis.h>

// GUI
#include <dynamic_reconfigure/server.h>
#include <vrep_daniel_master/DXParamsConfig.h>

class VrepDanielMaster : public Vrep {
  public:
    VrepDanielMaster();
    ~VrepDanielMaster();

  private:
    // V-REP関係
    bool GetHandle();

    // GUI
    void ParamCallback(vrep_daniel_master::DXParamsConfig &config, uint32_t level);

    // CallBack関数
    void TimerCallback(const ros::TimerEvent& event);
    void DxTimerCallback(const ros::TimerEvent& event);
    void CallBackOfCmdVel(geometry_msgs::Twist cmd_vel);
    void CallBackOfAxis(axis_camera::Axis axis);

    // Axisカメラ視野角計算
    double ZoomToViewAngle(int zoom);

    // V-REP関係
    Object body_;
    std::vector<Object> left_wheel_;
    std::vector<Object> right_wheel_;

    std::vector<JointObject> left_motor_;
    std::vector<JointObject> right_motor_;

    std::vector<JointObject> sm_motor_;

    std::vector<JointObject> axis_pan_;
    std::vector<JointObject> axis_tilt_;

    std::vector<VisionSensor> axis_camera_;

    // GUI
    dynamic_reconfigure::Server<vrep_daniel_master::DXParamsConfig> server;
    dynamic_reconfigure::Server<vrep_daniel_master::DXParamsConfig>::CallbackType f;

    // ANIの動き
    double speed_left_;   // 左モータの速度
    double speed_right_;  // 右モータの速度
    double speed_level_;  // 直進レベル
    double turn_level_;   // 旋回レベル

    // Dynamixel
    std_msgs::Float32 dx_angle_; // Dynamixelの角度

    double speed_; // 回転速度
    double cycle_ ;// 回転周期

    // Axis Camera
    double pan_;  // panの指令値 "-170~170"
    double tilt_; // tiltの指令値 "-10~90"
    double zoom_; // zoomの指令値 "1~9999"

    // Timer
    ros::Timer timer_;
    ros::Timer dx_timer_;

    // NodeHandle
    ros::NodeHandle nh_;

    // ANIの制御周期[s]
    double sampling_time_;

    // Dynamixelの制御周期[s]
    double dx_sampling_time_;

    // Subscriber
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_axis_ptz_;

    // Publisher
    ros::Publisher pub_dx_;
};

#endif /* VREP_DANIEL_MASTER_SRC_VREP_DANIEL_MASTER_H_ */
