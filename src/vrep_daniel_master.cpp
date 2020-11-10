/**
 * @file vrep_daniel_master.cpp
 * @brief DANIELをvrep上で動かすためのクラス
 * @author Yusuke Yoshizaki
 * @date 2019/09/19
 * @detail
 *  シミュレータV-REP上のロボットを操作する．
 *  V-REPのremoteAPIを使用してシステムを構築する．
 */

#include "vrep_daniel_master.h"

// ANIの寸法、設定値
#define CRAWLER_HEIGHT 0.195 // クローラの高さ[m]
#define TREAD 0.31 // トレッド長[m]
#define SPEED 0.276 // 兄の速度[m/s]
#define OMEGA_MAX SPEED/(CRAWLER_HEIGHT/2) // 最大角速度
#define AXIS_MAX_VIEW_ANGLE 47.0// カメラの最大視野角[deg]

// V-REPの使うジョイント数やジョイントの番号
#define NUM_LEFT_WHEEL 8
#define NUM_LEFT_MOTOR 8
#define NUM_RIGHT_WHEEL 8
#define NUM_RIGHT_MOTOR 8
#define NUM_SM_MOTER 1
#define NUM_AXIS_PAN 1
#define NUM_AXIS_TILT 1
#define NUM_AXIS_CAMERA 1

// ジョイントや車輪の名前（基本形）
#define OBJECT_NAME "ani_base"
#define LEFT_WHEEL_OBJ_NAME "left_wheel_"
#define LEFT_MOTOR_OBJ_NAME "left_motor_"
#define RIGHT_WHEEL_OBJ_NAME "right_wheel_"
#define RIGHT_MOTOR_OBJ_NAME "right_motor_"
#define SM_MOTOR_OBJ_NAME "sm_base_to_sm_motor"
#define AXIS_PAN_OBJ_NAME "axis_base_to_axis_yaw"
#define AXIS_TILT_OBJ_NAME "axis_yaw_to_axis_pitch"
#define AXIS_CAMERA_OBJ_NAME "axis_camera"

//===== constructor & destrucor ==================================================//
/** @fn
 * @brief コンストラクタ
 * @param なし
 */
VrepDanielMaster::VrepDanielMaster(){

  // ANIの動き
  speed_left_ = 0.0*2*M_PI; //左モータの速度
  speed_right_= 0.0*2*M_PI; //右モータの速度
  speed_level_ = 1.0;       //直進レベル
  turn_level_ = 0.3;        //旋回レベル

  // Dynamixel関係
  speed_ = -1.0f; //回転速度
  cycle_ = -1.0f; //回転周期

  // Axis Camera関係
  pan_ = 0.0; // -170~170
  tilt_ = 0.0; // -10~180
  zoom_ = 1; // 1~9999

  // DANIELの制御周期[s]
  sampling_time_ = 1.0/50;    // 50Hz

  // Dynamixelの制御周期[s]
  dx_sampling_time_ = 1.0/50;  // 50Hz

  // シミュレーションの開始
  StartSimulation();

  // ハンドルの取得 + エラー報告
  if ( not GetHandle()){
    ROS_ERROR("cannot get handle!!");
  }

  // GUI
  f = boost::bind(&VrepDanielMaster::ParamCallback, this, _1, _2);
  server.setCallback(f);

  // Timer
  timer_ = nh_.createTimer(ros::Duration(sampling_time_), &VrepDanielMaster::TimerCallback,this);
  dx_timer_ = nh_.createTimer(ros::Duration(dx_sampling_time_), &VrepDanielMaster::DxTimerCallback,this);

  // Subscriber
  sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &VrepDanielMaster::CallBackOfCmdVel,this);
  sub_axis_ptz_ = nh_.subscribe("cmd", 1, &VrepDanielMaster::CallBackOfAxis, this);

  // Publisher
  pub_dx_ = nh_.advertise<std_msgs::Float32>("/dx_angle", 1);
}

/** @fn
 * @brief デストラクタ
 * @param なし
 */
VrepDanielMaster::~VrepDanielMaster()
{
}

//===== V-REP関係 ==================================================//
/** @fn
 * @brief シミュレータ上のオブジェクトのハンドル取得
 * @param なし
 * @return bool true:成功、false:失敗
 * @detail
 *  サンプリングタイム毎に命令を実行
 */
bool VrepDanielMaster::GetHandle() {
  //動的配列の要素数をリサイズ
  left_wheel_.resize(NUM_LEFT_WHEEL);
  left_motor_.resize(NUM_LEFT_MOTOR);
  right_wheel_.resize(NUM_RIGHT_WHEEL);
  right_motor_.resize(NUM_RIGHT_MOTOR);
  sm_motor_.resize(NUM_SM_MOTER);
  axis_pan_.resize(NUM_AXIS_PAN);
  axis_tilt_.resize(NUM_AXIS_TILT);
  axis_camera_.resize(NUM_AXIS_CAMERA);

  //ハンドル時にエラーがないかのチェック変数
  bool is_some_error_in_getting_handles = false;

  //ハンドルしたいV-REP上のオブジェクトの名前を設定
  //名前なのでString型
  std::string obj_name = OBJECT_NAME;
  if ( not body_.GetHandle( obj_name ) ) {
    //指定した名前のオブジェクトのハンドルに失敗した場合
    //ObjectクラスのGetHandleはbool型の返り値があり、失敗するとfalseが返ってくる
    is_some_error_in_getting_handles = true;
  }

  //  left_wheelのhandleを取得，保存
  for (uint8_t i_left_wheel=0; i_left_wheel<NUM_LEFT_WHEEL; i_left_wheel++) {
    std::string obj_name = LEFT_WHEEL_OBJ_NAME + std::to_string(i_left_wheel);
    if ( not left_wheel_[i_left_wheel].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

  //  right_wheelのhandleを取得，保存
  for (uint8_t i_right_wheel=0; i_right_wheel<NUM_RIGHT_WHEEL; i_right_wheel++) {
    std::string obj_name = RIGHT_WHEEL_OBJ_NAME + std::to_string(i_right_wheel);
    if ( not right_wheel_[i_right_wheel].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

  // left_motorのhandleを取得，保存
  for (uint8_t i_left_motor=0; i_left_motor<NUM_LEFT_MOTOR; i_left_motor++) {
    std::string obj_name = LEFT_MOTOR_OBJ_NAME + std::to_string(i_left_motor);
    if( not left_motor_[i_left_motor].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

  // right_motorのhandleを取得
  for (uint8_t i_right_motor=0; i_right_motor<NUM_RIGHT_MOTOR; i_right_motor++) {
    std::string obj_name = RIGHT_MOTOR_OBJ_NAME + std::to_string(i_right_motor);
    if( not right_motor_[i_right_motor].GetHandle( obj_name ) ) {
      is_some_error_in_getting_handles = true;
    }
  }

  // sm_motorのhandleを取得，保存
  obj_name = SM_MOTOR_OBJ_NAME;
  if( not sm_motor_[0].GetHandle( obj_name ) ) {
    is_some_error_in_getting_handles = true;
  }

  // axis_panのhanadleを取得、保存
  obj_name = AXIS_PAN_OBJ_NAME;
  if ( not axis_pan_[0].GetHandle( obj_name ) ){
    is_some_error_in_getting_handles = true;
  }

  // axis_tiltのhandleを取得、保存
  obj_name = AXIS_TILT_OBJ_NAME;
  if ( not axis_tilt_[0].GetHandle( obj_name ) ){
    is_some_error_in_getting_handles = true;
  }

  // axis_cameraのhandleを取得、保存
  obj_name = AXIS_CAMERA_OBJ_NAME;
  if ( not axis_camera_[0].GetHandle( obj_name ) ){
    is_some_error_in_getting_handles = true;
  }

  // エラーの有無を確認
  if (is_some_error_in_getting_handles){
    //どこか1箇所でもオブジェクトのハンドルに失敗した場合
    return false;
  }
  //オブジェクトのハンドルに成功した場合
  return true;
}

//===== GUI ==================================================//
/** @fn
 * @brief Dynamixelの回転速度or周期をGUIで表示させる
 * @param
 * @return なし
 * @detail
 *
 */
 void VrepDanielMaster::ParamCallback(vrep_daniel_master::DXParamsConfig &config, uint32_t level) {

   double pre_speed = speed_;
   double pre_cycle = cycle_;

   if (config.speed != pre_speed) { // 更新時のみ

     sm_motor_[0].SetTargetVelocity(config.speed, false);
     speed_ = config.speed;

   }else if ((config.cycle != pre_cycle) && (config.cycle != 0.0f)) { // 更新時のみ

     if (config.cycle == 120.0f || config.cycle == 0.0f)  // 最大値
       sm_motor_[0].SetTargetVelocity(0.0f, false);

     else sm_motor_[0].SetTargetVelocity(2.0f*M_PI/config.cycle, false); // 1/2を指令

     cycle_ = config.cycle;
   }
 }

//===== Callback ==================================================//
/** @fn
 * @brief ANI制御用タイマーコールバック
 * @param const ros::TimerEvent& event
 * @return なし
 * @detail
 *  サンプリングタイム毎に
 *  ANIのジョイント（モータ）に速度+Axisのジョイントに角度を送信
 *
 */
void VrepDanielMaster::TimerCallback(const ros::TimerEvent& event) {

  for (uint8_t i_left_motor=0; i_left_motor<NUM_LEFT_MOTOR; i_left_motor++) {
    left_motor_[i_left_motor].SetTargetVelocity(speed_left_, false);  // false:[rad/sec], true:[deg/sec]
  }

  for (uint8_t i_right_motor=0; i_right_motor<NUM_RIGHT_MOTOR; i_right_motor++) {
    right_motor_[i_right_motor].SetTargetVelocity(speed_right_, false);  // false:[rad/sec], true:[deg/sec]
  }

  axis_pan_[0].SetTargetAngle(pan_, true); // false:[rad/sec], true:[deg/sec]
  axis_tilt_[0].SetTargetAngle(tilt_, true); // false:[rad/sec], true:[deg/sec]

  axis_camera_[0].SetPerspectiveAngle(VrepDanielMaster::ZoomToViewAngle(zoom_) ,true); // false:[rad], true:[deg]

}

/** @fn
 * @brief 雲台Dynamixel角度取得用タイマーコールバック
 * @param const ros::TimerEvent& event
 * @return なし
 * @detail
 *  サンプリングタイム毎にDynamixelの角度を取得 + Publish
 */
void VrepDanielMaster::DxTimerCallback(const ros::TimerEvent& event){
  dx_angle_.data = sm_motor_[NUM_SM_MOTER - 1].GetAngle(false); // true:[deg/s], false:[rad/s]
  pub_dx_.publish(dx_angle_);
}

/** @fn
 * @brief Axisコールバック関数
 * @param axis_camera::Axis axis
 * @return なし
 * @detail
 *  AxisのPTZコマンドの値を受け取る
 */
void VrepDanielMaster::CallBackOfAxis(axis_camera::Axis axis){
  //PTZ値を代入
  pan_ = axis.pan;
  tilt_ = axis.tilt;
  zoom_ = axis.zoom;
}

 /** @fn
  * @brief cmd_velコールバック関数
  * @param geometry_msgs::Twist cmd_vel
  * @return なし
  * @detail
  *  cmd_velを受け取ってモータの角速度に変換
  */
void VrepDanielMaster::CallBackOfCmdVel(geometry_msgs::Twist vel){

  //速度を一度初期化
  speed_left_ = 0;
  speed_right_ = 0;

  // モータの角度速度に変換
  speed_left_ += (vel.linear.x - vel.angular.z*TREAD/2) / CRAWLER_HEIGHT/2;
  speed_right_ += (vel.linear.x + vel.angular.z*TREAD/2) / CRAWLER_HEIGHT/2;

}


double VrepDanielMaster::ZoomToViewAngle(int zoom){
  double view_angle = AXIS_MAX_VIEW_ANGLE;
  double max_view_angle = AXIS_MAX_VIEW_ANGLE;
  double zoom_mag = 1;

  if (zoom >= 1 && zoom <= 1247){
    zoom_mag = 0.9998*exp(0.0002*zoom);
  }else if(zoom > 1247 && zoom <= 2498){
    zoom_mag = 1.0824*exp(0.0002*zoom);
  }else if(zoom > 2499 && zoom <= 3749){
    zoom_mag = 0.8732*exp(0.0003*zoom);
  }else if(zoom > 3750 && zoom <= 5000){
    zoom_mag = 0.9993*exp(0.0002*zoom);
  }else if(zoom > 5001 && zoom <= 6246){
    zoom_mag = 0.6731*exp(0.0003*zoom);
  }else if(zoom > 6247 && zoom <= 8748){
    zoom_mag = 0.4432*exp(0.0004*zoom);
  }else if(zoom > 8749 && zoom <= 9999){
    zoom_mag = 0.0269*exp(0.0007*zoom);
  }else{
    ROS_WARN("zoom value is unsuitable!");
  }

  view_angle = max_view_angle / zoom_mag;

  return view_angle;
}
