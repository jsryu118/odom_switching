#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <deque>
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "microstrain_inertial_msgs/HumanReadableStatus.h"

#include "kf.h"


namespace ryu {

class SwitchingNode {
   public:
    SwitchingNode(ros::NodeHandle &nh) {
        std::string topic_gq7_status, topic_lio_status, topic_gnss_status, topic_gq7, topic_lio, topic_eskf, topic_pubodom, topic_pubpose, topic_pubvel, log_path; 
        nh.param<std::string>("topic_gq7_status", topic_gq7_status, "/ekf/status");
        nh.param<std::string>("topic_lio_status", topic_lio_status, "/ekf/status");
        nh.param<std::string>("topic_gnss_status", topic_gnss_status, "/gnss/status");
        nh.param<std::string>("topic_gq7", topic_gq7, "/ekf/odometry_map");
        nh.param<std::string>("topic_eskf", topic_eskf, "/eskf/odom");
        nh.param<std::string>("topic_pubodom", topic_pubodom, "/switching_result");
        nh.param<std::string>("topic_pubpose", topic_pubpose, "/current_pose");
        nh.param<std::string>("topic_pubvel", topic_pubvel, "/current_velocity");
        nh.param<std::string>("log_path", log_path, "odom_log.txt");
        bool use_lidar_odom;
        nh.param("use_lidar_odom", use_lidar_odom, false);

        // ROS sub & pub
        gq7_status_sub_ = nh.subscribe(topic_gq7_status, 10, &SwitchingNode::gq7_status_callback, this);
        lio_status_sub_ = nh.subscribe(topic_lio_status, 10, &SwitchingNode::lio_status_callback, this);
        gnss_status_sub_ = nh.subscribe(topic_gnss_status, 10, &SwitchingNode::gnss_status_callback, this);
        gq7_sub_ = nh.subscribe(topic_gq7, 10, &SwitchingNode::gq7_callback, this);
        lio_sub_ = nh.subscribe(topic_lio, 10, &SwitchingNode::lio_callback, this);
        eskf_sub_ = nh.subscribe(topic_eskf, 10, &SwitchingNode::eskf_callback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_pubodom, 10);

        nh.param("num_stable_threshold", num_stable_threshold, 5);
        stable_count=0;


        ////////////////// KF ///////////////////////  
        double pos_n, vel_n;
        nh.param("pos_n", pos_n, 0.001);
        nh.param("vel_n", vel_n, 0.001);
        kf_ptr_ = std::make_unique<KF>(pos_n, vel_n);

        if (use_lidar_odom){
            topic_pubpose += "_gnss"; 
            topic_pubvel += "_gnss";
        }
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic_pubpose, 10);
        vel_pub_ = nh.advertise<geometry_msgs::TwistStamped >(topic_pubvel, 10);

        if (std::ifstream(log_path)) {
            // If the file exists, delete it
            if (std::remove(log_path.c_str()) != 0) {
                ROS_ERROR("Failed to delete existing log file!");
            }
        }
        log_file_.open(log_path, std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            ROS_ERROR("Failed to open log file!");
        }
        ////////////////// TF ///////////////////////  
        nh.param("pub_tf", pub_tf, false);
        nh.param<std::string>("frame_id", frame_id, "map");
        nh.param<std::string>("child_frame_id", child_frame_id, "base_link");

        double publish_rate;
        nh.param("publish_rate", publish_rate, 100.0);  // 기본값 10Hz
        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate), &SwitchingNode::timerCallback, this);
    }

    ~SwitchingNode() {
    }

    void gq7_status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &gq7_status_msg);
    void lio_status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &lio_status_msg);
    void gnss_status_callback(const std_msgs::Bool &gnss_status_msg);
    void state_publisher(const nav_msgs::Odometry &odom_msg);
    void kf_state_publisher(const nav_msgs::Odometry &odom_msg);
    void gq7_callback(const nav_msgs::OdometryConstPtr &gq7_msg);
    void lio_callback(const nav_msgs::OdometryConstPtr &lio_msg);
    void eskf_callback(const nav_msgs::OdometryConstPtr &eskf_msg);
    void logger(bool stable_flag);
    void logger_single(int state_flag, const nav_msgs::OdometryConstPtr &odom_msg);
    void timerCallback(const ros::TimerEvent& event);
    int state_checker();

    void prepare_kf(const nav_msgs::Odometry& odom_msg);

    nav_msgs::Odometry apply_kf(const nav_msgs::Odometry& odom_msg);

   private:
    ros::Subscriber gq7_status_sub_;
    ros::Subscriber gq7_sub_;
    ros::Subscriber lio_status_sub_;
    ros::Subscriber gnss_status_sub_;
    
    ros::Subscriber lio_sub_;
    ros::Subscriber eskf_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher vel_pub_;
    ros::Timer timer_; 

    tf::TransformBroadcaster tf_broadcaster_;

    bool is_initialized_gq7_ = false;
    bool is_initialized_lio_ = false;
    bool is_initialized_eskf_ = false;
    bool is_gq7_stable_ = false;
    bool is_eskf_stable_ = true;
    bool is_lio_stable_ = false;
    bool is_gnss_stable_ = true;
    bool is_stable_switching_ = true;


    nav_msgs::Odometry gq7_odom;
    nav_msgs::Odometry eskf_odom;
    nav_msgs::Odometry lio_odom;
    std::ofstream log_file_;
    int num_switching = 0;
    int num_switching_threshold = 300; // 100hz x 3sec
    
    double switching_stable_threshold = 0.01; // m^2
    KFPtr kf_ptr_;
    double last_t;
    int stable_count;
    int num_stable_threshold;
    bool pub_tf;
    std::string frame_id, child_frame_id;

    bool is_stable_before_=false;

    const int CURRENTSTATELIO = 1;
    const int CURRENTSTATEGQ7 = 2;
    const int CURRENTSTATEESKF = 3;
    const int LIO2GQ7 = 4;
    const int LIO2ESKF = 5;
    const int GQ72ESKF = 6;
    const int GQ72LIO = 7;
    const int ESKF2GQ7 = 8;
    const int ESKF2LIO = 9;
    int current_state = -1;
    int current_changing_state = 0;
};

void SwitchingNode::prepare_kf(const nav_msgs::Odometry& odom_msg) {
    auto& state = kf_ptr_->state_ptr_->x;
    last_t = odom_msg.header.stamp.toSec();
    state(0) = odom_msg.pose.pose.position.x;
    state(1) = odom_msg.pose.pose.position.y;
    state(2) = odom_msg.pose.pose.position.z;
    state(3) = odom_msg.twist.twist.linear.x;
    state(4) = odom_msg.twist.twist.linear.y;
    state(5) = odom_msg.twist.twist.linear.z;
}

nav_msgs::Odometry SwitchingNode::apply_kf(const nav_msgs::Odometry& odom_msg) {
    double dt = odom_msg.header.stamp.toSec() - last_t;
    if (dt < DBL_EPSILON) dt = 0.0001;
    Eigen::Matrix<double, kStateDim, kStateDim> F = Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt; // Position to velocity

    // 잡음 공분산 행렬 Q
    Eigen::Matrix<double, kStateDim, kStateDim> Q = Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.01 * dt * dt); // Position noise
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (0.01 * dt); // Velocity noise

    kf_ptr_->predict(F, Q);

    Eigen::Matrix<double, 6, kStateDim> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Position
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // Velocity

    Eigen::Matrix<double, 6, 6> R;
    R.setIdentity();
    R.block<3, 3>(0, 0) *= 0.01;  // Position noise
    R.block<3, 3>(3, 3) *= 0.01;  // Velocity noise

    Eigen::Matrix<double, 6, 1> z;
    z.block<3, 1>(0, 0) = Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    z.block<3, 1>(3, 0) = Eigen::Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);

    kf_ptr_->update(H, R, z);
    const auto& state = kf_ptr_->state_ptr_->x;
    nav_msgs::Odometry kf_odom_msg= odom_msg;
    kf_odom_msg.pose.pose.position.x = state(0);
    kf_odom_msg.pose.pose.position.y = state(1);
    kf_odom_msg.pose.pose.position.z = state(2);
    kf_odom_msg.twist.twist.linear.x = state(3);
    kf_odom_msg.twist.twist.linear.y = state(4);
    kf_odom_msg.twist.twist.linear.z = state(5);
    last_t = kf_odom_msg.header.stamp.toSec();

    // Calculate the difference between state and odom_msg
    double dx = state(0) - odom_msg.pose.pose.position.x;
    double dy = state(1) - odom_msg.pose.pose.position.y;
    double dz = state(2) - odom_msg.pose.pose.position.z;

    // Calculate the parameter as the squared sum of differences
    double param = dx*dx + dy*dy + dz*dz;
    if (param < switching_stable_threshold){
        std::cout<< "converged" <<std::endl;
        is_stable_switching_ = true;
        current_changing_state = 0;
    }
    return kf_odom_msg;
}

void SwitchingNode::gq7_status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &gq7_status_msg) {
    int num_status = gq7_status_msg->status_flags.size();
    std::string first_status = gq7_status_msg->status_flags[0];
    std::string stable_status = "\"Stable\""; // 따옴표를 그대로 사용
    is_stable_before_ = is_gq7_stable_;
    if (num_status == 1 && first_status == stable_status) {
        if (!is_gq7_stable_){
            stable_count++;    
        }
        if (stable_count >= num_stable_threshold) {
            if (!is_gq7_stable_){
                is_gq7_stable_ = true;
                is_stable_switching_ = false;
            }
        }
        // logger(is_gq7_stable_);

    } else {
        stable_count = 0;
        if (is_gq7_stable_){
            is_gq7_stable_ = false;
            is_stable_switching_ = false;
        }
        // logger(is_gq7_stable_);
        // std::cout << "unstable" << std::endl;
    }
    current_changing_state = state_checker();
}

int SwitchingNode::state_checker() {
    int previous_state = current_state;
    if(is_lio_stable_) current_state = CURRENTSTATELIO;
    else{
        if(is_gnss_stable_){
            if(is_gq7_stable_) current_state = CURRENTSTATEGQ7;
            else if(is_eskf_stable_) current_state = CURRENTSTATEESKF;
        }
        else current_state = CURRENTSTATELIO;
    }

    if(previous_state==CURRENTSTATELIO && current_state==CURRENTSTATEESKF) return LIO2ESKF;
    else if(previous_state==CURRENTSTATELIO && current_state==CURRENTSTATEGQ7) return LIO2GQ7;
    else if(previous_state==CURRENTSTATEESKF && current_state==CURRENTSTATELIO) return ESKF2LIO;
    else if(previous_state==CURRENTSTATEESKF && current_state==CURRENTSTATEGQ7) return ESKF2GQ7;
    else if(previous_state==CURRENTSTATEGQ7 && current_state==CURRENTSTATELIO) return GQ72LIO;
    else if(previous_state==CURRENTSTATEGQ7 && current_state==CURRENTSTATEESKF) return GQ72ESKF;
    else return 0;
}

void SwitchingNode::lio_status_callback(const microstrain_inertial_msgs::HumanReadableStatusConstPtr &lio_status_msg) {

    is_lio_stable_ = true;
    // is_lio_stable_ = lio_status_msg->;

    current_changing_state = state_checker();
}

void SwitchingNode::gnss_status_callback(const std_msgs::Bool &gnss_status_msg) {
    if(!gnss_status_msg.data) is_gnss_stable_ = false;
    else is_gnss_stable_ = true;
    
    current_changing_state = state_checker();
}

void SwitchingNode::gq7_callback(const nav_msgs::OdometryConstPtr &gq7_msg) {
    if (!is_initialized_gq7_) is_initialized_gq7_ = true;
    gq7_odom = *gq7_msg;

    Eigen::Quaterniond orientation(
        gq7_msg->pose.pose.orientation.w,
        gq7_msg->pose.pose.orientation.x,
        gq7_msg->pose.pose.orientation.y,
        gq7_msg->pose.pose.orientation.z
    );

    Eigen::Vector3d linear_vel(
        gq7_msg->twist.twist.linear.x,
        gq7_msg->twist.twist.linear.y,
        gq7_msg->twist.twist.linear.z
    );

    Eigen::Vector3d transformed_vel = orientation * linear_vel;
    gq7_odom.twist.twist.linear.x = transformed_vel.x();
    gq7_odom.twist.twist.linear.y = transformed_vel.y();
    gq7_odom.twist.twist.linear.z = transformed_vel.z();
}

void SwitchingNode::lio_callback(const nav_msgs::OdometryConstPtr &lio_msg) {
    if (!is_initialized_lio_) is_initialized_lio_ = true;
    lio_odom = *lio_msg; 
}

void SwitchingNode::eskf_callback(const nav_msgs::OdometryConstPtr &eskf_msg) {
    if (!is_initialized_eskf_) is_initialized_eskf_ = true;
    eskf_odom = *eskf_msg;
}

void SwitchingNode::state_publisher(const nav_msgs::Odometry &sub_odom_msg) {
    nav_msgs::Odometry pub_odom_msg = sub_odom_msg;
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped vel_msg;
    odom_pub_.publish(pub_odom_msg);
    pose_msg.header = pub_odom_msg.header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = pub_odom_msg.pose.pose;
    vel_msg.header = pub_odom_msg.header;
    vel_msg.twist = pub_odom_msg.twist.twist;
    vel_msg.header.frame_id = "map";

    odom_pub_.publish(pub_odom_msg);
    pose_pub_.publish(pose_msg);
    vel_pub_.publish(vel_msg);

    // broadcast tf
    if (pub_tf){
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(
            pub_odom_msg.pose.pose.position.x, 
            pub_odom_msg.pose.pose.position.y, 
            pub_odom_msg.pose.pose.position.z));

        tf::Quaternion tf_quat(
            pub_odom_msg.pose.pose.orientation.x,
            pub_odom_msg.pose.pose.orientation.y,
            pub_odom_msg.pose.pose.orientation.z,
            pub_odom_msg.pose.pose.orientation.w);
        
        tf_transform.setRotation(tf_quat);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, pub_odom_msg.header.stamp, frame_id, child_frame_id));
    }
    prepare_kf(sub_odom_msg);
}

void SwitchingNode::kf_state_publisher(const nav_msgs::Odometry &sub_odom_msg) {
    nav_msgs::Odometry pub_odom_msg = sub_odom_msg;
    pub_odom_msg = apply_kf(sub_odom_msg);


    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped vel_msg;
    odom_pub_.publish(pub_odom_msg);
    pose_msg.header = pub_odom_msg.header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = pub_odom_msg.pose.pose;
    vel_msg.header = pub_odom_msg.header;
    vel_msg.twist = pub_odom_msg.twist.twist;
    vel_msg.header.frame_id = "map";

    odom_pub_.publish(pub_odom_msg);
    pose_pub_.publish(pose_msg);
    vel_pub_.publish(vel_msg);

    // broadcast tf
    if (pub_tf){
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(
            pub_odom_msg.pose.pose.position.x, 
            pub_odom_msg.pose.pose.position.y, 
            pub_odom_msg.pose.pose.position.z));

        tf::Quaternion tf_quat(
            pub_odom_msg.pose.pose.orientation.x,
            pub_odom_msg.pose.pose.orientation.y,
            pub_odom_msg.pose.pose.orientation.z,
            pub_odom_msg.pose.pose.orientation.w);
        
        tf_transform.setRotation(tf_quat);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_transform, pub_odom_msg.header.stamp, frame_id, child_frame_id));
    }
}

void SwitchingNode::timerCallback(const ros::TimerEvent& event) {
    if(!is_initialized_eskf_ && !is_initialized_lio_ && !is_initialized_gq7_) return;
    if(current_state<0) return;
    nav_msgs::Odometry pub_odom;
    if(current_changing_state == 0){
        if(current_state == CURRENTSTATEESKF) pub_odom = eskf_odom;
        else if(current_state == CURRENTSTATEGQ7) pub_odom = gq7_odom;
        else if(current_state == CURRENTSTATELIO) pub_odom = lio_odom;
        else return;
        state_publisher(pub_odom);
    }
    else{
        if(current_state == CURRENTSTATEESKF) pub_odom = eskf_odom;
        else if(current_state == CURRENTSTATEGQ7) pub_odom = gq7_odom;
        else if(current_state == CURRENTSTATELIO) pub_odom = lio_odom;
        else return;
        kf_state_publisher(pub_odom);
    }
}
}  // namespace ryu

int main(int argc, char **argv) {
    ros::init(argc, argv, "switching");

    ros::NodeHandle nh;
    ryu::SwitchingNode fusion_node(nh);

    ros::spin();

    return 0;
}
