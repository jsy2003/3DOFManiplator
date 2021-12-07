# 3 DOF Maniplator

## 0. 3DOF Maniplator workspace 를 작성

## 1. 로봇의 hardware_interface 노드를 작성
[robot_hardware_interface.h]
```
    #include <hardware_interface/joint_state_interface.h>
    #include <hardware_interface/joint_command_interface.h>
    #include <hardware_interface/robot_hw.h>
    #include <joint_limits_interface/joint_limits.h>
    #include <joint_limits_interface/joint_limits_interface.h>
    #include <joint_limits_interface/joint_limits_rosparam.h>
    #include <joint_limits_interface/joint_limits_urdf.h>
    #include <controller_manager/controller_manager.h>
    #include <boost/scoped_ptr.h>
    #include <ros/ros.h>
    #include <rospy_tutorials/Floats.h>
    #include <three_dof_plannar_manipulator/Floats_array.h>
    #include <angles/angles.h>
    
    class ROBOTHardwareInterface : public hardward_interface::RobotHW
    {
        public:
            ROBOTHardwareInterface(ros::NodeHandle &nh);
            ~ROBOTHardwareInterface();
            void init();
            void update(const ros::TimerEvent &e);
            void read();
            void write(ros::Duration elapsed_time)
            
            ros::Publisher      pub;
            ros::ServiceClient  client;
            rospy::tutorials::FLoats    joint_pub;
            three_dof_plannar_manipulator::Floats::array    joint_read;
            
        protected:
            hardware_interface::JointStateInterface         joint_state_interface_;
            hardware_interfrace::PositionJointInterface     position_joint_interface_;
            
            joint_limits_interface::PositionJointSaturationInterface        position_joint_saturation_interface_;
            joint_limits_interface::PositionJointSaturationLimitsInterface  positionJointSoftLimitsInterface_;
            
            int num_joint_;
            std::string joint_names_[3];
            double joint_position_[3];
            double joint_velocity_[3];
            double joint_effort_[3];
            double joint_commands_[3];
            
            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration elapsed_time_;
            double loop_hz_;
            bootst::shared_ptr<controller_manager::ControllerManager>   controller_manage_;
    };
    
    
