# 3 DOF Maniplator

## 0. 3DOF Maniplator workspace 를 작성
### 1. URDF 작성
### 2. Moveit! 설정

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
            
            ros::Publisher          pub;
            ros::ServiceClient      client;
            rospy::tutorials::Floats                        joint_pub;
            three_dof_plannar_manipulator::Floats_array     joint_read;
            
        protected:
            hardware_interface::JointStateInterface         joint_state_interface_;
            hardware_interfrace::PositionJointInterface     position_joint_interface_;
            
            joint_limits_interface::PositionJointSaturationInterface        position_joint_saturation_interface_;
            joint_limits_interface::PositionJointSaturationLimitsInterface  positionJointSoftLimitsInterface_;
            
            int num_joints_;
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
```

[robot_hardware_interface_node.cpp]
```
    #include <three_dof_plannar_manipulator/robot_hardware_interface.h>
    
     ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle &nh) : (nh_(nh)
     {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
            
        loop_hz_ = 4;
        ros::Duration  update_freq = ros::Duration(1.0/loop_hz_);
   
        pub = nh_.advertise<rospy_tutorials::Floats>("/joint_to_arduino", 10);
        client = nh_.serviceClient<three_dof_plannar_manipulator::Floats_array>("read_joint_state");
        
         non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
     }
     
     ROBOTHardwareInterface::~ROBOTHardwareInterface()
     {
     }
     
     void ROBOTHardwareInterface::init()
     {
        num_joints_; 3;
        joint_name_[0] = "joint1";
        joint_name_[1] = "joint2";
        joint_name_[2] = "joint3";
        
        for(int i=0;i<num_joints_;i++)
        {
            // create joint state interface
            hardware_interface::JointStateHandle    jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);
            
            // Create position joint interface
            hardware_interface::JointHandle     jointPositionHandle(jointStateHandle, &joint_commands_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);
        }
        
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
     }
     
     void ROBOTHardwareInterface::update(const ros::TimerEvent &e)
     {
          elapsed_time_ = ros::Duration(e.current_real - e.last_real);
          read();
          controller_manager_->update(ros::Time::now(), elapsed_time_);
          write(elapsed_time_);
     }
     
     void ROBOTHardwareInterface::read()
     {
        joint_read.request.req = 1.0;
        if(client.call(joint_read))
        {
            joint_position_[0] = angles::from_degree(90-joint_read.response.res[0]);
            joint_position_[1] = angles::from_degree(joint_read.response.res[1]-90);
            joint_position_[2] = angles::from_degree(joint_read.response.res[2]-90);
        }
        else
        {
            joint_position_[0] = 0;
            joint_position_[1] = 0;
            joint_position_[2] = 0;
        }
     }
     
     void ROBOTHardwareInterface::write(ros::Duration elapsed_time)
     {
        joint_pub.data.clear();
        joint_pub.data.push_back(90- ( angles::to_degree(joint_position_command_[0])));
        joint_pub.data.push_back(90+ ( angles::to_degree(joint_position_command_[1])));
        joint_pub.data.push_back(90+ ( angles::to_degree(joint_position_command_[2])));
        
        pub.publish(joint_pub);
     }
     
     int main(int argc, char **argv)
     {
        ros::init(argc, argv, "robot_hardware_interface");
        ros::NodeHandle nh;
        
        // 2 threads for controller service and for the Service client used to get the feedback from ardiuno
        ros::MultiThreadSpinner spinner(2);
        spinner.spin();
        return 0;
     }
```

[controller.yaml]
```
    three_dof_plannar_manipulator:
        #publish all joint states
        joint_update:
            type:joint_state_controller/JointStateController
            publish_rate:50
        
        #position controller
        joint1:
            type:position_controller/JointPositionController
            joint:joint1
            pid:{p:1.0, i:1.0, d:0.0}
            
        joint2:
            type:position_controller/JointPositionController
            joint:joint2
            pid:{p:1.0, i:1.0, d:0.0}
            
        joint3:
            type:position_controller/JointPositionController
            joint:joint3
            pid:{p:1.0, i:1.0, d:0.0}
```



