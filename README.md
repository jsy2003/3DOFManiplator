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

[service file ] - [Floats_array.srv]
```
    float32 req
    ---
    float32[] res
```


[CMakeList.txt file]
```
    find_package(catkin REQUIRED COMPONENTS
        actionlib
        control_msgs
        controller_manager
        roscpp
        rospy
        std_msgs
        std_srvs
        trajectory_msgs
        visualization_msgs
        message_generation
    )

    ## Generate services in the 'srv' folder
    add_service_files(
        FILES
        Floats_array.srv
        #   Service2.srv
    )

    ## Generate added messages and services with any dependencies listed here
    generate_messages(
        DEPENDENCIES
        std_msgs #   control_msgs#     trajectory_msgs#   visualization_msgs
    )

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
    )

    add_executable(robot_hardware_interface src/robot_hardware_interface_node.cpp)
    target_link_libraries(robot_hardware_interface ${catkin_LIBRARIES})
```

[package.xml file]
```
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>actionlib</build_depend>
    <build_depend>control_msgs</build_depend>
    <build_depend>controller_manager</build_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>rospy</build_depend>
    <build_depend>std_msgs</build_depend>
    <build_depend>std_srvs</build_depend>
    <build_depend>trajectory_msgs</build_depend>
    <build_depend>visualization_msgs</build_depend>
    <build_depend>message_generation</build_depend>
  
    <build_export_depend>actionlib</build_export_depend>
    <build_export_depend>control_msgs</build_export_depend>
    <build_export_depend>controller_manager</build_export_depend>
    <build_export_depend>roscpp</build_export_depend>
    <build_export_depend>rospy</build_export_depend>
    <build_export_depend>std_msgs</build_export_depend>
    <build_export_depend>std_srvs</build_export_depend>
    <build_export_depend>trajectory_msgs</build_export_depend>
    <build_export_depend>visualization_msgs</build_export_depend>
  
    <exec_depend>actionlib</exec_depend>
    <exec_depend>control_msgs</exec_depend>
    <exec_depend>controller_manager</exec_depend>
    <exec_depend>roscpp</exec_depend>
    <exec_depend>rospy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>std_srvs</exec_depend>
    <exec_depend>trajectory_msgs</exec_depend>
    <exec_depend>visualization_msgs</exec_depend>
    <exec_depend>message_runtime</exec_depend>
```

[launch file] - [check_urdf.launch]
```
    <?xml version="1.0"?>
    <launch>

        <arg name="model" default="$(find three_dof_planar_manipulator)/urdf/three_dof_planar_manipulator.urdf.xacro"/>
        <arg name="gui" default="true" />
        
        <param name="robot_description" command="$(find xacro)/xacro.py $(arg model)" />
        <param name="use_gui" value="$(arg gui)"/>
        
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" >
        </node>

        <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" /> 
  
        <node name="rviz" pkg="rviz" type="rviz"/> 
     </launch>
```

[launch file] - [check_motor_control.launch]
```
    <?xml version="1.0"?>
    <launch>
    
        <rosparam file="$(find three_dof_planar_manipulator)/config/controllers.yaml" command="load"/>
   
        <arg name="model" default="$(find three_dof_planar_manipulator)/urdf/three_dof_planar_manipulator.urdf.xacro"/>
        <arg name="gui" default="true" />
  
        <param name="robot_description" command="$(find xacro)/xacro.py $(arg model)" />
        <param name="use_gui" value="$(arg gui)"/>

        <node name="robot_hardware_interface" pkg="three_dof_planar_manipulator" type="robot_hardware_interface" output="screen"/>

        <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" >
        </node>
  
        <node name="rviz" pkg="rviz" type="rviz"/>
    
        <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
            args="
        	    /three_dof_planar_manipulator/joints_update
                /three_dof_planar_manipulator/joint1
                /three_dof_planar_manipulator/joint2
                /three_dof_planar_manipulator/joint3
            "/>
    </launch>
```

[scripts file] - [motion file] - [ik_marker.py]
```
    #!/usr/bin/env python
    
    import rospy
    import tf
    import copy
    
    import moveit_commander
    import moveit_msgs.msg
    import geometry_msgs.msg
    from math import pi
    from std_msgs.msg import string
    from moveit_commander.conversions import pose_to_list
    from geometry_msgs.msg import Pose
    
    from interactive_markers.interactive_marker_server import *
    from interactive_markers.menu_handler import *
    from visualization_msgs.msg import *
    from gemoetry_msgs.msg import Pose
    
    viapoints=[]
    viapoints_marker = MakerArray()
    count=0
    
    if __name__=="__main__":
        rospy.init_node("simple_marker")
        init_marker = InteractiveMarker()
        menu_handler = MenuHandler()
        
    
    
    
```

[scripts file] - [Vision file] - [color_threaholding.py]
```
```
[scripts file] - [Vision file] - [track_blob.py]
```
```
[scripts file] - [motion file] - [execute_motion.py]
```
```







