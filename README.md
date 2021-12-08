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
    
    def processFeedback(feedback):
        print("+++++++++++',feedback.menu_entry_id,'+++++++++++++++++++++")
        
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:
                group.set_position_target([feedback.pose.position.x, feedback.pose.position.y,feedback.pose.position.z])
                # group.set_pose_target(feedback.pose) 
                # give for 6 or more dof arms
                plan = group.go(wait=True)
                group.stop()
                group.clear_pose_targets()
                # pub.publish(feedback.pose)
            
            elif feedback.menu_entry_id == 2:
                listener.waitForTransform("eff", "link1", rospy.Time(), rospy.Duration(1.0))
                (trans, rot) = listener.lookupTransform("link1", "eff", rospy.Time())
                print trans, rot
                
                feedback.pose.position.x = trans[0]
                feedback.pose.position.y = trans[1]
                feedback.pose.position.z = trans[2]
                feedback.pose.rotation.x = trans[0]
                feedback.pose.rotation.y = trans[1]
                feedback.pose.rotation.z = trans[2]
                feedback.pose.rotation.w = trans[3]
                server.SetPose(feedback.marker_name, feedback.pose)
            
            elif feedback.menu_entry_id == 4:   
                viapoints.append(feedback.pose)
                print("count :", len(vispoints))
                marker.Marker()
                marker.header.frame_id = "link1"
                marker.type = Marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.005
                marker.scale.y = 0.005
                marker.scale.z = 0.005
                marker.color.r = 1
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1.0
                marker.pose = feedback.pose
                viapoints_marker.markers.append(marker)
                id = 0
                for m in viapoints_marker.markers:
                    m.id = id
                    id += 1
                marker_pub.publish(viapoints_marker)
          
            elif feedback.menu_entry_id == 5:
                if len(viapoints) > 0:
                    (plan, fraction) = group.Compute_cartesian_path(viapoints, 0.01, 0.0)
                    display_traj = moveit_msgs.msg.DisplayTrajectory()
                    display_traj.trajectory_start = robot.get_current_state()
                    display_traj.trajectory.append(plan)
                    display_traj_pub.publish(display_traj)
                    froup.execute(plan, wait = True)
                    
            elif feedback.menu_entry_id == 6: 
                if len(vispoints_marker.markers) > 0:
                    viapoints[:] = []
                    for m in vispoints_marker.markers:
                        m.action = Marker().DELETE
                    marker_pub.publish(viapoints_marker)
                    print ("Clearted all viapoints")
                else
                    print("No via points set")
                    
        server.applyChanges()
    
    if __name__=="__main__":
        rospy.init_node("simple_marker")
        init_marker = InteractiveMarker()
        menu_handler = MenuHandler()
            
        listener = tf.TransformListener()
        display_traj_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        marker_pub = rospy.Publisher("/viapoints", MarkerArray, queue_size=20)
        
        # moveit start
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        
        # Create an interface marker server on the topic namespace simple marker
        server = InteractiveMarkerServer("simple_marker")
        
        # Create an interactive marker for our server
        int_marker.header.frame_id = "link1"
        int_marker.name = "my_marker"
        int_marker.description = "simple 1 DOF Control"
        int_marker.scale = 0.05
        
       listener.waitForTransform("/eff", "/link1", rospy.Time(), rospy.Duration(1.0))
       print listener.frameExists("link1")
       print listener.frameExists("eff")
       (trans, rot) = listener.lookupTransform("link1", "eff", rospy.Time())
       print trans, rot
       
       int_marker.pose.position.x = trans[0]
       int_marker.pose.position.y = trans[1]
       int_marker.pose.position.z = trans[2]
       int_marker.pose.rotation.x = rot[0]
       int_marker.pose.rotation.y = rot[1]
       int_marker.pose.rotation.z = rot[2]
       int_marker.pose.rotation.w = rot[3]
        
       marker = Marker()
       marker.type = Marker.SPHERE
       marker.scale.x = 0.01
       marker.scale.y = 0.01
       marker.scale.z = 0.01
       marker.color.r = 1
       marker.color.g = 0
       marker.color.b = 0
       marker.color.a = 1.0
       
       rotate_control = InteractiveMarkerControl()
       rotate_control.orientation.w = 1
       rotate_control.orientation.x = 0
       rotate_control.orientation.y = 1
       rotate_control.orientation.z = 0
       rotate_control.name = "moving"
       rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
       rotate_control.always_visible = True
       roate_control.markers.append(marker)
        
       int_marker.controls.append(rotate_control)
       
       # add the interactive marker to our collection &
	   # tell the server to call processFeedback() when feedback arrives for it
       server.insert(int_marker, processFeedback)
       menu_handler.insert("Execute Motion", callback=processFeedback)
       menu_handler.insert("move to endeff", callback=processFeedback)
       
       viaPoints = menu_handler.insert("Via Points")
       
       menu_handler.insert("Add via-point", parent=ViaPoints, callback=processFeedback)
       menu_handler.insert("Execute Trajectory", parent=ViaPoints, callback=processFeedback)
       menu_handler.insert("Reset via-points", parent=ViaPoints, callback=processFeedback)
       
       menu_handler.apply(server, int_marker.name)
       
       # 'commit' changes and send to all clients
       server.applyChanges()
       rospy.spin()
```

[scripts file] - [Vision file] - [color_threaholding.py]
```
	#!/usr/bin/env python
		
	import cv2
	import numpy as np
	
	def nothing(pos):
		pass
		
	cap = cv2.VideoCapture(1)
	cv2.namedWindow("Thresholds")
	cv2.createTracker("LH", "Thresholds", 0, 255, nothing)
	cv2.createTracker("LS", "Thresholds", 0, 255, nothing)
	cv2.createTracker("LV", "Thresholds", 0, 255, nothing)
	cv2.createTracker("UH", "Thresholds", 255, 255, nothing)
	cv2.createTracker("US", "Thresholds", 255, 255, nothing)
	cv2.createTracker("UV", "Thresholds", 255, 255, nothing)
	
	while(1):
		_,img = cap.read()
		
		# converting frame(img i.e BGR) to HSV (hue-saturation-value)
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		
		lh = cv2.getTrackerbarPos("LH", "thresholds")
		ls = cv2.getTrackerbarPos("LS", "thresholds")
		lv = cv2.getTrackerbarPos("LV", "thresholds")
		
		uh = cv2.getTrackerbarPos("UH", "thresholds")
		us = cv2.getTrackerbarPos("US", "thresholds")
		uv = cv2.getTrackerbarPos("UV", "thresholds")
		
		# defining the Range of color
		color_lower = np.array([lh,ls,lv], np.uint8)
		color_upper = np.array([uh,us,uv], np.uint8)
		
		# finding the range of color in the image
		color = cv2.inRange(hsv, color_lower, color_upper)
		
		# Morphological transformation, Dilation 
		kernel = np.ones((5,5), "uint8")
		color = cv2.diate(color, kernel)
		cv2.imshow("color", color)
		cv2.imshow("org", img)
		
		if cv2.waitKey(1) == ord('q'):
			break
	cap.release()
	cv2.destoryAllWindoes()
```


[scripts file] - [Vision file] - [track_blob.py]
```
	#!/usr/bin/env python
	
	import rospy
	from std_msgs.msg import Int32
	from geometry_msgs.msg import Pose
	
	import sys
	import cv2
	import numpy as np
	
	rospy.init_node("track_blob")
	cap = cv2.VideoCapture(1)
	
	pub = rospy.Publisher("follow_blob", Pose, queue_size=10)
	target_pose = Pose()
	
	x_d = 0.0
	y_d = 0.0
	x_d_p = 0.0
	y_d_p = 0.0
	
	while(1):
		_, img = cap.read()
		
		# converting frame(img i.e BGR) to HSV (hue-saturation-value)
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		blue_lower = np.array([94,123,46], np.uint8)
		blue_upper = np.array([125,255,255], np.uint8)
		blue = cv2.inRange(hsv, blue_lower, blue_upper)
		
		# Morphological transformation, Dilation  	
		kernel = np.ones((5,5), "uint8")
		blue = cv2.dialte(blue, kernel)
		img = cv2.circle(img, (260,80),5,(255,0,0), -1)
		
		# Tracking the Blue Color
		(_,contours, hierarch) = cv2.findContours(blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		if len(contours) > 0:
			contour = max(contours, key=cv2.contourArea)
			area = cv2.contourArea(contour)
			if area > 800:
				x,y,w,h = cv2.boundingRect(contour)
				img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
				img=cv2.circle(img,((2*x+w)/2,(2*y+h)/2),5,(255,0,0),-1)
				img=cv2.line(img,(260,68),((2*x+w)/2,(2*y+h)/2),(0,255,0),2)
				
				x_d= (((2*y+h)/2)-68) * 0.06
				y_d= (((2*x+w)/2)-260) * 0.075
				s= 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)
				cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
				
				if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):
					target_pose.position.x=x_d*0.01
					target_pose.position.y=y_d*0.01
					target_pose.position.z=0.0
					pub.publish(target_pose)
			
					x_d_p=x_d
					y_d_p=y_d
					
			cv2.imshow("Mask",blue)
			cv2.imshow("Color Tracking",img)
			if cv2.waitKey(1)== ord('q'):
				break	
				
	cap.release()
	cv2.destroyAllWindows()			

```


[scripts file] - [motion file] - [execute_motion.py]
```
	#!/usr/bin/env python
	
	import rospy
	import sys
	import tf
	import moveit_commander
	import moveit_msgs.msg
	import geometry_msgs.msg
	
	from math import pi
	from std_msgs.msg import String
	from moveit_commander.conversions import pose_to_list
	from geometry_msgs.msg import Pose
	from std_msgs.msg import Int32
	
	
	rospy.init_node("subscriber_py")
	
	# moveit start
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "arm"
	group = moveit_commander.MoveGroupCommander(group_name)
	
	listener=tf.TransformListener()
	listener.waitForTransform('/eff','/link1',rospy.Time(), rospy.Duration(1.0))
	print listener.frameExists('link1')
	print listener.frameExists('eff')
	(trans,rot)=listener.lookupTransform('link1','eff',rospy.Time())
	print trans,rot

	rospy.Subscriber("follow_blob", Pose, my_callback, queue_size=10) 
	rospy.loginfo("subscriber_py node started and subscribed to topic_py") #debug statement
	rospy.spin()
	
```










