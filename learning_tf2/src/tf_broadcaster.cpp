
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);
	tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped tf_base_to_rot;
	geometry_msgs::TransformStamped tf_rot_to_obj;

	geometry_msgs::TransformStamped tf_base_to_rot2;
	geometry_msgs::TransformStamped tf_rot2_to_robot;

	geometry_msgs::TransformStamped tf_robot_to_camera;

	tf2::Quaternion q1a;
	tf2::Quaternion q1b;

	tf2::Quaternion q2a;
	tf2::Quaternion q2b;

	tf2::Quaternion q3;

	while(ros::ok())
	{

	tf_base_to_rot.header.stamp = ros::Time::now();
	tf_base_to_rot.header.frame_id = "base_frame";
	tf_base_to_rot.child_frame_id = "base_rotational_frame_1";
	q1a.setRPY(0.79, 0.0, 0.79);
	tf_base_to_rot.transform.rotation.x = q1a.x();
	tf_base_to_rot.transform.rotation.y = q1a.y();
	tf_base_to_rot.transform.rotation.z = q1a.z();
	tf_base_to_rot.transform.rotation.w = q1a.w();
	tf_base_to_rot.transform.translation.x = 0.0;
	tf_base_to_rot.transform.translation.y = 0.0;
	tf_base_to_rot.transform.translation.z = 0.0;

	tf_rot_to_obj.header.stamp = ros::Time::now();
	tf_rot_to_obj.header.frame_id = "base_rotational_frame_1";
	tf_rot_to_obj.child_frame_id = "object_frame";
	q1b.setRPY(0.0, 0.0, 0.0);
	tf_rot_to_obj.transform.rotation.x = q1b.x();
	tf_rot_to_obj.transform.rotation.y = q1b.y();
	tf_rot_to_obj.transform.rotation.z = q1b.z();
	tf_rot_to_obj.transform.rotation.w = q1b.w();
	tf_rot_to_obj.transform.translation.x = 0.0;
	tf_rot_to_obj.transform.translation.y = 1.0;
	tf_rot_to_obj.transform.translation.z = 1.0;


	tf_base_to_rot2.header.stamp = ros::Time::now();
	tf_base_to_rot2.header.frame_id = "base_frame";
	tf_base_to_rot2.child_frame_id = "base_rotational_frame_2";
	q2a.setRPY(0.0, 0.0, 1.5);
	tf_base_to_rot2.transform.rotation.x = q2a.x();
	tf_base_to_rot2.transform.rotation.y = q2a.y();
	tf_base_to_rot2.transform.rotation.z = q2a.z();
	tf_base_to_rot2.transform.rotation.w = q2a.w();
	tf_base_to_rot2.transform.translation.x = 0.0;
	tf_base_to_rot2.transform.translation.y = 0.0;
	tf_base_to_rot2.transform.translation.z = 0.0;

	tf_rot2_to_robot.header.stamp = ros::Time::now();
	tf_rot2_to_robot.header.frame_id = "base_rotational_frame_2";
	tf_rot2_to_robot.child_frame_id = "robot_frame";
	q2b.setRPY(0.0, 0.0, 0.0);
	tf_rot2_to_robot.transform.rotation.x = q2b.x();
	tf_rot2_to_robot.transform.rotation.y = q2b.y();
	tf_rot2_to_robot.transform.rotation.z = q2b.z();
	tf_rot2_to_robot.transform.rotation.w = q2b.w();
	tf_rot2_to_robot.transform.translation.x = 0.0;
	tf_rot2_to_robot.transform.translation.y = -1.0;
	tf_rot2_to_robot.transform.translation.z = 0.0;

	tf_robot_to_camera.header.stamp = ros::Time::now();
	tf_robot_to_camera.header.frame_id = "robot_frame";
	tf_robot_to_camera.child_frame_id = "camera_frame";
	tf_robot_to_camera.transform.translation.x = 0.0;
	tf_robot_to_camera.transform.translation.y = 0.1;
	tf_robot_to_camera.transform.translation.z = 0.1;
	q3.setRPY(0.0, 0.0, 0.0);
	tf_robot_to_camera.transform.rotation.x = q3.x();
	tf_robot_to_camera.transform.rotation.y = q3.y();
	tf_robot_to_camera.transform.rotation.z = q3.z();
	tf_robot_to_camera.transform.rotation.w = q3.w();

	br.sendTransform(tf_base_to_rot);
	br.sendTransform(tf_rot_to_obj);

	br.sendTransform(tf_base_to_rot2);
	br.sendTransform(tf_rot2_to_robot);
	
	br.sendTransform(tf_robot_to_camera);
	loop_rate.sleep();
	
	}
	
	return 0;

}