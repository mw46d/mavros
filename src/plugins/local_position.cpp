/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {

/**
 * @brief Local position plugin.
 * Publish local position to TF and PositionStamped,
 * send local position and visual position estimates to FCU.
 */
class LocalPositionPlugin : public MavRosPlugin {
public:
	LocalPositionPlugin()
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		pos_nh = ros::NodeHandle(nh, "position");

		pos_nh.param("send_tf", send_tf, true);
		pos_nh.param<std::string>("parent_frame_id", parent_frame_id, "local_origin");
		pos_nh.param<std::string>("frame_id", frame_id, "fcu");

		local_position = pos_nh.advertise<geometry_msgs::PoseStamped>("local", 10);
	}

	std::string const get_name() const {
		return "LocalPosition";
	}

	std::vector<uint8_t> const get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_LOCAL_POSITION_NED
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);
		handle_local_position_ned(pos_ned);
	}

private:
	UAS *uas;

	ros::NodeHandle pos_nh;
	ros::Publisher local_position;
	tf::TransformBroadcaster tf_broadcaster;

	std::string parent_frame_id;	//!< origin for TF
	std::string frame_id;		//!< frame for TF and Pose
	bool send_tf;

	void handle_local_position_ned(mavlink_local_position_ned_t &pos_ned) {
		ROS_DEBUG_THROTTLE_NAMED(10, "locpos", "Local position NED: boot_ms:%06d "
				"position:(%1.3f %1.3f %1.3f) speed:(%1.3f %1.3f %1.3f)",
				pos_ned.time_boot_ms,
				pos_ned.x, pos_ned.y, pos_ned.z,
				pos_ned.vx, pos_ned.vy, pos_ned.vz);

		/* TODO: check convertion to ENU
		 * velocity stored in ENU
		 */
		tf::Vector3 avel = uas->get_attitude_angular_velocity();
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos_ned.x, -pos_ned.y, -pos_ned.z));
		transform.setRotation(tf::createQuaternionFromRPY(avel.x(), avel.y(), avel.z()));

		if (send_tf)
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						ros::Time::now(),
						parent_frame_id, frame_id));

		// publish pose
		geometry_msgs::PoseStampedPtr pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(transform, pose->pose);
		pose->header.stamp = ros::Time::now();
		pose->header.frame_id = frame_id;

		local_position.publish(pose);
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LocalPositionPlugin, mavplugin::MavRosPlugin)

