/**
 * @brief Lidar Sensor plugin
 * @file distance_sensor.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
#include <unordered_map>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/LaserScan.h>

#define RAD2CDEG(x) (x * (180.0 * 100.0 / 3.141592653589793238463))

namespace mavplugin {
class LidarSensorPlugin;

/**
 * @brief Lidar sensor mapping storage item
 */
class LidarSensorItem {
public:
	typedef boost::shared_ptr<LidarSensorItem> Ptr;

	LidarSensorItem() :
		send_tf(false),
		sensor_id(0),
		orientation(-1),
		covariance(0),
		owner(nullptr),
		data_index(0)
	{ }

	// params
	bool send_tf;		//!< defines if a transform is sent or not
	uint8_t sensor_id;      //!< id of the sensor
	Eigen::Vector3d position;	//!< sensor position
	int orientation;	//!< check orientation of sensor if != -1
	int covariance;		//!< in centimeters, current specification
	std::string frame_id;	//!< frame id for send

	// topic handle
	ros::Publisher pub;
	ros::Publisher pub_int;
	ros::Subscriber sub;
	std::string topic_name;

	LidarSensorPlugin *owner;

	void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg);
	static Ptr create_item(LidarSensorPlugin *owner, std::string topic_name);

private:
	std::vector<float> data;	//!< array allocation for measurements
	size_t data_index;		//!< array index

	/**
	 * Calculate measurements variance to send to the FCU.
	 */
	float calculate_variance(float range) {
		if (data.size() < 50)
			// limits the size of the array to 50 elements
			data.push_back(range);
		else {
			data[data_index] = range;	// it starts rewriting the values from 1st element
			if (++data_index > 49)
				data_index = 0;		// restarts the index when achieves the last element
		}

		float average, variance, sum = 0, sum_ = 0;

		/*  Compute the sum of all elements */
		for (auto d : data)
			sum += d;

		average = sum / data.size();

		/*  Compute the variance */
		for (auto d : data)
			sum_ += (d - average) * (d - average);

		variance = sum_ / data.size();

		return variance;
	}
};

/**
 * @brief Lidar sensor plugin
 *
 * This plugin allows publishing distance sensor data, which is connected to
 * an offboard/companion computer through USB/Serial, to the FCU or vice-versa.
 */
class LidarSensorPlugin : public MavRosPlugin {
public:
	LidarSensorPlugin() :
		dist_nh("~lidar_sensor"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		XmlRpc::XmlRpcValue map_dict;
		if (!dist_nh.getParam("", map_dict)) {
			ROS_WARN_NAMED("lidar_sensor", "DS: plugin not configured!");
			return;
		}

		ROS_ASSERT(map_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

		for (auto &pair : map_dict) {
			ROS_DEBUG_NAMED("lidar_sensor", "DS: initializing mapping for %s", pair.first.c_str());
			auto it = LidarSensorItem::create_item(this, pair.first);

			if (it)
				sensor_map[it->sensor_id] = it;
			else
				ROS_ERROR_NAMED("lidar_sensor", "DS: bad config for %s", pair.first.c_str());
		}
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_ANGULAR_DISTANCE_SENSOR, &LidarSensorPlugin::handle_lidar_sensor)
		};
	}

private:
	friend class LidarSensorItem;

	ros::NodeHandle dist_nh;
	UAS *uas;

	std::unordered_map<uint8_t, LidarSensorItem::Ptr> sensor_map;

	/* -*- low-level send -*- */
	void distance_sensor(uint32_t time_boot_ms,
			uint16_t min_distance,
			uint16_t max_distance,
			uint16_t start_angle,
			uint16_t end_angle,
			uint16_t angle_increment,
			uint16_t *ranges, uint8_t *covariances) {
		mavlink_message_t msg;
		mavlink_msg_angular_distance_sensor_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				min_distance,
				max_distance,
				start_angle,
				end_angle,
				angle_increment,
				ranges,
				covariances);
                mavlink_angular_distance_sensor_t *mp = (mavlink_angular_distance_sensor_t *)(msg.payload64);
		ROS_INFO_NAMED("sys", "MW distance_sensor 00-13 = [ %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d ]",
			mp->ranges[0], mp->ranges[1], mp->ranges[2], mp->ranges[3], mp->ranges[4], mp->ranges[5], mp->ranges[6],
			mp->ranges[7], mp->ranges[8], mp->ranges[9], mp->ranges[10], mp->ranges[11], mp->ranges[12], mp->ranges[13]);
		UAS_FCU(uas)->send_message(&msg);
		ROS_INFO_NAMED("sys", "MX distance_sensor 00-13 = [ %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d ]",
			mp->ranges[0], mp->ranges[1], mp->ranges[2], mp->ranges[3], mp->ranges[4], mp->ranges[5], mp->ranges[6],
			mp->ranges[7], mp->ranges[8], mp->ranges[9], mp->ranges[10], mp->ranges[11], mp->ranges[12], mp->ranges[13]);
	}

	void handle_lidar_sensor(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_angular_distance_sensor_t lidar_sen;
		mavlink_msg_angular_distance_sensor_decode(msg, &lidar_sen);

		auto sensor = sensor_map[0];
		auto cscan_int = boost::make_shared<sensor_msgs::LaserScan>();
		cscan_int->header = uas->synchronized_header(sensor->frame_id, lidar_sen.time_boot_ms);
		cscan_int->angle_min = lidar_sen.start_angle;
		cscan_int->angle_max = lidar_sen.end_angle;
		cscan_int->angle_increment = lidar_sen.angle_increment;
		cscan_int->time_increment = 0.1;
		cscan_int->scan_time = lidar_sen.time_boot_ms;
		cscan_int->range_min = lidar_sen.min_distance;
		cscan_int->range_max = lidar_sen.max_distance;

		float f_ranges[36];
		float f_covariances[36];

                // Instead of reading all 36 values - Attempt to limit the message size:-(
		for (int i = 0; i < 36; i++) {
			f_ranges[i] = 0.0;
			f_covariances[i] = 0.0;
		}

		ROS_INFO_NAMED("sys", "MY distance_sensor 00-13 = [ %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d, %03d ]",
			lidar_sen.ranges[0], lidar_sen.ranges[1], lidar_sen.ranges[2], lidar_sen.ranges[3], lidar_sen.ranges[4], lidar_sen.ranges[5], lidar_sen.ranges[6],
			lidar_sen.ranges[7], lidar_sen.ranges[8], lidar_sen.ranges[9], lidar_sen.ranges[10], lidar_sen.ranges[11], lidar_sen.ranges[12], lidar_sen.ranges[13]);

		for (int i = 0; i < 14; i++) {
			f_ranges[(42 - i) % 36] = (float)(lidar_sen.ranges[i]) / 100.0;
			f_covariances[(42 - i) % 36] = (float)(lidar_sen.covariances[i]);
		}

		cscan_int->ranges = std::vector<float>(f_ranges, f_ranges + sizeof f_ranges / sizeof f_ranges[0]);
		cscan_int->intensities = std::vector<float>(f_covariances, f_covariances + sizeof f_covariances / sizeof f_covariances[0]);

		sensor->pub_int.publish(cscan_int);
	}
};

#define RAD2DEG(x) (x * 180.0 / 3.141592653589793238463)
#define DEG2RAD(x) (x * 3.141592653589793238463 / 180.0)

void LidarSensorItem::scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	const float combine_values = 10.0;

        if (msg->ranges.size() != (msg->ranges.size() / combine_values) * combine_values) {
		ROS_ERROR_NAMED("lidar_sensor", "scan_cb ranges.size (%d) is not multiple of combine_vales (%d)", msg->ranges.size(), (int)combine_values);
		return;
	}

	auto cscan = boost::make_shared<sensor_msgs::LaserScan>();
	cscan->header = msg->header;
	cscan->angle_min = DEG2RAD(5.0);
	cscan->angle_max = DEG2RAD(355.0);
	cscan->angle_increment = DEG2RAD(10.0);
	cscan->time_increment = msg->time_increment;
	cscan->scan_time = msg->scan_time;
	cscan->range_min = msg->range_min;
	cscan->range_max = msg->range_max;

	float f_ranges[36];
	float f_covariances[36];
	uint16_t ranges[36];
	uint8_t covariances[36];

	for (int i = 0; i < 36; i++) {
		f_ranges[i] = std::numeric_limits<float>::infinity();
		f_covariances[i] = 0.0;
		ranges[i] = 0;
		covariances[i] = 0;
	}

	for (int i = 0; i < msg->intensities.size(); i++) {
		int deg_angle = (360 + (int)RAD2DEG(msg->angle_min + i * msg->angle_increment)) % 360;
		int index = deg_angle / 10;

		if (msg->intensities[i] > 0.0) {
			if (msg->intensities[i] > f_covariances[index]) {
				f_covariances[index] = msg->intensities[i];
			}

			if (msg->ranges[i] < f_ranges[index]) {
				f_ranges[index] = msg->ranges[i];
			}
		}
	}

	for (int i = 0; i < 36; i++) {
		if (f_ranges[i] != std::numeric_limits<float>::infinity()) {
			float f = f_ranges[i];

			if (f < msg->range_min && f > (msg->range_min - 0.05)) { // XXX 5cm fudge factor
				f = msg->range_min;
			}
			else if (f > msg->range_max && f < (msg->range_max + 1.5)) { // 150cm fudge factor for long range
				f = msg->range_max;
			}

			ranges[i] = (uint16_t)(f * 100.0); // in cm
		}
		else {
			ranges[i] = 0;
		}
		covariances[i] = (f_covariances[i] > 0.0 ? covariance : 0);
	}

	// owner->distance_sensor(
	// 		msg->header.stamp.toNSec() / 1000000,
	// 		(uint16_t)(msg->range_min * 100.0),
	// 		(uint16_t)(msg->range_max * 100.0),
	// 		RAD2CDEG(msg->angle_min + (combine_values / 2.0) * msg->angle_increment),
	// 		RAD2CDEG(msg->angle_max),
	// 		RAD2CDEG(msg->angle_increment * combine_values),
	// 		ranges, covariances);

	uint16_t ranges_1[14];
	uint8_t covariances_1[14];
	for (int i = 0; i < 14; i++) {
		ranges_1[i] = ranges[(42 - i) % 36];
		covariances_1[i] = covariances[(42 - i) % 36];
	}
	owner->distance_sensor(
			msg->header.stamp.toNSec() / 1000000,
			(uint16_t)(msg->range_min * 100.0),
			(uint16_t)(msg->range_max * 100.0),
			RAD2CDEG(msg->angle_min + (combine_values / 2.0) * msg->angle_increment),
			RAD2CDEG(msg->angle_max),
			RAD2CDEG(msg->angle_increment * combine_values),
			ranges_1, covariances_1);

	cscan->ranges = std::vector<float>(f_ranges, f_ranges + sizeof f_ranges / sizeof f_ranges[0]);
	cscan->intensities = std::vector<float>(f_covariances, f_covariances + sizeof f_covariances / sizeof f_covariances[0]);
	
	pub.publish(cscan);
}

LidarSensorItem::Ptr LidarSensorItem::create_item(LidarSensorPlugin *owner, std::string topic_name)
{
	auto p = boost::make_shared<LidarSensorItem>();

	ros::NodeHandle pnh(owner->dist_nh, topic_name);

	p->owner = owner;
	p->topic_name = topic_name;

	p->sensor_id = 0;

	// subscriber params

	// optional
	pnh.param("covariance", p->covariance, 10);

	// create topic handles
	p->pub = owner->dist_nh.advertise<sensor_msgs::LaserScan>(topic_name + "_combined", 10);
	p->pub_int = owner->dist_nh.advertise<sensor_msgs::LaserScan>(topic_name + "_combined_int", 10);
	p->sub = owner->dist_nh.subscribe("/" + topic_name, 10, &LidarSensorItem::scan_cb, p.get());

	return p;
}
}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::LidarSensorPlugin, mavplugin::MavRosPlugin)
