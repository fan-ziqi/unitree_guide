#include <iostream>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <xpp_msgs/RobotStateCartesian.h>

int main()
{
	std::string src_bag = "/mnt/f/我的文件/研究生/四足项目/宇树/unitree_guide/src/thirdparty/xpp_msgs/bags/cyberdog_bag_15161.bag";
	std::string state_topic = "/xpp/state_des";

	rosbag::Bag bag;
	bag.open(src_bag, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string(state_topic));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	// rosbag::View view(bag);
	for(auto m: view)
	{
		xpp_msgs::RobotStateCartesian::ConstPtr state = m.instantiate<xpp_msgs::RobotStateCartesian>();
		if(state == nullptr)
		{
			std::cout << "null " << std::endl;
		}
		else
		{
			std::cout << "time_from_start:\t" << state->time_from_start << std::endl;
			std::cout << "base:\t" << std::endl;
			std::cout << "\tpose:\t" << std::endl;
			std::cout << "\t\tposition:\t"
			          << state->base.pose.position.x << "\t"
			          << state->base.pose.position.y << "\t"
			          << state->base.pose.position.z << std::endl;
			std::cout << "\t\torientation:\t"
			          << state->base.pose.orientation.x << "\t"
			          << state->base.pose.orientation.y << "\t"
			          << state->base.pose.orientation.z << "\t"
			          << state->base.pose.orientation.w << std::endl;

			std::cout << "\ttwist:\t" << std::endl;
			std::cout << "\t\tlinear:\t"
			          << state->base.twist.linear.x << "\t"
			          << state->base.twist.linear.y << "\t"
			          << state->base.twist.linear.z << std::endl;
			std::cout << "\t\tangular:\t"
			          << state->base.twist.angular.x << "\t"
			          << state->base.twist.angular.y << "\t"
			          << state->base.twist.angular.z << std::endl;

			std::cout << "\taccel:\t" << std::endl;
			std::cout << "\t\tlinear:\t"
			          << state->base.accel.linear.x << "\t"
			          << state->base.accel.linear.y << "\t"
			          << state->base.accel.linear.z << std::endl;
			std::cout << "\t\tangular:\t"
			          << state->base.accel.angular.x << "\t"
			          << state->base.accel.angular.y << "\t"
			          << state->base.accel.angular.z << std::endl;

			std::cout << "\tee_motion:\t" << std::endl;
			for(auto &ee: state->ee_motion)
			{
				std::cout << "\t\tpos:\t"
				          << ee.pos.x << "\t"
				          << ee.pos.y << "\t"
				          << ee.pos.z << std::endl;
				std::cout << "\t\tvel:\t"
				          << ee.vel.x << "\t"
				          << ee.vel.y << "\t"
				          << ee.vel.z << std::endl;
				std::cout << "\t\tacc:\t"
				          << ee.acc.x << "\t"
				          << ee.acc.y << "\t"
				          << ee.acc.z << std::endl;
			}

			std::cout << "\tee_forces:\t" << std::endl;
			for(auto &ee_force: state->ee_forces)
			{
				std::cout << "\t\t"
				          << ee_force.x << "\t"
				          << ee_force.y << "\t"
				          << ee_force.z << std::endl;
			}

			std::cout << "\tee_contact:\t" << std::endl;
			for(auto &contact: state->ee_contact)
			{
				if(contact == true) std::cout << "1\t" << contact;
				else std::cout << "0\t" << contact;
			}
			std::cout << std::endl << std::endl;

		}
	}
	bag.close();
	return 0;
}