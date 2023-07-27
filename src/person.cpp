#include <people_msgs_utils/person.h>

namespace people_msgs_utils {

Person::Person(const people_msgs::Person& person):
	Person(person.name, person.position, person.velocity, person.reliability, person.tagnames, person.tags)
{}

Person::Person(
	const std::string& name,
	const geometry_msgs::Point& position,
	const geometry_msgs::Point& velocity,
	const double& reliability,
	const std::vector<std::string>& tagnames,
	const std::vector<std::string>& tags
):
	name_(name),
	reliability_(reliability),
	occluded_(true),
	matched_(false),
	detection_id_(0),
	track_age_(0),
	group_age_(0)
{
	pose_.pose.position = position;
	// initial guess on orientation, may be adjusted using 'tags'
	tf2::Quaternion quat;
	quat.setRPY(0, 0, std::atan2(velocity.y, velocity.x));
	pose_.pose.orientation.x = quat.getX();
	pose_.pose.orientation.y = quat.getY();
	pose_.pose.orientation.z = quat.getZ();
	pose_.pose.orientation.w = quat.getW();

	vel_.pose.position = velocity;
	// initial guess on theta velocity
	vel_.pose.orientation.w = 1.0;

	// initial guess on COG of the group
	group_center_of_gravity_ = pose_.pose.position;

	// Basic data was saved in initializer list.
	// Now, check if tags contain some fancy data
	parseTags(tagnames, tags);
}

bool Person::parseStringBool(const std::string& str) {
	if (str == "True" || str == "true" || str == "1") {
		return true;
	}
	return false;
}

bool Person::parseTags(const std::vector<std::string>& tagnames, const std::vector<std::string>& tags) {
	if ((tagnames.size() != tags.size()) || tagnames.empty()) {
		// no additional data can be retrieved
		return false;
	}

	// create iterators for tagnames and tags
	const std::string DELIMITER = " ";
	std::vector<std::string>::const_iterator tag_value_it = tags.begin();
	for (
		std::vector<std::string>::const_iterator tag_it = tagnames.begin();
		tag_it != tagnames.end();
		tag_it++
	) {
		if (tag_it->find("orientation") != std::string::npos) {
			auto orient_components = parseString<double>(*tag_value_it, DELIMITER);
			if (orient_components.size() == 4) {
				pose_.pose.orientation.x = orient_components.at(0);
				pose_.pose.orientation.y = orient_components.at(1);
				pose_.pose.orientation.z = orient_components.at(2);
				pose_.pose.orientation.w = orient_components.at(3);
			}
		} else if (tag_it->find("pose_covariance") != std::string::npos) {
			auto cov = parseString<double>(*tag_value_it, DELIMITER);
			if (cov.size() == 36) {
				std::copy(cov.begin(), cov.end(), pose_.covariance.begin());
			}
		} else if (tag_it->find("twist_covariance") != std::string::npos) {
			auto cov = parseString<double>(*tag_value_it, DELIMITER);
			if (cov.size() == 36) {
				std::copy(cov.begin(), cov.end(), vel_.covariance.begin());
			}
		} else if (tag_it->find("occluded") != std::string::npos) {
			occluded_ = parseStringBool(*tag_value_it);
		} else if (tag_it->find("matched") != std::string::npos) {
			matched_ = parseStringBool(*tag_value_it);
		} else if (tag_it->find("detection_id") != std::string::npos) {
			detection_id_ = static_cast<unsigned int>(std::stoul(*tag_value_it));
		} else if (tag_it->find("track_age") != std::string::npos) {
			track_age_ = static_cast<unsigned int>(std::stoul(*tag_value_it));
		} else if (tag_it->find("group_id") != std::string::npos) {
			group_id_ = *tag_value_it;
		} else if (tag_it->find("group_age") != std::string::npos) {
			group_age_ = static_cast<unsigned int>(std::stoul(*tag_value_it));
		} else if (tag_it->find("group_track_ids") != std::string::npos) {
			group_track_ids_ = parseString<unsigned int>(*tag_value_it, DELIMITER);
		} else if (tag_it->find("group_center_of_gravity") != std::string::npos) {
			auto pos_v = parseString<double>(*tag_value_it, DELIMITER);
			if (pos_v.size() == 3) {
				group_center_of_gravity_.x = pos_v.at(0);
				group_center_of_gravity_.y = pos_v.at(1);
				group_center_of_gravity_.z = pos_v.at(2);
			}
		}
		tag_value_it++;
 	}
	return true;
}

} // namespace people_msgs_utils
