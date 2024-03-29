#include <people_msgs_utils/person.h>
#include <people_msgs_utils/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

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
	track_age_(0)
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

	// Basic data was saved in initializer list.
	// Now, check if tags contain some fancy data
	parseTags(tagnames, tags);
}

Person::Person(
	const std::string& name,
	const geometry_msgs::PoseWithCovariance& pose,
	const geometry_msgs::PoseWithCovariance& velocity,
	const double& reliability,
	bool occluded,
	bool matched,
	unsigned int detection_id,
	unsigned long int track_age,
	const std::string& group_name
):
	name_(name),
	pose_(pose),
	reliability_(reliability),
	vel_(velocity),
	occluded_(occluded),
	matched_(matched),
	detection_id_(detection_id),
	track_age_(track_age),
	group_id_(group_name)
{}

void Person::transform(const geometry_msgs::TransformStamped& transform) {
	// transform pose with covariance
	geometry_msgs::PoseWithCovarianceStamped pose_in;
	pose_in.header.frame_id = transform.header.frame_id;
	pose_in.pose = pose_;

	geometry_msgs::PoseWithCovarianceStamped pose_out;
	pose_out.header.frame_id = transform.child_frame_id;
	tf2::doTransform(pose_in, pose_out, transform);

	// transform velocity with covariance
	geometry_msgs::PoseWithCovarianceStamped vel_in;
	vel_in.header.frame_id = transform.header.frame_id;
	vel_in.pose = vel_;

	geometry_msgs::PoseWithCovarianceStamped vel_out;
	vel_out.header.frame_id = transform.child_frame_id;
	tf2::Transform transform_tf;
	tf2::fromMsg(transform.transform, transform_tf);
	vel_out.pose.covariance = tf2::transformCovariance(vel_in.pose.covariance, transform_tf);

	// robust approach for velocity transformation (simple `tf2::doTransform` will not do)
	auto pose_in_vel_comp = pose_in;
	// displacement after applying velocity
	pose_in_vel_comp.pose.pose.position.x += vel_in.pose.pose.position.x;
	pose_in_vel_comp.pose.pose.position.y += vel_in.pose.pose.position.y;
	pose_in_vel_comp.pose.pose.position.z += vel_in.pose.pose.position.z;

	// transform the pose with applied velocity to the requested frame
	geometry_msgs::PoseWithCovarianceStamped pose_out_vel_comp;
	tf2::doTransform(pose_in_vel_comp, pose_out_vel_comp, transform);

	// compute velocity of the person expressed in the costmap frame
	vel_out.pose.pose.position.x = pose_out_vel_comp.pose.pose.position.x - pose_out.pose.pose.position.x;
	vel_out.pose.pose.position.y = pose_out_vel_comp.pose.pose.position.y - pose_out.pose.pose.position.y;
	vel_out.pose.pose.position.z = pose_out_vel_comp.pose.pose.position.z - pose_out.pose.pose.position.z;

	// TODO: angular velocities stay the same only for 2D rotations; for complex rotations consider:
	// https://answers.ros.org/question/192273/how-to-implement-velocity-transformation/
	vel_out.pose.pose.orientation = vel_in.pose.pose.orientation;

	// overwrite pose with cov. and vel with cov.
	pose_ = pose_out.pose;
	vel_ = vel_out.pose;
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
			// primary key for later association
			group_id_ = *tag_value_it;
		}
		tag_value_it++;
 	}
	return true;
}

} // namespace people_msgs_utils
