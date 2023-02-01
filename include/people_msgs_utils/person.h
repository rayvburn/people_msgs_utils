#pragma once

#include <people_msgs/People.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2/utils.h>

#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace people_msgs_utils {

class Person {
public:
	static constexpr auto COV_MAT_SIZE = 36;
	static constexpr auto COV_XX_INDEX = 0;
	static constexpr auto COV_XY_INDEX = 1;
	static constexpr auto COV_YX_INDEX = 6;
	static constexpr auto COV_YY_INDEX = 7;
	static constexpr auto COV_ZZ_INDEX = 14;
	static constexpr auto COV_ROLLROLL_INDEX = 21;
	static constexpr auto COV_PITCHPITCH_INDEX = 28;
	static constexpr auto COV_YAWYAW_INDEX = COV_MAT_SIZE - 1;

	/**
	 * @brief Basic constructor from people_msgs/Person
	 */
	Person(const people_msgs::Person& person);

	/**
	 * @brief Basic constructor from people_msgs/Person contents
	 */
	Person(
		const std::string& name,
		const geometry_msgs::Point& position,
		const geometry_msgs::Point& velocity,
		const double& reliability,
		const std::vector<std::string>& tagnames,
		const std::vector<std::string>& tags
	);

	inline std::string getName() const {
		return name_;
	}

	inline unsigned int getID() const {
		return std::stoul(name_);
	}

	inline geometry_msgs::Pose getPose() const {
		return pose_.pose;
	}

	inline geometry_msgs::Point getPosition() const {
		return pose_.pose.position;
	}

	inline geometry_msgs::Quaternion getOrientation() const {
		return pose_.pose.orientation;
	}

	inline double getPositionX() const {
		return pose_.pose.position.x;
	}

	inline double getPositionY() const {
		return pose_.pose.position.y;
	}

	inline double getPositionZ() const {
		return pose_.pose.position.z;
	}

	inline double getOrientationYaw() const {
		return tf2::getYaw(pose_.pose.orientation);
	}

	/// @return 6x6 matrix with covariance values
	inline std::array<double, COV_MAT_SIZE> getCovariancePose() const {
		std::array<double, COV_MAT_SIZE> arr;
		std::copy(pose_.covariance.begin(), pose_.covariance.end(), arr.begin());
		return arr;
	}

	inline double getCovariancePoseXX() const {
		return pose_.covariance[COV_XX_INDEX];
	}

	inline double getCovariancePoseXY() const {
		return pose_.covariance[COV_XY_INDEX];
	}

	inline double getCovariancePoseYX() const {
		return pose_.covariance[COV_YX_INDEX];
	}

	inline double getCovariancePoseYY() const {
		return pose_.covariance[COV_YY_INDEX];
	}

	inline double getCovariancePoseYawYaw() const {
		return pose_.covariance[COV_YAWYAW_INDEX];
	}

	inline double getReliability() const {
		return reliability_;
	}

	inline geometry_msgs::Pose getVelocity() const {
		return vel_.pose;
	}

	inline double getVelocityX() const {
		return vel_.pose.position.x;
	}

	inline double getVelocityY() const {
		return vel_.pose.position.y;
	}

	inline double getVelocityZ() const {
		return vel_.pose.position.z;
	}

	inline double getVelocityTheta() const {
		return tf2::getYaw(vel_.pose.orientation);
	}

	/// @return 6x6 matrix with covariance values
	inline std::array<double, COV_MAT_SIZE> getCovarianceVelocity() const {
		std::array<double, COV_MAT_SIZE> arr;
		std::copy(vel_.covariance.begin(), vel_.covariance.end(), arr.begin());
		return arr;
	}

	inline double getCovarianceVelocityXX() const {
		return vel_.covariance[COV_XX_INDEX];
	}

	inline double getCovarianceVelocityXY() const {
		return vel_.covariance[COV_XY_INDEX];
	}

	inline double getCovarianceVelocityYX() const {
		return vel_.covariance[COV_YX_INDEX];
	}

	inline double getCovarianceVelocityYY() const {
		return vel_.covariance[COV_YY_INDEX];
	}

	inline double getCovarianceVelocityThTh() const {
		return vel_.covariance[COV_YAWYAW_INDEX];
	}

	inline bool isOccluded() const {
		return occluded_;
	}

	inline bool isMatched() const {
		return matched_;
	}

	inline unsigned int getDetectionID() const {
		return detection_id_;
	}

	inline unsigned long int getTrackAge() const {
		return track_age_;
	}

	inline bool isAssignedToGroup() const {
		return !group_id_.empty();
	}

	inline std::string getGroupName() const {
		return group_id_;
	}

	/**
	 * Retrieves ID of the group that person is assigned to
	 *
	 * Before call to this, check the @ref isAssignedToGroup method
	 */
	inline unsigned int getGroupID() const {
		// TODO: Catch exception / create a static counter
		return std::stol(group_id_);
	}

protected:
	/**
	 * @brief Returns true if tagnames and tags are valid, no matter if expected data were found inside
	 *
	 * This method parses only person-specific tags
	 */
	bool parseTags(const std::vector<std::string>& tagnames, const std::vector<std::string>& tags);

	/// Person ID (number) is treated as name
	std::string name_;
	/// Carries 2D pose (x, y, theta) with relevant covariances
	geometry_msgs::PoseWithCovariance pose_;
	/// Defines accuracy of person's pose and velocity
	double reliability_;
	/**
	 * Carries 2D velocity (x, y, theta) with relevant covariances
	 * NOTE: not twist as the velocity is expressed in a global coordinate system
	 */
	geometry_msgs::PoseWithCovariance vel_;

	bool occluded_;
	/// Whether person is currently matched by perception system
	bool matched_;

	/// Detection ID of the person
	unsigned int detection_id_;
	/// How long this person has been tracked
	unsigned long int track_age_;

	/// ID of the group that this person was classified to
	std::string group_id_;

}; // class Person

//! Abbrev. for container storing multiple objects
typedef std::vector<Person> People;

} // namespace people_msgs_utils
