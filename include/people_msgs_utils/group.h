#pragma once

#include <people_msgs/People.h>
#include <tf2/utils.h>

#include <people_msgs_utils/person.h>

#include <limits>
#include <tuple>

namespace people_msgs_utils {

/**
 * @brief Stores attributes of the group of people
 */
class Group {
public:
	/// Value assigned to variances that are not measured
	static constexpr auto COVARIANCE_UNKNOWN = 9999999.9;

	Group(
		const std::string& id,
		unsigned long int age,
		const std::vector<Person>& members,
		const std::vector<std::string>& member_ids,
		const std::vector<std::tuple<std::string, std::string, double>>& relations,
		const geometry_msgs::Point& center_of_gravity
	);

	/// @brief Constructor used by an aggregator of raw people_msgs
	Group(
		const std::string& id,
		const std::vector<Person>& members,
		std::vector<std::string> tagnames,
		std::vector<std::string> tags
	);

	/**
	 * @brief Transforms members and center of gravity and recalculates spatial model according to given @ref transform
	 *
	 * Assumes that the stored pose is expressed in the parent frame of the @ref transform,
	 * whereas child frame of the transform is the frame to transform into
	 *
	 * Uses calculations from tf2 library
	 */
	void transform(const geometry_msgs::TransformStamped& transform);

	/**
	 * Returns identifier of the group
	 */
	inline std::string getName() const {
		return group_id_;
	}

	inline unsigned long int getAge() const {
		return age_;
	}

	/**
	 * Returns a set of people (given in ctor) that belong to the group
	 *
	 * @details User should always check if getMembers() returns non-empty vector.
	 * The instance can be ill-formed if members are not given to constructor.
	 */
	inline std::vector<Person> getMembers() const {
		return members_;
	}

	inline std::vector<std::string> getMemberIDs() const {
		return member_ids_;
	}

	/// @brief Evaluates whether person identified as person_id is a member of the group
	bool hasMember(const std::string& person_id) const;

	/// @brief Returns social relations within the group expressed as tuple
	/// Tuple contents: track ID, track ID, relation estimation accuracy
	inline std::vector<std::tuple<std::string, std::string, double>> getSocialRelations() const {
		return social_relations_;
	}

	/// @brief Returns social relations of a specific member
	std::vector<std::pair<std::string, double>> getSocialRelations(const std::string& person_id) const;

	/**
	 * @defgroup spatialmodel Methods related to spatial (elliptical) model of the F-formation and its O-space
	 *
	 * @{
	 */
	inline geometry_msgs::Point getCenterOfGravity() const {
		return center_of_gravity_;
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
	inline std::array<double, Person::COV_MAT_SIZE> getCovariancePose() const {
		std::array<double, Person::COV_MAT_SIZE> arr;
		std::copy(pose_.covariance.begin(), pose_.covariance.end(), arr.begin());
		return arr;
	}

	inline double getCovariancePoseXX() const {
		return pose_.covariance[Person::COV_XX_INDEX];
	}

	inline double getCovariancePoseXY() const {
		return pose_.covariance[Person::COV_XY_INDEX];
	}

	inline double getCovariancePoseYX() const {
		return pose_.covariance[Person::COV_YX_INDEX];
	}

	inline double getCovariancePoseYY() const {
		return pose_.covariance[Person::COV_YY_INDEX];
	}

	/// Not supported, returns nearly zero variance
	inline double getCovariancePoseYawYaw() const {
		return pose_.covariance[Person::COV_YAWYAW_INDEX];
	}

	/// Returns pose estimation reliability of the group arising from the members reliabilities
	double getReliability() const;

	/// Returns length of the spatial model expressed in the local coordinate system (major axis of the ellipse)
	inline double getSpanX() const {
		return span_.x;
	}

	/// Returns length of the spatial model expressed in the local coordinate system (minor axis of the ellipse)
	inline double getSpanY() const {
		return span_.y;
	}

	/// @}

protected:
	/**
	 * @brief Returns true if tagnames and tags are valid, no matter if expected data were found inside
	 *
	 * This method parses only group-specific tags
	 */
	bool parseTags(const std::vector<std::string>& tagnames, const std::vector<std::string>& tags);

	/**
	 * @brief Computes parameters of a spatial model of the group represented by an ellipse with covariance
	 */
	void computeSpatialModel();

	std::string group_id_;
	/// How long person's group has been tracked
	unsigned long int age_;
	/// @brief Copy of group members
	std::vector<Person> members_;
	/// IDs of other people classified to the same group
	std::vector<std::string> member_ids_;
	/// Social relations within the group as tuple: track ID, other track ID, relation estimation accuracy
	std::vector<std::tuple<std::string, std::string, double>> social_relations_;
	/// Position of the group's center of gravity
	geometry_msgs::Point center_of_gravity_;
	/// Pose and covariance of the spatial model
	geometry_msgs::PoseWithCovariance pose_;
	/// Dimensions of the spatial model expressed in a local coordinate system
	geometry_msgs::Point span_;
};

//! Abbrev. for container storing multiple objects
typedef std::vector<Group> Groups;

// Dummy group that does not contain any track IDs
static const Group EMPTY_GROUP(
	std::to_string(std::numeric_limits<unsigned int>::max()),
	0,
	std::vector<Person>(),
	std::vector<std::string>(),
	std::vector<std::tuple<std::string, std::string, double>>(),
	geometry_msgs::Point{}
);

} // namespace people_msgs_utils
