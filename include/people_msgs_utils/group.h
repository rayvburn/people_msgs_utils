#pragma once

#include "person.h"

namespace people_msgs_utils {

class Group {
public:
	Group(
		std::string id,
		unsigned long int age,
		std::vector<unsigned int> track_ids,
		std::vector<std::tuple<unsigned int, unsigned int, double>> relations,
		geometry_msgs::Point center_of_gravity
	);

	static std::vector<Group> fromPeople(const std::vector<Person>& people);

	inline std::string getName() const {
		return group_id_;
	}

	inline unsigned int getID() const {
		return std::stoul(group_id_);
	}

	inline unsigned long int getAge() const {
		return group_age_;
	}

	inline std::vector<unsigned int> getTrackIDs() const {
		return group_track_ids_;
	}

	inline geometry_msgs::Point getCenterOfGravity() const {
		return group_center_of_gravity_;
	}

	/// @brief Returns social relations within the group expressed as tuple
	/// Tuple contents: track ID, track ID, relation estimation accuracy
	inline std::vector<std::tuple<unsigned int, unsigned int, double>> getSocialRelations() const {
		return social_relations_;
	}

protected:
	std::string group_id_;
	unsigned long int group_age_;
	std::vector<unsigned int> group_track_ids_;
	/// Social relations within the group
	std::vector<std::tuple<unsigned int, unsigned int, double>> social_relations_;
	geometry_msgs::Point group_center_of_gravity_;
};

// Dummy group that does not contain any track IDs
static const Group EMPTY_GROUP(
	std::to_string(std::numeric_limits<unsigned int>::max()),
	0,
	std::vector<unsigned int>(),
	std::vector<std::tuple<unsigned int, unsigned int, double>>(),
	geometry_msgs::Point{}
);

}
