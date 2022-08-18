#pragma once

#include "person.h"

namespace people_msgs_utils {

class Group {
public:
	Group(
		std::string id,
		unsigned long int age,
		std::vector<unsigned int> track_ids,
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

protected:
	std::string group_id_;
	unsigned long int group_age_;
	std::vector<unsigned int> group_track_ids_;
	geometry_msgs::Point group_center_of_gravity_;
};

// Dummy group that does not contain any track IDs
static const Group DUMMY_GROUP(
	std::to_string(std::numeric_limits<unsigned int>::max()),
	0,
	std::vector<unsigned int>(),
	geometry_msgs::Point{}
);

}
