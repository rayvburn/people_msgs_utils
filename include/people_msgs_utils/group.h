#pragma once

#include <people_msgs/People.h>

#include <people_msgs_utils/person.h>

#include <limits>
#include <tuple>

namespace people_msgs_utils {

class Group {
public:
	Group(
		const std::string& id,
		unsigned long int age,
		const std::vector<Person>& members,
		const std::vector<unsigned int>& member_ids,
		const std::vector<std::tuple<unsigned int, unsigned int, double>>& relations,
		const geometry_msgs::Point& center_of_gravity
	);

	/// @brief Constructor used by an aggregator of raw people_msgs
	Group(
		const std::string id,
		const std::vector<Person>& members,
		std::vector<std::string> tagnames,
		std::vector<std::string> tags
	);

	inline std::string getName() const {
		return group_id_;
	}

	inline unsigned int getID() const {
		// TODO: catch exception / create a static counter
		return std::stoul(group_id_);
	}

	inline unsigned long int getAge() const {
		return age_;
	}

	/// @brief Returns a set of people (given in ctor) that belong to the group
	inline std::vector<Person> getMembers() const {
		return members_;
	}

	inline std::vector<unsigned int> getMemberIDs() const {
		return member_ids_;
	}

	/// @brief Evaluates whether person identified as person_id is a member of the group
	bool hasMember(unsigned int person_id) const;

	inline geometry_msgs::Point getCenterOfGravity() const {
		return center_of_gravity_;
	}

	/// @brief Returns social relations within the group expressed as tuple
	/// Tuple contents: track ID, track ID, relation estimation accuracy
	inline std::vector<std::tuple<unsigned int, unsigned int, double>> getSocialRelations() const {
		return social_relations_;
	}

	/// @brief Returns social relations of a specific member
	std::vector<std::pair<unsigned int, double>> getSocialRelations(unsigned int person_id) const;

protected:
	/**
	 * @brief Returns true if tagnames and tags are valid, no matter if expected data were found inside
	 *
	 * This method parses only group-specific tags
	 */
	bool parseTags(const std::vector<std::string>& tagnames, const std::vector<std::string>& tags);

	std::string group_id_;
	/// How long person's group has been tracked
	unsigned long int age_;
	/// @brief Copy of group members
	std::vector<Person> members_;
	/// IDs of other people classified to the same group
	std::vector<unsigned int> member_ids_;
	/// Social relations within the group as tuple: track ID, other track ID, relation estimation accuracy
	std::vector<std::tuple<unsigned int, unsigned int, double>> social_relations_;
	/// Position of the group's center of gravity
	geometry_msgs::Point center_of_gravity_;
};

//! Abbrev. for container storing multiple objects
typedef std::vector<Group> Groups;

// Dummy group that does not contain any track IDs
static const Group EMPTY_GROUP(
	std::to_string(std::numeric_limits<unsigned int>::max()),
	0,
	std::vector<Person>(),
	std::vector<unsigned int>(),
	std::vector<std::tuple<unsigned int, unsigned int, double>>(),
	geometry_msgs::Point{}
);

} // namespace people_msgs_utils
