#include <people_msgs_utils/group.h>

namespace people_msgs_utils {

Group::Group(
	std::string id,
	unsigned long int age,
	std::vector<unsigned int> track_ids,
	geometry_msgs::Point center_of_gravity
):	group_id_(id),
	group_age_(age),
	group_track_ids_(track_ids),
	group_center_of_gravity_(center_of_gravity)	{}

std::vector<Group> Group::fromPeople(const std::vector<Person>& people) {
	struct GroupTemp {
		std::string name;
		unsigned int age;
		std::vector<unsigned int> track_ids;
		geometry_msgs::Point center_of_gravity;
	};

	std::vector<GroupTemp> groups_temp;

	for (const auto& person: people) {
		if (!person.isAssignedToGroup()) {
			// nothing to check further if the person was not assigned to any group
			continue;
		}

		bool found_matching_group = false;
		for (auto& group: groups_temp) {
			if (person.getGroupName() == group.name) {
				// found group of the person
				group.track_ids.push_back(person.getID());
				// found matching group, let's check next **person**
				found_matching_group = true;
				break;
			}
		}

		if (found_matching_group) {
			continue;
		}

		// person was not matched to existing groups - let's create a new one
		GroupTemp temp;
		temp.name = person.getGroupName();
		temp.age = person.getGroupAge();
		temp.track_ids.push_back(person.getID());
		temp.center_of_gravity = person.getGroupCenterOfGravity();
		groups_temp.push_back(temp);
	}

	// create proper groups from temporary ones
	std::vector<Group> groups;
	for (const auto& group_temp: groups_temp) {
		groups.emplace_back(group_temp.name, group_temp.age, group_temp.track_ids, group_temp.center_of_gravity);
	}
	return groups;
}

} // namespace people_msgs_utils
