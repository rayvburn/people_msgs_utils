#include <people_msgs_utils/group.h>

namespace people_msgs_utils {

Group::Group(
	std::string id,
	unsigned long int age,
	std::vector<unsigned int> track_ids,
	std::vector<std::tuple<unsigned int, unsigned int, double>> relations,
	geometry_msgs::Point center_of_gravity
):	group_id_(id),
	group_age_(age),
	group_track_ids_(track_ids),
	social_relations_(relations),
	group_center_of_gravity_(center_of_gravity)
{}

std::vector<Group> Group::fromPeople(const std::vector<Person>& people) {
	struct GroupTemp {
		std::string name;
		unsigned int age;
		std::vector<unsigned int> track_ids;
		std::vector<std::tuple<unsigned int, unsigned int, double>> relations;
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

	// fill up the group_temp's relations and strengths
	for (auto& group_temp: groups_temp) {
		// check if group will potentially have any social relations
		if (group_temp.track_ids.empty()) {
			break;
		}
		for (const auto& person: people) {
			auto it = std::find(group_temp.track_ids.cbegin(), group_temp.track_ids.cend(), person.getID());
			if (it == group_temp.track_ids.end()) {
				continue;
			}
			auto person_rels = person.getSocialRelations();
			for (const auto& person_rel: person_rels) {
				auto pid_self = person.getID();
				auto pid_othr = std::get<0>(person_rel);
				auto pstrength = std::get<1>(person_rel);

				// check if group already contains this relation
				bool already_there = false;
				for (const auto& group_rel: group_temp.relations) {
					auto gid1 = std::get<0>(group_rel);
					auto gid2 = std::get<1>(group_rel);
					// check for the inverse order
					if ((pid_othr == gid1 && pid_self == gid2) || (pid_othr == gid2 && pid_self == gid1)) {
						already_there = true;
						break;
					}
				}
				if (!already_there) {
					group_temp.relations.push_back(std::make_tuple(pid_self, pid_othr, pstrength));
				}
			}
		}
	}

	// create proper groups from temporary ones
	std::vector<Group> groups;
	for (const auto& group_temp: groups_temp) {
		groups.emplace_back(
			group_temp.name,
			group_temp.age,
			group_temp.track_ids,
			group_temp.relations,
			group_temp.center_of_gravity
		);
	}
	return groups;
}

} // namespace people_msgs_utils
