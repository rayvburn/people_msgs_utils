#include <people_msgs_utils/utils.h>

#include <map>
#include <tuple>

namespace people_msgs_utils {

std::pair<std::vector<Person>, std::vector<Group>> createFromPeople(const std::vector<people_msgs::Person>& people) {
	if (people.empty()) {
		return std::make_pair(std::vector<Person>(), std::vector<Group>());
	}

	/*
	 * Stage 1
	 */
	// convert and parse people data
	std::vector<Person> people_total;
	for (const auto& person_std: people) {
		people_total.emplace_back(Person(person_std));
	}

	/*
	 * Stage 2
	 */
	// temporary group that helps in further association
	struct GroupPrimitive {
		std::string id;
		std::vector<std::string> track_names;
	};
	// collect group IDs with member IDs
	std::vector<GroupPrimitive> groups_primitive;
	for (const auto& person: people_total) {
		if (!person.isAssignedToGroup()) {
			continue;
		}

		bool found_matching_group = false;
		for (auto& group: groups_primitive) {
			if (person.getGroupName() == group.id) {
				// found group of the person
				group.track_names.push_back(person.getName());
				// found matching group, let's check next **person**
				found_matching_group = true;
				break;
			}
		}

		if (found_matching_group) {
			continue;
		}
		// person was not matched to existing groups - let's create a new one
		GroupPrimitive temp;
		temp.id = person.getGroupName();
		temp.track_names.push_back(person.getName());
		groups_primitive.push_back(temp);
	}

	/*
	 * Stage 3
	 */
	// collect constructor data for groups
	struct GroupTemp {
		// TODO: consider reference (most likely not feasible) or shared_ptr
		std::vector<Person> people;
		std::vector<std::string> tagnames;
		std::vector<std::string> tags;
	};
	std::map<std::string, GroupTemp> people_grouped;
	for (const auto& groupp: groups_primitive) {
		for (const auto& person_std: people) {
			auto it = std::find(groupp.track_names.begin(), groupp.track_names.end(), person_std.name);
			if (it == groupp.track_names.end()) {
				// given person was not found among group's tracked names
				continue;
			}
			// find a 'utils' version of person
			auto person_util_it = std::find_if(
				people_total.cbegin(),
				people_total.cend(),
				[&](const Person& person) {
					return person.getName() == person_std.name;
				}
			);

			// person found, make sure that wasn't added already
			// firstly, the key
			if (people_grouped.find(groupp.id) != people_grouped.cend()) {
				// secondly, person by ID
				auto it_group = std::find_if(
					people_grouped[groupp.id].people.cbegin(),
					people_grouped[groupp.id].people.cend(),
					[&](const Person& person) {
						return person.getName() == person_std.name;
					}
				);
				if (it_group != people_grouped[groupp.id].people.cend()) {
					// already there
					continue;
				}
				// let's not update group data, just add the person
				people_grouped[groupp.id].people.push_back(*person_util_it);
				continue;
			}

			// all good, add
			people_grouped[groupp.id].people.push_back(*person_util_it);
			people_grouped[groupp.id].tagnames = person_std.tagnames;
			people_grouped[groupp.id].tags = person_std.tags;
		}
	}

	/*
	 * Stage 4
	 */
	// create groups
	std::vector<Group> groups_total;
	for (const auto& group: people_grouped) {
		groups_total.emplace_back(
			group.first,
			group.second.people,
			group.second.tagnames,
			group.second.tags
		);
	}

	return std::make_pair(people_total, groups_total);
}

bool parseStringBool(const std::string& str) {
	if (str == "True" || str == "true" || str == "1") {
		return true;
	}
	return false;
}

} // namespace people_msgs_utils
