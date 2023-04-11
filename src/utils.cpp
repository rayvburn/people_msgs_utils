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
	// create groups if they have multiple members assigned
	std::vector<Group> groups_total;
	for (const auto& group: people_grouped) {
		if (group.second.people.size() < 2) {
			continue;
		}
		groups_total.emplace_back(
			group.first,
			group.second.people,
			group.second.tagnames,
			group.second.tags
		);
	}

	/*
	 * Stage 5
	 *
	 * It may happen that not all people are tracked with the selected data source but groups will still
	 * have relations with extra IDs
	 */
	std::vector<Group> groups_total_cleaned;
	for (const auto& group: groups_total) {
		// keep only members tracked according to the @ref people_total container
		auto member_ids = group.getMemberIDs();
		std::vector<std::string> member_ids_valid;
		for (const auto& member_id: member_ids) {
			auto it = std::find_if(
				people_total.cbegin(),
				people_total.cend(),
				[&](const Person& person) {
					return person.getName() == member_id;
				}
			);
			if (it == people_total.cend()) {
				continue;
			}
			member_ids_valid.push_back(member_id);
		}
		// erase relations with inexisting member IDs
		auto relations = group.getSocialRelations();
		std::vector<std::tuple<std::string, std::string, double>> relations_valid;
		for (const auto& rel: relations) {
			auto member1_it = std::find(member_ids_valid.cbegin(), member_ids_valid.cend(), std::get<0>(rel));
			auto member2_it = std::find(member_ids_valid.cbegin(), member_ids_valid.cend(), std::get<1>(rel));
			if (member1_it != member_ids_valid.cend() && member2_it != member_ids_valid.cend()) {
				relations_valid.emplace_back(*member1_it, *member2_it, std::get<2>(rel));
			}
		}
		// keep only valid members
		auto members = group.getMembers();
		People members_valid;
		for (const auto& member: members) {
			auto it = std::find(
				member_ids_valid.cbegin(),
				member_ids_valid.cend(),
				member.getName()
			);
			if (it == member_ids_valid.cend()) {
				continue;
			}
			members_valid.push_back(member);
		}
		// recompute center of gravity as with changed members it may be outdated
		geometry_msgs::Point cog_valid;
		for (const auto& member: members_valid) {
			cog_valid.x += member.getPositionX();
			cog_valid.y += member.getPositionY();
			cog_valid.z += member.getPositionZ();
		}
		cog_valid.x /= members_valid.size();
		cog_valid.y /= members_valid.size();
		cog_valid.z /= members_valid.size();
		// create an instance with 'valid', i.e., recomputed/cleaned params
		groups_total_cleaned.emplace_back(
			group.getName(),
			group.getAge(),
			members_valid,
			member_ids_valid,
			relations_valid,
			cog_valid
		);
	}

	return std::make_pair(people_total, groups_total_cleaned);
}

std::vector<Group> fillGroupsWithMembers(const std::vector<Group>& groups, const std::vector<Person>& people) {
	std::vector<Group> groups_filled;
	for (const auto& group: groups) {
		std::vector<Person> people_from_group;
		auto people_ids = group.getMemberIDs();
		for (const auto& person: people) {
			bool is_member = std::find(people_ids.begin(), people_ids.end(), person.getName()) != people_ids.end();
			if (!is_member) {
				continue;
			}
			people_from_group.push_back(person);
		}
		groups_filled.emplace_back(
			group.getName(),
			group.getAge(),
			people_from_group,
			people_ids,
			group.getSocialRelations(),
			group.getCenterOfGravity()
		);
	}
	return groups_filled;
}

bool parseStringBool(const std::string& str) {
	if (str == "True" || str == "true" || str == "1") {
		return true;
	}
	return false;
}

// Template full specialization
template<>
std::vector<std::string> parseString<std::string>(const std::string& str, const std::string& delimiter) {
	std::vector<std::string> values;
	std::string payload(str);

	if (str.empty() || delimiter.empty()) {
		return values;
	}

	// https://stackoverflow.com/a/14266139
	size_t pos = 0;
	std::string token;
	while ((pos = payload.find(delimiter)) != std::string::npos) {
		token = payload.substr(0, pos);

		// check if token stores some valid chars and not whitespaces
		if(token.find_first_not_of(' ') == std::string::npos) {
			payload.erase(0, pos + delimiter.length());
			continue;
		}

		// convert with the biggest possible precision, then convert to desired type
		values.push_back(token);
		payload.erase(0, pos + delimiter.length());
	}

	bool token_with_whitespace_only = payload.find_first_not_of("\t\n ") == std::string::npos;
	if (payload.empty() || token_with_whitespace_only) {
		return values;
	}

	// token still stores some meaningful value
	values.push_back(payload);
}

} // namespace people_msgs_utils
