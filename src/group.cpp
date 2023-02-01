#include <people_msgs_utils/group.h>
#include <people_msgs_utils/utils.h>

namespace people_msgs_utils {

Group::Group(
	std::string id,
	unsigned long int age,
	const std::vector<Person>& members,
	std::vector<unsigned int> member_ids,
	std::vector<std::tuple<unsigned int, unsigned int, double>> relations,
	geometry_msgs::Point center_of_gravity
):
	group_id_(id),
	age_(age),
	members_(members),
	member_ids_(member_ids),
	social_relations_(relations),
	center_of_gravity_(center_of_gravity)
{}

Group::Group(
	const std::string id,
	const std::vector<Person>& members,
	std::vector<std::string> tagnames,
	std::vector<std::string> tags
):
	group_id_(id),
	members_(members)
{
	parseTags(tagnames, tags);
}

bool Group::hasMember(unsigned int person_id) const {
	return std::find_if(
		members_.begin(),
		members_.end(),
		[person_id](const Person& member) {
			return member.getID() == person_id;
		}
	) != members_.end();
}

std::vector<std::pair<unsigned int, double>> Group::getSocialRelations(unsigned int person_id) const {
	std::vector<std::pair<unsigned int, double>> relations_req;
	for (const auto& relation: getSocialRelations()) {
		// track ID, track ID, relation estimation accuracy
		auto person_id1 = std::get<0>(relation);
		auto person_id2 = std::get<1>(relation);
		auto strength = std::get<2>(relation);
		// select the other one compared to the `person_id`
		if (person_id1 == person_id) {
			relations_req.emplace_back(person_id2, strength);
		} else if (person_id2 == person_id) {
			relations_req.emplace_back(person_id1, strength);
		}
	}
	return relations_req;
}

bool Group::parseTags(const std::vector<std::string>& tagnames, const std::vector<std::string>& tags) {
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
		if (tag_it->find("group_id") != std::string::npos) {
			// primary key for later association
			group_id_ = *tag_value_it;
		} else if (tag_it->find("group_age") != std::string::npos) {
			age_ = static_cast<unsigned int>(std::stoul(*tag_value_it));
		} else if (tag_it->find("group_track_ids") != std::string::npos) {
			member_ids_ = parseString<unsigned int>(*tag_value_it, DELIMITER);
		} else if (tag_it->find("group_center_of_gravity") != std::string::npos) {
			auto pos_v = parseString<double>(*tag_value_it, DELIMITER);
			if (pos_v.size() == 3) {
				center_of_gravity_.x = pos_v.at(0);
				center_of_gravity_.y = pos_v.at(1);
				center_of_gravity_.z = pos_v.at(2);
			}
		} else if (tag_it->find("social_relations") != std::string::npos) {
			auto relation_v = parseString<double>(*tag_value_it, DELIMITER);
			// relations are expressed as triplets: ID, ID, strength
			if (!relation_v.empty() && relation_v.size() % 3 == 0) {
				for (
					auto it = relation_v.cbegin();
					it != relation_v.cend();
					it = it + 3
				) {
					unsigned int track_id1 = *it;
					unsigned int track_id2 = *(it+1);
					double strength = *(it+2);
					social_relations_.push_back(std::make_tuple(track_id1, track_id2, strength));
				}
			}
		}
		tag_value_it++;
	}
}

} // namespace people_msgs_utils
