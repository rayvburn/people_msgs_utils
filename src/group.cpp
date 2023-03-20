#include <people_msgs_utils/group.h>
#include <people_msgs_utils/utils.h>

#include <social_nav_utils/ellipse_fitting.h>

namespace people_msgs_utils {

Group::Group(
	const std::string& id,
	unsigned long int age,
	const std::vector<Person>& members,
	const std::vector<std::string>& member_ids,
	const std::vector<std::tuple<std::string, std::string, double>>& relations,
	const geometry_msgs::Point& center_of_gravity
):
	group_id_(id),
	age_(age),
	members_(members),
	member_ids_(member_ids),
	social_relations_(relations),
	center_of_gravity_(center_of_gravity)
{
	computeSpatialModel();
}

Group::Group(
	const std::string& id,
	const std::vector<Person>& members,
	std::vector<std::string> tagnames,
	std::vector<std::string> tags
):
	group_id_(id),
	members_(members)
{
	parseTags(tagnames, tags);
	computeSpatialModel();
}

void Group::transform(const geometry_msgs::TransformStamped& transform) {
	// transform members and recalculate spatial model
	for (auto& member: members_) {
		member.transform(transform);
	}

	// transform center of gravity
	geometry_msgs::PointStamped cog_in;
	cog_in.header.frame_id = transform.header.frame_id;
	cog_in.point = center_of_gravity_;

	geometry_msgs::PointStamped cog_out;
	cog_out.header.frame_id = transform.child_frame_id;
	tf2::doTransform(cog_in, cog_out, transform);

	// overwrite center of gravity (not calculated in @ref computeSpatialModel)
	center_of_gravity_ = cog_out.point;

	// recompute
	computeSpatialModel();
}

bool Group::hasMember(const std::string& person_id) const {
	return std::find_if(
		members_.begin(),
		members_.end(),
		[person_id](const Person& member) {
			return member.getName() == person_id;
		}
	) != members_.end();
}

std::vector<std::pair<std::string, double>> Group::getSocialRelations(const std::string& person_id) const {
	std::vector<std::pair<std::string, double>> relations_req;
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
			member_ids_ = parseString<std::string>(*tag_value_it, DELIMITER);
		} else if (tag_it->find("group_center_of_gravity") != std::string::npos) {
			auto pos_v = parseString<double>(*tag_value_it, DELIMITER);
			if (pos_v.size() == 3) {
				center_of_gravity_.x = pos_v.at(0);
				center_of_gravity_.y = pos_v.at(1);
				center_of_gravity_.z = pos_v.at(2);
			}
		} else if (tag_it->find("social_relations") != std::string::npos) {
			auto relation_v = parseString<std::string>(*tag_value_it, DELIMITER);
			// relations are expressed as triplets: ID, ID, strength
			if (!relation_v.empty() && relation_v.size() % 3 == 0) {
				for (
					auto it = relation_v.cbegin();
					it != relation_v.cend();
					it = it + 3
				) {
					std::string track_id1 = *it;
					std::string track_id2 = *(it+1);
					double strength = std::stod(*(it+2));
					social_relations_.push_back(std::make_tuple(track_id1, track_id2, strength));
				}
			}
		}
		tag_value_it++;
	}
	return true;
}

void Group::computeSpatialModel() {
	if (getMembers().empty()) {
		// spatial model cannot be defined for a group without members
		pose_.pose.position.x = center_of_gravity_.x;
		pose_.pose.position.y = center_of_gravity_.y;
		pose_.pose.orientation.w = 1.0;
		pose_.covariance.assign(COVARIANCE_UNKNOWN);
		span_.x = 0.0;
		span_.y = 0.0;
		return;
	}

	// Approximate O-space with an ellipse
	// - start with collecting points (member positions)
	std::vector<double> ospace_x;
	std::vector<double> ospace_y;
	for (const auto& person: getMembers()) {
	  ospace_x.push_back(person.getPositionX());
	  ospace_y.push_back(person.getPositionY());
	}
	social_nav_utils::EllipseFitting ellipse(ospace_x, ospace_y);

	// store
	pose_.pose.position.x = ellipse.getCenterX();
	pose_.pose.position.y = ellipse.getCenterY();
	pose_.pose.position.z = 0.0;
	tf2::Quaternion quat;
	quat.setRPY(0.0, 0.0, ellipse.getOrientation());
	pose_.pose.orientation.x = quat.getX();
	pose_.pose.orientation.y = quat.getY();
	pose_.pose.orientation.z = quat.getZ();
	pose_.pose.orientation.w = quat.getW();

	/*
	 * Compute `cost` of the robot being located in the current position - how it affects group's ease.
	 * Prepare Gaussian representation of the O-space's shape.
	 */
	// find uncertainty of the O-space mean position estimation based on member variances
	std::vector<double> variances_p_xx;
	std::vector<double> variances_p_xy;
	std::vector<double> variances_p_yx;
	std::vector<double> variances_p_yy;
	for (const auto& person: getMembers()) {
	  variances_p_xx.push_back(person.getCovariancePoseXX());
	  variances_p_xy.push_back(person.getCovariancePoseXY());
	  variances_p_yx.push_back(person.getCovariancePoseYX());
	  variances_p_yy.push_back(person.getCovariancePoseYY());
	}
	double variance_p_xx = *std::max_element(variances_p_xx.begin(), variances_p_xx.end());
	double variance_p_xy = *std::max_element(variances_p_xy.begin(), variances_p_xy.end());
	double variance_p_yx = *std::max_element(variances_p_yx.begin(), variances_p_yx.end());
	double variance_p_yy = *std::max_element(variances_p_yy.begin(), variances_p_yy.end());
	double variance_p_xyyx = std::max(variance_p_xy, variance_p_yx);

	// store
	pose_.covariance.assign(0.0);
	pose_.covariance.at(Person::COV_XX_INDEX) = variance_p_xx;
	pose_.covariance.at(Person::COV_XY_INDEX) = variance_p_xyyx;
	pose_.covariance.at(Person::COV_YX_INDEX) = variance_p_xyyx;
	pose_.covariance.at(Person::COV_YY_INDEX) = variance_p_yy;
	pose_.covariance.at(Person::COV_ZZ_INDEX) = COVARIANCE_UNKNOWN;
	pose_.covariance.at(Person::COV_ROLLROLL_INDEX) = COVARIANCE_UNKNOWN;
	pose_.covariance.at(Person::COV_PITCHPITCH_INDEX) = COVARIANCE_UNKNOWN;
	pose_.covariance.at(Person::COV_YAWYAW_INDEX) = 1.0 / COVARIANCE_UNKNOWN;

	// store the size of the ellipse
	span_.x = 2.0 * ellipse.getSemiAxisMajor();
	span_.y = 2.0 * ellipse.getSemiAxisMinor();
}

} // namespace people_msgs_utils
