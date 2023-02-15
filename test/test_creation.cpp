#include <gtest/gtest.h>
#include <people_msgs_utils/group.h>
#include <people_msgs_utils/person.h>
#include <people_msgs_utils/utils.h>

using namespace people_msgs_utils;

std::vector<people_msgs::Person> createSet1();
std::vector<people_msgs::Person> createSet2();
std::vector<std::string> createTagnames();
std::vector<std::string> createTagnamesGroup();
std::string createCovArray(double xx, double xy, double yy, double zz, double rr, double pp, double yawyaw);

// Test cases
TEST(ExtractionTest, singlePersonAttributes) {
	// input
	std::vector<people_msgs::Person> people_std = createSet1();
	ASSERT_EQ(people_std.size(), 1);
	// output
	Person person(people_std.at(0));

	EXPECT_EQ(person.getName(), "145");

	EXPECT_NEAR(person.getPositionX(), 1.123, 1e-06);
	EXPECT_NEAR(person.getPositionY(), 2.321, 1e-06);
	EXPECT_NEAR(person.getPositionZ(), 0.321, 1e-06);
	EXPECT_NEAR(person.getOrientationYaw(), 0.5, 1e-06);

	EXPECT_NEAR(person.getCovariancePoseXX(), 0.987, 1e-06);
	EXPECT_NEAR(person.getCovariancePoseXY(), 0.986, 1e-06);
	EXPECT_NEAR(person.getCovariancePoseYX(), 0.986, 1e-06);
	EXPECT_NEAR(person.getCovariancePoseYY(), 0.985, 1e-06);
	EXPECT_NEAR(person.getCovariancePoseYawYaw(), 0.984, 1e-06);

	EXPECT_NEAR(person.getReliability(), 0.987, 1e-06);

	EXPECT_NEAR(person.getVelocityX(), 0.123, 1e-06);
	EXPECT_NEAR(person.getVelocityY(), 0.321, 1e-06);
	EXPECT_NEAR(person.getVelocityZ(), 0.123, 1e-06);
	EXPECT_NEAR(person.getVelocityTheta(), 0.0, 1e-06);

	EXPECT_NEAR(person.getCovarianceVelocityXX(), 0.983, 1e-06);
	EXPECT_NEAR(person.getCovarianceVelocityXY(), 0.982, 1e-06);
	EXPECT_NEAR(person.getCovarianceVelocityYX(), 0.982, 1e-06);
	EXPECT_NEAR(person.getCovarianceVelocityYY(), 0.981, 1e-06);
	EXPECT_NEAR(person.getCovarianceVelocityThTh(), 99999.0, 1e-06);

	EXPECT_EQ(person.isOccluded(), true);
	EXPECT_EQ(person.isMatched(), true);
	EXPECT_EQ(person.getDetectionID(), 369);
	EXPECT_EQ(person.getTrackAge(), 963);
	EXPECT_EQ(person.isAssignedToGroup(), false);
	EXPECT_EQ(person.getGroupName(), std::string(""));
}

TEST(ExtractionTest, peopleInGroups) {
	// input
	std::vector<people_msgs::Person> people_std = createSet2();
	// output vectors
	std::vector<Person> people;
	std::vector<Group> groups;
	std::tie(people, groups) = createFromPeople(people_std);
	ASSERT_EQ(people.size(), 7);

	// assuming order of creation
	EXPECT_EQ(people.at(5).getName(), "8");
	ASSERT_EQ(people.at(5).isAssignedToGroup(), true);
	ASSERT_EQ(people.at(5).getGroupName(), "5");

	EXPECT_NEAR(people.at(5).getOrientationYaw(), -0.5, 1e-06);

	EXPECT_NEAR(people.at(5).getCovariancePoseXX(), 0.13, 1e-06);
	EXPECT_NEAR(people.at(5).getCovariancePoseXY(), 0.01, 1e-06);
	EXPECT_NEAR(people.at(5).getCovariancePoseYX(), 0.01, 1e-06);
	EXPECT_NEAR(people.at(5).getCovariancePoseYY(), 0.12, 1e-06);
	EXPECT_NEAR(people.at(5).getCovariancePoseYawYaw(), 0.13, 1e-06);

	EXPECT_NEAR(people.at(5).getReliability(), 0.378, 1e-06);

	EXPECT_NEAR(people.at(5).getVelocityX(), 0.3, 1e-06);
	EXPECT_NEAR(people.at(5).getVelocityY(), 0.3, 1e-06);
	EXPECT_NEAR(people.at(5).getVelocityZ(), 0.0, 1e-06);
	EXPECT_NEAR(people.at(5).getVelocityTheta(), 0.0, 1e-06);

	EXPECT_NEAR(people.at(5).getCovarianceVelocityXX(), 0.983, 1e-06);
	EXPECT_NEAR(people.at(5).getCovarianceVelocityXY(), 0.982, 1e-06);
	EXPECT_NEAR(people.at(5).getCovarianceVelocityYX(), 0.982, 1e-06);
	EXPECT_NEAR(people.at(5).getCovarianceVelocityYY(), 0.981, 1e-06);
	EXPECT_NEAR(people.at(5).getCovarianceVelocityThTh(), 99999.0, 1e-06);

	EXPECT_EQ(people.at(5).isOccluded(), false);
	EXPECT_EQ(people.at(5).isMatched(), true);
	EXPECT_EQ(people.at(5).getDetectionID(), 931);
	EXPECT_EQ(people.at(5).getTrackAge(), 179);

	// groups
	ASSERT_EQ(groups.size(), 2);
	for (const auto group: groups) {
		if (group.getName() == "5") {
			ASSERT_EQ(group.getName(), "5");
			EXPECT_EQ(group.getAge(), 159);

			auto track_ids = group.getMemberIDs();
			ASSERT_EQ(track_ids.size(), 3);
			// only one such entry must be found
			auto it = std::find(track_ids.begin(), track_ids.end(), "0");
			ASSERT_NE(it, track_ids.end());
			it = std::find(it + 1, track_ids.end(), "0");
			ASSERT_EQ(it, track_ids.end());

			// only one such entry must be found
			it = std::find(track_ids.begin(), track_ids.end(), "1");
			ASSERT_NE(it, track_ids.end());
			it = std::find(it + 1, track_ids.end(), "1");
			ASSERT_EQ(it, track_ids.end());

			// only one such entry must be found
			it = std::find(track_ids.begin(), track_ids.end(), "8");
			ASSERT_NE(it, track_ids.end());
			it = std::find(it + 1, track_ids.end(), "8");
			ASSERT_EQ(it, track_ids.end());

			EXPECT_EQ(group.getCenterOfGravity().x, 9.0);
			EXPECT_EQ(group.getCenterOfGravity().y, 8.5);
			EXPECT_EQ(group.getCenterOfGravity().z, 7.0);

		} else if (group.getName() == "9") {
			EXPECT_EQ(group.getName(), "9");
			EXPECT_EQ(group.getAge(), 147);

			auto track_ids = group.getMemberIDs();
			ASSERT_EQ(track_ids.size(), 2);
			// only one such entry must be found
			auto it = std::find(track_ids.begin(), track_ids.end(), "4");
			ASSERT_NE(it, track_ids.end());
			it = std::find(it + 1, track_ids.end(), "4");
			ASSERT_EQ(it, track_ids.end());

			// only one such entry must be found
			it = std::find(track_ids.begin(), track_ids.end(), "5");
			ASSERT_NE(it, track_ids.end());
			it = std::find(it + 1, track_ids.end(), "5");
			ASSERT_EQ(it, track_ids.end());

			EXPECT_EQ(group.getCenterOfGravity().x, 1.0);
			EXPECT_EQ(group.getCenterOfGravity().y, 2.5);
			EXPECT_EQ(group.getCenterOfGravity().z, 3.0);

		} else {
			ASSERT_EQ(true, false);
		}
	}
}

TEST(ExtractionTest, relationsInGroups) {
	// input
	std::vector<people_msgs::Person> people_std = createSet2();
	// output vectors
	std::vector<Person> people;
	std::vector<Group> groups;
	std::tie(people, groups) = createFromPeople(people_std);

	ASSERT_EQ(people.size(), 7);
	ASSERT_EQ(groups.size(), 2);

	for (const auto& group: groups) {
		std::vector<std::tuple<std::string, std::string, double>> relations = group.getSocialRelations();
		if (group.getName() == "5") {
			ASSERT_EQ(relations.size(), 3);
			// order might not be deterministic
			for (const auto& relation: relations) {
				if (
					(std::get<0>(relation) == "0" && std::get<1>(relation) == "1")
					|| (std::get<0>(relation) == "1" && std::get<1>(relation) == "0")
				) {
					ASSERT_EQ(std::get<2>(relation), 0.459);
				} else if (
					(std::get<0>(relation) == "0" && std::get<1>(relation) == "8")
					|| (std::get<0>(relation) == "8" && std::get<1>(relation) == "0")
				) {
					ASSERT_EQ(std::get<2>(relation), 0.456);
				} else if (
					(std::get<0>(relation) == "1" && std::get<1>(relation) == "8")
					|| (std::get<0>(relation) == "8" && std::get<1>(relation) == "1")
				) {
					ASSERT_EQ(std::get<2>(relation), 0.789);
				} else {
					ASSERT_EQ(true, false);
				}
			}
		} else if (group.getName() == "9") {
			// 2 people -> 1 relation
			ASSERT_EQ(relations.size(), 1);
			// order might not be deterministic
			for (const auto& relation: relations) {
				if (
					(std::get<0>(relation) == "4" && std::get<1>(relation) == "5")
					|| (std::get<0>(relation) == "5" && std::get<1>(relation) == "4")
				) {
					ASSERT_EQ(std::get<2>(relation), 0.987);
				} else {
					ASSERT_EQ(true, false);
				}
			}
		} else {
			ASSERT_EQ(true, false);
		}
	}
}

TEST(ExtractionTest, relationsOfMember) {
	// input
	std::vector<people_msgs::Person> people_std = createSet2();
	// output vectors
	std::vector<Person> people;
	std::vector<Group> groups;
	std::tie(people, groups) = createFromPeople(people_std);

	ASSERT_EQ(people.size(), 7);
	ASSERT_EQ(groups.size(), 2);

	for (const auto& group: groups) {
		if (group.getName() == "5") {
			auto rels0 = group.getSocialRelations("0");
			for (const auto& rel0: rels0) {
				if (rel0.first == "1") {
					ASSERT_EQ(rel0.second, 0.459);
				} else if (rel0.first == "8") {
					ASSERT_EQ(rel0.second, 0.456);
				} else {
					ASSERT_EQ(true, false);
				}
			}
			auto rels1 = group.getSocialRelations("1");
			for (const auto& rel1: rels1) {
				if (rel1.first == "0") {
					ASSERT_EQ(rel1.second, 0.459);
				} else if (rel1.first == "8") {
					ASSERT_EQ(rel1.second, 0.789);
				} else {
					ASSERT_EQ(true, false);
				}
			}
			auto rels8 = group.getSocialRelations("8");
			for (const auto& rel8: rels8) {
				if (rel8.first == "0") {
					ASSERT_EQ(rel8.second, 0.456);
				} else if (rel8.first == "1") {
					ASSERT_EQ(rel8.second, 0.789);
				} else {
					ASSERT_EQ(true, false);
				}
			}
		} else if (group.getName() == "9") {
			auto rels4 = group.getSocialRelations("4");
			ASSERT_EQ(rels4.size(), 1);
			rels4.at(0).first == "5";
			rels4.at(0).second == 0.987;

			auto rels5 = group.getSocialRelations("5");
			ASSERT_EQ(rels5.size(), 1);
			rels5.at(0).first == "4";
			rels5.at(0).second == 0.987;
		} else {
			ASSERT_EQ(true, false);
		}
	}
}

TEST(ExtractionTest, groupHasMember) {
	// input
	std::vector<people_msgs::Person> people_std = createSet2();
	// output vectors
	std::vector<Person> people;
	std::vector<Group> groups;
	std::tie(people, groups) = createFromPeople(people_std);

	ASSERT_EQ(people.size(), 7);
	ASSERT_EQ(groups.size(), 2);

	for (const auto& group: groups) {
		if (group.getName() == "5") {
			ASSERT_TRUE(group.hasMember("0"));
			ASSERT_TRUE(group.hasMember("1"));
			ASSERT_TRUE(group.hasMember("8"));
		} else if (group.getName() == "9") {
			ASSERT_TRUE(group.hasMember("4"));
			ASSERT_TRUE(group.hasMember("5"));
		} else {
			ASSERT_EQ(true, false);
		}
	}
}

TEST(ExtractionTest, fillGroupsWithMembers) {
	std::vector<people_msgs::Person> people_std = createSet2();

	// groups created by hand (without members)
	std::vector<Person> people;
	std::tie(people, std::ignore) = createFromPeople(people_std);

	Group group5(
		std::string("5"),
		/* does not matter */ 0,
		/* no member instances! */ std::vector<Person>(),
		std::vector<std::string>{"0", "1", "8"},
		/* does not matter */ {{"0", "1", 0.459}, {"0", "8", 0.456}, {"1", "8", 0.789}},
		/* does not matter */ geometry_msgs::Point()
	);
	ASSERT_EQ(group5.getMemberIDs().size(), 3);
	ASSERT_TRUE(group5.getMembers().empty());

	Group group9(
		std::string("9"),
		0,
		std::vector<Person>(),
		std::vector<std::string>{"4", "5"},
		std::vector<std::tuple<std::string, std::string, double>>(),
		geometry_msgs::Point()
	);
	ASSERT_EQ(group9.getMemberIDs().size(), 2);
	ASSERT_TRUE(group9.getMembers().empty());

	// check vector with a single group
	std::vector<Group> groups{group5};
	groups = fillGroupsWithMembers(groups, people);
	ASSERT_EQ(groups.size(), 1);
	group5 = groups.at(0);
	ASSERT_EQ(group5.getMembers().size(), 3);
	ASSERT_EQ(group5.getMemberIDs().size(), 3);

	// check vector with multiple groups
	groups = std::vector<Group>{group5, group9};
	groups = fillGroupsWithMembers(groups, people);
	ASSERT_EQ(groups.size(), 2);
	for (const auto& group: groups) {
		if (group.getName() == "5") {
			ASSERT_EQ(group.getMemberIDs().size(), 3);
			ASSERT_EQ(group.getMembers().size(), 3);
			// make sure that required member IDs are found
			ASSERT_NE(
				std::find_if(
					group.getMembers().cbegin(),
					group.getMembers().cend(),
					[](const people_msgs_utils::Person& person) {
						return person.getName() == "0";
					}
				),
				group.getMembers().cend()
			);
			ASSERT_NE(
				std::find_if(
					group.getMembers().cbegin(),
					group.getMembers().cend(),
					[](const people_msgs_utils::Person& person) {
						return person.getName() == "1";
					}
				),
				group.getMembers().cend()
			);
			ASSERT_NE(
				std::find_if(
					group.getMembers().cbegin(),
					group.getMembers().cend(),
					[](const people_msgs_utils::Person& person) {
						return person.getName() == "8";
					}
				),
				group.getMembers().cend()
			);
		} else if (group.getName() == "9") {
			ASSERT_EQ(group.getMemberIDs().size(), 2);
			ASSERT_EQ(group.getMembers().size(), 2);
			// make sure that required member IDs are found
			ASSERT_NE(
				std::find_if(
					group.getMembers().cbegin(),
					group.getMembers().cend(),
					[](const people_msgs_utils::Person& person) {
						return person.getName() == "4";
					}
				),
				group.getMembers().cend()
			);
			ASSERT_NE(
				std::find_if(
					group.getMembers().cbegin(),
					group.getMembers().cend(),
					[](const people_msgs_utils::Person& person) {
						return person.getName() == "5";
					}
				),
				group.getMembers().cend()
			);
		} else {
			ASSERT_EQ(true, false);
		}
	}
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

// .........................................................................

std::vector<people_msgs::Person> createSet1() {
	std::vector<people_msgs::Person> people_set;

	people_msgs::Person person;
	person.name = "145";
	person.position.x = 1.123;
	person.position.y = 2.321;
	person.position.z = 0.321;
	person.velocity.x = 0.123;
	person.velocity.y = 0.321;
	person.velocity.z = 0.123;
	person.reliability = 0.987;
	person.tagnames = createTagnames();
	person.tags = {
		"0.0 0.0 0.247404 0.9689124",
		createCovArray(0.987, 0.986, 0.985, 99999.0, 99999.0, 99999.0, 0.984),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"true",
		"true",
		"369",
		"963",
		""
	};

	people_set.push_back(person);
	return people_set;
}

std::vector<people_msgs::Person> createSet2() {
	std::vector<people_msgs::Person> people_set;

	people_msgs::Person person;
	person.name = "0";
	person.position.x = 1.0;
	person.position.y = 2.0;
	person.position.z = 0.0;
	person.velocity.x = 0.3;
	person.velocity.y = 0.3;
	person.velocity.z = 0.0;
	person.reliability = 0.998;
	person.tagnames = createTagnamesGroup();
	person.tags = {
		"0.0 0.0 0.0 1.0",
		createCovArray(0.03, 0.01, 0.02, 99999.0, 99999.0, 99999.0, 0.03),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"1",
		"false",
		"123",
		"0987",
		"5",
		"159",
		"0 1 8",
		"9.0 8.5 7.0",
		"0 1 0.459 0 8 0.456 1 8 0.789"
	};
	people_set.push_back(person);

	person.name = "1";
	person.reliability = 0.978;
	person.tagnames = createTagnamesGroup();
	person.tags = {
		"0.0 0.0 -0.247404 0.9689124",
		createCovArray(0.03, 0.01, 0.02, 99999.0, 99999.0, 99999.0, 0.03),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"true",
		"false",
		"321",
		"456",
		"5",
		"159",
		"1 8 0",
		"9.0 8.5 7.0",
		"1 8 0.789 0 1 0.459 0 8 0.456"
	};
	people_set.push_back(person);

	person.name = "2";
	person.reliability = 0.478;
	person.tagnames = createTagnames();
	person.tags = {
		"0.0 0.0 0.0 1.0",
		createCovArray(0.13, 0.01, 0.12, 99999.0, 99999.0, 99999.0, 0.13),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"true",
		"false",
		"322",
		"466",
		""
	};
	people_set.push_back(person);

	person.name = "4";
	person.reliability = 0.778;
	person.tagnames = createTagnamesGroup();
	person.tags = {
		"0.0 0.0 0.247404 0.9689124",
		createCovArray(0.13, 0.01, 0.12, 99999.0, 99999.0, 99999.0, 0.13),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"false",
		"false",
		"951",
		"159",
		"9",
		"147",
		"4 5",
		"1.0 2.5 3.0",
		"4 5 0.987"
	};
	people_set.push_back(person);

	person.name = "5";
	person.reliability = 0.578;
	person.tagnames = createTagnamesGroup();
	person.tags = {
		"0.0 0.0 0.0 1.0",
		createCovArray(0.13, 0.01, 0.12, 99999.0, 99999.0, 99999.0, 0.13),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"true",
		"false",
		"941",
		"169",
		"9",
		"147",
		"5 4",
		"1.0 2.5 3.0",
		"4 5 0.987"
	};
	people_set.push_back(person);

	person.name = "8";
	person.reliability = 0.378;
	person.tagnames = createTagnamesGroup();
	person.tags = {
		"0.0 0.0 -0.247404 0.9689124",
		createCovArray(0.13, 0.01, 0.12, 99999.0, 99999.0, 99999.0, 0.13),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"false",
		"true",
		"931",
		"179",
		"5",
		"159",
		"8 1 0",
		"9.0 8.5 7.0",
		"8 1 0.789 0 1 0.459 0 8 0.456"
	};
	people_set.push_back(person);

	person.name = "9";
	person.reliability = 0.278;
	person.tagnames = createTagnames();
	person.tags = {
		"0.0 0.0 0.0 1.0",
		createCovArray(0.13, 0.01, 0.12, 99999.0, 99999.0, 99999.0, 0.13),
		createCovArray(0.983, 0.982, 0.981, 99999.0, 99999.0, 99999.0, 99999.0),
		"true",
		"true",
		"831",
		"279",
		""
	};
	people_set.push_back(person);

	return people_set;
}

std::vector<std::string> createTagnames() {
	std::vector<std::string> tagnames;
	tagnames.push_back("orientation");
	tagnames.push_back("pose_covariance");
	tagnames.push_back("twist_covariance");
	tagnames.push_back("occluded");
	tagnames.push_back("matched");
	tagnames.push_back("detection_id");
	tagnames.push_back("track_age");
	tagnames.push_back("group_id");
	return tagnames;
}

std::vector<std::string> createTagnamesGroup() {
	std::vector<std::string> tagnames = createTagnames();
	tagnames.push_back("group_age");
	tagnames.push_back("group_track_ids");
	tagnames.push_back("group_center_of_gravity");
	tagnames.push_back("social_relations");
	return tagnames;
}

std::string createCovArray(double xx, double xy, double yy, double zz, double rr, double pp, double yawyaw) {
	return std::string(
		/* 00 */         std::to_string(xx)
		/* 01 */ + " " + std::to_string(xy)
		/* 02 */ + " " + std::to_string(0.0)
		/* 03 */ + " " + std::to_string(0.0)
		/* 04 */ + " " + std::to_string(0.0)
		/* 05 */ + " " + std::to_string(0.0)
		//
		/* 06 */ + " " + std::to_string(xy)
		/* 07 */ + " " + std::to_string(yy)
		/* 08 */ + " " + std::to_string(0.0)
		/* 09 */ + " " + std::to_string(0.0)
		/* 10 */ + " " + std::to_string(0.0)
		/* 11 */ + " " + std::to_string(0.0)
		//
		/* 12 */ + " " + std::to_string(0.0)
		/* 13 */ + " " + std::to_string(0.0)
		/* 14 */ + " " + std::to_string(zz)
		/* 15 */ + " " + std::to_string(0.0)
		/* 16 */ + " " + std::to_string(0.0)
		/* 17 */ + " " + std::to_string(0.0)
		//
		/* 18 */ + " " + std::to_string(0.0)
		/* 19 */ + " " + std::to_string(0.0)
		/* 20 */ + " " + std::to_string(0.0)
		/* 21 */ + " " + std::to_string(rr)
		/* 22 */ + " " + std::to_string(0.0)
		/* 23 */ + " " + std::to_string(0.0)
		//
		/* 24 */ + " " + std::to_string(0.0)
		/* 25 */ + " " + std::to_string(0.0)
		/* 26 */ + " " + std::to_string(0.0)
		/* 27 */ + " " + std::to_string(0.0)
		/* 28 */ + " " + std::to_string(pp)
		/* 29 */ + " " + std::to_string(0.0)
		//
		/* 30 */ + " " + std::to_string(0.0)
		/* 31 */ + " " + std::to_string(0.0)
		/* 32 */ + " " + std::to_string(0.0)
		/* 33 */ + " " + std::to_string(0.0)
		/* 34 */ + " " + std::to_string(0.0)
		/* 35 */ + " " + std::to_string(yawyaw)
	);
}
