#include <gtest/gtest.h>
#include <people_msgs_utils/group.h>

using namespace people_msgs_utils;

// Test cases

/// Check position variances and span of the group
TEST(GroupTest, spatialModel) {
	// results based on Matlab implementation
	geometry_msgs::PoseWithCovariance pos1;
	pos1.pose.position.x = 1.0;
	pos1.pose.position.y = 3.0;
	pos1.pose.orientation.x = 0.0;
	pos1.pose.orientation.y = 0.0;
	pos1.pose.orientation.z = 0.954241556851;
	pos1.pose.orientation.w = 0.299036872608;
	pos1.covariance.at(Person::COV_XX_INDEX) = 0.0276496441588969;
	pos1.covariance.at(Person::COV_XY_INDEX) = -9.011976598363581e-08;
	pos1.covariance.at(Person::COV_YX_INDEX) = -9.011976598363581e-08;
	pos1.covariance.at(Person::COV_YY_INDEX) = 0.027649597818207795;
	pos1.covariance.at(Person::COV_ZZ_INDEX) = 9999999.0;
	pos1.covariance.at(Person::COV_ROLLROLL_INDEX) = 9999999.0;
	pos1.covariance.at(Person::COV_PITCHPITCH_INDEX) = 9999999.0;
	pos1.covariance.at(Person::COV_YAWYAW_INDEX) = 0.030461741978670857;

	geometry_msgs::PoseWithCovariance pos2;
	pos2.pose.position.x = 2.0;
	pos2.pose.position.y = 4.5;
	pos2.pose.orientation.x = 0.0;
	pos2.pose.orientation.y = 0.0;
	pos2.pose.orientation.z = 0.954241556851;
	pos2.pose.orientation.w = 0.299036872608;
	pos2.covariance.at(Person::COV_XX_INDEX) = 0.0876496441588969;
	pos2.covariance.at(Person::COV_XY_INDEX) = -9.011976598363581e-08;
	pos2.covariance.at(Person::COV_YX_INDEX) = -9.011976598363581e-08;
	pos2.covariance.at(Person::COV_YY_INDEX) = 0.127649597818207795;
	pos2.covariance.at(Person::COV_ZZ_INDEX) = 9999999.0;
	pos2.covariance.at(Person::COV_ROLLROLL_INDEX) = 9999999.0;
	pos2.covariance.at(Person::COV_PITCHPITCH_INDEX) = 9999999.0;
	pos2.covariance.at(Person::COV_YAWYAW_INDEX) = 0.030461741978670857;

	geometry_msgs::PoseWithCovariance pos3;
	pos3.pose.position.x = 3.0;
	pos3.pose.position.y = 3.0;
	pos3.pose.orientation.x = 0.0;
	pos3.pose.orientation.y = 0.0;
	pos3.pose.orientation.z = 0.954241556851;
	pos3.pose.orientation.w = 0.299036872608;
	pos3.covariance.at(Person::COV_XX_INDEX) = 0.4276496441588969;
	pos3.covariance.at(Person::COV_XY_INDEX) = -9.011976598363581e-08;
	pos3.covariance.at(Person::COV_YX_INDEX) = -9.011976598363581e-08;
	pos3.covariance.at(Person::COV_YY_INDEX) = 0.487649597818207795;
	pos3.covariance.at(Person::COV_ZZ_INDEX) = 9999999.0;
	pos3.covariance.at(Person::COV_ROLLROLL_INDEX) = 9999999.0;
	pos3.covariance.at(Person::COV_PITCHPITCH_INDEX) = 9999999.0;
	pos3.covariance.at(Person::COV_YAWYAW_INDEX) = 0.030461741978670857;

	geometry_msgs::PoseWithCovariance pos4;
	pos4.pose.position.x = 2.0;
	pos4.pose.position.y = 1.0;
	pos4.pose.orientation.x = 0.0;
	pos4.pose.orientation.y = 0.0;
	pos4.pose.orientation.z = 0.954241556851;
	pos4.pose.orientation.w = 0.299036872608;
	pos4.covariance.at(Person::COV_XX_INDEX) = 0.4276496441588969;
	pos4.covariance.at(Person::COV_XY_INDEX) = -9.011976598363581e-08;
	pos4.covariance.at(Person::COV_YX_INDEX) = -9.011976598363581e-08;
	pos4.covariance.at(Person::COV_YY_INDEX) = 0.487649597818207795;
	pos4.covariance.at(Person::COV_ZZ_INDEX) = 9999999.0;
	pos4.covariance.at(Person::COV_ROLLROLL_INDEX) = 9999999.0;
	pos4.covariance.at(Person::COV_PITCHPITCH_INDEX) = 9999999.0;
	pos4.covariance.at(Person::COV_YAWYAW_INDEX) = 0.030461741978670857;

	geometry_msgs::PoseWithCovariance vel;

	auto g = Group(
		"123",
		321,
		// members
		std::vector<Person>{
			{Person("01", pos1, vel, 0.987, false, true, 951, 852, "123")},
			{Person("02", pos2, vel, 0.986, false, true, 952, 853, "123")},
			{Person("03", pos3, vel, 0.985, false, true, 953, 854, "123")},
			{Person("04", pos4, vel, 0.984, false, true, 954, 855, "123")}
		},
		// member IDs
		std::vector<std::string>{"01", "02", "03", "04"},
		// relations
		std::vector<std::tuple<std::string, std::string, double>>{
			{"01", "02", 0.987},
			{"01", "03", 0.987},
			{"01", "04", 0.987},
			{"02", "04", 0.789},
			{"02", "03", 0.147}
		},
		geometry_msgs::Point()
	);

	ASSERT_DOUBLE_EQ(g.getCovariancePoseXX(), 0.427649644158897);
	ASSERT_NEAR(g.getCovariancePoseXY(), 0.0, 1e-06);
	ASSERT_NEAR(g.getCovariancePoseXY(), 0.0, 1e-06);
	ASSERT_DOUBLE_EQ(g.getCovariancePoseYY(), 0.487649597818208);

	ASSERT_NEAR(g.getSpanX(), 2.0 * 1.75000000000000, 1e-06);
	ASSERT_NEAR(g.getSpanY(), 2.0 * 1.01036297108185, 1e-06);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
