#pragma once

#include <people_msgs_utils/group.h>
#include <people_msgs_utils/person.h>

#include <string>
#include <vector>

namespace people_msgs_utils {

/**
 * @brief Evaluates each person from the given vector, parses string tags and returns a set of People and Groups
 *
 * @param people standard people_msgs vector
 */
std::pair<std::vector<Person>, std::vector<Group>> createFromPeople(const std::vector<people_msgs::Person>& people);

/**
 * @brief Helper function for parsing bool values
 */
bool parseStringBool(const std::string& str);

/**
 * @brief Parses string containing a set of T-type values
 *
 * @tparam T type of values
 */
template <typename T>
std::vector<T> parseString(const std::string& str, const std::string& delimiter) {
	std::vector<T> values;
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
		values.push_back(static_cast<T>(std::stod(token)));
		payload.erase(0, pos + delimiter.length());
	}

	bool token_with_whitespace_only = payload.find_first_not_of("\t\n ") == std::string::npos;
	if (payload.empty() || token_with_whitespace_only) {
		return values;
	}

	// token still stores some meaningful value
	values.push_back(static_cast<T>(std::stod(payload)));
}

} // namespace people_msgs_utils
