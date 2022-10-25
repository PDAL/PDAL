#include <iostream>

#include <nlohmann/json-schema.hpp>

/** @return number of failed tests */
size_t
testStringFormat(const std::string &format,
                 const std::vector<std::pair<std::string, bool>> &stringValidTests)
{
	size_t numberOfErrors = 0;

	for (auto stringValid = stringValidTests.begin(); stringValid != stringValidTests.end(); ++stringValid) {
		std::cout << "[INFO] Testing " << format << ": " << stringValid->first << "\n";

		try {
			nlohmann::json_schema::default_string_format_check(format, stringValid->first);

			if (!stringValid->second) {
				++numberOfErrors;
				std::cerr << "[ERROR] String with " << format << " format '" << stringValid->first
				          << "' validated even though it should NOT!\n";
			}
		} catch (std::exception &exception) {
			std::cout << "[INFO] Validation failed with: " << exception.what() << "\n";
			if (stringValid->second) {
				++numberOfErrors;
				std::cerr << "[ERROR] String with " << format << " format '" << stringValid->first
				          << "' did NOT validate even though it should!\n";
			}
		}
	}

	return numberOfErrors;
}

int main()
{
	size_t numberOfErrors = 0;

	const std::vector<std::pair<std::string, bool>> dateTimeChecks{
	    {"1985-04-12T23:20:50.52Z", true},
	    {"1996-12-19T16:39:57-08:00", true},
	    {"1990-12-31T23:59:60Z", true},
	    {"1990-12-31T15:59:60-08:00", true},
	    {"1937-01-01T12:00:27.87+00:20", true},
	    {"1985-4-12T23:20:50.52Z", false},
	    {"1985-04-12T23:20:50.52", false},
	    {"1985-04-12T24:00:00", false},
	    {"", false},
	    {"2019-04-30T11:11:11+01:00", true},
	    {"2019-04-31T11:11:11+01:00", false},
	    {"2019-02-28T11:11:11+01:00", true},
	    {"2019-02-29T11:11:11+01:00", false},
	    {"2020-02-29T11:11:11+01:00", true},
	    {"2020-02-30T11:11:11+01:00", false},
	    {"2020-02-29T23:59:59+01:00", true},
	    {"2020-02-29T23:59:60+01:00", false},
	    {"2020-02-29T23:59:60+00:00", true},
	    {"2020-02-29T23:60:59+01:00", false},
	    {"2019-09-30T11:11:11+01:00", true},
	    {"2019-09-31T11:11:11+01:00", false},
	    {"2019-09-30T11:11:11+23:59", true},
	    {"2019-09-30T11:11:11+24:00", false}};

	numberOfErrors += testStringFormat("date-time", dateTimeChecks);

	const std::vector<std::pair<std::string, bool>> ipv4Checks{
	    {"", false},
	    {"x99.99.99.99", false},
	    {"99.99.99.99x", false},
	    {"192.168.0.1", true},
	    {"127.0.0", false},
	    {"127.0.0.1", true},
	    {"127.0.0.0.1", false},
	    {"255.255.255.255", true},
	    {"255.255.255.256", false},
	    {"255.255.256.255", false},
	    {"255.256.255.255", false},
	    {"256.255.255.255", false},
	    {"256.256.256.256", false},
	    {"0x7f000001", false}};

	numberOfErrors += testStringFormat("ipv4", ipv4Checks);

	return numberOfErrors;
}
