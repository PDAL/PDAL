#include "../src/json-patch.hpp"

#include <iostream>

using nlohmann::json_patch;

#define OK(code)                                                    \
	do {                                                            \
		try {                                                       \
			code;                                                   \
		} catch (const std::exception &e) {                         \
			std::cerr << "UNEXPECTED FAILED: " << e.what() << "\n"; \
			return 1;                                               \
		}                                                           \
	} while (0)

#define KO(code)                                                \
	do {                                                        \
		try {                                                   \
			code;                                               \
			std::cerr << "UNEXPECTED SUCCESS.\n";               \
			return 1;                                           \
		} catch (const std::exception &e) {                         \
			std::cerr << "EXPECTED FAIL: " << e.what() << "\n"; \
		}                                                       \
	} while (0)

int main(void)
{
	OK( json_patch p1( R"([{"op":"add","path":"/0/renderable/bg","value":"Black"}])"_json));
	OK( json_patch p1( R"([{"op":"replace","path":"/0/renderable/bg","value":"Black"}])"_json));
	OK( json_patch p1( R"([{"op":"remove","path":"/0/renderable/bg"}])"_json));

	// value not needed
	KO( json_patch p1( R"([{"op":"remove","path":"/0/renderable/bg", "value":"Black"}])"_json));
	// value missing
	KO( json_patch p1( R"([{"op":"add","path":"/0/renderable/bg"}])"_json));
	// value missing
	KO( json_patch p1( R"([{"op":"replace","path":"/0/renderable/bg"}])"_json));

	// wrong op
	KO( json_patch p1( R"([{"op":"ad","path":"/0/renderable/bg","value":"Black"}])"_json));

	// invalid json-pointer
	KO( json_patch p1( R"([{"op":"add","path":"0/renderable/bg","value":"Black"}])"_json));

	return 0;
}
