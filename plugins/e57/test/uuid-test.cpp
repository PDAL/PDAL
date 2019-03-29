#include <gtest/gtest.h>

#include "../io/uuid.hpp"

using namespace uuid;


std::vector<std::string> split(const std::string& s, char c) {
    std::string::size_type i = 0;
    std::string::size_type j = s.find(c);
    std::vector<std::string> v;
    while (j != std::string::npos) {
        v.push_back(s.substr(i, j-i));
        i = ++j;
        j = s.find(c, j);

        if (j == std::string::npos)
            v.push_back(s.substr(i, s.length()));
    }
    return v;
}

TEST(UUID, uuidGenerate)
{
    std::string id = generate_uuid();
    ASSERT_EQ(id.size(), 36);

    std::vector<std::string> components = split(id,'-');
    ASSERT_EQ(components.size(),5);
    ASSERT_EQ(components[0].size(),8);
    ASSERT_EQ(components[1].size(),4);
    ASSERT_EQ(components[2].size(),4);
    ASSERT_EQ(components[3].size(),4);
    ASSERT_EQ(components[4].size(),12);
}