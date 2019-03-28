#ifndef _UUID_H_
#define _UUID_H_

#include <vector>
#include <iostream>
#include <sstream>
#include <random>
#include <climits>
#include <algorithm>
#include <functional>
#include <string>

namespace uuid{
unsigned char random_char();
std::string generate_hex(const unsigned int len) ;
std::string generate_uuid();
}

#endif