#include <iostream>
#include <string>
#include <set>
#include "JavaPipeline.hpp"

#ifndef _JAVAITERATOR_H_INCLUDED_
#define _JAVAITERATOR_H_INCLUDED_

using pdal::PointViewLess;
using pdal::PointViewPtr;

namespace libpdaljava
{
template <class K, class V>
class JavaIterator {
public:
	JavaIterator() {}
	JavaIterator(const std::set<K, V> set)
        : container{set}, curr_pos{0}
    { }
    JavaIterator(const std::set<K, V> *set)
        : container{*set}, curr_pos{0}
    { }
	bool hasNext() const {
		return curr_pos < container.size();
	}
	K next() {
	    if(!hasNext())
            throw java_error("iterator is out of bounds");

	    return *std::next(container.begin(), curr_pos++);
    }
    int size() const {
        return container.size();
    }

private:
	std::set<K, V> container;
	unsigned int curr_pos;
};

typedef JavaIterator<PointViewPtr, PointViewLess> PointViewIterator;
}
#endif
