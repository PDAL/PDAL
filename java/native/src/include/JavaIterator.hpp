/******************************************************************************
* Copyright (c) 2016, hobu Inc.  (info@hobu.co)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

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
