// Boost uuid_serialize.hpp header file  ----------------------------------------------//

// Copyright 2007 Andy Tompkins.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// Revision History
//  12 Nov 2007 - Initial Revision
//  25 Feb 2008 - moved to namespace boost::uuids::detail

#ifndef PDAL_EXTERNAL_BOOST_UUID_SERIALIZE_HPP
#define PDAL_EXTERNAL_BOOST_UUID_SERIALIZE_HPP

#include <pdal/external/boost/uuid/uuid.hpp>
#include <boost/serialization/level.hpp>

BOOST_CLASS_IMPLEMENTATION(pdal::external::boost::uuids::uuid, boost::serialization::primitive_type)

#endif // BOOST_UUID_SERIALIZE_HPP
