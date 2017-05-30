/******************************************************************************
* Copyright (c) 2016, Hobu Inc., hobu@hobu.co
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
*     * Neither the name of Hobu, Inc. Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
****************************************************************************/

#pragma once

#include <typeinfo>

#define TYPENAME_ADD(A) template<> struct TypeName<A> { static const char *get() { return #A; }};

using std::string;

namespace pdal
{
    template <typename T>
    struct TypeName
    {
        static string get()
        {
            return string (typeid(T).name());
        }
    };

    template <>
    struct TypeName<char>
    {
        static string get()
        {
            return "string";
        }
    };

    TYPENAME_ADD(int);
    TYPENAME_ADD(long);
    TYPENAME_ADD(float);
    TYPENAME_ADD(double);
    TYPENAME_ADD(bool);
} // namespace pdal
