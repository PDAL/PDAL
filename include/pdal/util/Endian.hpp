// Stolen from libLAS' endian.hpp
/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Endian macros
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 * This file has been stolen from <boost/endian.hpp> and
 * modified for libLAS purposes.
 *
 * (C) Copyright Mateusz Loskot 2007, mateusz@loskot.net
 * (C) Copyright Caleb Epstein 2005
 * (C) Copyright John Maddock 2006
 * Distributed under the Boost  Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or copy at
 * http://www.boost.org/LICENSE_1_0.txt)
 *
 * Revision History
 * 06 Feb 2006 - Initial Revision
 * 09 Nov 2006 - fixed variant and version bits for v4 guids
 * 13 Nov 2006 - added serialization
 * 17 Nov 2006 - added name-based guid creation
 * 20 Nov 2006 - add fixes for gcc (from Tim Blechmann)
 * 07 Mar 2007 - converted to header only
 * 20 Jan 2008 - removed dependency of Boost and modified for libLAS (by Mateusz Loskot)
 ******************************************************************************
 *
 * Copyright (c) 1997
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 *
 *
 * Copyright notice reproduced from <boost/detail/limits.hpp>, from
 * which this code was originally taken.
 *
 * Modified by Caleb Epstein to use <endian.h> with GNU libc and to
 * defined the BOOST_ENDIAN macro.
 ****************************************************************************/

#pragma once

# define SWAP_BE_TO_LE(p) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + sizeof(p) - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)

# define SWAP_LE_TO_BE(p) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + sizeof(p) - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)

# define SWAP_BE_TO_LE_N(p, n) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + n - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)

# define SWAP_LE_TO_BE_N(p, n) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + n - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)

# define SWAP_ENDIANNESS(p) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + sizeof(p) - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)


# define SWAP_ENDIANNESS_N(p, n) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + n - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)
