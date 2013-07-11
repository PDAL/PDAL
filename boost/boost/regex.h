/*
 *
 * Copyright (c) 1998-2000
 * Dr John Maddock
 *
 * Use, modification and distribution are subject to the 
 * Boost Software License, Version 1.0. (See accompanying file 
 * LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 */
 
 /*
  *   LOCATION:    see http://www.boost.org/libs/regex for documentation.
  *   FILE         regex.h
  *   VERSION      3.12
  *   DESCRIPTION: Declares POSIX API functions
  */

#ifndef BOOST_RE_REGEX_H
#define BOOST_RE_REGEX_H

#include <boost/cregex.hpp>

/*
*  add using declarations to bring POSIX API functions into
* global scope, only if this is C++ (and not C).
*/
#ifdef __cplusplus

using pdalboost::regoff_t;
using pdalboost::regex_tA;
using pdalboost::regmatch_t;
using pdalboost::REG_BASIC;
using pdalboost::REG_EXTENDED;
using pdalboost::REG_ICASE;
using pdalboost::REG_NOSUB;
using pdalboost::REG_NEWLINE;
using pdalboost::REG_NOSPEC;
using pdalboost::REG_PEND;
using pdalboost::REG_DUMP;
using pdalboost::REG_NOCOLLATE;
using pdalboost::REG_ESCAPE_IN_LISTS;
using pdalboost::REG_NEWLINE_ALT;
using pdalboost::REG_PERL;
using pdalboost::REG_AWK;
using pdalboost::REG_GREP;
using pdalboost::REG_EGREP;
using pdalboost::REG_ASSERT;
using pdalboost::REG_INVARG;
using pdalboost::REG_ATOI;
using pdalboost::REG_ITOA;

using pdalboost::REG_NOTBOL;
using pdalboost::REG_NOTEOL;
using pdalboost::REG_STARTEND;

using pdalboost::reg_comp_flags;
using pdalboost::reg_exec_flags;
using pdalboost::regcompA;
using pdalboost::regerrorA;
using pdalboost::regexecA;
using pdalboost::regfreeA;

#ifndef BOOST_NO_WREGEX
using pdalboost::regcompW;
using pdalboost::regerrorW;
using pdalboost::regexecW;
using pdalboost::regfreeW;
using pdalboost::regex_tW;
#endif

using pdalboost::REG_NOERROR;
using pdalboost::REG_NOMATCH;
using pdalboost::REG_BADPAT;
using pdalboost::REG_ECOLLATE;
using pdalboost::REG_ECTYPE;
using pdalboost::REG_EESCAPE;
using pdalboost::REG_ESUBREG;
using pdalboost::REG_EBRACK;
using pdalboost::REG_EPAREN;
using pdalboost::REG_EBRACE;
using pdalboost::REG_BADBR;
using pdalboost::REG_ERANGE;
using pdalboost::REG_ESPACE;
using pdalboost::REG_BADRPT;
using pdalboost::REG_EEND;
using pdalboost::REG_ESIZE;
using pdalboost::REG_ERPAREN;
using pdalboost::REG_EMPTY;
using pdalboost::REG_E_MEMORY;
using pdalboost::REG_E_UNKNOWN;
using pdalboost::reg_errcode_t;

#endif /* __cplusplus */

#endif /* BOOST_RE_REGEX_H */




