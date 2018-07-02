/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (howard@hobu.co)
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
 *     * Neither the name of the Howard Butler or Hobu, Inc.
 *       the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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


#ifndef INCLUDED_HEXER_EXPORT_HPP
#define INCLUDED_HEXER_EXPORT_HPP

#include <hexer/hexer_defines.h>

#ifndef HEXER_DLL
#if defined(HEXER_COMPILER_MSVC) && !defined(HEXER_DISABLE_DLL)
#if defined(HEXER_DLL_EXPORT)
#   define HEXER_DLL   __declspec(dllexport)
#elif defined(HEXER_DLL_IMPORT)
#   define HEXER_DLL   __declspec(dllimport)
#else
#   define HEXER_DLL
#endif
#else
#  if defined(USE_GCC_VISIBILITY_FLAG)
#    define HEXER_DLL     __attribute__ ((visibility("default")))
#  else
#    define HEXER_DLL
#  endif
#endif
#endif

#ifdef HEXER_COMPILER_MSVC
#pragma warning(disable:4251)// [templated class] needs to have dll-interface...
#endif

#endif
