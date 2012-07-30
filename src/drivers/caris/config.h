/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/
#ifndef INCLUDED_DRIVERS_CSAR_CONFIG_HPP
#define INCLUDED_DRIVERS_CSAR_CONFIG_HPP

#ifdef _MSC_VER

// disable msvc's "secure" and "deperated" warnings
#   ifndef _CRT_SECURE_NO_WARNINGS
#       define _CRT_SECURE_NO_WARNINGS
#   endif
#   ifndef _CRT_SECURE_NO_DEPRECATE
#       define _CRT_SECURE_NO_DEPRECATE
#   endif
#   ifndef _AFX_SECURE_NO_WARNINGS
#       define _AFX_SECURE_NO_WARNINGS
#   endif
#   ifndef _ATL_SECURE_NO_WARNINGS
#       define _ATL_SECURE_NO_WARNINGS
#   endif
#   ifndef _SCL_SECURE_NO_WARNINGS
#       define _SCL_SECURE_NO_WARNINGS
#   endif

    //! Warnings to disable when including 3rd party headers
#   define DISABLED_3RDPARTY_WARNINGS 4244 4251 4267 4345 4503 4510 4512 4610 4701 4702
#endif

#endif
