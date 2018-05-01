#define GEOGRAM_PSM
#ifndef GEO_STATIC_LIBS
#define GEO_DYNAMIC_LIBS
#endif
/*
 *  Copyright (c) 2012-2014, Bruno Levy
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     Bruno.Levy@inria.fr
 *     http://www.loria.fr/~levy
 *
 *     ALICE Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 */


/*
 *  This file is a PSM (pluggable software module)
 *   generated from the distribution of Geogram.
 *
 *  See Geogram documentation on:
 *   http://alice.loria.fr/software/geogram/doc/html/index.html
 *
 *  See documentation of the functions bundled in this PSM on:
 *   http://alice.loria.fr/software/geogram/doc/html/classGEO_1_1Delaunay.html
 */



/******* extracted from ../api/defs.h *******/

#ifndef GEOGRAM_API_DEFS
#define GEOGRAM_API_DEFS


/*
 * Deactivate warnings about documentation
 * We do that, because CLANG's doxygen parser does not know
 * some doxygen commands that we use (retval, copydoc) and
 * generates many warnings for them...
 */
#if defined(__clang__)
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wdocumentation-unknown-command" 
#endif


#if defined(GEO_DYNAMIC_LIBS)
   #if defined(_MSC_VER)
      #define GEO_IMPORT __declspec(dllimport) 
      #define GEO_EXPORT __declspec(dllexport) 
   #elif defined(__GNUC__)
      #define GEO_IMPORT  
      #define GEO_EXPORT __attribute__ ((visibility("default")))
   #else
      #define GEO_IMPORT
      #define GEO_EXPORT
   #endif
#else
   #define GEO_IMPORT
   #define GEO_EXPORT
#endif

#ifdef geogram_EXPORTS
#define GEOGRAM_API GEO_EXPORT
#else
#define GEOGRAM_API GEO_IMPORT
#endif


#define NO_GEOGRAM_API

typedef int GeoMesh;

typedef unsigned char geo_coord_index_t;

typedef unsigned int geo_index_t;

typedef int geo_signed_index_t;

typedef double geo_coord_t;

typedef int geo_boolean;

enum {
    GEO_FALSE = 0,
    GEO_TRUE = 1
};

#endif


/******* extracted from ../basic/common.h *******/

#ifndef GEOGRAM_BASIC_COMMON
#define GEOGRAM_BASIC_COMMON


// iostream should be included before anything else,
// otherwise 'cin', 'cout' and 'cerr' will be uninitialized.
#include <iostream>



namespace GEO {

    void GEOGRAM_API initialize();

    void GEOGRAM_API terminate();
}


#if (defined(NDEBUG) || defined(GEOGRAM_PSM)) && !defined(GEOGRAM_PSM_DEBUG)
#undef GEO_DEBUG
#undef GEO_PARANOID
#else
#define GEO_DEBUG
#define GEO_PARANOID
#endif

// =============================== LINUX defines ===========================

#if defined(__ANDROID__)
#define GEO_OS_ANDROID
#endif

#if defined(__linux__)

#define GEO_OS_LINUX
#define GEO_OS_UNIX

#ifndef GEO_OS_ANDROID
#define GEO_OS_X11
#endif

#if defined(_OPENMP)
#  define GEO_OPENMP
#endif

#if defined(__INTEL_COMPILER)
#  define GEO_COMPILER_INTEL
#elif defined(__clang__)
#  define GEO_COMPILER_CLANG
#elif defined(__GNUC__)
#  define GEO_COMPILER_GCC
#else
#  error "Unsupported compiler"
#endif

// The following works on GCC and ICC
#if defined(__x86_64)
#  define GEO_ARCH_64
#else
#  define GEO_ARCH_32
#endif

// =============================== WINDOWS defines =========================

#elif defined(WIN32) || defined(_WIN64)

#define GEO_OS_WINDOWS

#if defined(_OPENMP)
#  define GEO_OPENMP
#endif

#if defined(_MSC_VER)
#  define GEO_COMPILER_MSVC
#else
#  error "Unsupported compiler"
#endif

#if defined(_WIN64)
#  define GEO_ARCH_64
#else
#  define GEO_ARCH_32
#endif

// =============================== APPLE defines ===========================

#elif defined(__APPLE__)

#define GEO_OS_APPLE
#define GEO_OS_UNIX

#if defined(_OPENMP)
#  define GEO_OPENMP
#endif

#if defined(__clang__)
#  define GEO_COMPILER_CLANG
#else
#  error "Unsupported compiler"
#endif

#if defined(__x86_64) || defined(__ppc64__)
#  define GEO_ARCH_64
#else
#  define GEO_ARCH_32
#endif

// =============================== Emscripten defines  ======================

#elif defined(__EMSCRIPTEN__)

#define GEO_OS_UNIX
#define GEO_OS_LINUX
#define GEO_OS_EMSCRIPTEN
#define GEO_ARCH_64
#define GEO_COMPILER_EMSCRIPTEN

// =============================== Unsupported =============================
#else

#error "Unsupported operating system"

#endif

#ifdef DOXYGEN_ONLY
// Keep doxygen happy
#define GEO_OS_WINDOWS
#define GEO_OS_APPLE
#define GEO_OS_ANDROID
#define GEO_ARCH_32
#define GEO_COMPILER_INTEL
#define GEO_COMPILER_MSVC
#endif

#define CPP_CONCAT_(A, B) A ## B

#define CPP_CONCAT(A, B) CPP_CONCAT_(A, B)

#if defined(GOMGEN)
#define GEO_NORETURN
#elif defined(GEO_COMPILER_CLANG) || \
    defined(GEO_COMPILER_GCC)   || \
    defined(GEO_COMPILER_EMSCRIPTEN) || \
    defined(GEO_COMPILER_INTEL)
#define GEO_NORETURN __attribute__((noreturn))
#else
#define GEO_NORETURN
#endif

#if defined(GOMGEN)
#define GEO_NORETURN_DECL 
#elif defined(GEO_COMPILER_MSVC)
#define GEO_NORETURN_DECL __declspec(noreturn)
#else
#define GEO_NORETURN_DECL 
#endif

#if defined(GEO_COMPILER_CLANG) || defined(GEO_COMPILER_EMSCRIPTEN)
#if __has_feature(cxx_noexcept)
#define GEO_NOEXCEPT noexcept
#endif
#endif

#ifndef GEO_NOEXCEPT
#define GEO_NOEXCEPT throw()
#endif

#define FOR(I,UPPERBND) for(index_t I = 0; I<index_t(UPPERBND); ++I)

#endif


/******* extracted from ../basic/assert.h *******/

#ifndef GEOGRAM_BASIC_ASSERT
#define GEOGRAM_BASIC_ASSERT

#include <string>


namespace GEO {

    enum AssertMode {
        
        ASSERT_THROW,
        
        ASSERT_ABORT,
	
	ASSERT_BREAKPOINT
    };

    void GEOGRAM_API set_assert_mode(AssertMode mode);

    AssertMode GEOGRAM_API assert_mode();

    GEO_NORETURN_DECL void GEOGRAM_API geo_abort() GEO_NORETURN;

    GEO_NORETURN_DECL void GEOGRAM_API geo_breakpoint() GEO_NORETURN;
    
    GEO_NORETURN_DECL void GEOGRAM_API geo_assertion_failed(
        const std::string& condition_string,
        const std::string& file, int line
    ) GEO_NORETURN;

    GEO_NORETURN_DECL void GEOGRAM_API geo_range_assertion_failed(
        double value, double min_value, double max_value,
        const std::string& file, int line
    ) GEO_NORETURN;

    GEO_NORETURN_DECL void GEOGRAM_API geo_should_not_have_reached(
        const std::string& file, int line
    ) GEO_NORETURN;
}

// Three levels of assert:
// use geo_assert() and geo_range_assert()               non-expensive asserts
// use geo_debug_assert() and geo_debug_range_assert()   expensive asserts
// use geo_parano_assert() and geo_parano_range_assert() very exensive asserts

#define geo_assert(x) {                                      \
        if(!(x)) {                                               \
            GEO::geo_assertion_failed(#x, __FILE__, __LINE__);   \
        }                                                        \
}

#define geo_range_assert(x, min_val, max_val) {              \
        if(((x) < (min_val)) || ((x) > (max_val))) {             \
            GEO::geo_range_assertion_failed(x, min_val, max_val, \
                __FILE__, __LINE__                               \
            );                                                   \
        }                                                        \
}

#define geo_assert_not_reached {                             \
        GEO::geo_should_not_have_reached(__FILE__, __LINE__);    \
}

#ifdef GEO_DEBUG
#define geo_debug_assert(x) geo_assert(x)
#define geo_debug_range_assert(x, min_val, max_val) geo_range_assert(x, min_val, max_val)
#else
#define geo_debug_assert(x)
#define geo_debug_range_assert(x, min_val, max_val)
#endif

#ifdef GEO_PARANOID
#define geo_parano_assert(x) geo_assert(x)
#define geo_parano_range_assert(x, min_val, max_val) geo_range_assert(x, min_val, max_val)
#else
#define geo_parano_assert(x)
#define geo_parano_range_assert(x, min_val, max_val)
#endif

#endif


/******* extracted from ../basic/argused.h *******/

#ifndef GEOGRAM_BASIC_ARGUSED
#define GEOGRAM_BASIC_ARGUSED



namespace GEO {

    template <class T>
    inline void geo_argused(const T&) {
    }
}

#endif


/******* extracted from ../basic/numeric.h *******/

#ifndef GEOGRAM_BASIC_NUMERIC
#define GEOGRAM_BASIC_NUMERIC

#include <cmath>
#include <float.h>
#include <limits.h>

// Visual C++ ver. < 2010 does not have C99 stdint.h,
// using a fallback portable one.
#if defined(GEO_OS_WINDOWS) && (_MSC_VER < 1600)
#else
#include <stdint.h>
#endif

#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace GEO {

    namespace Numeric {

        
        typedef void* pointer;

        
        typedef int8_t int8;

        
        typedef int16_t int16;

        
        typedef int32_t int32;

        
        typedef int64_t int64;

        
        typedef uint8_t uint8;

        
        typedef uint16_t uint16;

        
        typedef uint32_t uint32;

        
        typedef uint64_t uint64;

        
        typedef float float32;

        
        typedef double float64;

        inline float32 max_float32() {
            return (std::numeric_limits<float32>::max)();
        }

        inline float32 min_float32() {
            // Note: numeric_limits<>::min() is not
            // what we want (it returns the smallest
            // positive non-denormal).
            return -max_float32();
        }

        inline float64 max_float64() {
            return (std::numeric_limits<float64>::max)();
        }

        inline float64 min_float64() {
            // Note: numeric_limits<>::min() is not
            // what we want (it returns the smallest
            // positive non-denormal).
            return -max_float64();
        }

        bool GEOGRAM_API is_nan(float32 x);

        bool GEOGRAM_API is_nan(float64 x);

        void GEOGRAM_API random_reset();

        int32 GEOGRAM_API random_int32();

        float32 GEOGRAM_API random_float32();

        float64 GEOGRAM_API random_float64();

        template <class T, bool is_numeric>
        struct LimitsHelper : std::numeric_limits<T> {
        };

        template <class T>
        struct LimitsHelper<T, true> : std::numeric_limits<T> {
            
            static const size_t size = sizeof(T);
            
            static const size_t numbits = 8 * sizeof(T);
        };

        template <class T>
        struct Limits : 
            LimitsHelper<T, std::numeric_limits<T>::is_specialized> {
        };
    }

    

    template <class T>
    inline T geo_max(T x1, T x2) {
        return x1 < x2 ? x2 : x1;
    }

    template <class T>
    inline T geo_min(T x1, T x2) {
        return x2 < x1 ? x2 : x1;
    }

    enum Sign {
        
        NEGATIVE = -1,
        
        ZERO = 0,
        
        POSITIVE = 1
    };

    template <class T>
    inline Sign geo_sgn(const T& x) {
        return (x > 0) ? POSITIVE : (
            (x < 0) ? NEGATIVE : ZERO
        );
    }

    template <class T>
    inline T geo_abs(T x) {
        return (x < 0) ? -x : x;
    }

    template <class T>
    inline T geo_sqr(T x) {
        return x * x;
    }

    template <class T>
    inline void geo_clamp(T& x, T min, T max) {
        if(x < min) {
            x = min;
        } else if(x > max) {
            x = max;
        }
    }

    template <class T>
    inline void geo_swap(T& x, T& y) {
        T z = x;
        x = y;
        y = z;
    }

    typedef geo_index_t index_t;

    inline index_t max_index_t() {
        return (std::numeric_limits<index_t>::max)();
    }

    typedef geo_signed_index_t signed_index_t;

    inline signed_index_t max_signed_index_t() {
        return (std::numeric_limits<signed_index_t>::max)();
    }

    inline signed_index_t min_signed_index_t() {
        return (std::numeric_limits<signed_index_t>::min)();
    }

    typedef geo_coord_index_t coord_index_t;

    inline double round(double x) {
	return ((x - floor(x)) > 0.5 ? ceil(x) : floor(x));
    }
}

#endif


/******* extracted from ../basic/memory.h *******/

#ifndef GEOGRAM_BASIC_MEMORY
#define GEOGRAM_BASIC_MEMORY

#include <vector>
#include <string.h>
#include <stdlib.h>

#ifdef GEO_OS_WINDOWS

#include <windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#else

#include <unistd.h>

#endif


namespace GEO {

    namespace Memory {
        
        typedef unsigned char byte;

        
        typedef unsigned char word8;

        
        typedef unsigned short word16;

        
        typedef unsigned int word32;

        
        typedef byte* pointer;

	
	typedef void (*function_pointer)();
	
#define nil 0

        inline void clear(void* addr, size_t size) {
            ::memset(addr, 0, size);
        }

        inline void copy(void* to, const void* from, size_t size) {
            ::memcpy(to, from, size);
        }

	inline pointer function_pointer_to_generic_pointer(function_pointer fptr) {
	    // I know this is ugly, but I did not find a simpler warning-free
	    // way that is portable between all compilers.
	    pointer result = nil;
	    ::memcpy(&result, &fptr, sizeof(pointer));
	    return result;
	}

	inline function_pointer generic_pointer_to_function_pointer(pointer ptr) {
	    // I know this is ugly, but I did not find a simpler warning-free
	    // way that is portable between all compilers.
	    function_pointer result = nil;
	    ::memcpy(&result, &ptr, sizeof(pointer));
	    return result;
	}

	inline function_pointer generic_pointer_to_function_pointer(void* ptr) {
	    // I know this is ugly, but I did not find a simpler warning-free
	    // way that is portable between all compilers.
	    function_pointer result = nil;
	    ::memcpy(&result, &ptr, sizeof(pointer));
	    return result;
	}
	
#define GEO_MEMORY_ALIGNMENT 64

        template <int DIM>
        struct PointAlignment {
            static const size_t value = 1;
        };

        template <>
        struct PointAlignment<2> {
            static const size_t value = 16;
        };

        template <>
        struct PointAlignment<3> {
            static const size_t value = 8;
        };

        template <>
        struct PointAlignment<4> {
            static const size_t value = 32;
        };

        template <>
        struct PointAlignment<6> {
            static const size_t value = 16;
        };

        template <>
        struct PointAlignment<8> {
            static const size_t value = 64;
        };

#define geo_dim_alignment(dim) GEO::Memory::PointAlignment<dim>::value

        inline void* aligned_malloc(
            size_t size, size_t alignment = GEO_MEMORY_ALIGNMENT
        ) {
#if   defined(GEO_OS_ANDROID)
            // Alignment not supported under Android.
            geo_argused(alignment);
            return malloc(size);
#elif defined(GEO_COMPILER_INTEL)
            return _mm_malloc(size, alignment);
#elif defined(GEO_COMPILER_GCC) || defined(GEO_COMPILER_CLANG)
            void* result;
            return posix_memalign(&result, alignment, size) == 0
                   ? result : 0;
#elif defined(GEO_COMPILER_MSVC)
            return _aligned_malloc(size, alignment);
#else
            geo_argused(alignment);
            return malloc(size);
#endif
        }

        inline void aligned_free(void* p) {
#if   defined(GEO_OS_ANDROID)
            // Alignment not supported under Android.
            free(p);
#elif defined(GEO_COMPILER_INTEL)
            _mm_free(p);
#elif defined(GEO_COMPILER_GCC) || defined(GEO_COMPILER_CLANG)
            free(p);
#elif defined(GEO_COMPILER_MSVC)
            _aligned_free(p);
#else
            free(p);
#endif
        }

#if   defined(GEO_OS_ANDROID)
#define geo_decl_aligned(var) var
#elif defined(GEO_COMPILER_INTEL)
#define geo_decl_aligned(var) __declspec(aligned(GEO_MEMORY_ALIGNMENT)) var
#elif defined(GEO_COMPILER_GCC) || defined(GEO_COMPILER_CLANG)
#define geo_decl_aligned(var) var __attribute__((aligned(GEO_MEMORY_ALIGNMENT)))
#elif defined(GEO_COMPILER_MSVC)
#define geo_decl_aligned(var) __declspec(align(GEO_MEMORY_ALIGNMENT)) var
#elif defined(GEO_COMPILER_EMSCRIPTEN)
#define geo_decl_aligned(var) var        
#endif

#if   defined(GEO_OS_ANDROID)
#define geo_assume_aligned(var, alignment)
#elif defined(GEO_COMPILER_INTEL)
#define geo_assume_aligned(var, alignment) \
    __assume_aligned(var, alignment)
#elif defined(GEO_COMPILER_CLANG)
#define geo_assume_aligned(var, alignment)
        // GCC __builtin_assume_aligned is not yet supported by clang-3.3
#elif defined(GEO_COMPILER_GCC)
#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 7
#define geo_assume_aligned(var, alignment) \
        *(void**) (&var) = __builtin_assume_aligned(var, alignment)
        // the GCC way of specifiying that a pointer is aligned returns
        // the aligned pointer (I can't figure out why). It needs to be
        // affected otherwise it is not taken into account (verified by
        // looking at the output of gcc -S)
#else
#define geo_assume_aligned(var, alignment)        
#endif        
#elif defined(GEO_COMPILER_MSVC)
#define geo_assume_aligned(var, alignment)
        // TODO: I do not know how to do that with MSVC
#elif defined(GEO_COMPILER_EMSCRIPTEN)        
#define geo_assume_aligned(var, alignment)
#endif

#if   defined(GEO_COMPILER_INTEL)
#define geo_restrict __restrict
#elif defined(GEO_COMPILER_GCC) || defined(GEO_COMPILER_CLANG)
#define geo_restrict __restrict__
#elif defined(GEO_COMPILER_MSVC)
#define geo_restrict __restrict
#elif defined(GEO_COMPILER_EMSCRIPTEN)
#define geo_restrict 
#endif

        inline bool is_aligned(
            void* p, size_t alignment = GEO_MEMORY_ALIGNMENT
        ) {
            return (reinterpret_cast<size_t>(p) & (alignment - 1)) == 0;
        }

        inline void* align(void* p) {
            size_t offset = (
                GEO_MEMORY_ALIGNMENT -
                (reinterpret_cast<size_t>(p) & (GEO_MEMORY_ALIGNMENT - 1))
            ) & (GEO_MEMORY_ALIGNMENT - 1);
            return reinterpret_cast<char*>(p) + offset;
        }

#define geo_aligned_alloca(size) \
    GEO::Memory::align(alloca(size + GEO_MEMORY_ALIGNMENT - 1))

        template <class T, int ALIGN = GEO_MEMORY_ALIGNMENT>
        class aligned_allocator {
        public:
            
            typedef T value_type;

            
            typedef T* pointer;

            
            typedef T& reference;

            
            typedef const T* const_pointer;

            
            typedef const T& const_reference;

            
            typedef ::std::size_t size_type;

            
            typedef ::std::ptrdiff_t difference_type;

            template <class U>
            struct rebind {
                
                typedef aligned_allocator<U> other;
            };

            pointer address(reference x) {
                return &x;
            }

            const_pointer address(const_reference x) {
                return &x;
            }

            pointer allocate(
                size_type nb_elt, ::std::allocator<void>::const_pointer hint = 0
            ) {
                geo_argused(hint);
                pointer result = static_cast<pointer>(
                    aligned_malloc(sizeof(T) * nb_elt, ALIGN)
                );
                return result;
            }

            void deallocate(pointer p, size_type nb_elt) {
                geo_argused(nb_elt);
                aligned_free(p);
            }

            size_type max_size() const {
                ::std::allocator<char> a;
                return a.max_size() / sizeof(T);
            }

            void construct(pointer p, const_reference val) {
                new (static_cast<void*>(p))value_type(val);
            }

            void destroy(pointer p) {
                p->~value_type();
#ifdef GEO_COMPILER_MSVC
                (void) p; // to avoid a "unreferenced variable" warning
#endif
            }

            template <class T2, int A2> operator aligned_allocator<T2, A2>() {
                return aligned_allocator<T2,A2>();
            }
        };

        template <typename T1, int A1, typename T2, int A2>
        inline bool operator== (
            const aligned_allocator<T1, A1>&, const aligned_allocator<T2, A2>&
        ) {
            return true;
        }

        template <typename T1, int A1, typename T2, int A2>
        inline bool operator!= (
            const aligned_allocator<T1, A1>&, const aligned_allocator<T2, A2>&
        ) {
            return false;
        }
    }

    

    template <class T>
    class vector : public ::std::vector<T, Memory::aligned_allocator<T> > {
        typedef ::std::vector<T, Memory::aligned_allocator<T> > baseclass;

    public:
        vector() :
            baseclass() {
        }

        explicit vector(index_t size) :
            baseclass(size) {
        }

        explicit vector(index_t size, const T& val) :
            baseclass(size, val) {
        }

        index_t size() const {
            //   casts baseclass::size() from size_t (64 bits)
            //   to index_t (32 bits), because all
            //   indices in Vorpaline are supposed to fit in 32 bits (index_t).
            // TODO: geo_debug_assert(baseclass::size() < max index_t)
            return index_t(baseclass::size());
        }

        T& operator[] (index_t i) {
            geo_debug_assert(i < size());
            return baseclass::operator[] (i);
        }

        const T& operator[] (index_t i) const {
            geo_debug_assert(i < size());
            return baseclass::operator[] (i);
        }

        T& operator[] (signed_index_t i) {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }

        const T& operator[] (signed_index_t i) const {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }

        T* data() {
            return size() == 0 ? nil : &(*this)[0];
        }

        const T* data() const {
            return size() == 0 ? nil : &(*this)[0];
        }

    };

    template <>
    class vector<bool> : public ::std::vector<bool> {
        typedef ::std::vector<bool> baseclass;

    public:
        
        vector() :
            baseclass() {
        }

        
        explicit vector(index_t size) :
            baseclass(size) {
        }

        
        explicit vector(index_t size, bool val) :
            baseclass(size, val) {
        }

        
        index_t size() const {
            //   casts baseclass::size() from size_t (64 bits)
            //   to index_t (32 bits), because all
            //   indices in Vorpaline are supposed to fit in 32 bits (index_t).
            // TODO: geo_debug_assert(baseclass::size() < max index_t)
            return index_t(baseclass::size());
        }

        // TODO: operator[] with bounds checking (more complicated
        // than just returning bool&, check implementation in STL).
    };
}

#endif


/******* extracted from ../basic/counted.h *******/

#ifndef GEOGRAM_BASIC_COUNTED
#define GEOGRAM_BASIC_COUNTED



namespace GEO {

    class GEOGRAM_API Counted {
    public:
        void ref() const {
            ++nb_refs_;
        }

        void unref() const {
            --nb_refs_;
            geo_debug_assert(nb_refs_ >= 0);
            if(nb_refs_ == 0) {
                delete this;
            }
        }

        bool is_shared() const {
            return nb_refs_ > 1;
        }

        static void ref(const Counted* counted) {
            if(counted != nil) {
                counted->ref();
            }
        }

        static void unref(const Counted* counted) {
            if(counted != nil) {
                counted->unref();
            }
        }

    protected:
        Counted() :
            nb_refs_(0) {
        }

        virtual ~Counted();

    private:
        
        Counted(const Counted&);
        
        Counted& operator= (const Counted&);

        mutable int nb_refs_;
    };
}

#endif


/******* extracted from ../basic/smart_pointer.h *******/

#ifndef GEOGRAM_BASIC_SMART_POINTER
#define GEOGRAM_BASIC_SMART_POINTER



namespace GEO {

    

    template <class T>
    class SmartPointer {
    public:
        SmartPointer() :
            pointer_(nil) {
        }

        SmartPointer(T* ptr) :
            pointer_(ptr) {
            T::ref(pointer_);
        }

        SmartPointer(const SmartPointer<T>& rhs) :
            pointer_(rhs) {
            T::ref(pointer_);
        }

        ~SmartPointer() {
            T::unref(pointer_);
        }

        SmartPointer<T>& operator= (T* ptr) {
            if(ptr != pointer_) {
                T::unref(pointer_);
                pointer_ = ptr;
                T::ref(pointer_);
            }
            return *this;
        }

        SmartPointer<T>& operator= (const SmartPointer<T>& rhs) {
            T* rhs_p = rhs.get();
            if(rhs_p != pointer_) {
                T::unref(pointer_);
                pointer_ = rhs_p;
                T::ref(pointer_);
            }
            return *this;
        }

        void reset() {
            T::unref(pointer_);
            pointer_ = nil;
        }

        T* operator-> () const {
            geo_assert(pointer_ != nil);
            return pointer_;
        }

        T& operator* () const {
            geo_assert(pointer_ != nil);
            return *pointer_;
        }

        operator T* () const {
            return pointer_;
        }

        T* get() const {
            return pointer_;
        }

        bool is_nil() const {
            return pointer_ == nil;
        }

    private:
        T* pointer_;
    };

    template <class T1, class T2>
    inline bool operator== (const SmartPointer<T1>& lhs, const SmartPointer<T2>& rhs) {
        return lhs.get() == rhs.get();
    }

    template <class T1, class T2>
    inline bool operator!= (const SmartPointer<T1>& lhs, const SmartPointer<T2>& rhs) {
        return lhs.get() != rhs.get();
    }

    template <class T1, class T2>
    inline bool operator< (const SmartPointer<T1>& lhs, const SmartPointer<T2>& rhs) {
        return lhs.get() < rhs.get();
    }

    template <class T1, class T2>
    inline bool operator<= (const SmartPointer<T1>& lhs, const SmartPointer<T2>& rhs) {
        return lhs.get() <= rhs.get();
    }

    template <class T1, class T2>
    inline bool operator> (const SmartPointer<T1>& lhs, const SmartPointer<T2>& rhs) {
        return lhs.get() > rhs.get();
    }

    template <class T1, class T2>
    inline bool operator>= (const SmartPointer<T1>& lhs, const SmartPointer<T2>& rhs) {
        return lhs.get() >= rhs.get();
    }
}

#endif


/******* extracted from ../basic/environment.h *******/

#ifndef GEOGRAM_BASIC_ENVIRONMENT
#define GEOGRAM_BASIC_ENVIRONMENT

#include <string>
#include <vector>
#include <map>


namespace GEO {

    class Environment;
    
    

    class GEOGRAM_API VariableObserver {
    public:
        VariableObserver(const std::string& var_name);

        virtual void value_changed(const std::string& new_value) = 0;

        virtual ~VariableObserver();

        const std::string& observed_variable() const {
            return observed_variable_;
        }

    private:
        std::string observed_variable_;
        Environment* environment_;
    };

    

    class GEOGRAM_API VariableObserverList {
    public:
        VariableObserverList() :
            block_notify_(false) {
        }

        void notify_observers(const std::string& value);

        void add_observer(VariableObserver* observer);

        void remove_observer(VariableObserver* observer);

    private:
        
        typedef std::vector<VariableObserver*> Observers;
        Observers observers_;
        bool block_notify_;
    };

    

    class GEOGRAM_API Environment : public Counted {
    public:
        static Environment* instance();

        static void terminate();

        virtual bool add_environment(Environment* env);

        bool has_value(const std::string& name) const;

        virtual bool get_value(
            const std::string& name, std::string& value
        ) const;

        std::string get_value(const std::string& name) const;

        virtual bool set_value(
            const std::string& name, const std::string& value
        );

        virtual Environment* find_environment(const std::string& name);
        
        virtual bool add_observer(
            const std::string& name, VariableObserver* observer
        );

        virtual bool remove_observer(
            const std::string& name, VariableObserver* observer
        );

        virtual bool notify_observers(
            const std::string& name, bool recursive = false
        );

    protected:
        virtual ~Environment();

        virtual bool get_local_value(
            const std::string& name, std::string& value
        ) const = 0;

        virtual bool set_local_value(
            const std::string& name, const std::string& value
        ) = 0;

        bool notify_observers(
            const std::string& name, const std::string& value,
            bool recursive
        );

        bool notify_local_observers(
            const std::string& name, const std::string& value
        );

    private:
        
        typedef SmartPointer<Environment> Environment_var;

        
        typedef std::vector<Environment_var> Environments;

        
        typedef std::map<std::string, VariableObserverList> ObserverMap;

        static Environment_var instance_;
        Environments environments_;
        ObserverMap observers_;
    };

    

    class SystemEnvironment : public Environment {
    protected:
        
        virtual ~SystemEnvironment();

        virtual bool set_local_value(
            const std::string& name, const std::string& value
        );

        virtual bool get_local_value(
            const std::string& name, std::string& value
        ) const;
    };
}

#endif


/******* extracted from ../basic/logger.h *******/

#ifndef GEOGRAM_BASIC_LOGGER
#define GEOGRAM_BASIC_LOGGER

#ifdef __cplusplus

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <stdlib.h>


namespace GEO {

    class Logger;
    class LoggerStream;

    class NO_GEOGRAM_API LoggerStreamBuf : public std::stringbuf {
    public:
        LoggerStreamBuf(LoggerStream* loggerStream) :
            loggerStream_(loggerStream) {
        }

    private:
        virtual int sync();

    private:
        LoggerStream* loggerStream_;
    };

    

    class NO_GEOGRAM_API LoggerStream : public std::ostream {
    public:
        LoggerStream(Logger* logger);

        virtual ~LoggerStream();

    protected:
        void notify(const std::string& str);

    private:
        Logger* logger_;
        friend class LoggerStreamBuf;
    };

    

    class GEOGRAM_API LoggerClient : public Counted {
    public:
        virtual void div(const std::string& title) = 0;

        virtual void out(const std::string& str) = 0;

        virtual void warn(const std::string& str) = 0;

        virtual void err(const std::string& str) = 0;

        virtual void status(const std::string& str) = 0;

        virtual ~LoggerClient();
    };

    
    typedef SmartPointer<LoggerClient> LoggerClient_var;

    

    class GEOGRAM_API ConsoleLogger : public LoggerClient {
    public:
        ConsoleLogger();

        void div(const std::string& title);

        void out(const std::string& str);

        void warn(const std::string& str);

        void err(const std::string& str);

        void status(const std::string& str);

    protected:
        virtual ~ConsoleLogger();
    };

    

    class GEOGRAM_API FileLogger : public LoggerClient {
    public:
        FileLogger();

        FileLogger(const std::string& file_name);

        void div(const std::string& title);

        void out(const std::string& str);

        void warn(const std::string& str);

        void err(const std::string& str);

        void status(const std::string& str);

    protected:
        virtual ~FileLogger();

        void set_file_name(const std::string& file_name);

    private:
        std::string log_file_name_;
        std::ostream* log_file_;
    };

    

    class GEOGRAM_API Logger : public Environment {
    public:
        static void initialize();

        static void terminate();

        static Logger* instance();


        static bool is_initialized();
        
       
        static std::ostream& div(const std::string& title);

        static std::ostream& out(const std::string& feature);

        static std::ostream& err(const std::string& feature);

        static std::ostream& warn(const std::string& feature);

        static std::ostream& status();

        void register_client(LoggerClient* client);

        void unregister_client(LoggerClient* client);

        void unregister_all_clients();
        
        bool is_client(LoggerClient* client) const;

        void set_quiet(bool flag);

        bool is_quiet() const {
            return quiet_;
        }


        void set_minimal(bool flag);

        bool is_minimal() const {
            return minimal_;
        }
        
        void set_pretty(bool flag);

        bool is_pretty() const {
            return pretty_;
        }

    protected:
        Logger();

        virtual ~Logger();

        
        std::ostream& div_stream(const std::string& title);

        
        std::ostream& out_stream(const std::string& feature);

        
        std::ostream& err_stream(const std::string& feature);

        
        std::ostream& warn_stream(const std::string& feature);

        
        std::ostream& status_stream();

        void notify(LoggerStream* sender, const std::string& message);

        void notify_out(const std::string& message);

        void notify_warn(const std::string& message);

        void notify_err(const std::string& message);

        void notify_status(const std::string& message);

        virtual bool set_local_value(
            const std::string& name, const std::string& value
        );

        virtual bool get_local_value(
            const std::string& name, std::string& value
        ) const;

    private:
        static SmartPointer<Logger> instance_;

        LoggerStream out_;
        LoggerStream warn_;
        LoggerStream err_;
        LoggerStream status_;

        // features we want or don't want to log (only applies to 'out').

        
        typedef std::set<std::string> FeatureSet;
        FeatureSet log_features_;
        FeatureSet log_features_exclude_;
        bool log_everything_;
        std::string log_file_name_;

        std::string current_feature_;
        bool current_feature_changed_;

        
        typedef std::set<LoggerClient_var> LoggerClients;
        LoggerClients clients_; // list of registered clients

        bool quiet_;
        bool pretty_;
        bool minimal_;
        
        friend class LoggerStream;
    };

    

}

extern "C" {
    int GEOGRAM_API geogram_printf(const char* format, ...);

    int GEOGRAM_API geogram_fprintf(FILE* out, const char* format, ...);
}

#else

#include <stdlib.h>

#ifndef GEOGRAM_API
#define GEOGRAM_API
#endif

extern int GEOGRAM_API geogram_printf(const char* format, ...);

extern int GEOGRAM_API geogram_fprintf(FILE* out, const char* format, ...);

#endif

#endif


/******* extracted from ../basic/atomics.h *******/

#ifndef GEOGRAM_BASIC_ATOMICS
#define GEOGRAM_BASIC_ATOMICS



#ifdef GEO_OS_LINUX
#  if defined(GEO_OS_EMSCRIPTEN) 
#    define GEO_USE_DUMMY_ATOMICS
#  elif defined(GEO_OS_ANDROID) || defined(GEO_OS_RASPBERRY)
#    define GEO_USE_ARM_ATOMICS
#  else
#    define GEO_USE_X86_ATOMICS
#  endif
#endif

#if defined(GEO_USE_DUMMY_ATOMICS)

inline void geo_pause() {
}

inline char atomic_bittestandset_x86(volatile unsigned int*, unsigned int) {
    return 0;
}

inline char atomic_bittestandreset_x86(volatile unsigned int*, unsigned int) {
    return 0;
}

#elif defined(GEO_USE_ARM_ATOMICS)


typedef GEO::Numeric::uint32 arm_mutex_t;


#ifdef __aarch64__

inline void lock_mutex_arm(volatile arm_mutex_t* lock) {
    while(__sync_lock_test_and_set(lock, 1) != 0);
}

inline void unlock_mutex_arm(volatile arm_mutex_t* lock) {
    __sync_lock_release(lock);
}

inline unsigned int atomic_bitset_arm(volatile unsigned int* ptr, unsigned int bit) {
    return __sync_fetch_and_or(ptr, 1u << bit) & (1u << bit);
}

inline unsigned int atomic_bitreset_arm(volatile unsigned int* ptr, unsigned int bit) {
    return __sync_fetch_and_and(ptr, ~(1u << bit)) & (1u << bit);
}

inline void memory_barrier_arm() {
    // Full memory barrier.
    __sync_synchronize();
}

inline void wait_for_event_arm() {
    /* TODO */    
}

inline void send_event_arm() {
    /* TODO */    
}

#else

inline void lock_mutex_arm(volatile arm_mutex_t* lock) {
    arm_mutex_t tmp;
    __asm__ __volatile__ (
        "1:     ldrex   %0, [%1]     \n" // read lock
        "       cmp     %0, #0       \n" // check if zero
        "       wfene                \n" // wait for event if non-zero
        "       strexeq %0, %2, [%1] \n" // attempt to store new value
        "       cmpeq   %0, #0       \n" // test if store succeeded
        "       bne     1b           \n" // retry if not
        "       dmb                  \n" // memory barrier
        : "=&r" (tmp)
        : "r" (lock), "r" (1)
        : "cc", "memory");
}

inline void unlock_mutex_arm(volatile arm_mutex_t* lock) {
    __asm__ __volatile__ (
        "       dmb              \n" // ensure all previous access are observed
        "       str     %1, [%0] \n" // clear the lock
        "       dsb              \n" // ensure completion of clear lock ...
        "       sev              \n" // ... before sending the event
        :
        : "r" (lock), "r" (0)
        : "cc", "memory");
}

inline unsigned int atomic_bitset_arm(volatile unsigned int* ptr, unsigned int bit) {
    unsigned int tmp;
    unsigned int result;
    unsigned int OK;
    __asm__ __volatile__ (
        "1:     ldrex   %1, [%5]           \n" // result = *ptr
        "       orr     %0, %1, %6, LSL %4 \n" // tmp = result OR (1 << bit)
        "       strex   %2, %0, [%5]       \n" // *ptr = tmp, status in OK
        "       teq     %2, #0             \n" // if !OK then
        "       bne     1b                 \n" //    goto 1:
        "       and     %1, %1, %6, LSL %4 \n" // result = result AND (1 << bit)
        : "=&r" (tmp), "=&r" (result), "=&r" (OK), "+m" (*ptr)
        : "r" (bit), "r" (ptr), "r" (1)
        : "cc"
    );
    return result;
}

inline unsigned int atomic_bitreset_arm(volatile unsigned int* ptr, unsigned int bit) {
    unsigned int tmp;
    unsigned int result;
    unsigned int OK;
    __asm__ __volatile__ (
        "1:     ldrex   %1, [%5]           \n" // result = *ptr
        "       bic     %0, %1, %6, LSL %4 \n" // tmp = result AND NOT(1 << bit)
        "       strex   %2, %0, [%5]       \n" // *ptr = tmp, status in OK
        "       teq     %2, #0             \n" // if !OK then
        "       bne     1b                 \n" //    goto 1:
        "       and     %1, %1, %6, LSL %4 \n" // result = result AND (1 << bit)
        : "=&r" (tmp), "=&r" (result), "=&r" (OK), "+m" (*ptr)
        : "r" (bit), "r" (ptr), "r" (1)
        : "cc"
    );
    return result;
}

inline void memory_barrier_arm() {
    __asm__ __volatile__ (
        "dmb \n"
        : : : "memory"
    );
}

inline void wait_for_event_arm() {
    __asm__ __volatile__ (
        "wfe \n"
        : : : 
    );
}

inline void send_event_arm() {
    __asm__ __volatile__ (
        "dsb \n" // ensure completion of store operations
        "sev \n"
        : : : 
    );
}

#endif

#elif defined(GEO_USE_X86_ATOMICS)

#  define GEO_USE_X86_PAUSE

#  ifdef GEO_USE_X86_PAUSE

inline void geo_pause() {
    __asm__ __volatile__ (
        "pause;\n"
    );
}

#  else
#    ifdef __ICC
#      define geo_pause _mm_pause
#    else
#      define geo_pause __builtin_ia32_pause
#    endif

#  endif

inline char atomic_bittestandset_x86(volatile unsigned int* ptr, unsigned int bit) {
    char out;
#if defined(__x86_64)
    __asm__ __volatile__ (
        "lock; bts %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then set bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=r" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#else
    __asm__ __volatile__ (
        "lock; bts %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then set bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=q" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#endif
    return out;
}

inline char atomic_bittestandreset_x86(volatile unsigned int* ptr, unsigned int bit) {
    char out;
#if defined(__x86_64)
    __asm__ __volatile__ (
        "lock; btr %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then reset bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=r" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#else
    __asm__ __volatile__ (
        "lock; btr %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then reset bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=q" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#endif
    return out;
}

#elif defined(GEO_OS_APPLE)

#include <libkern/OSAtomic.h>

#elif defined(GEO_OS_WINDOWS)

#include <windows.h>
#include <intrin.h>
#pragma intrinsic(_InterlockedCompareExchange8)
#pragma intrinsic(_InterlockedCompareExchange16)
#pragma intrinsic(_InterlockedCompareExchange)
#pragma intrinsic(_interlockedbittestandset)
#pragma intrinsic(_interlockedbittestandreset)
#pragma intrinsic(_ReadBarrier)
#pragma intrinsic(_WriteBarrier)
#pragma intrinsic(_ReadWriteBarrier)

#endif // GEO_OS_WINDOWS

#endif


/******* extracted from ../basic/thread_sync.h *******/

#ifndef GEOGRAM_BASIC_THREAD_SYNC
#define GEOGRAM_BASIC_THREAD_SYNC

#include <vector>

#ifdef GEO_OS_APPLE
# define GEO_USE_DEFAULT_SPINLOCK_ARRAY
# include <AvailabilityMacros.h>
# if defined(MAC_OS_X_VERSION_10_12) && MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_12
#   include <os/lock.h>
# endif
#endif

#ifdef geo_debug_assert
#define geo_thread_sync_assert(x) geo_debug_assert(x)
#else
#define geo_thread_sync_assert(x) 
#endif


namespace GEO {

    namespace Process {

#if defined(GEO_OS_ANDROID) || defined(GEO_OS_RASPBERRY)

        
        typedef arm_mutex_t spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(spinlock& x) {
            lock_mutex_arm(&x);
        }

        inline void release_spinlock(spinlock& x) {
            unlock_mutex_arm(&x);
        }

#elif defined(GEO_OS_LINUX) && !defined(GEO_OS_RASPBERRY)

        
        typedef unsigned char spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(volatile spinlock& x) {
            while(__sync_lock_test_and_set(&x, 1) == 1) {
                // Intel recommends to have a PAUSE asm instruction
                // in the spinlock loop.
                geo_pause();
            }
        }

        inline void release_spinlock(volatile spinlock& x) {
            // Note: Since on intel processor, memory writes
            // (of data types <= bus size) are atomic, we could
            // simply write 'x=0' instead, but this would
            // lack the 'memory barrier with release semantics'
            // required to avoid compiler and/or processor
            // reordering (important for relaxed memory
            // models such as Itanium processors).
            __sync_lock_release(&x);
        }

#elif defined(GEO_OS_APPLE)

#if defined(MAC_OS_X_VERSION_10_12) && MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_12
        
        typedef os_unfair_lock spinlock;
        
        
#       define GEOGRAM_SPINLOCK_INIT OS_UNFAIR_LOCK_INIT
        //inline void init_spinlock(spinlock & s) { s = OS_UNFAIR_LOCK_INIT; }
        //inline bool try_acquire_spinlock (spinlock & s) { return os_unfair_lock_trylock(&s); }
        inline void acquire_spinlock    (spinlock & s) { os_unfair_lock_lock(&s); }
        inline void release_spinlock  (spinlock & s) { os_unfair_lock_unlock(&s); }
#else
        
        typedef OSSpinLock spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT OS_SPINLOCK_INIT
        inline void acquire_spinlock(volatile spinlock& x) {
            OSSpinLockLock(&x);
        }

        inline void release_spinlock(volatile spinlock& x) {
            OSSpinLockUnlock(&x);
        }
#endif // __MAC_10_12

#elif defined(GEO_OS_WINDOWS)

        
        typedef short spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(volatile spinlock& x) {
            while(_InterlockedCompareExchange16(&x, 1, 0) == 1) {
                // Intel recommends to have a PAUSE asm instruction
                // in the spinlock loop. Under MSVC/Windows,
                // YieldProcessor() is a macro that calls the
                // (undocumented) _mm_pause() intrinsic function
                // that generates a PAUSE opcode.
                YieldProcessor();
            }
            // We do not need _ReadBarrier() here since
            // _InterlockedCompareExchange16
            // "acts as a full barrier in VC2005" according to the doc
        }

        inline void release_spinlock(volatile spinlock& x) {
            _WriteBarrier();   // prevents compiler reordering
            x = 0;
        }

#endif

#if defined(GEO_USE_DEFAULT_SPINLOCK_ARRAY)

        // TODO: implement memory-efficient version for
        // MacOSX MacOSX does have atomic bit
        // manipulation routines (OSAtomicTestAndSet()),
        // and also  has OSAtomicAnd32OrigBarrier() and
        // OSAtomicOr32OrigBarrier() functions that can
        // be used instead (Orig for 'return previous value'
        // and Barrier for ('include a memory barrier').
        // Android needs additional routines in atomics.h/atomics.cpp

        class SpinLockArray {
        public:
            SpinLockArray() {
            }

            SpinLockArray(index_t size_in) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                spinlocks_.assign(size_in, GEOGRAM_SPINLOCK_INIT);
            }

            void clear() {
                spinlocks_.clear();
            }

            index_t size() const {
                return index_t(spinlocks_.size());
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                GEO::Process::acquire_spinlock(spinlocks_[i]);
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                GEO::Process::release_spinlock(spinlocks_[i]);
            }

        private:
            std::vector<spinlock> spinlocks_;
        };

#elif defined(GEO_OS_ANDROID) || defined(GEO_OS_RASPBERRY)

        class SpinLockArray {
        public:
            typedef Numeric::uint32 word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                word_t b = word_t(i & 31);
                // Loop while previously stored value has its bit set.
                while((atomic_bitset_arm(&spinlocks_[w], b)) != 0) {
                    // If somebody else has the lock, sleep.
                    //  It is important to sleep here, else atomic_bitset_arm()
                    // keeps acquiring the exclusive monitor (even for testing)
                    // and this slows down everything.
                    wait_for_event_arm();
                }
                memory_barrier_arm();
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                memory_barrier_arm();
                index_t w = i >> 5;
                word_t b = word_t(i & 31);
                atomic_bitreset_arm(&spinlocks_[w], b);
                //   Now wake up the other threads that started
                // sleeping if they did not manage to acquire
                // the lock.
                send_event_arm();
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };

#elif defined(GEO_OS_LINUX) && !defined(GEO_OS_RASPBERRY)

        class SpinLockArray {
        public:
            typedef Numeric::uint32 word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                while(atomic_bittestandset_x86(&spinlocks_[w], b)) {
                    // Intel recommends to have a PAUSE asm instruction
                    // in the spinlock loop. It is generated using the
                    // following intrinsic function of GCC.
                    geo_pause();
                }
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                // Note: we need here to use a synchronized bit reset
                // since &= is not atomic.
                atomic_bittestandreset_x86(&spinlocks_[w], b);
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };

#elif defined(GEO_OS_WINDOWS)
        class SpinLockArray {
        public:
            typedef LONG word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                while(_interlockedbittestandset(&spinlocks_[w], b)) {
                    // Intel recommends to have a PAUSE asm instruction
                    // in the spinlock loop. Under MSVC/Windows,
                    // YieldProcessor() is a macro that calls the
                    // (undocumented) _mm_pause() intrinsic function
                    // that generates a PAUSE opcode.
                    YieldProcessor();
                }
                // We do not need here _ReadBarrier() since
                // _interlockedbittestandset
                // "acts as a full barrier in VC2005" according to the doc
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                // Note1: we need here to use a synchronized bit reset
                // since |= is not atomic.
                // Note2: We do not need here _WriteBarrier() since
                // _interlockedbittestandreset
                // "acts as a full barrier in VC2005" according to the doc
                _interlockedbittestandreset(&spinlocks_[w], b);
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };

#else

#error Found no implementation of SpinLockArray

#endif


    }
}

#endif


/******* extracted from ../basic/packed_arrays.h *******/

#ifndef GEOGRAM_BASIC_PACKED_ARRAYS
#define GEOGRAM_BASIC_PACKED_ARRAYS



namespace GEO {

    class GEOGRAM_API PackedArrays {
    public:
        PackedArrays();

        ~PackedArrays();

        bool thread_safe() const {
            return thread_safe_;
        }

        void set_thread_safe(bool flag);

        void init(
            index_t nb_arrays,
            index_t Z1_block_size,
            bool static_mode = false
        );

        void clear();

        index_t nb_arrays() const {
            return nb_arrays_;
        }

        index_t array_size(index_t array_index) const {
            geo_debug_assert(array_index < nb_arrays_);
            return Z1_[array_index * Z1_stride_];
        }

        void get_array(
            index_t array_index, vector<index_t>& array, bool lock = true
        ) const {
            if(lock) {
                lock_array(array_index);
            }
            array.resize(array_size(array_index));
            if(array.size() != 0) {
                get_array(array_index, &array[0], false);
            }
            if(lock) {
                unlock_array(array_index);
            }
        }

        void get_array(
            index_t array_index, index_t* array, bool lock = true
        ) const;

        void set_array(
            index_t array_index,
            index_t array_size, const index_t* array_elements,
            bool lock = true
        );

        void set_array(
            index_t array_index,
            const vector<index_t>& array,
            bool lock = true
        ) {
            if(array.size() == 0) {
                set_array(array_index, 0, nil, lock);
            } else {
                set_array(
                    array_index, index_t(array.size()), &array[0], lock
                );
            }
        }

        void resize_array(
            index_t array_index, index_t array_size, bool lock
        );

        void lock_array(index_t array_index) const {
            if(thread_safe_) {
                Z1_spinlocks_.acquire_spinlock(array_index);
            }
        }

        void unlock_array(index_t array_index) const {
            if(thread_safe_) {
                Z1_spinlocks_.release_spinlock(array_index);
            }
        }

        void show_stats();

    protected:
        bool static_mode() const {
            return ZV_ == nil;
        }

    private:
        
        PackedArrays(const PackedArrays& rhs);

        
        PackedArrays& operator= (const PackedArrays& rhs);

    private:
        index_t nb_arrays_;
        index_t Z1_block_size_;
        index_t Z1_stride_;
        index_t* Z1_;
        index_t** ZV_;
        bool thread_safe_;
        mutable Process::SpinLockArray Z1_spinlocks_;
    };
}

#endif


/******* extracted from ../basic/progress.h *******/

#ifndef GEOGRAM_BASIC_PROGRESS
#define GEOGRAM_BASIC_PROGRESS



namespace GEO {

    class GEOGRAM_API ProgressClient : public Counted {
    public:
        virtual void begin() = 0;

        virtual void progress(index_t step, index_t percent) = 0;

        virtual void end(bool canceled) = 0;

    protected:
        
        virtual ~ProgressClient();
    };

    
    typedef SmartPointer<ProgressClient> ProgressClient_var;

    

    struct GEOGRAM_API TaskCanceled : std::exception {
        virtual const char* what() const GEO_NOEXCEPT;
    };

    

    class ProgressTask;

    namespace Progress {
        void GEOGRAM_API initialize();

        void GEOGRAM_API terminate();

        void GEOGRAM_API set_client(ProgressClient* client);

        GEOGRAM_API const ProgressTask* current_task();

        void GEOGRAM_API cancel();

        bool GEOGRAM_API is_canceled();

        void GEOGRAM_API clear_canceled();
    }

    

    class GEOGRAM_API ProgressTask {
    public:
        ProgressTask(
            const std::string& task_name, index_t max_steps,
            bool quiet 
        );

        ProgressTask(
            const std::string& task_name = "", index_t max_steps = 100
        );

        virtual ~ProgressTask();

        virtual void progress(index_t step);

        virtual void next();

        bool is_canceled() const;

        void reset();

        void reset(index_t max_steps);

        const std::string& task_name() const {
            return task_name_;
        }

        double start_time() const {
            return start_time_;
        }

        index_t max_steps() const {
            return max_steps_;
        }

        index_t step() const {
            return step_;
        }

        index_t percent() const {
            return percent_;
        }

    protected:
        virtual void update();

    private:
        std::string task_name_;
        double start_time_;
        bool quiet_;
        index_t max_steps_;
        index_t step_;
        index_t percent_;
    };
}

#endif


/******* extracted from ../basic/process.h *******/

#ifndef GEOGRAM_BASIC_PROCESS
#define GEOGRAM_BASIC_PROCESS

#include <functional>


namespace GEO {


#if defined(GEO_COMPILER_GCC) || defined(GEO_COMPILER_CLANG)
#define GEO_THREAD_LOCAL __thread
#elif defined(GEO_COMPILER_MSVC) || defined(GEO_COMPILER_INTEL)
#define GEO_THREAD_LOCAL __declspec(thread)
#elif defined(GEO_COMPILER_EMSCRIPTEN)
#define GEO_THREAD_LOCAL __thread    
#else
#error "Unknown compiler"
#endif


    class GEOGRAM_API Thread : public Counted {
    public:

        Thread() : id_(0) {
        }

        virtual void run() = 0;

        index_t id() const {
            return id_;
        }

        static Thread* current();

    protected:
        
        virtual ~Thread();


    private:
        void set_id(index_t id_in) {
            id_ = id_in;
        }

        static void set_current(Thread* thread);

        index_t id_;

        // ThreadManager needs to access set_current() and 
        // set_id().
        friend class ThreadManager;
    };

    
    typedef SmartPointer<Thread> Thread_var;

    typedef std::vector<Thread_var> ThreadGroup;

    template <class THREAD>
    class TypedThreadGroup : public ThreadGroup {
    public:
        TypedThreadGroup() {
        }

        THREAD* operator[] (index_t i) {
            geo_debug_assert(i < size());
            Thread* result = ThreadGroup::operator[] (i);
            return static_cast<THREAD*>(result);
        }
    };

    class GEOGRAM_API ThreadManager : public Counted {
    public:
        virtual void run_threads(ThreadGroup& threads);

        virtual index_t maximum_concurrent_threads() = 0;

        virtual void enter_critical_section() = 0;

        virtual void leave_critical_section() = 0;

    protected:
        virtual void run_concurrent_threads(
            ThreadGroup& threads, index_t max_threads
        ) = 0;


        static void set_thread_id(Thread* thread, index_t id) {
            thread->set_id(id);
        }

        static void set_current_thread(Thread* thread) {
            Thread::set_current(thread);
        }

        
        virtual ~ThreadManager();
    };

    
    typedef SmartPointer<ThreadManager> ThreadManager_var;

    class GEOGRAM_API MonoThreadingThreadManager : public ThreadManager {
    public:
        virtual index_t maximum_concurrent_threads();

        virtual void enter_critical_section();

        virtual void leave_critical_section();

    protected:
        
        virtual ~MonoThreadingThreadManager();

        virtual void run_concurrent_threads(
            ThreadGroup& threads, index_t max_threads
        );
    };

    namespace Process {

        void GEOGRAM_API initialize();

        void GEOGRAM_API terminate();

        void GEOGRAM_API show_stats();
        
        void GEOGRAM_API brute_force_kill();

        index_t GEOGRAM_API maximum_concurrent_threads();

        void GEOGRAM_API run_threads(ThreadGroup& threads);

        void GEOGRAM_API enter_critical_section();

        void GEOGRAM_API leave_critical_section();

        index_t GEOGRAM_API number_of_cores();

        void GEOGRAM_API set_thread_manager(ThreadManager* thread_manager);

        bool GEOGRAM_API is_running_threads();

        void GEOGRAM_API enable_FPE(bool flag);

        bool GEOGRAM_API FPE_enabled();

        void GEOGRAM_API enable_multithreading(bool flag);

        bool GEOGRAM_API multithreading_enabled();

        void GEOGRAM_API set_max_threads(index_t num_threads);

        index_t GEOGRAM_API max_threads();

        void GEOGRAM_API enable_cancel(bool flag);

        bool GEOGRAM_API cancel_enabled();

        size_t GEOGRAM_API used_memory();

        size_t GEOGRAM_API max_used_memory();


        std::string GEOGRAM_API executable_filename();
    }

    template <class Func>
    class ParallelForThread : public Thread {
    public:
        ParallelForThread(
            const Func& func, index_t from, index_t to, index_t step = 1
        ) :
            func_(func),
            from_(from),
            to_(to),
            step_(step) {
        }

        virtual void run() {
            for(index_t i = from_; i < to_; i += step_) {
                const_cast<Func&> (func_)(i);
            }
        }

    protected:
        
        virtual ~ParallelForThread() {
        }

    private:
        const Func& func_;
        index_t from_;
        index_t to_;
        index_t step_;
    };

    template <class Func>
    inline void parallel_for(
        const Func& func, index_t from, index_t to,
        index_t threads_per_core = 1,
        bool interleaved = false
    ) {
#ifdef GEO_OS_WINDOWS
        // TODO: This is a limitation of WindowsThreadManager, to be fixed.
        threads_per_core = 1;
#endif

        index_t nb_threads = geo_min(
            to - from,
            Process::maximum_concurrent_threads() * threads_per_core
        );

	nb_threads = geo_max(1u, nb_threads);
	
        index_t batch_size = (to - from) / nb_threads;
        if(Process::is_running_threads() || nb_threads == 1) {
            for(index_t i = from; i < to; i++) {
                const_cast<Func&> (func)(i);
            }
        } else {
            ThreadGroup threads;
            if(interleaved) {
                for(index_t i = 0; i < nb_threads; i++) {
                    threads.push_back(
                        new ParallelForThread<Func>(
                            func, from + i, to, nb_threads
                        )
                    );
                }
            } else {
                index_t cur = from;
                for(index_t i = 0; i < nb_threads; i++) {
                    if(i == nb_threads - 1) {
                        threads.push_back(
                            new ParallelForThread<Func>(
                                func, cur, to
                            )
                        );
                    } else {
                        threads.push_back(
                            new ParallelForThread<Func>(
                                func, cur, cur + batch_size
                            )
                        );
                    }
                    cur += batch_size;
                }
            }
            Process::run_threads(threads);
        }
    }

    template <class T> class ParallelForMemberCallback {
    public:
        typedef void (T::*fptr)(index_t);

        ParallelForMemberCallback(T* object, fptr f) :
            object_(object), f_(f) {
        }

        void operator()(index_t i) {
            (*object_.*f_)(i);
        }
    private:
        T* object_;
        fptr f_;
    };
    

    template <class T> ParallelForMemberCallback<T>
    parallel_for_member_callback(T* obj, void (T::* fun)(index_t)) {
        return ParallelForMemberCallback<T>(obj, fun);
    }
    
}

#endif


/******* extracted from ../basic/psm.h *******/

#ifndef GEOGRAM_BASIC_PSM
#define GEOGRAM_BASIC_PSM


#include <assert.h>
#include <iostream>
#include <string>

#ifndef GEOGRAM_PSM
#define GEOGRAM_PSM
#endif

#ifndef GEOGRAM_BASIC_ASSERT

#define geo_assert(x) assert(x)
#define geo_range_assert(x, min_val, max_val) \
    assert((x) >= (min_val) && (x) <= (max_val))
#define geo_assert_not_reached assert(0)

#ifdef GEO_DEBUG
#define geo_debug_assert(x) assert(x)
#define geo_debug_range_assert(x, min_val, max_val) \
    assert((x) >= (min_val) && (x) <= (max_val))
#else
#define geo_debug_assert(x) 
#define geo_debug_range_assert(x, min_val, max_val)
#endif

#ifdef GEO_PARANOID
#define geo_parano_assert(x) geo_assert(x)
#define geo_parano_range_assert(x, min_val, max_val) \
    geo_range_assert(x, min_val, max_val)
#else
#define geo_parano_assert(x)
#define geo_parano_range_assert(x, min_val, max_val)
#endif

#endif

#ifndef geo_cite
#define geo_cite(x)
#endif

#ifndef geo_cite_with_info
#define geo_cite_with_info(x,y)
#endif

#ifndef GEOGRAM_BASIC_LOGGER

namespace GEO {
    namespace Logger {
        inline std::ostream& out(const std::string& name) {
            return std::cout << " [" << name << "]";
        }

        inline std::ostream& err(const std::string& name) {
            return std::cerr << "E[" << name << "]";
        }

        inline std::ostream& warn(const std::string& name) {
            return std::cerr << "W[" << name << "]";
        }
    }
    
}

#endif

#ifndef FPG_UNCERTAIN_VALUE
#define FPG_UNCERTAIN_VALUE 0
#endif



#ifndef GEOGRAM_BASIC_THREAD_SYNC
#define GEOGRAM_SPINLOCK_INIT 0

namespace GEO {
    namespace Process {
    
        typedef int spinlock;
        
        inline void acquire_spinlock(spinlock& x) {
            // Not implemented yet for PSMs
            geo_argused(x);
            geo_assert_not_reached;
        }
    
        inline void release_spinlock(spinlock& x) {
            // Not implemented yet for PSMs
            geo_argused(x); 
            geo_assert_not_reached;       
        }
    }
}
#endif

#define GEOGRAM_WITH_PDEL

#endif

/******* extracted from ../basic/factory.h *******/

#ifndef GEOGRAM_BASIC_FACTORY
#define GEOGRAM_BASIC_FACTORY

#include <string>
#include <map>
#include <vector>
#include <typeinfo>


namespace GEO {

    class GEOGRAM_API InstanceRepo {
    public:
        typedef Counted Instance;

        template <class InstanceType>
        static InstanceType& instance() {
            const std::string name = typeid(InstanceType).name();
            Instance* instance = get(name);
            if(instance == nil) {
                instance = new InstanceType;
                add(name, instance);
            }
            return *static_cast<InstanceType*>(instance);
        }

    private:
        static void add(const std::string& name, Instance* instance);

        static Instance* get(const std::string& name);
    };

    

    template <class FactoryCreator>
    class Factory : public InstanceRepo::Instance {
    public:
        typedef typename FactoryCreator::CreatorType CreatorType;

        template <class ConcreteType>
        static void register_creator(const std::string& name) {
            Factory& self = instance();
            self.registry_[name] =
                FactoryCreator::template create<ConcreteType>;
        }

        static CreatorType find_creator(const std::string& name) {
            Factory& self = instance();
            typename Registry::const_iterator i = self.registry_.find(name);
            return i == self.registry_.end() ? nil : i->second;
        }

        static void list_creators(std::vector<std::string>& names) {
            Factory& self = instance();
            typename Registry::const_iterator i;
            for(i = self.registry_.begin(); i != self.registry_.end(); ++i) {
                names.push_back(i->first);
            }
        }

        static bool has_creator(const std::string& name) {
            Factory& self = instance();
            typename Registry::const_iterator i;
            for(i = self.registry_.begin(); i != self.registry_.end(); ++i) {
                if(i->first == name) {
                    return true;
                }
            }
            return false;
        }

        template <class ConcreteType>
        struct RegisterCreator {
            RegisterCreator(const std::string& name) {
                Factory::template register_creator<ConcreteType>(name);
            }
        };

    protected:
        virtual ~Factory() {
        }

    private:
        static inline Factory& instance() {
            return InstanceRepo::instance<Factory>();
        }

        typedef std::map<std::string, CreatorType> Registry;
        Registry registry_;
    };

    template <class Type>
    struct FactoryCreator0 {
        typedef Type* (* CreatorType)();

        template <class ConcreteType>
        static Type* create() {
            return new ConcreteType;
        }
    };

    template <class Type>
    class Factory0 : public Factory<FactoryCreator0<Type> > {
        typedef Factory<FactoryCreator0<Type> > BaseClass;

    public:
        static Type* create_object(const std::string& name) {
            typename BaseClass::CreatorType creator =
                BaseClass::find_creator(name);
            return creator == nil ? nil : (* creator)();
        }
    };

    template <class Type, class Param1>
    struct FactoryCreator1 {
        typedef Type* (* CreatorType)(const Param1&);

        template <class ConcreteType>
        static Type* create(const Param1& param1) {
            return new ConcreteType(param1);
        }
    };

    template <class Type, class Param1>
    class Factory1 : public Factory<FactoryCreator1<Type, Param1> > {
        typedef Factory<FactoryCreator1<Type, Param1> > BaseClass;

    public:
        static Type* create_object(const std::string& name, const Param1& param1) {
            typename BaseClass::CreatorType creator =
                BaseClass::find_creator(name);
            return creator == nil ? nil : (* creator)(param1);
        }
    };

#define geo_register_creator(FactoryType, ConcreteType, name) \
    static FactoryType::RegisterCreator<ConcreteType> \
    CPP_CONCAT(Factory_register_creator_, __LINE__) (name); \
    geo_argused(CPP_CONCAT(Factory_register_creator_, __LINE__))
}

#endif


/******* extracted from ../basic/vecg.h *******/

#ifndef GEOGRAM_BASIC_VECG
#define GEOGRAM_BASIC_VECG


#include <iostream>
#include <cfloat>
#include <cmath>


namespace GEO {

    template <index_t DIM, class T>
    class vecng {
    public:
        
        static const index_t dim = DIM;

        
        typedef vecng<DIM, T> vector_type;

        
        typedef T value_type;

        vecng() {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] = T(0);
            }
        }

        vecng(const vecng<DIM,T>& rhs) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] = rhs.data_[i];
            }
        }
        
        // This one should never be called :
        // a template constructor cannot be a copy constructor

        template <class T2>
        explicit vecng(const vecng<DIM, T2>& v) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] = T(v[i]);
            }
        }

        // to avoid compilation problems
        template <class T2, index_t DIM2>
        explicit vecng(
            const vecng<DIM2, T2>& v
        ) {
            geo_debug_assert(DIM2 == DIM);
            for(index_t i = 0; i < DIM; i++) {
                data_[i] = T(v[i]);
            }
        }

        template <class T2>
        explicit vecng(const T2* v) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] = T(v[i]);
            }
        }

        vector_type& operator= (const vector_type& v) {
            memcpy(data_, v.data(), DIM * sizeof(T));
            return *this;
        }

        index_t dimension() const {
            return DIM;
        }

        T* data() {
            return data_;
        }

        const T* data() const {
            return data_;
        }

        inline T& operator[] (index_t i) {
            geo_debug_assert(i < DIM);
            return data()[i];
        }

        inline const T& operator[] (index_t i) const {
            geo_debug_assert(i < DIM);
            return data()[i];
        }

        inline T length2() const {
            T result = T(0);
            for(index_t i = 0; i < DIM; i++) {
                result += data_[i] * data_[i];
            }
            return result;
        }

        inline T length() const {
            return sqrt(length2());
        }

        inline T distance2(const vector_type& v) const {
            T result(0);
            for(index_t i = 0; i < DIM; i++) {
                result += geo_sqr(v.data_[i] - data_[i]);
            }
            return result;
        }

        inline T distance(const vector_type& v) const {
            return sqrt(distance2(v));
        }

        // operators

        inline vector_type& operator+= (const vector_type& v) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] += v.data_[i];
            }
            return *this;
        }

        inline vector_type& operator-= (const vector_type& v) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] -= v.data_[i];
            }
            return *this;
        }

        template <class T2>
        inline vector_type& operator*= (T2 s) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] *= T(s);
            }
            return *this;
        }

        template <class T2>
        inline vector_type& operator/= (T2 s) {
            for(index_t i = 0; i < DIM; i++) {
                data_[i] /= T(s);
            }
            return *this;
        }

        inline vector_type operator+ (const vector_type& v) const {
            vector_type result(*this);
            for(index_t i = 0; i < DIM; i++) {
                result.data_[i] += v.data_[i];
            }
            return result;
        }

        inline vector_type operator- (const vector_type& v) const {
            vector_type result(*this);
            for(index_t i = 0; i < DIM; i++) {
                result.data_[i] -= v.data_[i];
            }
            return result;
        }

        template <class T2>
        inline vector_type operator* (T2 s) const {
            vector_type result(*this);
            for(index_t i = 0; i < DIM; i++) {
                result.data_[i] *= T(s);
            }
            return result;
        }

        template <class T2>
        inline vector_type operator/ (T2 s) const {
            vector_type result(*this);
            for(index_t i = 0; i < DIM; i++) {
                result.data_[i] /= T(s);
            }
            return result;
        }

        inline vector_type operator- () const {
            vector_type result;
            for(index_t i = 0; i < DIM; i++) {
                result.data_[i] = -data_[i];
            }
            return result;
        }

    private:
        T data_[DIM];
    };

    template <index_t DIM, class T>
    inline T dot(
        const vecng<DIM, T>& v1, const vecng<DIM, T>& v2
    ) {
        T result = 0;
        for(index_t i = 0; i < DIM; i++) {
            result += v1[i] * v2[i];
        }
        return result;
    }

    template <class T2, index_t DIM, class T>
    inline vecng<DIM, T> operator* (
        T2 s, const vecng<DIM, T>& v
    ) {
        vecng<DIM, T> result;
        for(index_t i = 0; i < DIM; i++) {
            result[i] = T(s) * v[i];
        }
        return result;
    }

    // Compatibility with GLSL

    template <index_t DIM, class T>
    inline T length(const vecng<DIM, T>& v) {
        return v.length();
    }

    template <index_t DIM, class T>
    inline T length2(const vecng<DIM, T>& v) {
        return v.length2();
    }

    template <index_t DIM, class T>
    inline T distance2(
        const vecng<DIM, T>& v1, const vecng<DIM, T>& v2
    ) {
        return v2.distance2(v1);
    }

    template <index_t DIM, class T>
    inline T distance(
        const vecng<DIM, T>& v1, const vecng<DIM, T>& v2
    ) {
        return v2.distance(v1);
    }

    template <index_t DIM, class T>
    inline vecng<DIM, T> normalize(
        const vecng<DIM, T>& v
    ) {
        T s = length(v);
        if(s > 1e-30) {
            s = T(1) / s;
        }
        return s * v;
    }

    template <index_t DIM, class T>
    inline vecng<DIM, T> mix(
        const vecng<DIM, T>& v1, const vecng<DIM, T>& v2, T s
    ) {
        return (T(1) - s) * v1 + s * v2;
    }

    

    template <class T>
    class vecng<2, T> {
    public:
        
        static const index_t dim = 2;

        
        typedef vecng<dim, T> vector_type;

        
        typedef T value_type;

        
        vecng() :
            x(0),
            y(0) {
        }

        vecng(T x_in, T y_in) :
            x(x_in),
            y(y_in) {
        }

        
        template <class T2>
        explicit vecng(const vecng<dim, T2>& v) :
            x(v.x),
            y(v.y) {
        }

        
        template <class T2>
        explicit vecng(const T2* v) :
            x(v[0]),
            y(v[1]) {
        }

        
        inline T length2() const {
            return x * x + y * y;
        }

        
        inline T length() const {
            return sqrt(x * x + y * y);
        }

        
        inline T distance2(const vector_type& v) const {
            T dx = v.x - x;
            T dy = v.y - y;
            return dx * dx + dy * dy;
        }

        
        inline T distance(const vector_type& v) const {
            return sqrt(distance2(v));
        }

        
        inline vector_type& operator+= (const vector_type& v) {
            x += v.x;
            y += v.y;
            return *this;
        }

        
        inline vector_type& operator-= (const vector_type& v) {
            x -= v.x;
            y -= v.y;
            return *this;
        }

        
        template <class T2>
        inline vector_type& operator*= (T2 s) {
            x *= T(s);
            y *= T(s);
            return *this;
        }

        
        template <class T2>
        inline vector_type& operator/= (T2 s) {
            x /= T(s);
            y /= T(s);
            return *this;
        }

        
        inline vector_type operator+ (const vector_type& v) const {
            return vector_type(x + v.x, y + v.y);
        }

        
        inline vector_type operator- (const vector_type& v) const {
            return vector_type(x - v.x, y - v.y);
        }

        
        template <class T2>
        inline vector_type operator* (T2 s) const {
            return vector_type(x * T(s), y * T(s));
        }

        
        template <class T2>
        inline vector_type operator/ (T2 s) const {
            return vector_type(x / T(s), y / T(s));
        }

        
        inline vector_type operator- () const {
            return vector_type(-x, -y);
        }

        
        index_t dimension() const {
            return dim;
        }

        
        T* data() {
            return &x;
        }

        
        const T* data() const {
            return &x;
        }

        
        inline T& operator[] (index_t i) {
            geo_debug_assert(i < dim);
            return data()[i];
        }

        
        inline const T& operator[] (index_t i) const {
            geo_debug_assert(i < dim);
            return data()[i];
        }

        
        T x;
        
        T y;
    };

    template <class T>
    inline T dot(
        const vecng<2, T>& v1, const vecng<2, T>& v2
    ) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    template <class T>
    inline T det(
        const vecng<2, T>& v1, const vecng<2, T>& v2
    ) {
        return v1.x * v2.y - v1.y * v2.x;
    }

    template <class T2, class T>
    inline vecng<2, T> operator* (
        T2 s, const vecng<2, T>& v
    ) {
        return vecng<2, T>(T(s) * v.x, T(s) * v.y);
    }

    

    template <class T>
    class vecng<3, T> {
    public:
        
        static const index_t dim = 3;

        
        typedef vecng<dim, T> vector_type;

        
        typedef T value_type;

        
        vecng() :
            x(0),
            y(0),
            z(0) {
        }

        vecng(T x_in, T y_in, T z_in) :
            x(x_in),
            y(y_in),
            z(z_in) {
        }

        
        template <class T2>
        explicit vecng(const vecng<dim, T2>& v) :
            x(v.x),
            y(v.y),
            z(v.z) {
        }

        
        template <class T2>
        explicit vecng(const T2* v) :
            x(v[0]),
            y(v[1]),
            z(v[2]) {
        }

        
        inline T length2() const {
            return x * x + y * y + z * z;
        }

        
        inline T length() const {
            return sqrt(x * x + y * y + z * z);
        }

        
        inline T distance2(const vector_type& v) const {
            T dx = v.x - x;
            T dy = v.y - y;
            T dz = v.z - z;
            return dx * dx + dy * dy + dz * dz;
        }

        
        inline T distance(const vector_type& v) const {
            return sqrt(distance2(v));
        }

        
        inline vector_type& operator+= (const vector_type& v) {
            x += v.x;
            y += v.y;
            z += v.z;
            return *this;
        }

        
        inline vector_type& operator-= (const vector_type& v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            return *this;
        }

        
        template <class T2>
        inline vector_type& operator*= (T2 s) {
            x *= T(s);
            y *= T(s);
            z *= T(s);
            return *this;
        }

        
        template <class T2>
        inline vector_type& operator/= (T2 s) {
            x /= T(s);
            y /= T(s);
            z /= T(s);
            return *this;
        }

        
        inline vector_type operator+ (const vector_type& v) const {
            return vector_type(x + v.x, y + v.y, z + v.z);
        }

        
        inline vector_type operator- (const vector_type& v) const {
            return vector_type(x - v.x, y - v.y, z - v.z);
        }

        
        template <class T2>
        inline vector_type operator* (T2 s) const {
            return vector_type(x * T(s), y * T(s), z * T(s));
        }

        
        template <class T2>
        inline vector_type operator/ (T2 s) const {
            return vector_type(x / T(s), y / T(s), z / T(s));
        }

        
        inline vector_type operator- () const {
            return vector_type(-x, -y, -z);
        }

        
        index_t dimension() const {
            return dim;
        }

        
        T* data() {
            return &x;
        }

        
        const T* data() const {
            return &x;
        }

        
        inline T& operator[] (index_t i) {
            geo_debug_assert(i < dim);
            return data()[i];
        }

        
        inline const T& operator[] (index_t i) const {
            geo_debug_assert(i < dim);
            return data()[i];
        }

        
        T x;
        
        T y;
        
        T z;
    };

    template <class T>
    inline T dot(
        const vecng<3, T>& v1, const vecng<3, T>& v2
    ) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    template <class T>
    inline vecng<3, T> cross(
        const vecng<3, T>& v1, const vecng<3, T>& v2
    ) {
        return vecng<3, T>(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x
        );
    }

    template <class T2, class T>
    inline vecng<3, T> operator* (
        T2 s, const vecng<3, T>& v
    ) {
        return vecng<3, T>(T(s) * v.x, T(s) * v.y, T(s) * v.z);
    }

    

    template <class T>
    class vecng<4, T> {
    public:
        
        static const index_t dim = 4;

        
        typedef vecng<dim, T> vector_type;

        
        typedef T value_type;

        
        vecng() :
            x(0),
            y(0),
            z(0),
            w(0) {
        }

        vecng(T x_in, T y_in, T z_in, T w_in) :
            x(x_in),
            y(y_in),
            z(z_in),
            w(w_in) {
        }

        
        template <class T2>
        explicit vecng(const vecng<dim, T2>& v) :
            x(v.x),
            y(v.y),
            z(v.z),
            w(v.w) {
        }

        
        template <class T2>
        explicit vecng(const T2* v) :
            x(v[0]),
            y(v[1]),
            z(v[2]),
            w(v[3]) {
        }

        
        inline T length2() const {
            return x * x + y * y + z * z + w * w;
        }

        
        inline T length() const {
            return sqrt(x * x + y * y + z * z + w * w);
        }

        
        inline T distance2(const vector_type& v) const {
            T dx = v.x - x;
            T dy = v.y - y;
            T dz = v.z - z;
            T dw = v.w - w;
            return dx * dx + dy * dy + dz * dz + dw * dw;
        }

        
        inline T distance(const vector_type& v) const {
            return sqrt(distance2(v));
        }

        
        index_t dimension() const {
            return dim;
        }

        
        inline vector_type& operator+= (const vector_type& v) {
            x += v.x;
            y += v.y;
            z += v.z;
            w += v.w;
            return *this;
        }

        
        inline vector_type& operator-= (const vector_type& v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            w -= v.w;
            return *this;
        }

        
        template <class T2>
        inline vector_type& operator*= (T2 s) {
            x *= T(s);
            y *= T(s);
            z *= T(s);
            w *= T(s);
            return *this;
        }

        
        template <class T2>
        inline vector_type& operator/= (T2 s) {
            x /= T(s);
            y /= T(s);
            z /= T(s);
            w /= T(s);
            return *this;
        }

        
        inline vector_type operator+ (const vector_type& v) const {
            return vector_type(x + v.x, y + v.y, z + v.z, w + v.w);
        }

        
        inline vector_type operator- (const vector_type& v) const {
            return vector_type(x - v.x, y - v.y, z - v.z, w - v.w);
        }

        
        template <class T2>
        inline vector_type operator* (T2 s) const {
            return vector_type(x * T(s), y * T(s), z * T(s), w * T(s));
        }

        
        template <class T2>
        inline vector_type operator/ (T2 s) const {
            return vector_type(x / T(s), y / T(s), z / T(s), w / T(s));
        }

        
        inline vector_type operator- () const {
            return vector_type(-x, -y, -z, -w);
        }

        
        T* data() {
            return &x;
        }

        
        const T* data() const {
            return &x;
        }

        
        inline T& operator[] (index_t i) {
            geo_debug_assert(i < dim);
            return data()[i];
        }

        
        inline const T& operator[] (index_t i) const {
            geo_debug_assert(i < dim);
            return data()[i];
        }

        
        T x;
        
        T y;
        
        T z;
        
        T w;
    };

    template <class T>
    inline T dot(
        const vecng<4, T>& v1, const vecng<4, T>& v2
    ) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
    }

    template <class T2, class T>
    inline vecng<4, T> operator* (
        T2 s, const vecng<4, T>& v
    ) {
        return vecng<4, T>(T(s) * v.x, T(s) * v.y, T(s) * v.z, T(s) * v.w);
    }

    template <index_t DIM, class T>
    inline std::ostream& operator<< (
        std::ostream& out, const GEO::vecng<DIM, T>& v
    ) {
        const char* sep = "";
        for(index_t i = 0; i < DIM; i++) {
            out << sep << v[i];
            sep = " ";
        }
        return out;
    }

    template <index_t DIM, class T>
    inline std::istream& operator>> (
        std::istream& in, GEO::vecng<DIM, T>& v
    ) {
        for(index_t i = 0; i < DIM; i++) {
            in >> v[i];
        }
        return in;
    }
}

#endif


/******* extracted from ../basic/matrix.h *******/

#ifndef GEOGRAM_BASIC_MATRIX
#define GEOGRAM_BASIC_MATRIX



namespace GEO {

    

    inline double det2x2(
        double a11, double a12,                    
        double a21, double a22
    ) {                                 
        return a11*a22-a12*a21 ;
    }

    inline double det3x3(
        double a11, double a12, double a13,                
        double a21, double a22, double a23,                
        double a31, double a32, double a33
    ) {
    return
         a11*det2x2(a22,a23,a32,a33)   
        -a21*det2x2(a12,a13,a32,a33)   
        +a31*det2x2(a12,a13,a22,a23);
    }   


    inline double det4x4(
        double a11, double a12, double a13, double a14,
        double a21, double a22, double a23, double a24,               
        double a31, double a32, double a33, double a34,  
        double a41, double a42, double a43, double a44  
    ) {
        double m12 = a21*a12 - a11*a22;
        double m13 = a31*a12 - a11*a32;
        double m14 = a41*a12 - a11*a42;
        double m23 = a31*a22 - a21*a32;
        double m24 = a41*a22 - a21*a42;
        double m34 = a41*a32 - a31*a42;

        double m123 = m23*a13 - m13*a23 + m12*a33;
        double m124 = m24*a13 - m14*a23 + m12*a43;
        double m134 = m34*a13 - m14*a33 + m13*a43;
        double m234 = m34*a23 - m24*a33 + m23*a43;
        
        return (m234*a14 - m134*a24 + m124*a34 - m123*a44);
    }   


    template <index_t DIM, class FT>
    class Matrix {
    public:
        
        typedef Matrix<DIM, FT> matrix_type;

        
        typedef FT value_type;

        
        static const index_t dim = DIM;

        inline Matrix() {
            load_identity();
        }

        explicit Matrix(const FT* vals) {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] = *vals;
                    ++vals;
                }
            }
        }
        
        inline index_t dimension() const {
            return DIM;
        }

        inline void load_zero() {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] = FT(0);
                }
            }
        }

        inline void load_identity() {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] = (i == j) ? FT(1) : FT(0);
                }
            }
        }

        inline bool is_identity() const {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    FT rhs = ((i == j) ? FT(1) : FT(0));
                    if(coeff_[i][j] != rhs) {
                        return false;
                    }
                }
            }
            return true;
        }
        
        inline FT& operator() (index_t i, index_t j) {
            geo_debug_assert(i < DIM);
            geo_debug_assert(j < DIM);
            return coeff_[i][j];
        }

        inline const FT& operator() (index_t i, index_t j) const {
            geo_debug_assert(i < DIM);
            geo_debug_assert(j < DIM);
            return coeff_[i][j];
        }

        inline matrix_type& operator+= (const matrix_type& m) {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] += m.coeff_[i][j];
                }
            }
            return *this;
        }

        inline matrix_type& operator-= (const matrix_type& m) {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] -= m.coeff_[i][j];
                }
            }
            return *this;
        }

        inline matrix_type& operator*= (FT val) {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] *= val;
                }
            }
            return *this;
        }

        inline matrix_type& operator/= (FT val) {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    coeff_[i][j] /= val;
                }
            }
            return *this;
        }

        inline matrix_type operator+ (const matrix_type& m) const {
            matrix_type result = *this;
            result += m;
            return result;
        }

        inline matrix_type operator- (const matrix_type& m) const {
            matrix_type result = *this;
            result -= m;
            return result;
        }

        inline matrix_type operator* (FT val) const {
            matrix_type result = *this;
            result *= val;
            return result;
        }

        inline matrix_type operator/ (FT val) const {
            matrix_type result = *this;
            result /= val;
            return result;
        }

        matrix_type operator* (const matrix_type& m) const {
            matrix_type result;
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    result.coeff_[i][j] = FT(0);
                    for(index_t k = 0; k < DIM; k++) {
                        result.coeff_[i][j] += coeff_[i][k] * m.coeff_[k][j];
                    }
                }
            }
            return result;
        }

        matrix_type inverse() const {
            matrix_type result;
            bool invertible = compute_inverse(result);
            geo_assert(invertible);
            return result;
        }


        bool compute_inverse(matrix_type& result) const {
            FT val=FT(0.0), val2=FT(0.0);
            matrix_type tmp = (*this);

            result.load_identity();

            for(index_t i = 0; i != DIM; i++) {
                val = tmp(i, i);                     /* find pivot */
                index_t ind = i;
                for(index_t j = i + 1; j != DIM; j++) {
                    if(fabs(tmp(j, i)) > fabs(val)) {
                        ind = j;
                        val = tmp(j, i);
                    }
                }

                if(ind != i) {
                    for(index_t j = 0; j != DIM; j++) {
                        val2 = result(i, j);
                        result(i, j) = result(ind, j);
                        result(ind, j) = val2;           /* swap columns */
                        val2 = tmp(i, j);
                        tmp(i, j) = tmp(ind, j);
                        tmp(ind, j) = val2;
                    }
                }

                if(val == 0.0) {
                    return false;
                }

                for(index_t j = 0; j != DIM; j++) {
                    tmp(i, j) /= val;
                    result(i, j) /= val;
                }

                for(index_t j = 0; j != DIM; j++) {
                    if(j == i) {
                        continue;                       /* eliminate column */
                    }
                    val = tmp(j, i);
                    for(index_t k = 0; k != DIM; k++) {
                        tmp(j, k) -= tmp(i, k) * val;
                        result(j, k) -= result(i, k) * val;
                    }
                }
            }
            
            return true;
        }

        matrix_type transpose() const {
            matrix_type result;
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j < DIM; j++) {
                    result(i, j) = (* this)(j, i);
                }
            }
            return result;
        }

        

        inline const FT* data() const {
            return &(coeff_[0][0]);
        }

        

        inline FT* data() {
            return &(coeff_[0][0]);
        }

        void get_lower_triangle(FT* store) const {
            for(index_t i = 0; i < DIM; i++) {
                for(index_t j = 0; j <= i; j++) {
                    *store++ = coeff_[i][j];
                }
            }
        }

    private:
        FT coeff_[DIM][DIM];
    };

    

    template <index_t DIM, class FT>
    inline std::ostream& operator<< (
        std::ostream& output, const Matrix<DIM, FT>& m
    ) {
        const char* sep = "";
        for(index_t i = 0; i < DIM; i++) {
            for(index_t j = 0; j < DIM; j++) {
                output << sep << m(i, j);
                sep = " ";
            }
        }
        return output;
    }

    template <index_t DIM, class FT>
    inline std::istream& operator>> (
        std::istream& input, Matrix<DIM, FT>& m
    ) {
        for(index_t i = 0; i < DIM; i++) {
            for(index_t j = 0; j < DIM; j++) {
                input >> m(i, j);
            }
        }
        return input;
    }

    

    template <index_t DIM, class FT> inline
    void mult(const Matrix<DIM, FT>& M, const FT* x, FT* y) {
        for(index_t i = 0; i < DIM; i++) {
            y[i] = 0;
            for(index_t j = 0; j < DIM; j++) {
                y[i] += M(i, j) * x[j];
            }
        }
    }

    

    template <index_t DIM, class FT> inline
    vecng<DIM,FT> operator*(
        const Matrix<DIM, FT>& M, const vecng<DIM,FT>& x
    ) {
        vecng<DIM,FT> y;
        for(index_t i = 0; i < DIM; i++) {
            y[i] = 0;
            for(index_t j = 0; j < DIM; j++) {
                y[i] += M(i, j) * x[j];
            }
        }
        return y;
    }

    
    
}

#endif


/******* extracted from ../basic/geometry.h *******/

#ifndef GEOGRAM_BASIC_GEOMETRY
#define GEOGRAM_BASIC_GEOMETRY



namespace GEO {

    

    typedef vecng<2, Numeric::float64> vec2;

    typedef vecng<3, Numeric::float64> vec3;

    typedef vecng<4, Numeric::float64> vec4;

    typedef vecng<2, Numeric::float32> vec2f;

    typedef vecng<3, Numeric::float32> vec3f;

    typedef vecng<4, Numeric::float32> vec4f;

   
    typedef vecng<2, Numeric::int32> vec2i;

    typedef vecng<3, Numeric::int32> vec3i;

    typedef vecng<4, Numeric::int32> vec4i;
   
   
    typedef Matrix<2, Numeric::float64> mat2;

    typedef Matrix<3, Numeric::float64> mat3;

    typedef Matrix<4, Numeric::float64> mat4;

    

    namespace Geom {

        inline vec3 barycenter(const vec3& p1, const vec3& p2) {
            return vec3(
                0.5 * (p1.x + p2.x),
                0.5 * (p1.y + p2.y),
                0.5 * (p1.z + p2.z)
            );
        }

        inline vec2 barycenter(const vec2& p1, const vec2& p2) {
            return vec2(
                0.5 * (p1.x + p2.x),
                0.5 * (p1.y + p2.y)
            );
        }

        inline vec3 barycenter(
            const vec3& p1, const vec3& p2, const vec3& p3
        ) {
            return vec3(
                (p1.x + p2.x + p3.x) / 3.0,
                (p1.y + p2.y + p3.y) / 3.0,
                (p1.z + p2.z + p3.z) / 3.0
            );
        }

        inline vec2 barycenter(
            const vec2& p1, const vec2& p2, const vec2& p3
        ) {
            return vec2(
                (p1.x + p2.x + p3.x) / 3.0,
                (p1.y + p2.y + p3.y) / 3.0
            );
        }

        inline double cos_angle(const vec3& a, const vec3& b) {
	    double lab = ::sqrt(length2(a)*length2(b));
            double result = (lab > 1e-20) ? (dot(a, b) / lab) : 1.0;
            // Numerical precision problem may occur, and generate
            // normalized dot products that are outside the valid
            // range of acos.
	    geo_clamp(result, -1.0, 1.0);
	    return result;
        }

        inline double angle(const vec3& a, const vec3& b) {
            return ::acos(cos_angle(a, b));
        }

        inline double cos_angle(const vec2& a, const vec2& b) {
	    double lab = ::sqrt(length2(a)*length2(b));
            double result = (lab > 1e-20) ? (dot(a, b) / lab) : 1.0;
            // Numerical precision problem may occur, and generate
            // normalized dot products that are outside the valid
            // range of acos.
	    geo_clamp(result, -1.0, 1.0);
	    return result;
        }

        inline double det(const vec2& a, const vec2& b) {
            return a.x * b.y - a.y * b.x;
        }

        inline double angle(const vec2& a, const vec2& b) {
            return det(a, b) > 0 ?
                   ::acos(cos_angle(a, b)) :
                   -::acos(cos_angle(a, b));
        }

        inline vec3 triangle_normal(
            const vec3& p1, const vec3& p2, const vec3& p3
        ) {
            return cross(p2 - p1, p3 - p1);
        }

	inline double triangle_area_3d(
	    const double* p1, const double* p2, const double* p3
	) {
	    double Ux = p2[0] - p1[0];
	    double Uy = p2[1] - p1[1];
	    double Uz = p2[2] - p1[2];
	    
	    double Vx = p3[0] - p1[0];
	    double Vy = p3[1] - p1[1];
	    double Vz = p3[2] - p1[2];
	    
	    double Nx = Uy*Vz - Uz*Vy;
	    double Ny = Uz*Vx - Ux*Vz;
	    double Nz = Ux*Vy - Uy*Vx;
	    return 0.5 * ::sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	}
	
        inline double triangle_area(
            const vec3& p1, const vec3& p2, const vec3& p3
        ) {
	    return triangle_area_3d(p1.data(), p2.data(), p3.data());
        }

        inline double triangle_signed_area_2d(
            const double* p1, const double* p2, const double* p3
        ) {
	    double a = p2[0]-p1[0];
	    double b = p3[0]-p1[0];
	    double c = p2[1]-p1[1];
	    double d = p3[1]-p1[1];
	    return 0.5*(a*d-b*c);
        }
	
        inline double triangle_signed_area(
            const vec2& p1, const vec2& p2, const vec2& p3
        ) {
            return 0.5 * det(p2 - p1, p3 - p1);
        }

        inline double triangle_area(
            const vec2& p1, const vec2& p2, const vec2& p3
        ) {
            return ::fabs(triangle_signed_area(p1, p2, p3));
        }

        inline double triangle_area_2d(
            const double* p1, const double* p2, const double* p3
        ) {
	    return ::fabs(triangle_signed_area_2d(p1,p2,p3));
	}
	
        vec2 GEOGRAM_API triangle_circumcenter(
            const vec2& p1, const vec2& p2, const vec2& p3
        );

        inline bool has_nan(const vec3& v) {
            return
                Numeric::is_nan(v.x) ||
                Numeric::is_nan(v.y) ||
                Numeric::is_nan(v.z);
        }

        inline bool has_nan(const vec2& v) {
            return
                Numeric::is_nan(v.x) ||
                Numeric::is_nan(v.y);
        }

        vec3 GEOGRAM_API perpendicular(const vec3& V);

        inline double tetra_signed_volume(
            const vec3& p1, const vec3& p2,
            const vec3& p3, const vec3& p4
        ) {
            return dot(p2 - p1, cross(p3 - p1, p4 - p1)) / 6.0;
        }

        inline double tetra_signed_volume(
            const double* p1, const double* p2,
            const double* p3, const double* p4
        ) {
            return tetra_signed_volume(
                *reinterpret_cast<const vec3*>(p1),
                *reinterpret_cast<const vec3*>(p2),
                *reinterpret_cast<const vec3*>(p3),
                *reinterpret_cast<const vec3*>(p4)
            );
        }

        inline double tetra_volume(
            const vec3& p1, const vec3& p2,
            const vec3& p3, const vec3& p4
        ) {
            return ::fabs(tetra_signed_volume(p1, p2, p3, p4));
        }

        vec3 GEOGRAM_API tetra_circum_center(
            const vec3& p1, const vec3& p2,
            const vec3& p3, const vec3& p4
        );

        inline void triangle_centroid(
            const vec3& p, const vec3& q, const vec3& r,
            double a, double b, double c,
            vec3& Vg, double& V
        ) {
            double abc = a + b + c;
            double area = Geom::triangle_area(p, q, r);
            V = area / 3.0 * abc;
            double wp = a + abc;
            double wq = b + abc;
            double wr = c + abc;
            double s = area / 12.0;
            Vg.x = s * (wp * p.x + wq * q.x + wr * r.x);
            Vg.y = s * (wp * p.y + wq * q.y + wr * r.y);
            Vg.z = s * (wp * p.z + wq * q.z + wr * r.z);
        }

        inline double triangle_mass(
            const vec3& p, const vec3& q, const vec3& r,
            double a, double b, double c
        ) {
            return Geom::triangle_area(p, q, r) / 3.0 * (
                sqrt(::fabs(a)) + sqrt(::fabs(b)) + sqrt(::fabs(c))
            );
        }

        inline vec3 random_point_in_triangle(
            const vec3& p1,
            const vec3& p2,
            const vec3& p3
        ) {
            double s = Numeric::random_float64();
            double t = Numeric::random_float64();
            if(s + t > 1) {
                s = 1.0 - s;
                t = 1.0 - t;
            }
            double u = 1.0 - s - t;
            return vec3(
                u * p1.x + s * p2.x + t * p3.x,
                u * p1.y + s * p2.y + t * p3.y,
                u * p1.z + s * p2.z + t * p3.z
            );
        }
    }

    struct Plane {

        Plane(const vec3& p1, const vec3& p2, const vec3& p3) {
            vec3 n = cross(p2 - p1, p3 - p1);
            a = n.x;
            b = n.y;
            c = n.z;
            d = -(a * p1.x + b * p1.y + c * p1.z);
        }

        Plane(const vec3& p, const vec3& n) {
            a = n.x;
            b = n.y;
            c = n.z;
            d = -(a * p.x + b * p.y + c * p.z);
        }

        Plane(
            double a_in, double b_in, double c_in, double d_in
        ) :
            a(a_in),
            b(b_in),
            c(c_in),
            d(d_in) {
        }

        Plane() {
        }

        vec3 normal() const {
            return vec3(a, b, c);
        }

        double a, b, c, d;
    };


    class Box {
    public:
        double xyz_min[3];
        double xyz_max[3];

        bool contains(const vec3& b) const {
            for(coord_index_t c = 0; c < 3; ++c) {
                if(b[c] < xyz_min[c]) {
                    return false;
                }
                if(b[c] > xyz_max[c]) {
                    return false;
                }
            }
            return true;
        }
    };

    inline bool bboxes_overlap(const Box& B1, const Box& B2) {
        for(coord_index_t c = 0; c < 3; ++c) {
            if(B1.xyz_max[c] < B2.xyz_min[c]) {
                return false;
            }
            if(B1.xyz_min[c] > B2.xyz_max[c]) {
                return false;
            }
        }
        return true;
    }

    inline void bbox_union(Box& target, const Box& B1, const Box& B2) {
        for(coord_index_t c = 0; c < 3; ++c) {
            target.xyz_min[c] = geo_min(B1.xyz_min[c], B2.xyz_min[c]);
            target.xyz_max[c] = geo_max(B1.xyz_max[c], B2.xyz_max[c]);
        }
    }
}

#endif


/******* extracted from ../numerics/predicates.h *******/

#ifndef GEOGRAM_NUMERICS_PREDICATES
#define GEOGRAM_NUMERICS_PREDICATES



namespace GEO {

    namespace PCK {

        Sign GEOGRAM_API side1_SOS(
            const double* p0, const double* p1,
            const double* q0,
            coord_index_t DIM
        );

        Sign GEOGRAM_API side2_SOS(
            const double* p0, const double* p1, const double* p2,
            const double* q0, const double* q1,
            coord_index_t DIM
        );

        Sign GEOGRAM_API side3_SOS(
            const double* p0, const double* p1, 
            const double* p2, const double* p3,
            const double* q0, const double* q1, const double* q2,
            coord_index_t DIM
        );

        Sign GEOGRAM_API side3_3dlifted_SOS(
            const double* p0, const double* p1, 
            const double* p2, const double* p3,
            double h0, double h1, double h2, double h3,
            const double* q0, const double* q1, const double* q2
        );
        
        Sign GEOGRAM_API side4_SOS(
            const double* p0,
            const double* p1, const double* p2,
            const double* p3, const double* p4,
            const double* q0, const double* q1,
            const double* q2, const double* q3,
            coord_index_t DIM
        );


        Sign GEOGRAM_API side4_3d(
            const double* p0,
            const double* p1, const double* p2,
            const double* p3, const double* p4
        );

        Sign GEOGRAM_API side4_3d_SOS(
            const double* p0, const double* p1, 
            const double* p2, const double* p3, const double* p4
        );
       
         Sign GEOGRAM_API in_sphere_3d_SOS(
            const double* p0, const double* p1, 
            const double* p2, const double* p3,
            const double* p4
         );


         Sign GEOGRAM_API in_circle_2d_SOS(
             const double* p0, const double* p1, const double* p2,
             const double* p3
         );

	 
         Sign GEOGRAM_API in_circle_3d_SOS(
             const double* p0, const double* p1, const double* p2,
             const double* p3
         );


        Sign GEOGRAM_API in_circle_3dlifted_SOS(
            const double* p0, const double* p1, const double* p2,
            const double* p3,
            double h0, double h1, double h2, double h3
        );
        
        Sign GEOGRAM_API orient_2d(
            const double* p0, const double* p1, const double* p2
        );


#ifndef GEOGRAM_PSM        
        inline Sign orient_2d(
            const vec2& p0, const vec2& p1, const vec2& p2
        ) {
            return orient_2d(p0.data(),p1.data(),p2.data());
        }
#endif

	
        Sign GEOGRAM_API orient_2dlifted_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3, 
            double h0, double h1, double h2, double h3
        );
	
        
        Sign GEOGRAM_API orient_3d(
            const double* p0, const double* p1,
            const double* p2, const double* p3
        );


#ifndef GEOGRAM_PSM        
        inline Sign orient_3d(
            const vec3& p0, const vec3& p1,
            const vec3& p2, const vec3& p3
        ) {
            return orient_3d(p0.data(),p1.data(),p2.data(),p3.data());
        }
#endif
        
        Sign GEOGRAM_API orient_3dlifted(
            const double* p0, const double* p1,
            const double* p2, const double* p3, const double* p4,
            double h0, double h1, double h2, double h3, double h4
        );


        Sign GEOGRAM_API orient_3dlifted_SOS(
            const double* p0, const double* p1,
            const double* p2, const double* p3, const double* p4,
            double h0, double h1, double h2, double h3, double h4
        );


	Sign GEOGRAM_API det_3d(
	    const double* p0, const double* p1, const double* p2
	);

	bool GEOGRAM_API aligned_3d(
	    const double* p0, const double* p1, const double* p2
	);
	
	Sign GEOGRAM_API dot_3d(
	    const double* p0, const double* p1, const double* p2
	);

	
        void GEOGRAM_API show_stats();

        void GEOGRAM_API initialize();

        void GEOGRAM_API terminate();
    }
}

#endif


/******* extracted from ../basic/string.h *******/

#ifndef GEOGRAM_BASIC_STRING
#define GEOGRAM_BASIC_STRING


#include <string>
#include <sstream>
#include <stdexcept>
#include <iomanip>

#include <vector>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <limits.h>


namespace GEO {

    /*
     * \brief String manipulation utilities.
     */
    namespace String {

        void GEOGRAM_API split_string(
            const std::string& in,
            char separator,
            std::vector<std::string>& out,
            bool skip_empty_fields = true
        );

        bool GEOGRAM_API split_string(
            const std::string& in,
            char separator,
            std::string& left,
            std::string& right
        );
        
        std::string GEOGRAM_API join_strings(
            const std::vector<std::string>& in,
            char separator
        );

        std::string GEOGRAM_API join_strings(
            const std::vector<std::string>& in,
            const std::string& separator
        );

        std::string GEOGRAM_API to_lowercase(const std::string& s);

        std::string GEOGRAM_API to_uppercase(const std::string& s);

        inline std::string char_to_string(char c) {
            char s[2];
            s[0] = c;
            s[1] = '\0';
            return std::string(s);
        }

        std::string GEOGRAM_API quote(
            const std::string& s, char quotes = '\"'
        );

        bool GEOGRAM_API string_starts_with(
            const std::string& haystack, const std::string& needle
        );

        bool GEOGRAM_API string_ends_with(
            const std::string& haystack, const std::string& needle
        );

        template <class T>
        inline std::string to_string(const T& value) {
            std::ostringstream out;
	    // Makes sure that double-precision number are displayed
	    // with a sufficient number of digits. This is important
	    // to avoid losing precision when using ASCII files.
	    out << std::setprecision(17);
            out << value;
            return out.str();
        }

        template <class T>
        inline std::string to_display_string(const T& value) {
	    return to_string(value);
	}


        template <>
        inline std::string to_display_string(const double& value) {
            std::ostringstream out;	    
            out << value;
            return out.str();
	}

        template <>
        inline std::string to_display_string(const float& value) {
            std::ostringstream out;	    
            out << value;
            return out.str();
	}
	
        template <>
        inline std::string to_string(const bool& value) {
            return value ? "true" : "false";
        }

        class GEOGRAM_API ConversionError : public std::logic_error {
        public:
            ConversionError(const std::string& s, const std::string& type);

            virtual const char* what() const GEO_NOEXCEPT;
        };

        template <class T>
        inline bool from_string(const char* s, T& value) {
            std::istringstream in(s);
            return (in >> value) && (in.eof() || ((in >> std::ws) && in.eof()));
        }

        template <class T>
        inline bool from_string(const std::string& s, T& value) {
            return from_string(s.c_str(), value);
        }

        template <>
        inline bool from_string(const char* s, double& value) {
            errno = 0;
            char* end;
            value = strtod(s, &end);
            return end != s && *end == '\0' && errno == 0;
        }

        template <typename T>
        inline bool string_to_signed_integer(const char* s, T& value) {
            errno = 0;
            char* end;
#ifdef GEO_OS_WINDOWS
            Numeric::int64 v = _strtoi64(s, &end, 10);
#else
            Numeric::int64 v = strtoll(s, &end, 10);
#endif
            if(
                end != s && *end == '\0' && errno == 0 &&
                v >= (std::numeric_limits<T>::min)() &&
                v <= (std::numeric_limits<T>::max)()
            ) {
                value = static_cast<T>(v);
                return true;
            }

            return false;
        }

        template <>
        inline bool from_string(const char* s, Numeric::int8& value) {
            return string_to_signed_integer(s, value);
        }

        template <>
        inline bool from_string(const char* s, Numeric::int16& value) {
            return string_to_signed_integer(s, value);
        }

        template <>
        inline bool from_string(const char* s, Numeric::int32& value) {
            return string_to_signed_integer(s, value);
        }

        template <>
        inline bool from_string(const char* s, Numeric::int64& value) {
            errno = 0;
            char* end;
#ifdef GEO_OS_WINDOWS
            value = _strtoi64(s, &end, 10);
#else
            value = strtoll(s, &end, 10);
#endif
            return end != s && *end == '\0' && errno == 0;
        }

        template <typename T>
        inline bool string_to_unsigned_integer(const char* s, T& value) {
            errno = 0;
            char* end;
#ifdef GEO_OS_WINDOWS
            Numeric::uint64 v = _strtoui64(s, &end, 10);
#else
            Numeric::uint64 v = strtoull(s, &end, 10);
#endif
            if(
                end != s && *end == '\0' && errno == 0 &&
                v <= (std::numeric_limits<T>::max)()
            ) {
                value = static_cast<T>(v);
                return true;
            }

            return false;
        }

        template <>
        inline bool from_string(const char* s, Numeric::uint8& value) {
            return string_to_unsigned_integer(s, value);
        }

        template <>
        inline bool from_string(const char* s, Numeric::uint16& value) {
            return string_to_unsigned_integer(s, value);
        }

        template <>
        inline bool from_string(const char* s, Numeric::uint32& value) {
            return string_to_unsigned_integer(s, value);
        }

        template <>
        inline bool from_string(const char* s, Numeric::uint64& value) {
            errno = 0;
            char* end;
#ifdef GEO_OS_WINDOWS
            value = _strtoui64(s, &end, 10);
#else
            value = strtoull(s, &end, 10);
#endif
            return end != s && *end == '\0' && errno == 0;
        }

        template <>
        inline bool from_string(const char* s, bool& value) {
            if(strcmp(s, "true") == 0 ||
                strcmp(s, "True") == 0 ||
                strcmp(s, "1") == 0
            ) {
                value = true;
                return true;
            }
            if(strcmp(s, "false") == 0 ||
                strcmp(s, "False") == 0 ||
                strcmp(s, "0") == 0
            ) {
                value = false;
                return true;
            }
            return false;
        }

        inline int to_int(const std::string& s) {
            int value;
            if(!from_string(s, value)) {
                throw ConversionError(s, "integer");
            }
            return value;
        }

        inline unsigned int to_uint(const std::string& s) {
            unsigned int value;
            if(!from_string(s, value)) {
                throw ConversionError(s, "integer");
            }
            return value;
        }

        inline double to_double(const std::string& s) {
            double value;
            if(!from_string(s, value)) {
                throw ConversionError(s, "double");
            }
            return value;
        }

        inline bool to_bool(const std::string& s) {
            bool value;
            if(!from_string(s, value)) {
                throw ConversionError(s, "boolean");
            }
            return value;
        }
    }
}

#endif


/******* extracted from ../basic/line_stream.h *******/

#ifndef GEOGRAM_BASIC_LINE_STREAM
#define GEOGRAM_BASIC_LINE_STREAM

#include <cstring>
#include <stdio.h>


namespace GEO {

    class GEOGRAM_API LineInput {
    public:
        LineInput(const std::string& filename);

        ~LineInput();

        bool OK() const {
            return ok_;
        }

        bool eof() const {
            return feof(F_) ? true : false;
        }

        bool get_line();

        index_t nb_fields() const {
            return index_t(field_.size());
        }

        size_t line_number() const {
            return line_num_;
        }

        char* field(index_t i) {
            geo_assert(i < nb_fields());
            return field_[i];
        }

        const char* field(index_t i) const {
            geo_assert(i < nb_fields());
            return field_[i];
        }

        signed_index_t field_as_int(index_t i) const {
            signed_index_t result = 0;
            if(!String::from_string(field(i), result)) {
                conversion_error(i, "integer");
            }
            return result;
        }

        index_t field_as_uint(index_t i) const {
            index_t result = 0;
            if(!String::from_string(field(i), result)) {
                conversion_error(i, "unsigned integer");
            }
            return result;
        }

        double field_as_double(index_t i) const {
            double result = 0;
            if(!String::from_string(field(i), result)) {
                conversion_error(i, "floating point");
            }
            return result;
        }

        bool field_matches(index_t i, const char* s) const {
            return strcmp(field(i), s) == 0;
        }

        void get_fields(const char* separators = " \t\r\n");

        const char* current_line() const {
            return line_;
        }

    private:
        GEO_NORETURN_DECL void conversion_error(
            index_t index, const char* type
        ) const GEO_NORETURN ;

        static const index_t MAX_LINE_LEN = 65535;

        FILE* F_;
        std::string file_name_;
        size_t line_num_;
        char line_[MAX_LINE_LEN];
        std::vector<char*> field_;
        bool ok_;
    };
}

#endif


/******* extracted from ../basic/stopwatch.h *******/

#ifndef GEOGRAM_BASIC_STOPWATCH
#define GEOGRAM_BASIC_STOPWATCH




#ifdef GEO_OS_WINDOWS
#else
#include <sys/types.h>
#include <sys/times.h>
#endif




namespace GEO {


    class GEOGRAM_API SystemStopwatch {
    public:
        SystemStopwatch();

        void print_elapsed_time(std::ostream& os) const;

        double elapsed_user_time() const;

        static double now();

    private:
#if defined(GEO_OS_WINDOWS)
        long start_;
#elif defined(GEO_OS_EMSCRIPTEN)
        double startf_;
#else        
        tms start_;
        clock_t start_user_;
#endif
    };

    

    class GEOGRAM_API ProcessorStopwatch {
    public:
        ProcessorStopwatch() {
            start_ = now();
        }

        static Numeric::uint64 now();

        Numeric::uint64 elapsed_time() const {
            return now() - start_;
        }

    private:
        Numeric::uint64 start_;
    };

    

    class GEOGRAM_API Stopwatch {
    public:
        Stopwatch(const std::string& task_name, bool verbose=true) :
  	    task_name_(task_name), verbose_(verbose) {
        }

        double elapsed_time() const {
            return W_.elapsed_user_time();
        }


        ~Stopwatch() {
	    if(verbose_) {
		Logger::out(task_name_)
		    << "Elapsed time: " << W_.elapsed_user_time()
		    << " s" << std::endl;
	    }
        }

        

    private:
        std::string task_name_;
	bool verbose_;
        SystemStopwatch W_;
    };
}

#endif


/******* extracted from ../basic/command_line.h *******/

#ifndef GEOGRAM_BASIC_COMMAND_LINE
#define GEOGRAM_BASIC_COMMAND_LINE



namespace GEO {

    namespace CmdLine {

        void GEOGRAM_API initialize();

        void GEOGRAM_API terminate();


	void GEOGRAM_API set_config_file_name(const std::string& filename);

	std::string GEOGRAM_API get_config_file_name();
	
        enum ArgType {
            
            ARG_UNDEFINED = 0,
            
            ARG_INT = 1,
            
            ARG_DOUBLE = 2,
            
            ARG_STRING = 4,
            
            ARG_BOOL = 8,
            
            ARG_PERCENT = 16
        };

        enum ArgFlags {
            
            ARG_FLAGS_DEFAULT = 0,
            
            ARG_ADVANCED = 1
        };

        void GEOGRAM_API declare_arg_group(
            const std::string& name,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        );

        void GEOGRAM_API declare_arg(
            const std::string& name,
            ArgType type,
            const std::string& default_value,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        );

        ArgType GEOGRAM_API get_arg_type(const std::string& name);

        bool GEOGRAM_API arg_is_declared(const std::string& name);

        inline void declare_arg(
            const std::string& name,
            const std::string& default_value,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        ) {
            declare_arg(
                name, ARG_STRING, default_value,
                description, flags
            );
        }

        inline void declare_arg(
            const std::string& name,
            const char* default_value,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        ) {
            declare_arg(
                name, ARG_STRING, default_value,
                description, flags
            );
        }

        inline void declare_arg(
            const std::string& name,
            int default_value,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        ) {
            declare_arg(
                name, ARG_INT, String::to_string(default_value),
                description, flags
            );
        }

        inline void declare_arg(
            const std::string& name,
            double default_value,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        ) {
            declare_arg(
                name, ARG_DOUBLE, String::to_string(default_value),
                description, flags
            );
        }

        inline void declare_arg(
            const std::string& name,
            bool default_value,
            const std::string& description,
            ArgFlags flags = ARG_FLAGS_DEFAULT
        ) {
            declare_arg(
                name, ARG_BOOL, default_value ? "true" : "false",
                description, flags
            );
        }

        inline void declare_arg_percent(
            const std::string& name,
            double default_value,
            const std::string& description = "...",
            ArgFlags flags = ARG_FLAGS_DEFAULT
        ) {
            declare_arg(
                name, ARG_PERCENT, String::to_string(default_value) + "%",
                description, flags
            );
        }

        bool GEOGRAM_API parse(
            int argc, char** argv, std::vector<std::string>& unparsed_args,
            const std::string& additional_arg_specs = ""
        );

        bool GEOGRAM_API parse(
            int argc, char** argv
        );

	int GEOGRAM_API argc();

	
	typedef char** charptrptr; // Need to do that else the compiler thinks
	                           // that GEOGRAM_API qualifies the ptr instead
	                           // of the function.
	
	charptrptr GEOGRAM_API argv();
	
        void GEOGRAM_API show_usage(
            const std::string& additional_args = "",
            bool advanced = false
        );

        std::string GEOGRAM_API get_arg(const std::string& name);

        int GEOGRAM_API get_arg_int(const std::string& name);

        unsigned int GEOGRAM_API get_arg_uint(const std::string& name);

        double GEOGRAM_API get_arg_double(const std::string& name);

        double GEOGRAM_API get_arg_percent(
            const std::string& name, double reference
        );

        bool GEOGRAM_API get_arg_bool(const std::string& name);

        bool GEOGRAM_API set_arg(
            const std::string& name, const std::string& value
        );

        inline bool set_arg(const std::string& name, const char* value) {
            return set_arg(name, std::string(value));
        }

        void GEOGRAM_API set_arg(const std::string& name, int value);

        void GEOGRAM_API set_arg(const std::string& name, unsigned int value);

        void GEOGRAM_API set_arg(const std::string& name, double value);

        void GEOGRAM_API set_arg(const std::string& name, bool value);

        void GEOGRAM_API set_arg_percent(const std::string& name, double value);

        

        void GEOGRAM_API get_args(std::vector<std::string>& args);

        index_t GEOGRAM_API ui_terminal_width();

        void GEOGRAM_API ui_separator(
            const std::string& title,
            const std::string& short_title = ""
        );

        void GEOGRAM_API ui_separator();

        void GEOGRAM_API ui_close_separator();

        void GEOGRAM_API ui_message(
            const std::string& message,
            index_t wrap_margin
        );

        void GEOGRAM_API ui_message(
            const std::string& message
        );

        void GEOGRAM_API ui_clear_line();

        void GEOGRAM_API ui_progress(
            const std::string& task_name, index_t val,
            index_t percent, bool clear = true
        );

        void GEOGRAM_API ui_progress_time(
            const std::string& task_name,
            double elapsed, bool clear = true
        );

        void GEOGRAM_API ui_progress_canceled(
            const std::string& task_name,
            double elapsed, index_t percent, bool clear = true
        );

        std::string GEOGRAM_API ui_feature(
            const std::string& feature, bool show = true
        );
    }
}

#endif


/******* extracted from ../basic/command_line_args.h *******/

#ifndef GEOGRAM_BASIC_COMMAND_LINE_ARGS
#define GEOGRAM_BASIC_COMMAND_LINE_ARGS



namespace GEO {

    namespace CmdLine {

        bool GEOGRAM_API import_arg_group(
            const std::string& name
        );

        bool GEOGRAM_API set_profile(
            const std::string& name
        );
    }
}

#endif


/******* extracted from delaunay.h *******/

#ifndef GEOGRAM_DELAUNAY_DELAUNAY
#define GEOGRAM_DELAUNAY_DELAUNAY

#include <stdexcept>


namespace GEO {

    class Mesh;

    

    class GEOGRAM_API Delaunay : public Counted {
    public:
        struct InvalidDimension : std::logic_error {
            InvalidDimension(
                coord_index_t dimension,
                const char* name,
                const char* expected
            );

            virtual const char* what() const GEO_NOEXCEPT;
        };


        struct InvalidInput : std::logic_error {

            InvalidInput(int error_code_in);

            InvalidInput(const InvalidInput& rhs);

            virtual ~InvalidInput() GEO_NOEXCEPT;
            
            virtual const char* what() const GEO_NOEXCEPT;

            int error_code;

            vector<index_t> invalid_facets;
        };
        
        static Delaunay* create(
            coord_index_t dim, const std::string& name = "default"
        );


        static void initialize();

        coord_index_t dimension() const {
            return dimension_;
        }

        index_t cell_size() const {
            return cell_size_;
        }

        virtual void set_vertices(
            index_t nb_vertices, const double* vertices
        );


        void set_reorder(bool x) {
            do_reorder_ = x;
        }


        virtual void set_BRIO_levels(const vector<index_t>& levels);

        const double* vertices_ptr() const {
            return vertices_;
        }

        const double* vertex_ptr(index_t i) const {
            geo_debug_assert(i < nb_vertices());
            return vertices_ + vertex_stride_ * i;
        }

        index_t nb_vertices() const {
            return nb_vertices_;
        }

        virtual bool supports_constraints() const;

        virtual void set_constraints(const Mesh* mesh) {
            geo_assert(supports_constraints());
            constraints_ = mesh;
        }

        void set_refine(bool x) {
            refine_ = x;
        }

        bool get_refine() const {
            return refine_;
        }

        void set_quality(double qual) {
            quality_ = qual;
        }
        
        const Mesh* constraints() const {
            return constraints_;
        }

        index_t nb_cells() const {
            return nb_cells_;
        }

        index_t nb_finite_cells() const {
            geo_debug_assert(keeps_infinite());
            return nb_finite_cells_;
        }

        const signed_index_t* cell_to_v() const {
            return cell_to_v_;
        }

        const signed_index_t* cell_to_cell() const {
            return cell_to_cell_;
        }

        virtual index_t nearest_vertex(const double* p) const;

        signed_index_t cell_vertex(index_t c, index_t lv) const {
            geo_debug_assert(c < nb_cells());
            geo_debug_assert(lv < cell_size());
            return cell_to_v_[c * cell_v_stride_ + lv];
        }

        signed_index_t cell_adjacent(index_t c, index_t lf) const {
            geo_debug_assert(c < nb_cells());
            geo_debug_assert(lf < cell_size());
            return cell_to_cell_[c * cell_neigh_stride_ + lf];
        }

        bool cell_is_infinite(index_t c) const;

        bool cell_is_finite(index_t c) const {
            return !cell_is_infinite(c);
        }
        
        index_t index(index_t c, signed_index_t v) const {
            geo_debug_assert(c < nb_cells());
            geo_debug_assert(v < (signed_index_t) nb_vertices());
            for(index_t iv = 0; iv < cell_size(); iv++) {
                if(cell_vertex(c, iv) == v) {
                    return iv;
                }
            }
            geo_assert_not_reached;
        }

        index_t adjacent_index(index_t c1, index_t c2) const {
            geo_debug_assert(c1 < nb_cells());
            geo_debug_assert(c2 < nb_cells());
            for(index_t f = 0; f < cell_size(); f++) {
                if(cell_adjacent(c1, f) == signed_index_t(c2)) {
                    return f;
                }
            }
            geo_assert_not_reached;
        }

        signed_index_t vertex_cell(index_t v) const {
            geo_debug_assert(v < nb_vertices());
            geo_debug_assert(v < v_to_cell_.size());
            return v_to_cell_[v];
        }

        
        signed_index_t next_around_vertex(index_t c, index_t lv) const {
            geo_debug_assert(c < nb_cells());
            geo_debug_assert(lv < cell_size());
            return cicl_[cell_size() * c + lv];
        }

        void get_neighbors(
            index_t v, vector<index_t>& neighbors
        ) const {
            geo_debug_assert(v < nb_vertices());
            if(store_neighbors_) {
                neighbors_.get_array(v, neighbors);
            } else {
                get_neighbors_internal(v, neighbors);
            }
        }

        void save_histogram(std::ostream& out) const;

        bool stores_neighbors() const {
            return store_neighbors_;
        }

        void set_stores_neighbors(bool x) {
            store_neighbors_ = x;
            if(store_neighbors_) {
                set_stores_cicl(true);
            }
        }

        bool stores_cicl() const {
            return store_cicl_;
        }

        void set_stores_cicl(bool x) {
            store_cicl_ = x;
        }


        bool keeps_infinite() const {
            return keep_infinite_;
        }

        void set_keeps_infinite(bool x) {
            keep_infinite_ = x;
        }
        
        bool thread_safe() const {
            return neighbors_.thread_safe();
        }

        void set_thread_safe(bool x) {
            neighbors_.set_thread_safe(x);
        }

        void set_default_nb_neighbors(index_t x) {
            default_nb_neighbors_ = x;
        }

        index_t default_nb_neighbors() const {
            return default_nb_neighbors_;
        }

        void clear_neighbors() {
            neighbors_.clear();
        }

        void set_keep_regions(bool x) {
	    keep_regions_ = x;
	}

        virtual index_t region(index_t t) const;
	    
	
    protected:
        Delaunay(coord_index_t dimension);

        virtual ~Delaunay();

        virtual void get_neighbors_internal(
            index_t v, vector<index_t>& neighbors
        ) const;

        virtual void set_arrays(
            index_t nb_cells,
            const signed_index_t* cell_to_v, const signed_index_t* cell_to_cell
        );

        void update_v_to_cell();

        void update_cicl();

        virtual void update_neighbors();

        void set_next_around_vertex(
            index_t c1, index_t lv, index_t c2
        ) {
            geo_debug_assert(c1 < nb_cells());
            geo_debug_assert(c2 < nb_cells());
            geo_debug_assert(lv < cell_size());
            cicl_[cell_size() * c1 + lv] = signed_index_t(c2);
        }

    public:
        virtual void store_neighbors_CB(index_t i);

    protected:
        void set_dimension(coord_index_t dim) {
            dimension_ = dim;
            vertex_stride_ = dim;
            cell_size_ = index_t(dim) + 1;
            cell_v_stride_ = cell_size_;
            cell_neigh_stride_ = cell_size_;
        }

        coord_index_t dimension_;
        index_t vertex_stride_;
        index_t cell_size_;
        index_t cell_v_stride_;
        index_t cell_neigh_stride_;

        const double* vertices_;
        index_t nb_vertices_;
        index_t nb_cells_;
        const signed_index_t* cell_to_v_;
        const signed_index_t* cell_to_cell_;
        vector<signed_index_t> v_to_cell_;
        vector<signed_index_t> cicl_;
        bool is_locked_;
        PackedArrays neighbors_;
        bool store_neighbors_;
        index_t default_nb_neighbors_;

        bool do_reorder_; 

        const Mesh* constraints_;

        bool refine_;
        double quality_;

        bool store_cicl_; 

        bool keep_infinite_;

        index_t nb_finite_cells_;

	bool keep_regions_;
    };

    typedef SmartPointer<Delaunay> Delaunay_var;

    typedef Factory1<Delaunay, coord_index_t> DelaunayFactory;

#define geo_register_Delaunay_creator(type, name) \
    geo_register_creator(GEO::DelaunayFactory, type, name)
}

#endif

