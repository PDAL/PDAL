//
//! Copyright (c) 2011
//! Brandon Kohn
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
    
namespace pdalboost{} namespace boost = pdalboost; namespace pdalboost{ namespace numeric {

    template <>
    struct numeric_cast_traits
        <
            char
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            char
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int8_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint8_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int16_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint16_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int32_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint32_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::int64_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            pdalboost::uint64_t
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            float
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            float
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            double
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            double
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , char
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<char> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::int8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::uint8_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint8_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::int16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::uint16_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint16_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::int32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::uint32_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint32_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::int64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::int64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , pdalboost::uint64_t
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<pdalboost::uint64_t> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , float
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<float> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<double> rounding_policy;
    }; 
    
    template <>
    struct numeric_cast_traits
        <
            long double
          , long double
        >
    {
        typedef def_overflow_handler overflow_policy;
        typedef UseInternalRangeChecker range_checking_policy;
        typedef Trunc<long double> rounding_policy;
    }; 
       
}}
