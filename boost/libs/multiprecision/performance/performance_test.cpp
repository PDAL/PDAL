///////////////////////////////////////////////////////////////
//  Copyright 2011 John Maddock. Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_

#define BOOST_CHRONO_HEADER_ONLY

#ifdef _MSC_VER
#  define _SCL_SECURE_NO_WARNINGS
#endif

#if !defined(TEST_MPF) && !defined(TEST_MPZ) && \
   !defined(TEST_CPP_DEC_FLOAT) && !defined(TEST_MPFR) && !defined(TEST_MPQ) \
   && !defined(TEST_TOMMATH) && !defined(TEST_TOMMATH_BOOST_RATIONAL) && !defined(TEST_MPZ_BOOST_RATIONAL)\
   && !defined(TEST_CPP_INT) && !defined(TEST_CPP_INT_RATIONAL)
#  define TEST_MPF
#  define TEST_MPZ
#  define TEST_MPQ
#  define TEST_MPFR
#  define TEST_CPP_DEC_FLOAT
#  define TEST_MPQ
#  define TEST_TOMMATH
#  define TEST_CPP_INT
#  define TEST_CPP_INT_RATIONAL

#ifdef _MSC_VER
#pragma message("CAUTION!!: No backend type specified so testing everything.... this will take some time!!")
#endif
#ifdef __GNUC__
#pragma warning "CAUTION!!: No backend type specified so testing everything.... this will take some time!!"
#endif

#endif

#if defined(TEST_MPF) || defined(TEST_MPZ) || defined(TEST_MPQ) || defined(TEST_MPZ_BOOST_RATIONAL)
#include <boost/multiprecision/gmp.hpp>
#include <boost/multiprecision/rational_adaptor.hpp>
#endif
#ifdef TEST_CPP_DEC_FLOAT
#include <boost/multiprecision/cpp_dec_float.hpp>
#endif
#if defined(TEST_MPFR)
#include <boost/multiprecision/mpfr.hpp>
#endif
#if defined(TEST_TOMMATH) || defined(TEST_TOMMATH_BOOST_RATIONAL)
#include <boost/multiprecision/tommath.hpp>
#include <boost/multiprecision/rational_adaptor.hpp>
#endif
#if defined(TEST_CPP_INT) || defined(TEST_CPP_INT_RATIONAL)
#include <boost/multiprecision/cpp_int.hpp>
#endif

#include <boost/chrono.hpp>
#include <vector>
#include <map>
#include <string>
#include <cstring>
#include <cctype>
#include <iostream>
#include <iomanip>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>

template <class Clock>
struct stopwatch
{
   typedef typename Clock::duration duration;
   stopwatch()
   {
      m_start = Clock::now();
   }
   duration elapsed()
   {
      return Clock::now() - m_start;
   }
   void reset()
   {
      m_start = Clock::now();
   }

private:
   typename Clock::time_point m_start;
};

unsigned bits_wanted; // for integer types

namespace pdalboost {} namespace boost = pdalboost; namespace pdalboost{ namespace multiprecision{

template<>
class number_category<pdalboost::int64_t> : public mpl::int_<number_kind_integer>{};
template<>
class number_category<pdalboost::uint64_t> : public mpl::int_<number_kind_integer>{};

}}

template <class T, int Type>
struct tester
{
   tester()
   {
      a.assign(500, 0);
      for(int i = 0; i < 500; ++i)
      {
         b.push_back(generate_random());
         c.push_back(generate_random());
         small.push_back(gen());
      }
   }
   double test_add()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] + c[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_subtract()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] - c[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_add_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] + 1;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_subtract_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] - 1;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_multiply()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned k = 0; k < b.size(); ++k)
            a[k] = b[k] * c[k];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_multiply_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] * 3;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_divide()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] / c[i] + b[i] / small[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_divide_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] / 3;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_str(const pdalboost::mpl::false_&)
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < b.size(); ++i)
         a[i] = pdalboost::lexical_cast<T>(pdalboost::lexical_cast<std::string>(b[i]));
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_str(const pdalboost::mpl::true_&)
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < b.size(); ++i)
         a[i].assign(b[i].str());
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_str()
   {
      return test_str(pdalboost::is_class<T>());
   }
   //
   // The following tests only work for integer types:
   //
   double test_mod()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] % c[i] + b[i] % small[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_mod_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] % 254;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_or()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] | c[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_or_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] | 234;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_and()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] & c[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_and_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] & 234;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_xor()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] ^ c[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_xor_int()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] ^ 234;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_complement()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = ~b[i];
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_left_shift()
   {
      int shift = std::numeric_limits<T>::is_bounded ? std::numeric_limits<T>::digits : bits_wanted;
      shift /= 2;
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] << shift;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_right_shift()
   {
      int shift = std::numeric_limits<T>::is_bounded ? std::numeric_limits<T>::digits : bits_wanted;
      shift /= 2;
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] >> shift;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_gcd()
   {
      using pdalboost::math::gcd;
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = gcd(b[i], c[i]);
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_powm()
   {
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 25; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = powm(b[i], b[i] / 2, c[i]);
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   double test_construct()
   {
      std::allocator<T> a;
      T* pt = a.allocate(1000);
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < 1000; ++i)
            new(pt+i) T();
         for(unsigned i = 0; i < 1000; ++i)
            a.destroy(pt+i);
      }
      double result = pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
      a.deallocate(pt, 1000);
      return result;
   }
   double test_construct_unsigned()
   {
      std::allocator<T> a;
      T* pt = a.allocate(1000);
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < 1000; ++i)
            new(pt+i) T(i);
         for(unsigned i = 0; i < 1000; ++i)
            a.destroy(pt+i);
      }
      double result = pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
      a.deallocate(pt, 1000);
      return result;
   }
   double test_construct_unsigned_ll()
   {
      std::allocator<T> a;
      T* pt = a.allocate(1000);
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned long long j = 0; j < 1000; ++j)
            new(pt+j) T(j);
         for(unsigned j = 0; j < 1000; ++j)
            a.destroy(pt+j);
      }
      double result = pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
      a.deallocate(pt, 1000);
      return result;
   }

   //
   // Hetero operations:
   //
   template <class U>
   static U get_hetero_test_value(pdalboost::mpl::false_ const&)
   {
      return U(2) / 3;
   }
   template <class U>
   static U get_hetero_test_value(pdalboost::mpl::true_ const&)
   {
      return (std::numeric_limits<U>::max)() >> 4;
   }
   template <class U>
   static U get_hetero_test_value()
   {
      return get_hetero_test_value<U>(pdalboost::is_integral<U>());
   }
   template <class U>
   double test_multiply_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] * val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_inplace_multiply_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      for(unsigned i = 0; i < b.size(); ++i)
         a[i] = b[i];
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] *= val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_add_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] + val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_inplace_add_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      for(unsigned i = 0; i < b.size(); ++i)
         a[i] = b[i];
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] += val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_subtract_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] - val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_inplace_subtract_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      for(unsigned i = 0; i < b.size(); ++i)
         a[i] = b[i];
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] -= val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_divide_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] = b[i] / val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
   template <class U>
   double test_inplace_divide_hetero()
   {
      static const U val = get_hetero_test_value<U>();
      for(unsigned i = 0; i < b.size(); ++i)
         a[i] = b[i];
      stopwatch<pdalboost::chrono::high_resolution_clock> w;
      for(unsigned i = 0; i < 1000; ++i)
      {
         for(unsigned i = 0; i < b.size(); ++i)
            a[i] /= val;
      }
      return pdalboost::chrono::duration_cast<pdalboost::chrono::duration<double> >(w.elapsed()).count();
   }
private:
   T generate_random()
   {
      return generate_random(pdalboost::mpl::int_<Type>());
   }
   T generate_random(const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_floating_point>&)
   {
      T val = gen();
      T prev_val = -1;
      while(val != prev_val)
      {
         val *= (gen.max)();
         prev_val = val;
         val += gen();
      }
      int e;
      val = frexp(val, &e);

      typedef typename T::backend_type::exponent_type e_type;
      static pdalboost::random::uniform_int_distribution<e_type> ui(-30, 30);
      return ldexp(val, static_cast<int>(ui(gen)));
   }
   T generate_random(const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_integer>&)
   {
      typedef pdalboost::random::mt19937::result_type random_type;

      T max_val;
      unsigned digits;
      if(std::numeric_limits<T>::is_bounded)
      {
         max_val = (std::numeric_limits<T>::max)();
         digits = std::numeric_limits<T>::digits;
      }
      else
      {
         max_val = T(1) << bits_wanted;
         digits = bits_wanted;
      }

      unsigned bits_per_r_val = std::numeric_limits<random_type>::digits - 1;
      while((random_type(1) << bits_per_r_val) > (gen.max)()) --bits_per_r_val;

      unsigned terms_needed = digits / bits_per_r_val + 1;

      T val = 0;
      for(unsigned i = 0; i < terms_needed; ++i)
      {
         val *= (gen.max)();
         val += gen();
      }
      val %= max_val;
      return val;
   }
   T generate_random(const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_rational>&)
   {
      typedef pdalboost::random::mt19937::result_type random_type;
      typedef typename pdalboost::multiprecision::component_type<T>::type IntType;

      IntType max_val;
      unsigned digits;
      if(std::numeric_limits<IntType>::is_bounded)
      {
         max_val = (std::numeric_limits<IntType>::max)();
         digits = std::numeric_limits<IntType>::digits;
      }
      else
      {
         max_val = IntType(1) << bits_wanted;
         digits = bits_wanted;
      }

      unsigned bits_per_r_val = std::numeric_limits<random_type>::digits - 1;
      while((random_type(1) << bits_per_r_val) > (gen.max)()) --bits_per_r_val;

      unsigned terms_needed = digits / bits_per_r_val + 1;

      IntType val = 0;
      IntType denom = 0;
      for(unsigned i = 0; i < terms_needed; ++i)
      {
         val *= (gen.max)();
         val += gen();
      }
      for(unsigned i = 0; i < terms_needed; ++i)
      {
         denom *= (gen.max)();
         denom += gen();
      }
      if(denom == 0)
         denom = 1;
      val %= max_val;
      denom %= max_val;
      return T(val, denom);
   }
   std::vector<T> a, b, c, small;
   static pdalboost::random::mt19937 gen;
};

template <class N, int V>
pdalboost::random::mt19937 tester<N, V>::gen;

const char* category_name(const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_integer>&)
{
   return "integer";
}
const char* category_name(const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_floating_point>&)
{
   return "float";
}
const char* category_name(const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_rational>&)
{
   return "rational";
}

//
// Keys in order are:
// Category
// Operator
// Type
// Precision
// Time
//
std::map<std::string, std::map<std::string, std::map<std::string, std::map<int, double> > > > result_table;

void report_result(const char* cat, const char* type, const char* op, unsigned precision, double time)
{
   std::cout << std::left << std::setw(15) << type << std::setw(10) << precision << std::setw(35) << op << time << std::endl;
   result_table[cat][op][type][precision] = time;
}

template <class Number, int N>
void test_int_ops(tester<Number, N>& t, const char* type, unsigned precision, const pdalboost::mpl::int_<pdalboost::multiprecision::number_kind_integer>&)
{
   const char* cat = "integer";
   report_result(cat, type, "%", precision, t.test_mod());
   report_result(cat, type, "|", precision, t.test_or());
   report_result(cat, type, "&", precision, t.test_and());
   report_result(cat, type, "^", precision, t.test_xor());
   //report_result(cat, type, "~", precision, t.test_complement());
   report_result(cat, type, "<<", precision, t.test_left_shift());
   report_result(cat, type, ">>", precision, t.test_right_shift());
   // integer ops:
   report_result(cat, type, "%(int)", precision, t.test_mod_int());
   report_result(cat, type, "|(int)", precision, t.test_or_int());
   report_result(cat, type, "&(int)", precision, t.test_and_int());
   report_result(cat, type, "^(int)", precision, t.test_xor_int());
   report_result(cat, type, "gcd", precision, t.test_gcd());
   report_result(cat, type, "powm", precision, t.test_powm());
}
template <class Number, int N, class U>
void test_int_ops(tester<Number, N>& t, const char* type, unsigned precision, const U&)
{
}

template <class Number>
void test(const char* type, unsigned precision)
{
   bits_wanted = precision;
   tester<Number, pdalboost::multiprecision::number_category<Number>::value> t;
   const char* cat = category_name(typename pdalboost::multiprecision::number_category<Number>::type());
   //
   // call t.test_multiply() first so that the destination operands are
   // forced to perform whatever memory allocation may be needed.  That way
   // we measure only algorithm performance, and not memory allocation effects.
   //
   t.test_multiply();
   //
   // Now the actual tests:
   //
   report_result(cat, type, "+", precision, t.test_add());
   report_result(cat, type, "-", precision, t.test_subtract());
   report_result(cat, type, "*", precision, t.test_multiply());
   report_result(cat, type, "/", precision, t.test_divide());
   report_result(cat, type, "str", precision, t.test_str());
   // integer ops:
   report_result(cat, type, "+(int)", precision, t.test_add_int());
   report_result(cat, type, "-(int)", precision, t.test_subtract_int());
   report_result(cat, type, "*(int)", precision, t.test_multiply_int());
   report_result(cat, type, "/(int)", precision, t.test_divide_int());
   // construction and destruction:
   report_result(cat, type, "construct", precision, t.test_construct());
   report_result(cat, type, "construct(unsigned)", precision, t.test_construct_unsigned());
   report_result(cat, type, "construct(unsigned long long)", precision, t.test_construct_unsigned_ll());
   test_int_ops(t, type, precision, typename pdalboost::multiprecision::number_category<Number>::type());
   // Hetero ops:
   report_result(cat, type, "+(unsigned long long)", precision, t.template test_add_hetero<unsigned long long>());
   report_result(cat, type, "-(unsigned long long)", precision, t.template test_subtract_hetero<unsigned long long>());
   report_result(cat, type, "*(unsigned long long)", precision, t.template test_multiply_hetero<unsigned long long>());
   report_result(cat, type, "/(unsigned long long)", precision, t.template test_divide_hetero<unsigned long long>());
   report_result(cat, type, "+=(unsigned long long)", precision, t.template test_inplace_add_hetero<unsigned long long>());
   report_result(cat, type, "-=(unsigned long long)", precision, t.template test_inplace_subtract_hetero<unsigned long long>());
   report_result(cat, type, "*=(unsigned long long)", precision, t.template test_inplace_multiply_hetero<unsigned long long>());
   report_result(cat, type, "/=(unsigned long long)", precision, t.template test_inplace_divide_hetero<unsigned long long>());
}

void quickbook_results()
{
   //
   // Keys in order are:
   // Category
   // Operator
   // Type
   // Precision
   // Time
   //
   typedef std::map<std::string, std::map<std::string, std::map<std::string, std::map<int, double> > > >::const_iterator  category_iterator;
   typedef std::map<std::string, std::map<std::string, std::map<int, double> > >::const_iterator                          operator_iterator;
   typedef std::map<std::string, std::map<int, double> >::const_iterator                                                  type_iterator;
   typedef std::map<int, double>::const_iterator                                                                          precision_iterator;

   for(category_iterator i = result_table.begin(); i != result_table.end(); ++i)
   {
      std::string cat = i->first;
      cat[0] = std::toupper(cat[0]);
      std::cout << "[section:" << i->first << "_performance " << cat << " Type Perfomance]" << std::endl;

      for(operator_iterator j = i->second.begin(); j != i->second.end(); ++j)
      {
         std::string op = j->first;
         std::cout << "[table Operator " << op << std::endl;
         std::cout << "[[Backend]";

         for(precision_iterator k = j->second.begin()->second.begin(); k != j->second.begin()->second.end(); ++k)
         {
            std::cout << "[" << k->first << " Bits]";
         }
         std::cout << "]\n";

         std::vector<double> best_times(j->second.begin()->second.size(), (std::numeric_limits<double>::max)());
         for(unsigned m = 0; m < j->second.begin()->second.size(); ++m)
         {
            for(type_iterator k = j->second.begin(); k != j->second.end(); ++k)
            {
               precision_iterator l = k->second.begin();
               std::advance(l, m);
               if(best_times[m] > l->second)
                  best_times[m] = l->second ? l->second : best_times[m];
            }
         }

         for(type_iterator k = j->second.begin(); k != j->second.end(); ++k)
         {
            std::cout << "[[" << k->first << "]";

            unsigned m = 0;
            for(precision_iterator l = k->second.begin(); l != k->second.end(); ++l)
            {
               double rel_time = l->second / best_times[m];
               if(rel_time == 1)
                  std::cout << "[[*" << rel_time << "]";
               else
                  std::cout << "[" << rel_time;
               std::cout << " (" << l->second << "s)]";
               ++m;
            }

            std::cout << "]\n";
         }

         std::cout << "]\n";
      }

      std::cout << "[endsect]" << std::endl;
   }
}


int main()
{
#ifdef TEST_INT64
   test<pdalboost::uint64_t>("pdalboost::uint64_t", 64);
#endif
#ifdef TEST_MPF
   test<pdalboost::multiprecision::mpf_float_50>("gmp_float", 50);
   test<pdalboost::multiprecision::mpf_float_100>("gmp_float", 100);
   test<pdalboost::multiprecision::mpf_float_500>("gmp_float", 500);
#endif
#ifdef TEST_MPZ
   test<pdalboost::multiprecision::mpz_int>("gmp_int", 128);
   test<pdalboost::multiprecision::mpz_int>("gmp_int", 256);
   test<pdalboost::multiprecision::mpz_int>("gmp_int", 512);
   test<pdalboost::multiprecision::mpz_int>("gmp_int", 1024);
#endif
#ifdef TEST_CPP_INT
   //test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_int_backend<64, 64, pdalboost::multiprecision::unsigned_magnitude, pdalboost::multiprecision::unchecked, void>, pdalboost::multiprecision::et_off> >("cpp_int(unsigned, fixed)", 64);
   //test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_int_backend<64, 64, pdalboost::multiprecision::signed_magnitude, pdalboost::multiprecision::unchecked, void>, pdalboost::multiprecision::et_off> >("cpp_int(fixed)", 64);
   test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_int_backend<128, 128, pdalboost::multiprecision::signed_magnitude, pdalboost::multiprecision::unchecked, void>, pdalboost::multiprecision::et_off> >("cpp_int(fixed)", 128);
   test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_int_backend<256, 256, pdalboost::multiprecision::signed_magnitude, pdalboost::multiprecision::unchecked, void>, pdalboost::multiprecision::et_off> >("cpp_int(fixed)", 256);
   test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_int_backend<512, 512, pdalboost::multiprecision::signed_magnitude, pdalboost::multiprecision::unchecked, void>, pdalboost::multiprecision::et_off> >("cpp_int(fixed)", 512);
   test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_int_backend<1024, 1024, pdalboost::multiprecision::signed_magnitude, pdalboost::multiprecision::unchecked, void>, pdalboost::multiprecision::et_off> >("cpp_int(fixed)", 1024);

   test<pdalboost::multiprecision::cpp_int>("cpp_int", 128);
   test<pdalboost::multiprecision::cpp_int>("cpp_int", 256);
   test<pdalboost::multiprecision::cpp_int>("cpp_int", 512);
   test<pdalboost::multiprecision::cpp_int>("cpp_int", 1024);
#endif
#ifdef TEST_CPP_INT_RATIONAL
   test<pdalboost::multiprecision::cpp_rational>("cpp_rational", 128);
   test<pdalboost::multiprecision::cpp_rational>("cpp_rational", 256);
   test<pdalboost::multiprecision::cpp_rational>("cpp_rational", 512);
   test<pdalboost::multiprecision::cpp_rational>("cpp_rational", 1024);
#endif
#ifdef TEST_MPQ
   test<pdalboost::multiprecision::mpq_rational>("mpq_rational", 128);
   test<pdalboost::multiprecision::mpq_rational>("mpq_rational", 256);
   test<pdalboost::multiprecision::mpq_rational>("mpq_rational", 512);
   test<pdalboost::multiprecision::mpq_rational>("mpq_rational", 1024);
#endif
#ifdef TEST_TOMMATH
   test<pdalboost::multiprecision::tom_int>("tommath_int", 128);
   test<pdalboost::multiprecision::tom_int>("tommath_int", 256);
   test<pdalboost::multiprecision::tom_int>("tommath_int", 512);
   test<pdalboost::multiprecision::tom_int>("tommath_int", 1024);
   /*
   //
   // These are actually too slow to test!!!
   //
   test<pdalboost::multiprecision::tom_rational>("tom_rational", 128);
   test<pdalboost::multiprecision::tom_rational>("tom_rational", 256);
   test<pdalboost::multiprecision::tom_rational>("tom_rational", 512);
   test<pdalboost::multiprecision::tom_rational>("tom_rational", 1024);
   */
#endif
#ifdef TEST_CPP_DEC_FLOAT
   test<pdalboost::multiprecision::cpp_dec_float_50>("cpp_dec_float", 50);
   test<pdalboost::multiprecision::cpp_dec_float_100>("cpp_dec_float", 100);
   test<pdalboost::multiprecision::number<pdalboost::multiprecision::cpp_dec_float<500> > >("cpp_dec_float", 500);
#endif
#ifdef TEST_MPFR
   test<pdalboost::multiprecision::mpfr_float_50>("mpfr_float", 50);
   test<pdalboost::multiprecision::mpfr_float_100>("mpfr_float", 100);
   test<pdalboost::multiprecision::mpfr_float_500>("mpfr_float", 500);
#endif
   quickbook_results();
   return 0;
}

