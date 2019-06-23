// Synopsis: Simple point class
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>

#ifndef SEB_POINT_H
#define SEB_POINT_H

#include <vector>
#include "Seb_configure.h"

namespace SEB_NAMESPACE {
  
  template<typename Float>
  class Point
  // A simple class representing a d-dimensional point.
  {
  public: // types:
    typedef typename std::vector<Float>::const_iterator Const_iterator;
    typedef typename std::vector<Float>::iterator Iterator;
    
  public: // construction and destruction:
    
    Point(int d)
    // Constructs a d-dimensional point with undefined coordinates.
    : c(d)
    {
    }
    
    template<typename InputIterator>
    Point(int d,InputIterator first)
    // Constructs a d-dimensional point with Cartesian center
    // coordinates [first,first+d).
    : c(first,first+d)
    {
    }
    
  public: // access:
    
    const Float& operator[](unsigned int i) const
    // Returns a const-reference to the i-th coordinate.
    {
      SEB_ASSERT(0 <= i && i < c.size());
      return c[i];
    }
    
    Float& operator[](unsigned int i)
    // Returns a reference to the i-th coordinate.
    {
      SEB_ASSERT(0 <= i && i < c.size());
      return c[i];
    }
    
    Const_iterator begin() const
    // Returns a const-iterator to the first of the d Cartesian coordinates.
    {
      return c.begin();
    }
    
    Const_iterator end() const
    // Returns the past-the-end iterator corresponding to begin().
    {
      return c.end();
    }
    
  private: // member fields:
    std::vector<Float> c;       // Cartesian center coordinates
  };
  
} // namespace SEB_NAMESPACE

#endif // SEB_POINT_H
