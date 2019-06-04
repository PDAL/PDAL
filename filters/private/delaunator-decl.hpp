/******************************************************************************
* MIT License
*
* Copyright (c) 2018 Volodymyr Bilonenko
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
****************************************************************************/

#include <vector>

/**
 * This is a re-declaration of the public interface of the
 * delaunator::Delaunator class found in
 * filters/private/delaunator.hpp .  This file exists to allow that
 * class to be included from more than one place in the code (The
 * existence of non-inlined functions within that header prevents it
 * from being included more than once [multipple definition errors]).
 */
namespace delaunator {
  class Delaunator {
  public:
    std::vector<double> const& coords;
    std::vector<std::size_t> triangles;
    std::vector<std::size_t> halfedges;
    std::vector<std::size_t> hull_prev;
    std::vector<std::size_t> hull_next;
    std::vector<std::size_t> hull_tri;
    std::size_t hull_start;

    Delaunator(std::vector<double> const& in_coords);

    double get_hull_area();
  };
}
