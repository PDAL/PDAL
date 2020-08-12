/*
Copyright 2016 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A local copy of the license and additional notices are located with the
source distribution at:

http://github.com/Esri/lepcc/

Contributors:  Ronald Poirrier
*/

#pragma once
#include <cassert>
#include <type_traits>

#ifndef _NOEXCEPT
#define _NOEXCEPT	noexcept //c++ 11
#endif

#define LEPCC_ASSERT assert

namespace lepcc
{
  //! Wrap constant pointer to raw memory to provide basic safety (bound checks)
  template< class T >
  class const_array
  {
  public:
    typedef const T* const_iterator;
    const_array(const T* data, size_t size) : m_data(data), m_size(size) {}
    const T& operator[](size_t i) const _NOEXCEPT{ LEPCC_ASSERT(i >= 0 && i < m_size); return m_data[i]; }
    const T* data() const _NOEXCEPT{ return m_data; }
    size_t size() const  _NOEXCEPT{ return m_size; }

    const T&       front() const _NOEXCEPT{ return operator[](0); }
    const T&       back() const  _NOEXCEPT{ return operator[](m_size - 1); }
    const_iterator begin() const _NOEXCEPT{ return &front(); }
    const_iterator end() const   _NOEXCEPT{ return &back() + 1; }

  private:
    const T*  m_data;
    size_t    m_size;
  };

}    // namespace

