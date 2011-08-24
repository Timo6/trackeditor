//  Vamos Automotive Simulator
//  Copyright (C) 2008 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#include <vector>
#include <cassert>

namespace Vamos_Geometry
{
  template <typename T>
  class Circular_Buffer
  {
  public:
    Circular_Buffer (size_t size);

    size_t size () { return m_used; }
    void push_back (T);
    const T& operator [] (size_t index);

  private:
    std::vector <T> m_elements;
    size_t m_used;
    size_t m_start_index;
  };

  template <typename T>
  Circular_Buffer <T>::Circular_Buffer (size_t size)
    : m_used (0),
      m_start_index (0)
  {
    m_elements.resize (size);
  }

  template <typename T>
  void
  Circular_Buffer <T>::push_back (T element)
  {
    m_elements [m_start_index] = element;
    m_used = std::min (m_used + 1, m_elements.size ());
    m_start_index = ((m_start_index + 1) % m_elements.size ());
  }

  template <typename T>
  const T&
  Circular_Buffer <T>::operator [] (size_t index)
  {
    assert (index < m_used);
    return m_elements [(m_start_index + index) % m_used];
  }
}

#endif // not _CIRCULAR_BUFFER_H_
