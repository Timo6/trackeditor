// Spline.cc - a cubic spline interpolator.
//
//  Vamos Automotive Simulator
//  Copyright (C) 2001--2004 Sam Varner
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

#include "Spline.h"
#include "Numeric.h"

#include <cmath>
#include <cassert>

using namespace Vamos_Geometry;

// Construct an empty curve.
Spline::Spline (double first_slope, double last_slope) :
  m_first_slope (first_slope),
  m_last_slope (last_slope),
  m_calculated (false),
  m_slope (0.0)
{
}

// Construct a cuvre from an array of points.
Spline::Spline (const std::vector <Two_Vector>& points,
                double first_slope, double last_slope) :
  Interpolator (points),
  m_first_slope (first_slope),
  m_last_slope (last_slope),
  m_calculated (false),
  m_slope (0.0)
{
}

// Add a point to the curve.
void 
Spline::load (const Two_Vector& point)
{
  m_points.push_back (point);
  m_calculated = false;
}

// Add multiple points to the curve.
void 
Spline::load (const std::vector <Two_Vector>& points)
{
  for (std::vector <Two_Vector>::const_iterator it = points.begin ();
       it != points.end ();
       it++)
    {
      m_points.push_back (*it);
    }
  m_calculated = false;
}

// Remove all points from the curve.
void 
Spline::clear ()
{
  m_points.clear ();
  m_calculated = false;
}

// Set the slopes of the endpoints.
void
Spline::set_slope (double start_slope, double end_slope)
{
  m_first_slope = start_slope;
  m_last_slope = end_slope;
  m_calculated = false;
}

// Remove points with x > LIMIT.
void 
Spline::remove_greater (double limit)
{
  for (size_t size = 0; size < m_points.size (); size++)
    {
      if (m_points [size].x > limit)
        {
          m_points.resize (size);
          m_calculated = false;
          return;
        }
    }
}

// Scale all of the x values by FACTOR.
void 
Spline::scale (double factor)
{
  for (std::vector <Two_Vector>::iterator it = m_points.begin ();
       it != m_points.end ();
       it++)
    {
      it->x *= factor;
    }

  m_calculated = false;
}

// calculate() and interpolate() follow the discussion on cubic
// splines found in Numerical Recipes.  The implementation here is
// original. 

// Return the y value at the x value DISTANCE
double 
Spline::interpolate (double distance) const
{
  Interpolator::interpolate (distance);

  if (m_points.size () < 2)
    {
      m_slope = 0.0;
      m_second_derivative = 0.0;
      if (m_points.empty ())
        {
          return 0.0;
        }
      return m_points [0].y;
    }

  // calculate() only needs to be called once for a given set of
  // points.
  if (!m_calculated)
    calculate ();

  const size_t low = low_index (distance);
  const size_t high = low + 1;
  const double diff = m_points [high].x - m_points [low].x;

  // Evaluate the coefficients for the cubic spline equation.
  const double a = (m_points [high].x - distance) / diff;
  const double b = 1.0 - a;
  const double sq = diff*diff / 6.0;
  const double a2 = a*a;
  const double b2 = b*b;

  // Find the first derivitive.
  m_slope =
    (m_points [high].y - m_points [low].y)/diff
    - (3.0 * a2 - 1.0) / 6.0 * diff * m_second_deriv [low]
    + (3.0 * b2 - 1.0) / 6.0 * diff * m_second_deriv [high];

  m_second_derivative = 
    Vamos_Geometry::interpolate (distance, 
                                 m_points [low].x, m_second_deriv [low],
                                 m_points [high].x, m_second_deriv [high]);

  // Return the interpolated value.
  return a * m_points [low].y 
    + b * m_points [high].y 
    + a * (a2 - 1.0) * sq * m_second_deriv [low] 
    + b * (b2 - 1.0) * sq * m_second_deriv [high];
}

double 
Spline::slope (double distance) const
{
  // The slope is calculated and stored when interpolate() is called.
  interpolate (distance);
  return m_slope;
}

double
Spline::second_derivative (double distance) const
{
  // The slope is calculated and stored when interpolate() is called.
  interpolate (distance);
  return m_second_derivative;
}

// Calculate the coefficients for interpolation.
void 
Spline::calculate () const
{
  size_t n = m_points.size ();
  double* a = new double [n];
  double* b = new double [n];
  double* c = new double [n];
  double* r = new double [n];

  // Fill in the arrays that represent the tridiagonal matrix.
  // a [0] is not used. 
  double diff = m_points [1].x - m_points [0].x;
  b [0] = diff / 3.0;
  c [0] = diff / 6.0;
  r [0] = (m_points [1].y - m_points [0].y) / diff - m_first_slope;
    
  for (size_t i = 1; i < n - 1; i++)
    {
      double diff1 = m_points [i+1].x - m_points [i].x;
      double diff2 = m_points [i].x - m_points [i-1].x;

      a [i] = diff2 / 6.0;
      b [i] = (m_points [i+1].x - m_points [i-1].x) / 3.0;
      c [i] = diff1 / 6.0;
      r [i] = (m_points [i+1].y - m_points [i].y) / diff1
        - (m_points [i].y - m_points [i-1].y) / diff2;
    }

  diff = m_points [n-1].x - m_points [n-2].x;
  a [n-1] = diff / 6.0;
  b [n-1] = diff / 3.0;
  // c [n-1] is not used.
  r [n-1] = m_last_slope - (m_points [n-1].y - m_points [n-2].y) / diff;
    
  // Gauss-Jordan Elimination
  for (size_t i = 1; i < n; i++)
    {
      // Replace row i with row i - k * row (i-1) such that A_{i,i-1} = 0.0.
      double factor = a [i] / b [i-1];
      // A_{i,i-1} is not used again, so it need not be calculated.
      b [i] -= factor * c [i-1];
      // A_{i,i+1} is unchanged because A_{i-1,i+1} = 0.0.
      r [i] -= factor * r [i-1];
    }
  
  // Back-Substitution
  
  // Solve for y"[N].
  m_second_deriv.resize (n);
  m_second_deriv [n-1] = r [n-1] / b [n-1];
  for (int i = n - 2; i >= 0; i--)
    {
      // Use the solution for y"[i+1] to find y"[i].
      m_second_deriv [i] = (r [i] - c [i] * m_second_deriv [i+1]) / b [i];
    }
  
  delete [] r;
  delete [] c;
  delete [] b;
  delete [] a;

  m_calculated = true;
}

// Return the normal to the tanget at DISTANCE.
Two_Vector 
Spline::normal (double distance) const
{
  interpolate (distance);
  double theta = std::atan (m_slope);
  return Two_Vector (-std::sin (theta), std::cos (theta));
}

// Add 'delta' to all points.
void
Spline::shift (double delta)
{
  for (std::vector <Two_Vector>::iterator it = m_points.begin ();
	   it != m_points.end ();
	   it++)
	{
	  it->y += delta;
	}
}

//-----------------------------------------------------------------------------
Parametric_Spline::Parametric_Spline (double first_x_slope, double last_x_slope,
                                      double first_y_slope, double last_y_slope)
  : m_x (first_x_slope, last_x_slope),
    m_y (first_y_slope, last_y_slope)
{
}

void
Parametric_Spline::load (double parameter, const Two_Vector& point)
{
  m_x.load (Two_Vector (parameter, point.x));
  m_y.load (Two_Vector (parameter, point.y));
}

void
Parametric_Spline::clear ()
{
  m_x.clear ();
  m_y.clear ();
}

Two_Vector
Parametric_Spline::interpolate (double parameter) const
{
  return Two_Vector (m_x.interpolate (parameter),
                     m_y.interpolate (parameter));
}

size_t
Parametric_Spline::size () const
{
  assert (m_x.size () == m_y.size ());
  return m_x.size ();
}

Two_Vector 
Parametric_Spline::operator [] (size_t i) const
{
  return Two_Vector (m_x [i].y, m_y [i].y);
}

double
Parametric_Spline::parameter (size_t i) const
{
  return m_x [i].x;
}

//-----------------------------------------------------------------------------
Spline_Path::Spline_Path ()
  : m_segments (1)
{
}

Spline_Path::Spline_Path (const std::vector <Two_Vector>& points)
{
  load (points);
}

void
Spline_Path::load (const Two_Vector& point)
{
  m_segments.rbegin ()->load (point);
}

void
Spline_Path::load (const std::vector <Two_Vector>& points)
{
  Spline next;
  // Make the last point of the previous spline segment the first
  // point of the new one.
  if ((m_segments.size () > 1) || (m_segments [0].size () > 0))
    {
      const Spline& previous = *m_segments.rbegin ();
      next.load (previous [previous.size () - 1]);
    }
  next.load (points);
  if ((m_segments.size () == 1) && (m_segments [0].size () == 0))
    m_segments [0].load (points);
  m_segments.push_back (next);
}

void
Spline_Path::clear ()
{
  m_segments.resize (1);
  m_segments.begin ()->clear ();
}

void
Spline_Path::remove_greater (double limit)
{
  for (std::vector <Spline>::reverse_iterator it = m_segments.rbegin ();
       it != m_segments.rend ();
       it++)
    {
      if ((*it) [0].x <= limit)
        return it->remove_greater (limit);
    }
}

void
Spline_Path::scale (double factor)
{
  for (std::vector <Spline>::iterator it = m_segments.begin ();
       it != m_segments.end ();
       it++)
    {
      it->scale (factor);
    }
}

double
Spline_Path::interpolate (double dist) const
{
  std::vector <Spline>::const_reverse_iterator it;
  for (it = m_segments.rbegin (); it != m_segments.rend (); it++)
    {
      if ((*it) [0].x <= dist)
        return it->interpolate (dist);
    }
  return m_segments.begin ()->interpolate (dist);
}

Two_Vector
Spline_Path::normal (double dist) const
{
  std::vector <Spline>::const_reverse_iterator it;
  for (it = m_segments.rbegin (); it != m_segments.rend (); it++)
    {
      if ((*it) [0].x <= dist)
        return it->normal (dist);
    }
  return m_segments.begin ()->normal (dist);
}

void
Spline_Path::shift (double delta)
{
  for (std::vector <Spline>::iterator it = m_segments.begin ();
       it != m_segments.end ();
       it++)
    {
      it->shift (delta);
    }
}

size_t 
Spline_Path::size () const
{
  size_t total = 0;
  for (std::vector <Spline>::const_iterator it = m_segments.begin ();
       it != m_segments.end ();
       it++)
    {
      total += it->size ();
    }
  return total;
}

const Two_Vector& 
Spline_Path::operator [] (size_t i) const
{
  for (std::vector <Spline>::const_iterator it = m_segments.begin ();
       it != m_segments.end ();
       it++)
    {
      if (i < it->size ())
        return (*it) [i];
      i -= it->size ();
    }
  return m_segments [0][i];
}
