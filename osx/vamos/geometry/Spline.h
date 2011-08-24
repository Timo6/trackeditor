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

#ifndef _SPLINE_H_
#define _SPLINE_H_

#include "Interpolator.h"
#include "Two_Vector.h"

#include <vector>

namespace Vamos_Geometry
{
  class Spline : public Interpolator
  {
  public:
    // Construct an empty curve.
    Spline (double first_slope = 0.0, 
            double last_slope = 0.0);

    // Construct a cuvre from an array of points.
    Spline (const std::vector <Two_Vector>& points,
            double first_slope = 0.0, 
            double last_slope = 0.0);

    // Add a point to the curve.
    virtual void load (const Two_Vector& point);
	virtual void load (double x, double y) { load (Two_Vector (x, y)); }

    // Add multiple points to the curve.
    virtual void load (const std::vector <Two_Vector>& points);

	// Replace all points on the curve.
	virtual void replace (const std::vector <Two_Vector>& points)
    {
      clear ();
      load (points);
    }

    // Remove all points from the curve.
    virtual void clear ();

    // Set the slopes of the endpoints.
    void set_slope (double start_slope, double end_slope);

    // Remove points with x > LIMIT.
    virtual void remove_greater (double limit);

    // Scale all of the x values by FACTOR.
    virtual void scale (double factor);

    // Return the y value at the x value DIST
    virtual double interpolate (double dist) const;

    double slope (double distance) const;

    double second_derivative (double distance) const;

    // Return the normal to the tanget at DIST.
    virtual Two_Vector normal (double dist) const;

    // Add 'delta' to all points.
    virtual void shift (double delta);

    // Return the number of control points.
    virtual size_t size () const { return m_points.size (); }

    virtual const Two_Vector& operator [] (size_t i) const { return m_points [i]; }

  private:
    // The array of calculated second derivatives.
    mutable std::vector <double> m_second_deriv;

    // The first derivative of the spline at the first point.
    double m_first_slope;

    // The first derivative of the spline at the last point.
    double m_last_slope;

    // True if the second derivatives have been calculated.
    mutable bool m_calculated;

    // The 1st and 2nd derivatives at the interpolated point
    // calculated during the last call to interpolate().
    mutable double m_slope;
    mutable double m_second_derivative;

    // Calculate the coefficients for interpolation.
    void calculate () const;

    // The segment index from the previous interpolation.
    mutable size_t m_last_index;
  };

  class Parametric_Spline
  {
  public:
    Parametric_Spline (double first_x_slope, double last_x_slope,
                       double first_y_slope, double last_y_slope);

	// Add a point to the curve.
	void load (double parameter, const Two_Vector& point);
	void load (double parameter, double x, double y) 
    { load (parameter, Two_Vector (x, y)); }

	// Remove all points from the curve.
	void clear ();

	// Return the point at PARAMETER.
	Two_Vector interpolate (double parameter) const;

    // Return the number of control points.
    size_t size () const;

    Two_Vector operator [] (size_t i) const;

    double parameter (size_t i) const;

  private:
    Spline m_x;
    Spline m_y;
  };

  class Spline_Path : public Interpolator
  {
  public:
    // Construct an empty path.
    Spline_Path ();

    // Construct a path with one spline segment.
    Spline_Path (const std::vector <Two_Vector>& points);

	// Add a point to the last spline segment.
	virtual void load (const Two_Vector& point);
	virtual void load (double x, double y) { load (Two_Vector (x, y)); }

    // Add a spline segment to the path.
    virtual void load (const std::vector <Two_Vector>& points); 

	// Replace all points on the curve.
	virtual void replace (const std::vector <Two_Vector>& points)
    {
      clear ();
      load (points);
    }

    // Remove all points from the path.
    virtual void clear ();

	// Remove points with x > LIMIT.
	virtual void remove_greater (double limit);

	// Scale all of the x values by FACTOR.
	virtual void scale (double factor);

	// Return the y value at the x value DIST
	virtual double interpolate (double dist) const;

	// Return the normal to the tanget at DIST.
	virtual Two_Vector normal (double dist) const;

    // Add 'delta' to all points.
    virtual void shift (double delta);

    // Return the number of control points.
    virtual size_t size () const;

    virtual const Two_Vector& operator [] (size_t i) const;

  private:
    std::vector <Spline> m_segments;
  };
}

#endif
