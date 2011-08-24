//	Vamos Automotive Simulator
//  Copyright (C) 2001--2003 Sam Varner
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

#ifndef _ROAD_SEGMENT_H_
#define _ROAD_SEGMENT_H_

#include "../geometry/Conversions.h"
#include "../geometry/Linear_Interpolator.h"
#include "../geometry/Rectangle.h"
#include "../geometry/Spline.h"
#include "../geometry/Three_Vector.h"
#include "../geometry/Two_Vector.h"

#include <string>
#include <vector>
#include <complex>

namespace Vamos_Track
{
  typedef std::vector <Vamos_Geometry::Two_Vector> TPoints;

  //===========================================================================
  class Kerb
  {
	std::vector <Vamos_Geometry::Two_Vector> m_points;
	Vamos_Geometry::Linear_Interpolator m_profile;
	double m_start;
	double m_start_transition_length;
	double m_start_transition_width;
	double m_end;
	double m_end_transition_length;
	double m_end_transition_width;
	bool m_full_length;

  public:
	Kerb (const std::vector <Vamos_Geometry::Two_Vector>& profile,
		  double start, 
		  double start_transition_length,
		  double start_transition_width,
		  double end, 
		  double end_transition_length,
		  double end_transition_width);

	void set_length (double length);

	double transition_start () const { return m_start; }
	double transition_end () const { return m_end; }
	double start () const { return m_start + m_start_transition_length; }
	double end () const { return m_end - m_end_transition_length; }
	double start_transition_width () const { return m_start_transition_width; }
	double end_transition_width () const { return m_end_transition_width; }
	double width () const;
	bool on_kerb (double dist) const;
	double elevation (double along, double from_inside);
	double angle (double along, double from_inside);
	const Vamos_Geometry::Two_Vector& point (size_t substrip) const;
	size_t substrips () const { return m_points.size () - 1; }
  };

  //===========================================================================
  class Banking
  {
    TPoints m_angles;
    double m_start_angle;
    double m_end_angle;
    double m_pivot_from_center;
    Vamos_Geometry::Spline* mp_bank_curve;

  public:
    Banking ();
    ~Banking ();

    void set (const TPoints& angles, double end_angle, double pivot_from_center);
    double angle (double along) const;
    double height (double along, double from_center) const;
    void set_start (double start_angle, double length);
    double start_angle () const { return m_start_angle; }
    double end_angle () const { return m_end_angle; }
  };

  //=============================================================================
  class Pit_Lane_Transition
  {
  public:
    Pit_Lane_Transition ();

    void set_merge (Vamos_Geometry::Direction direction, 
                    Vamos_Geometry::Direction side,
                    double split_or_join, 
                    double merge,
                    double angle);
    void set_width (double pit_width, 
                    double left_shoulder, 
                    double right_shoulder);
    Vamos_Geometry::Direction direction () const { return m_direction; }
    Vamos_Geometry::Direction side () const { return m_side; }
    double merge () const { return active () ? m_merge : 0.0; }
    double split_or_join () const { return active () ? m_split_or_join : 0.0; }
    double width (Vamos_Geometry::Direction side, double distance, bool narrow) const;
    double shoulder_width () const { return m_pit_shoulder_width; }
    void scale (double factor);
    double angle () const { return m_angle; }
    double skew (double length) const { return length / cos (m_angle); }
    // Scale length to account for the skew.

    bool active () const { return m_merge_is_set && m_width_is_set; }

  private:
    Vamos_Geometry::Direction m_direction;
    Vamos_Geometry::Direction m_side;
    double m_split_or_join;
    double m_merge;
    double m_angle;
    double m_pit_width;
    double m_pit_shoulder_width;

    bool m_merge_is_set;
    bool m_width_is_set;
  };

  //===========================================================================
  class Road_Segment
  {
    friend class Segment_Iterator;

  public:
    Road_Segment (double length,
                  double radius,
                  double left_width,
                  double right_width,
                  double left_road_width,
                  double right_road_width);

	virtual ~Road_Segment ();

	double length () const { return m_length; }
	void set_length (double new_length);

	double radius () const { return m_radius; }
    bool is_straight () const { return m_radius == 0.0; }

    double arc () const 
    { return (m_radius == 0.0) ? 0.0 : m_length / m_radius; }
    void set_arc (double new_arc) { set_length (m_radius * new_arc); }

    Vamos_Geometry::Three_Vector center_of_curve () const;

	double left_width (double distance, bool narrow = false) const;
	double right_width (double distance, bool narrow = false) const;
    double width (double distance) const 
    { return left_width (distance) + right_width (distance); }
	double left_road_width_no_pit (double distance) const;
	double right_road_width_no_pit (double distance) const;
	double left_road_width (double distance, bool narrow = false) const;
	double right_road_width (double distance, bool narrow = false) const;
	double left_wall_height () const { return m_left_wall_height; }
	double right_wall_height () const { return m_right_wall_height; }
    double left_racing_line_width (double distance) const;
    double right_racing_line_width (double distance) const;

    void set_left_width (double distance, double width);
    void set_right_width (double distance, double width);
    void set_left_road_width (double distance, double width);
    void set_right_road_width (double distance, double width);
    void set_racing_line_adjustment (double across, double distance);
    void set_racing_line_margin (double margin);

    void set_wall_heights (double left_height, double right_height);

    void set_start_skew (double skew) { m_start_skew = skew; }
    void set_end_skew (double skew) { m_end_skew = skew; }
    double start_skew () const { return m_start_skew; }
    double end_skew () const { return m_end_skew; }

	double elevation (double along, double from_center) const;
	void build_elevation (Vamos_Geometry::Spline* elevation,
						  double start_distance);

    // A convenience function for getting the elevation at a point in
    // world coordinates.  Use 'coordinates()' if you need the x and y
    // track coodinates as well.
    double world_elevation (const Vamos_Geometry::Three_Vector& world_position) const;

    const Banking& banking () const { return m_banking; }

    // Return the normal vector at the given location.
	Vamos_Geometry::Three_Vector normal (double along, 
                                         double from_center, 
                                         const Vamos_Geometry::Three_Vector& bump) const;

    // Return the normal vector ignoring bumpiness.
	Vamos_Geometry::Three_Vector normal (double along, double from_center) const;

	Vamos_Geometry::Three_Vector 
    barrier_normal (double along, 
                    double from_center,
                    const Vamos_Geometry::Three_Vector& bump) const;

	Vamos_Geometry::Three_Vector barrier_normal (double along, double from_center) const;

	Vamos_Geometry::Three_Vector end_coords () const;
	Vamos_Geometry::Three_Vector start_coords () const 
	{ return m_start_coords; } 

	double angle (double along) const 
    { return m_start_angle + arc () * along / m_length; }

    void set_kerb (Kerb* kerb, Vamos_Geometry::Direction side);

	void scale (double factor);

    void set_pit_lane (Vamos_Geometry::Direction direction,
                       Vamos_Geometry::Direction side,
                       double split_or_join, 
                       double merge,
                       double angle);
    void set_pit_width (double width, 
                        double left_shoulder, 
                        double right_shoulder);
    double pit_width () const;
    bool on_pit_merge (double distance, double from_center) const;

	virtual void set_start (const Vamos_Geometry::Three_Vector& start_coords, 
                            double start_distance,
                            double start_angle, 
                            double start_bank,
                            const std::vector <double>& texture_offsets);

	double start_angle () const { return m_start_angle; }
	double end_angle () const { return m_start_angle + arc (); }
    double pit_angle () const { return angle (m_pit.split_or_join ()); }

	double start_distance () const { return m_start_distance; }
	double end_distance () const { return m_start_distance + m_length; }

	void last_segment (bool last) { m_last_segment = last; }

    // Do the world-to-track coordinate transformation.
	double coordinates (const Vamos_Geometry::Three_Vector& world_pos,
						Vamos_Geometry::Three_Vector& track_pos) const;

    // Do the track-to-world coordinate transformation.
    Vamos_Geometry::Three_Vector position (double along, 
                                           double from_center) const;

    const Pit_Lane_Transition& pit () const;

    void narrow (Vamos_Geometry::Direction side, double delta_width);
    // Subtract delta_width from this segment's width.

    void set_racing_line_curvature_factor (double factor)
    { m_racing_line_curvature_factor = factor; }

    double racing_line_curvature_factor () const 
    { return m_racing_line_curvature_factor; }

  protected:
    double kerb_width (Vamos_Geometry::Direction side, double along) const;
    void set_banking (const TPoints& bank_points, 
                      double end_angle, 
                      double pivot)
    { m_banking.set (bank_points, end_angle, pivot); }

    void set_elevation_points (const TPoints& elevation)
    { m_elevation_points = elevation; }
    void set_widths (const TPoints& right, const TPoints& right_road,
                     const TPoints& left_road, const TPoints& left);
    double pit_road_connection () const;

    bool is_last_segment () const { return m_last_segment; }

  private:
	// The length of the centerline of the segment.
	double m_length;
	// The radius of curvature; 0.0 for a straight segment. 
	double m_radius;

	// The distance from the center line to the walls.
	Vamos_Geometry::Linear_Interpolator m_left_width;
	Vamos_Geometry::Linear_Interpolator m_right_width;

	// The distance from the center line to the edges of the track.
	Vamos_Geometry::Linear_Interpolator m_left_road_width;
	Vamos_Geometry::Linear_Interpolator m_right_road_width;

    double m_racing_line_adjustment;
    double m_racing_line_adjustment_distance;

	// The heights of the walls.
	double m_left_wall_height;
	double m_right_wall_height;

    double m_start_skew;
    double m_end_skew;

	TPoints m_elevation_points;
	Vamos_Geometry::Spline* mp_elevation_curve;

    Banking m_banking;

	Kerb* mp_left_kerb;
	Kerb* mp_right_kerb;

    Pit_Lane_Transition m_pit;

    // The distance along the track where this segment starts.
	double m_start_distance;
	double m_start_angle;
	Vamos_Geometry::Three_Vector m_start_coords;

	// True if this is the last segment of a circuit.
	bool m_last_segment;

    double m_racing_line_curvature_factor;

	Road_Segment (const Road_Segment& segment);

    void scale_widths (double factor);

    // Find the angle from the beginning of a curve.
    double get_curve_angle (const Vamos_Geometry::Three_Vector& position, 
                            double across) const;

    // How far is the position off the end (+ve) or beginning (-ve) of
    // the segment?  Return 0 if the position is on the segment.
    double off_track_distance (const Vamos_Geometry::Three_Vector& 
                               track_position) const;

    double extra_road_width (Vamos_Geometry::Direction side, 
                             double distance,
                             bool narrow) const;
  };
}

#endif // not _ROAD_SEGMENT_H_
