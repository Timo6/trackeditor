//  Robot_Driver.h - a computer-controlled driver
//
//	Vamos Automotive Simulator
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

#ifndef _ROBOT_DRIVER_H_
#define _ROBOT_DRIVER_H_

#include "Controls.h"
#include "Driver.h"
#include "../geometry/PID.h"
#include "../geometry/Spline.h"

namespace Vamos_Body
{
  class Car;
}

namespace Vamos_Geometry
{
  struct Three_Vector;
}

namespace Vamos_Track
{
  class Strip_Track;
  class Road;
  class Gl_Road_Segment;
}

namespace Vamos_World
{
  struct Car_Information;
  class Braking_Equation;

  class Braking
  {
  public:
    Braking ();
    ~Braking ();
    bool is_braking () const { return m_is_braking; }
    void start (double start, 
                double length, 
                double track_length,
                const Braking_Equation& equation);
    void end ();
    bool check_done_braking (double distance);
    double maximum_speed (double distance) const;

  private:
    double distance_from_start (double distance) const;
    bool past_end (double distance) const;

    double m_start;
    double m_length;
    double m_track_length;
    bool m_is_braking;
    Braking_Equation* mp_braking_equation;
  };

  class Robot_Racing_Line
  {
  public:
    Robot_Racing_Line (const Vamos_Track::Road& road,
                       const std::vector <double>& lateral_acceleration);
    Vamos_Geometry::Three_Vector target (double distance, double lead) const;
    Vamos_Geometry::Two_Vector braking_margin (const Braking_Equation* braking,
                                               double distance,
                                               double lane_shift,
                                               double balance,
                                               double lead,
                                               double air_density_factor) const;
    double maximum_speed (double distance, 
                          double lane_shift,
                          double balance, 
                          double lead,
                          double air_density_factor) const;

  private:
    const Vamos_Track::Road* mp_road;
    std::vector <double> m_lateral_acceleration;
  };

  class Robot_Driver : public Driver
  {
  public:
    Robot_Driver (Vamos_Body::Car* car_in, Vamos_Track::Strip_Track* track_in);
    ~Robot_Driver ();

    void interact (bool do_interact) { m_interact = do_interact; }
    void set_cars (const std::vector <Car_Information>* cars);

    // Let the driver know about the slipstream.
    virtual void set_air_density_factor (double factor) { m_air_density_factor = factor; }

    // Step the driver forward in time.
    virtual void propagate (double time_step);

    virtual void draw ();

  private:
    enum State
      {
        PARKED,
        STARTING,
        DRIVING
      };

    // The point that is kept on the racing line is this far ahead of the
    // car.
    double target_distance () const;

    // The position of the point that the driver tries to keep on the racing line.
    Vamos_Geometry::Three_Vector target_position () const;

    double target_slip_ratio () const;
    double target_slip_angle () const;

    void detect_collisions (const Vamos_Geometry::Three_Vector& track_position);
    void steer (const Vamos_Geometry::Three_Vector& track_position);
    Vamos_Geometry::Three_Vector lane_shift (const Vamos_Geometry::Three_Vector& target);
    void shift ();
    void accelerate (const Vamos_Geometry::Three_Vector& track_position);
    void set_steering (double angle);
    void set_gas (double gas);
    void set_brake (double brake);

    const Vamos_Track::Gl_Road_Segment& current_segment () const;
    const Vamos_Track::Gl_Road_Segment& next_curve (int index = -1) const;
    const Vamos_Track::Gl_Road_Segment& previous_curve (int index = -1) const;
    double longitudinal_slip () const;
    double transverse_slip () const;
    double total_slip () const;
    double speed () const;
    Vamos_Geometry::Two_Vector velocity_margin (double distance);

    // Return the speed-dependent performance parameters.
    double deceleration () const;

    const std::vector <Car_Information>* mp_cars;

    Vamos_Geometry::PID m_steer_control;
    Vamos_Geometry::PID m_gas_control;
    Vamos_Geometry::PID m_brake_control;

    // Parameters from the car.
    double m_slip_ratio;
    std::vector <double> m_deceleration;

    size_t m_road_index;
    size_t m_segment_index;//!! should use world's car info
    size_t m_target_segment;
    Vamos_Track::Strip_Track* mp_track;
    bool m_reset;
    double m_shift_time;
    State m_state;
    double m_state_time;
    double m_time_step;
    double m_lane_shift;
    // The fraction from the racing line to the edge of the track of
    // the path the car is currently following. 1 -> left edge, -1 ->
    // right edge, 0 -> on racing line.
    double m_lane_shift_timer;
    double m_air_density_factor;
    bool m_interact;

    Braking m_braking;
    Braking_Equation* mp_braking_equation;
    Robot_Racing_Line m_racing_line;
  };
}

#endif // not _ROBOT_DRIVER_H_
