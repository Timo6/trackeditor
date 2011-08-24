//  Robot_Driver.cc - a computer-controlled driver
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

#include "Robot_Driver.h"
#include "../body/Car.h"
#include "../body/Wheel.h"
#include "../geometry/Calculations.h"
#include "../geometry/Constants.h"
#include "../geometry/Numeric.h"
#include "../geometry/Three_Vector.h"
#include "../track/Strip_Track.h"
#include "World.h"

#include <limits>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Track;
using namespace Vamos_World;

//-----------------------------------------------------------------------------
namespace Vamos_World
{
  // Function object that returns speed as a function of distance
  // under braking.
  class Braking_Equation
  {
  public:
    // Set the equation's paramters.
    Braking_Equation (double deceleration, double drag, double speed);

    // Return the speed at 'distance'.
    double operator () (double distance) const;

  private:
    // Target deceleration rate (m^2/s).
    double m_deceleration;
    // The square of the initial velocity (m^2/s^2).
    double m_v_squared;
    // Aerodynamic drag factor (1/m).  Drag force is mv^2'm_drag'
    double m_drag;
  };
}

// Set the equation's paramters.
Braking_Equation::Braking_Equation (double deceleration, 
                                    double drag, 
                                    double speed)
  : m_deceleration (deceleration),
    m_v_squared (square (speed)),
    m_drag (drag / m_v_squared)
{
}

// Return the speed at 'distance'.
double
Braking_Equation::operator () (double distance) const
{
  double radicand;
  if (std::abs (m_drag) > 1.0e-9)
    {
      radicand = ((m_v_squared - m_deceleration / m_drag) 
                  * exp (2.0 * m_drag * distance))
        + m_deceleration / m_drag;
    }
  else
    {
      // This is the low-drag limit of the general function above,
      // but we can't just plug in drag=0 there.
      radicand = m_v_squared - 2.0 * m_deceleration * distance;
    }
  return (radicand < 0.0) ? 0.0 : std::sqrt (radicand);
}

//-----------------------------------------------------------------------------
Robot_Driver::Robot_Driver (Car* car_in, Strip_Track* track_in)
 : Driver (car_in),
   mp_cars (0),
   m_steer_control (0.5, 0.0, 0.03),
   m_gas_control (1.0, 0.0, 0.0),
   m_brake_control (0.3, 0.0, 0.0),
   m_slip_ratio (car_in->get_robot_parameters ().slip_ratio),
   m_deceleration (car_in->get_robot_parameters ().deceleration),
   m_road_index (0),
   m_segment_index (0),
   m_target_segment (0),
   mp_track (track_in),
   m_shift_time (0.0),
   m_state (PARKED),
   m_state_time (0.0),
   m_time_step (1.0),
   m_lane_shift (0.0),
   m_lane_shift_timer (0.0),
   m_air_density_factor (1.0),
   m_interact (true),
   mp_braking_equation (0),
   m_racing_line (mp_track->get_road (m_road_index),
                  car_in->get_robot_parameters ().lateral_acceleration)
{
}

Robot_Driver::~Robot_Driver ()
{
  delete mp_braking_equation;
}

void
Robot_Driver::set_cars (const std::vector <Car_Information>* cars)
{
  mp_cars = cars;
}

// Step the driver forward in time.
void 
Robot_Driver::propagate (double time_step) 
{
  m_time_step = time_step;
  if (m_lane_shift != 0.0)
    m_lane_shift_timer += time_step;

  switch (m_state)
    {
    case PARKED:
      set_brake (1.0);
      mp_car->shift (0);
      mp_car->disengage_clutch (0.0);
      if (mp_car->engine ()->rotational_speed () < mp_car->engine ()->stall_speed ())
        mp_car->start_engine ();
      set_gas (0.0);

      m_state_time += time_step;
      if (m_state_time > 1.0)
        {
          set_brake (0.0);
          m_state_time = 0.0;
          m_state = STARTING;
        }
      return;

    case STARTING:
      if (m_state_time == 0.0)
        {
          mp_car->engage_clutch (3.0);
          mp_car->shift (1);
        }

      m_state_time += time_step;
      if (m_state_time > 3.0)
        {
          m_state_time = 0.0;
          m_state = DRIVING;
        }
      break;

    case DRIVING:
      break;
    }

  const Three_Vector track_position = 
    mp_track->track_coordinates (mp_car->chassis ().position (), 
                                 m_road_index, 
                                 m_segment_index);

  steer (track_position);
  shift ();
  accelerate (track_position);

  // Detect collisions last since we may override steering and
  // braking.
  if (m_interact)
    detect_collisions (track_position);
}

void
Robot_Driver::draw ()
{
  // Uncomment to draw a point where the driver is aiming.
  // const Three_Vector ahead = mp_track->track_coordinates (mp_car->center_position (),
  //                                                         m_road_index, 
  //                                                         m_segment_index);
  // glLoadIdentity ();
  // glPointSize (8.0);
  // glBegin (GL_POINTS);
  // const Three_Vector target (target_position ());
  // const Three_Vector line = 
  //   lane_shift (m_racing_line.target (ahead.x, target_distance ()));
  // glColor3d (0.0, 0.8, 0.0);
  // glVertex3d (target.x, target.y, current_segment ().world_elevation (target) + 0.1);
  // glColor3d (8.0, 0.0, 0.0);
  // glVertex3d (line.x, line.y, current_segment ().world_elevation (line) + 0.1);
  // glEnd ();
}

// The point that is kept on the racing line is this far ahead of the
// car.
double 
Robot_Driver::target_distance () const
{
  return 2.0 * mp_car->length () + 0.2 * speed ();
}

// The position of the point that the driver tries to keep on the racing line.
Three_Vector
Robot_Driver::target_position () const
{
  return mp_car->chassis ().transform_to_world 
    (mp_car->center () + Three_Vector (target_distance (), 0.0, 0.0));
}

// Get the segment that the car is currently on.
const Gl_Road_Segment&
Robot_Driver::current_segment () const
{
  return *mp_track->get_road (m_road_index).segments ()[m_segment_index];
}

Three_Vector
Robot_Driver::lane_shift (const Three_Vector& target)
{
  const Road& road = mp_track->get_road (m_road_index);
  const Three_Vector track = road.track_coordinates (target, m_target_segment);

  const double across = m_lane_shift 
    * (m_lane_shift > 0.0
       ? road.racing_line ().left_width (road, track.x) - track.y
       : road.racing_line ().right_width (road, track.x) + track.y);

  const Gl_Road_Segment& segment = *road.segments ()[m_target_segment];
  double along = wrap (track.x, road.length ());
  Three_Vector world = road.position (along, track.y + across, segment);
  world.z = 0.0;
  return world;
}

void
Robot_Driver::steer (const Three_Vector& track_position)
{
  const Three_Vector line = m_racing_line.target (track_position.x, target_distance ());
  const Three_Vector shifted = lane_shift (line);
  Three_Vector goal = shifted - mp_car->center_position ();
  goal.z = 0.0;
  Three_Vector target = target_position () - mp_car->center_position ();
  target.z = 0.0;

  set_steering (target.cross (goal).z);

  // Return to the racing line if shifting doesn't make much of a
  // difference.
  if ((line - shifted).magnitude () < mp_car->width ())
    m_lane_shift = 0.0;
}

void
Robot_Driver::shift ()
{
  if (m_state == STARTING)
    return;

  // Gear Selection
  int gear = mp_car->transmission ()->gear ();
  double omega = mp_car->engine ()->rotational_speed ();
  double up_omega = omega
    * mp_car->transmission ()->gear_ratio (gear + 1)
    / mp_car->transmission ()->gear_ratio (gear);
  double down_omega = omega
    * mp_car->transmission ()->gear_ratio (gear - 1)
    / mp_car->transmission ()->gear_ratio (gear);

  double power = mp_car->engine ()->power (1.0, omega);
  double up_power = mp_car->engine ()->power (1.0, up_omega);
  double down_power = mp_car->engine ()->power (1.0, down_omega);

  const double slip_threshold = (gear > 2 ? 0.7 : 0.4) * target_slip_ratio ();

  if (mp_car->clutch ()->engaged ()
      && (omega < 1.1 * mp_car->engine ()->stall_speed ()))
    {
      m_state = PARKED;
    }
  else if (m_shift_time > 0.15)
      m_shift_time = 0.0;
  else if (m_shift_time != 0.0)
      m_shift_time += m_time_step;
  else if ((gear < mp_car->transmission ()->forward_gears ())
           && mp_car->clutch ()->engaged () 
           && power < up_power)
    {
      m_shift_time = m_time_step;
      mp_car->shift_up ();
    }
  else if (mp_car->clutch ()->engaged ()
           && total_slip () < slip_threshold
           && 1.05 * power < down_power)
    // The multiplier on 'power' makes the driver less eager to downshift 
    // and avoids upsetting the balance.
    {
      m_shift_time = m_time_step;
      mp_car->shift_down ();
    }
}

double
Robot_Driver::longitudinal_slip () const
{
  return abs_max (mp_car->wheel (0)->slip ().x,
                  mp_car->wheel (1)->slip ().x,
                  mp_car->wheel (2)->slip ().x,
                  mp_car->wheel (3)->slip ().x);
}

double
Robot_Driver::target_slip_ratio () const
{
  return m_slip_ratio;
}

double
Robot_Driver::target_slip_angle () const
{
  return abs_max (mp_car->wheel (0)->peak_slip_angle (),
                  mp_car->wheel (1)->peak_slip_angle (),
                  mp_car->wheel (2)->peak_slip_angle (),
                  mp_car->wheel (3)->peak_slip_angle ());
}

double
Robot_Driver::total_slip () const
{
  return Three_Vector (longitudinal_slip (), transverse_slip (), 0.0).magnitude ();
}

double
Robot_Driver::transverse_slip () const
{
  return abs_max (mp_car->wheel (0)->slip ().y,
                  mp_car->wheel (1)->slip ().y,
                  mp_car->wheel (2)->slip ().y,
                  mp_car->wheel (3)->slip ().y);
}

double
Robot_Driver::speed () const
{
  return mp_car->chassis ().cm_velocity ().magnitude();
}

void
Robot_Driver::accelerate (const Three_Vector& track_position)
{
  m_braking.check_done_braking (track_position.x);

  Two_Vector margin = velocity_margin (track_position.x);
  if (!m_braking.is_braking () && (margin.x < 0.0))
    {
      m_braking.start (track_position.x, 
                       margin.y, 
                       mp_track->get_road (m_road_index).length (),
                       *mp_braking_equation);
      // This call may cause is_braking() to return true below.
    }

  if (m_braking.is_braking ())
    {
      const double diff = speed () - m_braking.maximum_speed (track_position.x);
      set_gas (0.0);
      set_brake (std::min (diff, target_slip_ratio () - total_slip ()));
    }
  else
    {
      double diff = std::min (target_slip_ratio () - total_slip (),
                              0.1 * (m_racing_line.maximum_speed (track_position.x, 
                                                                  m_lane_shift,
                                                                  mp_car->balance (),
                                                                  target_distance (),
                                                                  m_air_density_factor)
                                      - speed ()));
      if (m_state == STARTING)
        {
          diff = std::min (diff,
                           mp_car->engine ()->peak_engine_speed ()
                           - mp_car->engine ()->rotational_speed ());
        }
      set_gas (diff);
      set_brake (0.0);
    }
}

void
Robot_Driver::set_steering (double angle)
{
  const double max_angle = 1.5 * target_slip_angle ();
  mp_car->steer (clip (m_steer_control.output (angle, m_time_step), 
                       -max_angle, 
                       max_angle),
                 0.0, 
                 true);
  // true => direct steering: Ignore non-linearity and speed-sensitivity.
}

void
Robot_Driver::set_gas (double gas)
{
  mp_car->gas (clip (m_gas_control.output (gas, m_time_step), 0.0, 1.0));
}

void
Robot_Driver::set_brake (double brake)
{
  mp_car->brake (clip (m_brake_control.output (brake, m_time_step), 0.0, 1.0));
}

// Check for potential collisions.
void
Robot_Driver::detect_collisions (const Three_Vector& track_position)
{
  if (mp_cars == 0) return;

  // Ignore cars that won't make contact for this many seconds.
  double minimum_crash_time = 10.0;
  double minimum_distance = 10.0;
  double cross = 0.0;

  // Loop through the other cars.
  for (std::vector <Car_Information>::const_iterator it = mp_cars->begin ();
       it != mp_cars->end ();
       it++)
    {
      if (it->car == mp_car)
        continue;

      size_t segment = it->segment_index;
      const Three_Vector other_track_position = 
        mp_track->track_coordinates (it->car->chassis ().position (), 
                                     m_road_index, 
                                     segment);
      if (!is_in_range (other_track_position.x - track_position.x, 
                        -0.5 * mp_car->length (), 5.0 * mp_car->length ()))
        {
          //!! handle wraping around the track.

          // Ignore cars that are far away.
          continue;
        }

      const Three_Vector r1 = mp_car->chassis ().cm_position ();
      const Three_Vector v1 = mp_car->chassis ().cm_velocity ();
      const Three_Vector r2 = it->car->chassis ().cm_position ();
      const Three_Vector v2 = it->car->chassis ().cm_velocity ();

      // The how close the other car will come if both move at
      // constant velocity.
      const double closest = closest_approach (r1, v1, r2, v2);
      const double closing = closing_speed (r1, v1, r2, v2);

      if ((closest < 3.0 * mp_car->length ()) && (closing > 0.0))
        {
          const Three_Vector delta_r = r2 - r1;
          const double distance = delta_r.magnitude ();
          minimum_crash_time = std::min (minimum_crash_time, distance / closing);
          minimum_distance = std::min (minimum_distance, closest);
          cross = v1.cross (delta_r).z;
        }
    }

  static const double shift_step = 1.0;
  if ((minimum_crash_time < 3.0))// || (minimum_distance < 2.0 * mp_car->length ()))
  {
    if (total_slip () < 10.8)//!
      {
        if (cross < 0.0)
          m_lane_shift = std::min (1.0, m_lane_shift + shift_step);
        else
          m_lane_shift = std::max (-1.0, m_lane_shift - shift_step);
      }
    else
      set_gas (0.0);

    if (std::abs (m_lane_shift) == 1.0)
      set_gas (0.0);
  }
  else
    {
      if (m_lane_shift > 0.0)
        m_lane_shift = std::max (0.0, m_lane_shift - shift_step);
      else if (m_lane_shift < 0.0)
        m_lane_shift = std::min (0.0, m_lane_shift + shift_step);
    }
}

// Return the minimum difference between the maximum speed on the
// racing line and the minimum possible future car speeds
Two_Vector
Robot_Driver::velocity_margin (double distance)
{
  delete mp_braking_equation;
  const double drag = mp_car->chassis ().aerodynamic_drag ()
    / mp_car->chassis ().mass ();
  mp_braking_equation = new Braking_Equation (deceleration (),
                                              drag,
                                              speed ());

  Two_Vector margin = m_racing_line.braking_margin (mp_braking_equation, 
                                                    distance, 
                                                    m_lane_shift,
                                                    mp_car->balance (),
                                                    target_distance (),
                                                    m_air_density_factor);

  return margin;
}

double 
Robot_Driver::deceleration () const
{
  return m_deceleration [0] + speed () * m_deceleration [1];
}

//-----------------------------------------------------------------------------
Braking::Braking ()
  : m_start (0.0),
    m_length (0.0),
    m_is_braking (false),
    mp_braking_equation (0)
{
}

Braking::~Braking ()
{
  delete mp_braking_equation;
}

void
Braking::start (double start, 
                double length,
                double track_length,
                const Braking_Equation& equation)
{
  m_start = start;
  m_length = length;
  m_track_length = track_length;
  m_is_braking = true;
  delete mp_braking_equation;
  mp_braking_equation = new Braking_Equation (equation);
}

void
Braking::end ()
{
  m_is_braking = false;
  delete mp_braking_equation;
  mp_braking_equation = 0;
}

bool
Braking::check_done_braking (double distance)
{
   if (past_end (distance))
     end ();
  return !m_is_braking;
}

double
Braking::distance_from_start (double distance) const
{
  if (distance >= m_start)
    return (distance - m_start);
  else // wrap around the track
    return (distance + m_track_length - m_start);
}

bool
Braking::past_end (double distance) const
{
  return (distance_from_start (distance) > m_length);
}

double
Braking::maximum_speed (double distance) const
{
  if (mp_braking_equation == 0)
    return std::numeric_limits <double>::max ();
  return (*mp_braking_equation) (distance_from_start (distance));
}

//-----------------------------------------------------------------------------
Robot_Racing_Line::Robot_Racing_Line (const Road& road,
                                      const std::vector <double>& lateral_acceleration)
  : mp_road (&road),
    m_lateral_acceleration (lateral_acceleration)
{
}

double
Robot_Racing_Line::maximum_speed (double distance, 
                                  double lane_shift,
                                  double balance, 
                                  double lead,
                                  double air_density_factor) const
{
  const double along = mp_road->racing_line ().distance (distance);
  const double curvature = std::abs (mp_road->racing_line ()
                                     .curvature (along + lead, lane_shift));

  const double a0 = m_lateral_acceleration [0] * balance;
  // attenuate a1 in slipstream
  const double a1 = m_lateral_acceleration [1] * balance * air_density_factor;

  if (a1 < curvature)
    return std::sqrt (a0 / (curvature - a1));
  else
    return std::numeric_limits <double>::max ();
}

Three_Vector
Robot_Racing_Line::target (double distance, double lead) const
{
  const double along = mp_road->racing_line ().distance (distance);
  return mp_road->racing_line ().position (along + lead);
}

Two_Vector
Robot_Racing_Line::braking_margin (const Braking_Equation* braking, 
                                   double distance,
                                   double lane_shift,
                                   double balance,
                                   double lead,
                                   double air_density_factor) const
// Return the minimum speed difference, and its distance.
{
  double margin = std::numeric_limits <double>::max ();
  double d_margin = 0.0;
  double braking_speed;
  for (double d = 0.0; (braking_speed = (*braking) (d)) > 0.0; d += 1.0)
    {
      double speed_limit = 
        maximum_speed (distance + d, lane_shift, balance, lead, air_density_factor);
      const double delta_speed = speed_limit - braking_speed;
      if (delta_speed < margin)
        {
          margin = delta_speed;
          d_margin = d;
        }
    }

  return Two_Vector (margin, d_margin);
}
