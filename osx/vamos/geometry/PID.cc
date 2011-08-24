//  PID.h - a proportional, integral, derivative controller.
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

#include "PID.h"
#include <iostream>
Vamos_Geometry::
PID::PID (double p, double i, double d)
  : kp (p),
    ki (i),
    kd (d),
    integral (0.0),
    previous_error (0.0),
    value (0.0)
{
}

double Vamos_Geometry::
PID::output (double error, double dt)
{
  if (dt == 0.0)
    return value;

  double proportional = kp * error;
  integral += ki * error * dt;
  double derivative = kd * (error - previous_error) / dt;

  previous_error = error;

  value = proportional + integral + derivative;
  return value;
}
