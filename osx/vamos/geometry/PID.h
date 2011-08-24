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

#ifndef _PID_H_
#define _PID_H_

namespace Vamos_Geometry
{
  class PID
  {
  public:
    PID (double p, double i, double d);
    double output (double error, double dt);
    double output () const { return value; }

  private:
    double kp;
    double ki;
    double kd;
    
    double integral;
    double previous_error;

    double value;
  };
}

#endif // not _PID_H_
