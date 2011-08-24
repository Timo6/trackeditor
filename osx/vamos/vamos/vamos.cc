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

#include <cstdlib>
#include <getopt.h>

#include <GL/glut.h>

#include "../body/Gl_Car.h"
#include "../media/Texture_Image.h"
#include "../media/Ac3d.h"
#include "../track/Strip_Track.h"
#include "../world/Gl_World.h"
#include "../world/Sounds.h"
#include "../world/Interactive_Driver.h"
#include "../world/Robot_Driver.h"

using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_World;

bool get_options (int argc, char* argv [],
                  std::string& car_file,
                  std::string& track_file,
                  std::string& world_file,
                  std::string& controls_file,
                  size_t& number_of_opponents,
                  size_t& focused_car,
                  double& volume,
                  bool& map_mode,
                  bool& full_screen,
                  bool& no_interaction,
                  bool& demo,
                  std::vector <double>& parameter)
{
  // Set the default options.
  car_file = "default-car";
  track_file = "default-track";
  world_file = "default-world";
  controls_file = "default-controls";
  number_of_opponents = 0;
  focused_car = 0;
  map_mode = false;
  full_screen = false;
  no_interaction = false;
  demo = false;

  bool error = false;

  while (true)
    {
      static struct option long_options [] =
        {
          { "car", required_argument, 0, 'c' },
          { "track", required_argument, 0, 't' },
          { "world", required_argument, 0, 'w' },
          { "controls", required_argument, 0, 'a' },
          { "opponents", required_argument, 0, 'o' },
          { "focused-car", required_argument, 0, 'i' },
          { "volume", required_argument, 0, 'l' },
          { "map", no_argument, 0, 'm' },
          { "demo", no_argument, 0, 'd' },
          { "full-screen", no_argument, 0, 'f' },
          { "no-interaction", no_argument, 0, 'n' },
          { "version", no_argument, 0, 'v' },
          { 0, 0, 0, 0 }
        };

      int option_index = 0;
      int c = getopt_long (argc, argv, "c:t:w:a:o:i:l:mdfnv", 
                           long_options, &option_index);
      if (c == -1)
        break;

      switch (c)
        {
        case 0:
          break;
        case 'c':
          car_file = optarg;
          break;
        case 't':
          track_file = optarg;
          break;
        case 'w':
          world_file = optarg;
          break;
        case 'a':
          controls_file = optarg;
          break;
        case 'o':
          number_of_opponents = atoi (optarg);
          break;
        case 'i':
          focused_car = atoi (optarg);
          break;
        case 'l':
          volume = atoi (optarg) / 100.0;
          break;
        case 'm':
          map_mode = true;
          break;
        case 'd':
          demo = true;
          break;
        case 'f':
          full_screen = true;
          break;
        case 'n':
          no_interaction = true;
          break;
        case 'v':
          std::cout << PACKAGE_STRING << std::endl;
          std::exit (EXIT_SUCCESS);
        case '?':
          error = true;
        }
    }

  // Assume the rest of the arguments are values of adjustable
  // parameters.
  for (int index = optind; index < argc; index++)
    parameter.push_back (atof (argv [index]));

  return error;
}

std::string
get_path (std::string file, std::string section)
{
  std::string path = "../data/" + section + "/" + file + ".xml";
  {
    std::ifstream check (path.c_str ());
    if (check)
      return path;
  }
  path = DATADIR "/" + section + "/" + file + ".xml";
  {
    std::ifstream check (path.c_str ());
    if (check)
      return path;
  }

  return file;
}

int main (int argc, char* argv [])
{
  std::string car_file;
  std::string track_file;
  std::string world_file;
  std::string controls_file;
  size_t number_of_opponents;
  size_t focused_car;
  double volume = 1.0;
  bool map_mode;
  bool full_screen;
  bool no_interaction;
  bool demo;
  std::vector <double> parameter;

  bool error = get_options (argc, argv,
                            car_file, track_file, 
                            world_file, controls_file,
                            number_of_opponents,
                            focused_car,
                            volume,
                            map_mode,
                            full_screen,
                            no_interaction,
                            demo,
                            parameter);

  if (error)
    {
      std::cerr << "Usage: vamos "
                << "[-m|--map] "
                << "[[-t|--track=] TRACK_FILE] "
                << "[[-c|--car=] CAR_FILE] "
                << "[[-w|--world=] WORLD_FILE] "
                << "[[-a|--controls=] CONTROLS_FILE] "
                << "[[-o|--opponents=] NUMBER_OF_OPPONENTS] "
                << "[[-i|--focused-car=] FOCUSED_CAR_INDEX] "
                << "[[-l|--volume=] VOLUME_PERCENT] "
                << "[-f|--full-screen] "
                << "[-n|--no-interaction] "
                << "[-d|--demo] "
                << std::endl;
      std::exit (EXIT_FAILURE);
    }

  Atmosphere air (1.2, Three_Vector (0.0, 0.0, 0.0));

  // Check the source directory for data files.
  std::string data_dir = "../data/";
  try
    {
      Texture_Image test_tex (data_dir + "textures/wall.png");
    }
  catch (Missing_Texture_File)
    {
      // They're not there.  Maybe this is an installed version.
      data_dir = DATADIR "/";
      try
        {
          Texture_Image test_tex (data_dir + "textures/wall.png");
        }
      catch (Missing_Texture_File)
        {
          std::cerr << "Couldn't find the data direcory, ../data or "
                    << DATADIR "/." << std::endl;
          std::exit (EXIT_FAILURE);
        }
    }

  Vamos_Track::Strip_Track road;
  road.set_parameters (parameter);

  Sounds* sounds = 0;
  if (!map_mode)
    {
      sounds = new Sounds ();
      try
        {
          sounds->read (data_dir + "sounds/", "default-sounds.xml");
        }
      catch (XML_Exception& error)
        {
          std::cerr << error.message () << std::endl;
          std::exit (EXIT_FAILURE);
        }
      sounds->master_volume (volume);
    }
  Gl_World world (argc, argv, &road, &air, sounds, full_screen);
  world.cars_can_interact (!no_interaction);

  try
    {
      road.read (data_dir, get_path (track_file, "tracks"));
    }
  catch (XML_Exception& error)
    {
      std::cerr << error.message () << std::endl;
      std::exit (EXIT_FAILURE);
    }

  Vamos_Body::Gl_Car* car = 0;
  if (!map_mode)
    {
      try
        {
          const double grid_interval = 8.0;
          double across = 3.0;
          double start_distance = 10.0;
          const Three_Vector drop (0.0, 0.0, 0.6);
          Three_Matrix orientation;
          orientation.rotate (Three_Vector (0.0, 0.0, road.start_direction ()));
          Three_Vector position;
          size_t index = 0;

          if (!demo)
            {
              start_distance += grid_interval;
              car = new Vamos_Body::Gl_Car (road.position (start_distance, across) + drop,
                                            orientation);
              car->read (data_dir, get_path (car_file, "cars"));
              position = Three_Vector(0.0, 0.0, 
                                      -car->chassis ().lowest_contact_position () + 0.5);
              car->chassis ().translate (position);
              car->start_engine ();
              world.add_car (car, new Interactive_Driver (car));
              world.set_controlled_car (index);
              index++;
              across = -across;
            }
          else
            number_of_opponents = std::max (size_t (1), number_of_opponents);

          for (size_t i = 0; i < number_of_opponents; i++)
            {
              start_distance += grid_interval;
              car = new Vamos_Body::Gl_Car (road.position (start_distance, across) + drop,
                                            orientation);
              car->read (data_dir, get_path (car_file, "cars"));
              position = Three_Vector(0.0, 0.0, 
                                      -car->chassis ().lowest_contact_position () + 0.5);
              car->chassis ().translate (position);
              car->start_engine ();
              Robot_Driver* driver = new Robot_Driver (car, &road);
              //!! Who deletes the drivers?
              driver->interact (!no_interaction);
              world.add_car (car, driver);
              index++;
              across = -across;
            }

          if (car != 0)
            {
              road.build_racing_line (number_of_opponents >= 1);
              if (focused_car > world.number_of_cars () - 1)
                {
                  std::cerr << argv [0] 
                            << ": focused car (-n) must be less than " 
                            << "the total number of cars" 
                            << std::endl;
                  std::exit (EXIT_FAILURE);
                }
              world.set_focused_car (focused_car);
            }
        }
      catch (XML_Exception& error)
        {
          std::cerr << error.message () << std::endl;
          std::exit (EXIT_FAILURE);
        }
      catch (Vamos_Media::Malformed_Ac3d_File& error)
        {
          std::cerr << error.message () << std::endl;
          std::exit (EXIT_FAILURE);
        }
    }
  else
    road.build_racing_line (0.0);

  try
    {
      world.read (data_dir, 
                  get_path (world_file, "worlds"), 
                  get_path (controls_file, "controls"));
    }
  catch (XML_Exception& error)
    {
      std::cerr << error.message () << std::endl;
      std::exit (EXIT_FAILURE);
    }
  world.start ();

  delete sounds;

  return EXIT_SUCCESS;
}
