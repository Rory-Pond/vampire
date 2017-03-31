//-----------------------------------------------------------------------------
//
// This source file is part of the VAMPIRE open source package under the
// GNU GPL (version 2) licence (see licence file for details).
//
// (c) R F L Evans 2016. All rights reserved.
//
//-----------------------------------------------------------------------------

// C++ standard library headers
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

// Vampire headers
#include "create.hpp"

// Internal create header
#include "internal.hpp"

namespace sim{
   //                           Domain Wall Generation
   //
   // User enters these options
   // Magnetisation Axis:      x, y, z    [This is the axis which the domain wall will flip]
   // Progression Axis:        x, y, z    [This is the axis which the domain wall progresses]
   // Rotaion Axis:            ±x, ±y, ±z [This is the axis which the domain wall rotates around]
   // Lengh of Domain Wall:    float      [This is the number of atoms wide the domain wall will be]
   // Position of Domain Wall: float 0-1  [This is the positon of the domain wall as a percentage of the system size]
   //
   // Rotaion Axis == Magnetisation Axis  [This is an impossible transformation and will cuase error]
   // Rotaion Axis == Progression axis    [This is a bloch wall]
   // Rotaion Axis =/= Progression Axis   [This is a neal wall]
   //
   // Example usage:
   // float angle = DomainWallAngle(coord[i], Len, pos, Mot_axis);
   // spin[i] = DomainWallRotation(spin[i], Rot_axis, angle);

   const std::vector<int> x_axis = {1, 0, 0};
   const std::vector<int> y_axis = {0, 1, 0};
   const std::vector<int> z_axis = {0, 0, 1};

   const std::vector<int> x_axis_ = {-1, 0, 0};
   const std::vector<int> y_axis_ = {0, -1, 0};
   const std::vector<int> z_axis_ = {0, 0, -1};

   std::vector<float> domainWallRotation(std::vector<float> spin, std::vector<int> Rot, float angle);
   float domainWallAngle(std::vector<float> coord, float len, float pos, std::vector<int> Mot);

   void domainwall_generate(  std::vector<float> &sx, 
                              std::vector<float> &sy, 
                              std::vector<float> &sz, 
                              std::vector<float> &cx, 
                              std::vector<float> &cy, 
                              std::vector<float> &cz,
                              float len,
                              float pos,
                              std::vector<int> Rot_axis,
                              std::vector<int> Mot_axis){
      f

   }

   std::vector<float> domainWallRotation(std::vector<float> spin, std::vector<int> Rot_axis, float angle)
   {
      int index = Rot_axis[0] * 0 + Rot_axis[1] * 1 + Rot_axis[2] * 2;
      std::vector<float> result(3, 0);

      switch (abs(index))
      {
      case 0:
         angle = angle * Rot_axis[0];
         result[0] = spin[0];
         result[1] = spin[1] * cos(angle) - spin[2] * sin(angle);
         result[2] = spin[1] * sin(angle) + spin[2] * cos(angle);
         break;
      case 1:
         angle = angle * Rot_axis[1];
         result[0] = spin[0] * cos(angle) + spin[2] * sin(angle);
         result[1] = spin[1];
         result[2] = (-spin[0]) * sin(angle) + spin[2] * cos(angle);
         break;
      case 2:
         angle = angle * Rot_axis[2];
         result[0] = spin[0] * cos(angle) - spin[1] * sin(angle);
         result[1] = spin[0] * sin(angle) + spin[1] * cos(angle);
         result[2] = spin[2];
         break;
      }
      return result;
   }

   float domainWallAngle(std::vector<float> coord, float len, float pos, std::vector<int> Mot_axis)
   {
      int index = Mot_axis[0] * 0 + Mot_axis[1] * 1 + Mot_axis[2] * 2;

      float dir = tanh((1 / len) * (coord[index] - pos));
      float angle = ((dir + 1) / 2) * M_PI;
      return angle;
   }
}