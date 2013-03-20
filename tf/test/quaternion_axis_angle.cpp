/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <sys/time.h>
#include <cstdio>

#include "tf/LinearMath/Transform.h"


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};


int main(int argc, char **argv){

  unsigned int runs = 400;
  seed_rand();

  std::vector<tf::Vector3> axes;
  axes.push_back(tf::Vector3(1,0,0));
  axes.push_back(tf::Vector3(0,1,0));
  axes.push_back(tf::Vector3(0,0,1));

  tf::Quaternion q_identity(0,0,0,1);
  tf::Quaternion q_rotated(0,0,0,1);

  for(unsigned int a = 0; a < axes.size(); a++)
  {
    const tf::Vector3& axis = axes[a];

    for ( unsigned int i = 0; i < 360 ; i+=60 )
    {
      double angle = i*M_PI/180;
      q_rotated.setRotation(axis, angle);

      // Compute rotation axis from quaternion
      tf::Vector3 axis_out = q_rotated.getAxis();

      // Print the input values
      std::printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
      std::printf("                         input: %.3f @ [%.3f, %.3f, %.3f]\n",
                  angle,
                  axis.x(), axis.y(), axis.z());

      // Compute and print the axis/angle of this quaternion explicitly from identity.
      tfScalar angle_out = 2*q_rotated.angle(q_identity); // why the HECK is this the half angle?
      tfScalar angle_shortest = q_rotated.angleShortestPath(q_identity);
      //std::printf("angle(identity), angleShortestPath(identity):\n");
      std::printf("               angle(identity): %.3f @ [%.3f, %.3f, %.3f] \n",
                  angle_out, axis_out.x(), axis_out.y(), axis_out.z());
      std::printf("   angleShortestPath(identity): %.3f\n",
                  angle_shortest);

      // Compute and print the axis/angle of this quaternion (from identity is implied)
      angle_out = q_rotated.getAngle();
      angle_shortest = q_rotated.getAngleShortestPath();
      //std::printf("getAngle(), getAngleShortestPath() :\n");
      std::printf("                    getAngle(): %.3f @ [%.3f, %.3f, %.3f] \n",
                  angle_out, axis_out.x(), axis_out.y(), axis_out.z());
      std::printf("        getAngleShortestPath(): %.3f\n",
                  angle_shortest);
    }
  }

  return 0;
}


