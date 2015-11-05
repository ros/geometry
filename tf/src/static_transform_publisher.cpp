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

#include <cstdio>
#include "tf/transform_broadcaster.h"

class TransformSender
{
public:
  //constructor
  TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
  { 
    tf::Quaternion q;
    q.setRPY(roll, pitch,yaw);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );
  };
  TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string& frame_id, const std::string& child_frame_id) :
    transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)), time, frame_id, child_frame_id){};
  //Clean up ros connections
  ~TransformSender() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;



  // A function to call to send data periodically
  void send (ros::Time time) {
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

private:
  tf::StampedTransform transform_;

};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);
  ros::NodeHandle node_;
  TransformSender* tf_sender;

  double x, y, z, yaw, pitch, roll, qx, qy, qz, qw, period;
  std::string frame_id, child_frame_id;
  bool use_rpy;

  if(argc == 11)
  {
    x               = atof(argv[1]);
    y               = atof(argv[2]);
    z               = atof(argv[3]);
    qx              = atof(argv[4]);
    qy              = atof(argv[5]);
    qz              = atof(argv[6]);
    qw              = atof(argv[7]);
    period          = atof(argv[10]);
    frame_id        = argv[8];
    child_frame_id  = argv[9];
    use_rpy = false;
  } 
  else if (argc == 10)
  {
    x               = atof(argv[1]);
    y               = atof(argv[2]);
    z               = atof(argv[3]);
    yaw             = atof(argv[4]);
    pitch           = atof(argv[5]);
    roll            = atof(argv[6]);
    period          = atof(argv[9]);
    frame_id        = argv[7];
    child_frame_id  = argv[8];
    use_rpy = true;
  }
  else
  {
    printf("A command line utility for manually sending a transform.\n");
    printf("It will periodicaly republish the given transform. \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id  period(milliseconds) \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds) \n");
    printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    printf("of the child_frame_id.  \n");
    ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments");
    return -1;
  }

  ros::Duration sleeper(period/1000.0);

  if (frame_id.compare(child_frame_id) == 0)
    ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", frame_id.c_str(), child_frame_id.c_str());

  if (use_rpy)
    tf_sender = new TransformSender(x, y, z, yaw, pitch, roll,
                                    ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                                    frame_id, child_frame_id);
  else
    tf_sender = new TransformSender(x, y, z, qx, qy, qz, qw,
                                    ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                                    frame_id, child_frame_id);

  while(node_.ok())
  {
    tf_sender->send(ros::Time::now() + sleeper);
    ROS_DEBUG("Sending transform from %s with parent %s\n", argv[8], argv[9]);
    sleeper.sleep();
  }

  return 0;
};

