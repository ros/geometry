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
  ros::NodeHandle node_;
  TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
  { 
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
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

void help()
{
  printf("A command line utility for manually sending a transform.\n");
  printf("It will periodicaly republish the given transform. \n");
  printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period(milliseconds) \n");
  printf("OR \n");
  printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period(milliseconds) \n");
  printf("As an alternative, you may supply these arguments as ROS parameters (names as above). \n");
  printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
  printf("of the child_frame_id. \n");
  ROS_ERROR("static_transform_publisher exited due to not having the right number of arguments (9, 10 or 0)");
}

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);

  double period, x, y, z, rx, ry, rz, w;
  std::string frame_id, child_frame_id;
  bool is_quat;
  
  if (argc == 11) {
    x =  atof(argv[1]);
    y =  atof(argv[2]);
    z =  atof(argv[3]);
    rx = atof(argv[4]);
    ry = atof(argv[5]);
    rz = atof(argv[6]);
    w =  atof(argv[7]);
    frame_id = argv[8];
    child_frame_id = argv[9];
    period = atof(argv[10]);
    is_quat = true;
  }
  else if (argc == 10) {
    x =  atof(argv[1]);
    y =  atof(argv[2]);
    z =  atof(argv[3]);
    rx = atof(argv[4]);
    ry = atof(argv[5]);
    rz = atof(argv[6]);
    frame_id = argv[7];
    child_frame_id = argv[8];
    period = atof(argv[9]);
    is_quat = false;
  }
  else if (argc == 1) {
    try {
      ros::param::get("~period", period);
      ros::param::get("~x",  x);
      ros::param::get("~y",  y);
      ros::param::get("~z",  z);

      if (ros::param::has("~yaw")) {
        ros::param::get("~yaw",   rz);
        ros::param::get("~pitch", ry);
        ros::param::get("~roll",  rx);
        is_quat = false;
      }
      else {
        ros::param::get("~qx", rz);
        ros::param::get("~qy", ry);
        ros::param::get("~qz", rx);
        ros::param::get("~qw", w);
        is_quat = true;
      }

      ros::param::get("~frame_id", frame_id);
      ros::param::get("~child_frame_id", child_frame_id);
    }
    catch (ros::InvalidNameException& e) {
      help();
      return -1;
    }
  }
  else {
    help();
    return -1;
  }

  if (frame_id == child_frame_id) {
    ROS_FATAL("target_frame and source frame are the same (%s, %s), this cannot work", child_frame_id.c_str(), frame_id.c_str());
  }

  ros::Duration sleeper(period/1000.0);

  if (is_quat) {
    TransformSender tf_sender(x, y, z, rx, ry, rz, w,
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              frame_id, child_frame_id);

    while(tf_sender.node_.ok())
    {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", child_frame_id.c_str(), frame_id.c_str());
      sleeper.sleep();
    }

    return 0;
  }
  else {
    TransformSender tf_sender(x, y, z, rz, ry, rx,  // Note that yaw comes first here
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              frame_id, child_frame_id);

    while(tf_sender.node_.ok())
    {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", child_frame_id.c_str(), frame_id.c_str());
      sleeper.sleep();
    }

    return 0;
  }
}

