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
#include <dynamic_reconfigure/server.h>
#include "tf/transform_broadcaster.h"
#include "tf/TransformSenderConfig.h"

class TransformSender
{
public:
  ros::NodeHandle node_;
  //constructor
  TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string& frame_id, const std::string& child_frame_id)
  { 
    tf::Quaternion q;
    q.setRPY(roll, pitch,yaw);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x,y,z)), time, frame_id, child_frame_id );
    reconf_init();
  };
  TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string& frame_id, const std::string& child_frame_id) :
    transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)), time, frame_id, child_frame_id)
  {
    reconf_init();
  };
  //Clean up ros connections
  ~TransformSender() { }

  //A pointer to the rosTFServer class
  tf::TransformBroadcaster broadcaster;



  // A function to call to send data periodically
  void send (ros::Time time) {
    transform_.stamp_ = time;
    broadcaster.sendTransform(transform_);
  };

  // Dynamic reconfigure callback
  void reconf_callback(tf::TransformSenderConfig &config, uint32_t level)
  {
    tf::Quaternion q;
    tf::Transform t;
    double R, P, Y;

    switch(level) // sent by dynamic reconfigure at first run
    {
      case CHANGE_ALL:
        // Update config with current values
        config.x = transform_.getOrigin().x();
        config.y = transform_.getOrigin().y();
        config.z = transform_.getOrigin().z();

        // Update RPY in radians
        transform_.getBasis().getRPY(R, P, Y);
        config.roll_rad = R;
        config.pitch_rad = P;
        config.yaw_rad = Y;
        // Update RPY in degrees
        toDegrees(R, P, Y);
        config.roll_deg= R;
        config.pitch_deg= P;
        config.yaw_deg = Y;

        config.qw = transform_.getRotation().w();
        config.qx = transform_.getRotation().x();
        config.qy = transform_.getRotation().y();
        config.qz = transform_.getRotation().z();
        break;

      case CHANGE_XYZ:
        t = tf::Transform(transform_.getRotation(), tf::Vector3(config.x, config.y, config.z));
        transform_ = tf::StampedTransform(t, ros::Time::now(), transform_.frame_id_, transform_.child_frame_id_);
        break;

      case CHANGE_RPY_DEG:
        R = config.roll_deg;
        P = config.pitch_deg;
        Y = config.yaw_deg;
        toRadians(R, P, Y);

        q.setRPY(R, P, Y);
        t = tf::Transform(q, transform_.getOrigin());
        transform_ = tf::StampedTransform(t, ros::Time::now(), transform_.frame_id_, transform_.child_frame_id_);
        // Update quaternion
        config.qw = transform_.getRotation().w();
        config.qx = transform_.getRotation().x();
        config.qy = transform_.getRotation().y();
        config.qz = transform_.getRotation().z();
        // Update RPY in radians
        config.roll_rad = R;
        config.pitch_rad = P;
        config.yaw_rad = Y;
        break;

      case CHANGE_RPY_RAD:
        R = config.roll_rad;
        P = config.pitch_rad;
        Y = config.yaw_rad;

        q.setRPY(R, P, Y);
        t = tf::Transform(q, transform_.getOrigin());
        transform_ = tf::StampedTransform(t, ros::Time::now(), transform_.frame_id_, transform_.child_frame_id_);
        // Update quaternion
        config.qw = transform_.getRotation().w();
        config.qx = transform_.getRotation().x();
        config.qy = transform_.getRotation().y();
        config.qz = transform_.getRotation().z();
        // Update RPY in degrees
        toDegrees(R, P, Y);
        config.roll_deg= R;
        config.pitch_deg= P;
        config.yaw_deg = Y;
        break;

      case CHANGE_QUAT:
        q = tf::Quaternion(config.qx, config.qy, config.qz, config.qw);

        // If new quaternion is not valid use previous one and issue error
        if(q.length2() == 0.0){
          q = transform_.getRotation();
          ROS_ERROR("Reconfigure: quaternion length is 0.0. Using previous value");
        }
        // Check normalization
        else if(q.length2() > 1.0 + DBL_EPSILON || q.length2() < 1.0 - DBL_EPSILON)
        {
          q = q.normalize();
          ROS_WARN("Reconfigure: quaternion is not normalized. Normalizing.");
        }
        t = tf::Transform(q, transform_.getOrigin());
        transform_ = tf::StampedTransform(t, ros::Time::now(), transform_.frame_id_, transform_.child_frame_id_);

        // Update quaternion with corrected value
        config.qw = q.w();
        config.qx = q.x();
        config.qy = q.y();
        config.qz = q.z();

        // Update RPY in radians
        transform_.getBasis().getRPY(R, P, Y);
        config.roll_rad = R;
        config.pitch_rad = P;
        config.yaw_rad = Y;
        // Update RPY in degrees
        toDegrees(R, P, Y);
        config.roll_deg= R;
        config.pitch_deg= P;
        config.yaw_deg = Y;

        // Reset checkbox
        config.use_quaternion = false;
        break;
    }
  }

private:
  enum{
    CHANGE_NOTHING = 0,
    CHANGE_XYZ = 1 << 0,
    CHANGE_RPY_RAD = 1 << 1,
    CHANGE_RPY_DEG = 1 << 2,
    CHANGE_QUAT = 1 << 3,
    CHANGE_ALL = 0xffffffff
  };

  tf::StampedTransform transform_;
  dynamic_reconfigure::Server<tf::TransformSenderConfig> reconf_server_;

  // Set dynamic reconfigure callback
  void reconf_init()
  {
    reconf_server_.setCallback(boost::bind(&TransformSender::reconf_callback, this, _1, _2));
  }

  void toDegrees(double &r, double &p, double &y)
    {
      r = r * 180.0 / M_PI;
      p = p * 180.0 / M_PI;
      y = y * 180.0 / M_PI;
    }

    void toRadians(double &r, double &p, double &y)
    {
      r = r / 180.0 * M_PI;
      p = p / 180.0 * M_PI;
      y = y / 180.0 * M_PI;
    }
};

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv,"static_transform_publisher", ros::init_options::AnonymousName);

  if(argc == 11)
  {
    ros::Duration sleeper(atof(argv[10])/1000.0);

    if (strcmp(argv[8], argv[9]) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[8], argv[9]);

    TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                              atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]),
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              argv[8], argv[9]);



    while(tf_sender.node_.ok())
    {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", argv[8], argv[9]);
      ros::spinOnce();
      sleeper.sleep();
    }

    return 0;
  } 
  else if (argc == 10)
  {
    ros::Duration sleeper(atof(argv[9])/1000.0);

    if (strcmp(argv[7], argv[8]) == 0)
      ROS_FATAL("target_frame and source frame are the same (%s, %s) this cannot work", argv[7], argv[8]);

    TransformSender tf_sender(atof(argv[1]), atof(argv[2]), atof(argv[3]),
                              atof(argv[4]), atof(argv[5]), atof(argv[6]),
                              ros::Time() + sleeper, //Future dating to allow slower sending w/o timeout
                              argv[7], argv[8]);



    while(tf_sender.node_.ok())
    {
      tf_sender.send(ros::Time::now() + sleeper);
      ROS_DEBUG("Sending transform from %s with parent %s\n", argv[7], argv[8]);
      ros::spinOnce();
      sleeper.sleep();
    }

    return 0;

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


};

