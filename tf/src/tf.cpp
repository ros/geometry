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

/** \author Tully Foote */

#include "tf/tf.h"
#include <sys/time.h>
#include "ros/assert.h"
#include "ros/ros.h"
#include "angles/angles.h"

using namespace tf;

// Must provide storage for non-integral static const class members.
// Otherwise you get undefined symbol errors on OS X (why not on Linux?).
// Thanks to Rob for pointing out the right way to do this.
// In C++0x this must be initialized here #5401
const double tf::Transformer::DEFAULT_CACHE_TIME = 10.0;


enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

struct CanTransformAccum
{
  CompactFrameID gather(TimeCache* cache, ros::Time time, std::string* error_string)
  {
    return cache->getParent(time, error_string);
  }

  void accum(bool source)
  {
  }

  void finalize(WalkEnding end, ros::Time _time)
  {
  }

  TransformStorage st;
};

struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCache* cache, ros::Time time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  void finalize(WalkEnding end, ros::Time _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        tf::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        tf::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  ros::Time time;
  tf::Quaternion source_to_top_quat;
  tf::Vector3 source_to_top_vec;
  tf::Quaternion target_to_top_quat;
  tf::Vector3 target_to_top_vec;

  tf::Quaternion result_quat;
  tf::Vector3 result_vec;
};


std::string assert_resolved(const std::string& prefix, const std::string& frame_id)
{
  if (frame_id.size() > 0)
    if (frame_id[0] != '/')
      ROS_DEBUG("TF operating on not fully resolved frame id %s, resolving using local prefix %s", frame_id.c_str(), prefix.c_str());
  return tf::resolve(prefix, frame_id);
};

std::string tf::resolve(const std::string& prefix, const std::string& frame_name)
{
  //  printf ("resolveping prefix:%s with frame_name:%s\n", prefix.c_str(), frame_name.c_str());
  if (frame_name.size() > 0)
    if (frame_name[0] == '/')
    {
      return frame_name;
    }
  if (prefix.size() > 0)
  {
    if (prefix[0] == '/')
    {
      std::string composite = prefix;
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }
    else
    {
      std::string composite;
      composite = "/";
      composite.append(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }

  }
  else
 {
    std::string composite;
    composite = "/";
    composite.append(frame_name);
    return composite;
  }
};



Transformer::Transformer(bool interpolating,
                                ros::Duration cache_time):
  cache_time(cache_time),
  interpolating (interpolating), 
  using_dedicated_thread_(false),
  fall_back_to_wall_time_(false)
{
  max_extrapolation_distance_.fromNSec(DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(NULL);// new TimeCache(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

Transformer::~Transformer()
{
  /* deallocate all frames */
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  for (std::vector<TimeCache*>::iterator  cache_it = frames_.begin(); cache_it != frames_.end(); ++cache_it)
  {
    delete (*cache_it);
  }

};


void Transformer::clear()
{
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector< TimeCache*>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      (*cache_it)->clearList();
    }
  }
}


template<typename F>
int Transformer::walkToTopParent(F& f, ros::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const
{
  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_id == target_id)
  {
    f.finalize(Identity, time);
    return NO_ERROR;
  }

  //If getting the latest get the latest common time
  if (time == ros::Time())
  {
    int retval = getLatestCommonTime(target_id, source_id, time, error_string);
    if (retval != NO_ERROR)
    {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;
  while (frame != 0)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, 0);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      f.finalize(TargetParentOfSource, time);
      return NO_ERROR;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  while (frame != top_parent)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      return NO_ERROR;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);

  return NO_ERROR;
}



bool Transformer::setTransform(const StampedTransform& transform, const std::string& authority)
{

  StampedTransform mapped_transform((tf::Transform)transform, transform.stamp_, transform.frame_id_, transform.child_frame_id_);
  mapped_transform.child_frame_id_ = assert_resolved(tf_prefix_, transform.child_frame_id_);
  mapped_transform.frame_id_ = assert_resolved(tf_prefix_, transform.frame_id_);

 
  bool error_exists = false;
  if (mapped_transform.child_frame_id_ == mapped_transform.frame_id_)
  {
    ROS_ERROR("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), mapped_transform.child_frame_id_.c_str());
    error_exists = true;
  }

  if (mapped_transform.child_frame_id_ == "/")//empty frame id will be mapped to "/"
  {
    ROS_ERROR("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
    error_exists = true;
  }

  if (mapped_transform.frame_id_ == "/")//empty parent id will be mapped to "/"
  {
    ROS_ERROR("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", mapped_transform.child_frame_id_.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(mapped_transform.getOrigin().x()) || std::isnan(mapped_transform.getOrigin().y()) || std::isnan(mapped_transform.getOrigin().z())||
      std::isnan(mapped_transform.getRotation().x()) ||       std::isnan(mapped_transform.getRotation().y()) ||       std::isnan(mapped_transform.getRotation().z()) ||       std::isnan(mapped_transform.getRotation().w()))
  {
    ROS_ERROR("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
              mapped_transform.child_frame_id_.c_str(), authority.c_str(),
              mapped_transform.getOrigin().x(), mapped_transform.getOrigin().y(), mapped_transform.getOrigin().z(),
              mapped_transform.getRotation().x(), mapped_transform.getRotation().y(), mapped_transform.getRotation().z(), mapped_transform.getRotation().w()
              );
    error_exists = true;
  }

  if (error_exists)
    return false;

  {
    boost::recursive_mutex::scoped_lock lock(frame_mutex_);
    CompactFrameID frame_number = lookupOrInsertFrameNumber(mapped_transform.child_frame_id_);
    TimeCache* frame = getFrame(frame_number);
    if (frame == NULL)
    {
    	frames_[frame_number] = new TimeCache(cache_time);
    	frame = frames_[frame_number];
    }

    if (frame->insertData(TransformStorage(mapped_transform, lookupOrInsertFrameNumber(mapped_transform.frame_id_), frame_number)))
    {
      frame_authority_[frame_number] = authority;
    }
    else
    {
      ROS_WARN("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at ", mapped_transform.child_frame_id_.c_str(), mapped_transform.stamp_.toSec(), authority.c_str());
      return false;
    }
  }

  {
    boost::mutex::scoped_lock lock(transforms_changed_mutex_);
    transforms_changed_();
  }

  return true;
};


void Transformer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                     const ros::Time& time, StampedTransform& transform) const
{
	  std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
	  std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

	  if (mapped_tgt == mapped_src) {
		  transform.setIdentity();
		  transform.child_frame_id_ = mapped_src;
		  transform.frame_id_       = mapped_tgt;
		  transform.stamp_          = now();
		  return;
	  }

	  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

	  CompactFrameID target_id = lookupFrameNumber(mapped_tgt);
	  CompactFrameID source_id = lookupFrameNumber(mapped_src);

	  std::string error_string;
	  TransformAccum accum;
	  int retval = walkToTopParent(accum, time, target_id, source_id, &error_string);
	  if (retval != NO_ERROR)
	  {
	    switch (retval)
	    {
	    case CONNECTIVITY_ERROR:
	      throw ConnectivityException(error_string);
	    case EXTRAPOLATION_ERROR:
	      throw ExtrapolationException(error_string);
	    case LOOKUP_ERROR:
	      throw LookupException(error_string);
	    default:
	      ROS_ERROR("Unknown error code: %d", retval);
	      ROS_BREAK();
	    }
	  }

	  transform.setOrigin(accum.result_vec);
	  transform.setRotation(accum.result_quat);
	  transform.child_frame_id_ = mapped_src;
	  transform.frame_id_       = mapped_tgt;
	  transform.stamp_          = accum.time;
};


void Transformer::lookupTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                     const ros::Time& source_time, const std::string& fixed_frame, StampedTransform& transform) const
{
  tf::StampedTransform temp1, temp2;
  lookupTransform(fixed_frame, source_frame, source_time, temp1);
  lookupTransform(target_frame, fixed_frame, target_time, temp2);
  transform.setData( temp2 * temp1);
  transform.stamp_ = temp2.stamp_;
  transform.frame_id_ = target_frame;
  transform.child_frame_id_ = source_frame;

};


void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame,
                              const ros::Time& time, const ros::Duration& averaging_interval, 
                              geometry_msgs::Twist& twist) const
{
  lookupTwist(tracking_frame, observation_frame, observation_frame, tf::Point(0,0,0), tracking_frame, time, averaging_interval, twist);
};
// ref point is origin of tracking_frame, ref_frame = obs_frame


void Transformer::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                 const tf::Point & reference_point, const std::string& reference_point_frame, 
                 const ros::Time& time, const ros::Duration& averaging_interval, 
                 geometry_msgs::Twist& twist) const
{
  ros::Time latest_time, target_time;
  getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); ///\TODO check time on reference point too

  if (ros::Time() == time)
    target_time = latest_time;
  else
    target_time = time;

  ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);
  
  ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
  ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
  StampedTransform start, end;
  lookupTransform(observation_frame, tracking_frame, start_time, start);
  lookupTransform(observation_frame, tracking_frame, end_time, end);


  tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
  tf::Quaternion quat_temp;
  temp.getRotation(quat_temp);
  tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
  tfScalar ang = quat_temp.getAngle();
  
  double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
  double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
  double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();


  tf::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(), 
                       (delta_y)/corrected_averaging_interval.toSec(),
                       (delta_z)/corrected_averaging_interval.toSec());
  tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());


  // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)


  //correct for the position of the reference frame
  tf::StampedTransform inverse;
  lookupTransform(reference_frame,tracking_frame,  target_time, inverse);
  tf::Vector3 out_rot = inverse.getBasis() * twist_rot;
  tf::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);


  //Rereference the twist about a new reference point
  // Start by computing the original reference point in the reference frame:
  tf::Stamped<tf::Point> rp_orig(tf::Point(0,0,0), target_time, tracking_frame);
  transformPoint(reference_frame, rp_orig, rp_orig);
  // convert the requrested reference point into the right frame
  tf::Stamped<tf::Point> rp_desired(reference_point, target_time, reference_point_frame);
  transformPoint(reference_frame, rp_desired, rp_desired);
  // compute the delta
  tf::Point delta = rp_desired - rp_orig;
  // Correct for the change in reference point 
  out_vel = out_vel + out_rot * delta;
  // out_rot unchanged   

  /*
    printf("KDL: Rotation %f %f %f, Translation:%f %f %f\n", 
         out_rot.x(),out_rot.y(),out_rot.z(),
         out_vel.x(),out_vel.y(),out_vel.z());
  */   

  twist.linear.x =  out_vel.x();
  twist.linear.y =  out_vel.y();
  twist.linear.z =  out_vel.z();
  twist.angular.x =  out_rot.x();
  twist.angular.y =  out_rot.y();
  twist.angular.z =  out_rot.z();

};

bool Transformer::waitForTransform(const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time,
                                   const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  if (!using_dedicated_thread_)
  {
    std::string error_string = "Do not call waitForTransform unless you are using another thread for populating data. Without a dedicated thread it will always timeout.  If you have a seperate thread servicing tf messages, call setUsingDedicatedThread(true)";
    ROS_ERROR("%s",error_string.c_str());
    
    if (error_msg) 
      *error_msg = error_string;
    return false;
  }
  ros::Time start_time = now();
  std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
  std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

  while (ok() && now() >= start_time && (now() - start_time) < timeout)
  {
	  if (frameExists(mapped_tgt) && frameExists(mapped_src) && (canTransform(mapped_tgt, mapped_src, time, error_msg)))
		  return true;

	  usleep(polling_sleep_duration.sec * 1000000 + polling_sleep_duration.nsec / 1000); //hack to avoid calling ros::Time::now() in Duration.sleep
  }
  return false;
}

bool Transformer::canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
                    const ros::Time& time, std::string* error_msg) const
{
  if (target_id == 0 || source_id == 0)
  {
    return false;
  }

  CanTransformAccum accum;
  if (walkToTopParent(accum, time, target_id, source_id, error_msg) == NO_ERROR)
  {
    return true;
  }

  return false;
}

bool Transformer::canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
                                  const ros::Time& time, std::string* error_msg) const
{
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool Transformer::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const ros::Time& time, std::string* error_msg) const
{
	std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
	std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

	if (mapped_tgt == mapped_src)
		return true;

	boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  if (!frameExists(mapped_tgt) || !frameExists(mapped_src))
	  return false;

  CompactFrameID target_id = lookupFrameNumber(mapped_tgt);
  CompactFrameID source_id = lookupFrameNumber(mapped_src);

  return canTransformNoLock(target_id, source_id, time, error_msg);
}



bool Transformer::canTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                               const ros::Time& source_time, const std::string& fixed_frame,
                               std::string* error_msg) const
{
  return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
};

bool Transformer::waitForTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                                   const ros::Time& source_time, const std::string& fixed_frame,
                                   const ros::Duration& timeout, const ros::Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  return waitForTransform(target_frame, fixed_frame, target_time, timeout, polling_sleep_duration, error_msg) && waitForTransform(fixed_frame, source_frame, source_time, timeout, polling_sleep_duration, error_msg);
};


bool Transformer::getParent(const std::string& frame_id, ros::Time time, std::string& parent) const
{
  std::string mapped_frame_id = assert_resolved(tf_prefix_, frame_id);
  tf::TimeCache* cache;
  try
  {
    cache = getFrame(lookupFrameNumber(mapped_frame_id));
  }
  catch  (tf::LookupException &ex)
  {
    ROS_ERROR("Transformer::getParent: %s",ex.what());
    return false;
  }

  TransformStorage temp;
  if (! cache->getData(time, temp)) {
    ROS_DEBUG("Transformer::getParent: No data for parent of %s", mapped_frame_id.c_str());
    return false;
  }
  if (temp.frame_id_ == 0) {
    ROS_DEBUG("Transformer::getParent: No parent for %s", mapped_frame_id.c_str());
    return false;
  }

  parent = lookupFrameString(temp.frame_id_);
  return true;
};


bool Transformer::frameExists(const std::string& frame_id_str) const
{
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  std::string frame_id_resolveped = assert_resolved(tf_prefix_, frame_id_str);
  
  return frameIDs_.count(frame_id_resolveped);
}

void Transformer::setExtrapolationLimit(const ros::Duration& distance)
{
  max_extrapolation_distance_ = distance;
}

void Transformer::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
{
  if (!out)
  {
    return;
  }
  *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                     lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}

struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};

int Transformer::getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, ros::Time& time, std::string* error_string) const
{
	  std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
	  std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

	  if (!frameExists(mapped_tgt) || !frameExists(mapped_src)) {
		  time = ros::Time();
		  return LOOKUP_ERROR;
	  }

	  CompactFrameID source_id = lookupFrameNumber(mapped_src);
	  CompactFrameID target_id = lookupFrameNumber(mapped_tgt);
	  return getLatestCommonTime(source_id, target_id, time, error_string);
}


int Transformer::getLatestCommonTime(CompactFrameID target_id, CompactFrameID source_id, ros::Time & time, std::string * error_string) const
{
  if (source_id == target_id)
  {
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    time = now();
    return NO_ERROR;
  }

  std::vector<P_TimeAndFrameID> lct_cache;

  // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  ros::Time common_time = ros::TIME_MAX;
  while (frame != 0)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (!latest.first.isZero())
    {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      time = common_time;
      if (time == ros::TIME_MAX)
      {
        time = ros::Time();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_time = ros::TIME_MAX;
  CompactFrameID common_parent = 0;
  while (true)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      break;
    }

    if (!latest.first.isZero())
    {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
    if (it != lct_cache.end()) // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      time = common_time;
      if (time == ros::TIME_MAX)
      {
        time = ros::Time();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (common_parent == 0)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it)
    {
      if (!it->first.isZero())
      {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent)
      {
        break;
      }
    }
  }

  if (common_time == ros::TIME_MAX)
  {
    common_time = ros::Time();
  }

  time = common_time;
  return NO_ERROR;
}



/*
int Transformer::lookupLists(unsigned int target_frame, ros::Time time, unsigned int source_frame, TransformLists& lists, std::string * error_string) const
{
  ///\todo add fixed frame support

  //Clear lists before operating
  lists.forwardTransforms.clear();
  lists.inverseTransforms.clear();
  //  TransformLists mTfLs;
  if (target_frame == source_frame)
    return 0;  //Don't do anythign if we're not going anywhere

  TransformStorage temp;

  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  unsigned int last_inverse;
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  {
    if (error_string) *error_string = "Source frame '"+lookupFrameString(frame)+"' does not exist is tf tree.";
    return LOOKUP_ERROR;//throw LookupException("Frame didn't exist");
  }
  while (true)
    {
      //      printf("getting data from %d:%s \n", frame, lookupFrameString(frame).c_str());

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);

      if (! pointer->getData(time, temp))
      {
        last_inverse = frame;
        // this is thrown when there is no data
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0)
      {
        last_inverse = frame;
        break;
      }
      lists.inverseTransforms.push_back(temp);

      frame = temp.frame_id_num_;


      // Check if we've gone too deep.  A loop in the tree would cause this
      if (counter++ > MAX_GRAPH_DEPTH)
      {
        if (error_string)
        {
          std::stringstream ss;
          ss<<"The tf tree is invalid because it contains a loop." << std::endl
            << allFramesAsString() << std::endl;
          *error_string =ss.str();
        }
        return LOOKUP_ERROR;
        //        throw(LookupException(ss.str()));
      }
    }

  frame = target_frame;
  counter = 0;
  unsigned int last_forward;
  if (getFrame(frame) == NULL)
  {
    if (error_string) *error_string = "Target frame '"+lookupFrameString(frame)+"' does not exist is tf tree.";
    return LOOKUP_ERROR;
  }//throw LookupException("fixme");; //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  while (true)
    {

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);


      if(!  pointer->getData(time, temp))
      {
        last_forward = frame;
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0)
      {
        last_forward = frame;
        break;
      }
      //      std::cout << "pushing back" << temp.frame_id_ << std::endl;
      lists.forwardTransforms.push_back(temp);
      frame = temp.frame_id_num_;

      // Check if we've gone too deep.  A loop in the tree would cause this
      if (counter++ > MAX_GRAPH_DEPTH){
        if (error_string)
        {
          std::stringstream ss;
          ss<<"The tf tree is invalid because it contains a loop." << std::endl
            << allFramesAsString() << std::endl;
          *error_string = ss.str();
        }
        return LOOKUP_ERROR;//throw(LookupException(ss.str()));
      }
    }

  std::string connectivity_error("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                                 lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                                 "Tf has two or more unconnected trees.");
  // Check the zero length cases
  if (lists.inverseTransforms.size() == 0)
  {
    if (lists.forwardTransforms.size() == 0) //If it's going to itself it's already been caught
    {
      if (error_string) *error_string = connectivity_error;
      return CONNECTIVITY_ERROR;
    }

    if (last_forward != source_frame)  //\todo match with case A
    {
      if (error_string) *error_string = connectivity_error;
      return CONNECTIVITY_ERROR;
    }
    else return 0;
  }

  if (lists.forwardTransforms.size() == 0)
  {
    if (lists.inverseTransforms.size() == 0)  //If it's going to itself it's already been caught
    {//\todo remove THis is the same as case D
      if (error_string) *error_string = connectivity_error;
      return CONNECTIVITY_ERROR;
    }

    try
    {
      if (lookupFrameNumber(lists.inverseTransforms.back().frame_id_) != target_frame)
      {
        if (error_string) *error_string = connectivity_error;
        return CONNECTIVITY_ERROR;
    }
    else return 0;
    }
    catch (tf::LookupException & ex)
    {
      if (error_string) *error_string = ex.what();
      return LOOKUP_ERROR;
    }
  }


  // Make sure the end of the search shares a parent.
  if (last_forward != last_inverse)
  {
    if (error_string) *error_string = connectivity_error;
    return CONNECTIVITY_ERROR;
  }
  // Make sure that we don't have a no parent at the top
  try
  {
    if (lookupFrameNumber(lists.inverseTransforms.back().child_frame_id_) == 0 || lookupFrameNumber( lists.forwardTransforms.back().child_frame_id_) == 0)
    {
      //if (error_string) *error_string = "NO_PARENT at top of tree";
      if (error_string) *error_string = connectivity_error;
      return CONNECTIVITY_ERROR;
    }

    while (lookupFrameNumber(lists.inverseTransforms.back().child_frame_id_) == lookupFrameNumber(lists.forwardTransforms.back().child_frame_id_))
    {
      lists.inverseTransforms.pop_back();
      lists.forwardTransforms.pop_back();

      // Make sure we don't go beyond the beginning of the list.
      // (The while statement above doesn't fail if you hit the beginning of the list,
      // which happens in the zero distance case.)
      if (lists.inverseTransforms.size() == 0 || lists.forwardTransforms.size() == 0)
	break;
    }
  }
  catch (tf::LookupException & ex)
  {
    if (error_string) *error_string = ex.what();
    return LOOKUP_ERROR;
  }
  return 0;

  }
  */

/*
bool Transformer::test_extrapolation_one_value(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const
{
  std::stringstream ss;
  ss << std::fixed;
  ss.precision(3);

  if (tr.mode_ == ONE_VALUE)
  {
    if (tr.stamp_ - target_time > max_extrapolation_distance_ || target_time - tr.stamp_ > max_extrapolation_distance_)
    {
      if (error_string) {
        ss << "You requested a transform at time " << (target_time).toSec() 
           << ",\n but the tf buffer only contains a single transform " 
           << "at time " << tr.stamp_.toSec() << ".\n";
        if ( max_extrapolation_distance_ > ros::Duration(0))
        {
          ss << "The tf extrapollation distance is set to " 
             << (max_extrapolation_distance_).toSec() <<" seconds.\n";
        }
        *error_string = ss.str();
      }
      return true;
    }
  }
  return false;
}


bool Transformer::test_extrapolation_past(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const
{
  std::stringstream ss;
  ss << std::fixed;
  ss.precision(3);

  if (tr.mode_ == EXTRAPOLATE_BACK &&  tr.stamp_ - target_time > max_extrapolation_distance_)
  {
    if (error_string) {
      ss << "You requested a transform that is " << (now() - target_time).toSec() << " seconds in the past, \n"
         << "but the tf buffer only has a history of " << (now() - tr.stamp_).toSec()  << " seconds.\n";
      if ( max_extrapolation_distance_ > ros::Duration(0))
      {
        ss << "The tf extrapollation distance is set to " 
           << (max_extrapolation_distance_).toSec() <<" seconds.\n";
      }
      *error_string = ss.str();
    }
    return true;
  }
  return false;
}


bool Transformer::test_extrapolation_future(const ros::Time& target_time, const TransformStorage& tr, std::string* error_string) const
{
  std::stringstream ss;
  ss << std::fixed;
  ss.precision(3);

  if( tr.mode_ == EXTRAPOLATE_FORWARD && target_time - tr.stamp_ > max_extrapolation_distance_)
  {
    if (error_string){
      ss << "You requested a transform that is " << (now() - target_time).toSec()*1000 << " miliseconds in the past, \n"
         << "but the most recent transform in the tf buffer is " << (now() - tr.stamp_).toSec()*1000 << " miliseconds old.\n";
      if ( max_extrapolation_distance_ > ros::Duration(0))
      {
        ss << "The tf extrapollation distance is set to " 
           << (max_extrapolation_distance_).toSec() <<" seconds.\n";
      }
      *error_string = ss.str();
    }
    return true;
  }
  return false;
}


bool Transformer::test_extrapolation(const ros::Time& target_time, const TransformLists& lists, std::string * error_string) const
{
  std::stringstream ss;
  ss << std::fixed;
  ss.precision(3);
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
  {
    if (test_extrapolation_one_value(target_time, lists.inverseTransforms[i], error_string)) return true;
    if (test_extrapolation_past(target_time, lists.inverseTransforms[i], error_string)) return true;
    if (test_extrapolation_future(target_time, lists.inverseTransforms[i], error_string)) return true;
  }

  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
  {
    if (test_extrapolation_one_value(target_time, lists.forwardTransforms[i], error_string)) return true;
    if (test_extrapolation_past(target_time, lists.forwardTransforms[i], error_string)) return true;
    if (test_extrapolation_future(target_time, lists.forwardTransforms[i], error_string)) return true;
  }

  return false;
}
*/



/*
std::string Transformer::chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame) const
{
  std::string error_string;
  std::stringstream mstream;
  TransformLists lists;
  ///\todo check return code
  try
  {
    lookupLists(lookupFrameNumber(target_frame), target_time, lookupFrameNumber(source_frame), lists, &error_string);
  }
  catch (tf::LookupException &ex)
  {
    mstream << ex.what();
    return mstream.str();
  }
  mstream << "Inverse Transforms:" <<std::endl;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      mstream << lists.inverseTransforms[i].child_frame_id_<<", ";
    }
  mstream << std::endl;

  mstream << "Forward Transforms: "<<std::endl ;
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      mstream << lists.forwardTransforms[i].child_frame_id_<<", ";
    }
  mstream << std::endl;
  return mstream.str();
}
*/

//@todo - Fix this to work with new data structures
void Transformer::chainAsVector(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame, std::vector<std::string>& output) const
{
  std::string error_string;

  output.clear(); //empty vector

  std::stringstream mstream;
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCache* frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if (frame_ptr->getData(ros::Time(), temp))
        frame_id_num = temp.frame_id_;
      else
      {
        frame_id_num = 0;
      }
      output.push_back(frameIDs_reverse[frame_id_num]);
  }
}

std::string Transformer::allFramesAsString() const
{
  std::stringstream mstream;
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCache* frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if(  frame_ptr->getData(ros::Time(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
  }

  return mstream.str();
}

std::string Transformer::allFramesAsDot() const
{
  std::stringstream mstream;
  mstream << "digraph G {" << std::endl;
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  ros::Time current_time = now();

  if (frames_.size() ==1)
    mstream <<"\"no tf data recieved\"";

  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);
    
   //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    if(  getFrame(counter)->getData(ros::Time(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    if (frame_id_num != 0)
    {
      std::string authority = "no recorded authority";
      std::map<unsigned int, std::string>::const_iterator it = frame_authority_.find(counter);
      if (it != frame_authority_.end())
        authority = it->second;

      double rate = getFrame(counter)->getListLength() / std::max((getFrame(counter)->getLatestTimestamp().toSec() -
                                                                   getFrame(counter)->getOldestTimestamp().toSec() ), 0.0001);

      mstream << std::fixed; //fixed point notation
      mstream.precision(3); //3 decimal places
      mstream << "\"" << frameIDs_reverse[frame_id_num] << "\"" << " -> "
              << "\"" << frameIDs_reverse[counter] << "\"" << "[label=\""
        //<< "Time: " << current_time.toSec() << "\\n"
              << "Broadcaster: " << authority << "\\n"
              << "Average rate: " << rate << " Hz\\n"
              << "Most recent transform: " << (current_time - getFrame(counter)->getLatestTimestamp()).toSec() << " sec old \\n"
        //    << "(time: " << getFrame(counter)->getLatestTimestamp().toSec() << ")\\n"
        //    << "Oldest transform: " << (current_time - getFrame(counter)->getOldestTimestamp()).toSec() << " sec old \\n"
        //    << "(time: " << (getFrame(counter)->getOldestTimestamp()).toSec() << ")\\n"
              << "Buffer length: " << (getFrame(counter)->getLatestTimestamp()-getFrame(counter)->getOldestTimestamp()).toSec() << " sec\\n"
              <<"\"];" <<std::endl;
    }
  }
  
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    if(  getFrame(counter)->getData(ros::Time(), temp))
      frame_id_num = temp.frame_id_;
    else
      {
	frame_id_num = 0;
      }

    if(frameIDs_reverse[frame_id_num]=="NO_PARENT")
    {
      mstream << "edge [style=invis];" <<std::endl;
      mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
              << "\"Recorded at time: " << current_time.toSec() << "\"[ shape=plaintext ] ;\n "
	      << "}" << "->" << "\"" << frameIDs_reverse[counter]<<"\";" <<std::endl;
    }
  }
  mstream << "}";
  return mstream.str();
}


bool Transformer::ok() const { return true; }

void Transformer::getFrameStrings(std::vector<std::string> & vec) const
{
  vec.clear();

  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    vec.push_back(frameIDs_reverse[counter]);
  }
  return;
}

tf::TimeCache* Transformer::getFrame(unsigned int frame_id) const
{
  if (frame_id == 0) /// @todo check larger values too
    return NULL;
  else
    return frames_[frame_id];
};


void Transformer::transformQuaternion(const std::string& target_frame, const Stamped<Quaternion>& stamped_in, Stamped<Quaternion>& stamped_out) const
{
  tf::assertQuaternionValid(stamped_in);

  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame,
                                  const Stamped<tf::Vector3>& stamped_in,
                                  Stamped<tf::Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  /** \todo may not be most efficient */
  tf::Vector3 end = stamped_in;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::Vector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};

void Transformer::transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
                                      const Stamped<Quaternion>& stamped_in,
                                      const std::string& fixed_frame,
                                      Stamped<Quaternion>& stamped_out) const
{
  tf::assertQuaternionValid(stamped_in);
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame, const ros::Time& target_time,
                                  const Stamped<Vector3>& stamped_in,
                                  const std::string& fixed_frame,
                                  Stamped<Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  /** \todo may not be most efficient */
  tf::Vector3 end = stamped_in;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::Vector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const ros::Time& target_time,
                                 const Stamped<Point>& stamped_in,
                                 const std::string& fixed_frame,
                                 Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};

void Transformer::transformPose(const std::string& target_frame, const ros::Time& target_time,
                                const Stamped<Pose>& stamped_in,
                                const std::string& fixed_frame,
                                Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};

boost::signals::connection Transformer::addTransformsChangedListener(boost::function<void(void)> callback)
{
  boost::mutex::scoped_lock lock(transforms_changed_mutex_);
  return transforms_changed_.connect(callback);
}

void Transformer::removeTransformsChangedListener(boost::signals::connection c)
{
  boost::mutex::scoped_lock lock(transforms_changed_mutex_);
  c.disconnect();
}
