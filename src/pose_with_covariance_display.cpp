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

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/validate_floats.h>

#include "pose_with_covariance_display.h"
#include "covariance_visual.h"

#include <Eigen/Dense>

using namespace rviz;

namespace mav_pose_rviz_plugin
{

PoseWithCovarianceDisplay::PoseWithCovarianceDisplay()
{
}

void PoseWithCovarianceDisplay::onInitialize()
{
  MFDClass::onInitialize();

  boost::shared_ptr<CovarianceVisual> visual(new CovarianceVisual(scene_manager_, scene_node_, true) );
  covariance_ = visual;

}

PoseWithCovarianceDisplay::~PoseWithCovarianceDisplay()
{
}

void PoseWithCovarianceDisplay::onEnable()
{
  MFDClass::onEnable();
}


void PoseWithCovarianceDisplay::processMessage( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message )
{
  if( !validateFloats( message->pose.pose ) || !validateFloats( message->pose.covariance ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose.pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  covariance_->setPosition( position );
  covariance_->setOrientation( orientation );

  context_->queueRender();
}

void PoseWithCovarianceDisplay::reset()
{
  MFDClass::reset();
}

} // namespace mav_pose_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( mav_pose_rviz_plugin::PoseWithCovarianceDisplay, rviz::Display )
