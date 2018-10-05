#include "covariance_visual.h"

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreQuaternion.h>

#include <ros/console.h>

#include <sstream>

using namespace rviz;

namespace mav_pose_rviz_plugin
{

namespace
{

}

CovarianceVisual::CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_local_rotation, bool is_visible, float pos_scale, float ori_scale, float ori_offset)
: Object( scene_manager )
{
  // Main node of the visual
  root_node_ = parent_node->createChildSceneNode();
  position_shape_ = new rviz::Shape(rviz::Shape::Cube, scene_manager_, root_node_);

  root_node_->setVisible( true );
  setVisible( is_visible );
}

CovarianceVisual::~CovarianceVisual()
{
  ROS_INFO("destructor");
  delete position_shape_;

  ROS_INFO("delete");
  
  scene_manager_->destroySceneNode( root_node_->getName() );
  ROS_INFO("exit destructor");
}


void CovarianceVisual::setUserData( const Ogre::Any& data )
{
  position_shape_->setUserData( data );
}

void CovarianceVisual::setVisible( bool visible )
{
  root_node_->setVisible( visible );
}

const Ogre::Vector3& CovarianceVisual::getPosition() 
{
  return root_node_->getPosition();
}

const Ogre::Quaternion& CovarianceVisual::getOrientation()
{
  return root_node_->getOrientation();
}

void CovarianceVisual::setPosition( const Ogre::Vector3& position )
{
  root_node_->setPosition( position );
}

void CovarianceVisual::setOrientation( const Ogre::Quaternion& orientation )
{
  root_node_->setVisible(true);
  root_node_->setOrientation( orientation );
}

} // namespace mav_pose_rviz_plugin
