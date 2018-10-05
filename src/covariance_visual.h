#ifndef COVARIANCE_VISUAL_H
#define COVARIANCE_VISUAL_H

#include <rviz/ogre_helpers/object.h>

#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/PoseWithCovariance.h>

#include <Eigen/Dense>

#include <OgreVector3.h>
#include <OgreColourValue.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
}

namespace rviz
{
class Shape;
}

namespace Eigen
{
  typedef Matrix<double,6,6> Matrix6d;
}

namespace mav_pose_rviz_plugin
{

// class CovarianceProperty;

/**
 * \class CovarianceVisual
 * \brief CovarianceVisual consisting in a ellipse for position and 2D ellipses along the axis for orientation.
 */
class CovarianceVisual : public rviz::Object
{
public:

// private:
  /**
   * \brief Private Constructor
   * 
   * CovarianceVisual can only be constructed by friend class CovarianceProperty.
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_object A rviz object that this covariance will be attached.
   * @param is_local_rotation Initial attachment of the rotation part
   * @param is_visible Initial visibility
   * @param pos_scale Scale of the position covariance
   * @param ori_scale Scale of the orientation covariance
   */
  CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_local_rotation, bool is_visible = true, float pos_scale = 1.0f, float ori_scale = 0.1f, float ori_offset = 0.1f);
public:
  virtual ~CovarianceVisual();
  /**
   * \brief Get the shape used to display position covariance
   * @return the shape used to display position covariance
   */
  rviz::Shape* getPositionShape() { return position_shape_; }

  /**
   * \brief Sets user data on all ogre objects we own
   */
  virtual void setUserData( const Ogre::Any& data );

  /**
   * \brief Sets visibility of this covariance
   * 
   * Convenience method that sets visibility of both position and orientation parts.
   */
  virtual void setVisible( bool visible );

  /**
   * \brief Sets position of the frame this covariance is attached
   */
  virtual void setPosition( const Ogre::Vector3& position );

  /**
   * \brief Sets orientation of the frame this covariance is attached
   */
  virtual void setOrientation( const Ogre::Quaternion& orientation );

private:

  Ogre::SceneNode* root_node_;

  rviz::Shape* position_shape_;   ///< Ellipse used for the position covariance

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  virtual void setScale( const Ogre::Vector3& scale ) {};
  virtual void setColor( float r, float g, float b, float a ) {};
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

};

} // namespace mav_pose_rviz_plugin

#endif /* COVARIANCE_VISUAL_H */
