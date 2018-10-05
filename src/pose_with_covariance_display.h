// -*- c++-mode -*-
#ifndef POSE_WITH_COVARIANCE_DISPLAY_H
#define POSE_WITH_COVARIANCE_DISPLAY_H

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rviz/message_filter_display.h>
#include <rviz/selection/forwards.h>

namespace rviz
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class Shape;
}

namespace mav_pose_rviz_plugin
{

class CovarianceVisual;
// class CovarianceProperty;

class PoseWithCovarianceDisplaySelectionHandler;
typedef boost::shared_ptr<PoseWithCovarianceDisplaySelectionHandler> PoseWithCovarianceDisplaySelectionHandlerPtr;

/** @brief Displays the pose from a geometry_msgs::PoseWithCovarianceStamped message. */
class PoseWithCovarianceDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>
{
Q_OBJECT
public:

  PoseWithCovarianceDisplay();
  virtual ~PoseWithCovarianceDisplay();

  virtual void onInitialize();
  virtual void reset();

protected:
  virtual void onEnable();

private Q_SLOTS:

private:
  void clear();

  virtual void processMessage( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message );

  boost::shared_ptr<CovarianceVisual> covariance_;


  friend class PoseWithCovarianceDisplaySelectionHandler;
};

} // namespace mav_pose_rviz_plugin

#endif // POSE_WITH_COVARIANCE_DISPLAY_H
