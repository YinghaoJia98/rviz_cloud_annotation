#ifndef RVIZ_FIXED_Z_ORIENTATION_ORTHO_VIEW_CONTROLLER_H
#define RVIZ_FIXED_Z_ORIENTATION_ORTHO_VIEW_CONTROLLER_H

#include <rviz/frame_position_tracking_view_controller.h>

#include <OgreQuaternion.h>

#include "fixed_xyz_orientation_ortho_view_controller_base.h"

namespace FixedZOrientationOrthoViewControllerNs
{

  class FixedZOrientationOrthoViewController : public rviz::FramePositionTrackingViewController
  {
    Q_OBJECT
  public:
    FixedZOrientationOrthoViewController();
    ~FixedZOrientationOrthoViewController() override;

    void onInitialize() override;

    void handleMouseEvent(rviz::ViewportMouseEvent &evt) override;

    void lookAt(const Ogre::Vector3 &point_rel_world) override;

    void reset() override;

    void mimic(rviz::ViewController *source_view) override;

    void update(float dt, float ros_dt) override;

  protected:
    void onTargetFrameChanged(const Ogre::Vector3 &old_reference_position,
                              const Ogre::Quaternion &old_reference_orientation) override;

    /** Set the camera orientation based on angle_. */
    void orientCamera();

    void setPosition(const Ogre::Vector3 &pos_rel_target);
    void move(float x, float z);
    void updateCamera();

    rviz::FloatProperty *scale_property_;
    rviz::FloatProperty *angle_property_;
    rviz::FloatProperty *x_property_;
    rviz::FloatProperty *y_property_;
    rviz::FloatProperty *z_property_;
    bool dragging_;

    rviz::BoolProperty *InvertDirection_;
    bool InvertDirectionInStdBool_;

  private Q_SLOTS:
    void InvertZAxis();
  };

} // end namespace FixedZOrientationOrthoViewControllerNs

#endif // RVIZ_FIXED_Z_ORIENTATION_ORTHO_VIEW_CONTROLLER_H
