#ifndef RVIZ_FIXED_Y_ORIENTATION_ORTHO_VIEW_CONTROLLER_H
#define RVIZ_FIXED_Y_ORIENTATION_ORTHO_VIEW_CONTROLLER_H

#include <rviz/frame_position_tracking_view_controller.h>

#include <OgreQuaternion.h>

#include "fixed_xyz_orientation_ortho_view_controller_base.h"

namespace FixedYOrientationOrthoViewControllerNs
{

  class FixedYOrientationOrthoViewController : public rviz::FramePositionTrackingViewController
  {
    Q_OBJECT
  public:
    FixedYOrientationOrthoViewController();
    ~FixedYOrientationOrthoViewController() override;

    void onInitialize() override;

    void handleMouseEvent(rviz::ViewportMouseEvent &evt) override;

    void lookAt(const Ogre::Vector3 &point_rel_world) override;

    void reset() override;

    /** @brief Configure the settings of this view controller to give,
     * as much as possible, a similar view as that given by the
     * @a source_view.
     *
     * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
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
    rviz::FloatProperty *z_property_;
    bool dragging_;
  };

} // end namespace FixedYOrientationOrthoViewControllerNs

#endif // RVIZ_FIXED_Y_ORIENTATION_ORTHO_VIEW_CONTROLLER_H
