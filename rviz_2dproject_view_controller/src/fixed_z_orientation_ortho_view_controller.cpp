#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz/ogre_helpers/ogre_vector.h>
#include <OgreViewport.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/orthographic.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/viewport_mouse_event.h>

#include "fixed_z_orientation_ortho_view_controller.h"

namespace FixedZOrientationOrthoViewControllerNs
{
  FixedZOrientationOrthoViewController::FixedZOrientationOrthoViewController() : dragging_(false)
  {
    scale_property_ =
        new rviz::FloatProperty("Scale", 10, "How much to scale up the size of things in the scene.", this);
    angle_property_ = new rviz::FloatProperty("Angle", 0, "Angle around the X axis to rotate.", this);
    x_property_ = new rviz::FloatProperty("X", 0, "X component of camera position.", this);
    y_property_ = new rviz::FloatProperty("Y", 0, "Y component of camera position.", this);
    z_property_ = new rviz::FloatProperty("Z", 0, "Z component of camera position.", this);

    InvertDirection_ = new rviz::BoolProperty("Invert Z Axis", false, "Invert camera's Z axis for Y-down environments/models.",
                                              this, &FixedZOrientationOrthoViewController::InvertZAxis);
    InvertDirectionInStdBool_ = false;
  }

  FixedZOrientationOrthoViewController::~FixedZOrientationOrthoViewController()
  {
  }

  void FixedZOrientationOrthoViewController::onInitialize()
  {
    rviz::FramePositionTrackingViewController::onInitialize();

    camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    camera_->setFixedYawAxis(false);
    invert_z_->hide();
  }

  void FixedZOrientationOrthoViewController::reset()
  {
    scale_property_->setFloat(10);
    angle_property_->setFloat(0);
    x_property_->setFloat(0);
    y_property_->setFloat(0);
    z_property_->setFloat(0);
  }

  void FixedZOrientationOrthoViewController::handleMouseEvent(rviz::ViewportMouseEvent &event)
  {
    if (event.shift())
    {
      setStatus("<b>Left-Click:</b> Move X/Y.");
    }
    else
    {
      setStatus("<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  "
                "<b>Shift</b>: More options.");
    }

    bool moved = false;

    int32_t diff_x = 0;
    int32_t diff_y = 0;

    if (event.type == QEvent::MouseButtonPress)
    {
      dragging_ = true;
    }
    else if (event.type == QEvent::MouseButtonRelease)
    {
      dragging_ = false;
    }
    else if (dragging_ && event.type == QEvent::MouseMove)
    {
      diff_x = event.x - event.last_x;
      diff_y = event.y - event.last_y;
      moved = true;
    }

    if (event.left() && !event.shift())
    {
      setCursor(Rotate2D);
      angle_property_->add(diff_x * 0.005);
      orientCamera();
    }
    else if (event.middle() || (event.shift() && event.left()))
    {
      setCursor(MoveXY);
      float scale = scale_property_->getFloat();
      move(-diff_x / scale, diff_y / scale);
    }
    else if (event.right())
    {
      setCursor(Zoom);
      scale_property_->multiply(1.0 - diff_y * 0.01);
    }
    else
    {
      setCursor(event.shift() ? MoveXY : Rotate2D);
    }

    if (event.wheel_delta != 0)
    {
      int diff = event.wheel_delta;
      scale_property_->multiply(1.0 - (-diff) * 0.001);

      moved = true;
    }

    if (moved)
    {
      context_->queueRender();
      emitConfigChanged();
    }
  }

  void FixedZOrientationOrthoViewController::orientCamera()
  {

    camera_->setOrientation(
        Ogre::Quaternion(Ogre::Radian(angle_property_->getFloat()), Ogre::Vector3::UNIT_Z));
    if (!InvertDirectionInStdBool_)
    {
      camera_->setDirection(Ogre::Vector3::NEGATIVE_UNIT_Z);
    }
    else
    {
      camera_->setDirection(Ogre::Vector3::UNIT_Z);
    }
    // camera_->setDirection(Ogre::Vector3::NEGATIVE_UNIT_Y);
  }

  void FixedZOrientationOrthoViewController::mimic(rviz::ViewController *source_view)
  {
    rviz::FramePositionTrackingViewController::mimic(source_view);

    if (FixedZOrientationOrthoViewController *source_ortho =
            qobject_cast<FixedZOrientationOrthoViewController *>(source_view))
    {
      scale_property_->setFloat(source_ortho->scale_property_->getFloat());
      angle_property_->setFloat(source_ortho->angle_property_->getFloat());
      x_property_->setFloat(source_ortho->x_property_->getFloat());
      y_property_->setFloat(source_ortho->y_property_->getFloat());
      z_property_->setFloat(source_ortho->z_property_->getFloat());
    }
    else
    {
      Ogre::Camera *source_camera = source_view->getCamera();
      setPosition(source_camera->getPosition());
    }
  }

  void FixedZOrientationOrthoViewController::update(float dt, float ros_dt)
  {
    rviz::FramePositionTrackingViewController::update(dt, ros_dt);
    updateCamera();
  }

  void FixedZOrientationOrthoViewController::lookAt(const Ogre::Vector3 &point)
  {
    setPosition(point - target_scene_node_->getPosition());
  }

  void FixedZOrientationOrthoViewController::onTargetFrameChanged(
      const Ogre::Vector3 &old_reference_position,
      const Ogre::Quaternion & /*old_reference_orientation*/)
  {
    move(old_reference_position.x - reference_position_.x,
         old_reference_position.y - reference_position_.y);
  }

  void FixedZOrientationOrthoViewController::updateCamera()
  {
    orientCamera();

    float width = camera_->getViewport()->getActualWidth();
    float height = camera_->getViewport()->getActualHeight();

    float scale = scale_property_->getFloat();
    Ogre::Matrix4 proj;

    rviz::buildScaledOrthoMatrixForXProjection(proj, -width / scale / 2, width / scale / 2, -height / scale / 2,
                                               height / scale / 2, camera_->getNearClipDistance(), camera_->getFarClipDistance());

    // Apply scaling
    Ogre::Matrix4 scale_transform = Ogre::Matrix4::IDENTITY;
    scale_transform = Ogre::Matrix4::getScale(Ogre::Vector3(scale, scale, scale));

    camera_->setCustomProjectionMatrix(true, proj); // proj

    // For Z, we use half of the far-clip distance set in
    // selection_context.cpp, so that the shader program which computes
    // depth can see equal distances above and below the Z=0 plane.
    camera_->setPosition(x_property_->getFloat(), y_property_->getFloat(), z_property_->getFloat());
  }

  void FixedZOrientationOrthoViewController::setPosition(const Ogre::Vector3 &pos_rel_target)
  {
    x_property_->setFloat(pos_rel_target.x);
    y_property_->setFloat(pos_rel_target.y);
    z_property_->setFloat(pos_rel_target.z);
  }

  void FixedZOrientationOrthoViewController::move(float dx, float dy)
  {
    float angle = angle_property_->getFloat();
    x_property_->add(dx * std::cos(angle) - dy * std::sin(angle));
    y_property_->add(dx * std::sin(angle) + dy * std::cos(angle));
  }

  void FixedZOrientationOrthoViewController::InvertZAxis()
  {
    if (InvertDirection_->getBool())
    {
      InvertDirectionInStdBool_ = true;
    }
    else
    {
      InvertDirectionInStdBool_ = false;
    }
  }

} // end namespace FixedZOrientationOrthoViewControllerNs

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(FixedZOrientationOrthoViewControllerNs::FixedZOrientationOrthoViewController, rviz::ViewController)
