/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "fixed_x_orientation_ortho_view_controller.h"

namespace rviz
{
  void buildScaledOrthoMatrixForXProjection(Ogre::Matrix4 &proj,
                                            float left,
                                            float right,
                                            float bottom,
                                            float top,
                                            float near,
                                            float far)
  {
    float invw = 1 / (right - left);
    float invh = 1 / (top - bottom);
    float invd = 1 / (far - near);

    proj = Ogre::Matrix4::ZERO;
    proj[0][0] = 2 * invw;
    proj[0][3] = -(right + left) * invw;
    proj[1][1] = 2 * invh;
    proj[1][3] = -(top + bottom) * invh;
    proj[2][2] = -2 * invd;
    proj[2][3] = -(far + near) * invd;
    proj[3][3] = 1;
  }
} // namespace rviz

namespace FixedXOrientationOrthoViewControllerNs
{
  FixedXOrientationOrthoViewController::FixedXOrientationOrthoViewController() : dragging_(false)
  {
    scale_property_ =
        new rviz::FloatProperty("Scale", 10, "How much to scale up the size of things in the scene.", this);
    angle_property_ = new rviz::FloatProperty("Angle", 0, "Angle around the X axis to rotate.", this);
    y_property_ = new rviz::FloatProperty("Y", 0, "Y component of camera position.", this);
    z_property_ = new rviz::FloatProperty("Z", 0, "Z component of camera position.", this);
  }

  FixedXOrientationOrthoViewController::~FixedXOrientationOrthoViewController()
  {
  }

  void FixedXOrientationOrthoViewController::onInitialize()
  {
    rviz::FramePositionTrackingViewController::onInitialize();

    camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    camera_->setFixedYawAxis(false);
    invert_z_->hide();

    // camera_ = context_->getSceneManager()->createCamera("CustomOrthoCamera");
    // context_->getSceneManager()->getRootSceneNode()->attachObject(camera_);
    // camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }

  void FixedXOrientationOrthoViewController::reset()
  {
    scale_property_->setFloat(10);
    angle_property_->setFloat(0);
    y_property_->setFloat(0);
    z_property_->setFloat(0);
  }

  void FixedXOrientationOrthoViewController::handleMouseEvent(rviz::ViewportMouseEvent &event)
  {
    if (event.shift())
    {
      setStatus("<b>Left-Click:</b> Move Y/Z.");
    }
    else
    {
      setStatus("<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move Y/Z.  <b>Right-Click:</b>: Zoom.  "
                "<b>Shift</b>: More options.");
    }

    bool moved = false;

    int32_t diff_y = 0;
    int32_t diff_z = 0;

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
      diff_y = event.x - event.last_x;
      diff_z = event.y - event.last_y;
      moved = true;
    }

    if (event.left() && !event.shift())
    {
      setCursor(Rotate2D);
      angle_property_->add(diff_y * 0.005);
      orientCamera();
    }
    else if (event.middle() || (event.shift() && event.left()))
    {
      setCursor(MoveXY);
      float scale = scale_property_->getFloat();
      move(-diff_z / scale, -diff_y / scale);
    }
    else if (event.right())
    {
      setCursor(Zoom);
      scale_property_->multiply(1.0 - diff_z * 0.01);
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

  void FixedXOrientationOrthoViewController::orientCamera()
  {

    camera_->setOrientation(
        Ogre::Quaternion(Ogre::Radian(angle_property_->getFloat()), Ogre::Vector3::UNIT_X));

    camera_->setDirection(Ogre::Vector3::NEGATIVE_UNIT_X);
    // std::cout << "camera orientation is " << camera_->getOrientation() << std::endl;
    // std::cout << "camera real orientation is " << camera_->getRealOrientation() << std::endl;
    // std::cout << "camera position is " << camera_->getPosition() << std::endl;
    // std::cout << "camera real position is " << camera_->getRealPosition() << std::endl;
    // std::cout << "camera direction is " << camera_->getDirection() << std::endl;
    // std::cout << "camera RealDirection is " << camera_->getRealDirection() << std::endl;
    // std::cout << "camera up is " << camera_->getUp() << std::endl;
    // std::cout << "camera real up is " << camera_->getRealUp() << std::endl;
    // std::cout << "camera right is " << camera_->getRight() << std::endl;
    // std::cout << "camera real right is " << camera_->getRealRight() << std::endl;

    // std::cout << "The whole camera is " << *camera_ << std::endl;
  }

  void FixedXOrientationOrthoViewController::mimic(rviz::ViewController *source_view)
  {
    rviz::FramePositionTrackingViewController::mimic(source_view);

    if (FixedXOrientationOrthoViewController *source_ortho =
            qobject_cast<FixedXOrientationOrthoViewController *>(source_view))
    {
      scale_property_->setFloat(source_ortho->scale_property_->getFloat());
      angle_property_->setFloat(source_ortho->angle_property_->getFloat());
      y_property_->setFloat(source_ortho->y_property_->getFloat());
      z_property_->setFloat(source_ortho->z_property_->getFloat());
    }
    else
    {
      Ogre::Camera *source_camera = source_view->getCamera();
      setPosition(source_camera->getPosition());
    }
  }

  void FixedXOrientationOrthoViewController::update(float dt, float ros_dt)
  {
    rviz::FramePositionTrackingViewController::update(dt, ros_dt);
    updateCamera();
  }

  void FixedXOrientationOrthoViewController::lookAt(const Ogre::Vector3 &point)
  {
    setPosition(point - target_scene_node_->getPosition());
  }

  void FixedXOrientationOrthoViewController::onTargetFrameChanged(
      const Ogre::Vector3 &old_reference_position,
      const Ogre::Quaternion & /*old_reference_orientation*/)
  {
    move(old_reference_position.y - reference_position_.y,
         old_reference_position.z - reference_position_.z);
  }

  void FixedXOrientationOrthoViewController::updateCamera()
  {
    orientCamera();

    float width = camera_->getViewport()->getActualWidth();
    float height = camera_->getViewport()->getActualHeight();

    float scale = scale_property_->getFloat();
    Ogre::Matrix4 proj;
    // rviz::buildScaledOrthoMatrix(proj, -width / scale / 2, width / scale / 2, -height / scale / 2,
    //                              height / scale / 2, camera_->getNearClipDistance(),
    //                              camera_->getFarClipDistance());

    rviz::buildScaledOrthoMatrixForXProjection(proj, -width / scale / 2, width / scale / 2, -height / scale / 2,
                                               height / scale / 2, camera_->getNearClipDistance(), camera_->getFarClipDistance());

    // rviz::buildScaledOrthoMatrixForXProjection(proj, camera_->getNearClipDistance(), camera_->getFarClipDistance(),
    //                                            -width / scale / 2, width / scale / 2,
    //                                            -height / scale / 2, height / scale / 2);

    // Apply scaling
    Ogre::Matrix4 scale_transform = Ogre::Matrix4::IDENTITY;
    scale_transform = Ogre::Matrix4::getScale(Ogre::Vector3(scale, scale, scale));

    camera_->setCustomProjectionMatrix(true, proj); // proj

    // For Z, we use half of the far-clip distance set in
    // selection_context.cpp, so that the shader program which computes
    // depth can see equal distances above and below the Z=0 plane.
    camera_->setPosition(100.0, y_property_->getFloat(), z_property_->getFloat());
  }

  void FixedXOrientationOrthoViewController::setPosition(const Ogre::Vector3 &pos_rel_target)
  {
    y_property_->setFloat(pos_rel_target.y);
    z_property_->setFloat(pos_rel_target.z);
  }

  void FixedXOrientationOrthoViewController::move(float dy, float dz)
  {
    float angle = angle_property_->getFloat();
    y_property_->add(dy * std::cos(angle) - dz * std::sin(angle));
    z_property_->add(dy * std::sin(angle) + dz * std::cos(angle));
  }

} // end namespace FixedXOrientationOrthoViewControllerNs

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(FixedXOrientationOrthoViewControllerNs::FixedXOrientationOrthoViewController, rviz::ViewController)
