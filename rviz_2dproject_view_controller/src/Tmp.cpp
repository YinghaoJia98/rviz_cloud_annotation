#include "custom_ortho_view.h"
#include <pluginlib/class_list_macros.h>
#include <rviz/display_context.h>

namespace custom_rviz_plugin
{
    CustomOrthoViewController::CustomOrthoViewController()
        : custom_angle_(0.0), scale_factor_(1.0), projection_plane_("X-Z")
    {
    }

    void CustomOrthoViewController::onInitialize()
    {
        camera_ = scene_manager_->createCamera("CustomOrthoCamera");
        context_->getSceneManager()->getRootSceneNode()->attachObject(camera_);
        camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC); // Orthographic projection
    }

    void CustomOrthoViewController::reset()
    {
        custom_angle_ = 0.0;
        scale_factor_ = 1.0;
        projection_plane_ = "X-Z"; // Default projection
    }

    void CustomOrthoViewController::onUpdate(float dt, float ros_dt)
    {
        // Apply custom transformations based on parameters
        Ogre::Vector3 position = camera_->getPosition();

        // Adjust the position and rotation based on the selected plane
        if (projection_plane_ == "X-Z")
        {
            position.y = 0.0; // Project onto X-Z plane
            camera_->setOrientation(Ogre::Quaternion(Ogre::Degree(custom_angle_), Ogre::Vector3::UNIT_Y));
        }
        else if (projection_plane_ == "Y-Z")
        {
            position.x = 0.0; // Project onto Y-Z plane
            camera_->setOrientation(Ogre::Quaternion(Ogre::Degree(custom_angle_), Ogre::Vector3::UNIT_X));
        }

        camera_->setPosition(position);

        // Apply scaling
        Ogre::Matrix4 scale_transform = Ogre::Matrix4::IDENTITY;
        scale_transform = Ogre::Matrix4::getScale(Ogre::Vector3(scale_factor_, scale_factor_, scale_factor_));
        camera_->setCustomProjectionMatrix(true, scale_transform);
    }

    void CustomOrthoViewController::setAngle(float angle)
    {
        custom_angle_ = angle;
    }

    void CustomOrthoViewController::setScale(float scale)
    {
        scale_factor_ = scale;
    }

    void CustomOrthoViewController::setProjectionPlane(const std::string &plane)
    {
        projection_plane_ = plane;
    }
} // namespace custom_rviz_plugin

PLUGINLIB_EXPORT_CLASS(custom_rviz_plugin::CustomOrthoViewController, rviz::ViewController)
