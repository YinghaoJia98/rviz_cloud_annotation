#ifndef RVIZ_FIXED_XYZ_ORIENTATION_ORTHO_VIEW_CONTROLLER_BASE_H
#define RVIZ_FIXED_XYZ_ORIENTATION_ORTHO_VIEW_CONTROLLER_BASE_H

namespace Ogre
{
    class Matrix4;
}

namespace rviz
{
    class FloatProperty;
    class Shape;
    class SceneNode;
    void buildScaledOrthoMatrixForXProjection(Ogre::Matrix4 &proj,
                                              float left,
                                              float right,
                                              float bottom,
                                              float top,
                                              float near,
                                              float far);
}
#endif // RVIZ_FIXED_XYZ_ORIENTATION_ORTHO_VIEW_CONTROLLER_BASE_H