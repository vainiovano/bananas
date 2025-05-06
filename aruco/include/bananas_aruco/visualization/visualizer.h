#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gsl/pointers>

#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreInput.h>
#include <OgrePrerequisites.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>
#include <bananas_aruco/world.h>

namespace visualizer {

class Visualizer {
  public:
    Visualizer();

    void update(world::BoardId id,
                const affine_rotation::AffineRotation &placement);
    void update(const world::FitResult &fit);

    /// Render a new frame.
    void refresh();

    void addObject(world::BoardId id, const board::BoxSettings &box);
    void addObject(world::BoardId id, const board::GridSettings &grid);
    void forceVisible(world::BoardId id);

  private:
    class KeyHandler : public OgreBites::InputListener {
      public:
        explicit KeyHandler(OgreBites::CameraMan *camera_manager);

        auto
        mousePressed(const OgreBites::MouseButtonEvent &evt) -> bool override;
        auto
        mouseReleased(const OgreBites::MouseButtonEvent &evt) -> bool override;

      private:
        OgreBites::CameraMan *camera_manager_;
    };

    /// A wrapper that initializes an OgreBites::ApplicationContext and calls
    /// Ogre::ApplicationContextBase::initApp() for it.
    class InitializedContext : public OgreBites::ApplicationContext {
      public:
        InitializedContext();
    };

    struct ColoredObject {
        gsl::not_null<Ogre::SceneNode *> node;
        Ogre::MaterialPtr material;
    };

    InitializedContext context_{};
    gsl::not_null<Ogre::Root *> root_;
    gsl::not_null<Ogre::SceneManager *> scene_manager_;
    gsl::not_null<Ogre::SceneNode *> camera_node_;
    OgreBites::CameraMan camera_manager_;
    KeyHandler key_handler_;
    gsl::not_null<Ogre::SceneNode *> static_environment_;
    ColoredObject camera_visualization_;
    std::unordered_map<world::BoardId, ColoredObject> objects_{};
    std::unordered_set<world::BoardId> forced_visible_{};
};

} // namespace visualizer

#endif // VISUALIZER_H_
