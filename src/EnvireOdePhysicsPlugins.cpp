/**
 * \file EnvireOdePhysicsPlugins.cpp
 * \author Malte Langosz
 *
 */

/* Conventions:
 *   - the includes should be defined in the header file
 *   - atomic variables shoud use snake_case
 *   - instances of classes should use camel_case
 *   - method names are camel_case
 *   - always use braces
 *   - braces start in new line
 *   - indent with four tabs
 */

#include <smurf/Robot.hpp>

#include "EnvireOdePhysicsPlugins.hpp"

//#include "PhysicsMapper.h"

#include <mars/utils/mathUtils.h>
#include <mars/utils/misc.h>
#include <mars_interfaces/SceneParseException.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

// todo: extend interfaces to not require the ode_physics library on compile time
#include <mars_ode_physics/WorldPhysicsLoader.hpp>

#include <mars_interfaces/Logging.hpp>

#define SIM_CENTER_FRAME_NAME "world"
typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

namespace mars
{
    namespace envire_ode_physics
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        EnvireOdePhysicsPlugins::EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface(theManager)
        {
            physicsLoader = libManager->getLibraryAs<ode_physics::WorldPhysicsLoader>("mars_ode_physics", true);
            GraphEventDispatcher::subscribe(ControlCenter::envireGraph.get());
        }

        EnvireOdePhysicsPlugins::~EnvireOdePhysicsPlugins()
        {
            // TODO: do we need to delete control?
            if(physicsLoader)
            {
                libManager->releaseLibrary("mars_ode_physics");
            }
        }

        /*

          todo:
          Howto manage ode objects and sync them with envire representation:
          1. handle/manage map by ids (frameId, etc.)
          2. store ode objects in envire graph directly
          3. clone graph for physics representation

          General representation of data in envire graph?
          1. store visual, collision, inertial items seperately
          2. store geometry items that have annotations like physical properties, visual properties, etc.
          - Only store information vs. store objects with functionallity

         */

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::ItemAddedEvent& e)
        {
            LOG_DEBUG("Added generic item: %s", e.frame.c_str());
            std::string typeName = "unknown";
            LOG_DEBUG("\tclassName: %s", e.item->getClassName().c_str());
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Frame>>& e)
        {
            LOG_WARN("Added smurf::Frame item: %s", e.frame.c_str());
            LOG_DEBUG("\t %s", e.item->getData().getName().c_str());
            //e.item->getData() // provides the smurf joint class shared pointer
            std::string frame = e.frame;
            ConfigMap config;
            config["name"] = e.frame;
            // move the graph upwards and search for a physics world
            
            // DynamicObject *newFrame = callbackWorld->control->physics->createFrame(control->dataBroker, config);

            // frameMap[frame] = newFrame;
            // // todo: set pose
            // envire::core::Transform t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frame);
            // newFrame->setPosition(t.transform.translation);
            // newFrame->setRotation(t.transform.orientation);
            // if(control->graphics)
            // {
            //     // crate empty object to be able to use the frame debug option of mars_graphics
            //     ConfigMap config;
            //     config["origname"] = "empty";
            //     config["filename"] = "PRIMITIVE";
            //     config["name"] = e.frame;
            //     config["type"] = "empty";
            //     config["createFrame"] = true;
            //     interfaces::NodeData nodeData;
            //     nodeData.fromConfigMap(&config, "");
            //     unsigned long drawID = control->graphics->addDrawObject(nodeData, 0);
            //     control->graphics->setDrawObjectPos(drawID, t.transform.translation);
            //     control->graphics->setDrawObjectRot(drawID, t.transform.orientation);
            //     visualFrameMap[drawID] = e.frame;
            // }
        }


        // std::string EnvireOdePhysicsPlugins::getRootFrame()
        // {
        //     return std::string(SIM_CENTER_FRAME_NAME);
        // }


        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Inertial>>& e)
        {
            // LOG_INFO("Added smurf::Inertial item: %s", e.frame.c_str());
            // // todo: check that we really have the frame in the map
            // const envire::core::GraphTraits::vertex_descriptor vertex = envireGraph->vertex(e.frame);
            // envire::core::GraphTraits::vertex_descriptor parentVertex = graphTreeView->tree[vertex].parent;
            // envire::core::FrameId parentFrame = envireGraph->getFrameId(parentVertex);
            // LOG_INFO("parent Frame: %s", parentFrame.c_str());
            // //std::string type = e.item->getData().getTypeString();
            // std::string type = "inertial";
            // LOG_DEBUG("\t\ttype: %s", type.c_str());

            // ConfigMap config;
            // config["name"] = e.item->getData().getName();
            // config["parentFrame"] = parentFrame;
            // config["inertia"]["i00"] = e.item->getData().getUrdfInertial().ixx;
            // config["inertia"]["i01"] = e.item->getData().getUrdfInertial().ixy;
            // config["inertia"]["i02"] = e.item->getData().getUrdfInertial().ixz;
            // config["inertia"]["i10"] = e.item->getData().getUrdfInertial().ixy;
            // config["inertia"]["i11"] = e.item->getData().getUrdfInertial().iyy;
            // config["inertia"]["i12"] = e.item->getData().getUrdfInertial().iyz;
            // config["inertia"]["i20"] = e.item->getData().getUrdfInertial().ixz;
            // config["inertia"]["i21"] = e.item->getData().getUrdfInertial().iyz;
            // config["inertia"]["i22"] = e.item->getData().getUrdfInertial().izz;
            // config["mass"] = e.item->getData().getUrdfInertial().mass;
            // config["type"] = type;
            // // todo: check hirarchy issues with closed loops
            // envire::core::Transform t = envireGraph->getTransform(parentVertex, vertex);
            // vectorToConfigItem(&(config["position"]), &(t.transform.translation));
            // quaternionToConfigItem(&(config["orientation"]), &(t.transform.orientation));
            // callbackWorld->control->physics->createObject(config);
        }

    } // end of namespace envire_ode_physics

} // end of namespace mars

DESTROY_LIB(mars::envire_ode_physics::EnvireOdePhysicsPlugins);
CREATE_LIB(mars::envire_ode_physics::EnvireOdePhysicsPlugins);
