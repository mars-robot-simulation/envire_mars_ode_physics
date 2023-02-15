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
            //GraphEventDispatcher::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Frame>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Inertial>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>::subscribe(ControlCenter::envireGraph.get());
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
            std::shared_ptr<PhysicsInterface> physicsInterface = nullptr;
            if(ControlCenter::activePhysicsInterface)
            {
                physicsInterface = ControlCenter::activePhysicsInterface;
            }
            // todo: else search the tree upwards for a PhysicsInterface
            if(!physicsInterface)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no PhysicsInterface found!");
            }
            if(physicsInterface->getLibName() == "mars_ode_physics")
            {
                LOG_WARN("OdePhysicsPlugin: Added smurf::Frame item: %s", e.frame.c_str());
                LOG_DEBUG("\t %s", e.item->getData().getName().c_str());
                //e.item->getData() // provides the smurf joint class shared pointer
                ConfigMap config;
                config["name"] = e.frame;
                // we are responsible
                DynamicObject *newFrame = physicsInterface->createFrame(ControlCenter::theDataBroker, config);

                // todo: set pose
                envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
                newFrame->setPosition(t.transform.translation);
                newFrame->setRotation(t.transform.orientation);
            }
        }


        // std::string EnvireOdePhysicsPlugins::getRootFrame()
        // {
        //     return std::string(SIM_CENTER_FRAME_NAME);
        // }


        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Inertial>>& e)
        {
            std::shared_ptr<PhysicsInterface> physicsInterface = nullptr;
            if(ControlCenter::activePhysicsInterface)
            {
                physicsInterface = ControlCenter::activePhysicsInterface;
            }
            // todo: else search the tree upwards for a PhysicsInterface
            if(!physicsInterface)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no PhysicsInterface found!");
            }
            if(physicsInterface->getLibName() == "mars_ode_physics")
            {
                LOG_INFO("OdePhysicsPlugin: Added smurf::Inertial item: %s", e.frame.c_str());
                // todo: check that we really have the frame in the map
                const envire::core::GraphTraits::vertex_descriptor vertex = ControlCenter::envireGraph->vertex(e.frame);
                envire::core::GraphTraits::vertex_descriptor parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
                envire::core::FrameId parentFrame = ControlCenter::envireGraph->getFrameId(parentVertex);
                LOG_INFO("parent Frame: %s", parentFrame.c_str());
                //std::string type = e.item->getData().getTypeString();
                std::string type = "inertial";
                LOG_DEBUG("\t\ttype: %s", type.c_str());

                ConfigMap config;
                config["name"] = e.item->getData().getName();
                config["parentFrame"] = parentFrame;
                config["inertia"]["i00"] = e.item->getData().getUrdfInertial().ixx;
                config["inertia"]["i01"] = e.item->getData().getUrdfInertial().ixy;
                config["inertia"]["i02"] = e.item->getData().getUrdfInertial().ixz;
                config["inertia"]["i10"] = e.item->getData().getUrdfInertial().ixy;
                config["inertia"]["i11"] = e.item->getData().getUrdfInertial().iyy;
                config["inertia"]["i12"] = e.item->getData().getUrdfInertial().iyz;
                config["inertia"]["i20"] = e.item->getData().getUrdfInertial().ixz;
                config["inertia"]["i21"] = e.item->getData().getUrdfInertial().iyz;
                config["inertia"]["i22"] = e.item->getData().getUrdfInertial().izz;
                config["mass"] = e.item->getData().getUrdfInertial().mass;
                config["type"] = type;
                // todo: check hirarchy issues with closed loops
                envire::core::Transform t = ControlCenter::envireGraph->getTransform(parentVertex, vertex);
                vectorToConfigItem(&(config["position"]), &(t.transform.translation));
                quaternionToConfigItem(&(config["orientation"]), &(t.transform.orientation));
                physicsInterface->createObject(config);
            }
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e)
        {
            std::shared_ptr<PhysicsInterface> physicsInterface = nullptr;
            if(ControlCenter::activePhysicsInterface)
            {
                physicsInterface = ControlCenter::activePhysicsInterface;
            }
            // todo: else search the tree upwards for a PhysicsInterface
            if(!physicsInterface)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no PhysicsInterface found!");
            }
            if(physicsInterface->getLibName() == "mars_ode_physics")
            {
                LOG_DEBUG("OdePhysicsPlugin: Added smurf::joint item: %s", e.frame.c_str());
                LOG_DEBUG("\t %s", e.item->getData().getName().c_str());
                urdf::JointSharedPtr joint = e.item->getData().getJointModel();

                ConfigMap config;
                // todo: prefix have to move to joint strings directly
                config["name"] = ControlCenter::prefix + joint->name;
                config["parent_link_name"] = ControlCenter::prefix + joint->parent_link_name;
                config["child_link_name"] = ControlCenter::prefix + joint->child_link_name;
                // deprecated: config["anchorpos"] = "node2"; // always use the child_link as the anchor since joint and child_link are in the same frame
                envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
                Vector p = t.transform.translation;
                const Eigen::Affine3d &transform = e.item->getData().getParentToJointOrigin();
                p += t.transform.orientation*transform.translation();
                Vector a(joint->axis.x, joint->axis.y, joint->axis.z);
                a = t.transform.orientation*transform.rotation()*a;
                config["anchor"]["x"] = p.x();
                config["anchor"]["y"] = p.y();
                config["anchor"]["z"] = p.z();

                // FIXME: we do not at this point read the joint "maxeffort" and "maxvelocity"
                // limits as they are effectively motor values and should be used only
                // if there are no explicit motor values defined
                if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS)
                {
                    config["type"] = "hinge";
                } else if (joint->type == urdf::Joint::PRISMATIC)
                {
                    config["type"] = "slider";
                } else if (joint->type == urdf::Joint::FIXED)
                {
                    config["type"] = "fixed";
                    LOG_ERROR("Add fixed joint");
                } else {
                    // we don't support the type yet and use a fixed joint
                    config["type"] = "fixed";
                    LOG_ERROR("Add fixed joint");
                }

//         // transform the joint's axis into global coordinates
//         urdf::Pose pose = getGlobalPose(childlink);
//         urdf::Pose axispose;
// //        axispose.position = childlink->parent_joint->parent_to_joint_origin_transform.rotation * joint->axis;
//         axispose.position = pose.rotation * joint->axis;
//         Vector v;
//         v = Vector(axispose.position.x, axispose.position.y, axispose.position.z);
                // todo: add assumption that joint axis is defined in childlink coordinates?
                config["axis1"]["x"] = a.x();
                config["axis1"]["y"] = a.y();
                config["axis1"]["z"] = a.z();

                // reduce DataBroker load
                config["reducedDataPackage"] = true;
                JointInterface *jInterface = physicsInterface->createJoint(ControlCenter::theDataBroker, config);
            }
        }

    } // end of namespace envire_ode_physics

} // end of namespace mars

DESTROY_LIB(mars::envire_ode_physics::EnvireOdePhysicsPlugins);
CREATE_LIB(mars::envire_ode_physics::EnvireOdePhysicsPlugins);