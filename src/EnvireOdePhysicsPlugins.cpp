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
#include "EnvireOdePhysicsPlugins.hpp"

//#include "PhysicsMapper.h"

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/SceneParseException.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <mars_interfaces/sim/JointInterface.h>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

// todo: extend interfaces to not require the ode_physics library on compile time
#include <mars_ode_physics/WorldPhysicsLoader.hpp>

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/MARSDefs.h>

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
            envireGraph = ControlCenter::envireGraph;
            graphTreeView = ControlCenter::graphTreeView;
            dataBroker = ControlCenter::theDataBroker;
            init();
        }

        EnvireOdePhysicsPlugins::EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager,
                                                         std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                                         std::shared_ptr<envire::core::TreeView> graphTreeView,
                                                         data_broker::DataBrokerInterface *dataBroker) :
            lib_manager::LibInterface(theManager), envireGraph(envireGraph), graphTreeView(graphTreeView), dataBroker(dataBroker)
        {
            init();
        }

        EnvireOdePhysicsPlugins::~EnvireOdePhysicsPlugins()
        {
        }

        void EnvireOdePhysicsPlugins::init(void)
        {
            //GraphEventDispatcher::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Link>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Inertial>>::subscribe(envireGraph.get());

            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Fixed>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Revolute>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Continuous>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Prismatic>>::subscribe(envireGraph.get());
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

        // TODO: this should be moved out from here
        std::shared_ptr<SubControlCenter> EnvireOdePhysicsPlugins::getControlCenter(envire::core::FrameId frame)
        {
            // search for physics interface in graph
            bool done = false;
            while(!done)
            {
                const envire::core::GraphTraits::vertex_descriptor vertex = envireGraph->vertex(frame);
                envire::core::GraphTraits::vertex_descriptor parentVertex = graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        envire::core::EnvireGraph::ItemIterator<SubControlItem> it = envireGraph->getItem<SubControlItem>(frame);
                        return it->getData();
                    }
                    catch (...)
                    {
                    }
                }
                else
                {
                    done = true;
                }
            }
            return nullptr;
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::ItemAddedEvent& e)
        {
            LOG_DEBUG("Added generic item: %s", e.frame.c_str());
            std::string typeName = "unknown";
            LOG_DEBUG("\tclassName: %s", e.item->getClassName().c_str());
        }

        // std::string EnvireOdePhysicsPlugins::getRootFrame()
        // {
        //     return std::string(SIM_CENTER_FRAME_NAME);
        // }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Link>>& e)
        {
            std::shared_ptr<SubControlCenter> control = getControlCenter(e.frame);

            // todo: else search the tree upwards for a PhysicsInterface
            if(!control)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no control found!");
            }
            // TODO: why we need to hardcode the name of the lib???
            if(control->physics->getLibName() == "mars_ode_physics")
            {
                LOG_WARN("OdePhysicsPlugin: Added envire::base_types::Link item: %s", e.frame.c_str());
                LOG_DEBUG("\t %s", e.item->getData().name.c_str());
                ConfigMap config = e.item->getData().getFullConfigMap();
                // we are responsible
                std::shared_ptr<DynamicObject> newFrame = control->physics->createFrame(dataBroker, config);

                // todo: set pose
                envire::core::Transform t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
                newFrame->setPosition(t.transform.translation);
                newFrame->setRotation(t.transform.orientation);

                // store DynamicObject in graph
                DynamicObjectItem item;
                item.dynamicObject = newFrame;
                item.pluginName = "mars_ode_physics";   // TODO: use the lib name from control->physics->getLibName()
                envire::core::Item<DynamicObjectItem>::Ptr objectItemPtr(new envire::core::Item<DynamicObjectItem>(item));
                //envire::core::Item<DynamicObject*>::Ptr objectItemPtr(new envire::core::Item<DynamicObject*>(newFrame));
                envireGraph->addItemToFrame(e.frame, objectItemPtr);
                ControlCenter::linkIDManager->addIfUnknown(config["name"]);
            }
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Inertial>>& e)
        {
            std::shared_ptr<SubControlCenter> control = getControlCenter(e.frame);
            if(!control)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no control found!");
            }
            // TODO: why we need to hardcode the name of the lib???
            if(control->physics->getLibName() == "mars_ode_physics")
            {
                LOG_WARN("OdePhysicsPlugin: Added envire::base_types::Inertia item: %s", e.frame.c_str());
                // todo: check that we really have the frame in the map
                const envire::core::GraphTraits::vertex_descriptor vertex = envireGraph->vertex(e.frame);
                envire::core::GraphTraits::vertex_descriptor parentVertex = graphTreeView->tree[vertex].parent;
                envire::core::FrameId parentFrame = envireGraph->getFrameId(parentVertex);
                LOG_INFO("parent Frame: %s", parentFrame.c_str());

                ConfigMap config = e.item->getData().getFullConfigMap();
                config["parentFrame"] = parentFrame;
                // config["name"]
                // config["mass"]
                // ------------------------------
                // TODO: this is a conversion to old mars config
                // should be replaced in physics
                config["inertia"]["i00"] = config["xx"];
                config["inertia"]["i01"] = config["xy"];
                config["inertia"]["i02"] = config["xz"];
                config["inertia"]["i10"] = config["xy"];
                config["inertia"]["i11"] = config["yy"];
                config["inertia"]["i12"] = config["yz"];
                config["inertia"]["i20"] = config["xz"];
                config["inertia"]["i21"] = config["yz"];
                config["inertia"]["i22"] = config["zz"];
                // --------------------------------
                // todo: check hirarchy issues with closed loops
                envire::core::Transform t = envireGraph->getTransform(parentVertex, vertex);
                vectorToConfigItem(&(config["position"]), &(t.transform.translation));
                quaternionToConfigItem(&(config["orientation"]), &(t.transform.orientation));
                control->physics->createObject(config);
            }
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Fixed>>& e)
        {
            LOG_WARN("OdePhysicsPlugin: Added envire::base_types::joints::Fixed item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::base_types::joints::Fixed item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            envire::base_types::joints::Fixed &joint = e.item->getData();
            ConfigMap config = joint.getFullConfigMap();

            // find the parent and child links that are connected by the joint
            setLinksFixedJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Revolute>>& e)
        {

            LOG_WARN("OdePhysicsPlugin: Added envire::base_types::joints::Revolute item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::base_types::joints::Revolute item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            envire::base_types::joints::Revolute &joint = e.item->getData();
            ConfigMap config = joint.getFullConfigMap();
            // TODO: change the type in mars to revolute in urdf loader
            config["type"] = "hinge";

            // find the parent and child links that are connected by the joint
            setLinksDynamicJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Continuous>>& e)
        {
            LOG_WARN("OdePhysicsPlugin: Added envire::base_types::joints::Continuous item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::base_types::joints::Continuous item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            envire::base_types::joints::Continuous &joint = e.item->getData();
            ConfigMap config = joint.getFullConfigMap();
            // TODO: change the type in mars to revolute in urdf loader
            config["type"] = "hinge";

            // find the parent and child links that are connected by the joint
            setLinksDynamicJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Prismatic>>& e)
        {

            LOG_WARN("OdePhysicsPlugin: Added envire::base_types::joints::Prismatic item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::base_types::joints::Prismatic item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            envire::base_types::joints::Prismatic &joint = e.item->getData();
            ConfigMap config = joint.getFullConfigMap();
            // TODO: change the type in mars to revolute in urdf loader
            config["type"] = "prismatic";

            // find the parent and child links that are connected by the joint
            setLinksDynamicJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::setLinksFixedJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
            using LinkItem = envire::core::Item<::envire::base_types::Link>;
            using LinkItemItr = envire::core::EnvireGraph::ItemIterator<LinkItem>;
            using VertexDesc = envire::core::GraphTraits::vertex_descriptor;

            // the parent link is stored in the parent frame
            // the child link is stored in the same frame as fixed joint
            VertexDesc vertex = envireGraph->getVertex(frameId);

            // get link from parent frame as parent link for a joint
            VertexDesc parentVertex = graphTreeView->getParent(vertex);
            envire::core::FrameId parentFrameId = envireGraph->getFrameId(parentVertex);

            if (containsOneLink(parentFrameId) == false)
            {
                LOG_ERROR_S << "Can not create a new joint";
                return;
            }

            LinkItemItr parentLinkItemItr = envireGraph->getItem<LinkItem>(parentFrameId);
            envire::base_types::Link &parentLink = parentLinkItemItr->getData();

            // get link from the same frame as child link for a joint
            envire::core::FrameId childFrameId = frameId;

            if (containsOneLink(childFrameId) == false)
            {
                LOG_ERROR_S << "Can not create a new joint";
                return;
            }
            LinkItemItr childLinkItemItr = envireGraph->getItem<LinkItem>(childFrameId);
            envire::base_types::Link &childLink = childLinkItemItr->getData();

            // set connected links
            config["parent_link_name"] = parentLink.name;
            config["child_link_name"] = childLink.name;
        }

        void EnvireOdePhysicsPlugins::setLinksDynamicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
                using LinkItem = envire::core::Item<::envire::base_types::Link>;
                using LinkItemItr = envire::core::EnvireGraph::ItemIterator<LinkItem>;
                using VertexDesc = envire::core::GraphTraits::vertex_descriptor;

                // the parent link is stored in the parent frame
                // the child link is stored in the child frame as fixed joint
                VertexDesc vertex = envireGraph->getVertex(frameId);

                // get link from parent frame as parent link for a joint
                VertexDesc parentVertex = graphTreeView->getParent(vertex);
                envire::core::FrameId parentFrameId = envireGraph->getFrameId(parentVertex);

                if (containsOneLink(parentFrameId) == false)
                {
                    LOG_ERROR_S << "Can not create a new joint";
                    return;
                }

                LinkItemItr parentLinkItemItr = envireGraph->getItem<LinkItem>(parentFrameId);
                envire::base_types::Link &parentLink = parentLinkItemItr->getData();

                // get link from child frame as child link for a joint
                // there should be only one child frame, which is connected to joint frame
                const std::unordered_set<VertexDesc>& children = graphTreeView->tree[vertex].children;
                if (children.size() == 0)
                {
                    LOG_ERROR_S << "Can not create a new joint, since the frame " << frameId << " contains no child frame.";
                    return;
                } else if (children.size() > 1)
                {
                    LOG_ERROR_S << "Can not create a new joint, since the frame " << frameId << " contains several child frames.";
                    return;
                }

                auto itr = children.begin();
                VertexDesc childVertex = *(itr);
                envire::core::FrameId childFrameId = envireGraph->getFrameId(childVertex);

                if (containsOneLink(childFrameId) == false)
                {
                    LOG_ERROR_S << "Can not create a new joint";
                    return;
                }
                LinkItemItr childLinkItemItr = envireGraph->getItem<LinkItem>(childFrameId);
                envire::base_types::Link &childLink = childLinkItemItr->getData();

                // set connected links
                config["parent_link_name"] = parentLink.name;
                config["child_link_name"] = childLink.name;
        }

        void EnvireOdePhysicsPlugins::createPhysicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
            std::shared_ptr<SubControlCenter> control = getControlCenter(frameId);
            if(!control)
            {
                LOG_ERROR_S << "EnvireOdePhysicsPlugins::itemAdded: no control found!";
                return;
            }
            // TODO: why we need to hardcode the name of the lib???
            if(control->physics->getLibName() == "mars_ode_physics")
            {
                // set absolute position of joint
                envire::core::Transform trans = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
                Vector anchor = trans.transform.translation;
                config["anchor"]["x"] = anchor.x();
                config["anchor"]["y"] = anchor.y();
                config["anchor"]["z"] = anchor.z();

                // set absolute orientation of axis
                Vector axis(config["axis"]["x"], config["axis"]["y"], config["axis"]["z"]);
                axis = trans.transform.orientation*axis;
                // todo: add assumption that joint axis is defined in childlink coordinates?
                config["axis1"]["x"] = axis.x();
                config["axis1"]["y"] = axis.y();
                config["axis1"]["z"] = axis.z();

                // reduce DataBroker load
                config["reducedDataPackage"] = true;

                std::shared_ptr<JointInterface> jInterface = control->physics->createJoint(dataBroker, config);
                // store JointInterface in graph
                JointInterfaceItem item;
                item.jointInterface = jInterface;
                envire::core::Item<JointInterfaceItem>::Ptr jointItemPtr(new envire::core::Item<JointInterfaceItem>(item));
                envireGraph->addItemToFrame(frameId, jointItemPtr);

                // id == 0 is invalid indicating getID that no specific id is desired
                const unsigned long desiredId = config.get("desired_id", 0);
                ControlCenter::jointIDManager->addIfUnknown(config["name"], desiredId);
            }
        }

        bool EnvireOdePhysicsPlugins::containsOneLink(const envire::core::FrameId &frameId)
        {
            // check if there is only one link item in the parent frame
            size_t linkNumb = envireGraph->getItemCount<envire::core::Item<::envire::base_types::Link>>(frameId);
            if (linkNumb == 0)
            {
                LOG_ERROR_S << "The frame " << frameId << " does not contain a link item.";
                return false;
            } else if (linkNumb > 1)
            {
                LOG_ERROR_S << "There are multiple link items in the frame " << frameId << ".";
                return false;
            }

            return true;
        }
    } // end of namespace envire_ode_physics

} // end of namespace mars

DESTROY_LIB(mars::envire_ode_physics::EnvireOdePhysicsPlugins);
CREATE_LIB(mars::envire_ode_physics::EnvireOdePhysicsPlugins);
