/**
 * \file EnvireOdePhysicsPlugins.cpp
 * \author Malte Langosz
 *
 */

#include "EnvireOdePhysicsPlugins.hpp"

#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/SceneParseException.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <mars_interfaces/sim/JointInterface.h>

// TODO: extend interfaces to not require the ode_physics library on compile time
#include <mars_ode_physics/WorldPhysicsLoader.hpp>

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/MARSDefs.h>


namespace mars
{
    namespace envire_ode_physics
    {
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        EnvireOdePhysicsPlugins::EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface{theManager},
            envireGraph{ControlCenter::envireGraph},
            graphTreeView{ControlCenter::graphTreeView},
            dataBroker{ControlCenter::theDataBroker}
        {
            init();
        }

        EnvireOdePhysicsPlugins::EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager,
                                                         std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                                         std::shared_ptr<envire::core::TreeView> graphTreeView,
                                                         data_broker::DataBrokerInterface *dataBroker) :
            lib_manager::LibInterface{theManager},
            envireGraph{envireGraph},
            graphTreeView{graphTreeView},
            dataBroker{dataBroker}
        {
            init();
        }

        EnvireOdePhysicsPlugins::~EnvireOdePhysicsPlugins()
        {
        }

        void EnvireOdePhysicsPlugins::init(void)
        {
            //GraphEventDispatcher::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::Link>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::Inertial>>::subscribe(envireGraph.get());

            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Fixed>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Revolute>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Continuous>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Prismatic>>::subscribe(envireGraph.get());
        }

        /*

          TODO:
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
                const auto& vertex = envireGraph->vertex(frame);
                const auto& parentVertex = graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        const auto& it = envireGraph->getItem<SubControlItem>(frame);
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
            const auto typeName = std::string{"unknown"};
            LOG_DEBUG("\tclassName: %s", e.item->getClassName().c_str());
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Link>>& e)
        {
            auto control = getControlCenter(e.frame);

            // TODO: else search the tree upwards for a PhysicsInterface
            if(!control)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no control found!");
            }
            // TODO: why we need to hardcode the name of the lib???
            if(control->physics->getLibName() == "mars_ode_physics")
            {
                LOG_WARN("OdePhysicsPlugin: Added envire::types::Link item: %s", e.frame.c_str());
                LOG_DEBUG("\t %s", e.item->getData().name.c_str());
                auto config = e.item->getData().getFullConfigMap();
                // we are responsible
                auto newFrame = control->physics->createFrame(dataBroker, config);

                // TODO: set pose
                const auto t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
                newFrame->setPosition(t.transform.translation);
                newFrame->setRotation(t.transform.orientation);

                // store DynamicObject in graph
                DynamicObjectItem item;
                item.dynamicObject = newFrame;
                item.pluginName = control->physics->getLibName();
                envire::core::Item<DynamicObjectItem>::Ptr objectItemPtr{new envire::core::Item<DynamicObjectItem>{item}};
                envireGraph->addItemToFrame(e.frame, objectItemPtr);
            }
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Inertial>>& e)
        {
            auto control = getControlCenter(e.frame);
            if(!control)
            {
                LOG_ERROR("EnvireOdePhysicsPlugins::itemAdded: no control found!");
            }
            // TODO: why we need to hardcode the name of the lib???
            if(control->physics->getLibName() == "mars_ode_physics")
            {
                LOG_WARN("OdePhysicsPlugin: Added envire::types::Inertia item: %s", e.frame.c_str());
                // todo: check that we really have the frame in the map
                const auto& vertex = envireGraph->vertex(e.frame);
                const auto& parentVertex = graphTreeView->tree[vertex].parent;
                const auto& parentFrame = envireGraph->getFrameId(parentVertex);
                LOG_INFO("parent Frame: %s", parentFrame.c_str());

                auto config = e.item->getData().getFullConfigMap();
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
                const auto& t = envireGraph->getTransform(parentVertex, vertex);
                utils::vectorToConfigItem(&(config["position"]), &(t.transform.translation));
                utils::quaternionToConfigItem(&(config["orientation"]), &(t.transform.orientation));
                control->physics->createObject(config);
            }
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Fixed>>& e)
        {
            LOG_WARN("OdePhysicsPlugin: Added envire::types::joints::Fixed item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::types::joints::Fixed item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            auto& joint = e.item->getData();
            auto config = joint.getFullConfigMap();

            // find the parent and child links that are connected by the joint
            setLinksFixedJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Revolute>>& e)
        {

            LOG_WARN("OdePhysicsPlugin: Added envire::types::joints::Revolute item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::types::joints::Revolute item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            auto& joint = e.item->getData();
            auto config = joint.getFullConfigMap();
            // TODO: change the type in mars to revolute in urdf loader
            config["type"] = "hinge";

            // find the parent and child links that are connected by the joint
            setLinksDynamicJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Continuous>>& e)
        {
            LOG_WARN("OdePhysicsPlugin: Added envire::types::joints::Continuous item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::types::joints::Continuous item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            auto& joint = e.item->getData();
            auto config = joint.getFullConfigMap();
            // TODO: change the type in mars to revolute in urdf loader
            config["type"] = "hinge";

            // find the parent and child links that are connected by the joint
            setLinksDynamicJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Prismatic>>& e)
        {

            LOG_WARN("OdePhysicsPlugin: Added envire::types::joints::Prismatic item: %s", e.frame.c_str());
            LOG_DEBUG_S << "OdePhysicsPlugin: Added envire::types::joints::Prismatic item: " << e.frame;
            LOG_DEBUG_S << "joint name: " << e.item->getData().name;

            auto& joint = e.item->getData();
            auto config = joint.getFullConfigMap();
            // TODO: change the type in mars to revolute in urdf loader
            config["type"] = "prismatic";

            // find the parent and child links that are connected by the joint
            setLinksDynamicJoint(config, e.frame);
            // create joint in physic
            createPhysicJoint(config, e.frame);
        }

        void EnvireOdePhysicsPlugins::setLinksFixedJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
            // the parent link is stored in the parent frame
            // the child link is stored in the same frame as fixed joint
            const auto& vertex = envireGraph->getVertex(frameId);

            // get link from parent frame as parent link for a joint
            const auto& parentVertex = graphTreeView->getParent(vertex);
            const auto& parentFrameId = envireGraph->getFrameId(parentVertex);

            if (!containsOneLink(parentFrameId))
            {
                LOG_ERROR_S << "Can not create a new joint";
                return;
            }

            auto parentLinkItemItr = envireGraph->getItem<envire::core::Item<::envire::types::Link>>(parentFrameId);
            auto& parentLink = parentLinkItemItr->getData();

            // get link from the same frame as child link for a joint
            const auto& childFrameId = frameId;

            if (!containsOneLink(childFrameId))
            {
                LOG_ERROR_S << "Can not create a new joint";
                return;
            }
            auto childLinkItemItr = envireGraph->getItem<envire::core::Item<::envire::types::Link>>(childFrameId);
            const auto& childLink = childLinkItemItr->getData();

            // set connected links
            config["parent_link_name"] = parentLink.name;
            config["child_link_name"] = childLink.name;
        }

        void EnvireOdePhysicsPlugins::setLinksDynamicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
                // the parent link is stored in the parent frame
                // the child link is stored in the child frame as fixed joint
                const auto& vertex = envireGraph->getVertex(frameId);

                // get link from parent frame as parent link for a joint
                const auto& parentVertex = graphTreeView->getParent(vertex);
                const auto& parentFrameId = envireGraph->getFrameId(parentVertex);

                if (!containsOneLink(parentFrameId))
                {
                    LOG_ERROR_S << "Can not create a new joint";
                    return;
                }

                auto parentLinkItemItr = envireGraph->getItem<envire::core::Item<::envire::types::Link>>(parentFrameId);
                const auto& parentLink = parentLinkItemItr->getData();

                // get link from child frame as child link for a joint
                // there should be only one child frame, which is connected to joint frame
                const auto& children = graphTreeView->tree[vertex].children;
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
                const auto& childVertex = *(itr);
                const auto& childFrameId = envireGraph->getFrameId(childVertex);

                if (!containsOneLink(childFrameId))
                {
                    LOG_ERROR_S << "Can not create a new joint";
                    return;
                }
                auto childLinkItemItr = envireGraph->getItem<envire::core::Item<::envire::types::Link>>(childFrameId);
                const auto& childLink = childLinkItemItr->getData();

                // set connected links
                config["parent_link_name"] = parentLink.name;
                config["child_link_name"] = childLink.name;
        }

        void EnvireOdePhysicsPlugins::createPhysicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId)
        {
            auto control = getControlCenter(frameId);
            if(!control)
            {
                LOG_ERROR_S << "EnvireOdePhysicsPlugins::itemAdded: no control found!";
                return;
            }
            // TODO: why we need to hardcode the name of the lib???
            if(control->physics->getLibName() == "mars_ode_physics")
            {
                // set absolute position of joint
                const auto& trans = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
                const auto& anchor = trans.transform.translation;
                config["anchor"]["x"] = anchor.x();
                config["anchor"]["y"] = anchor.y();
                config["anchor"]["z"] = anchor.z();

                // set absolute orientation of axis
                Vector axis{config["axis"]["x"], config["axis"]["y"], config["axis"]["z"]};
                axis = trans.transform.orientation*axis;
                // todo: add assumption that joint axis is defined in childlink coordinates?
                config["axis1"]["x"] = axis.x();
                config["axis1"]["y"] = axis.y();
                config["axis1"]["z"] = axis.z();

                // reduce DataBroker load
                config["reducedDataPackage"] = true;

                const auto& jInterface = control->physics->createJoint(dataBroker, config);
                // store JointInterface in graph
                JointInterfaceItem item;
                item.jointInterface = jInterface;
                envire::core::Item<JointInterfaceItem>::Ptr jointItemPtr{new envire::core::Item<JointInterfaceItem>(item)};
                envireGraph->addItemToFrame(frameId, jointItemPtr);

                // id == 0 is invalid indicating getID that no specific id is desired
                const auto desiredId = static_cast<unsigned long>(config.get("desired_id", 0));
                // TODO: Enable requesting desired id for added joint
            }
        }

        bool EnvireOdePhysicsPlugins::containsOneLink(const envire::core::FrameId &frameId) const
        {
            // check if there is only one link item in the parent frame
            const size_t num_links = envireGraph->getItemCount<envire::core::Item<::envire::types::Link>>(frameId);
            if (num_links == 0)
            {
                LOG_ERROR_S << "The frame " << frameId << " does not contain a link item.";
                return false;
            } else if (num_links > 1)
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
