/**
 * \file EnvireOdePhysicsPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load physics representation based on envire items
 *
 */

#pragma once

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_types/Link.hpp>
#include <envire_types/Inertial.hpp>
#include <envire_types/joints/Fixed.hpp>
#include <envire_types/joints/Revolute.hpp>
#include <envire_types/joints/Continuous.hpp>
#include <envire_types/joints/Prismatic.hpp>

#include <lib_manager/LibInterface.hpp>
#include <data_broker/DataBrokerInterface.h>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/PhysicsInterface.h>
#include <mars_utils/Vector.h>

//TODO: add prismatic joint into base types and here


namespace mars
{
    namespace ode_physics
    {
        class WorldPhysicsLoader;
    }

    namespace envire_ode_physics
    {
        // move the typedef to separate file
        class EnvireOdePhysicsPlugins : public lib_manager::LibInterface,
                                        public envire::core::GraphEventDispatcher,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::Link>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::Inertial>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Fixed>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Revolute>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Continuous>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Prismatic>>

        {

        public:
            EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager,
                                    std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                    std::shared_ptr<envire::core::TreeView> graphTreeView,
                                    data_broker::DataBrokerInterface *dataBroker=NULL);
            virtual ~EnvireOdePhysicsPlugins();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string{"envire_mars_ode_physics"};
            }

            CREATE_MODULE_INFO();

            void init(void);

            // envire callbacks
            virtual void itemAdded(const envire::core::ItemAddedEvent& e) override;

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Link>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Inertial>>& e) override;

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Fixed>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Revolute>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Continuous>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Prismatic>>& e) override;

        private:
            std::shared_ptr<envire::core::EnvireGraph> envireGraph;
            std::shared_ptr<envire::core::TreeView> graphTreeView;
            data_broker::DataBrokerInterface *dataBroker;

            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            void setLinksFixedJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
            void setLinksDynamicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
            void createPhysicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
            bool containsOneLink(const envire::core::FrameId &frameId) const;
        };

    } // end of namespace envire_ode_physics
} // end of namespace mars
