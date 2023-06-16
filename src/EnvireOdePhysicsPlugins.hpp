/**
 * \file EnvireOdePhysicsPlugins.hpp
 * \author Malte Langosz
 * \brief Plugin class to load physics representation based on envire items
 *
 */

#pragma once
#include <mars_interfaces/sim/PhysicsInterface.h>
#include <mars/utils/Vector.h>


#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <mars_ode_collision/objects/Object.hpp>

#include <iostream>

#include <envire_base_types/Link.hpp>
#include <envire_base_types/Inertial.hpp>
#include <envire_base_types/joints/Fixed.hpp>
#include <envire_base_types/joints/Revolute.hpp>
#include <envire_base_types/joints/Continuous.hpp>
#include <envire_base_types/joints/Prismatic.hpp>

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
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Link>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Inertial>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Fixed>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Revolute>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Continuous>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Prismatic>>

        {

        public:
            EnvireOdePhysicsPlugins(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~EnvireOdePhysicsPlugins();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string("envire_mars_ode_physics");
            }

            CREATE_MODULE_INFO();

            // envire callbacks
            virtual void itemAdded(const envire::core::ItemAddedEvent& e) override;

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Link>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Inertial>>& e) override;

            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Fixed>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Revolute>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Continuous>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Prismatic>>& e) override;

        private:
            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);

            void setLinksFixedJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
            void setLinksDynamicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
            void createPhysicJoint(configmaps::ConfigMap &config, const envire::core::FrameId &frameId);
            bool containsOneLink(const envire::core::FrameId &frameId);
        };

    } // end of namespace envire_ode_physics
} // end of namespace mars
