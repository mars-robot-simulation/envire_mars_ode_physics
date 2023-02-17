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

namespace mars
{
    namespace ode_physics
    {
        class WorldPhysicsLoader;
    }

    namespace envire_ode_physics
    {

        class EnvireOdePhysicsPlugins : public lib_manager::LibInterface,
                                        public envire::core::GraphEventDispatcher,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Frame>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Inertial>>,
                                        public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>
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
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Frame>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Inertial>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e) override;

        private:
            std::shared_ptr<interfaces::SubControlCenter> getControlCenter(envire::core::FrameId frame);
        };

    } // end of namespace envire_ode_physics
} // end of namespace mars
