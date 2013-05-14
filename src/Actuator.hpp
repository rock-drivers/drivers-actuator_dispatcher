/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef _ACTUATOR_HPP
#define _ACTUATOR_HPP

#include "OrocosStateAggregator.hpp"

#ifndef RTT_COMPONENT
#define VIRTUAL = 0
#else
#define VIRTUAL { throw("COULD NOT USE AN ABSTRACT CLASS"); }
#endif

namespace interfaces {

    /*! \class Actuator 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','interfaces::Actuator')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Actuator 
    {
    
    protected:
	std::vector<OrocosCommandDispatcher *> cmdDispatches;
	std::vector<StateAggregator *> stateDispatches;
	std::vector<int> actuatorToDispatchMap;

	///The nominal time between two status updates of an actuator
	base::Time statusInterval;
	
	virtual void statusDispatchAdded(int dispatchId, std::vector<int> actuatorIds) VIRTUAL;
	void setNewActuatorStatus(int actuatorId, const base::Time stateTime, const base::actuators::MotorState &state);

	virtual void setCommand(int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value) VIRTUAL;
	
	/**
	 * Pull all data from the dispatch ports
	 * */
	void processDispatched();

	/* Handler for the dispatch operation
         */
        virtual bool dispatch(::std::string const & name, ::std::vector< boost::int32_t > const & actuatorMap, RTT::TaskContext *task);

    public:
        Actuator();
        
        /** Default deconstructor of Actuator
         */
	~Actuator();

        void cleanup();
    };
}

#endif

