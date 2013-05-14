#include "Actuator.hpp"
#include "OrocosStateAggregator.hpp"


namespace interfaces{

Actuator::Actuator()
{
}


Actuator::~Actuator()
{
}

bool Actuator::dispatch(::std::string const & name, ::std::vector< boost::int32_t > const & actuatorMap, RTT::TaskContext *task)
{
    OrocosCommandDispatcher *disp;
    try {
	disp = new OrocosCommandDispatcher(*task, boost::bind(&Actuator::setCommand, this, _1 , _2, _3), name, actuatorMap);
    } catch (std::runtime_error e)
    {
	std::cout << e.what() << std::endl;
	return false;
    }
    cmdDispatches.push_back(disp);

    OrocsStateAggregator *agg;
    try {
	agg = new OrocsStateAggregator(*task, name, actuatorMap, statusInterval);
    } catch (std::runtime_error e)
    {
	std::cout << e.what() << std::endl;
	delete disp;
	return false;
    } 
    
    int dispId = stateDispatches.size();
    stateDispatches.push_back(agg);
    
    //setup mapping from actuator ids to dispatch
    const std::vector<int32_t> ids = agg->getActuatorIds();
    for(std::vector<int32_t>::const_iterator it = ids.begin(); it != ids.end();it++)
    {
	if(actuatorToDispatchMap.size() <= (uint32_t) *it)
	    actuatorToDispatchMap.resize(*it + 1);
	
	actuatorToDispatchMap[*it] = dispId;
    }

    //inform the driver implementation about the
    //added dispatch
    statusDispatchAdded(dispId, ids);
    
    return true;
}

void Actuator::processDispatched()
{
    for(std::vector<OrocosCommandDispatcher *>::iterator it = cmdDispatches.begin(); it != cmdDispatches.end(); it++)
    {
	(*it)->process();
    }
}

void Actuator::setNewActuatorStatus(int actuatorId, const base::Time stateTime, const base::actuators::MotorState& state)
{
    const int dispId = actuatorToDispatchMap[actuatorId];
    stateDispatches[dispId]->setNewStatus(actuatorId, stateTime, state);
}


void Actuator::cleanup()
{
    for(unsigned int i = 0; i < cmdDispatches.size(); i++)
    {
	delete cmdDispatches[i];
    }
    cmdDispatches.clear();

    for(unsigned int i = 0; i < stateDispatches.size(); i++)
    {
	delete stateDispatches[i];
    }
    stateDispatches.clear();    
}
}
