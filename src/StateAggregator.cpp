#include "StateAggregator.hpp"
#include <iostream>
#include <math.h>

StateAggregator::StateAggregator(const std::vector<int32_t>& statusMap, base::Time statusInterval) : actuatorCnt(0), updateCnt(0), statusInterval(statusInterval)
{
    status.resize(statusMap.size());

    for (unsigned int i = 0; i < statusMap.size(); ++i)
    {
        int board_idx = abs(statusMap[i]) - 1;
	//zero has the special meaning
	//of 'do not use this id'
	if(statusMap[i] == 0)
	{
	    status.states[i].setInvalid();
	    continue;
	}
	
	actuatorCnt++;
	StateInfo info;
	info.stateId  = board_idx;
	info.outputPosition = i;
	info.isReverse = statusMap[i] < 0;
	
	stateMap.insert(std::make_pair(board_idx, info));
	
	if(stateIdToInfo.size() <= board_idx)
	{
	    stateIdToInfo.resize(board_idx + 1, NULL);
	}
	stateIdToInfo[board_idx] = &(stateMap[board_idx]);
    }

    reset();
}

const std::vector<int32_t> StateAggregator::getActuatorIds() const
{
    std::vector<int32_t> ret;
    for(std::map<int, StateInfo>::const_iterator it = stateMap.begin(); it != stateMap.end(); it++)
    {
	ret.push_back(it->second.stateId);
    }
    return ret;
}


void StateAggregator::reset()
{
    //reinit the update counter
    updateCnt = actuatorCnt;
    
    //reset the updated flag
    for(std::map<int, StateInfo>::iterator it = stateMap.begin(); it != stateMap.end(); it++)
    {
	it->second.updated = false;
    }
}


void StateAggregator::setNewStatus(int stateId, const base::Time stateTime, const base::actuators::MotorState& state)
{
    
    StateInfo &info(*(stateIdToInfo[stateId]));
    
    if(status.time > stateTime)
    {
	std::cout << "Warning, got state update for actuator from the past " << status.time << " " << stateTime << std::endl;
    }
    
    //if the time of the samples differ by more than an interval 
    //than the stateSet is missaligned in time.
    //if info.updated is set we lost a sample
    if((stateTime - status.time > statusInterval) || info.updated)
    {
	writeStatus(status);
	reset();
    }

    info.updated = true;
    info.updateTime = stateTime;
    status.states[stateId] = state;
    if(info.isReverse)
    {
	status.states[stateId].invert();
    }
    
    //allways use the latest time
    status.time = stateTime;

    //nominal case
    updateCnt--;
    if(updateCnt <= 0)
    {
	writeStatus(status);
	reset();
    }
}

CommandDispatcher::CommandDispatcher(const std::vector< int32_t >& actuatorMap, boost::function<void (int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value)> setCommandCallback): setCommandCallback(setCommandCallback)
{
    for(uint32_t i = 0; i < actuatorMap.size();i++)
    {
	int board_idx = abs(actuatorMap[i]) - 1;
	//zero has the special meaning
	//of 'do not use this id'
	if(actuatorMap[i] == 0)
	{
	    continue;
	}
	InputMapping m;
	m.actuatorId = board_idx;
	m.inputId = i;
	m.isReverse = actuatorMap[i] < 0;
	
	inputMap.push_back(m);
    }	
}

const std::vector<int32_t> CommandDispatcher::getActuatorIds() const
{
    std::vector<int32_t> ret;
    for(std::vector<InputMapping>::const_iterator it = inputMap.begin(); it != inputMap.end(); it++)
    {
	ret.push_back(it->actuatorId);
    }
    return ret;

}

void CommandDispatcher::processCommand(base::actuators::Command cmd)
{
    for(std::vector<InputMapping>::const_iterator it = inputMap.begin(); it != inputMap.end(); it++)
    {
	const int i = it->inputId;
	if(it->isReverse)
	{
	    cmd.invert(i);
	}
	setCommandCallback(it->actuatorId, cmd.mode[i], cmd.target[i]);
    }
};


