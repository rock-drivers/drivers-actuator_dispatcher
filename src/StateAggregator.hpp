#ifndef STATEAGGREGATOR_HPP
#define STATEAGGREGATOR_HPP
#include <vector>
#include <map>
#include <base/time.h>
#include <base/actuators/status.h>
#include <base/actuators/commands.h>
#include <boost/function.hpp>

class StateAggregator
{
    class StateInfo
    {
    public:
	int stateId;
	int outputPosition;
	bool isReverse;
	bool updated;
	base::Time updateTime;
    };
    
    ///amount of dispatched actuators
    int actuatorCnt;
    
    ///conter for arrived state updates
    int updateCnt;
    
    base::actuators::Status status;
    std::vector<StateInfo *> stateIdToInfo;
    
    std::map<int, StateInfo> stateMap;

    ///The nominal time between two state updates
    base::Time statusInterval;
    
    void reset();

protected:
    const base::actuators::Status &getStatus() const 
    {
	return status;
    };
    
public:
    
    const std::vector<int32_t> getActuatorIds() const;
    
    StateAggregator(const std::vector<int32_t>& statusMap, base::Time statusInterval);
    virtual ~StateAggregator() {};
    
    void setNewStatus(int stateId, const base::Time stateTime, const base::actuators::MotorState& state);

    virtual void writeStatus(const base::actuators::Status &status) = 0;	
};

class CommandDispatcher
{
    class InputMapping
    {
    public:
	int actuatorId;
	int inputId;
	bool isReverse;
    };    
    
    ///map from positions in the base::actuator::command to the driver ids
    std::vector<InputMapping> inputMap;
    
    boost::function<void (int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value)> setCommandCallback;
    
public:
    const std::vector<int32_t> getActuatorIds() const;
    
    CommandDispatcher(std::vector< boost::int32_t > const & actuatorMap, boost::function<void (int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value)> setCommandCallback);
    virtual ~CommandDispatcher() {};
    
    void processCommand(base::actuators::Command cmd);    
};



#endif // STATEAGGREGATOR_HPP
