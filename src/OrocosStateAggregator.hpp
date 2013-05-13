#ifndef OROCOS_COMMAND_DISPATCHER_HPP
#define OROCOS_COMMAND_DISPATCHER_HPP

#include "StateAggregator.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

class OrocsStateAggregator: public StateAggregator
{
private:
    RTT::OutputPort< base::actuators::Status > *outputPort;
    RTT::TaskContext &task;
public:
    OrocsStateAggregator(RTT::TaskContext &task, ::std::string const & name, const std::vector< int32_t >& statusMap, base::Time statusInterval);
    
    ~OrocsStateAggregator();
    
    virtual void writeStatus(const base::actuators::Status& status);
};


class OrocosCommandDispatcher: public CommandDispatcher
{
    RTT::InputPort< base::actuators::Command > *port;
    RTT::TaskContext &task;

    base::actuators::Command inputSample;
public:
    OrocosCommandDispatcher(RTT::TaskContext &task, boost::function<void (int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value)> setCommandCallback, ::std::string const & name, ::std::vector< boost::int32_t > const & actuatorMap);
    
    ~OrocosCommandDispatcher();
    
    void process();
};

#endif
