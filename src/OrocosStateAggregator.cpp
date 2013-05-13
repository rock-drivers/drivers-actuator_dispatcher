#include "OrocosStateAggregator.hpp"

OrocsStateAggregator::OrocsStateAggregator(RTT::TaskContext& task, const std::string& name, const std::vector< int32_t >& statusMap, base::Time statusInterval) : StateAggregator(statusMap, statusInterval), task(task)
{
    std::string outputPortName("status_" + name);

    // Verify name clashes
    if (task.provides()->hasService(outputPortName))
    {
	RTT::log(RTT::Error) << "cannot create a dispatch called '" << name << "' as a port named  '" << outputPortName << "' is already in use in the task interface" << RTT::endlog();
	throw std::runtime_error(std::string("cannot create a dispatch called '") + name + "' as a port named  '" + outputPortName + "' is already in use in the task interface");
    }

    outputPort = new RTT::OutputPort< base::actuators::Status >(outputPortName);
    task.addPort(*outputPort);

    // Initialize the connections with the preallocated data structure, so that
    // further write() are hard-realtime
    outputPort->setDataSample(getStatus());
}
    
OrocsStateAggregator::~OrocsStateAggregator()
{
    task.provides()->removePort(outputPort->getName());
    delete outputPort;
}

void OrocsStateAggregator::writeStatus(const base::actuators::Status& status)
{
    outputPort->write(status);
}


OrocosCommandDispatcher::OrocosCommandDispatcher(RTT::TaskContext &task, boost::function<void (int32_t actuatorId, base::actuators::DRIVE_MODE mode, double value)> setCommandCallback, ::std::string const & name, ::std::vector< boost::int32_t > const & actuatorMap): CommandDispatcher(actuatorMap, setCommandCallback), task(task)
{
    std::string inputPortName("cmd_" + name);

	// Verify name clashes
    if (task.provides()->hasService(inputPortName))
    {
	RTT::log(RTT::Error) << "cannot create a dispatch called '" << inputPortName << "' as a port named '" << inputPortName << "' is already in use in the task interface" << RTT::endlog();
	throw std::runtime_error(std::string("cannot create a dispatch called '") + inputPortName + "' as a port named '" + inputPortName + "' is already in use in the task interface");
    }

    port = new RTT::InputPort< base::actuators::Command >(inputPortName);
    task.addPort(*port);
    
    task.provides()->addEventPort(*port);
    
    inputSample.resize(actuatorMap.size());
}

OrocosCommandDispatcher::~OrocosCommandDispatcher()
{
    task.provides()->removePort(port->getName());
    delete port;
}

void OrocosCommandDispatcher::process()
{
    bool gotData = false;
    
    while(port->read(inputSample, false) == RTT::NewData)
	gotData = true;
    
    if(gotData)
	processCommand(inputSample);
    
};
