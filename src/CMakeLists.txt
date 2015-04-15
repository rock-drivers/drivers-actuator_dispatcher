add_definitions("-DOROCOS_TARGET=$ENV{OROCOS_TARGET}")

rock_find_cmake(Boost)

rock_library(actuator_dispatcher
    SOURCES OrocosStateAggregator.cpp StateAggregator.cpp Actuator.cpp
    HEADERS OrocosStateAggregator.hpp StateAggregator.hpp Actuator.hpp
    DEPS_PKGCONFIG base-lib orocos-rtt-$ENV{OROCOS_TARGET})
