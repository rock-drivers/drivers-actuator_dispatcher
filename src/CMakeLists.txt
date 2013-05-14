rock_library(actuator_dispatcher
    SOURCES OrocosStateAggregator.cpp StateAggregator.cpp Actuator.cpp
    HEADERS OrocosStateAggregator.hpp StateAggregator.hpp Actuator.hpp
    DEPS_PKGCONFIG base-lib)


