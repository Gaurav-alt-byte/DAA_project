add_library(sumo_integration STATIC sumo_bridge.cpp)
target_include_directories(sumo_integration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Raw-socket TraCI client (traci_client.h) — no libsumo dependency, just
# the platform socket library.
if(WIN32)
    target_link_libraries(sumo_integration PUBLIC ws2_32)
endif()
# Once sumo_bridge.cpp exists:
# find_package(LibSUMO REQUIRED)  # or point at $ENV{SUMO_HOME}
# target_link_libraries(sumo_integration INTERFACE libsumocpp)
