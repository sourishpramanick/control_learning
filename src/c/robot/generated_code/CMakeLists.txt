add_library(codegen
    nlp_solver_generated.c)

# Find Ipopt library from CasADi's site-packages
find_library(IPOPT_LIBRARY
    NAMES ipopt
    PATHS /home/sourish/miniconda3/envs/mpc_dl/lib/python3.11/site-packages/casadi
    NO_DEFAULT_PATH
)

if(NOT IPOPT_LIBRARY)
    message(FATAL_ERROR "Ipopt library not found in CasADi installation")
endif()

message(STATUS "Found Ipopt library: ${IPOPT_LIBRARY}")

# Link against Ipopt
target_link_libraries(codegen
    PUBLIC
        ${IPOPT_LIBRARY}
        casadi
)

target_include_directories(codegen
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)