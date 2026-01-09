add_library(solver_code_gen
    robot/generated_code/nlp_solver_generated.c
)

# Ipopt library
target_link_directories(solver_code_gen
    PUBLIC
        /usr/local/lib
)

# Link solver_code_gen to its dependencies
target_link_libraries(solver_code_gen
    PUBLIC
        casadi
        ipopt
)

# Make solver_code_gen headers available
target_include_directories(solver_code_gen
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/generated_code
        /usr/local/include/
)