PROJECT(popf2)

set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}
                          ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
                          ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules)


set(DOXYFILE_TARGET_SUFFIX popf)
include(UseDoxygen OPTIONAL)

add_subdirectory(util)
add_subdirectory(pddl)
add_subdirectory(kk)
add_subdirectory(stn)
add_subdirectory(itc)
add_subdirectory(executor)
add_subdirectory(hRelax)
add_subdirectory(TRH)

#IF(NOT DISABLE_CLP)
find_package(Clp)
find_package(CoinUtils)
find_package(Cbc)
find_package(Cgl)
find_package(OSI)
#ENDIF(NOT DISABLE_CLP)

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing ${val_BINARY_DIR} ${PDDL_SOURCE_DIR} ${PDDL_BINARY_DIR} ${TRH_SOURCE_DIR} ${TRH_BINARY_DIR} ${CLP_INCLUDES}/coin ${CBC_INCLUDES}/coin ${CPLEX_INCLUDES} ${CONCERT_INCLUDES})

set(popf2_build_srcs
    HTrio.cpp
    compressionsafescheduler.cpp
    totalordertransformer.cpp
    partialordertransformer.cpp
    FFSolver.cpp
    FFEvent.cpp
    globals.cpp
    lpscheduler.cpp
    RPGBuilder.cpp
    RPGBuilderAnalysis.cpp
    RPGBuilderEvaluation.cpp
    RPGBuilderNumerics.cpp
    temporalanalysis.cpp
    minimalstate.cpp
    temporalconstraints.cpp
    numericanalysis.cpp
    solver.cpp)

add_library(POPF2Common STATIC ${popf2_build_srcs})
target_link_libraries(POPF2Common ParsePDDL Inst UTIL PDDL TRH)

add_library(POPF3Common STATIC ${popf2_build_srcs})
target_link_libraries(POPF3Common ParsePDDL Inst UTIL PDDL TRH)
SET_TARGET_PROPERTIES(POPF3Common PROPERTIES COMPILE_FLAGS "-DPOPF3ANALYSIS")

add_library(StochasticPOPF2Common STATIC ${popf2_build_srcs} StochasticDurations.cpp)
target_link_libraries(StochasticPOPF2Common ParsePDDL Inst UTIL PDDL TRH)
SET_TARGET_PROPERTIES(StochasticPOPF2Common PROPERTIES COMPILE_FLAGS "-DSTOCHASTICDURATIONS -DPOPF3ANALYSIS")

#if((NOT DISABLE_CLP) AND CLP_INCLUDES)
if (CLP_INCLUDES AND CBC_INCLUDES)
    ADD_EXECUTABLE(popf2-clp popfMain.cpp solver-clp.cpp)
    
    target_link_libraries(popf2-clp
        POPF2Common
        ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES}
    )

    ADD_EXECUTABLE(popf3-clp popfMain.cpp solver-clp.cpp)
    
    target_link_libraries(popf3-clp
        POPF3Common
        ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES}
    )
    SET_TARGET_PROPERTIES(popf3-clp PROPERTIES COMPILE_FLAGS "-DPOPF3ANALYSIS")

    ADD_EXECUTABLE(popf2-clp-stochastic popfMain.cpp solver-clp.cpp)
    
    target_link_libraries(popf2-clp-stochastic
        StochasticPOPF2Common
        ${CBC_SOLVER_LIBRARY} ${CBC_LIBRARY} ${CGL_LIBRARIES} ${OSICLP_LIBRARIES} ${OSI_LIBRARIES} ${CLP_LIBRARIES}  ${COINUTILS_LIBRARIES} -lgsl
    )
    SET_TARGET_PROPERTIES(popf2-clp-stochastic PROPERTIES COMPILE_FLAGS "-DSTOCHASTICDURATIONS -DPOPF3ANALYSIS")
    
#endif((NOT DISABLE_CLP) AND CLP_INCLUDES)
endif(CLP_INCLUDES AND CBC_INCLUDES)

if(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)
    ADD_EXECUTABLE(popf2-cplex popfMain.cpp solver-cpx.cpp)
    
    target_link_libraries(popf2-cplex    
        POPF2Common
        ${ILOCPLEX} ${CPLEX} ${CONCERT} pthread
    )
    
enable_testing()

endif(CPLEX AND ILOCPLEX AND CONCERT AND CPLEX_INCLUDES AND CONCERT_INCLUDES)

