cmake_minimum_required(VERSION 2.4)

project(PDDL)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(${val_SOURCE_DIR} ${val_SOURCE_DIR}/parsing)

set(libPDDL_SRCS
	PDDLObject.cpp
	Proposition.cpp
	PropositionFactory.cpp
	Literal.cpp
	LiteralFactory.cpp
	PNE.cpp
	ExpressionTree.cpp
	PNEEffect.cpp
	PNEFactory.cpp
	TIL.cpp
	PendingAction.cpp
	Metric.cpp
	PDDLUtils.cpp
	PDDLDomain.cpp
	PDDLState.cpp
	PDDLDomainFactory.cpp
	PDDLStateFactory.cpp
)

add_library(PDDL ${libPDDL_SRCS})
SET_TARGET_PROPERTIES(PDDL PROPERTIES COMPILE_FLAGS "-DTOTALORDERSTATES -DPOPF3ANALYSIS")

