/*
 * PDDLUtils.h
 *
 *  Created on: 25 Aug 2015
 *      Author: tony
 */

#ifndef __PDDLUTILS
#define __PDDLUTILS

#include <list>
#include <set>
#include <string>
#include <map>

#include "PDDLObject.h"
#include "Proposition.h"
#include "Literal.h"
#include "PNE.h"
#include "TIL.h"
#include "PDDLState.h"

#include "../instantiation.h"
#include "ptree.h"
#include "../RPGBuilder.h"
#include "../FFEvent.h"
#include "../FFSolver.h"

namespace PDDL {

const std::string BASE_TYPE_CLASS = "OBJECT";
const std::string TIL_ACTION_PREFIX = "at-";

//PDDL Type helper functions
std::string getPDDLTypeString(const VAL::pddl_typed_symbol * type);
std::string getArgumentString(
		const VAL::typed_symbol_list<VAL::var_symbol> * arguments);
std::string getArgumentString(
		const VAL::typed_symbol_list<VAL::parameter_symbol> * arguments);
std::string getOperatorString(VAL::comparison_op op);
std::string getAssignmentString(VAL::assign_op op);
std::string getOperandString(const Planner::RPGBuilder::Operand & operand,
		const std::map<PDDL::PDDLObject, std::string> & parameterTable);
bool isOperator(const Planner::RPGBuilder::Operand & operand);
bool isOperand(const Planner::RPGBuilder::Operand & operand);
std::string getExpressionString(const VAL::expression * exp);
std::string getTimeSpecString(VAL::time_spec time_spec);
std::string getGoalString(const VAL::goal * goal);
std::string getEffectsString(const VAL::effect_lists * effects);

//Literal, PNE and TIL Helper Functions
std::set<PDDLObject> & extractParameters(Inst::Literal * literal,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants);
std::set<PDDLObject> & extractParameters(Inst::PNE * pne,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants);
set<PDDLObject> & extractParameters(
		std::list<Planner::RPGBuilder::Operand> * formula,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants);
set<PDDLObject> & extractParameters(VAL::simple_effect * prop,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants);
std::set<PDDLObject> & extractParameters(const Planner::RPGBuilder::FakeTILAction * til,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants);
set<PDDLObject> & extractParameters(
		const VAL::parameter_symbol_list * parameter_symbol_list,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants);
std::map<PDDLObject, std::string> generateParameterTable(
		const std::set<PDDLObject> & parameters);

//Action Helper Functions
std::list<PDDL::Literal> getActionConditions(int actionID,
		VAL::time_spec timeQualifier);
std::list<PDDL::Proposition> getActionEffects(int actionID,
		VAL::time_spec timeQualifier, bool positive);
std::string getActionName(int actionNum);
bool supported(const PDDL::Proposition * proposition,
		std::list<PDDL::Proposition> * propositions);
bool isEqual(const Planner::FFEvent * one, const Planner::FFEvent * two);
bool isTILAction(std::string eventName, int minDur, int maxDur);
Inst::instantiatedOp * getOperator(std::string actionInstance);

//Basic conversion functions
PDDL::PDDLObject getPDDLObject(const VAL::pddl_typed_symbol * pddlType);
PDDL::Proposition getFunction(const VAL::func_decl * func);
PDDL::TIL getTIL(Planner::RPGBuilder::FakeTILAction aTIL, double aTimestamp,
		std::list<std::pair<std::string, std::string> > constants = std::list<
				std::pair<std::string, std::string> >());

//Plan Helper Functions
std::list<std::string> getPlanPrefix(const std::list<Planner::FFEvent>& plan);
bool isAfter(const Planner::FFEvent * event, const Planner::FFEvent * after,
		std::list<Planner::FFEvent> * plan);
bool isBefore(const Planner::FFEvent * event, const Planner::FFEvent * before,
		std::list<Planner::FFEvent> * plan);

}

#endif /* __PDDLUTILS */
