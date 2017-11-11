/*
 * PDDLUtils.cpp
 *
 *  Created on: 25 Aug 2015
 *      Author: tony
 */

#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <regex>

#include "PDDLObject.h"
#include "PDDLUtils.h"
#include "PDDLStateFactory.h"
#include "PropositionFactory.h"
#include "LiteralFactory.h"
#include "PNEFactory.h"
#include "TILFactory.h"

#include "../globals.h"
#include "../RPGBuilder.h"
#include "../util/Util.h"
#include "../../VALfiles/TimSupport.h"

using namespace std;

namespace PDDL {

//PDDL Type Helper Functions
/**
 * Outputs the type of PDDL object in string format
 */
std::string getPDDLTypeString(const VAL::pddl_typed_symbol * type) {
	ostringstream output;
	if (type->either_types) {
		output << "(either ";
		VAL::pddl_type_list::const_iterator typeItr =
				type->either_types->begin();
		for (; typeItr != type->either_types->end(); typeItr++) {
			string typeName = (*typeItr)->getName();
			std::transform(typeName.begin(), typeName.end(), typeName.begin(),
					::toupper);
			output << typeName << " ";
		}
		output << ")";
	} else if (type->type) {
		string typeName = type->type->getName();
		std::transform(typeName.begin(), typeName.end(), typeName.begin(),
				::toupper);
		output << typeName;
	} else {
		//There was no type sepcified, defaulting
		output << BASE_TYPE_CLASS;
	}
	return output.str();
}

/**
 * Cycles through a list of arguments and prints them as a PDDL formatted string
 * for predicate or function parameters.
 */
std::string getArgumentString(
		const VAL::typed_symbol_list<VAL::var_symbol> * arguments) {
	ostringstream output;
	VAL::typed_symbol_list<VAL::var_symbol>::const_iterator argItr =
			arguments->begin();
	for (; argItr != arguments->end(); argItr++) {
		const VAL::var_symbol * arg = *argItr;
		string name = arg->getName();
		std::transform(name.begin(), name.end(), name.begin(), ::toupper);
		output << "?" << name << " - " << PDDL::getPDDLTypeString(arg) << " ";
	}
	return output.str();
}

/**
 * Cycles through arguments and prints them as a PDDL formatted string
 * for conditiona and effect clauses. This also handles Constants
 */
std::string getArgumentString(
		const VAL::typed_symbol_list<VAL::parameter_symbol> * arguments) {
	ostringstream output;
	VAL::typed_symbol_list<VAL::parameter_symbol>::const_iterator argItr =
			arguments->begin();
	for (; argItr != arguments->end(); argItr++) {
		const VAL::parameter_symbol * arg = *argItr;
		string typeName = arg->getName();
		std::transform(typeName.begin(), typeName.end(), typeName.begin(),
				::toupper);
		const VAL::var_symbol * varArg =
				dynamic_cast<const VAL::var_symbol *>(arg);
		if (varArg) {
			output << "?";
		}
		output << typeName << " ";
	}
	return output.str();
}

std::string getOperatorString(VAL::comparison_op op) {
	switch (op) {
	case VAL::comparison_op::E_GREATER:
		return ">";
		break;
	case VAL::comparison_op::E_GREATEQ:
		return ">=";
		break;
	case VAL::comparison_op::E_LESS:
		return "<";
		break;
	case VAL::comparison_op::E_LESSEQ:
		return "<=";
		break;
	case VAL::comparison_op::E_EQUALS:
		return "=";
		break;
	};
}

std::string getAssignmentString(VAL::assign_op op) {
	switch (op) {
	case VAL::assign_op::E_ASSIGN:
		return "assign";
		break;
	case VAL::assign_op::E_DECREASE:
		return "decrease";
		break;
	case VAL::assign_op::E_INCREASE:
		return "increase";
		break;
	case VAL::assign_op::E_SCALE_DOWN:
		return "/=";
		break;
	case VAL::assign_op::E_SCALE_UP:
		return "*=";
		break;
	};
}

string getOperandString(const Planner::RPGBuilder::Operand & operand,
		const map<PDDLObject, string> & parameterTable) {
	switch (operand.numericOp) {
	case Planner::RPGBuilder::math_op::NE_ADD:
		return "+";
		break;
	case Planner::RPGBuilder::math_op::NE_SUBTRACT:
		return "-";
		break;
	case Planner::RPGBuilder::math_op::NE_MULTIPLY:
		return "*";
		break;
	case Planner::RPGBuilder::math_op::NE_DIVIDE:
		return "/";
		break;
	case Planner::RPGBuilder::math_op::NE_CONSTANT:
	{
		ostringstream output;
		output << operand.constantValue;
		return output.str();
		break;
	}
	case Planner::RPGBuilder::math_op::NE_FLUENT: {
		if (operand.fluentValue < 0) {
			return "?duration";
		} else {
			Inst::PNE* aPNE = Planner::RPGBuilder::getPNE(operand.fluentValue);
			PDDL::PNE pne = PNEFactory::getInstance()->getPNE(aPNE, 0);
			return pne.toActionEffectString(parameterTable);
		}
		break;
	}
	case Planner::RPGBuilder::math_op::NE_VIOLATION:
	default:
		cerr
				<< "Something went wrong printing numeric effect. Unhandled math operation (ID: " << operand.numericOp << ")."
				<< endl;
		return NULL;
	}
}

bool isOperator(const Planner::RPGBuilder::Operand & operand) {
	if ((operand.numericOp == Planner::RPGBuilder::math_op::NE_ADD) ||
			(operand.numericOp == Planner::RPGBuilder::math_op::NE_SUBTRACT) ||
			(operand.numericOp == Planner::RPGBuilder::math_op::NE_MULTIPLY) ||
			(operand.numericOp == Planner::RPGBuilder::math_op::NE_DIVIDE)) {
		return true;
	}
	return false;
}

bool isOperand(const Planner::RPGBuilder::Operand & operand) {
	if ((operand.numericOp == Planner::RPGBuilder::math_op::NE_CONSTANT) ||
			(operand.numericOp == Planner::RPGBuilder::math_op::NE_FLUENT)) {
		return true;
	}
	return false;
}

/**
 *
 * Formats expressions into PDDL. Only basic expressions implemented
 * FIXME: Only basic expressions implemented
 */
std::string getExpressionString(const VAL::expression * exp) {
	std::ostringstream output;
   const VAL::plus_expression * plusExp =
			dynamic_cast<const VAL::plus_expression *>(exp);
	const VAL::minus_expression * minusExp =
			dynamic_cast<const VAL::minus_expression *>(exp);
	const VAL::mul_expression * mulExp =
			dynamic_cast<const VAL::mul_expression *>(exp);
	const VAL::div_expression * divExp =
			dynamic_cast<const VAL::div_expression *>(exp);
	const VAL::comparison * comp = dynamic_cast<const VAL::comparison *>(exp);
	const VAL::func_term * func = dynamic_cast<const VAL::func_term *>(exp);
	const VAL::special_val_expr * specVal =
			dynamic_cast<const VAL::special_val_expr *>(exp);
	const VAL::int_expression * intExp =
			dynamic_cast<const VAL::int_expression *>(exp);

	if (comp) {
		output << "(" << getOperatorString(comp->getOp()) << " "
				<< getExpressionString(comp->getLHS()) << " "
				<< getExpressionString(comp->getRHS()) << ")";
	} else if (plusExp) {
		output << "(* " << getExpressionString(plusExp->getLHS()) << " "
				<< getExpressionString(plusExp->getRHS()) << ")";
	} else if (minusExp) {
		output << "(- " << getExpressionString(minusExp->getLHS()) << " "
				<< getExpressionString(minusExp->getRHS()) << ")";
	} else if (mulExp) {
		output << "(* " << getExpressionString(mulExp->getLHS()) << " "
				<< getExpressionString(mulExp->getRHS()) << ")";
	} else if (divExp) {
		output << "(/ " << getExpressionString(divExp->getLHS()) << " "
				<< getExpressionString(divExp->getRHS()) << ")";
	} else if (func) {
		string name = func->getFunction()->getName();
		std::transform(name.begin(), name.end(), name.begin(),
				::toupper);
		output << "(" << name << " "
				<< getArgumentString(func->getArgs()) << ")";
	} else if (specVal) {
		if (specVal->getKind() == VAL::special_val::E_HASHT)
			output << "hasht";
		else if (specVal->getKind() == VAL::special_val::E_DURATION_VAR)
			output << "?duration";
		else if (specVal->getKind() == VAL::special_val::E_TOTAL_TIME)
			output << "total-time";
		else
			output << "?? ";
	} else if (intExp) {
		output << intExp->double_value();
	} else {
		cerr << "Something went wrong, unhandled expression." << endl;
		exp->display(0);
	}
	return output.str();
}

std::string getTimeSpecString(VAL::time_spec time_spec) {
	switch (time_spec) {
	case VAL::time_spec::E_AT:
		return "at";
	case VAL::time_spec::E_AT_END:
		return "at end";
	case VAL::time_spec::E_AT_START:
		return "at start";
	case VAL::time_spec::E_OVER_ALL:
		return "over all";
	case VAL::time_spec::E_CONTINUOUS:
		return "continuous";
	};
	return "???";
}

std::string getGoalString(const VAL::goal * goal) {
	std::ostringstream output;
	const VAL::conj_goal * conjGoal = dynamic_cast<const VAL::conj_goal *>(goal);
	const VAL::timed_goal * timedGoal =
			dynamic_cast<const VAL::timed_goal *>(goal);
	const VAL::simple_goal * simpleGoal =
			dynamic_cast<const VAL::simple_goal *>(goal);
	const VAL::comparison * compGoal =
			dynamic_cast<const VAL::comparison *>(goal);
	const VAL::neg_goal * negGoal = dynamic_cast<const VAL::neg_goal *>(goal);
	if (conjGoal) {
		output << "\t\t\t(and " << endl;
		VAL::goal_list::const_iterator goalItr = conjGoal->getGoals()->begin();
		for (; goalItr != conjGoal->getGoals()->end(); goalItr++) {
			const VAL::goal * aGoal = *goalItr;
			output << "\t\t\t\t" << getGoalString(aGoal) << endl;
		}
		output << "\t\t\t)" << endl;
	} else if (timedGoal) {
		output << "(" << getTimeSpecString(timedGoal->getTime()) << " "
				<< getGoalString(timedGoal->getGoal()) << ")";
	} else if (simpleGoal) {
		PDDL::Literal lit = LiteralFactory::getInstance()->getLiteral(simpleGoal);
		output << lit;
	} else if (compGoal) {
		output << getExpressionString(compGoal);
	} else if (negGoal) {
		output << "(not " << getGoalString(negGoal->getGoal()) << ")";
	} else {
		cerr << "Something went wrong printing goals. Unhandled Goal." << endl;
		goal->display(0);
	}
	return output.str();
}

std::string getEffectsString(const VAL::effect_lists * effects) {
	ostringstream output;
	int numEffects = effects->add_effects.size() + effects->del_effects.size()
			+ effects->timed_effects.size() + effects->assign_effects.size()
			+ effects->cond_assign_effects.size() + effects->cond_effects.size()
			+ effects->forall_effects.size();
	if (numEffects > 1) {
		output << "\t\t\t(and" << endl;
	}
	//timed effects
	VAL::pc_list<VAL::timed_effect*>::const_iterator teffItr =
			effects->timed_effects.begin();
	for (; teffItr != effects->timed_effects.end(); teffItr++) {
		const VAL::timed_effect* tEffect = *teffItr;
		output << "\t\t\t\t(" << getTimeSpecString(tEffect->ts) << " "
				<< getEffectsString(tEffect->effs) << ")" << endl;
	}
	//add effects
	VAL::pc_list<VAL::simple_effect*>::const_iterator addEffItr =
			effects->add_effects.begin();
	for (; addEffItr != effects->add_effects.end(); addEffItr++) {
		const VAL::simple_effect* addEffect = *addEffItr;
		PDDL::Proposition prop = PropositionFactory::getInstance()->
			getProposition(addEffect->prop, true);
		output << prop;
	}
	//del effects
	VAL::pc_list<VAL::simple_effect*>::const_iterator delEffItr =
			effects->del_effects.begin();
	for (; delEffItr != effects->del_effects.end(); delEffItr++) {
		const VAL::simple_effect* delEffect = *delEffItr;
		PDDL::Proposition prop = PropositionFactory::getInstance()->
			getProposition(delEffect->prop, true);
		PDDL::Literal lit = PDDL::Literal(prop, false);
		output << lit;
	}
	//assign effects
	VAL::pc_list<VAL::assignment*>::const_iterator assignEffItr =
			effects->assign_effects.begin();
	for (; assignEffItr != effects->assign_effects.end(); assignEffItr++) {
		const VAL::assignment* assignEffect = *assignEffItr;
		output << "(" << getAssignmentString(assignEffect->getOp()) << " "
				<< getExpressionString(assignEffect->getFTerm()) << " "
				<< getExpressionString(assignEffect->getExpr()) << ")";
	}
	if (numEffects > 1) {
		output << "\t\t\t)";
	}
	return output.str();
}

//Literal, PNE and TIL Helper Functions
set<PDDLObject> & extractParameters(Inst::Literal * literal,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants) {
	return extractParameters(literal->toProposition()->args, parameters,
			constants);
}

set<PDDLObject> & extractParameters(Inst::PNE * pne,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants) {
	return extractParameters(pne->getFunc()->getArgs(), parameters, constants);
}

set<PDDLObject> & extractParameters(std::list<Planner::RPGBuilder::Operand> * formula,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants) {
	std::list<Planner::RPGBuilder::Operand>::const_iterator formItr = formula->begin();
	for (; formItr != formula->end(); formItr++) {
		if (formItr->numericOp == Planner::RPGBuilder::math_op::NE_FLUENT) {
			if (formItr->fluentValue > 0) {
				Inst::PNE* aPNE = Planner::RPGBuilder::getPNE(formItr->fluentValue);
				parameters = extractParameters(aPNE, parameters, constants);
			}
		}
	}
	return parameters;
}

set<PDDLObject> & extractParameters(VAL::simple_effect* prop,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants) {
	return extractParameters(prop->prop->args, parameters, constants);
}

std::set<PDDLObject> & extractParameters(const Planner::RPGBuilder::FakeTILAction * til,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants) {
	//get params for adds
	list<Inst::Literal*>::const_iterator addItr = til->addEffects.begin();
	for (; addItr != til->addEffects.end(); addItr++) {
		extractParameters(*addItr, parameters, constants);
	}
	//get params for dels
	list<Inst::Literal*>::const_iterator delItr = til->delEffects.begin();
	for (; delItr != til->delEffects.end(); delItr++) {
		extractParameters(*delItr, parameters, constants);
	}
	return parameters;
}

set<PDDLObject> & extractParameters(
		const VAL::parameter_symbol_list * parameter_symbol_list,
		set<PDDLObject> & parameters,
		std::list<std::pair<std::string, std::string> > constants) {
	VAL::parameter_symbol_list::const_iterator argItr =
			parameter_symbol_list->begin();
	const VAL::parameter_symbol_list::const_iterator argItrEnd =
			parameter_symbol_list->end();
	for (; argItr != argItrEnd; argItr++) {
		const VAL::parameter_symbol * param = *argItr;
		string name = param->getName();
		std::transform(name.begin(), name.end(), name.begin(), ::toupper);
		std::list<std::pair<std::string, std::string> >::const_iterator constItr =
				constants.begin();
		bool constant = false;
		for (; constItr != constants.end(); constItr++) {
			if (name == constItr->first) {
				constant = true;
			}
		}
		if (!constant) {
			PDDLObject pddlObj = getPDDLObject(param);
			parameters.insert(pddlObj);
		}
	}
	return parameters;
}

std::map<PDDLObject, std::string> generateParameterTable(
		const std::set<PDDLObject> & parameters) {
	map<PDDLObject, string> parameterTable;
	//FIXME: means that there can only be 24 parameters before we create errors
	char letter = 'a';
	std::set<PDDLObject>::const_iterator paramItr = parameters.begin();
	for (; paramItr != parameters.end(); paramItr++) {
		PDDLObject pddlObj = *paramItr;
		std::ostringstream paramVar;
		paramVar << "?" << static_cast<char>(letter);
		letter++;
		parameterTable.insert(
				std::pair<PDDLObject, std::string>(pddlObj,
						paramVar.str()));
	}
	return parameterTable;
}

//Action Helper Functions

/**
 * returns the effects of an action which match the time qualified and sign
 */
std::list<PDDL::Proposition> getActionEffects(const Planner::FFEvent * action, 
	bool positive) {
	
	std::list<Inst::Literal*> effects;

	//Depending on time spec (start or end) add required effects
	if (action->time_spec == VAL::time_spec::E_AT_START) {
		int id = action->action->getID();
		if (positive) {
			effects = Planner::RPGBuilder::getStartPropositionAdds()[id];
		} else {
			effects =
					Planner::RPGBuilder::getStartPropositionDeletes()[id];
		}
	} else if (action->time_spec == VAL::time_spec::E_AT_END) {
		int id = action->action->getID();
		if (positive) {
			effects = Planner::RPGBuilder::getEndPropositionAdds()[id];
		} else {
			effects = Planner::RPGBuilder::getEndPropositionDeletes()[id];
		}
	} else if (action->time_spec == VAL::time_spec::E_AT) {
		int tilID = action->divisionID;
		if (positive) {
			effects = Planner::RPGBuilder::getAllTimedInitialLiterals()[tilID]->addEffects;
		} else {
			effects = Planner::RPGBuilder::getAllTimedInitialLiterals()[tilID]->delEffects;
		}
	} else {
		std::cerr << "This case not catered for.";
		assert(false);
	}
	return PropositionFactory::getInstance()->getPropositions(effects);
}

std::list<PDDL::Literal> getActionConditions(const Planner::FFEvent * action) {

	if (action->time_spec == VAL::time_spec::E_AT) {
		//TILs have no conditions
		return std::list<PDDL::Literal>();
	}

	int actionID = action->action->getID();
	std::list<Inst::Literal*> positiveConditions;
	std::list<Inst::Literal*> negativeConditions;
	//Add invariant (over all) conditions regardless
	//Positive First
	std::list<Inst::Literal*> tmpConditions =
			Planner::RPGBuilder::getInvariantPropositionalPreconditions()[actionID];
	positiveConditions.insert(positiveConditions.end(), tmpConditions.begin(),
			tmpConditions.end());
	//Then Negative
	tmpConditions =
			Planner::RPGBuilder::getInvariantNegativePropositionalPreconditions()[actionID];
	negativeConditions.insert(negativeConditions.end(), tmpConditions.begin(),
			tmpConditions.end());

	//Depending on time spec (start or end) add required conditions

	if (action->time_spec == VAL::time_spec::E_AT_START) {
		//Positive First
		tmpConditions =
				Planner::RPGBuilder::getStartPropositionalPreconditions()[actionID];
		positiveConditions.insert(positiveConditions.end(),
				tmpConditions.begin(), tmpConditions.end());
		//Then Negative
		tmpConditions =
				Planner::RPGBuilder::getStartNegativePropositionalPreconditions()[actionID];
		negativeConditions.insert(negativeConditions.end(),
				tmpConditions.begin(), tmpConditions.end());
	} else if (action->time_spec == VAL::time_spec::E_AT_END) {
		//Positive First
		tmpConditions =
				Planner::RPGBuilder::getEndPropositionalPreconditions()[actionID];
		positiveConditions.insert(positiveConditions.end(),
				tmpConditions.begin(), tmpConditions.end());
		//Then Negative
		tmpConditions =
				Planner::RPGBuilder::getEndNegativePropositionalPreconditions()[actionID];
		negativeConditions.insert(negativeConditions.end(),
				tmpConditions.begin(), tmpConditions.end());
	} else {
		std::cerr << "This case not catered for.";
		assert(false);
	}

	std::list<PDDL::Literal> literals;
	std::list<PDDL::Literal> tmpLiterals = LiteralFactory::getInstance()->getLiterals(
			&positiveConditions, true);
	literals.insert(literals.end(), tmpLiterals.begin(), tmpLiterals.end());
	tmpLiterals = LiteralFactory::getInstance()->getLiterals(&negativeConditions, false);
	literals.insert(literals.end(), tmpLiterals.begin(), tmpLiterals.end());
	return literals;
}

std::string getActionName(const Planner::FFEvent * action) {
	std::ostringstream output;
	if ((action->time_spec == VAL::time_spec::E_AT_START) ||
		(action->time_spec == VAL::time_spec::E_AT_END)){
		output << getOperatorName(action->action);		
	} else if (action->time_spec == VAL::time_spec::E_AT) {
		Planner::RPGBuilder::FakeTILAction * til =
					Planner::RPGBuilder::getAllTimedInitialLiterals()[action->divisionID];
		output << "at-" 
			<< PDDL::TILFactory::getInstance()->getTIL(*til, til->duration).getName();
	} else {
		std::cerr << "This case not catered for.";
		assert(false);		
	}
	return output.str();
}

std::string getOperatorName(Inst::instantiatedOp* action) {
	ostringstream output;
	output << action->getHead()->getName();
	VAL::var_symbol_list::const_iterator paramItr =
			action->forOp()->parameters->begin();
	const VAL::var_symbol_list::const_iterator paramItrEnd =
			action->forOp()->parameters->end();
	for (; paramItr != paramItrEnd; paramItr++) {
		output << "-" << ((*action->getEnv())[*paramItr])->getName();
	}
	return output.str();
}

bool supported(const PDDL::Proposition * proposition,
		std::list<PDDL::Proposition> * propositions) {
	std::list<PDDL::Proposition>::iterator propItr = propositions->begin();
	const std::list<PDDL::Proposition>::iterator propItrEnd =
			propositions->end();
	for (; propItr != propItrEnd; propItr++) {
		if ((*propItr) == (*proposition)) {
			return true;
		}
	}
	return false;
}

Inst::instantiatedOp * getOperator(std::string actionInstance) {
	int instantiatedOpCount = Planner::RPGBuilder::getInstantiatedOpCount();
	for (int i = 0; i < instantiatedOpCount; i++) {
		Inst::instantiatedOp * op = Planner::RPGBuilder::getInstantiatedOp(i);
		ostringstream op_inst;
		op->write(op_inst);
		if (!actionInstance.compare(op_inst.str())){
			//Action Operator Found
			return op;
		}
	}
	return NULL;
}

// Basic Conversions Functions

PDDL::PDDLObject getPDDLObject(const VAL::pddl_typed_symbol * pddlType) {
	string name = pddlType->getName();
	std::transform(name.begin(), name.end(), name.begin(), ::toupper);
	list<string> type;
	if (pddlType->either_types) {
		VAL::pddl_type_list::const_iterator typeItr =
				pddlType->either_types->begin();
		for (; typeItr != pddlType->either_types->end(); typeItr++) {
			string paramType = (*typeItr)->getName();
			std::transform(paramType.begin(), paramType.end(),
					paramType.begin(), ::toupper);
			type.push_back(paramType);
		}
	} else if (pddlType->type) {
		string paramType = pddlType->type->getName();
		std::transform(paramType.begin(), paramType.end(), paramType.begin(),
				::toupper);
		type.push_back(paramType);
	} else {
		//There was no type sepcified, defaulting
		type.push_back(BASE_TYPE_CLASS);
	}
	return PDDLObject(name, type);
}

PDDL::Proposition getFunction(const VAL::func_decl * func) {
	string name = func->getFunction()->getName();
	std::transform(name.begin(), name.end(), name.begin(), ::toupper);
	list<string> arguments;
	VAL::typed_symbol_list<VAL::var_symbol>::const_iterator argItr =
			func->getArgs()->begin();
	for (; argItr != func->getArgs()->end(); argItr++) {
		const VAL::var_symbol * arg = *argItr;
		ostringstream argStr;
		string argName = arg->getName();
		std::transform(argName.begin(), argName.end(), argName.begin(), ::toupper);
		argStr << "?" << argName << " - " << PDDL::getPDDLTypeString(*argItr);
		arguments.push_back(argStr.str());
	} 
	return PDDL::Proposition(name, arguments);
}

//Plan Helper Functions

std::list<std::string> getPlanPrefix(const std::list<Planner::FFEvent>& plan) {
	std::list<std::string> prefix;
	std::list<Planner::FFEvent>::const_iterator eventItr = plan.begin();
	const std::list<Planner::FFEvent>::const_iterator eventItrEnd = plan.end();
	for (; eventItr != eventItrEnd; eventItr++) {
		ostringstream name;
		Inst::instantiatedOp* op = (*eventItr).action;
		name << (*eventItr).lpTimestamp << ": " << getActionName(&(*eventItr));
		prefix.push_back(name.str());
	}
	return prefix;
}

/**
 * Is 'event' before parameter 2 (aka: 'before') in the plan
 * note that this goes through the plan in reverse
 */
bool isAfter(const Planner::FFEvent * event, const Planner::FFEvent * after,
		const std::list<Planner::FFEvent> & plan) {
	std::list<Planner::FFEvent>::const_reverse_iterator eventItr = plan.rbegin();
	const std::list<Planner::FFEvent>::const_reverse_iterator eventItrEnd =
			plan.rend();
	for (; eventItr != eventItrEnd; eventItr++) {
		const Planner::FFEvent * ev = &(*eventItr);
		if (ev == event) {
			return true;
		} else if (ev == after) {
			return false;
		}
	}
	return false;
}

/**
 * Is 'event' before parameter 2 (aka: 'before') in the plan
 */
bool isBefore(const Planner::FFEvent * event, const Planner::FFEvent * before,
		const std::list<Planner::FFEvent> & plan) {
	std::list<Planner::FFEvent>::const_iterator eventItr = plan.begin();
	const std::list<Planner::FFEvent>::const_iterator eventItrEnd = plan.end();
	for (; eventItr != eventItrEnd; eventItr++) {
		const Planner::FFEvent * ev = &(*eventItr);
		if (ev == event) {
			return true;
		} else if (ev == before) {
			return false;
		}
	}
	return false;
}

}
