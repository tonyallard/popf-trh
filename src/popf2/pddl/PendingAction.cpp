/*
 * PendingAction.cpp
 *
 *  Created on: 14 Jul 2015
 *      Author: tony
 */

#include <sstream>
#include <string>

#include "PendingAction.h"
#include "PDDLUtils.h"

using namespace std;
namespace PDDL {

list<PDDL::Proposition> PendingAction::getRequiredPropositionsParameterised() const {

	list<PDDL::Proposition> requiredProps;
	list<PDDL::Proposition>::const_iterator reqItr = requiredObjects.begin();
	for (; reqItr != requiredObjects.end(); reqItr++) {
		PDDL::Proposition propParameterised = reqItr->getParameterisedProposition(parameters, true);
		requiredProps.push_back(propParameterised);
	}
	return requiredProps;
}

std::ostream & operator<<(std::ostream & output, const PendingAction & action) {
	const map<PDDLObject, string> & parameterTable = action.getParameters();
	output << "\t(:durative-action " << action.name << endl;
	output << "\t\t:parameters (";
	map<PDDLObject, string>::const_iterator paramItr =
			parameterTable.begin();
	for (; paramItr != parameterTable.end(); paramItr++) {
		output << paramItr->second << " - " << paramItr->first.getTypeString()
				<< " ";
	}
	output << ")\n";
	output << "\t\t:duration (= ?duration " << action.timestamp << ")" << endl;
	output << "\t\t:condition (and " << endl;

	std::list<std::pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > >::const_iterator condItr =
			action.conditions.begin();
	const std::list<
			std::pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > >::const_iterator condItrEnd =
			action.conditions.end();
	for (; condItr != condItrEnd; condItr++) {
		std::pair<PDDL::Proposition, std::pair<VAL::time_spec, bool> > cond =
				*condItr;
		output << "\t\t\t(" << getTimeSpecString(cond.second.first) << " ";
		// Check if it is a negative condition
		if (!cond.second.second) {
			output << "(not ";
		}
		// Actually output condition
		output << cond.first.toParameterisedString(parameterTable);
		// Close negative if required
		if (!cond.second.second) {
			output << ") ";
		}
		output << ")" << endl;
	}
	output << "\t\t)" << endl;
	output << "\t\t:effect (and" << endl;

	//output propositional effects
	//Adds
	std::list<std::pair<Proposition, VAL::time_spec> >::const_iterator litItr =
			action.literalAddEffects.begin();
	std::list<std::pair<Proposition, VAL::time_spec> >::const_iterator litItrEnd =
			action.literalAddEffects.end();
	for (; litItr != litItrEnd; litItr++) {
		output << "\t\t\t(" << getTimeSpecString(litItr->second) << " ";
		output << litItr->first.toParameterisedString(parameterTable) << ")"
				<< endl;
	}
	//Dels
	litItr = action.literalDelEffects.begin();
	litItrEnd = action.literalDelEffects.end();
	for (; litItr != litItrEnd; litItr++) {
		output << "\t\t\t(" << getTimeSpecString(litItr->second) << " (not "
				<< litItr->first.toParameterisedString(parameterTable)
				<< "))\n";
	}
	//second get pne effects
	std::list<std::pair<PNEEffect, VAL::time_spec> >::const_iterator pneItr =
			action.pneEffects.begin();
	const std::list<std::pair<PNEEffect, VAL::time_spec> >::const_iterator pneItrEnd =
			action.pneEffects.end();
	for (; pneItr != pneItrEnd; pneItr++) {
		output << "\t\t\t(" << getTimeSpecString(pneItr->second) << " ";
		output << pneItr->first.toParameterisedString(parameterTable) << ")" << endl;
	}
	output << "\t\t)" << endl << "\t)" << endl;
	return output;
}
}

