/*
 * PNEFactory.cpp
 *
 *  Created on: 1 May 2016
 *      Author: tony
 */

#include "PNEFactory.h"

using namespace std;

namespace PDDL {

PNEFactory * PNEFactory::INSTANCE = NULL;

PNEFactory * PNEFactory::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new PNEFactory();
	}
	return INSTANCE;
}

PDDL::PNE PNEFactory::getPNE(const Inst::PNE * aPNE, double value) {
	string name = aPNE->getFunc()->getFunction()->getName();
	transform(name.begin(), name.end(), name.begin(), ::toupper);
	std::list<string> params = getParameters(aPNE->getFunc()->getArgs());
	return PDDL::PNE(name, params, value);
}



PDDL::PNE PNEFactory::getPNE(const VAL::func_term * func, double value) {
	string name = func->getFunction()->getName();
	transform(name.begin(), name.end(), name.begin(), ::toupper);
	std::list<string> params = getParameters(func->getArgs());
	return PDDL::PNE(name, params, value);
}

list<string> PNEFactory::getParameters(
	const VAL::parameter_symbol_list * params) {

	list<string> parameters;
	VAL::parameter_symbol_list::const_iterator argItr =
			params->begin();
	const VAL::parameter_symbol_list::const_iterator argItrEnd =
			params->end();
	for (; argItr != argItrEnd; argItr++) {
		parameters.push_back(getParameter(*argItr));
	}
	return parameters;
}

string PNEFactory::getParameter(VAL::pddl_typed_symbol * symbol) {
	string argName = symbol->getName();
	transform(argName.begin(), argName.end(), argName.begin(), ::toupper);
	return argName;
}

}
