/*
 * PNEFactory.h
 *
 *  Created on: 1 May 2016
 *      Author: tony
 */

#ifndef COLIN_PDDL_PNEFACTORY_H_
#define COLIN_PDDL_PNEFACTORY_H_

#include <string>
#include <list>

#include "PNE.h"
#include "ptree.h"
#include "instantiation.h"

using namespace std;

namespace PDDL {

class PNEFactory {
public:

	static PNEFactory * getInstance();
	PDDL::PNE getPNE(const Inst::PNE * aPNE, double value);
	PDDL::PNE getPNE(const VAL::func_term * func, double value);

private:
	//Singleton Instance
	static PNEFactory * INSTANCE;

	//Private constructor
	PNEFactory(){};
	//Singleton
	PNEFactory(PNEFactory const & other) {
	}
	;
	//Singleton
	PNEFactory& operator=(PNEFactory const&) {
	}
	;

	//Private functions
	list<string> getParameters(
		const VAL::parameter_symbol_list * params);
	string getParameter(VAL::pddl_typed_symbol * symbol);

};

}

#endif /* COLIN_PDDL_PNEFACTORY_H_ */
