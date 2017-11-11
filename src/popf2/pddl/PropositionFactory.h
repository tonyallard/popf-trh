/*
 * PDDLStateFactory.h
 *
 *  Created on: 2 Jul 2015
 *      Author: tony
 */

#ifndef __PROPOSITIONFACTORY
#define __PROPOSITIONFACTORY

#include <string>
#include <list>

#include "Proposition.h"

#include "ptree.h"
#include "instantiation.h"

namespace PDDL {

class PropositionFactory {
public:

	static PropositionFactory * getInstance();

	std::list<PDDL::Proposition> getPropositions(const std::list<Inst::Literal*> & literals,
			bool isTemplate = false, bool showType = false);
	PDDL::Proposition getProposition(const VAL::pred_decl * predicate);
	PDDL::Proposition getProposition(const Inst::Literal * aLiteral,
			bool isTemplate = false, bool showType = false);

	//Un-Grounded
	PDDL::Proposition getProposition(const VAL::proposition * prop,
			bool isTemplate = false, bool showType = false);

	//Grounded
	PDDL::Proposition getGroundedProposition(
			const VAL::proposition * prop, VAL::FastEnvironment * env, bool showType);

	PDDL::Proposition getEmptyProposition();

private:
	//Singleton Instance
	static PropositionFactory * INSTANCE;
	static const string EMPTY_PROPOSITION_NAME;
	
	//Private constructor
	PropositionFactory(){};
	//Singleton
	PropositionFactory(PropositionFactory const & other) {
	}
	;
	//Singleton
	PropositionFactory& operator=(PropositionFactory const&) {
	}
	;

	/*Private members*/

	//Un-Grounded
	list<string> getParameters(
		const VAL::var_symbol_list * params, bool isTemplate = false, bool showType = false);
	list<string> getParameters(const VAL::parameter_symbol_list * params,
		bool isTemplate = false, bool showType = false);
	string getParameter(VAL::pddl_typed_symbol * symbol, bool isTemplate, bool showType);

	//Grounded
	list<string> getGroundedParameters(const VAL::parameter_symbol_list * params,
			VAL::FastEnvironment * env, bool showType = false);
	string getGroundedParameter(VAL::pddl_typed_symbol * symbol,
			VAL::FastEnvironment * env, bool showType);
};

}

#endif /* __PROPOSITIONFACTORY */
