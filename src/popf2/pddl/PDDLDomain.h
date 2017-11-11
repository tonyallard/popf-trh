/*
 * PDDLDomain.h
 *
 *  Created on: 28 Apr 2016
 *      Author: tony
 */

#ifndef __PDDLDOMAIN
#define __PDDLDOMAIN

#include <string>
#include <list>
#include <set>
#include <utility>
#include <iostream>

#include "Proposition.h"
#include "PDDLObject.h"
#include "TIL.h"

#include "../RPGBuilder.h"

using namespace std;

namespace PDDL {

class PDDLDomain {

private:
	string name;
	list<string> requirements;
	list<PDDL::PDDLObject> types;
	list<PDDL::Proposition> predicates;
	list<PDDL::Proposition> functions;
	std::list<std::pair<std::string, std::string> > constants;
	list<std::string> actions;
	list<PDDL::Proposition> tilPredicates;
	list<PDDL::Proposition> tilGoalPredicates;
	list<PDDL::Proposition> tilRequiredObjects;
	set<PDDLObject> domainObjectSymbolTable;
	list<PDDL::Proposition> pendingActionRequiredObjects;
	std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> tilMap;

	string getHeaderString() const;
	string getRequirementsString() const;
	string getTypesString() const;
	string getPredicatesString() const;
	string getFunctionsString() const;
	string getConstantsString() const;
	string getActionsString() const;
	string getTerminationString() const;

public:
	PDDLDomain(){};
	PDDLDomain(string name, list<string> requirements, list<PDDL::PDDLObject> types,
					list<PDDL::Proposition> predicates,	list<PDDL::Proposition> functions, 
					std::list<std::pair<std::string, std::string> > constants, 
					list<std::string> actions) : 
					name(name), requirements(requirements), types(types),
					predicates(predicates), functions(functions), 
					constants(constants), actions(actions){
	}
	;
	PDDLDomain(		string name, list<string> requirements, 
					list<PDDL::PDDLObject> types,
					list<PDDL::Proposition> predicates,	
					list<PDDL::Proposition> functions, 
					std::list<std::pair<std::string, std::string> > constants, 
					list<std::string> actions,
					std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> tilMap, 
					list<PDDL::Proposition> tilPredicates, list<PDDL::Proposition> tilGoalPredicates, 
					list<PDDL::Proposition> tilRequiredObjects,	
					list<PDDL::Proposition> pendingActionRequiredObjects, 
					set<PDDLObject> domainObjectSymbolTable) :
					name(name), requirements(requirements), types(types),
					predicates(predicates), functions(functions), 
					constants(constants), actions(actions), tilMap(tilMap),
					tilPredicates(tilPredicates), tilGoalPredicates(tilGoalPredicates),
					tilRequiredObjects(tilRequiredObjects), pendingActionRequiredObjects(pendingActionRequiredObjects),
					domainObjectSymbolTable(domainObjectSymbolTable) {};
	inline std::string getName() {return name;}
	inline const list<string> & getRequirements() {return requirements;}
	inline const list<PDDL::PDDLObject> & getTypes() {return types;}
	inline const list<PDDL::Proposition> & getPredicates() {return predicates;}
	inline const list<PDDL::Proposition> & getFunctions() {return functions;}
	inline const list<pair<string, string> > & getConstants() {return constants;}
	inline const list<std::string> & getActions() {return actions;}
	inline const std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> & getTILMap() {return tilMap;}
	inline const list<PDDL::Proposition> & getTILGoalPredicates() {return tilGoalPredicates;}
	inline const list<PDDL::Proposition> & getTILPredicates() {return tilPredicates;}
	inline const list<PDDL::Proposition> & getTILRequiredObjects() {return tilRequiredObjects;}
	inline const list<PDDL::Proposition> & getPendingActionRequiredObjects() {return pendingActionRequiredObjects;}
	inline const set<PDDLObject> & getDomainObjectSymbolTable() {return domainObjectSymbolTable;}
	std::string toString() const;
	void writeToFile(std::string filePath, std::string fileName);
	friend std::ostream & operator<<(std::ostream & output, const PDDLDomain & domain);
};

}

#endif /* __PDDLDOMAIN */
