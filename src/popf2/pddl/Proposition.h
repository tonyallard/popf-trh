/*
 * Proposition.h
 *
 *  Created on: 2 Jul 2015
 *      Author: tony
 */

#ifndef __PDDL_PROPOSITION
#define __PDDL_PROPOSITION

#include <list>
#include <map>
#include <string>
#include <iostream>

#include "PDDLObject.h"

namespace PDDL {

class Proposition {

private:
	std::string name;
	std::list<std::string> arguments;

public:
	Proposition(std::string name, std::list<std::string> arguments) : name(name), arguments(arguments) {};
	Proposition(){};
	const std::string & getName() const { return name; };
	const std::list<std::string> & getArguments() const {return arguments; };
	std::string toParameterisedString(const std::map<PDDLObject, std::string> & parameterTable) const;
	Proposition getParameterisedProposition(
			const std::map<PDDLObject, std::string> & parameterTable, bool showTypes = false) const;
	bool operator==(const Proposition & other);
	bool operator!=(const Proposition & other);
	bool operator<(const Proposition & other) const;
	bool operator>(const Proposition & other) const;
	bool operator<=(const Proposition & other) const;
	bool operator>=(const Proposition & other) const;
	static std::string getDecoratedName(const Proposition & proposition);
	friend std::string getDecoratedName(const Proposition & proposition);
	friend std::ostream & operator<<(std::ostream & output, const Proposition & proposition);
};

}



#endif /* __PDDL_PROPOSITION */
