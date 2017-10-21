/*
 * PNE.cpp
 *
 *  Created on: 8 Jul 2015
 *      Author: tony
 */

#include <iostream>
#include <sstream>

#include "PNE.h"

using namespace std;
namespace PDDL {

std::string PNE::toActionEffectString(
		const map<PDDLObject, string> & parameterTable) const {
	ostringstream output;
	output << "(" << name << " ";
	std::list<std::string>::const_iterator argItr = arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd = arguments.end();
	for (; argItr != argItrEnd; argItr++) {
		// Find the corresponding parameters
		std::map<PDDLObject, std::string>::const_iterator paramItr =
				parameterTable.begin();
		for (; paramItr != parameterTable.end(); paramItr++) {
			if (paramItr->first.getName().compare(*argItr) == 0) {
				output << paramItr->second << " ";
			}
		}
	}
	output << ")";
	return output.str();
}

std::ostream & operator<<(std::ostream & output, const PNE & pne) {
	output << "(= (" << pne.name << " ";
	std::list<std::string>::const_iterator argItr = pne.arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd =
			pne.arguments.end();
	for (; argItr != argItrEnd; argItr++) {
		output << (*argItr) << " ";
	}
	output << ") " << pne.value << ")";
	return output;
}

bool PNE::operator==(const PNE & other) {
	if (name.compare(other.name)) {
		return false;
	}
	if (arguments.size() != other.arguments.size()) {
		return false;
	}
	std::list<std::string>::const_iterator argItr = arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd = arguments.end();
	std::list<std::string>::const_iterator otherArgItr =
			other.arguments.begin();
	for (; argItr != argItrEnd; argItr++, otherArgItr++) {
		if (argItr->compare(*otherArgItr)) {
			return false;
		}
	}

	if (value != other.value) {
		return false;
	}

	return true;
}

}

