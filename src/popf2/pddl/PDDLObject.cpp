/*
 * PDDLObject.cpp
 *
 *  Created on: 25 Oct 2015
 *      Author: tony
 */

#include <sstream>
#include "PDDLObject.h"

using namespace std;
namespace PDDL {

string PDDLObject::getTypeString() const {
	ostringstream output;
	if (type.size() > 1) {
		output << "(either ";
	}
	std::list<std::string>::const_iterator typeItr = type.begin();
	for (; typeItr != type.end(); typeItr++) {
		output << *typeItr << " ";
	}
	if (type.size() > 1) {
		output << ")";
	}
	return output.str();
}

bool PDDLObject::operator<(const PDDLObject & other) const {
	return name < other.name;
}

bool PDDLObject::operator==(const PDDLObject & other) {
	//different name
	if (name.compare(other.name)) {
		return false;
	}
	if (type.size() != other.type.size()) {
		return false;
	}
	std::list<std::string>::const_iterator typeItr = type.begin();
	std::list<std::string>::const_iterator otherTypeItr = other.type.begin();
	for (; typeItr != type.end(); typeItr++, otherTypeItr++) {
		std::string typeName = *typeItr;
		std::string otherTypeName = *otherTypeItr;
		if (typeName.compare(otherTypeName)) {
			return false;
		}
	}
	return true;
}

ostream & operator<<(ostream & out, const PDDLObject & pddlObject) {
	out << pddlObject.name << " - " << pddlObject.getTypeString();
	return out;
}
}
