/*
 * Proposition.cpp
 *
 *  Created on: 8 Jul 2015
 *      Author: tony
 */

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <algorithm>

#include "Proposition.h"

namespace PDDL {

/**
 * Output the proposition using parameters according to the parameter table
 * FIXME: Assumes arguments that are not in the parameter table are constants
 * Should probably use the contants table to confirm this.
 */
std::string Proposition::toParameterisedString(
		const std::map<PDDLObject, std::string> & parameterTable) const {
	std::ostringstream output;
	output << "(" << name << " ";
	std::list<std::string>::const_iterator argItr = arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd = arguments.end();
	for (; argItr != argItrEnd; argItr++) {
		bool found = false;
		// Find the corresponding parameters
		std::map<PDDLObject, std::string>::const_iterator paramItr = parameterTable.begin();
		for (; paramItr != parameterTable.end(); paramItr++) {
			std::pair<PDDLObject, std::string> param = *paramItr;
			if (param.first.getName().compare(*argItr) == 0) {
				output << param.second << " ";
				found = true;
			}
		}
		//Must be a constant
		if (!found) {
			string constant = *argItr;
			std::transform(constant.begin(), constant.end(), constant.begin(),
							::toupper);
			output << constant << " ";
		}
	}
	output << ")";
	return output.str();
}

Proposition Proposition::getParameterisedProposition(
		const std::map<PDDLObject, std::string> & parameterTable, bool showTypes /*= false*/) const {
	string name = this->name;
	list<string> args;
	std::list<std::string>::const_iterator argItr = arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd = arguments.end();
	for (; argItr != argItrEnd; argItr++) {
		bool found = false;
		// Find the corresponding parameters
		std::map<PDDLObject, std::string>::const_iterator paramItr = parameterTable.begin();
		for (; paramItr != parameterTable.end(); paramItr++) {
			std::pair<PDDLObject, std::string> param = *paramItr;
			if (param.first.getName().compare(*argItr) == 0) {
				string argName = param.second;
				if (showTypes) {
					argName += " - " + param.first.getTypeString();
				}
				args.push_back(argName);
				found = true;
			}
		}
		//Must be a constant
		if (!found) {
			string constant = *argItr;
			std::transform(constant.begin(), constant.end(), constant.begin(),
							::toupper);
			args.push_back(constant);
		}
	}
	return Proposition(name, args);
}

std::ostream & operator<<(std::ostream & output,
		const Proposition & proposition) {
	output << "(" << proposition.name << " ";
	std::list<std::string>::const_iterator argItr =
			proposition.arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd =
			proposition.arguments.end();
	for (; argItr != argItrEnd; argItr++) {
		output << (*argItr) << " ";
	}
	output << ")";
	return output;
}

std::string Proposition::getDecoratedName(const Proposition & proposition) {
	std::ostringstream output;
	output << proposition.name;
	std::list<std::string>::const_iterator argItr =
			proposition.arguments.begin();
	const std::list<std::string>::const_iterator argItrEnd =
			proposition.arguments.end();
	for (; argItr != argItrEnd; argItr++) {
		output << "-" << (*argItr);
	}
	return output.str();
}

bool Proposition::operator==(const Proposition & other) {
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
	return true;
}

bool Proposition::operator!=(const Proposition & other) {
	return !((*this) == other);
}

bool Proposition::operator<(const Proposition & other) const{
	if (name < other.name) {
		return true;
	} else if (!(other.name < name) && 
		(arguments.size() < other.arguments.size())) {
		return true;
	} else if (arguments.size() == other.arguments.size()) {
		std::list<std::string>::const_iterator argItr = arguments.begin();
		std::list<std::string>::const_iterator otherArgItr = other.arguments.begin();
		for (; argItr != arguments.end(); argItr++, otherArgItr++) {
			if (!(*argItr < *otherArgItr)) {
				return false;
			}
		}
		return true;
	}
	return false;
}

bool Proposition::operator>(const Proposition & other) const{
	return other < (*this);
}

bool Proposition::operator<=(const Proposition & other) const{
	return !(other < (*this));
}

bool Proposition::operator>=(const Proposition & other) const{
	return !((*this) < other);
}

}

