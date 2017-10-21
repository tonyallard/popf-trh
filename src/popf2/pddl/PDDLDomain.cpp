/*
 * PDDLDomain.cpp
 *
 *  Created on: 28 Apr 2016
 *      Author: tony
 */

#include <sstream>
#include <fstream>

#include "PDDLDomain.h"

using namespace std;

namespace PDDL {

string PDDLDomain::getHeaderString() const {
	ostringstream output;
	output << "(define (domain " << name << ")" << endl;
	return output.str();
}

string PDDLDomain::getRequirementsString() const {
	ostringstream output;
	output << "\t(:requirements";
	list<string>::const_iterator reqItr = requirements.begin();
	for (; reqItr != requirements.end(); reqItr++) {
		output << " " << *reqItr;
	}
	output << ")" << std::endl;
	return output.str();
}

string PDDLDomain::getTypesString() const {
	ostringstream output;
	output << "\t(:types" << endl;
	list<PDDL::PDDLObject>::const_iterator typeItr = types.begin();
	for (; typeItr != types.end(); typeItr++) {
		output << "\t\t" << *typeItr << endl;
	}
	output << "\t)" << endl;
	return output.str();
}

string PDDLDomain::getPredicatesString() const {
	ostringstream output;
	output << "\t(:predicates" << endl;
	list<PDDL::Proposition>::const_iterator predItr = predicates.begin();
	for (; predItr != predicates.end(); predItr++) {
		output << "\t\t" << *predItr << endl;	
	}
	output << "\t)" << endl;
	return output.str();
}

string PDDLDomain::getFunctionsString() const {
	ostringstream output;
	output << "\t(:functions" << endl;

	list<PDDL::Proposition>::const_iterator funcItr = functions.begin();
	for (; funcItr != functions.end(); funcItr++) {
		output << "\t\t" << *funcItr << endl;
	}
	output << "\t)" << endl;
	return output.str();
}

string PDDLDomain::getConstantsString() const {
	ostringstream output;
	if (constants.size()) {
		output << "\t(:constants" << endl;
		std::list<std::pair<std::string, std::string> >::const_iterator constItr =
				constants.begin();
		for (; constItr != constants.end(); constItr++) {
			std::pair<std::string, std::string> constant = *constItr;
			output << "\t\t" << constant.first << " - " << constant.second
					<< endl;
		}
		output << "\t)" << endl;
	}
	return output.str();
}

string PDDLDomain::getActionsString() const {
	ostringstream output;
	list<string>::const_iterator acItr = actions.begin();
	for (; acItr != actions.end(); acItr++) {
		output << *acItr << endl;
	}
	return output.str();
}

std::string PDDLDomain::getTerminationString() const {
	return ")";
}
	
std::string PDDLDomain::toString() const {
	ostringstream output;
	output << getHeaderString();
	output << getRequirementsString();
	output << getTypesString();
	output << getPredicatesString();
	output << getFunctionsString();
	output << getConstantsString();
	output << getActionsString();
	output << getTerminationString();
	return output.str();
}

void PDDLDomain::writeToFile(std::string filePath, std::string fileName) {
	ofstream myFile;
	ostringstream fullFilePath;
	fullFilePath << filePath << fileName << ".pddl";
	myFile.open(fullFilePath.str());
	myFile << *this;
	myFile.close();
}

std::ostream & operator<<(std::ostream & output, const PDDLDomain & domain) {
	output << domain.toString();
	return output;
}
}
