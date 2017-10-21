/*
 * PDDLState.cpp
 *
 *  Created on: 19 Jun 2015
 *      Author: tony
 */
#include <sstream>
#include <fstream>
 
#include "PDDLState.h"
#include "PDDLDomainFactory.h"

using namespace std;
namespace PDDL {

string PDDLState::toString() {
	ostringstream output;

	output << getLiteralString();
	output << getPNEString();
	return output.str();
}

std::string PDDLState::getObjectSymbolTableString() {
	ostringstream output;
	set<PDDLObject>::const_iterator objItr = objectSymbolTable.begin();
	const set<PDDLObject>::const_iterator objItrEnd = objectSymbolTable.end();
	for (; objItr != objItrEnd; objItr++) {
		output << "\t\t" << *objItr << std::endl;
	}
	return output.str();
}

std::string PDDLState::getLiteralString() {
	ostringstream output;
	// Literals to String
	list<Proposition>::const_iterator litItr = literals.begin();
	const list<Proposition>::const_iterator litItrEnd = literals.end();
	for (; litItr != litItrEnd; litItr++) {
		output << "\t\t" << *litItr << "\n";
	}
	return output.str();
}

std::string PDDLState::getPNEString() {
	ostringstream output;
	// PNEs to String
	list<PNE>::const_iterator pneItr = pnes.begin();
	const list<PNE>::const_iterator pneItrEnd = pnes.end();
	for (; pneItr != pneItrEnd; pneItr++) {
		output << "\t\t" << *pneItr << "\n";
	}
	return output.str();
}

string PDDLState::getPlanPrefixString() {
	ostringstream output;
	std::list<std::string>::const_iterator actionItr = planPrefix.begin();
	const std::list<std::string>::const_iterator actionItrEnd =
			planPrefix.end();
	for (; actionItr != actionItrEnd; actionItr++) {
		output << ";" << *actionItr << "\n";
	}
	return output.str();
}

std::string PDDLState::getGoalString() {
	ostringstream output;
	std::list<Proposition>::const_iterator goalItr = goals.begin();
	for (; goalItr != goals.end(); goalItr++) {
		output << *goalItr << " ";
	}
	return output.str();
}

std::string PDDLState::getTILGoalString() {
	ostringstream output;
	if (!tilAchievedPredicates.size()) {
		return output.str();
	}
	std::list<Proposition>::const_iterator tilItr = tilAchievedPredicates.begin();
	for (; tilItr != tilAchievedPredicates.end(); tilItr++) {
		output << "(" << tilItr->getName() << ") ";
	}
	return output.str();
}

void PDDLState::writeStateToFile(string filePath, string fileName) {
	ofstream myFile;
	ostringstream fullFilePath;
	fullFilePath << filePath << fileName << ".pddl";
	myFile.open(fullFilePath.str());
	myFile << ";time stamp: " << timestamp << endl;
	myFile << ";heuristic: " << heuristic << endl;
	myFile << ";plan prefix" << endl;
	myFile << getPlanPrefixString() << endl;
	myFile << "(define (problem " << fileName << ")" << endl;
	myFile << "\t(:domain " << VAL::current_analysis->the_domain->name << ")"
			<< std::endl;
	myFile << "\t(:objects" << std::endl << getObjectSymbolTableString()
			<< std::endl;
	myFile << "\t)" << endl;
	myFile << "\t(:init" << endl;
	myFile << toString();
	myFile << "\t)" << endl;
	myFile << "\t(:goal (and " << getGoalString() << "))" << endl;
	myFile << "\t(:metric " << metric << ")" << endl;
	myFile << ")";
	myFile.close();
}

void PDDLState::writeDeTILedStateToFile(std::string filePath,
		std::string fileName) {
	ofstream myFile;
	ostringstream fullFilePath;
	fullFilePath << filePath << fileName << ".pddl";
	myFile.open(fullFilePath.str());
	myFile << ";time stamp: " << timestamp << endl;
	myFile << ";heuristic: " << heuristic << endl;
	myFile << ";plan prefix" << endl;
	myFile << getPlanPrefixString() << endl;
	myFile << "(define (problem " << fileName << ")" << endl;
	myFile << "\t(:domain " << VAL::current_analysis->the_domain->name << ")"
			<< std::endl;
	myFile << "\t(:objects" << std::endl << getObjectSymbolTableString();
	myFile << "\t)" << endl;
	myFile << "\t(:init" << endl;
	myFile << getLiteralString() << getPNEString();
	myFile << "\t)" << endl;
	myFile << "\t(:goal (and " << getGoalString() << " " << getTILGoalString()
			<< "))" << endl;
	myFile << "\t(:metric " << metric << ")" << endl;
	myFile << ")";
	myFile.close();
}
}
