/*
 * TemporalConflictRelaxation.cpp
 *
 *  Created on: 22 Sep 2015
 *      Author: tony
 */

#include "TemporalConflictRelaxation.h"
#include "../globals.h"
 #include "../solver-clp.h"
 #include <typeinfo>

using namespace hRelax;
using namespace std;

const double TemporalConflictRelaxation::INF_UPPER_BOUND = LPinfinity;
const double TemporalConflictRelaxation::UNITY_COEFFICIENT = 1.0;

/**
 * Returns the minimum relaxations for a set of temporal constraints
 * Then returns those constraints (unmodified) with the new relaxed constraint value.
 */
std::set<
		std::pair<const Util::triple<const Planner::FFEvent *, double> *, double> > TemporalConflictRelaxation::determineTemporalRelaxations(
		std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
		const Planner::FFEvent * initialEvent) {

	//Determine the TIL constraints
	set<const Util::triple<const Planner::FFEvent *, double> *> tilConstraints =
			getTILConstraints(conflicts, initialEvent);

	//Get relaxable constraints. These are the decision vars
	addRelaxableConstraints(conflicts, tilConstraints);

	//Add new decision variables and constraints
	addDecisionVarsToObjectiveFunction(conflicts);
	addSTNConsistencyConstraint(conflicts);
	map<int, double> solution = solve();
	
	//Link relaxations to relaxable constraint
	std::set<std::pair<const Util::triple<const Planner::FFEvent *, double> *,
					double> > relaxations;
	map<const Util::triple<const Planner::FFEvent *, double> *, int>::const_iterator mapItr
		= conflictToMatrixIdxMap.begin();
	for (; mapItr != conflictToMatrixIdxMap.end(); mapItr++) {
		const Util::triple<const Planner::FFEvent *, double> * conflict =
				mapItr->first;
		if (relaxableConstraints.find(conflict) != relaxableConstraints.end()) {
			relaxations.insert(std::pair<
					const Util::triple<const Planner::FFEvent *, double> *,
					double>(conflict,
					solution[conflictToMatrixIdxMap[conflict]]));
		}
	}
	return relaxations;
}

map<int, double> TemporalConflictRelaxation::solve() {
	//Build MILP Model
	// cout << "Creating the CLP" << endl;
	MILPSolver * model = new MILPSolverCLP();
	// cout << "CLP has been created" << endl;
	int objFuncSize = decisionVars.size();
	model->addEmptyRealCols(objFuncSize);
	// cout << "writing vars" << endl;
	//Build Objective Function
	vector<Util::triple<pair<int,double>, double, double> >::const_iterator objItr = 
		decisionVars.begin();
	for (; objItr != decisionVars.end(); objItr++) {
		Util::triple<pair<int,double>, double, double> decisionVar 
			= *objItr;
		model->setObjCoeff(decisionVar.first.first, decisionVar.first.second);
		model->setColBounds(decisionVar.first.first, decisionVar.second, decisionVar.third);
	}

	vector<vector<pair<int,double> > >::const_iterator rowItr 
		= constraintRows.begin();
	for (; rowItr != constraintRows.end(); rowItr++) {
		vector<pair<int,double> > constraintRow = *rowItr;
		model->addRow(constraintRow, 0.0, INF_UPPER_BOUND);
	}
	
	//Minimise the objective function
	model->setMaximiseObjective(false);
	model->hush();
	//Solve 
	model->solve(false);
	// cout << "We Finished, size: " << objFuncSize << endl;
	//Store Solutions
	map<int, double> solution;
	for (int i = 0; i < objFuncSize; i++) {
		// cout << i << ", " << model->getSolution()[i] << endl;
		double soln = model->getSingleSolutionVariableValue(i);
		solution.insert(pair<int, double>(i, soln));
	}
	delete model;
	model = 0;
	// cout << "model deleted" << endl;
	return solution;
}

/*
 * 
 */
void TemporalConflictRelaxation::addDecisionVarsToObjectiveFunction(
	std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts) {
	//Determine how many decision vars there already is
	//This is to add to the correct array position
	const int objFuncSize = decisionVars.size();
	int matrixIdx = objFuncSize;
	std::set<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator confItr =
			conflicts.begin();
	for (; confItr != conflicts.end(); confItr++) {
		const Util::triple<const Planner::FFEvent *, double> * conflict =
				*confItr;
		//If its already part of the objective
		//function, don't re-add it.
		if (constraintExistsInSolver(conflict)) {
			continue;
		}
		//Now we need to add a new decision var
		//First add it to the matrix idx to conflict map
		pair<const Util::triple<const Planner::FFEvent *, double> *, int> matrixMap(
				conflict, matrixIdx);
		conflictToMatrixIdxMap.insert(matrixMap);
		
		pair<int,double> decisionVarCoeff(matrixIdx, UNITY_COEFFICIENT);
		Util::triple<pair<int,double>, double, double> decisionVar;

		if (relaxableConstraints.find(conflict) != relaxableConstraints.end()) {
			//This is relaxable create the appropriate bounds
			decisionVar.make_triple(decisionVarCoeff, conflict->second, INF_UPPER_BOUND);
		
		} else {
			//The bounds should ensure this value does not change
			decisionVar.make_triple(decisionVarCoeff, conflict->second, conflict->second);
		}
		decisionVars.push_back(decisionVar);
		matrixIdx++;
	}
}

/*
 * Create constraint
 * Everything involved in the loop should
 * Sum to something zero or greater for the STN
 * to be consistent
 */
void TemporalConflictRelaxation::addSTNConsistencyConstraint(
			std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts) {
	std::vector<std::pair<int, double> > constraintRow;
	std::set<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator confItr
		= conflicts.begin();
	for (; confItr != conflicts.end(); confItr++) {
		const Util::triple<const Planner::FFEvent *, double> * conflict
			= *confItr;
		int idx = conflictToMatrixIdxMap[conflict];
		//Add decision coefficients for constraints later
		constraintRow.push_back(std::pair<int, double>(idx, UNITY_COEFFICIENT));
	}
	constraintRows.push_back(constraintRow);
}

/* 
 * Checks whether the constraint exists in the solver.
 * This is done via a Constraint to Idx Map and requires this to be correct
 */
bool TemporalConflictRelaxation::constraintExistsInSolver(
	const Util::triple<const Planner::FFEvent *, double> * constraint) {
	return conflictToMatrixIdxMap.find(constraint) != conflictToMatrixIdxMap.end();
}

/**
 * Finds the til constraint by looking for the conflict with a greater than zero link
 * As the others should be negative to create the cycle
 */
set<const Util::triple<const Planner::FFEvent *, double> *> TemporalConflictRelaxation::getTILConstraints(
		const std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
		const Planner::FFEvent * initialEvent) {

	set<const Util::triple<const Planner::FFEvent *, double> *> tilConstraints;

	std::set<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator conflictItr =
			conflicts.begin();

	for (; conflictItr != conflicts.end(); conflictItr++) {
		const Util::triple<const Planner::FFEvent *, double> * conflict =
				*conflictItr;
		//Check if this is the TIL
		if (((conflict->first == initialEvent)
				&& (conflict->third->time_spec == VAL::time_spec::E_AT))
				|| ((conflict->third == initialEvent)
						&& (conflict->first->time_spec == VAL::time_spec::E_AT))) {
			tilConstraints.insert(conflict);
		}
	}
	return tilConstraints;
}

/*
 * Add to the list any constraint that is relaxable
 */
void TemporalConflictRelaxation::addRelaxableConstraints(
		const std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
		const std::set<const Util::triple<const Planner::FFEvent *, double> *> & tilConstraints) {

	std::set<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator conflictItr =
			conflicts.begin();
	for (; conflictItr != conflicts.end(); conflictItr++) {
		const Util::triple<const Planner::FFEvent *, double> * conflict =
				*conflictItr;
		//If the edge is not a TIL constraint, it is not a duration constraint
		//and it is not the link between a TIL and the next action
		//then it is relaxable
		if ((tilConstraints.find(conflict) == tilConstraints.end()) //Ignore til constraints
				&& (conflict->first->action != conflict->third->action) //Ignore duration constraints
				&& (conflict->first->time_spec != VAL::time_spec::E_AT)) { //Ignore precedence between a TIL and the next action
			relaxableConstraints.insert(conflict);
		}
	}
}