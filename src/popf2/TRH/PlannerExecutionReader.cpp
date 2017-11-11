#include <sstream>

#include "PlannerExecutionReader.h"

#include "../pddl/PDDLUtils.h"

namespace TRH {

const string PlannerExecutionReader::RELAXED_PLAN_SIZE_DELIM = "Relaxed plan length is: ";
const string PlannerExecutionReader::H_STATES_EVAL_DELIM = "; States evaluated: ";
const string PlannerExecutionReader::SOLUTION_FOUND = ";;;; Solution Found";

const string PlannerExecutionReader::H_PLAN_DELIM_START = "=====Plan Start====="; 
const string PlannerExecutionReader::H_PLAN_DELIM_STOP = "=====Plan Stop=====";

PlannerExecutionReader::PlannerExecutionReader(string plannerOutput, 
	std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> tilMap) {

	statesEvaluatedInHeuristic = getHeuristicStatesEvaluated(plannerOutput);
	relaxedPlanSize = getRelaxedPLanLength(plannerOutput);
	solutionFound = getIsSolutionFound(plannerOutput);
	if (relaxedPlanSize > 0) {
		list<string> relaxedPlanStr = getRelaxedPlanStr(plannerOutput);
		relaxedPlan = getRelaxedPlan(relaxedPlanStr, tilMap);
		helpfulActions = getHelpfulActions(relaxedPlanStr);
	}
}

bool PlannerExecutionReader::getIsSolutionFound(const string & plannerOutput) {
	if (plannerOutput.find(SOLUTION_FOUND) != string::npos) {
		return true;
	}
	return false;
}

int PlannerExecutionReader::getHeuristicStatesEvaluated(const string & plannerOutput) {
	int pos = plannerOutput.find(H_STATES_EVAL_DELIM);
	if (pos != -1) {
		int posEnd = plannerOutput.find("\n", pos);
		string statesEvalStr = plannerOutput.substr(pos + H_STATES_EVAL_DELIM.size(), posEnd-(pos + H_STATES_EVAL_DELIM.size()));
		int statesEval = stoi(statesEvalStr);
		return statesEval;
	}
	return -1;
}

int PlannerExecutionReader::getRelaxedPLanLength(const string & plannerOutput) {
	int pos = plannerOutput.find(RELAXED_PLAN_SIZE_DELIM);
	if (pos != -1) {
		int posEnd = plannerOutput.find("\n", pos);
		string relaxedPlanSizeStr = plannerOutput.substr(pos + RELAXED_PLAN_SIZE_DELIM.size(), 
			posEnd-(pos + RELAXED_PLAN_SIZE_DELIM.size()));
		int relaxedPlanSize = stoi(relaxedPlanSizeStr);
		return relaxedPlanSize;
	}
	return -1;
}

list<string> PlannerExecutionReader::getRelaxedPlanStr(const string & output) {
	list<string> relaxedPlanStr;

	int startPos = output.find(H_PLAN_DELIM_START);
	int endPos = output.find(H_PLAN_DELIM_STOP);
	string planSection = output.substr(startPos + H_PLAN_DELIM_START.size(), 
		endPos-(startPos + H_PLAN_DELIM_START.size()));
	// cout << "Relaxed Plan" << endl;
	// cout << planSection << endl;
	//Iterate through actions
	std::istringstream iss(planSection);
	for (std::string line; std::getline(iss, line); ){
		//Guaranteed to be a double if this is an action
		string firstFive = line.substr(0, 5);
		if (Util::isDouble(firstFive.c_str())) {
			// cout << line << endl;
			relaxedPlanStr.push_back(line);
		}
	}
	return relaxedPlanStr;
}

list<Planner::ActionSegment> PlannerExecutionReader::getHelpfulActions(list<string> planStr) {
	list<Planner::ActionSegment> rPlan;
	// cout << "Helpful Actions" << endl;
	list<string>::const_iterator planStrItr = planStr.begin();
	for (; planStrItr != planStr.end(); planStrItr++) {
		string actionStr = *planStrItr;
		// cout << actionStr << endl;
		int actionNameStartPos = actionStr.find("(");
		int actionNameEndPos = actionStr.find(")") + 1;
		string actionInstance = actionStr.substr(actionNameStartPos, 
			actionNameEndPos - actionNameStartPos);
		Inst::instantiatedOp * op = PDDL::getOperator(actionInstance);

		if (op != 0) {
			Planner::ActionSegment start_snap_action = Planner::ActionSegment(op, 
				VAL::E_AT_START, 0, Planner::RPGHeuristic::emptyIntList);
			rPlan.push_back(start_snap_action);
			if (!Planner::RPGBuilder::getRPGDEs(op->getID()).empty()) {
				//Durative Action
				Planner::ActionSegment end_snap_action = Planner::ActionSegment(op, 
					VAL::E_AT_END, 0, Planner::RPGHeuristic::emptyIntList);
				rPlan.push_back(end_snap_action);
			}
		}
	}
	return rPlan;
}

list<Planner::FFEvent> PlannerExecutionReader::getRelaxedPlan(list<string> planStr, 
	std::map<PDDL::TIL, const Planner::RPGBuilder::FakeTILAction *> tilMap) {
	list<Planner::FFEvent> rPlan;

	list<string>::const_iterator planStrItr = planStr.begin();
	for (; planStrItr != planStr.end(); planStrItr++) {
		string actionStr = *planStrItr;
		int actionNameStartPos = actionStr.find("(");
		int actionNameEndPos = actionStr.find(")") + 1;
		string actionInstance = actionStr.substr(actionNameStartPos, 
			actionNameEndPos - actionNameStartPos);
		string actionName = actionInstance.substr(1, actionInstance.find(" ") - 1);
		Inst::instantiatedOp * op = PDDL::getOperator(actionInstance);
		
		int startTimePos = actionStr.find(":");
		double startTime = stod(actionStr.substr(0, startTimePos));

		int actionDurationStartPos = actionStr.find("[") + 1;
		int actionDurationEndPos = actionStr.find("]");
		double duration = stod(actionStr.substr(actionDurationStartPos, 
			actionDurationEndPos - actionDurationStartPos));

		if (op != 0) {
			bool end_snap_action_defined = false;
			Planner::FFEvent end_snap_action;

			Planner::FFEvent start_snap_action = Planner::FFEvent(op, 
				startTime, startTime);
			start_snap_action.lpTimestamp = startTime;

			if (!Planner::RPGBuilder::getRPGDEs(op->getID()).empty()) {
				//Durative Action
				end_snap_action_defined = true;
				double durActEndStart = startTime + duration;
				end_snap_action = Planner::FFEvent(op, 
					rPlan.size() - 1, durActEndStart, durActEndStart);
				end_snap_action.lpTimestamp = durActEndStart;
				end_snap_action.pairWithStep = rPlan.size();
				start_snap_action.pairWithStep = end_snap_action.pairWithStep + 1;
			}
			rPlan.push_back(start_snap_action);
			if (end_snap_action_defined) {
				rPlan.push_back(end_snap_action);
			}
		} else { //Check if it is a TIL
			std::map<PDDL::TIL, 
				const Planner::RPGBuilder::FakeTILAction *>::const_iterator tilItr = tilMap.begin();
			for (; tilItr != tilMap.end(); tilItr++) {
				//case insensitive find
				string tilName = tilItr->first.getName();
				transform(tilName.begin(), tilName.end(), tilName.begin(), ::toupper);
				transform(actionStr.begin(), actionStr.end(), actionStr.begin(), ::toupper);
				if (actionStr.find(tilName) != std::string::npos) {
					Planner::FFEvent til_action(tilItr->first.getTILIndex());
					til_action.lpTimestamp = startTime;
					rPlan.push_back(til_action);

				}
				//else probably a partial action: ignore
			}
		}	
	}
	return rPlan;
}

}