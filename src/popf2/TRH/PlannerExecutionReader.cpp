#include <sstream>

#include "PlannerExecutionReader.h"

#include "../pddl/PDDLUtils.h"

#include "../RPGBuilder.h"

namespace TRH {

const string PlannerExecutionReader::RELAXED_PLAN_SIZE_DELIM = "Relaxed plan length is: ";
const string PlannerExecutionReader::H_STATES_EVAL_DELIM = "; States evaluated: ";
const string PlannerExecutionReader::H_DEAD_ENDS_DELIM = "#; Dead Ends encountered: ";
const string PlannerExecutionReader::SOLUTION_FOUND = ";;;; Solution Found";

const string PlannerExecutionReader::H_PLAN_DELIM_START = "=====Plan Start====="; 
const string PlannerExecutionReader::H_PLAN_DELIM_STOP = "=====Plan Stop=====";

PlannerExecutionReader::PlannerExecutionReader(string plannerOutput, 
	const std::list<PDDL::TIL> & tils,
	const Planner::MinimalState & state, double timeStamp) {

	statesEvaluatedInHeuristic = getHeuristicStatesEvaluated(plannerOutput);
	deadEndsEncounteredInHeuristic = getDeadEndsEncountered(plannerOutput);
	solutionFound = getIsSolutionFound(plannerOutput);
	relaxedPlanLength = 0;
	if (solutionFound) {
		list<string> relaxedPlanStr = getRelaxedPlanStr(plannerOutput);
		relaxedPlan = getRelaxedPlan(relaxedPlanStr, tils);
		if (Planner::FF::helpfulActions) {
			helpfulActions = getHelpfulActions(relaxedPlan, state, timeStamp);
		}
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

int PlannerExecutionReader::getDeadEndsEncountered(const string & plannerOutput) {
	int pos = plannerOutput.find(H_DEAD_ENDS_DELIM);
	if (pos != -1) {
		int posEnd = plannerOutput.find("\n", pos);
		string deadEndsStr = plannerOutput.substr(pos + H_DEAD_ENDS_DELIM.size(), posEnd-(pos + H_DEAD_ENDS_DELIM.size()));
		int deadEnds = stoi(deadEndsStr);
		return deadEnds;
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

list<Planner::ActionSegment> PlannerExecutionReader::getHelpfulActions(
	const list<Planner::FFEvent> & plan,
	const Planner::MinimalState & state,
	double timeStamp) {
	list<Planner::ActionSegment> helpfulActions;
	// cout << "Helpful Actions" << endl;
	
	//Add end of currently executing action if applicable
	std::map<int, std::set<int> >::const_iterator saItr =
			state.startedActions.begin();
	const std::map<int, std::set<int> >::const_iterator saItrEnd =
			state.startedActions.end();

	for (; saItr != saItrEnd; saItr++) {
		Inst::instantiatedOp* action = Planner::RPGBuilder::getInstantiatedOp(
			saItr->first);
		Planner::ActionSegment act(action, 
			VAL::time_spec::E_AT_END, 
			state.nextTIL, Planner::RPGHeuristic::emptyIntList);
		if (Planner::RPGBuilder::getHeuristic()->testApplicability(state, timeStamp, act, false, false)) {
			helpfulActions.push_back(act);
		}
	}

	//Add other actions in the relaxed plan if applicable
	list<Planner::FFEvent>::const_iterator planItr = plan.begin();
	for (; planItr != plan.end(); planItr++) {
		// cout << PDDL::getActionName(&(*planItr)) << endl;
		Planner::ActionSegment act(planItr->action, planItr->time_spec, 
			state.nextTIL, Planner::RPGHeuristic::emptyIntList);
		if (Planner::RPGBuilder::getHeuristic()->testApplicability(state, timeStamp, act, false, false)) {
			helpfulActions.push_back(act);
		}
	}
	return helpfulActions;
}


list<Planner::FFEvent> PlannerExecutionReader::getRelaxedPlan(list<string> planStr, 
	const std::list<PDDL::TIL> & tils) {
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
			relaxedPlanLength++;
			if (end_snap_action_defined) {
				rPlan.push_back(end_snap_action);
				relaxedPlanLength++;
			}
		} else { //Check if it is a TIL
			std::list<PDDL::TIL>::const_iterator tilItr = tils.begin();
			bool found = false;
			for (; tilItr != tils.end(); tilItr++) {
				//case insensitive find
				string tilName = tilItr->getName();
				transform(tilName.begin(), tilName.end(), tilName.begin(), ::toupper);
				transform(actionStr.begin(), actionStr.end(), actionStr.begin(), ::toupper);
				if (actionStr.find(tilName) != std::string::npos) {
					Planner::FFEvent til_action(tilItr->getTILIndex());
					til_action.lpTimestamp = startTime;
					rPlan.push_back(til_action);
					relaxedPlanLength++;
					found = true;
					break;
				}
			}
			//else probably a partial action: add 1
			if (!found) {
				relaxedPlanLength++;
				// cout << actionStr << " not Found." << endl;
			}
		}	
	}
	return rPlan;
}

}