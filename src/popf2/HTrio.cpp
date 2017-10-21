/*
 * HTrio.cpp
 *
 *  Created on: 28 Apr 2015
 *      Author: tony
 */

#include "HTrio.h"
#include "FFSolver.h"

using namespace Planner;

HTrio::HTrio(const double & hvalue, const double & msIn, const double & mseIn, const int & planLength, const char *
#ifndef NDEBUG
	diagnosisIn
#endif
	)
	: heuristicValue(hvalue), makespan(msIn), makespanEstimate(mseIn)
#ifndef NDEBUG
	, diagnosis(diagnosisIn)
#endif
{
	if (FF::WAStar) {
		if (FF::biasD) {
			qbreak = planLength + 1;
		} else if (FF::biasG) {
			qbreak = heuristicValue;
		} else {
			qbreak = 0;
		}
	} else {
		qbreak = planLength + 1;
	}
}

HTrio::HTrio(const HTrio & h) : heuristicValue(h.heuristicValue), makespan(h.makespan), makespanEstimate(h.makespanEstimate), qbreak(h.qbreak)
#ifndef NDEBUG
	, diagnosis(h.diagnosis)
#endif
{};

HTrio & HTrio::operator =(const HTrio & h) {
	heuristicValue = h.heuristicValue;
	makespan = h.makespan;
	makespanEstimate = h.makespanEstimate;
	qbreak = h.qbreak;
#ifndef NDEBUG
	diagnosis = h.diagnosis;
#endif
	return *this;
}

bool HTrio::operator<(const HTrio & other) const {
	if (qbreak < other.qbreak) return true;
	if (qbreak > other.qbreak) return false;

	if (!FF::makespanTieBreak) return false;

	if ((makespan - other.makespan) < -0.0001) return true;
	if ((makespan - other.makespan) > 0.0001) return false;

//            if ((makespanEstimate - other.makespanEstimate) < -0.0001) return true;
//            if ((makespanEstimate - other.makespanEstimate) > 0.0001) return false;


	return false;
}