/*
 * StartEvent.h
 *
 *  Created on: 28 Apr 2015
 *      Author: tony
 */

#ifndef COLIN_STARTEVENT_H_
#define COLIN_STARTEVENT_H_

#include <set>
#include <values.h>
#include <assert.h>
#include <math.h>

using std::set;

namespace Planner
{

struct StartEvent {
    int actID;
    int divisionsApplied;
    int stepID;
    double advancingDuration;
    double minDuration;
    double maxDuration;
    double elapsed;
    double minAdvance;
//  ScheduleNode* compulsaryEnd;
    bool terminated;
    bool ignore;
    int fanIn;
    set<int> endComesBefore;
    set<int> endComesBeforePair;

    set<int> endComesAfter;
    set<int> endComesAfterPair;


    inline set<int> & getEndComesBefore() {
        return endComesBefore;
    };
    inline set<int> & getEndComesAfter() {
        return endComesAfter;
    };
    inline set<int> & getEndComesAfterPair() {
        return endComesAfterPair;
    };
    inline set<int> & getEndComesBeforePair() {
        return endComesBeforePair;
    };


    double lpMinTimestamp;
    double lpMaxTimestamp;


    StartEvent(const int & a, const int & da, const int & s, const double & mind, const double & maxd, const double &e) : actID(a), divisionsApplied(da), stepID(s), advancingDuration(mind), minDuration(mind), maxDuration(maxd), elapsed(e), minAdvance(DBL_MAX), terminated(false), ignore(false), fanIn(0), lpMinTimestamp(0.0), lpMaxTimestamp(DBL_MAX) {};

    bool operator ==(const StartEvent & e) const {
        return (actID == e.actID &&
                divisionsApplied == e.divisionsApplied &&
                stepID == e.stepID &&
                fabs(minDuration - e.minDuration) < 0.0005 &&
                fabs(maxDuration - e.maxDuration) < 0.0005 &&
                fabs(elapsed - e.elapsed) < 0.0005 &&
                fabs(advancingDuration - e.advancingDuration) < 0.0005 &&
//          compulsaryEnd == e.compulsaryEnd &&
                terminated == e.terminated &&
                fanIn == e.fanIn &&
                endComesBefore == e.endComesBefore);
    }

    void endMustComeAfter(const int & i) {
        assert(i >= 0); endComesAfter.insert(i);
    }
    void endMustComeAfterPair(const int & i) {
        assert(i >= 0); endComesAfterPair.insert(i);
    }

    void actionHasFinished(const int & i) {
        assert(i >= 0); endComesAfter.erase(i);
    }

};
}

#endif /* COLIN_STARTEVENT_H_ */
