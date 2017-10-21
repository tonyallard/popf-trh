#ifndef __FFSOLVER
#define __FFSOLVER

#include "RPGBuilder.h"

#include "FFEvent.h"

#include "SearchQueueItem.h"
#include "ExtendedMinimalState.h"
#include "HTrio.h"
#include "StartEvent.h"

#include "pddl/PDDLState.h"
#include "pddl/PDDLStateFactory.h"

#include <ptree.h>

#include <map>
#include <list>

using std::map;
using std::list;

namespace Planner
{

class ParentData;

class FakeFFEvent : public FFEvent
{
private:
    StartEvent * toUpdate;
public:
    FakeFFEvent(StartEvent * const e, instantiatedOp * a, const int & pw, const double & dMin, const double &
                dMax)
            : FFEvent(a, pw, dMin, dMax), toUpdate(e) {
        lpMinTimestamp = e->lpMinTimestamp;
        lpMaxTimestamp = e->lpMaxTimestamp;
    };

    virtual ~FakeFFEvent() {
//      cout << "~FakeFFEvent, moving bounds of [" << lpMinTimestamp << ",";
//      if (lpMaxTimestamp == DBL_MAX) {
//          cout << "inf";
//      } else {
//          cout << lpMaxTimestamp;
//      }
//      cout << "] for end of " << *(action) << " back to SEQ entry\n";
        toUpdate->lpMinTimestamp = lpMinTimestamp;
        toUpdate->lpMaxTimestamp = lpMaxTimestamp;
    }

    virtual void passInMinMax(const double & a, const double & b) {
        toUpdate->lpMinTimestamp = lpMinTimestamp = a;
        toUpdate->lpMaxTimestamp = lpMaxTimestamp = b;
    }


};
/*
class ImplicitFFEvent : public FFEvent {
private:
    FFEvent * toUpdate;
public:
        ImplicitFFEvent(FFEvent * const e, instantiatedOp * a, const int & pw, const double & dMin, const double & dMax)
        : FFEvent(a,pw,dMin,dMax), toUpdate(e)
    {
        lpMinTimestamp = e->lpMinTimestamp + e->minDuration;
        lpMaxTimestamp = e->lpMaxTimestamp;
                if (lpMaxTimestamp != DBL_MAX) {
                    if (e->maxDuration == DBL_MAX) {
                        lpMaxTimestamp = DBL_MAX;
                    } else {
                        lpMaxTimestamp += e->maxDuration;
                    }
                }
    };

        void pushToStart();

        ~ImplicitFFEvent() {
//      cout << "~FakeFFEvent, moving bounds of [" << lpMinTimestamp << ",";
//      if (lpMaxTimestamp == DBL_MAX) {
//          cout << "inf";
//      } else {
//          cout << lpMaxTimestamp;
//      }
//      cout << "] for end of " << *(action) << " back to SEQ entry\n";
                pushToStart();
    }

    virtual void passInMinMax(const double & a, const double & b) {
        lpMinTimestamp = a;
        lpMaxTimestamp = b;
                pushToStart();
    }


};
*/

struct SecondaryExtendedStateEquality {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
};


struct WeakExtendedStateEquality {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
};

struct SecondaryExtendedStateLessThan {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
    bool operator()(const ExtendedMinimalState * const a, const ExtendedMinimalState * const b) const;
};


struct WeakExtendedStateLessThan {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
    bool operator()(const ExtendedMinimalState * const a, const ExtendedMinimalState * const b) const;
};

struct ExtendedStateLessThanOnPropositionsAndNonDominatedVariables {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
    bool operator()(const ExtendedMinimalState * const a, const ExtendedMinimalState * const b) const;
};

struct FullExtendedStateLessThan {
    bool operator()(const ExtendedMinimalState & a, const ExtendedMinimalState & b) const;
    bool operator()(const ExtendedMinimalState * const a, const ExtendedMinimalState * const b) const;
};

struct Solution {
  
    list<FFEvent> * plan;   
    TemporalConstraints * constraints;
    double quality;
    
    Solution();
    void update(const list<FFEvent> & newPlan, const TemporalConstraints * const newCons, const double & newMetric);
    
};

class StateHash;

class FF
{
public:
    static std::map<int, int> EHC_PERFORMANCE_HISTOGRAM;
    static bool USE_TRH;
    static int STATES_EVALUATED;
    static int STATES_EVALUATED_IN_HEURISTIC;
    static int DEAD_END_COUNT;
    static Solution workingBestSolution;
    static bool scheduleToMetric;
private:

    static bool skipRPG;

	static void incrementEHCPerformance(int EHCSearchStateCount);

    static HTrio calculateHeuristicAndCompressionSafeSchedule(ExtendedMinimalState & theState, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, list<ActionSegment> & helpfulActions, list<FFEvent> & header, list<FFEvent> & now, const int & stepID, PDDL::PDDLStateFactory pddlFactory, map<double, list<pair<int, int> > > * justApplied = 0, double tilFrom = 0.001);
    static HTrio calculateHeuristicAndSchedule(ExtendedMinimalState & theState, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, ParentData * const p, list<ActionSegment> & helpfulActions, list<FFEvent> & header, list<FFEvent> & now, const int & stepID, PDDL::PDDLStateFactory pddlFactory, bool considerCache = false, map<double, list<pair<int, int> > > * justApplied = 0, double tilFrom = 0.001);

    

    /** @brief Check that a successor state has made meaningful progress
     *
     * Actions, when applied, should at least change *something* from the parent state,
     * in a way that might be desirable: adding a new fact, improving the value of a
     * numeric variable (i.e. not making it provably worse); or starting an action
     * which, at the end, does one of these two things.  If this isn't the case,
     * then even if the state (or its partial-order annotations) are different, then
     * it's not meaningfully different.
     *
     * @param parent  The parent state
     * @param child   The successor state under scrutiny
     * @param theAction  The action applied to transform <code>parent</code> into <code>child</code>
     * @retval <code>true</code> if child is meaningfully different to parent
     */
    static bool stateHasProgressedBeyondItsParent(const ActionSegment & theAction, const ExtendedMinimalState & parent, const ExtendedMinimalState & child);    
    
    static void evaluateStateAndUpdatePlan(auto_ptr<SearchQueueItem> & succ, ExtendedMinimalState & state, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, ParentData * const incrementalData, list<ActionSegment> & helpfulActionsExport, const ActionSegment & actID, list<FFEvent> & header, PDDL::PDDLStateFactory pddlFactory);

//  static void justEvaluateNotReuse(auto_ptr<SearchQueueItem> & succ, RPGHeuristic* rpg, ExtendedMinimalState & state, ExtendedMinimalState * prevState, set<int> & goals, set<int> & goalFluents, list<ActionSegment> & helpfulActionsExport, list<FFEvent> & extraEvents, list<FFEvent> & header, HTrio & bestNodeLimitHeuristic, list<FFEvent> *& bestNodeLimitPlan, bool & bestNodeLimitGoal, bool & stagnant, map<double, list<pair<int,int> > > * justApplied, double tilFrom=0.001);


//  static bool checkTSTemporalSoundness(RPGHeuristic* const rpg, ExtendedMinimalState & theState, const int & theAction, const VAL::time_spec & ts, const double & incr, int oldTIL=-1);
    static bool precedingActions(ExtendedMinimalState & theState, const ActionSegment & actionSeg, list<ActionSegment> & alsoMustDo, int oldTIL = -1, double moveOn = 0.001);

    static bool checkTemporalSoundness(ExtendedMinimalState & theState, const ActionSegment & actionSeg, int oldTIL = -1, double moveOn = 0.001);

    static bool carryOnSearching(const MinimalState & theState,  const list<FFEvent> & plan);
    
public:

    static void printPlanAsDot(ostream & o, const list<FFEvent> & plan, const TemporalConstraints * cons);
    
    static bool steepestDescent;
    static bool bestFirstSearch;
    static bool helpfulActions;
    static bool pruneMemoised;
    static bool firstImprover;
    static bool incrementalExpansion;
    static bool skipEHC;
    static bool zealousEHC;
    static bool startsBeforeEnds;
    static bool invariantRPG;
    static bool tsChecking;
    static bool timeWAStar;
    static bool WAStar;
    static double doubleU;
    static bool biasG;
    static bool biasD;
    static bool makespanTieBreak;
    static bool planMustSucceed;
    static bool nonDeletorsFirst;
    static bool openListOrderLowMakespanFirst;
    static bool useDominanceConstraintsInStateHash;
    static bool allowCompressionSafeScheduler;
    
    //static list<instantiatedOp*> * solveSubproblem(LiteralSet & startingState, vector<pair<PNE*, double> > & startingFluents, SubProblem* const s);
    static Solution search(bool & reachedGoal);

    static list<FFEvent> * doBenchmark(bool & reachedGoal, list<FFEvent> * soln, const bool doLoops = true);
    static list<FFEvent> * reprocessPlan(list<FFEvent> * soln, TemporalConstraints * cons);
    static ExtendedMinimalState * applyActionToState(ActionSegment & theAction, const ExtendedMinimalState & parent);
    static double evaluateMetric(const MinimalState & theState, const list<FFEvent> & plan, const bool printMetric=true);
    static void makeJustApplied(map<double, list<pair<int, int> > > & justApplied, double & tilFrom, ExtendedMinimalState & state, const bool & lastIsSpecial);
    static StateHash* getStateHash();
};


};

#endif
