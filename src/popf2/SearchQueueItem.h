/*
 * SearchQueueItem.h
 *
 *  Created on: 28 Apr 2015
 *      Author: tony
 */

#ifndef COLIN_SEARCHQUEUEITEM_H_
#define COLIN_SEARCHQUEUEITEM_H_

#include <list>

#include "ExtendedMinimalState.h"
#include "FFEvent.h"
#include "HTrio.h"

namespace Planner {

class SearchQueueItem
{

private:
    ExtendedMinimalState * internalState;
    bool ownState;

public:
#ifdef STATEHASHDEBUG
    bool mustNotDeleteState;
#endif


    inline ExtendedMinimalState * state() {
        return internalState;
    }

    list<FFEvent> plan;

    list<ActionSegment> helpfulActions;
    HTrio heuristicValue;

    SearchQueueItem()
            : internalState(0), ownState(false) {
#ifdef STATEHASHDEBUG
        mustNotDeleteState = false;
#endif
    }



    /**
     *  Create a search queue item for the specified state.
     *
     *  @param sIn              The state to store in the search queue item
     *  @param clearIfDeleted   If <code>true</code>, mark that <code>sIn</code> should be deleted
     *                          if the search queue item is deleted (unless <code>releaseState()</code>
     *                          is called first).
     */
    SearchQueueItem(ExtendedMinimalState * const sIn, const bool clearIfDeleted)
            : internalState(sIn), ownState(clearIfDeleted) {
#ifdef STATEHASHDEBUG
        mustNotDeleteState = false;
#endif
    }

    ~SearchQueueItem() {
        if (ownState) {
#ifdef STATEHASHDEBUG
            assert(!mustNotDeleteState);
#endif
            delete internalState;
        }
    }


    /**
     *  Return the state held in this search queue item, flagging that it should not be deleted by
     *  the search queue item's destructor.
     *
     *  @return The state held in this search queue item
     */
    ExtendedMinimalState * releaseState() {
        assert(ownState);
        ownState = false;
        return internalState;
    }


    void printPlan() {
        if (Globals::globalVerbosity & 2) {
            list<FFEvent>::iterator planItr = plan.begin();
            const list<FFEvent>::iterator planEnd = plan.end();

            for (int i = 0; planItr != planEnd; ++planItr, ++i) {
                if (!planItr->getEffects) cout << "(( ";
                if (planItr->action) {
                    cout << i << ": " << *(planItr->action) << ", " << (planItr->time_spec == VAL::E_AT_START ? "start" : "end");
                } else if (planItr->time_spec == VAL::E_AT) {
                    cout << i << ": TIL " << planItr->divisionID;

                } else {
                    cout << i << ": null node!";
                    assert(false);
                }
                if (!planItr->getEffects) cout << " ))";
                cout << " at " << planItr->lpMinTimestamp;
                cout << "\n";
            }
        }
    }

};

}

#endif /* COLIN_SEARCHQUEUEITEM_H_ */
