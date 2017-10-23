/*
 * TIL.h
 *
 *  Created on: 2 Jul 2015
 *      Author: tony
 */

#ifndef __PDDL_TIL
#define __PDDL_TIL

#include <list>
#include <set>

#include "PDDLObject.h"
#include "Proposition.h"

namespace PDDL {

class TIL {
private:
	std::list<Proposition> addEffects;
	std::list<Proposition> delEffects;
	double timestamp;
	std::set<PDDLObject> parameters;

public:
	TIL(std::list<Proposition> addEffects, std::list<Proposition> delEffects,
			double timestamp, std::set<PDDLObject> parameters) :
			addEffects(addEffects), delEffects(delEffects), timestamp(timestamp), parameters(parameters) {
	}
	;
	std::string getName() const;
	double getTimestamp() const { return timestamp; };
	const std::list<Proposition> & getAddEffects() const { return addEffects; };
	const std::list<Proposition> & getDelEffects() const { return delEffects; };
	const std::set<PDDLObject> & getParameters() const {return parameters; };
	friend std::ostream & operator<<(std::ostream & output, const TIL & til);
	static bool TILTimestampComparator(const TIL & first, const TIL & second);
	friend bool TILTimestampComparator(const TIL & first, const TIL & second);
};

}

#endif /* __PDDL_TIL */
