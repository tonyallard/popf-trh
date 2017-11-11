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
	int tilIndex;
	std::list<Proposition> addEffects;
	std::list<Proposition> delEffects;
	std::set<PDDLObject> parameters;

public:
	TIL(int tilIndex, std::list<Proposition> addEffects, 
			std::list<Proposition> delEffects,
			std::set<PDDLObject> parameters) :
			tilIndex(tilIndex), addEffects(addEffects), 
			delEffects(delEffects), parameters(parameters) {
	}
	;

	TIL(TIL const & other) : tilIndex(other.tilIndex), addEffects(other.addEffects), 
		delEffects(other.delEffects),
		parameters(other.parameters) {
	};

	TIL operator=(TIL const & other) {
		return TIL(other);
	};

	std::string getName() const;
	inline int getTILIndex() const { return tilIndex; };
	const std::list<Proposition> & getAddEffects() const { return addEffects; };
	const std::list<Proposition> & getDelEffects() const { return delEffects; };
	const std::set<PDDLObject> & getParameters() const {return parameters; };
	friend std::ostream & operator<<(std::ostream & output, const TIL & til);
	static bool TILTimestampComparator(const TIL & first, const TIL & second);
	friend bool TILTimestampComparator(const TIL & first, const TIL & second);

	friend bool operator<(const TIL & lhs, const TIL & rhs);

};

}

#endif /* __PDDL_TIL */
