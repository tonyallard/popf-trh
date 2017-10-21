/*
 * Literal.h
 *
 *  Created on: 30 Aug 2015
 *      Author: tony
 */

#ifndef COLIN_PDDL_LITERAL_H_
#define COLIN_PDDL_LITERAL_H_

#include <string>
#include <iostream>

#include "Proposition.h"

namespace PDDL {

class Literal {
private:
	bool positive;
	Proposition proposition;

public:
	Literal(PDDL::Proposition proposition, bool positive) : proposition(proposition), positive(positive) {};
	Literal(std::string name, std::list<std::string> arguments, bool positive);
	Literal(){};
	inline bool isPositive() const { return positive; };
	inline bool isNegative() const { return !positive; };
	inline const Proposition * getProposition() const { return &proposition; };
	bool operator==(const Literal & other);
	bool operator!=(const Literal & other);
	bool operator<(const Literal & other) const;
	bool operator>(const Literal & other) const;
	bool operator<=(const Literal & other) const;
	bool operator>=(const Literal & other) const;
	friend std::ostream & operator<<(std::ostream & output,
			const Literal & literal);
};

}



#endif /* COLIN_PDDL_LITERAL_H_ */
