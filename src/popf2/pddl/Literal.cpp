/*
 * Literal.cpp
 *
 *  Created on: 30 Aug 2015
 *      Author: tony
 */

#include "Literal.h"

namespace PDDL {

Literal::Literal(std::string name, std::list<std::string> arguments, bool positive) {
	proposition = Proposition(name, arguments);
	this->positive = positive;
}

std::ostream & operator<<(std::ostream & output, const Literal & literal) {
	if (!literal.positive) {
		output << "(not ";
	}
	output << literal.proposition;
	if (!literal.positive) {
		output << ")";
	}
	return output;
}

bool Literal::operator==(const Literal & other) {
	if (this->proposition != other.proposition) {
		return false;
	}
	if (this->isPositive() != other.isPositive()){
		return false;
	}
	return true;
}

bool Literal::operator!=(const Literal & other) {
	return !((*this) == other);
}

bool Literal::operator<(const Literal & other) const{
	return ((proposition < other.proposition) ||
		(!(other.proposition < proposition) && 
			(positive < other.positive)));
}

bool Literal::operator>(const Literal & other) const{
	return other < (*this);
}

bool Literal::operator<=(const Literal & other) const{
	return !(other < (*this));
}

bool Literal::operator>=(const Literal & other) const{
	return !((*this) < other);
}

}

