/*
 * Metric.cpp
 *
 *  Created on: 20 Dec 2015
 *      Author: tony
 */
#include <iostream>

#include "Metric.h"
using namespace std;

namespace PDDL {

std::ostream & operator<<(std::ostream & output, const Metric & metric) {
	//If there is no metric return
	if (!metric.variables.size()) {
		return output;
	}
	if (metric.minimise) {
		output << "minimize ";
	} else {
		output << "maximize ";
	}
	std::list<std::string>::const_iterator varItr = metric.variables.begin();
	for (; varItr != metric.variables.end(); varItr++) {
		output << *varItr << " ";
	}
	return output;
}

}
