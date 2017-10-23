/*
 * Metric.h
 *
 *  Created on: 20 Dec 2015
 *      Author: tony
 */

#ifndef __PDDL_METRIC
#define __PDDL_METRIC

#include <iostream>
#include <string>
#include <list>

namespace PDDL {
class Metric {

private:
	bool minimise;
	std::list<std::string> variables;

public:
	Metric(){};
	Metric(bool minimise, std::list<std::string> variables):minimise(minimise), variables(variables){};
	friend std::ostream & operator<<(std::ostream & output, const Metric & metric);
};
}


#endif /* __PDDL_METRIC */
