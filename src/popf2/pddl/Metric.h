/*
 * Metric.h
 *
 *  Created on: 20 Dec 2015
 *      Author: tony
 */

#ifndef COLIN_PDDL_METRIC_H_
#define COLIN_PDDL_METRIC_H_

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


#endif /* COLIN_PDDL_METRIC_H_ */
