/*
 * HTrio.h
 *
 *  Created on: 28 Apr 2015
 *      Author: tony
 */

#ifndef COLIN_HTRIO_H_
#define COLIN_HTRIO_H_

namespace Planner {

class HTrio
	{

	public:

		double heuristicValue;
		double makespan;
		double makespanEstimate;
		double qbreak;

#ifndef NDEBUG
		const char * diagnosis;
#endif

		HTrio() {};
		HTrio(const double & hvalue, const double & msIn, const double & mseIn, const int & planLength, const char *
		#ifndef NDEBUG
			diagnosisIn
		#endif
	);
		HTrio(const HTrio & h);
		HTrio & operator =(const HTrio & h);
		bool operator<(const HTrio & other) const;

	};

}
#endif /* COLIN_HTRIO_H_ */
