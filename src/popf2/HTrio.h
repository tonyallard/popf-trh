/*
 * HTrio.h
 *
 *  Created on: 28 Apr 2015
 *      Author: tony
 */

#ifndef __HTRIO
#define __HTRIO

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
#endif /* __HTRIO */
