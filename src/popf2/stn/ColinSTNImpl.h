/*
 * STNImpl.h
 *
 *  Created on: 31 Aug 2015
 *      Author: tony
 */

#ifndef __STN_COLINSTNIMPL_H_
#define __STN_COLINSTNIMPL_H_

#include "STN.h"
#include "../FFEvent.h"
#include "../globals.h"

namespace stn {

class ColinSTNImpl : public STN<const Planner::FFEvent *, double> {

private:
	std::string toMatrixString(std::vector<std::vector<double> > matrix);

public:
	const static double MAX_ACTION_SEPARATION;
	const static double MIN_ACTION_SEPARATION;
	const static double ACCURACY;

	ColinSTNImpl() {};
	double getEdge(const Planner::FFEvent * first, const Planner::FFEvent * third);
	void removeAllEdges(set<const Util::triple<const Planner::FFEvent *, 
		double> *> toRemove);
	std::vector<std::vector<double> > toMatrix();
	std::string toMatrixString();
	std::string getMatixLegendStr();
	bool isConsistent ();
	bool FIFOLabelCorrectingAlgorithm();
	std::vector<const Planner::FFEvent *> getSuccessors(const Planner::FFEvent * node);
	std::string getD3Output() const;
	
	static ColinSTNImpl makeColinSTN(const std::list<Planner::FFEvent> & plan,
			std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > ordering);

	friend std::ostream & operator<<(std::ostream & output, const ColinSTNImpl & colinSTN);

};

	std::ostream & operator<<(std::ostream & output,
				const Util::triple<const Planner::FFEvent *, double> & edge);

}



#endif /* __STN_COLINSTNIMPL_H_ */
