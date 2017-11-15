/*
 * ITC.h
 *
 *  Created on: 13 Sep 2015
 *      Author: tony
 */

#ifndef __ITC_ITC_H_
#define __ITC_ITC_H_

#include <map>
#include <vector>
#include <set>

#include "../stn/ColinSTNImpl.h"

namespace ITC {

class ITC {

private:
	static ITC * INSTANCE;
	//Singleton
	ITC() {
	}
	;
	ITC(ITC const & other) {
	}
	;
	ITC& operator=(ITC const&) {
	}
	;

	//Label Correcting
	void initialiseLC(stn::ColinSTNImpl * network,
			const Planner::FFEvent * start,
			std::map<const Planner::FFEvent *, double> & dist);
	std::set<const Util::triple<const Planner::FFEvent *, double> *> extractConflictsLC(
			const Planner::FFEvent * node,
			std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
			stn::ColinSTNImpl * network,
			std::map<const Planner::FFEvent *, const Planner::FFEvent *> & ptr);
	//Minimum Spanning Tree
	void initialiseMST(stn::ColinSTNImpl * network,
		const Planner::FFEvent * start,
		std::map<const Planner::FFEvent *, double> & fwdDist,
		std::map<const Planner::FFEvent *, double> & revDist,
		std::map<const Planner::FFEvent *, bool> & lb,
		std::map<const Planner::FFEvent *, bool> & ub);
	std::set<const Util::triple<const Planner::FFEvent *, double> *> extractConflictsMST(
			const Planner::FFEvent * node, stn::ColinSTNImpl * network,
			std::map<const Planner::FFEvent *, const Planner::FFEvent *> & fwdPtr,
			std::map<const Planner::FFEvent *, const Planner::FFEvent *> & revPtr);
	//Floyd-Warshall
	void initialiseFW(stn::ColinSTNImpl * network,
		map<const Planner::FFEvent *, map<const Planner::FFEvent *, double> > & dist,
	map<const Planner::FFEvent *, map<const Planner::FFEvent *, 
		const Planner::FFEvent *> > & ptr);
	std::set<const Util::triple<const Planner::FFEvent *, double> *> extractConflictsFW(
		const Planner::FFEvent * node, const Planner::FFEvent * end,
		std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
		stn::ColinSTNImpl * network,
		map<const Planner::FFEvent *, map<const Planner::FFEvent *, 
		const Planner::FFEvent *> > & nextPtr);

public:
	static ITC * getInstance();
	std::set<const Util::triple<const Planner::FFEvent *, double> *> checkTemporalConsistencyLC(
			stn::ColinSTNImpl * network, const Planner::FFEvent * start);
	std::set<const Util::triple<const Planner::FFEvent *, double> *> checkTemporalConsistencyMST(
		stn::ColinSTNImpl * network, const Planner::FFEvent * start);
	std::set<const Util::triple<const Planner::FFEvent *, double> *> checkTemporalConsistencyBF(
		stn::ColinSTNImpl * network, const Planner::FFEvent * start);
	std::set<const Util::triple<const Planner::FFEvent *, double> *> checkTemporalConsistencyFW(
		stn::ColinSTNImpl * network);
};

}

#endif /* __ITC_ITC_H_ */
