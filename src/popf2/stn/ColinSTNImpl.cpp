/*
 * STNImpl.cpp
 *
 *  Created on: 1 Sep 2015
 *      Author: tony
 */

#include <limits>
#include <sstream>
#include <iostream>
#include <queue>

#include "ColinSTNImpl.h"
#include "../RPGBuilder.h"
#include "PDDLUtils.h"

using namespace std;

namespace stn {

const double ColinSTNImpl::MAX_ACTION_SEPARATION = std::numeric_limits<int>::max() * 1.0;
/*Use This definition for Label Correcting ITC.
 *otherwise it will take forever.
 */
// const double ColinSTNImpl::MAX_ACTION_SEPARATION = 200000;
const double ColinSTNImpl::MIN_ACTION_SEPARATION = EPSILON;
/*This is to counter floating point arithmatic */
const double ColinSTNImpl::ACCURACY = EPSILON/10.0;

double ColinSTNImpl::getEdge(const Planner::FFEvent * vert1,
		const Planner::FFEvent * vert2) {
	const Util::triple<const Planner::FFEvent *, double> * edge = 
		STN<const Planner::FFEvent *, double>::getEdge(vert1, vert2);
	if (!edge) {
		return MAX_ACTION_SEPARATION;
	}
	return edge->second;
}

void ColinSTNImpl::removeAllEdges(set<const Util::triple<const Planner::FFEvent *, 
	double> *> toRemove) {
	set<int> indiciesToRemove;
	set<const Util::triple<const Planner::FFEvent *, 
		double> *>::const_iterator removeItr = toRemove.begin();

	for (; removeItr != toRemove.end(); removeItr++) {
		std::vector<Util::triple<const Planner::FFEvent *, 
		double> >::iterator it = std::find(edges.begin(), 
			edges.end(), **removeItr);
		if (it != STN::edges.end()) {
			edges.erase(it);
		}
		
	}
}

std::vector<std::vector<double> > ColinSTNImpl::toMatrix() {
	std::vector<std::vector<double> > matrix(size());
	for (int i = 0; i < size(); i++) {
		matrix[i].resize(size());
	}
	for (int i = 0; i < size(); i++) {
		for (int j = 0; j < size(); j++) {
			if (i == j) {
				matrix[i][j] = 0;
			} else {
				const Planner::FFEvent * event1 = getVertex(i);
				const Planner::FFEvent * event2 = getVertex(j);
				const typename Util::triple<const Planner::FFEvent *, double> * edge =
						STN::getEdge(event1, event2);
				if (!edge) {
					matrix[i][j] = MAX_ACTION_SEPARATION;
				} else {
					matrix[i][j] = edge->second;
				}
			}
		}
	}
	return matrix;
}

std::string ColinSTNImpl::toMatrixString() {
	std::vector<std::vector<double> > matrix = toMatrix();
	return toMatrixString(matrix);
}

std::string ColinSTNImpl::toMatrixString(
		std::vector<std::vector<double> > matrix) {
	std::ostringstream output;
	int size = matrix.size();
	for (int i = 0; i < size; i++) {
		output << "|";
		for (int j = 0; j < size; j++) {
			char temp[50];
			std::sprintf(temp, "%14.3f", matrix[i][j]);
			output << temp << "|";
		}
		output << endl;
	}
	return output.str();
}

std::string ColinSTNImpl::getMatixLegendStr() {
	std::ostringstream output;
	for (int i = 0; i < size(); i++) {
		output << i << ": " << PDDL::getActionName(getVertex(i)) << endl;
	}
	return output.str();
}

/**
 * Floyd-Warshall algorithm
 */
// bool ColinSTNImpl::isConsistent() {
// 	std::vector<std::vector<double> > matrix = toMatrix();
// 	for (int k = 0; k < size(); k++) {
// 		for (int i = 0; i < size(); i++) {
// 			for (int j = 0; j < size(); j++) {
// 				double d = matrix[i][k] + matrix[k][j];
// 				if (matrix[i][j] - d > ACCURACY) {
// 					matrix[i][j] = d;
// 				}
// 				if ((i==j) && (matrix[i][j] < -ACCURACY)) {
// 					cout << "Negative Cycle: " << std::setprecision(6) << matrix[i][j] << endl;
// 					return false;
// 				}
// 			}
// 		}
// 	}
// 	return true;
// }

bool ColinSTNImpl::isConsistent(const Planner::FFEvent * source) {
	
	int num_nodes = size();
	map<const Planner::FFEvent *, double> distance;
	map<const Planner::FFEvent *, bool> currently_in_queue;
	map<const Planner::FFEvent *, int> times_in_queue;
	map<const Planner::FFEvent *, const Planner::FFEvent *> predecessor;


	//Initialise vars
	for (int i = 0; i < num_nodes; i++) {
		const Planner::FFEvent * node_i = getVertex(i);
		distance[node_i] = stn::ColinSTNImpl::MAX_ACTION_SEPARATION;
		currently_in_queue[node_i] = false;
		times_in_queue[node_i] = 0;
		predecessor[node_i] = NULL;
	}

	//Initialise queue
	std::queue<const Planner::FFEvent *> q;

	distance[source] = 0;
	currently_in_queue[source] = true;
	times_in_queue[source] = 1;
	q.push(source);

	bool negative_cycle_exists = false;

	while ((!q.empty()) && (!negative_cycle_exists)) {
		const Planner::FFEvent * node = q.front();
		q.pop();
		currently_in_queue[node] = false;
		const std::vector<const Util::triple<const Planner::FFEvent *, 
			double> *> outEdges = getOutEdges(node);
		std::vector<const Util::triple<const Planner::FFEvent *, 
			double> *>::const_iterator outEdgeItr = outEdges.begin();

		for (; outEdgeItr != outEdges.end(); outEdgeItr++) {
			const Planner::FFEvent * neighbor = (*outEdgeItr)->third;
			if ((distance[neighbor] == stn::ColinSTNImpl::MAX_ACTION_SEPARATION) ||
				(distance[neighbor] > distance[node] + (*outEdgeItr)->second 
					+ ACCURACY)){
                
				predecessor[neighbor] = node;

				distance[neighbor] = distance[node] + (*outEdgeItr)->second;
				if (!currently_in_queue[neighbor]) {
					currently_in_queue[neighbor] = true;
					times_in_queue[neighbor] += 1;
					if (times_in_queue[neighbor] > num_nodes) {
						negative_cycle_exists = true;
						break;
					}
					q.push(neighbor);
				}
			}
		}
	}
	return !negative_cycle_exists;
}

bool ColinSTNImpl::FIFOLabelCorrectingAlgorithm() {
	std::vector<const Planner::FFEvent *> queue;
	std::map<const Planner::FFEvent *, double> dist;

	const double negativeCycleLimit = -1 * this->size()
			* ColinSTNImpl::MAX_ACTION_SEPARATION;

	//Initialise distances to maximum
	for (int i = 0; i < this->size(); i++) {
		const Planner::FFEvent * node = this->getVertex(i);
		dist[node] = ColinSTNImpl::MAX_ACTION_SEPARATION;
	}

	//Initialise Queue
	const Planner::FFEvent * start = this->getVertex(0);

	dist[start] = 0;
	queue.insert(queue.begin(), start);

	while (!queue.empty()) {
		const Planner::FFEvent * item = queue.back();
		queue.pop_back();
		std::vector<const Planner::FFEvent *> successors = getSuccessors(item);
		std::vector<const Planner::FFEvent *>::const_iterator succItr =
				successors.begin();
		for (; succItr != successors.end(); succItr++) {
			const Planner::FFEvent * succ = *succItr;
			if (dist[succ] > dist[item] + this->getEdge(item, succ)) {
				dist[succ] = dist[item] + this->getEdge(item, succ);
				queue.insert(queue.begin(), succ);
				//Check for -ve cycles
				if (dist[succ] <= negativeCycleLimit) {
					return false;
				}
			}
		}
	}
	return true;
}

std::vector<const Planner::FFEvent *> ColinSTNImpl::getSuccessors(
		const Planner::FFEvent * node) {
	std::vector<const Planner::FFEvent *> successors;
	std::vector<const Util::triple<const Planner::FFEvent *, double> *> outEdges =
			getOutEdges(node);
	std::vector<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator outEdgeItr =
			outEdges.begin();
	for (; outEdgeItr != outEdges.end(); outEdgeItr++) {
		successors.push_back((*outEdgeItr)->third);
	}
	return successors;
}

ColinSTNImpl ColinSTNImpl::makeColinSTN(const std::list<Planner::FFEvent> & events,
			std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> > orderingConstraints,
			const Planner::FFEvent * initialEvent) {
	
	ColinSTNImpl stn;

	//add action duration constraints
	std::list<Planner::FFEvent>::const_iterator eventItr = events.begin();
	const std::list<Planner::FFEvent>::const_iterator eventItrEnd =
			events.end();
	for (; eventItr != eventItrEnd; eventItr++) {
		if (eventItr->pairWithStep >= 0) {
			std::list<Planner::FFEvent>::const_iterator predItr =
					events.begin();
			std::advance(predItr, eventItr->pairWithStep);
			Util::triple<const Planner::FFEvent *, double> constraint;
			const Planner::FFEvent * first = &(*eventItr);
			const Planner::FFEvent * third = &(*predItr);
			double duration = Planner::RPGBuilder::getOpMaxDuration(
					first->action->getID(), 1);
			//negate for the lower constraint
			if (first->pairWithStep < third->pairWithStep) {
				duration = -Planner::RPGBuilder::getOpMinDuration(
					first->action->getID(), 1);
			}
			constraint.make_triple(first, duration, third);
			stn.addEdge(constraint);
		}
	}

	//Add causal constraints
	const double minCausalLink = -MIN_ACTION_SEPARATION;
	const double maxCausalLink = MAX_ACTION_SEPARATION / 2;
	std::set<std::pair<const Planner::FFEvent *, const Planner::FFEvent *> >::const_iterator ordItr =
			orderingConstraints.begin();
	const std::set<
			std::pair<const Planner::FFEvent *, const Planner::FFEvent *> >::const_iterator ordItrEnd =
			orderingConstraints.end();
	for (; ordItr != ordItrEnd; ordItr++) {
		const std::pair<const Planner::FFEvent *, const Planner::FFEvent *> * causalLink =
				&(*ordItr);
		const Planner::FFEvent * first = causalLink->first;
		const Planner::FFEvent * third = causalLink->second;
		Util::triple<const Planner::FFEvent *, double> minConstraint;
		Util::triple<const Planner::FFEvent *, double> maxConstraint;
		if (first == initialEvent) {
			//Remember that the initial event
			//shouldn't affect execution
			minConstraint.make_triple(third, 0.0, first);
		} else {
			minConstraint.make_triple(third, minCausalLink, first);
		}
		maxConstraint.make_triple(first, maxCausalLink, third);
		stn.addEdge(minConstraint);
		stn.addEdge(maxConstraint);
	}
	return stn;
}

std::string ColinSTNImpl::getD3Output() const {
	ostringstream output;
	output << "{" << endl;
	output << "nodes: [" << endl;
	output << "\t";
	for (int i = 0; i < size(); i++) {
		const Planner::FFEvent * node = getVertex(i);
		output << "{" << endl;
		output << "\t\tid: \'" << PDDL::getActionName(node) 
			<< "-" << node->time_spec << "\'," << endl;
		output << "\t\tsize: " << 10 << endl;
		output << "\t}";
		if (i != (size() - 1)) {
			output << ", ";
		}
	}
	output << "]," << endl;
    output << "links: [" << endl;
	std::vector<Util::triple<const Planner::FFEvent *, double> >::const_iterator constItr =
	 		edges.begin();    
	output << "\t";
	for (; constItr != edges.end(); constItr++) {
		Util::triple<const Planner::FFEvent *, double> constraint = *constItr;
		if (constItr != edges.begin()) {
			output << ", ";
		}
		output << "{" << endl;
		output << "\t\tsource: \'" << PDDL::getActionName(constraint.first) 
			<< "-" << constraint.first->time_spec << "\'," << endl;
		output << "\t\ttarget: \'" << PDDL::getActionName(constraint.third) 
			<< "-" << constraint.third->time_spec << "\'," << endl;
		output << "\t\tweight: " << (constraint.second < (MAX_ACTION_SEPARATION / 4) ? 
			(10000 / 4) : 10000) << endl;
		output << "\t}";
	}
	output << endl << "]" << endl;
    output << "};" << endl;
	return output.str();
}

std::ostream & operator<<(std::ostream & output,
		const ColinSTNImpl & colinSTN) {
	output << "There are " << colinSTN.size() << " verticies and "
			<< colinSTN.edges.size() << " constraints." << std::endl;

	std::vector<Util::triple<const Planner::FFEvent *, double> >::const_iterator constItr =
			colinSTN.edges.begin();
	const std::vector<Util::triple<const Planner::FFEvent *, double> >::const_iterator constItrEnd =
			colinSTN.edges.end();
	for (; constItr != constItrEnd; constItr++) {
		Util::triple<const Planner::FFEvent *, double> constraint = *constItr;
		output << PDDL::getActionName(constraint.first)
				<< "-" << constraint.first->time_spec
				<< "--[" << constraint.second << "]-->"
				<< PDDL::getActionName(constraint.third)
				<< "-" << constraint.third->time_spec
				<< std::endl;
	}
	return output;
}

std::ostream & operator<<(std::ostream & output,
				const Util::triple<const Planner::FFEvent *, double> & edge) {
	output << PDDL::getActionName(edge.first)
		<< "-" << edge.first->time_spec
		<< "--[" << edge.second << "]-->"
		<< PDDL::getActionName(edge.third)
		<< "-" << edge.third->time_spec;
}
}
