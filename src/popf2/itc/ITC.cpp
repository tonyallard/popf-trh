/*
 * ITC.cpp
 *
 *  Created on: 13 Sep 2015
 *      Author: tony
 */

#include "ITC.h"
 
using namespace std;

namespace ITC {

ITC * ITC::INSTANCE = NULL;

ITC * ITC::getInstance() {
	if (!INSTANCE) {
		INSTANCE = new ITC();
	}
	return INSTANCE;
}

void ITC::initialiseLC(stn::ColinSTNImpl * network,
		const Planner::FFEvent * start,
		std::map<const Planner::FFEvent *, double> & dist) {
	for (int i = 0; i < network->size(); i++) {
		const Planner::FFEvent * node = network->getVertex(i);
		dist[node] = stn::ColinSTNImpl::MAX_ACTION_SEPARATION;
		if (node == start) {
			dist[node] = 0.0;
		}
	}
}

/**
 * Conducts label correcting algorithm in which negative cycles are followed until they reach a
 * limit at which point a negative cycle is declared and followed back to extract conflicts
 *
 * Distance is measured from the start node, which was initialised in the constructor.
 */
std::set<const Util::triple<const Planner::FFEvent *, double> *> ITC::checkTemporalConsistencyLC(
		stn::ColinSTNImpl * network, const Planner::FFEvent * start) {
	cout << "Label Correcting" << endl;
	//Initialise required variables
	std::map<const Planner::FFEvent *, double> dist;
	std::map<const Planner::FFEvent *, const Planner::FFEvent *> ptr;
	std::vector<const Planner::FFEvent *> queue;
	queue.push_back(start);
	initialiseLC(network, start, dist);
	const double negativeCycleLimit = -1 * network->size()
			* stn::ColinSTNImpl::MAX_ACTION_SEPARATION;
	//Using a FIFO queue
	while (!queue.empty()) {
		//pop item at the back
		const Planner::FFEvent * item = queue.back();
		queue.pop_back();
		// get successors to item
		// these are the links eminating out from the item
		std::vector<const Planner::FFEvent *> successors =
				network->getSuccessors(item);
		//Cycle through successors, update their distances and add them to
		//The FIFO queue to have their successors checked
		std::vector<const Planner::FFEvent *>::const_iterator succItr =
				successors.begin();
		for (; succItr != successors.end(); succItr++) {
			const Planner::FFEvent * succ = *succItr;
			// If we have found a shorter path
			if (dist[succ] > dist[item] + network->getEdge(item, succ)) {
				//update the path
				dist[succ] = dist[item] + network->getEdge(item, succ);
				ptr[succ] = item;
				//insert new item to check at the front
				queue.insert(queue.begin(), succ);
				//Check for -ve cycles
				if (dist[succ] <= negativeCycleLimit) {
					//Negative Cycle Detected
					cout << "Negative Cycle Detected: " << dist[succ] 
							<< " <= " << negativeCycleLimit << endl;
					std::set<const Util::triple<const Planner::FFEvent *, double> *> conflicts;
					return extractConflictsLC(succ, conflicts, network, ptr);
				}
			}
		}
	}
	std::set<const Util::triple<const Planner::FFEvent *, double> *> emptySet;
	return emptySet;
}

/**
 * Cycle through back ptrs starting at the node the negative cycle was identified at
 * Keep adding nodes and conflicted links to their respective lists, until the same node
 * is encountered (closing the loop on the cycle).
 */
std::set<const Util::triple<const Planner::FFEvent *, double> *> ITC::extractConflictsLC(
		const Planner::FFEvent * node,
		std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
		stn::ColinSTNImpl * network,
		std::map<const Planner::FFEvent *, const Planner::FFEvent *> & ptr) {
	
	const Util::triple<const Planner::FFEvent *, double> * edge = network->
		STN::getEdge(ptr[node], node);
	if (conflicts.find(edge) != conflicts.end()) {
		return conflicts;
	}
	conflicts.insert(edge);
	return extractConflictsLC(ptr[node], conflicts, network, ptr);
}

void ITC::initialiseMST(stn::ColinSTNImpl * network,
		const Planner::FFEvent * start,
		std::map<const Planner::FFEvent *, double> & fwdDist,
		std::map<const Planner::FFEvent *, double> & revDist,
		std::map<const Planner::FFEvent *, bool> & lb,
		std::map<const Planner::FFEvent *, bool> & ub) {
	for (int i = 0; i < network->size(); i++) {
		const Planner::FFEvent * node = network->getVertex(i);
		fwdDist[node] = stn::ColinSTNImpl::MAX_ACTION_SEPARATION;
		revDist[node] = stn::ColinSTNImpl::MAX_ACTION_SEPARATION;
		if (node == start) {
			fwdDist[node] = 0.0;
			revDist[node] = 0.0;
		}
		lb[node] = true;
		ub[node] = true;
	}
}

std::set<const Util::triple<const Planner::FFEvent *, 
	double> *> ITC::checkTemporalConsistencyMST(
		stn::ColinSTNImpl * network, const Planner::FFEvent * start) {
	cout << "Minimum Spanning Tree" << endl;
	//Initialise required variables
	std::map<const Planner::FFEvent *, double> fwdDist;
	std::map<const Planner::FFEvent *, double> revDist;
	std::map<const Planner::FFEvent *, bool> lb;
	std::map<const Planner::FFEvent *, bool> ub;
	std::map<const Planner::FFEvent *, const Planner::FFEvent *> fwdPtr;
	std::map<const Planner::FFEvent *, const Planner::FFEvent *> revPtr;
	std::vector<const Planner::FFEvent *> queue;
	queue.push_back(start);
	initialiseMST(network, start, fwdDist, revDist, lb, ub);
	//Using a FIFO queue
	while (!queue.empty()) {
		//pop item at the back
		const Planner::FFEvent * item = queue.back();
		queue.pop_back();
		//Process the out edges of this node if required.
		if (ub[item]) {
			std::vector<const Util::triple<const Planner::FFEvent *, double> *> successors = 
				network->getOutEdges(item);
			std::vector<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator succItr =
				successors.begin();
			for (; succItr != successors.end(); succItr++) {
				const Planner::FFEvent * succ = (*succItr)->third;
				//Check for shorter path
				if (fwdDist[item] + (*succItr)->second - fwdDist[succ]
					< -stn::ColinSTNImpl::ACCURACY) {
					//update
					fwdDist[succ] = fwdDist[item] + (*succItr)->second;
					revPtr[succ] = item;
					//Make sure dependent out edges are updated.
					ub[succ] = true;
					//Check for negative cycle
					if (fwdDist[succ] + revDist[succ] < -stn::ColinSTNImpl::ACCURACY) {
						//Negative Cycle Detected
						cout << "Negative Cycle Detected (fwd): " << fwdDist[succ] 
							<< " + " << revDist[succ] << " < " 
							<< -stn::ColinSTNImpl::ACCURACY << endl;
						return extractConflictsMST(succ, network, fwdPtr, revPtr);
					}
					//Add node to back of queue to be processed.
					//Adding it to the back is actually important.
					if(std::find(queue.begin(), queue.end(), succ) == queue.end()) {
						queue.insert(queue.end(), succ);	
					}
				}
			}
		}
		//Process the in edges of this node if required.
		if (lb[item]) {
			std::vector<const Util::triple<const Planner::FFEvent *, double> *> predecessors = 
				network->getInEdges(item);
			std::vector<const Util::triple<const Planner::FFEvent *, double> *>::const_iterator predItr =
				predecessors.begin();
			for (; predItr != predecessors.end(); predItr++) {
				const Planner::FFEvent * pred = (*predItr)->first;
				//Check for shorter path
				if (revDist[item] + (*predItr)->second - revDist[pred]
					< -stn::ColinSTNImpl::ACCURACY) {
					//update
					revDist[pred] = revDist[item] + (*predItr)->second;
					fwdPtr[pred] = item;
					//Make sure dependent out edges are updated.
					lb[pred] = true;
					//Check for negative cycle
					if (fwdDist[pred] + revDist[pred] < -stn::ColinSTNImpl::ACCURACY) {
						//Negative Cycle Detected
						cout << "Negative Cycle Detected (bwd): " << fwdDist[pred] 
							<< " + " << revDist[pred] << " < " 
							<< -stn::ColinSTNImpl::ACCURACY << endl;
						return extractConflictsMST(pred, network, fwdPtr, revPtr);
					}
					//Add node to back of queue to be processed.
					//Adding it to the back is actually important.
					if(std::find(queue.begin(), queue.end(), pred) == queue.end()) {
						queue.insert(queue.end(), pred);	
					}
				}
			}
		}
		//Node has been processed
		ub[item] = false;
		lb[item] = false;
	}
	std::set<const Util::triple<const Planner::FFEvent *, double> *> emptySet;
	return emptySet;
}

std::set<const Util::triple<const Planner::FFEvent *, double> *> ITC::extractConflictsMST(
			const Planner::FFEvent * node, stn::ColinSTNImpl * network,
			std::map<const Planner::FFEvent *, const Planner::FFEvent *> & fwdPtr,
			std::map<const Planner::FFEvent *, const Planner::FFEvent *> & revPtr) {
	
	std::set<const Planner::FFEvent *> fwdConflictedNodes;
	std::set<const Planner::FFEvent *> revConflictedNodes;
	std::set<const Util::triple<const Planner::FFEvent *, double> *> fwdConflicts;
	std::set<const Util::triple<const Planner::FFEvent *, double> *> revConflicts;

	//Get fwd pointer from node
	const Planner::FFEvent * fwdNode = node;
	while (fwdConflictedNodes.find(fwdNode) == fwdConflictedNodes.end()) {
		if (fwdNode == NULL) {
			break;
		}
		fwdConflictedNodes.insert(fwdNode);
		if (fwdPtr[fwdNode] == NULL) {
			break;
		}
		//Get edge associated with fwdNode to fwdPtr; 
		const Util::triple<const Planner::FFEvent *, double> * fwdConflictedEdge = 
			((::stn::STN<const Planner::FFEvent *, double>*)network)->
				getEdge(fwdNode, fwdPtr[fwdNode]);
		fwdConflicts.insert(fwdConflictedEdge);
		fwdNode = fwdPtr[fwdNode];
	}
	const Planner::FFEvent * revNode = node;
	while (revConflictedNodes.find(revNode) == revConflictedNodes.end()) {
		if (revNode == NULL) {
			break;
		}
		revConflictedNodes.insert(revNode);
		if (revPtr[revNode] == NULL) {
			break;
		}
		//Get edge associated with revPtr to revNode
		const Util::triple<const Planner::FFEvent *, double> * revConflictedEdge = 
			((::stn::STN<const Planner::FFEvent *, double>*)network)->
				getEdge(revPtr[revNode], revNode);
		revConflicts.insert(revConflictedEdge);
		revNode = revPtr[revNode];
	}
	fwdConflicts.insert(revConflicts.begin(), revConflicts.end());
	return fwdConflicts;
}
/**
 * Bellman-Ford algorithm
 * FIXME: This doesn't work, but thats ok there are another 2 that do.
 */
std::set<const Util::triple<const Planner::FFEvent *, 
	double> *> ITC::checkTemporalConsistencyBF(stn::ColinSTNImpl * network,
		const Planner::FFEvent * start) {
	cout << "Bellman–Ford" << endl;
	std::map<const Planner::FFEvent *, double> dist;
	std::map<const Planner::FFEvent *, const Planner::FFEvent *> ptr;
	initialiseLC(network, start, dist);
	//Run Belman-Ford for n iterations to detect negative cycles
	//This should ensure the predeccessor pointer is popoulated!
	for (int k = 0; k < network->size(); k++) {
		for (int i = 0; i < network->size(); i++) {
			const Planner::FFEvent * node_i = network->getVertex(i);
			std::vector<const Util::triple<const Planner::FFEvent *, 
				double> *> edges = network->getOutEdges(node_i);
			std::vector<const Util::triple<const Planner::FFEvent *, 
				double> *>::const_iterator edgeItr = edges.begin();
			for (; edgeItr != edges.end(); edgeItr++) {
				const Planner::FFEvent * node_j = (*edgeItr)->third;
				if (dist[node_i] + (*edgeItr)->second - dist[node_j] 
						< -stn::ColinSTNImpl::ACCURACY) {
					dist[node_j] = dist[node_i] + (*edgeItr)->second;
					ptr[node_j] = node_i;
				}
			}
		}
	}

	//Look for negative cycles and trace the predeccessor ptr backwards!
	std::vector<Util::triple<const Planner::FFEvent *, 
			double> > edges = network->getEdges();
	std::vector<Util::triple<const Planner::FFEvent *, 
			double> >::const_iterator edgeItr = edges.begin();
	for (; edgeItr != edges.end(); edgeItr++) {
		const Util::triple<const Planner::FFEvent *, 
			double> edge = *edgeItr;
		if (dist[edge.first] + edge.second - dist[edge.third] 
			< -stn::ColinSTNImpl::ACCURACY) {
			cout << "Negative Cycle Detected: " << dist[edge.first] << " + " 
				<< edge.second << " < " << dist[edge.third] << endl;
			/*Uses same extraction technique as Label-Correcting*/
			std::set<const Util::triple<const Planner::FFEvent *, double> *> conflicts;
			return extractConflictsLC(edge.third, conflicts, network, ptr);
		}
	}
	std::set<const Util::triple<const Planner::FFEvent *, double> *> emptySet;
	return emptySet;
}

void ITC::initialiseFW(stn::ColinSTNImpl * network,
		map<const Planner::FFEvent *, map<const Planner::FFEvent *, double> > & dist,
	map<const Planner::FFEvent *, map<const Planner::FFEvent *, 
		const Planner::FFEvent *> > & ptr) {
	for (int i = 0; i < network->size(); i++) {
		const Planner::FFEvent * node_i = network->getVertex(i);
		for (int j = 0; j < network->size(); j++) {
			const Planner::FFEvent * node_j = network->getVertex(j);
			if (node_i == node_j) {
				dist[node_i][node_j] = 0.0;
			} else {
				const Util::triple<const Planner::FFEvent *, double> * edge = 
					network->STN::getEdge(node_i, node_j);
				if (edge) {
					dist[node_i][node_j] = edge->second;
					ptr[node_i][node_j] = node_i;
				} else {
					dist[node_i][node_j] = stn::ColinSTNImpl::MAX_ACTION_SEPARATION;
				}
			}
		}
	}
}

std::set<const Util::triple<const Planner::FFEvent *, 
	double> *> ITC::checkTemporalConsistencyFW(stn::ColinSTNImpl * network) {
	// cout << "Floyd-Warshall" << endl;
	map<const Planner::FFEvent *, map<const Planner::FFEvent *, double> > dist;
	map<const Planner::FFEvent *, map<const Planner::FFEvent *, 
		const Planner::FFEvent *> > ptr;
	initialiseFW(network, dist, ptr);
	for (int k = 0; k < network->size(); k++) {
		const Planner::FFEvent * node_k = network->getVertex(k);
		for (int i = 0; i < network->size(); i++) {
			const Planner::FFEvent * node_i = network->getVertex(i);
			for (int j = 0; j < network->size(); j++) {
				const Planner::FFEvent * node_j = network->getVertex(j);
				if ((dist[node_i][node_j] - (dist[node_i][node_k] 
					+ dist[node_k][node_j])) > stn::ColinSTNImpl::ACCURACY) {
					dist[node_i][node_j] = dist[node_i][node_k] + dist[node_k][node_j];
					ptr[node_i][node_j] = ptr[node_k][node_j];
				}

				//Check for negative cycle
				if ((node_i == node_j) && (dist[node_i][node_j] < -stn::ColinSTNImpl::ACCURACY)) {
					// cout << "Negative Cycle Found: " << dist[node_i][node_i] << endl;
					std::set<const Util::triple<const Planner::FFEvent *, double> *> conflicts;
					return extractConflictsFW(node_i, node_i, conflicts, network, ptr);
				}
			}
		}
	}
	
	std::set<const Util::triple<const Planner::FFEvent *, double> *> emptySet;
	return emptySet;
}

std::set<const Util::triple<const Planner::FFEvent *, double> *> ITC::extractConflictsFW(
		const Planner::FFEvent * start, const Planner::FFEvent * end,
		std::set<const Util::triple<const Planner::FFEvent *, double> *> & conflicts,
		stn::ColinSTNImpl * network,
		map<const Planner::FFEvent *, map<const Planner::FFEvent *, 
		const Planner::FFEvent *> > & ptr) {

	const Util::triple<const Planner::FFEvent *, double> * edge = 
				network->STN::getEdge(ptr[start][end], end);
	if (conflicts.find(edge) != conflicts.end()) {
		return conflicts;
	}
	conflicts.insert(edge);
	return extractConflictsFW(start, ptr[start][end], conflicts, network, ptr);
}
}
