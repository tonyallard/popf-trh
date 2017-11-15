/*
 * stn.h
 *
 *  Created on: 31 Aug 2015
 *      Author: tony
 */

#ifndef __STN_STN_H_
#define __STN_STN_H_

#include <vector>
#include <set>

#include "../util/Util.h"

namespace stn {

template<typename T1, typename T2>
class STN {
protected:
	std::vector<T1> verticies;
	std::vector<Util::triple<T1, T2> > edges;

	void addVertex(T1 vertex) {
		if (!vertexExists(vertex)) {
			verticies.push_back(vertex);
		}
	}
	;

public:
	STN() {
	}
	;
	void addEdge(Util::triple<T1, T2> edge) {
		if (!edgeExists(edge)) {
			edges.push_back(edge);
			addVertex(edge.first);
			addVertex(edge.third);
		}
	}
	;

	const std::vector<Util::triple<T1, T2> > & getEdges() const {
		return edges;
	}

	const Util::triple<T1, T2> * getEdge(T1 vert1, T1 vert2) const {
		typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			const Util::triple<T1, T2> * sample = &(*edgeItr);
			if ((vert1 == sample->first) && (vert2 == sample->third)) {
				return sample;
			}
		}
		return 0;
	}
	;

	bool edgeExists(Util::triple<T1, T2> edge) const {
		typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			const Util::triple<T1, T2> * sample = &(*edgeItr);
			if ((sample->first == edge.first)
					&& (sample->third == edge.third)) {
				return true;
			}
		}
		return false;
	}
	;

	bool edgeExists(T1 vertex1, T1 vertex2) const {
		typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			const Util::triple<T1, T2> * sample = &(*edgeItr);
			if ((sample->first == vertex1) && (sample->third == vertex2)) {
				return true;
			}
		}
		return false;
	}
	;

	bool updateEdgeWeight(T1 vertex1, T1 vertex2, T2 weight) {
		typename std::vector<Util::triple<T1, T2> >::iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			Util::triple<T1, T2> * sample = &(*edgeItr);
			if ((sample->first == vertex1) && (sample->third == vertex2)) {
				sample->second = weight;
				return true;
			}
		}
		return false;
	}
	;

	T1 getVertex(int idx) const {
		return verticies[idx];
	}
	;

	bool vertexExists(T1 vertex) const {
		typename std::vector<T1>::const_iterator verItr = verticies.begin();
		const typename std::vector<T1>::const_iterator verItrEnd =
				verticies.end();

		for (; verItr != verItrEnd; verItr++) {
			const T1 sample = *verItr;
			if (sample == vertex) {
				return true;
			}
		}
		return false;
	}
	;

	int size() const {
		return verticies.size();
	}
	;

	const std::vector<const Util::triple<T1, T2> *> getOutEdges(T1 node) const {
		std::vector<const Util::triple<T1, T2> *> outEdges;
		typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			const Util::triple<T1, T2> * sample = &(*edgeItr);
			if (sample->first == node) {
				outEdges.push_back(sample);
			}
		}
		return outEdges;
	}
	;

	const std::vector<const Util::triple<T1, T2> *> getInEdges(T1 node) const {
		std::vector<const Util::triple<T1, T2> *> inEdges;
		typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			const Util::triple<T1, T2> * sample = &(*edgeItr);
			if (sample->third == node) {
				inEdges.push_back(sample);
			}
		}
		return inEdges;
	}
	;

	const std::set<const Util::triple<T1, T2> *> getAllEdgesBetween(
			std::set<T1> nodes) const {
		std::set<const Util::triple<T1, T2> *> matchingEdges;
		typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItr =
				edges.begin();
		const typename std::vector<Util::triple<T1, T2> >::const_iterator edgeItrEnd =
				edges.end();
		for (; edgeItr != edgeItrEnd; edgeItr++) {
			const Util::triple<T1, T2> * sample = &(*edgeItr);
			if ((nodes.find(sample->first) != nodes.end())
					&& (nodes.find(sample->third) != nodes.end())) {
				matchingEdges.insert(sample);
			}
		}
		return matchingEdges;
	}
	;
};

}

#endif /* __STN_STN_H_ */
