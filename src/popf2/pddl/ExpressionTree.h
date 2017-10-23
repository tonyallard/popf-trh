/*
 * ExpNode.h
 *
 *  Created on: 22 Jul 2016
 *      Author: tony
 */

#ifndef __UTIL_EXPNODE
#define __UTIL_EXPNODE

#include <string>
#include <list>
#include <deque>

#include "PDDLObject.h"
#include "../RPGBuilder.h"

using namespace std;

namespace PDDL {

struct Node {
	string element;
	bool isOp;
	struct Node *left, *right;
};

class ExpressionTree {
private:
	deque<Node *> data;
	int top;

	//Interfaces are static
	//Tree is generated internally
	ExpressionTree() {
	}
	~ExpressionTree() {
		emptyStack();
	}
	bool isEmpty();
	void emptyStack();
	void push(struct Node * item);
	struct Node * pop();
public:
	static ExpressionTree * postfix2expTree(
			list<Planner::RPGBuilder::Operand> formula,
			const map<PDDLObject, string> & parameterTable);
	static string preOrder(const Node * x);
	struct Node * getExpRoot() {
		return data.front();
	}
};
}
#endif /* __UTIL_EXPNODE */
