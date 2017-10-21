/*
 * ExpNode.h
 *
 *  Created on: 22 Jul 2016
 *      Author: tony
 */

#ifndef COLIN_UTIL_EXPNODE_H_
#define COLIN_UTIL_EXPNODE_H_

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
#endif /* COLIN_UTIL_EXPNODE_H_ */
