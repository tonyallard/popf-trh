/*
 * ExpressionTree.cpp
 *
 *  Created on: 22 Jul 2016
 *      Author: tony
 */

#include <iostream>
#include <sstream>

#include "ExpressionTree.h"
#include "PDDLUtils.h"

using namespace std;

namespace PDDL {

bool ExpressionTree::isEmpty() {
	return data.empty();
}

void ExpressionTree::emptyStack() {
	deque<Node *>::const_iterator dataItr = data.begin();
	const deque<Node *>::const_iterator dataItrEnd = data.end();
	for (; dataItr != dataItrEnd; dataItr++) {
		delete *dataItr;
	}
	data.clear();
}

void ExpressionTree::push(struct Node * item) {
	data.push_front(item);
}

struct Node * ExpressionTree::pop() {
	struct Node * ret = NULL;
	if (!isEmpty()) {
		ret = data.front();
		data.pop_front();
	}
	return ret;
}

ExpressionTree * ExpressionTree::postfix2expTree(
		list<Planner::RPGBuilder::Operand> formula,
		const map<PDDLObject, string> & parameterTable) {
	ExpressionTree * expTree = new ExpressionTree();
	struct Node *newNode, *op1, *op2;
	list<Planner::RPGBuilder::Operand>::const_iterator formItr =
			formula.begin();
	const list<Planner::RPGBuilder::Operand>::const_iterator formItrEnd =
			formula.end();
	for (; formItr != formItrEnd; formItr++) {
		/*
		 * if its an operator
		 * pop the last two operands and link
		 */
		if (isOperator(formItr->numericOp)) {
			op1 = expTree->pop();
			op2 = expTree->pop();
			newNode = new Node();
			newNode->left = op2;
			newNode->right = op1;
			newNode->element = getOperandString(*formItr, parameterTable);
			newNode->isOp = true;
			expTree->push(newNode);
		}
		/*
		 * Else if its an operand
		 * create the node and push
		 */
		else if (isOperand(formItr->numericOp)) {
			newNode = new Node();
			newNode->element = getOperandString(*formItr, parameterTable);
			newNode->isOp = false;
			expTree->push(newNode);
		}
	}
	return expTree;
}

string ExpressionTree::preOrder(const Node * x) {
	ostringstream output;
	if (x != NULL) {
		if (x->isOp) {
			output << "(";
		}
		output << x->element << " ";
		output << preOrder(x->left);
		output << preOrder(x->right);
		if (x->isOp) {
			output << ")";
		}
	}
	return output.str();
}

}
