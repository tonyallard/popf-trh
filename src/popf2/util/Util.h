/*
 * Util.h
 *
 *  Created on: 31 Aug 2015
 *      Author: tony
 */

#ifndef COLIN_UTIL_UTIL_H_
#define COLIN_UTIL_UTIL_H_

namespace Util {

template<typename T1, typename T2, typename T3 = T1>
struct triple {
	T1 first;
	T2 second;
	T3 third;
	void make_triple(T1 first, T2 second, T3 third) {
		this->first = first;
		this->second = second;
		this->third = third;
	};

	bool operator==(const triple<T1, T2, T3> & other) const{
		if (first != other.first) {
			return false;
		}
		if (second != other.second) {
			return false;
		}
		if (third != other.third) {
			return false;
		}
		return true;
	}

	bool operator!=(const triple<T1, T2, T3> & other) const{
		return !((*this) == other);
	}

	bool operator<(const triple<T1, T2, T3> & other) const{
		return ((first < other.first) ||
			(!(other.first < first) && (second < other.second)) ||
			(!(other.first < first) && !(other.second < second) 
			&& (third < other.third)));
	}

	bool operator>(const triple<T1, T2, T3> & other) const{
		return other < (*this);
	}

	bool operator<=(const triple<T1, T2, T3> & other) const{
		return !(other < (*this));
	}

	bool operator>=(const triple<T1, T2, T3> & other) const{
		return !((*this) < other);
	}
};

bool isDouble(const char* str);

}
#endif /* COLIN_UTIL_UTIL_H_ */
