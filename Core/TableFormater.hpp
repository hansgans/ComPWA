/*
 * TablePrinter.cxx
 *
 *  Created on: Jan 10, 2014
 *      Author: weidenka
 */

#ifndef TABLEFORMATER_CXX_
#define TABLEFORMATER_CXX_
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

//#include <boost/spirit/include/karma.hpp>
//using namespace boost::spirit::karma;
using namespace std;

class TableFormater {

public:
	TableFormater(std::ostream* output): curRow(0),curCol(0),totalWidth(0){
		out = output;
	};
	void delim(){
		*out<<"|";
		for(unsigned int i=0;i<totalWidth-1; i++) *out << "-" ;
		*out<<"|"<<endl;
	}
	void footer(){ delim(); }
	void header(){
		delim();
		for(unsigned int i=0;i<columnTitle.size();i++) *out << "| "<<std::setw(columnWidth[i])<<columnTitle[i]<<" ";
		*out<<"|"<<std::endl;
		delim();
	};
	void addColumn(std::string title, unsigned int fixlength=999){
		unsigned int length;
		if(fixlength!=999) length = fixlength;
		else length=title.length();
		columnWidth.push_back(length);
		totalWidth+=length+3;
		columnTitle.push_back(title);
	};
	//	friend std::ostream& operator<< (std::ostream &out, FitResult &fitres){ out<<fitres.finalLH; return out;};
	TableFormater& operator<<(DoubleParameter in){
		*out << "| ";
		if(in.HasError()){
			unsigned int halfWidth = (unsigned int)(columnWidth[curCol]-2)/2;
//			std::cout<<"111 "<<halfWidth<<std::endl;
			*out << std::setw(halfWidth) << in.GetValue() << "+-";
			*out << std::setw(halfWidth) << in.GetError() << " ";
		} else {
//			std::cout<<"131 "<<std::endl;
			*out << std::setw(columnWidth[curCol]) << in.GetValue() << " ";
		}
		curCol++;
		if(curCol==columnWidth.size()) {
			*out<<"|"<<std::endl;
			curRow++; curCol=0;
			//			delim();
		}
		return *this;
	};
	template<typename T> TableFormater& operator<<(T in){
		*out << "| " <<std::setw(columnWidth[curCol]) << in <<" ";
		curCol++;
		if(curCol==columnWidth.size()) {
			*out<<"|"<<std::endl;
			curRow++; curCol=0;
			//			delim();
		}
		return *this;
	};
private:
	ostream* out;
	std::vector<unsigned int> columnWidth;
	std::vector<std::string> columnTitle;

	unsigned int curRow;
	unsigned int curCol;
	unsigned int totalWidth;
protected:
};


#endif /* TABLEFORMATER_CXX_ */