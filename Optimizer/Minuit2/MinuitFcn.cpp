//-------------------------------------------------------------------------------
// Copyright (c) 2013 Mathias Michel.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//     Mathias Michel - initial API and implementation
//-------------------------------------------------------------------------------
#include <cassert>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>

#include "Core/ParameterList.hpp"
#include "Core/Parameter.hpp"
#include "Core/Logging.hpp"
#include "Optimizer/Minuit2/MinuitFcn.hpp"
#include "Optimizer/ControlParameter.hpp"

namespace COMPWA {
using namespace ROOT::Minuit2;

MinuitFcn::MinuitFcn(std::shared_ptr<ControlParameter> myData, ParameterList& parList) :
		  _myDataPtr(myData), _parList(parList)
{
	if (0==_myDataPtr)
		throw std::runtime_error("MinuitFcn::MinuitFcn() | Data pointer is 0!");
}

MinuitFcn::~MinuitFcn()
{

}

double MinuitFcn::operator()(const std::vector<double>& x) const
{
	//ParameterList par;
	std::ostringstream paramOut;
	for(unsigned int i=0; i<x.size(); i++){
		std::shared_ptr<DoubleParameter> actPat = _parList.GetDoubleParameter(i);
		//std::cout<<i<<" "<<actPat->GetName()<<" "<<actPat->GetValue()
		//<<" "<<x[i]<<" "<<actPat->IsFixed()<<std::endl;
		if(!actPat->IsFixed())
			if(x[i]==x[i]){
				actPat->SetValue(x[i]);
				paramOut << x[i] << " ";//print only free parameters
			}
	}
	auto start = std::clock();
	double result = _myDataPtr->controlParameter(_parList);
	auto sec = (std::clock() - start)/CLOCKS_PER_SEC;

	BOOST_LOG_TRIVIAL(info) << std::setprecision(10)
	<< "MinuitFcn: -log(L) = "<< result
	<< std::setprecision(4)
	<<" Time: "<<sec<<"s"
	<<" nCalls: "<<_myDataPtr->nCalls();
	BOOST_LOG_TRIVIAL(debug) << "Parameters: "<<paramOut.str();

	return result;
}

double MinuitFcn::Up() const
{
	return 0.5; //TODO: Setter, LH 0.5, Chi2 1.
}

}

