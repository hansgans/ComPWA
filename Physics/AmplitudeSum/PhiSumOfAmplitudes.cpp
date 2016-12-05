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

#include <cmath> 

#include "qft++.h"

#include "Physics/AmplitudeSum/PhiSumOfAmplitudes.hpp"

namespace COMPWA {
 PhiSumOfAmplitudes::PhiSumOfAmplitudes(const char *name) : _name(name)
 { 
//   _pdfIter = _pdfList.createIterator() ;
//   _intIter = _intList.createIterator() ;
//   _phaseIter = _phaseList.createIterator() ;
//   _angIter = _angList.createIterator() ;
 } 


 PhiSumOfAmplitudes::PhiSumOfAmplitudes(const PhiSumOfAmplitudes& other, const char* name) :  
						 _name(name),
   _pdfList(other._pdfList),
   _intList(other._intList),
   _phaseList(other._phaseList),
   _angList(other._angList)
 { 
//   _pdfIter = _pdfList.createIterator() ;
//   _intIter = _intList.createIterator() ;
//   _phaseIter = _phaseList.createIterator() ;
//   _angIter = _angList.createIterator() ;
 } 
void PhiSumOfAmplitudes::addBW(std::shared_ptr<AmpAbsDynamicalFunction> theRes , std::shared_ptr<DoubleParameter> r, std::shared_ptr<DoubleParameter> phi, std::shared_ptr<AmpWigner2> theAng) {
  _pdfList.push_back(theRes);
  _intList.push_back(r);
  _phaseList.push_back(phi);
  _angList.push_back(theAng);
}

void PhiSumOfAmplitudes::addBW(std::shared_ptr<AmpAbsDynamicalFunction> theRes , std::shared_ptr<DoubleParameter> r, std::shared_ptr<DoubleParameter> phi) {
  _pdfList.push_back(theRes);
  _intList.push_back(r);
  _phaseList.push_back(phi);
  _angList.push_back(std::shared_ptr<AmpWigner2>(new AmpWigner2(1,0)));
}

double PhiSumOfAmplitudes::evaluate() const
 { 
   // ENTER EXPRESSION IN TERMS OF VARIABLE ARGUMENTS HERE 
   std::complex<double> res(0,0);

//   AmpRelBreitWignerRes *pdf;
//   RooRealVar *theInt;
//   RooRealVar *thePhase;
//   AmpWigner *ang;

//   _pdfIter->Reset();
//   _intIter->Reset();
//   _phaseIter->Reset();
//   _angIter->Reset();

   //   TIterator* _pdfIter = _pdfList.createIterator() ;
   //   AmpRelBreitWignerRes *pdf;


//   while((pdf      = (AmpRelBreitWignerRes*)_pdfIter->Next()) &&
//	 (theInt   = (RooRealVar*)_intIter->Next())        &&
//	 (thePhase = (RooRealVar*)_phaseIter->Next())      &&
//         (ang      = (AmpWigner*)_angIter->Next())  ) {
//     double a = theInt->getVal();
//     double phi = thePhase->getVal();
//     std::complex<double> eiphi(cos(phi), sin(phi));
//
//     res = res + pdf->evaluate() * a * eiphi * ang->evaluate();
//   }

   return fabs(3.1416+atan2(res.imag(),res.real()));
 } 
}

