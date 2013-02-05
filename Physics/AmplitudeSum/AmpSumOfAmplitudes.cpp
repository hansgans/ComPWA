/***************************************************************************** 
 * Project: RooFit                                                           * 
 *                                                                           * 
 * This code was autogenerated by RooClassFactory                            * 
 *****************************************************************************/ 

// Your description goes here... 

#include <cmath> 

#include "Riostream.h" 
#include "TMath.h" 

#include "RooAbsReal.h" 
#include "RooRealVar.h"
#include "RooAbsCategory.h" 
#include "RooLinkedListIter.h"

#include "qft++.h"

#include "Physics/AmplitudeSum/AmpSumOfAmplitudes.hpp"

using namespace ROOT;

//ClassImp(AmpSumOfAmplitudes);

AmpSumOfAmplitudes::AmpSumOfAmplitudes() :
       RooAbsPdf("Empty","Empty"),
      _pdfList("!pdfs","List of PDFs",this),
      _intList("!intensities","List of intensities",this),
      _phaseList("!phases","List of phases",this),
      _angList("!angdists","List of angular distributions",this)
{

}

 AmpSumOfAmplitudes::AmpSumOfAmplitudes(const char *name, const char *title) :
   RooAbsPdf(name,title), 
  _pdfList("!pdfs","List of PDFs",this),
  _intList("!intensities","List of intensities",this),
  _phaseList("!phases","List of phases",this),
  _angList("!angdists","List of angular distributions",this)
 { 
   _pdfIter = _pdfList.createIterator() ;
   _intIter = _intList.createIterator() ;
   _phaseIter = _phaseList.createIterator() ;
   _angIter = _angList.createIterator() ;
 } 


 AmpSumOfAmplitudes::AmpSumOfAmplitudes(const AmpSumOfAmplitudes& other, const char* name) :  
   RooAbsPdf(other,name), 
   _pdfList("!pdfs",this,other._pdfList),
   _intList("!intensities",this,other._intList),
   _phaseList("!phases",this,other._phaseList),
   _angList("!angdists",this,other._angList)
 { 
   _pdfIter = _pdfList.createIterator() ;
   _intIter = _intList.createIterator() ;
   _phaseIter = _phaseList.createIterator() ;
   _angIter = _angList.createIterator() ;
 } 

 AmpSumOfAmplitudes::~AmpSumOfAmplitudes(){
   //something TODO?
 }

void AmpSumOfAmplitudes::addBW(AmpAbsDynamicalFunction* theRes , RooRealVar &r, RooRealVar &phi, AmpWigner* theAng) {
  _pdfList.add(*theRes);
  _intList.add(r);
  _phaseList.add(phi);
  _angList.add(*theAng);
}

void AmpSumOfAmplitudes::addBW(AmpAbsDynamicalFunction* theRes , RooRealVar &r, RooRealVar &phi) {
  _pdfList.add(*theRes);
  _intList.add(r);
  _phaseList.add(phi);
  /*RooRealVar beta("beta", "mass", 0.);
  RooRealVar j ("j", "j", 0.);
  RooRealVar m ("m", "m", 0.);
  RooRealVar n ("n", "n", 0.) ; 
  _angList.add(AmpWigner("none", "none", beta, j, m, n));*/
  _angList.add(AmpWigner());
}


 Double_t AmpSumOfAmplitudes::evaluate() const 
 { 
   // ENTER EXPRESSION IN TERMS OF VARIABLE ARGUMENTS HERE 
   RooComplex res;

   AmpAbsDynamicalFunction *pdf;
   RooRealVar *theInt;
   RooRealVar *thePhase;
   AmpWigner *ang;

   _pdfIter->Reset();
   _intIter->Reset();
   _phaseIter->Reset();
   _angIter->Reset();

   //   TIterator* _pdfIter = _pdfList.createIterator() ;
   //   AmpRelBreitWignerRes *pdf;


   while((pdf      = (AmpAbsDynamicalFunction*)_pdfIter->Next()) &&
	 (theInt   = (RooRealVar*)_intIter->Next())        && 
	 (thePhase = (RooRealVar*)_phaseIter->Next())      &&
         (ang      = (AmpWigner*)_angIter->Next())  ) {
     double a = theInt->getVal();
     double phi = thePhase->getVal();
     RooComplex eiphi (cos(phi), sin(phi));

     res = res + pdf->evaluate() * a * eiphi * ang->evaluate();
   }

   return res.abs2() ; 
 } 

 Double_t AmpSumOfAmplitudes::evaluateSlice(RooComplex* reso, unsigned int nResos, unsigned int subSys=1) const 
 { 
   // ENTER EXPRESSION IN TERMS OF VARIABLE ARGUMENTS HERE 
   RooComplex res;

   AmpAbsDynamicalFunction *pdf;
   RooRealVar *theInt;
   RooRealVar *thePhase;
   AmpWigner *ang;

   _pdfIter->Reset();
   _intIter->Reset();
   _phaseIter->Reset();
   _angIter->Reset();

   //   TIterator* _pdfIter = _pdfList.createIterator() ;
   //   AmpRelBreitWignerRes *pdf;
   int itReso=0, sys=0;


   while((pdf      = (AmpAbsDynamicalFunction*)_pdfIter->Next()) &&
	 (theInt   = (RooRealVar*)_intIter->Next())        && 
	 (thePhase = (RooRealVar*)_phaseIter->Next())      &&
         (ang      = (AmpWigner*)_angIter->Next())  ) {
     double a = theInt->getVal();
     double phi = thePhase->getVal();
     RooComplex eiphi (cos(phi), sin(phi));
     if(itReso<2) sys = 0;
     else if(itReso<3) sys = 1;
     //else sys = 2;
     //sys = itReso;

     if(pdf->isSubSys(subSys))
       res = res + reso[sys] * ang->evaluate();
     else
       res = res + pdf->evaluate() * a * eiphi * ang->evaluate();

     itReso++;
   }

   return res.abs2() ; 
 } 

 /*Double_t AmpSumOfAmplitudes::evaluatePhi() const 
 { 
   // ENTER EXPRESSION IN TERMS OF VARIABLE ARGUMENTS HERE 
   RooComplex res;

   AmpRelBreitWignerRes *pdf;
   RooRealVar *theInt;
   RooRealVar *thePhase;
   AmpWigner *ang;

   _pdfIter->Reset();
   _intIter->Reset();
   _phaseIter->Reset();
   _angIter->Reset();

   //   TIterator* _pdfIter = _pdfList.createIterator() ;
   //   AmpRelBreitWignerRes *pdf;


   while((pdf      = (AmpRelBreitWignerRes*)_pdfIter->Next()) &&
	 (theInt   = (RooRealVar*)_intIter->Next())        && 
	 (thePhase = (RooRealVar*)_phaseIter->Next())      &&
         (ang      = (AmpWigner*)_angIter->Next())  ) {
     double a = theInt->getVal();
     double phi = thePhase->getVal();
     RooComplex eiphi (cos(phi), sin(phi));

     res = res + pdf->evaluate() * a * eiphi * ang->evaluate();
   }

   return atan2(res.im(),res.re()); 
 } */
