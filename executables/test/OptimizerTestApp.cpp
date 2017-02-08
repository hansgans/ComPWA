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
//! Test-Application of the Optimizer-IF.
/*!
 * @file OptimizerTestApp.cpp
 * This tiny application tests the interface to the Optimizers Minuit2 and Geneva.
 * The test dataset is generated in the PolyFit.hpp class, which creates smeared
 * 1-dim data according to a polynomial function. Then the Optimizer-IF implemen-
 * tations Minuit2 (MinuitIF.hpp) and Geneva (GenevaIF.hpp) are used one after
 * the other to fit the same polynomial to the smeared points. As a result the
 * optimized parameters are printed. Note: In this example Minuit2 uses the final
 * parameters of Geneva as starting values!
 */

// Standard header files go here
#include <iostream>
#include <cmath>
#include <sstream>
#include <vector>
#include <string>
#include <memory>

// Minimizer Interface header files go here
#include "Optimizer/Minuit2/MinuitIF.hpp"
#include "Optimizer/Geneva/GenevaIF.hpp"
#include "Core/ParameterList.hpp"
#include "Core/Parameter.hpp"
#include "Core/FitResult.hpp"
#include "Core/Logging.hpp"
#include "Core/RunManager.hpp"
#include "Core/TwoBodyKinematics.hpp"
#include "DataReader/RootGenerator/RootGenerator.hpp"
#include "DataReader/RootReader/RootReader.hpp"
#include "Estimator/MinLogLH/MinLogLH.hpp"

#include "Core/Amplitude.hpp"

#include "TH1D.h"
#include "TCanvas.h"

using ComPWA::DoubleParameter;
using ComPWA::TwoBodyKinematics;
using ComPWA::DataReader::Data;
using ComPWA::DataReader::RootReader::RootReader;
using ComPWA::ParameterList;
using ComPWA::Generator;
using ComPWA::Amplitude;
using ComPWA::Optimizer::Optimizer;
using ComPWA::FitResult;
using ComPWA::dataPoint;

/************************************************************************************************/
/**
 * The main function.
 */
<<<<<<< HEAD
int main(int argc, char **argv){
	std::cout << "  ComPWA Copyright (C) 2013  Mathias Michel " << std::endl;
	std::cout << "  This program comes with ABSOLUTELY NO WARRANTY; for details see license.txt" << std::endl;
	std::cout << std::endl;

	TwoBodyKinematics::createInstance("D0","K-","pi+",0.1);

	std::shared_ptr<Generator> gen = std::shared_ptr<Generator>(
			new UniformTwoBodyGenerator(1.8*1.8,2.0*2.0,1234567)
	);
	RunManager run;
	run.setGenerator(gen);

	DoubleParameter true_mass("mean",1.865,1.5,2.0,0.1);
	DoubleParameter true_width("width",0.004,0,0.1,0.01);
	DoubleParameter mass("mean",1.8,1.5,2.0,0.1);
	DoubleParameter width("width",0.05,0,0.1,0.01);
	ParameterList truePar, fitPar, initialPar;
	std::shared_ptr<Amplitude> truegaus(new GaussAmp("gaus",true_mass,true_width));
	truegaus->FillParameterList(truePar);
	std::shared_ptr<Amplitude> gaus(new GaussAmp("gaus",mass,width));
	gaus->FillParameterList(fitPar);
	gaus->FillParameterList(initialPar);

	run.setAmplitude(truegaus);
	std::shared_ptr<Data> toyData(new RootReader());
	run.setData(toyData);
	run.generate(1000);
	run.setAmplitude(gaus);

	std::shared_ptr<Data> toyPhsp(new RootReader());
	run.setPhspSample(toyPhsp);
	run.generatePhsp(100000);

	std::shared_ptr<ControlParameter> myFit(MinLogLH::createInstance( gaus, toyData, toyPhsp ));

	//--------------------------Minimizer IF --------------------------------------------------------
	std::shared_ptr<Optimizer> geneva_opti(new GenevaIF(myFit));
	std::shared_ptr<Optimizer> minuit_opti(new MinuitIF(myFit,fitPar));

	BOOST_LOG_TRIVIAL(info) << "Starting Parameters:";
	BOOST_LOG_TRIVIAL(info) << fitPar.to_str();
	BOOST_LOG_TRIVIAL(info) << "Running Geneva optimizer:";
	std::shared_ptr<FitResult> geneva_result = geneva_opti->exec(fitPar);
	geneva_result->setInitialParameters(initialPar);
	geneva_result->setTrueParameters(truePar);
	std::shared_ptr<Data> geneva_fit(new RootReader());
	run.setData(geneva_fit);
	run.generate(1000);

	gaus->FillParameterList(initialPar);

	BOOST_LOG_TRIVIAL(info) << "Running Minuit optimizer:";
	std::shared_ptr<FitResult> minuit_result = minuit_opti->exec(fitPar);
	minuit_result->setInitialParameters(initialPar);
	minuit_result->setTrueParameters(truePar);
	std::shared_ptr<Data> minuit_fit(new RootReader());
	run.setData(minuit_fit);
	run.generate(1000);

	geneva_result->print();
	minuit_result->print();

	//Plotting
	TH1D* h_data = new TH1D("data","data",100,1.8,2.0);
	TH1D* h_minuit = new TH1D("minuit","minuit",100,1.8,2.0);
	h_minuit->SetLineColor(kGreen);
	TH1D* h_geneva = new TH1D("geneva","geneva",100,1.8,2.0);
	h_minuit->SetLineColor(kBlue);
	for(int i =0; i<toyData->getNEvents(); i++){
		dataPoint p(toyData->getEvent(i));
		h_data->Fill(sqrt(p.getVal(0)));
	}
	for(int i =0; i<geneva_fit->getNEvents(); i++){
		dataPoint p(geneva_fit->getEvent(i));
		h_geneva->Fill(sqrt(p.getVal(0)));
	}
	for(int i =0; i<minuit_fit->getNEvents(); i++){
		dataPoint p(minuit_fit->getEvent(i));
		h_minuit->Fill(sqrt(p.getVal(0)));
	}
	TCanvas s;
	h_data->Draw();
	h_minuit->Draw("Sames");
	h_geneva->Draw("Sames");
	s.Print("OptimizerTest.root");
	return 0;
=======
int main(int argc, char **argv) {
  std::cout << "  ComPWA Copyright (C) 2013  Mathias Michel " << std::endl;
  std::cout
      << "  This program comes with ABSOLUTELY NO WARRANTY; for details see license.txt"
      << std::endl;
  std::cout << std::endl;

  TwoBodyKinematics::createInstance("D0", "K-", "pi+", 0.1);

  std::shared_ptr<Generator> gen = std::shared_ptr<Generator>(
      new ComPWA::DataReader::RootGenerator::UniformTwoBodyGenerator(1.8 * 1.8,
          2.0 * 2.0, 1234567));
  ComPWA::RunManager run;
  run.setGenerator(gen);

  DoubleParameter true_mass("mean", 1.865, 1.5, 2.0, 0.1);
  DoubleParameter true_width("width", 0.004, 0, 0.1, 0.01);
  DoubleParameter mass("mean", 1.8, 1.5, 2.0, 0.1);
  DoubleParameter width("width", 0.05, 0, 0.1, 0.01);
  ParameterList truePar, fitPar, initialPar;
  std::shared_ptr<Amplitude> truegaus(
      new ComPWA::GaussAmp("gaus", true_mass, true_width));
  truegaus->copyParameterList(truePar);
  std::shared_ptr<Amplitude> gaus(new ComPWA::GaussAmp("gaus", mass, width));
  gaus->copyParameterList(fitPar);
  gaus->copyParameterList(initialPar);

  run.setAmplitude(truegaus);
  std::shared_ptr<Data> toyData(new RootReader());
  run.setData(toyData);
  run.generate(1000);
  run.setAmplitude(gaus);

  std::shared_ptr<Data> toyPhsp(new RootReader());
  run.setPhspSample(toyPhsp);
  run.generatePhsp(100000);

  std::shared_ptr<ComPWA::Optimizer::ControlParameter> myFit(
      ComPWA::Estimator::MinLogLH::MinLogLH::createInstance(gaus, toyData,
          toyPhsp));

  //--------------------------Minimizer IF --------------------------------------------------------
  std::shared_ptr<Optimizer> geneva_opti(
      new ComPWA::Optimizer::Geneva::GenevaIF(myFit));
  std::shared_ptr<Optimizer> minuit_opti(
      new ComPWA::Optimizer::Minuit2::MinuitIF(myFit, fitPar));

  /*std::vector<std::shared_ptr<Optimizer> > myMinimizerList;
  // Add minimizers
  if (whichMinimizer=="Geneva") myMinimizerList.push_back(std::shared_ptr<Optimizer> (new GenevaIF(myFit)));
  else if (whichMinimizer=="Minuit") myMinimizerList.push_back(std::shared_ptr<Optimizer> (new MinuitIF(myFit,par)));
  else if (whichMinimizer=="all") {
    myMinimizerList.push_back(std::shared_ptr<Optimizer> (new GenevaIF(myFit)));
    myMinimizerList.push_back(std::shared_ptr<Optimizer> (new MinuitIF(myFit,par)));
    std::cout << "TEST" << std::endl;
  }else{
   std::cout << "Minimizer\t" << whichMinimizer << "\tdoesn't exist" << std::endl;
   return 0;
  }*/

  BOOST_LOG_TRIVIAL(info)<< "Starting Parameters:";
  BOOST_LOG_TRIVIAL(info)<< fitPar.to_str();
  BOOST_LOG_TRIVIAL(info)<< "Running Geneva optimizer:";
  std::shared_ptr<FitResult> geneva_result = geneva_opti->exec(fitPar);
  geneva_result->setInitialParameters(initialPar);
  geneva_result->setTrueParameters(truePar);
  std::shared_ptr<Data> geneva_fit(new RootReader());
  run.setData(geneva_fit);
  run.generate(1000);

  gaus->copyParameterList(initialPar);

  BOOST_LOG_TRIVIAL(info)<< "Running Minuit optimizer:";
  std::shared_ptr<FitResult> minuit_result = minuit_opti->exec(fitPar);
  minuit_result->setInitialParameters(initialPar);
  minuit_result->setTrueParameters(truePar);
  std::shared_ptr<Data> minuit_fit(new RootReader());
  run.setData(minuit_fit);
  run.generate(1000);

  geneva_result->print();
  minuit_result->print();

  //Plotting
  TH1D* h_data = new TH1D("data", "data", 100, 1.8, 2.0);
  TH1D* h_minuit = new TH1D("minuit", "minuit", 100, 1.8, 2.0);
  h_minuit->SetLineColor(kGreen);
  TH1D* h_geneva = new TH1D("geneva", "geneva", 100, 1.8, 2.0);
  h_minuit->SetLineColor(kBlue);
  for (int i = 0; i < toyData->getNEvents(); i++) {
    dataPoint p(toyData->getEvent(i));
    h_data->Fill(sqrt(p.getVal(0)));
  }
  for (int i = 0; i < geneva_fit->getNEvents(); i++) {
    dataPoint p(geneva_fit->getEvent(i));
    h_geneva->Fill(sqrt(p.getVal(0)));
  }
  for (int i = 0; i < minuit_fit->getNEvents(); i++) {
    dataPoint p(minuit_fit->getEvent(i));
    h_minuit->Fill(sqrt(p.getVal(0)));
  }
  TCanvas s;
  h_data->Draw();
  h_minuit->Draw("Sames");
  h_geneva->Draw("Sames");
  s.Print("OptimizerTest.root");
  return 0;
>>>>>>> dd7eb340b0f73d2b07005f687e586aae14fbc9fa
}