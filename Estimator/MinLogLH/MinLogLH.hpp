//-------------------------------------------------------------------------------
// Copyright (c) 2013 Mathias Michel.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//     Mathias Michel - initial API and implementation
//	   Peter Weidenkaff - Weights and background fractions
//-------------------------------------------------------------------------------

//! Negative Log Likelihood-Estimator.
/*! \class MinLogLH
 * @file MinLogLH.hpp
 * This class calculates a simple -log(LH) of a intensity and a dataset.
 * Data and Model are provided in the constructor using the Amplitude and Data
 * interfaces. The class itself fulfills the Estimator interface.
 */

#ifndef _MINLOGLHBKG_HPP
#define _MINLOGLHBKG_HPP

#include <vector>
#include <memory>
#include <string>

//PWA-Header
#include "Core/Amplitude.hpp"
#include "Core/Event.hpp"
#include "Core/ParameterList.hpp"
#include "Core/FunctionTree.hpp"
#include "DataReader/Data.hpp"
#include "Estimator/Estimator.hpp"

class MinLogLH : public Estimator {

public:

	//! Destructor
	virtual ~MinLogLH() { };

	//! Implementation of ControlParameter::controlParameter
	virtual double controlParameter(ParameterList& minPar);

	/** Create instance of MinLogLH.
	 * A binned efficiency correction is used. We expect that phspSample_ has efficiency values for
	 * each event.
	 *
	 * @param amp_ amplitude
	 * @param data_ data sample
	 * @param phspSample_ phsp sample for normalization. Efficiency values for each point needs
	 *  to be set beforehand.
	 * @param startEvent use #data_ from that position on
	 * @param nEvents number of events to process
	 * @return std::shared_ptr<Data> of existing instance or newly created instance
	 */
	static std::shared_ptr<ControlParameter> createInstance(
			std::shared_ptr<Amplitude> amp,
			std::shared_ptr<Data> data, std::shared_ptr<Data> phspSample,
			unsigned int startEvent=0, unsigned int nEvents=0);

	/** Create instance of MinLogLH.
	 * An unbinned efficiency correction is applied using #accSample_.
	 *
	 * @param amp_ amplitude
	 * @param data_ data sample
	 * @param phspSample_ phsp sample for normalization
	 * @param accSample_ sample of efficiency applied phsp events for unbinned efficiency correction
	 * @param startEvent use #data_ from that position on
	 * @param nEvents number of events to process
	 * @param sigFrac signal fraction in data sample
	 * @return std::shared_ptr<Data> of existing instance or newly created instance
	 */
	static std::shared_ptr<ControlParameter> createInstance(
			std::shared_ptr<Amplitude> amp,
			std::shared_ptr<Data> data, std::shared_ptr<Data> phspSample,
			std::shared_ptr<Data> accSample,
			unsigned int startEvent=0, unsigned int nEvents=0);

	/** Create instance of MinLogLH.
	 * An unbinned efficiency correction is applied using #accSample_.
	 *
	 * @param amp_ amplitude
	 * @param bkg_ background description amplitude
	 * @param data_ data sample
	 * @param phspSample_ phsp sample for normalization
	 * @param accSample_ sample of efficiency applied phsp events for unbinned efficiency correction
	 * @param startEvent use #data_ from that position on
	 * @param nEvents number of events to process
	 * @param sigFrac signal fraction in data sample
	 * @return std::shared_ptr<Data> of existing instance or newly created instance
	 */
	static std::shared_ptr<ControlParameter> createInstance(
			std::vector<std::shared_ptr<Amplitude> > ampVec,
			std::vector<double> frac,
			std::shared_ptr<Data> data, std::shared_ptr<Data> phspSample,
			std::shared_ptr<Data> accSample,
			unsigned int startEvent, unsigned int nEvents);

	/** Set new amplitude to existing instance
	 *
	 * @param amp_ amplitude
	 * @param data_ data sample
	 * @param phspSample_ phsp sample for normalization
	 * @param accSample_ sample of efficiency applied phsp events for unbinned efficiency correction
	 * @param startEvent use #data_ from that position on
	 * @param nEvents number of events to process
	 * @param useFuncTr use FunctionTree yes/no?
	 */
	virtual void setAmplitude(std::shared_ptr<Amplitude> amp,
			std::shared_ptr<Data> data,
			std::shared_ptr<Data> phspSample, std::shared_ptr<Data> accSample,
			unsigned int startEvent=0, unsigned int nEvents=0, bool useFuncTr=0);

	/** Set new amplitude to existing instance
	 *
	 * @param amp_ amplitude
	 * @param bkg_ amplitude
	 * @param data_ data sample
	 * @param phspSample_ phsp sample for normalization
	 * @param accSample_ sample of efficiency applied phsp events for unbinned efficiency correction
	 * @param startEvent use #data_ from that position on
	 * @param nEvents number of events to process
	 * @param useFuncTr use FunctionTree yes/no?
	 */
	virtual void setAmplitude(std::vector<std::shared_ptr<Amplitude> > ampVec,
			std::vector<double> frac,
			std::shared_ptr<Data> data_,
			std::shared_ptr<Data> phspSample_, std::shared_ptr<Data> accSample_,
			unsigned int startEvent=0, unsigned int nEvents=0, bool useFuncTr=0);

	//! Check if tree for LH calculation is available
	virtual bool hasTree() { return _useFunctionTree; }

	//! Get FunctionTree for LH calculation. Check first if its available using MinLogLH::hasTree().
	virtual std::shared_ptr<FunctionTree> getTree() {
		if(!_useFunctionTree)
			throw std::runtime_error("MinLogLH::getTree()  | You requested a "
					"tree which was not constructed!");
		return _tree;
	}

	//! Get Amplitude
	virtual std::vector<std::shared_ptr<Amplitude> > getAmplitudes() {
		return _ampVec;
	}

	//! Should we try to use the function tree? Function tree needs to be implemented in Amplitude
	virtual void setUseFunctionTree(bool t);

	//! Set scale of penalty term
	virtual void setPenaltyScale(double sc, int ampID=0);

	//! Get scale of penalty term
	virtual double getPenaltyScale() { return _penaltyLambda; }

	/** Calculate penalty term
	 * A penalty term can be added to the LH to punish unnecessary complexity of amplitudes in
	 * minimization. The strength of this term can be set via MinLogLH::setPenaltyScale(double).
	 * A detailed description of the approach can be found in arXiv:1505.05133 */
	virtual double calcPenalty();

	//! Get number of events in data set
	virtual int getNEvents() { return nEvts_; }

protected:
	//! Default Constructor
	MinLogLH() { };

	//! Constructor for a vector of amplitudes
	MinLogLH(std::vector<std::shared_ptr<Amplitude> > ampVec,
			std::vector<double> fraction, std::shared_ptr<Data> data,
			std::shared_ptr<Data> phspSample, std::shared_ptr<Data> accSample,
			unsigned int startEvent, unsigned int nEvents);

	//! Constructor for a single amplitude
	MinLogLH(std::shared_ptr<Amplitude> amp, std::shared_ptr<Data> data,
			std::shared_ptr<Data> phspSample, std::shared_ptr<Data> accSample,
			unsigned int startEvent, unsigned int nEvents);

	//! Uses ampTree and creates a tree that calculates the full LH
	virtual void iniLHtree();

	//! Sum up all weights in data set
	void calcSumOfWeights();

private:
	//! Initialize
	void Init();

	//! Reset instance
	void Reset();

	//! Use FunctionTree for LH calculation (yes/no)?
	bool _useFunctionTree;

	//! FunctionTree for Likelihood calculation
	std::shared_ptr<FunctionTree> _tree;

	//! Number of events in data sample
	unsigned int nEvts_;
	//! Number of event in phsp sample
	unsigned int nPhsp_;
	//! Process data sample from position #nStartEvt_ on
	unsigned int nStartEvt_;
	//! Number of events to process in _dataSample sample
	unsigned int nUseEvt_;

	//! Amplitudes
	std::vector<std::shared_ptr<Amplitude> > _ampVec;
	//! Fraction for each amplitude
	std::vector<double> _fraction;

	// Samples
	std::shared_ptr<Data> _dataSample; //! Data sample
	ParameterList _dataSampleList; //! _dataSample stored as ParameterList
	double _sumOfWeights; //! Sum of weights in _dataSample

	std::shared_ptr<Data> _phspSample; //! phsp sample for normalization
	ParameterList _phspSampleList;//! _phspSample stored as ParameterList

	std::shared_ptr<Data> _phspAccSample;//! Phsp sample with applied efficency
	ParameterList _phspAccSampleList;//! _phspAccSample stored as ParameterList
	double _phspAccSampleEff; //! Total efficiency of phsp with applied efficency

	double _penaltyLambda; //! Scale of penalty term
	int _penaltyAmpID; //! ID of amplitude that is used for penalty calc
};

#endif /* _MINLOGLHBKG_HPP */
