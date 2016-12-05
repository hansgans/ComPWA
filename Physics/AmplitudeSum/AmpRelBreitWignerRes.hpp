//-------------------------------------------------------------------------------
// Copyright (c) 2013 Mathias Michel.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//     Mathias Michel - initial API and implementation
//		Peter Weidenkaff - correct nominator, using dataPoint for data handling
//-------------------------------------------------------------------------------
//****************************************************************************
// Class for defining the relativistic Breit-Wigner resonance model, which
// includes the use of Blatt-Weisskopf barrier factors.
//****************************************************************************

// --CLASS DESCRIPTION [MODEL] --
// Class for defining the relativistic Breit-Wigner resonance model, which
// includes the use of Blatt-Weisskopf barrier factors.

#ifndef AMP_REL_BREIT_WIGNER_RES
#define AMP_REL_BREIT_WIGNER_RES

#include <vector>

#include "Core/Functions.hpp"
#include "Core/Exceptions.hpp"
#include "Core/FunctionTree.hpp"
#include "Physics/AmplitudeSum/AmpAbsDynamicalFunction.hpp"
#include "Physics/AmplitudeSum/AmpWigner2.hpp"
#include "Physics/AmplitudeSum/NonResonant.hpp"

namespace COMPWA {
class AmpRelBreitWignerRes : public AmpAbsDynamicalFunction {
public:

	AmpRelBreitWignerRes( normStyle nS=normStyle::one, int calls=30000 ) :
		AmpAbsDynamicalFunction( nS, calls ) { };

	AmpRelBreitWignerRes( const char *name,
			unsigned int varIdA, unsigned int varIdB,
			std::shared_ptr<DoubleParameter> mag,
			std::shared_ptr<DoubleParameter> phase,
			std::shared_ptr<DoubleParameter> mass,
			int spin, int m, int n, int P, int C,
			std::string mother, std::string particleA, std::string particleB,
			std::shared_ptr<DoubleParameter> width,
			std::shared_ptr<DoubleParameter> mesonRadius,
			std::shared_ptr<DoubleParameter> motherRadius,
			formFactorType type = formFactorType::BlattWeisskopf,
			int nCalls=30000, normStyle nS=normStyle::one );

	virtual ~AmpRelBreitWignerRes();

	//! Clone function
	virtual AmpRelBreitWignerRes* Clone(std::string newName="") const{
		auto tmp = (new AmpRelBreitWignerRes(*this));
		if(newName != "")
			tmp->SetName(newName);
		return tmp;
	}

	//! Configure resonance from ptree
	virtual void Configure(boost::property_tree::ptree::value_type const& v,
			ParameterList& list);
	//! Save resonance from to ptree
	virtual void Save(boost::property_tree::ptree &pt);

	//! Trigger recalculation of normalization
	virtual void CheckModified();

	std::string to_str() const;

	//! Calculation integral |dynamical amplitude|^2
	virtual double GetIntegral();

	//! Get resonance width
	double GetWidth() const { return _width->GetValue(); }
	/** Breit-Wigner function
	 *
	 * The dynamical function implemented here is taken from PDG2014 (Eq.47-22) for
	 * the one channel case.
	 * @J is angular momentum between A&B. In case A&B have spin 0 this is the spin for the resonace.
	 *
	 * @param mSq Invariant mass
	 * @param mR Resonance mass
	 * @param ma Mass particle A
	 * @param mb Mass particle B
	 * @param width  Width of resonance
	 * @param J Angular momentum between A&B
	 * @param mesonRadius Scale of interaction range
	 * @return
	 */
	static std::complex<double> dynamicalFunction(double mSq, double mR, double ma, double mb,
			double width, unsigned int J, double mesonRadius,
			formFactorType ffType=formFactorType::BlattWeisskopf);

	virtual std::complex<double> EvaluateAmp(dataPoint& point);

	virtual std::shared_ptr<FunctionTree> SetupTree(
			ParameterList& sample, ParameterList& toySample,std::string suffix);
protected:
	//! Resonance width
	std::shared_ptr<DoubleParameter> _width;
	double tmp_width;
	bool _width_writeByName;
};

class BreitWignerStrategy : public Strategy {
public:
	BreitWignerStrategy(const std::string resonanceName) :
		Strategy(ParType::MCOMPLEX), name(resonanceName) { }

	virtual const std::string to_str() const {
		return ("relativistic BreitWigner of "+name);
	}

	virtual bool execute(ParameterList& paras,
			std::shared_ptr<AbsParameter>& out);

protected:
	std::string name;
};
}
#endif
