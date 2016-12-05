//-------------------------------------------------------------------------------
// Copyright (c) 2013 michel.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//     michel - initial API and implementation
//-------------------------------------------------------------------------------
//! Simple \f$\chi^{2}\f$-Estimator.
/*! \class ChiOneD
 * @file ChiOneD.hpp
 * This class calculates a simple \f$\chi^{2}\f$ of a intensity and a dataset.
 * Data and Model are provided in the constructor using the Amplitude and Data
 * interfaces. The class itself fulfills the Estimator interface.
*/

#ifndef _EIFChiOneD_HPP
#define _EIFChiOneD_HPP

#include <vector>
#include <memory>
#include <string>

//PWA-Headers
#include "Core/Amplitude.hpp"
#include "Core/Event.hpp"
#include "Core/ParameterList.hpp"
#include "Estimator/Estimator.hpp"
#include "DataReader/Data.hpp"

namespace COMPWA {
class ChiOneD : public Estimator {

public:
  static std::shared_ptr<ControlParameter> createInstance(std::shared_ptr<Amplitude>, std::shared_ptr<Data>);
  virtual double controlParameter(ParameterList& minPar);

  /** Destructor */
  virtual ~ChiOneD();

protected:
  /// Default Constructor (0x0)
  ChiOneD(std::shared_ptr<Amplitude>, std::shared_ptr<Data>);

private:
  std::shared_ptr<Amplitude> pPIF_;
  std::shared_ptr<Data> pDIF_;

};
}
#endif /* _EIFChiOneD_HPP */
