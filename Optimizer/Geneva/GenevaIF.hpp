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
//! Wrapper of the Geneva Optimizer library.
/*! \class GenevaIF
 * @file GenevaIF.hpp
 * This class provides a wrapper around the Geneva library. It fulfills the
 * Optimizer interface to be easily adapted to other modules. Parameters for the
 * optimization have to be provided in a config-file, the data needs to be
 * provided with the ControlParameter interface.
*/

#ifndef _GENEVAIF_HPP
#define _GENEVAIF_HPP


// Boost header files go here

// Geneva header files go here

#include <vector>
#include <memory>
#include <iostream>
#include "Core/ParameterList.hpp"
#include "Core/Parameter.hpp"
#include "Optimizer/Geneva/GStartIndividual.hpp"
#include "Optimizer/ControlParameter.hpp"
#include "Optimizer/Optimizer.hpp"

// Geneva header files go here
#include "geneva/Go2.hpp"

namespace ComPWA {
class GenevaIF : public Optimizer {

public:
  /// Default Constructor (0x0)
  GenevaIF(std::shared_ptr<ControlParameter> theData, std::string inConfigFileDir="test/config/");
  virtual std::shared_ptr<FitResult> exec(ParameterList& par) ;

  /** Destructor */
  virtual ~GenevaIF();

  virtual void setServerMode();
  virtual void setClientMode(std::string serverip="localhost", unsigned int serverport=10000);

private:
  std::shared_ptr<ControlParameter> _myData;
  std::string configFileDir;
  Gem::Geneva::execMode parallelizationMode;
  Gem::Common::serializationMode serMode;
  bool clientMode;
  std::string ip;
  unsigned int port;
};
}
#endif /* _GENEVAIF_HPP */
