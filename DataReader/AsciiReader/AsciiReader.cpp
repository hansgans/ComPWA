//-------------------------------------------------------------------------------
// Copyright (c) 2013 Florian Feldbauer.
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//     Florian Feldbauer - initial API and implementation
//-------------------------------------------------------------------------------
//_____ I N C L U D E S _______________________________________________________

// ANSI C headers
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

// 3rd party headers
//#include "external/qft++/include/Tensor.h"

// local headers
#include "Core/Exceptions.hpp"
#include "DataReader/AsciiReader/AsciiReader.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

namespace ComPWA {
// Constructors and destructors
AsciiReader::AsciiReader( const std::string inConfigFile, const int particles  )
{

  std::ifstream currentStream;
  currentStream.open ( inConfigFile.c_str() );

  if ( !currentStream ) throw BadConfig( "Can not open " + inConfigFile );

  while( !currentStream.eof() ) {
    double e, px, py, pz;
    Event newEvent;

    for ( int parts = 0; parts < particles; parts++ ) {
      currentStream >> px >> py >> pz >> e;
      newEvent.addParticle( Particle( px, py, pz, e ) );
    }

    if (!currentStream.fail()) {
      fEvents.push_back( newEvent );
      // for ( parts = 0; parts < linesToSkip; parts++ )
      //   currentStream >> px >> py >> pz >> e;
    }
  }
  currentStream.close();
}

AsciiReader::~AsciiReader()
{
  fEvents.clear();
}

AsciiReader* AsciiReader::Clone() const
{
	//TODO: implement virtual functions and uncomment the following
	//	return new AsciiReader(*this);
//	return new AsciiReader();
}

AsciiReader* AsciiReader::EmptyClone() const
{
//	return new AsciiReader();
}
}
