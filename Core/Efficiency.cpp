//-------------------------------------------------------------------------------
// Copyright (c) 2013 Peter Weidenkaff.
#include "Core/Logging.hpp"
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//		Peter Weidenkaff -
//-------------------------------------------------------------------------------
#include "Core/Logging.hpp"
#include "Core/Efficiency.hpp"

Efficiency::Efficiency()
{

}

Efficiency::~Efficiency()
{

}

UnitEfficiency::UnitEfficiency()
{
	BOOST_LOG_TRIVIAL(info)<<"Efficiency: creating UnitEfficiency!";
};
