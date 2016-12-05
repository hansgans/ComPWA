//-------------------------------------------------------------------------------
// Copyright (c) 2013 Mathias Michel.
//
// This file is part of ComPWA, check license.txt for details
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the GNU Public License v3.0
// which accompanies this distribution, and is available at
// http://www.gnu.org/licenses/gpl.html
//
// Contributors:
//     Mathias Michel - initial API and implementation
//-------------------------------------------------------------------------------
//! Base class for internal parameter.
/*! \class AbsParameter
 * @file AbsParameter.hpp
 * This class defines the internal container of a parameter.
 * For the use in the function tree, the observer pattern is used and
 * this class takes over the role of the Subject. Therefore the actual
 * implementations of AbsParameter are the ConcreteSubjects of the
 * observer pattern and the TreeNodes take the role of the observers.
 */

#ifndef _ABSPARAMETER_HPP_
#define _ABSPARAMETER_HPP_

//#define LOCAL  __attribute__ ((visibility ("hidden")))

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <fstream>

#ifdef USESERIALIZATION
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/serialization.hpp>
#endif

#include "Core/ParObserver.hpp"

enum ParType { COMPLEX = 1, DOUBLE = 2, INTEGER = 3, BOOL = 4, MDOUBLE = 5, MCOMPLEX = 6, UNDEFINED = 0};
static const char* ParNames[7] = { "UNDEFINED", "COMPLEX", "DOUBLE", "INTEGER", "BOOL", "MDOUBLE", "MCOMPLEX"};

class AbsParameter //: public std::enable_shared_from_this<AbsParameter>
{
public:
	//! Constructor with name of parameter and optional type
	AbsParameter(std::string name, ParType type=ParType::UNDEFINED):name_(name), type_(type){

	}

	//! Destructor
	virtual ~AbsParameter(){
	}

	//! Getter for name of object
	virtual const std::string& GetName() const{
		return name_;
	}

	//! Getter for type of object
	virtual ParType type() const{
		return type_;
	}

	//! Getter for typename of object, to be defined by the actual implementation
	virtual std::string TypeName() const=0;

	//Observer Pattern Functions

	//! Attaches a new TreeNode as Observer
	void Attach(std::shared_ptr<ParObserver> newObserver){
		oberservingNodes.push_back(newObserver);;
	}

	//! Removes TreeNodes not needed as Observer anymore
	void Detach(std::shared_ptr<ParObserver> obsoleteObserver){
		oberservingNodes.erase(std::remove(oberservingNodes.begin(), oberservingNodes.end(), obsoleteObserver), oberservingNodes.end());
	}

	//! Notify all observing TreeNodes that parameter changed
	void Notify(){
		for(std::vector<std::shared_ptr<ParObserver> >::const_iterator iter = oberservingNodes.begin(); iter != oberservingNodes.end(); ++iter)
		{
			if(*iter != std::shared_ptr<ParObserver>())//Ist das richtig????
			{
				(*iter)->Update();
			}
		}
	}

	//! Return shared_pointer pointing to this Parameter
	//std::shared_ptr<AbsParameter> getptr() {
	//    return shared_from_this();
	//}

	//! friend function to stream parameter information to output
	/*!
	 * Declaring the stream-operator << as friend allows to stream parameter
	 * information to the output as easily as a generic type.
	 * \sa make_str(), to_str()
	 */
	friend std::ostream& operator<<( std::ostream& out, std::shared_ptr<AbsParameter> b ){
		return out << b->to_str();
	}

	//! friend function to stream parameter information to output
	/*!
	 * Declaring the stream-operator << as friend allows to stream parameter
	 * information to the output as easily as a generic type.
	 * \sa make_str(), to_str()
	 */
	friend std::ostream& operator<<( std::ostream& out, AbsParameter& b ){
		return out << b.to_str();
	}

	//! A public function returning a string with parameter information
	/*!
	 * This function simply returns the member string out_, which contains
	 * all parameter information. The string gets rebuild with every change
	 * of the parameter.
	 * \return string with parameter information
	 * \sa operator<<, make_str()
	 */
	virtual std::string to_str() {
		return make_str();
	}

	//! A public function returning a string with parameter value
	/*!
	 * This function simply returns the member string outVal_, which contains
	 * the parameter value. The string gets rebuild with every change
	 * of the parameter.
	 * \return string with parameter information
	 * \sa make_str()
	 */
	virtual std::string val_to_str() {
		return make_val_str();
	}

protected:
	std::string name_; /*!< internal name of the parameter */
	ParType type_; /*!< ParType enum for type of parameter */

	std::vector<std::shared_ptr<ParObserver> > oberservingNodes; /*!< list of observers, e.g. TreeNodes */
	//! Interface to output string, to be implemented by parameter implementations
	virtual std::string make_str() =0;
	//! Interface to output value, to be implemented by parameter implementations
	virtual std::string make_val_str() =0;

private:
#ifdef USESERIALIZATION
	friend class boost::serialization::access;
	template<class archive>
	void serialize(archive& ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(name_);
		ar & BOOST_SERIALIZATION_NVP(type_);
	}
#endif

};

#ifdef USESERIALIZATION
BOOST_SERIALIZATION_SHARED_PTR(AbsParameter)
BOOST_CLASS_IMPLEMENTATION( AbsParameter, boost::serialization::object_serializable )
BOOST_CLASS_TRACKING( AbsParameter, boost::serialization::track_never )
//BOOST_SERIALIZATION_ASSUME_ABSTRACT(AbsParameter)
#endif

#endif
