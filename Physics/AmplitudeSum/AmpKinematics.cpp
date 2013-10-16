/*
 * AmpKinematics.cpp
 *
 *  Created on: Oct 12, 2013
 *      Author: weidenka
 */
#include "Physics/AmplitudeSum/AmpKinematics.hpp"

AmpKinematics::AmpKinematics(const AmpKinematics& other) :
_ma(other._ma),
_mb(other._mb),
_mc(other._mc),
_M(other._M),
_mR(other._mR),
_type(other._type),
_spin(other._spin),
_m(other._m),
_n(other._n),
_mesonRadius(other._mesonRadius),
_motherRadius(other._motherRadius)
{

};
AmpKinematics::AmpKinematics(double mR, int spin, int m, int n, barrierType type, double mesonRadius=1.5, double motherRadius=1.5) :
						_ma(-999), _mb(-999), _mc(-999), _M(-999),
						_mR(mR),
						_type(type),
						_spin(spin),_m(m),_n(n),
						_mesonRadius(mesonRadius),
						_motherRadius(motherRadius)
{

};
AmpKinematics::AmpKinematics(double ma, double mb , double mc, double M, double mR, barrierType type, int spin, int m, int n, double mesonR=1.5, double motherR=1.5) :
						_ma(ma), _mb(mb), _mc(mc),
						_M(M),
						_mR(mR),
						_type(type),
						_spin(spin),_m(m),_n(n),
						_mesonRadius(mesonR),
						_motherRadius(motherR)
{

};

void AmpKinematics::setDecayMasses(double ma, double mb, double mc, double M) {
	_ma = ma;
	_mb = mb;
	_mc = mc;
	_M = M;
}
double AmpKinematics::q0(double ma, double mb) const {
	double mapb = ma + mb;
	double mamb = ma - mb;

	if( (_mR*_mR - mapb*mapb) < 0 ) {
		std::cout<<"AmpKinematics: Trying to calculate break-up momentum below threshold!"<<std::endl;
		return 0; //below threshold
	}
	return sqrt( (_mR*_mR - mapb*mapb) * (_mR*_mR - mamb*mamb)) / (2. * _mR );
}
double AmpKinematics::q(double x, double ma, double mb) const {
	double mapb = ma + mb;
	double mamb = ma - mb;

	if( (x*x - mapb*mapb) < 0 ) {
		std::cout<<"AmpKinematics: Trying to calculate break-up momentum below threshold!"<<std::endl;
		return 0; //below threshold
	}
	double result=sqrt( (x*x - mapb*mapb) * (x*x - mamb*mamb) ) / (2. * x );
//	std::cout<<"ss "<<ma<<" "<<mb<<" "<<x<<" "<<sqrt(x*x/4-ma*ma)<<" " <<result<<std::endl;
	return result;
}

// compute square of Blatt-Weisskopf barrier factor
double AmpKinematics::BLres2(double x) const {

	double t0= q0()*q0() * _mesonRadius*_mesonRadius;
	double t= q(x)*q(x) * _mesonRadius*_mesonRadius;
	return FormFactor(t0,t);
}
double AmpKinematics::BLmother2(double x) const {

	//calculate q* here
	double t0= q0()*q0() * _motherRadius*_motherRadius;
	double t= q(x)*q(x) * _motherRadius*_motherRadius;
	return FormFactor(t0,t);
}
double AmpKinematics::FormFactor(double z0, double z) const{
	double nom, denom;
	if (_spin == 0) return 1;
	else if (_spin == 1){
		if(_type==barrierType::BWPrime){
			nom = 1 + z0;
			denom = 1 + z;
		} else if(_type==barrierType::BW){
			nom = 2*z;
			denom = 1 + z;
		} else {
			std::cout<<"Wrong BLW factor definition: "<<_type<<std::endl;
			return 1;
		}
	}
	else if (_spin == 2) {
		if(_type==barrierType::BWPrime){
			nom = (z0-3)*(z0-3)+9*z0;
			denom = (z-3)*(z-3)+9*z;
		} else if(_type==barrierType::BW){
			nom = 13*z*z;
			denom = (z-3)*(z-3)+9*z;
		} else {
			std::cout<<"Wrong BLW factor definition: "<<_type<<std::endl;
			return 1;
		}
	}
	else{
		std::cout<<"Wrong spin value! BLW factors only implemented for spin 0,1 and 2! "<<std::endl;
	}
	return nom/denom;
}
