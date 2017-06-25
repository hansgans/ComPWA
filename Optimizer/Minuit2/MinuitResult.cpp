/*
 * FitResult.cpp
 *
 *  Created on: Jan 15, 2014
 *      Author: weidenka
 */

#include <numeric>
#include <cmath>
#include <cstdlib>

#include <boost/archive/xml_oarchive.hpp>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include "Core/ProgressBar.hpp"
#include "Core/Logging.hpp"
#include "Optimizer/Minuit2/MinuitResult.hpp"


namespace ComPWA {
/************** HELPER FUNCTION FOR GSL VECTOR AND MATRIX *******************/
/** Print gsl_matrix **/
inline void gsl_matrix_print(const gsl_matrix *m)
{
	for (size_t i = 0; i < m->size1; i++) {
		for (size_t j = 0; j < m->size2; j++) {
			printf("%g ", gsl_matrix_get(m, i, j));
		}
		printf("\n");
	}
};

/** Print gsl_vector **/
inline void gsl_vector_print(const gsl_vector *m)
{
	for (size_t i = 0; i < m->size; i++) {
		std::printf("%g ", gsl_vector_get(m, i));
	}
	std::printf("\n");
};

/** Convert ParameterList to gsl vector **/
inline gsl_vector* gsl_parameterList2Vec(const ParameterList& list){
	gsl_vector* tmp = gsl_vector_alloc(list.GetNDouble());
	unsigned int t=0;
	for(unsigned int o=0;o<list.GetNDouble();o++){
		std::shared_ptr<DoubleParameter> outPar = list.GetDoubleParameter(o);
		if(outPar->IsFixed()) continue;
		gsl_vector_set(tmp,t,outPar->GetValue());
		t++;
	}
	//resize vector
	gsl_vector* vec = gsl_vector_alloc(t);
	for(unsigned int i=0; i<vec->size; i++)
		gsl_vector_set(vec,i,gsl_vector_get(tmp,i));
	return vec;
};

/** Convert std::vector matrix to gsl matrix **/
inline gsl_matrix* gsl_vecVec2Matrix(const std::vector<std::vector<double>>& m){
	gsl_matrix* tmp = gsl_matrix_alloc(m.size(),m.at(0).size());
	for(size_t i=0; i<tmp->size1;i++){
		for(unsigned int j=0; j<tmp->size2;j++){
			gsl_matrix_set(tmp,i,j,m.at(i).at(j));
		}
	}
	return tmp;
};


/** Multivariate Gaussian using cholesky decomposition
 * A test application can be found at test/MultiVariateGaussianTestApp.cpp
 *
 * @param rnd Random number generator
 * @param vecSize Size of data vector
 * @param in Mean value(s)
 * @param cov Covariance matrix
 * @param res Resulting Vector
 */
void multivariateGaussian(const gsl_rng *rnd, const int vecSize,
		const gsl_vector *in, const gsl_matrix *cov, gsl_vector *res){
	//Generate and fill temporary covariance matrix
	gsl_matrix *tmpM= gsl_matrix_alloc(vecSize,vecSize);
	gsl_matrix_memcpy(tmpM,cov);

	//Cholesky decomposition
	int status = gsl_linalg_cholesky_decomp(tmpM);
	if(status == GSL_EDOM )
		BOOST_LOG_TRIVIAL(error)<<"Decomposition has failed!";

	//Compute vector of random gaussian variables
	for(unsigned int i=0; i<vecSize; i++)
		gsl_vector_set( res, i, gsl_ran_ugaussian(rnd) );

	//Debug
	//	gsl_matrix_print(cov);
	//	gsl_vector_print(in);
	//	gsl_vector_print(res);
	//	gsl_matrix_print(tmpM);

	/*From the GNU GSL Documentation:
	 * The function dtrmv compute the matrix-vector product x = op(A) x for the
	 * triangular matrix A, where op(A) = A, A^T, A^H for TransA = CblasNoTrans,
	 * CblasTrans, CblasConjTrans. When Uplo is CblasUpper then the upper
	 * triangle of A is used, and when Uplo is CblasLower then the lower
	 * triangle of A is used. If Diag is CblasNonUnit then the diagonal of
	 * the matrix is used, but if Diag is CblasUnit then the diagonal elements
	 * of the matrix A are taken as unity and are not referenced.
	 */
	gsl_blas_dtrmv(CblasLower, CblasNoTrans, CblasNonUnit, tmpM, res);

	gsl_vector_add(res,in);
	//free temporary object
	gsl_matrix_free(tmpM);
};
/************** HELPER FUNCTION FOR GSL VECTOR AND MATRIX *******************/

MinuitResult::MinuitResult() : initialLH(0), finalLH(0), trueLH(0),
calcInterference(0)
{

}

MinuitResult::MinuitResult(std::shared_ptr<ControlParameter> esti,
		FunctionMinimum result) :
		initialLH(0), finalLH(0), trueLH(0), calcInterference(0)
{
	est = std::static_pointer_cast<Estimator>(esti);
	_ampVec = est->getAmplitudes();
	penalty = est->calcPenalty();
	penaltyScale = est->getPenaltyScale();
	nEvents = est->getNEvents();
	init(result);
}

void MinuitResult::setResult(std::shared_ptr<ControlParameter> esti,
		FunctionMinimum result)
{
	est = std::static_pointer_cast<Estimator>(esti);
	_ampVec = est->getAmplitudes();
	penalty = est->calcPenalty();
	penaltyScale = est->getPenaltyScale();
	nEvents = est->getNEvents();
	init(result);
}

void MinuitResult::init(FunctionMinimum min)
{
	MnUserParameterState minState = min.UserState();

	if(minState.HasCovariance()){
		MnUserCovariance minuitCovMatrix = minState.Covariance();
		/* Size of Minuit covariance vector is given by dim*(dim+1)/2.
		 * dim is the dimension of the covariance matrix.
		 * The dimension can therefore be calculated as
		 * dim = -0.5+-0.5 sqrt(8*size+1)
		 */
		nFreeParameter = minuitCovMatrix.Nrow();
		globalCC = minState.GlobalCC().GlobalCC();
		cov = std::vector<std::vector<double>>(
				nFreeParameter,std::vector<double>(nFreeParameter));
		corr = std::vector<std::vector<double>>(
				nFreeParameter,std::vector<double>(nFreeParameter));
		for (unsigned i = 0; i < nFreeParameter; ++i)
			for (unsigned j = i; j < nFreeParameter; ++j){
				cov.at(i).at(j) = minuitCovMatrix(j,i);
				cov.at(j).at(i) = minuitCovMatrix(j,i);//fill lower half
			}
		for (unsigned i = 0; i < nFreeParameter; ++i)
			for (unsigned j = i; j < nFreeParameter; ++j){
				corr.at(i).at(j) =
						cov.at(i).at(j) / sqrt( cov.at(i).at(i) *
								cov.at(j).at(j) );
				corr.at(j).at(i) = corr.at(i).at(j);//fill lower half
			}

	} else
		BOOST_LOG_TRIVIAL(error)
		<< "MinuitResult: no valid correlation matrix available!";
	initialLH = -1;
	finalLH = minState.Fval();
	edm= minState.Edm();
	isValid = min.IsValid();
	covPosDef = min.HasPosDefCovar();
	hasValidParameters = min.HasValidParameters();
	hasValidCov = min.HasValidCovariance();
	hasAccCov = min.HasAccurateCovar();
	hasReachedCallLimit = min.HasReachedCallLimit();
	edmAboveMax = min.IsAboveMaxEdm();
	hesseFailed = min.HesseFailed();
	errorDef = min.Up();
	nFcn = min.NFcn();

	return;
}

//! Set list of true parameters
void MinuitResult::setTrueParameters(ParameterList truePars)
{
	trueParameters = truePars;
	if( trueParameters.GetNDouble() && est ){
		//Setting true parameter and calculate LH value
		Amplitude::UpdateAmpParameterList(_ampVec,trueParameters);
		SetTrueLH( est->controlParameter(trueParameters) );
		Amplitude::UpdateAmpParameterList(_ampVec,finalParameters);
	}

}

//! Set list of initial parameters
void MinuitResult::setInitialParameters(ParameterList iniPars)
{
	initialParameters = iniPars;
	if( initialParameters.GetNDouble() && est ){
		//Setting initial parameter and calculate LH value
		Amplitude::UpdateAmpParameterList(_ampVec,initialParameters);
		SetInitialLH( est->controlParameter(initialParameters) );
		Amplitude::UpdateAmpParameterList(_ampVec,finalParameters);
	}

}

void MinuitResult::genSimpleOutput(std::ostream& out)
{
	for(unsigned int o=0;o<finalParameters.GetNDouble();o++){
		std::shared_ptr<DoubleParameter> outPar =
				finalParameters.GetDoubleParameter(o);
		out<<outPar->GetValue()<<" "<<outPar->GetError()<<" ";
	}
	out<<"\n";

	return;
}


void MinuitResult::calcFractionError(ParameterList& parList,
		std::shared_ptr<Amplitude> amp, int nSets)
{
	if( nSets <= 0 ) return;
	if( !parList.GetNDouble() ) return;
	BOOST_LOG_TRIVIAL(info) << "Calculating errors of fit fractions using "
			<<nSets<<" sets of parameters...";

	//Setting up random number generator
	const gsl_rng_type * T;
	gsl_rng_env_setup();
	T = gsl_rng_default;
	gsl_rng* rnd = gsl_rng_alloc (T);

	//convert to GSL objects
	gsl_vector* gslFinalPar = gsl_parameterList2Vec(finalParameters);
	gsl_matrix* gslCov = gsl_vecVec2Matrix(cov);
	gsl_matrix_print(gslCov); //DEBUG

	std::vector<ParameterList> fracVect;
	progressBar bar(nSets);
	stringstream outFraction;
//	for(unsigned int i=0; i<nSets; i++){
	int i=0;
	while( i<nSets ){
		bool error=0;
		bar.nextEvent();
		gsl_vector* gslNewPar = gsl_vector_alloc(nFreeParameter);
		//generate set of smeared parameters
		multivariateGaussian( rnd, nFreeParameter,
				gslFinalPar, gslCov, gslNewPar );
		gsl_vector_print(gslNewPar);

		//deep copy of finalParameters
		ParameterList newPar;
		newPar.DeepCopy(finalParameters);

		std::size_t t=0;
		for(std::size_t o=0;o<newPar.GetNDouble();o++){
			std::shared_ptr<DoubleParameter> outPar =
					newPar.GetDoubleParameter(o);
			if(outPar->IsFixed()) continue;
			//set floating values to smeared values
			try{ //catch out-of-bound
				outPar->SetValue(gslNewPar->data[t]);
			} catch ( ParameterOutOfBound& ex ){
				error=1;
			}
			t++;
		}
		if( error ) continue; //skip this set if one parameter is out of bound

		//free vector
		gsl_vector_free(gslNewPar);
		//update amplitude with smeared parameters
		try{
			Amplitude::UpdateAmpParameterList(_ampVec, newPar);
		} catch ( ParameterOutOfBound& ex ){
			continue;
		}
		ParameterList tmp;
		amp->GetFitFractions(tmp);
		fracVect.push_back(tmp);
		i++;

		/******* DEBUGGING *******/
		//			if(i==0){
		//				for(int t=0; t<newPar.GetNDouble(); t++){
		//					if( newPar.GetDoubleParameter(t)->IsFixed()) continue;
		//					outFraction << newPar.GetDoubleParameter(t)->GetName()<<":";
		//				}
		//				for(int t=0; t<tmp.GetNDouble(); t++)
		//					outFraction << tmp.GetDoubleParameter(t)->GetName()<<":";
		//				outFraction << "norm" << std::endl;
		//			}
		//			for(int t=0; t<newPar.GetNDouble(); t++){
		//				if( newPar.GetDoubleParameter(t)->IsFixed()) continue;
		//				outFraction << newPar.GetDoubleParameter(t)->GetValue()<<" ";
		//			}
		//			for(int t=0; t<tmp.GetNDouble(); t++)
		//				outFraction << tmp.GetDoubleParameter(t)->GetValue()<<" ";
		//			double norm = _amp->GetIntegral();
		//			outFraction << norm;
		//			outFraction << std::endl;
		/******* DEBUGGING *******/
	}
	BOOST_LOG_TRIVIAL(info)<<" ------- "<<outFraction.str();

	//free objects
	gsl_vector_free(gslFinalPar);
	gsl_matrix_free(gslCov);
	gsl_rng_free(rnd);

	int nRes=parList.GetNDouble();
	//Calculate standard deviation
	for(unsigned int o=0;o<nRes;o++){
		double mean=0, sqSum=0., stdev=0;
		for(unsigned int i=0; i<fracVect.size();i++){
			double tmp = fracVect.at(i).GetDoubleParameter(o)->GetValue();
			mean += tmp;
			sqSum += tmp*tmp;
		}
		unsigned int s = fracVect.size();
		sqSum /= s;
		mean /= s;
		//this is cross-checked with the RMS of the distribution
		stdev = std::sqrt(sqSum - mean*mean);
		parList.GetDoubleParameter(o)->SetError(stdev);
	}

	//Set correct fit result
	Amplitude::UpdateAmpParameterList(_ampVec, finalParameters);
	return;
}

void MinuitResult::genOutput(std::ostream& out, std::string opt)
{
	bool printParam=1, printCorrMatrix=1, printCovMatrix=1;
	if(opt=="P") {//print only parameters
		printCorrMatrix=0; printCovMatrix=0;
	}
	out<<std::endl;
	out<<"--------------MINUIT2 FIT RESULT----------------"<<std::endl;
	if(!isValid) out<<"		*** MINIMUM NOT VALID! ***"<<std::endl;
	out<<std::setprecision(10);
	out<<"Initial Likelihood: "<<initialLH<<std::endl;
	out<<"Final Likelihood: "<<finalLH<<std::endl;
	if(trueLH)
		out<<"True Likelihood: "<<trueLH<<std::endl;

	out<<"Estimated distance to minimumn: "<<edm<<std::endl;
	if(edmAboveMax) out<<"		*** EDM IS ABOVE MAXIMUM! ***"<<std::endl;
	out<<"Error definition: "<<errorDef<<std::endl;
	out<<"Number of calls: "<<nFcn<<std::endl;
	if(hasReachedCallLimit)
		out<<"		*** LIMIT OF MAX CALLS REACHED! ***"<<std::endl;
	out<<"CPU Time : "<<time/60<<"min"<<std::endl;
	out<<std::setprecision(5)<<std::endl;


	if(!hasValidParameters)
		out<<"		*** NO VALID SET OF PARAMETERS! ***"<<std::endl;
	if(printParam){
		out<<"PARAMETERS:"<<std::endl;
		TableFormater* tableResult = new TableFormater(&out);
		printFitParameters(tableResult);
	}

	if(!hasValidCov)
		out<<"		*** COVARIANCE MATRIX NOT VALID! ***"<<std::endl;
	if(!hasAccCov)
		out<<"		*** COVARIANCE MATRIX NOT ACCURATE! ***"<<std::endl;
	if(!covPosDef)
		out<<"		*** COVARIANCE MATRIX NOT POSITIVE DEFINITE! ***"<<std::endl;
	if(hesseFailed)
		out<<"		*** HESSE FAILED! ***"<<std::endl;
	if(hasValidCov){
		unsigned int n=0;
		if(printCovMatrix){
			out<<"COVARIANCE MATRIX:"<<std::endl;
			TableFormater* tableCov = new TableFormater(&out);
			printCovarianceMatrix(tableCov);
		}
		if(printCorrMatrix){
			out<<"CORRELATION MATRIX:"<<std::endl;
			TableFormater* tableCorr = new TableFormater(&out);
			printCorrelationMatrix(tableCorr);
		}
	}
	out<<"FIT FRACTIONS:"<<std::endl;
	//Calculate and print fractions
	TableFormater tab(&out);
	//Calculate fit fractions for all amplitudes
	//	printFitFractions(&tab);
	//Calculate fit fractions for first amplitude only
	printFitFractions(&tab,_ampVec.at(0),nSetsFractionError);

	out<<std::setprecision(10);
	out<<"Final penalty term: "<<penalty<<std::endl;
	out<<"FinalLH w/o penalty: "<<finalLH-penalty<<std::endl;
	out<<"FinalLH w/ penalty: "<<finalLH<<std::endl;
	/* The Akaike (AIC) and Bayesian (BIC) information criteria are described in
	 * Schwarz, Anals of Statistics 6 No.2: 461-464 (1978)
	 * and
	 * IEEE Transacrions on Automatic Control 19, No.6:716-723 (1974) */
	ParameterList frac;
	try{
		_ampVec.at(0)->GetFitFractions(frac);
	} catch (std::exception& ex){
		BOOST_LOG_TRIVIAL(error) << "MinuitResult::genOutput() | Can not "
				"calculate fit fractions for amplitude 0.";
	}
	AIC = calcAIC(frac)-penalty;
	BIC = calcBIC(frac)-penalty;
	out<<"AIC: "<<AIC<<std::endl;
	out<<"BIC: "<<BIC<<std::endl;

	nResSignif=0;
	for(int i=0; i<fractionList.GetNDouble(); i++){
		double val = std::fabs(fractionList.GetDoubleParameter(i)->GetValue());
		if(val > 0.001) nResSignif++;
	}
	out<<"Number of Resonances > 10^-3: "<<nResSignif<<std::endl;

	if(calcInterference){
		auto ampItr = _ampVec.begin();
		for( ; ampItr != _ampVec.end(); ++ampItr)
			createInterferenceTable(out,(*ampItr));
	}

	out<<std::setprecision(5);//reset cout precision
	return;
}

void MinuitResult::createInterferenceTable(std::ostream& out,
		std::shared_ptr<Amplitude> amp)
{
	out<<"INTERFERENCE terms for "<<amp->GetName()<<": "<<std::endl;
	TableFormater* tableInterf = new TableFormater(&out);
	tableInterf->addColumn("Name 1",15);
	tableInterf->addColumn("Name 2",15);
	tableInterf->addColumn("Value",15);
	tableInterf->header();
	double sumInfTerms = 0;
	auto it = amp->GetResonanceItrFirst();
	for( ; it != amp->GetResonanceItrLast(); ++it){
		auto it2 = it;
		for( ; it2 != amp->GetResonanceItrLast(); ++it2){
			*tableInterf << (*it)->GetName();
			*tableInterf << (*it2)->GetName();
			double inf = amp->GetIntegralInterference(it,it2);
			*tableInterf << inf;
			sumInfTerms+=inf;
		}
	}
	tableInterf->delim();
	*tableInterf<<" "<<"Sum: "<<sumInfTerms;
	tableInterf->footer();
	out<<std::endl;
}

double MinuitResult::calcAIC(ParameterList& frac)
{
	double r=0;
	for(int i=0; i<frac.GetNDouble(); i++){
		double val = frac.GetDoubleParameter(i)->GetValue();
		if(val > 0.001) r++;
	}
	return (finalLH+2*r);
}

double MinuitResult::calcBIC(ParameterList& frac)
{
	double r=0;
	for(int i=0; i<frac.GetNDouble(); i++){
		double val = frac.GetDoubleParameter(i)->GetValue();
		if(val > 0.001) r++;
	}
	return (finalLH+r*std::log(nEvents));
}

void MinuitResult::printCorrelationMatrix(TableFormater* tableCorr)
{
	if(!hasValidCov) return;
	tableCorr->addColumn(" ",15);//add empty first column
	tableCorr->addColumn("GlobalCC",10);//global correlation coefficient

	//add columns in correlation matrix
	for(unsigned int o=0;o<finalParameters.GetNDouble();o++){
		std::shared_ptr<DoubleParameter> ppp =
				finalParameters.GetDoubleParameter(o);
		if(ppp->IsFixed()) continue;
		tableCorr->addColumn(ppp->GetName(),15);
	}

	unsigned int n=0;
	tableCorr->header();
	for(unsigned int o=0;o<finalParameters.GetNDouble();o++){
		std::shared_ptr<DoubleParameter> ppp =
				finalParameters.GetDoubleParameter(o);
		if(ppp->IsFixed()) continue;
		*tableCorr << ppp->GetName();
		*tableCorr << globalCC.at(n);
		for(unsigned int t=0;t<corr.size();t++) {
			if(n>=corr.at(0).size()) { *tableCorr<< " "; continue; }
			if(t>=n)*tableCorr << corr.at(n).at(t);
			else *tableCorr << "";
		}
		n++;
	}
	tableCorr->footer();
	return;
}

void MinuitResult::printCovarianceMatrix(TableFormater* tableCov)
{
	if(!hasValidCov) return;
	tableCov->addColumn(" ",17);//add empty first column
	//add columns first
	for(unsigned int o=0;o<finalParameters.GetNDouble();o++){
		if(!finalParameters.GetDoubleParameter(o)->IsFixed())
			tableCov->addColumn(
					finalParameters.GetDoubleParameter(o)->GetName(),17);
	}

	unsigned int n=0;
	tableCov->header();
	for(unsigned int o=0;o<finalParameters.GetNDouble();o++){
		std::shared_ptr<DoubleParameter> ppp =
				finalParameters.GetDoubleParameter(o);
		if(ppp->IsFixed()) continue;
		*tableCov << ppp->GetName();
		for(unsigned int t=0;t<cov.size();t++) {
			if(n>=cov.at(0).size()) { *tableCov<< " "; continue; }
			if(t>=n) *tableCov << cov.at(n).at(t);
			else *tableCov << "";
		}
		n++;
	}
	tableCov->footer();
	return;
}

void MinuitResult::writeXML(std::string filename)
{
#ifdef USESERIALIZATION
	std::ofstream ofs(filename);
	boost::archive::xml_oarchive oa(ofs);
	oa << boost::serialization::make_nvp("FitParameters", finalParameters);
	oa << boost::serialization::make_nvp("FitFractions", fractionList);
	ofs.close();
#endif

	return;
}

void MinuitResult::writeTeX(std::string filename)
{
	std::ofstream out(filename);
	TableFormater* tableResult = new TexTableFormater(&out);
	printFitParameters(tableResult);
	if(hasValidCov){
		unsigned int n=0;
		TableFormater* tableCov = new TexTableFormater(&out);
		printCovarianceMatrix(tableCov);
		TableFormater* tableCorr = new TexTableFormater(&out);
		printCorrelationMatrix(tableCorr);
	}
	TableFormater* fracTable = new TexTableFormater(&out);
	//calculate and print fractions if amplitude is set
	printFitFractions(fracTable);
	out.close();
	return;
}

bool MinuitResult::hasFailed()
{
	bool failed=0;
	if(!isValid) failed=1;
	//	if(!covPosDef) failed=1;
	//	if(!hasValidParameters) failed=1;
	//	if(!hasValidCov) failed=1;
	//	if(!hasAccCov) failed=1;
	//	if(hasReachedCallLimit) failed=1;
	//	if(hesseFailed) failed=1;

	return failed;
}
}
