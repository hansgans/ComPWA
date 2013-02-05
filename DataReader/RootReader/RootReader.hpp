//! Reader for data in Root-files.
/*! \class RootReader
 * @file RootReader.hpp
 * This class reads event-based data from root-files. It implements the
 * interface of Data.hpp.
*/

#ifndef _RootReader_HPP
#define _RootReader_HPP

#include <vector>
#include <map>
#include <memory>
#include <string>
#include <utility>

//PWA-Headers
#include "Core/Data.hpp"
#include "Core/PWAEvent.hpp"

//Root-Headers
#include "TMath.h"
#include "TLorentzVector.h"
#include "TParticle.h"
#include "TROOT.h"
#include "TFile.h"
#include "TClonesArray.h"
#include "TTree.h"
#include "TRandom3.h"

class RootReader : public Data {

public:
  /// Default Constructor (0x0)
  RootReader(const std::string inConfigFile, const bool binned);

  virtual const int getEvent(const int, PWAEvent&);
  virtual const int getBin(const int, double&, double&);
  virtual const int getEvent(const int, TLorentzVector& , TLorentzVector& , double&);

  virtual const unsigned int getNEvents() const {return fmaxEvents;};
  virtual const unsigned int getNBins() const {return fmaxBins;};

  /** Destructor */
  virtual ~RootReader();

protected:
  TFile* fFile;
  TTree* fTree;
  TClonesArray* fParticles;
  unsigned int fmaxEvents;
  unsigned int fEvent;
  bool fBinned;
  unsigned int fmaxBins;
  std::map<int, std::pair<double,double> > fBins;
  // vector<string> paramNames;

  virtual void bin();

};

#endif /* _RootReader_HPP */