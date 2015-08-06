/*
 * HelicityDecayTreeTestApp.cpp
 *
 *  Created on: May 5, 2015
 *      Author: steve
 */

#include <fstream>

#include "Physics/HelicityAmplitude/DecayConfiguration.hpp"
#include "Physics/HelicityAmplitude/DecayXMLConfigReader.hpp"
#include "Physics/HelicityAmplitude/DecayTreeFactory.hpp"

int main(int argc, char **argv) {
  std::cout << "  ComPWA Copyright (C) 2013  Stefan Pflueger " << std::endl;
  std::cout
      << "  This program comes with ABSOLUTELY NO WARRANTY; for details see license.txt"
      << std::endl;
  std::cout << std::endl;

  HelicityFormalism::DecayConfiguration decay_configuration;
  HelicityFormalism::DecayXMLConfigReader xml_reader(decay_configuration);
  xml_reader.readConfig("Physics/HelicityAmplitude/JPSI_ypipi.xml");

  HelicityFormalism::DecayTreeFactory decay_tree_factory(decay_configuration);

  std::vector<HelicityFormalism::DecayTree> decay_trees =
      decay_tree_factory.createDecayTrees();

  std::ofstream dot("graph.dot");

  std::vector<HelicityFormalism::DecayTree>::iterator decay_tree;
  for (decay_tree = decay_trees.begin(); decay_tree != decay_trees.end();
      ++decay_tree) {
    if (!decay_tree->hasCycles()) {
      decay_tree->print(dot);
    }
  }

  return 0;
}

