#include "boost/graph/graph_traits.hpp"

#include "TopologyAmplitudeFactory.hpp"
#include "DecayTree.hpp"

namespace HelicityFormalism {

TopologyAmplitudeFactory::TopologyAmplitudeFactory() {
  // TODO Auto-generated constructor stub
}

TopologyAmplitudeFactory::~TopologyAmplitudeFactory() {
  // TODO Auto-generated destructor stub
}

SequentialTwoBodyDecayAmplitude TopologyAmplitudeFactory::generateSequentialDecayAmplitude(
    const DecayTree& decay_tree) {
  SequentialTwoBodyDecayAmplitude full_decay_amplitude;

  const HelicityTree helicity_tree = decay_tree.getHelicityDecayTree();
  const std::vector<
      boost::graph_traits<HelicityFormalism::HelicityTree>::vertex_descriptor>& decay_vertex_list =
      decay_tree.getDecayVertexList();

  std::vector<
      boost::graph_traits<HelicityFormalism::HelicityTree>::vertex_descriptor>::const_iterator decay_vertex_iter;

  for (decay_vertex_iter = decay_vertex_list.begin();
      decay_vertex_iter != decay_vertex_list.end(); ++decay_vertex_iter) {

    TwoBodyDecaySpinInformation decay_spin_info;

    std::pair<boost::graph_traits<HelicityTree>::out_edge_iterator,
        boost::graph_traits<HelicityTree>::out_edge_iterator> ep;

    ep = boost::out_edges(*decay_vertex_iter, helicity_tree);

    unsigned final_state_particle_counter = 0;
    while (ep.first != ep.second) {
      ++final_state_particle_counter;
      boost::graph_traits<HelicityTree>::vertex_descriptor decay_product(
          boost::target(*ep.first, helicity_tree));
      if (1 == final_state_particle_counter) {
        decay_spin_info.final_state_.first =
            helicity_tree[decay_product].spin_information_;
      }
      else if (2 == final_state_particle_counter) {
        decay_spin_info.final_state_.second =
            helicity_tree[decay_product].spin_information_;
      }
      else {
        std::stringstream ss;
        ss
            << "HelicityAmplitudeTreeFactory: This decay vertex has more than 2 final state particles ("
            << final_state_particle_counter
            << " fs particles)! This is not supported in this model. Please fix the decay file.";
        throw std::runtime_error(ss.str());
      }
      ++ep.first;
    }

    decay_spin_info.initial_state_ =
        helicity_tree[*decay_vertex_iter].spin_information_;

    // create new amplitude if not yet existent
    if (two_body_decay_amplitude_list_.find(decay_spin_info)
        == two_body_decay_amplitude_list_.end()) {
      two_body_decay_amplitude_list_[decay_spin_info] = std::shared_ptr<
          TwoBodyDecayAmplitude>(new TwoBodyDecayAmplitude(decay_spin_info));
    }

    TwoBodyDecayInformation decay_info;
    decay_info.spin_info_ = decay_spin_info;
    decay_info.dynamical_info_.initial_state_ =
        helicity_tree[*decay_vertex_iter].dynamical_information_;
    std::shared_ptr<DynamicalFunctions::AbstractDynamicalFunction> abs_function =
        dynamical_function_factory_.generateDynamicalFunction(decay_info);
    full_decay_amplitude.push_back(
        std::make_pair(two_body_decay_amplitude_list_[decay_spin_info],
            abs_function));
  }

  return full_decay_amplitude;
}

std::vector<TopologyAmplitude> TopologyAmplitudeFactory::generateTopologyAmplitudes(
    const std::vector<DecayTree>& decay_tree_collection) {
  // first group decay trees according to their topology
  std::map<DecayTopology, std::vector<DecayTree> > topology_grouped_decay_trees;
  std::vector<DecayTree>::const_iterator decay_tree_iter;
  for (decay_tree_iter = decay_tree_collection.begin();
      decay_tree_iter != decay_tree_collection.end(); ++decay_tree_iter) {
    topology_grouped_decay_trees[createDecayTopology(*decay_tree_iter)].push_back(
        *decay_tree_iter);
  }

  std::vector<TopologyAmplitude> topology_amps;

  std::map<DecayTopology, std::vector<DecayTree> >::const_iterator grouped_decay_tree_iter;
  for (grouped_decay_tree_iter = topology_grouped_decay_trees.begin();
      grouped_decay_tree_iter != topology_grouped_decay_trees.end();
      ++grouped_decay_tree_iter) {
    TopologyAmplitude topology_amp;

    for (decay_tree_iter = grouped_decay_tree_iter->second.begin();
        decay_tree_iter != grouped_decay_tree_iter->second.end();
        ++decay_tree_iter) {
      topology_amp.sequential_decay_amplitude_list_.push_back(
          generateSequentialDecayAmplitude(*decay_tree_iter));
    }
    topology_amps.push_back(topology_amp);
  }

  return topology_amps;
}

DecayTopology TopologyAmplitudeFactory::createDecayTopology(
    const DecayTree& decay_tree) const {
  DecayTopology decay_topology;

  const HelicityTree helicity_tree = decay_tree.getHelicityDecayTree();
  const std::vector<
      boost::graph_traits<HelicityFormalism::HelicityTree>::vertex_descriptor>& decay_vertex_list =
      decay_tree.getDecayVertexList();

  std::vector<
      boost::graph_traits<HelicityFormalism::HelicityTree>::vertex_descriptor>::const_iterator decay_vertex_iter;

  for (decay_vertex_iter = decay_vertex_list.begin();
      decay_vertex_iter != decay_vertex_list.end(); ++decay_vertex_iter) {

    std::vector<std::vector<ParticleStateInfo> > fs_particle_lists =
        decay_tree.createDecayProductsFinalStateParticleLists(
            *decay_vertex_iter);

    TwoBodyDecayIndices indices;
    std::vector<ParticleStateInfo> mother_fs_particle_list;

    std::sort(fs_particle_lists.begin(), fs_particle_lists.end());
    if (fs_particle_lists.size() == 2) {
      for (unsigned int i = 0; i < fs_particle_lists.size(); ++i) {
        std::vector<std::vector<ParticleStateInfo> >::const_iterator result =
            std::find(decay_topology.final_state_content_lists_.begin(),
                decay_topology.final_state_content_lists_.end(),
                fs_particle_lists[i]);

        unsigned int index = result
            - decay_topology.final_state_content_lists_.begin();
        if (result == decay_topology.final_state_content_lists_.end()) {
          decay_topology.final_state_content_lists_.push_back(
              fs_particle_lists[i]);
        }

        if (i == 0) {
          indices.decay_products_.first = index;
        }
        else {
          indices.decay_products_.second = index;
        }

        mother_fs_particle_list.insert(mother_fs_particle_list.end(),
            fs_particle_lists[i].begin(), fs_particle_lists[i].end());
      }
    }
    else if (fs_particle_lists.size() != 0) {
      std::stringstream ss;
      ss
          << "HelicityAmplitudeTreeFactory: This decay vertex does not have 2 final state particles ("
          << fs_particle_lists.size()
          << " fs particles)! This is not supported in this model. Please fix the decay file.";
      throw std::runtime_error(ss.str());
    }

    // do the same for the mother
    std::vector<std::vector<ParticleStateInfo> >::const_iterator result =
        std::find(decay_topology.final_state_content_lists_.begin(),
            decay_topology.final_state_content_lists_.end(),
            mother_fs_particle_list);

    unsigned int mother_index = result
        - decay_topology.final_state_content_lists_.begin();
    if (result == decay_topology.final_state_content_lists_.end()) {
      decay_topology.final_state_content_lists_.push_back(
          mother_fs_particle_list);
    }
    indices.mother_index_ = mother_index;

    decay_topology.decay_node_infos_.push_back(indices);
  }
  return decay_topology;
}

} /* namespace HelicityFormalism */