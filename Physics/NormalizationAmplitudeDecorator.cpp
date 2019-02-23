// Copyright (c) 2013, 2017 The ComPWA Team.
// This file is part of the ComPWA framework, check
// https://github.com/ComPWA/ComPWA/license.txt for details.

#include <cmath>

#include "Core/Event.hpp"
#include "NormalizationAmplitudeDecorator.hpp"
#include "Physics/CoherentIntensity.hpp"
#include "Tools/Integration.hpp"

namespace ComPWA {
namespace Physics {

NormalizationAmplitudeDecorator::NormalizationAmplitudeDecorator(
    const std::string &name, std::shared_ptr<NamedAmplitude> amplitude,
    std::shared_ptr<ComPWA::Tools::IntegrationStrategy> integrator)
    : NamedAmplitude(name), UnnormalizedAmplitude(amplitude),
      NormedAmplitude(std::make_shared<CoherentIntensity>(
          amplitude->getName(),
          std::vector<std::shared_ptr<NamedAmplitude>>{amplitude})),
      Integrator(integrator) {
  Normalization = std::sqrt(1.0 / Integrator->integrate(NormedAmplitude));
  ParameterList TempParList;
  UnnormalizedAmplitude->addUniqueParametersTo(TempParList);
  PreviousParameterList.DeepCopy(TempParList);
}

std::complex<double> NormalizationAmplitudeDecorator::evaluate(
    const ComPWA::DataPoint &point) const {

  return Normalization * UnnormalizedAmplitude->evaluate(point);
}

void NormalizationAmplitudeDecorator::updateParametersFrom(
    const ParameterList &list) {
  // it is important to call the update on the unnormalized intensity first
  UnnormalizedAmplitude->updateParametersFrom(list);

  ParameterList TempParList;
  UnnormalizedAmplitude->addUniqueParametersTo(TempParList);
  for (unsigned int i = 0; i < TempParList.doubleParameters().size(); ++i) {
    if (TempParList.doubleParameter(i) !=
        PreviousParameterList.doubleParameter(i)) {
      LOG(DEBUG) << "NormalizationAmplitudeDecorator::updateParametersFrom(): "
                    "recalculating normalization!";
      Normalization = std::sqrt(1.0 / Integrator->integrate(NormedAmplitude));
      PreviousParameterList.DeepCopy(TempParList);
      break;
    }
  }
}

void NormalizationAmplitudeDecorator::addUniqueParametersTo(
    ParameterList &list) {
  UnnormalizedAmplitude->addUniqueParametersTo(list);
}

std::shared_ptr<FunctionTree>
NormalizationAmplitudeDecorator::createFunctionTree(
    const ParameterList &DataSample, const std::string &suffix) const {

  auto NodeName = "NormalizedAmplitude(" + getName() + ")" + suffix;
  size_t n = DataSample.mDoubleValue(0)->values().size();

  auto tr = std::make_shared<FunctionTree>(
      NodeName, MComplex("", n), std::make_shared<MultAll>(ParType::MCOMPLEX));

  tr->createNode("sqrt(Normalization)", ComPWA::ValueFactory(ParType::DOUBLE),
                 std::make_shared<ComPWA::SquareRoot>(ParType::DOUBLE),
                 NodeName);
  auto normtree = Integrator->createFunctionTree(NormedAmplitude, "_ampnorm");
  tr->insertTree(normtree, "sqrt(Normalization)");
  auto intenstree =
      UnnormalizedAmplitude->createFunctionTree(DataSample, suffix);
  tr->insertTree(intenstree, NodeName);

  return tr;
}

std::shared_ptr<const Amplitude>
NormalizationAmplitudeDecorator::getUnnormalizedAmplitude() const {
  return UnnormalizedAmplitude;
}

} // namespace Physics
} // namespace ComPWA
