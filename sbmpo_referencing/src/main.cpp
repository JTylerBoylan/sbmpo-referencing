#include <sbmpo/sbmpo.hpp>
#include <sbmpo_referencing/Referencer.hpp>
#include <sbmpo_models/UnicycleSteering.hpp>
#include <sbmpo_benchmarking/benchmark.hpp>
#include <iostream>

using namespace sbmpo_referencing;

int main(int argc, char ** argv) {

  // Path to csv workspace
  std::string csv_folder;

  // Check arguments
  if (argc > 1) {
      csv_folder = argv[1];
  } else {
      printf("\nMissing CSV folder path.\n");
      return 0;
  }

  Referencer<sbmpo_models::UnicycleSteeringModel> model;

  model.set_reference_factor(0.1);

  sbmpo_benchmarking::Benchmark benchmarker(csv_folder);

  benchmarker.benchmark(model);
  
  return 0;
}