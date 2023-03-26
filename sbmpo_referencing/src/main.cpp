#include <sbmpo/sbmpo.hpp>
#include <sbmpo_models/UnicycleSteering.hpp>
#include <sbmpo_referencing/ReferencerBenchmark.hpp>
#include <iostream>

using namespace sbmpo_referencing;
using namespace sbmpo_benchmarking;

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

  ReferencerBenchmark<sbmpo_models::UnicycleSteeringModel> benchmarker(csv_folder);

  benchmarker.set_reference_factor(0.1);

  benchmarker.benchmark(benchmarker);
  
  return 0;
}