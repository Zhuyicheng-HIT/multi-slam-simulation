#include "hybridfusion_map_fusion/hybridfusion_core.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace
{

std::map<std::string, std::string> parse_arguments(int argc, char ** argv)
{
  std::map<std::string, std::string> arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string key(argv[index]);
    if (key == "--help" || key == "-h") {
      arguments["help"] = "true";
      continue;
    }
    if (key.rfind("--", 0) != 0 || index + 1 >= argc) {
      throw std::runtime_error("arguments must be --name value pairs");
    }
    arguments[key.substr(2)] = argv[++index];
  }
  return arguments;
}

void usage()
{
  std::cout <<
    "Usage: hybridfusion_offline --dataset DATASET.yaml --config CONFIG.yaml "
    "--method initial|gicp|hybrid --output OUTPUT_DIR\n";
}

std::string expand_home(const std::string & path)
{
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      throw std::runtime_error("HOME is not set; use an absolute output path");
    }
    return std::string(home) + path.substr(1);
  }
  return path;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto arguments = parse_arguments(argc, argv);
    if (arguments.count("help") > 0) {
      usage();
      return 0;
    }
    for (const auto * required : {"dataset", "config", "method", "output"}) {
      if (arguments.count(required) == 0) {
        usage();
        throw std::runtime_error(std::string("missing --") + required);
      }
    }
    const auto config = hybridfusion_map_fusion::load_config(arguments.at("config"));
    const auto dataset = hybridfusion_map_fusion::load_dataset(arguments.at("dataset"));
    const auto visual = hybridfusion_map_fusion::load_cloud(dataset.visual_map_path);
    const auto lidar = hybridfusion_map_fusion::load_cloud(dataset.lidar_map_path);
    const auto result = hybridfusion_map_fusion::run_registration(
      arguments.at("method"), visual, lidar, dataset, config);
    const auto output_dir = expand_home(arguments.at("output"));
    hybridfusion_map_fusion::write_result_artifacts(
      output_dir, result, visual, lidar, dataset, config);
    std::cout << "HybridFusion method=" << result.method
              << " converged=" << std::boolalpha << result.converged
              << " translation_error_m=" << result.metrics.translation_error_m
              << " rotation_error_deg=" << result.metrics.rotation_error_deg
              << " overlap_mean_nn_m=" << result.metrics.overlap_mean_nn_m
              << " boundary_mean_nn_m=" << result.metrics.boundary_mean_nn_m
              << " inlier_ratio=" << result.metrics.inlier_ratio
              << " supplement_ratio=" << result.metrics.supplement_voxel_growth_ratio
              << " blocks=" << result.successful_blocks << "/"
              << (result.successful_blocks + result.failed_blocks)
              << " output=" << std::filesystem::absolute(output_dir)
              << std::endl;
    return result.converged ? 0 : 3;
  } catch (const std::exception & error) {
    std::cerr << "hybridfusion_offline: " << error.what() << std::endl;
    return 2;
  }
}
