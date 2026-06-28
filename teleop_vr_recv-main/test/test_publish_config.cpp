#include "teleop_vr_recv/config.h"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path writeConfig(const std::string& contents,
                                  const std::string& name) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream file(path);
  file << contents;
  return path;
}

}  // namespace

int main() {
  teleop_vr::Config defaults;
  assert(defaults.publish.mode == "both");
  assert(defaults.publish.publish_debug_raw);
  assert(defaults.validate());

  const auto cartesian_config = writeConfig(
      R"toml(
[publish]
mode = "cartesian"
publish_debug_raw = false
)toml",
      "teleop_vr_recv_publish_cartesian.toml");

  teleop_vr::Config cartesian;
  assert(cartesian.loadFromFile(cartesian_config.string()));
  assert(cartesian.publish.mode == "cartesian");
  assert(!cartesian.publish.publish_debug_raw);
  assert(cartesian.validate());

  const auto invalid_config = writeConfig(
      R"toml(
[publish]
mode = "invalid"
)toml",
      "teleop_vr_recv_publish_invalid.toml");

  teleop_vr::Config invalid;
  assert(invalid.loadFromFile(invalid_config.string()));
  assert(!invalid.validate());

  std::filesystem::remove(cartesian_config);
  std::filesystem::remove(invalid_config);
  return 0;
}
