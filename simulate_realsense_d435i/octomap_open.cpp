#include <rclcpp/rclcpp.hpp>
#include <octomap_server2/octomap_server.hpp>

#define USAGE "\nUSAGE: octomap_open <mapfile.[bt|ot]>\n" \
              "  mapfile.bt: filename of map to be saved (.bt: binary tree, .ot: general octree)\n"

using namespace std;
using namespace octomap;
using namespace octomap_server;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string mapFilename("");

  if (argc == 2)
    mapFilename = std::string(argv[1]);
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", USAGE);
    exit(1);
  }

  try {
    rclcpp::NodeOptions nodeOptions;
    octomap_server::OctomapServer ms(nodeOptions);
    ms.openFile(mapFilename);
  } catch(std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "octomap_open exception: %s", e.what());
    exit(2);
  }

  exit(0);
}
