/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>
#include <unistd.h>

#include <octomap_msgs/srv/get_octomap.hpp>
using OctomapSrv = octomap_msgs::srv::GetOctomap;

#define USAGE "\nUSAGE: octomap_saver [-f] <mapfile.[bt|ot]>\n" \
                "  -f: Query for the full occupancy octree, instead of just the compact binary one\n" \
		"  mapfile.bt: filename of map to be saved (.bt: binary tree, .ot: general octree)\n"

using namespace std;
using namespace octomap;

class MapSaver{
public:
  MapSaver(const std::string& mapname, bool full){
    auto n = rclcpp::Node::make_shared("octomap_saver");
    
    std::string servname = "octomap_binary";
    if (full)
      servname = "octomap_full";
    // auto nsrv = n->get_node_services_interface();
    RCLCPP_INFO(n->get_logger(), "Requesting the map from %s...", servname.c_str());
    
    auto client = n->create_client<OctomapSrv>(servname);
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(n->get_logger(), "waiting for service to appear...");
    }
    
    auto req = std::make_shared<OctomapSrv::Request>();
    auto result_future = client->async_send_request(req);    
    while(rclcpp::ok() && rclcpp::spin_until_future_complete(n, result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(n->get_logger(), "Request to %s failed; trying again...", servname.c_str());
      usleep(1000000);
    }

    if (rclcpp::ok()){ // skip when CTRL-C
      auto resp = result_future.get();
      AbstractOcTree* tree = octomap_msgs::msgToMap(resp->map);
      AbstractOccupancyOcTree* octree = NULL;
      if (tree){
        octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
      } else {
        RCLCPP_ERROR(n->get_logger(), "Error creating octree from received message");
        if (resp->map.id == "ColorOcTree")
          RCLCPP_WARN(n->get_logger(), "You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
      }

      if (octree){
        RCLCPP_INFO(n->get_logger(), "Map received (%zu nodes, %f m res), saving to %s", octree->size(), octree->getResolution(), mapname.c_str());
        
        std::string suffix = mapname.substr(mapname.length()-3, 3);
        if (suffix== ".bt"){ // write to binary file:
          if (!octree->writeBinary(mapname)){
            RCLCPP_ERROR(n->get_logger(), "Error writing to file %s", mapname.c_str());
          }
        } else if (suffix == ".ot"){ // write to full .ot file:
          if (!octree->write(mapname)){
            RCLCPP_ERROR(n->get_logger(), "Error writing to file %s", mapname.c_str());
          }
        } else{
          RCLCPP_ERROR(n->get_logger(), "Unknown file extension, must be either .bt or .ot");
        }


      } else{
        RCLCPP_ERROR(n->get_logger(), "Error reading OcTree from stream");
      }

      delete octree;

    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  std::string mapFilename("");
  bool fullmap = false;
  if (argc == 3 && strcmp(argv[1], "-f")==0){
    fullmap = true;
    mapFilename = std::string(argv[2]);
  } else if (argc == 2)
    mapFilename = std::string(argv[1]);
  else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", USAGE);
    exit(1);
  }

  try{
    MapSaver ms(mapFilename, fullmap);
  }catch(std::runtime_error& e){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "octomap_saver exception: %s", e.what());
    exit(2);
  }

  exit(0);
}
