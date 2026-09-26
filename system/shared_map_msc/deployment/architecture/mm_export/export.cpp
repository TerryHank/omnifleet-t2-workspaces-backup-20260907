#include <mp2p_icp/metricmap.h>
#include <mola_metric_maps/HashedVoxelPointCloud.h>
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=3)return 2;
  mp2p_icp::metric_map_t map;
  if(!map.load_from_file(argv[1]))return 3;
  auto layer=std::dynamic_pointer_cast<mola::HashedVoxelPointCloud>(map.layers.at("localmap"));
  if(!layer)return 4;
  if(!layer->saveToTextFile(argv[2]))return 5;
  std::cout<<"Saved localmap points for offline registration\n";
}
