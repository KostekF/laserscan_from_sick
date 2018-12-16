#include "laserscanFromSick.h"



int main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_sick_pointcloud_to_laserscan");
  LaserscanFromSick pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
