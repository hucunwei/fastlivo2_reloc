#include "optimization.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // RTK-GNSS backend optimization (GTSAM factor graph, auto-degrades without GPS data)
  optimization opti(nh);

  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  
  return 0;
}