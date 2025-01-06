#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
    for(int i=0;i<argc;++i)
    {
        std::cout<<"argv: "<<argv[i]<<std::endl;
    }
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();
  cout << "1" << endl;
  return 0;
}
