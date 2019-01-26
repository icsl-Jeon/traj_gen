#include <ros/ros.h>
#include <traj_gen/PolyTrajGen.h>
int main(int argc,char *argv[]){

    ros::init(argc,argv,"traj_gen_server");
    TrajGen::traj_gen_server server;
    
    ros::spin();
    return 0;

}
