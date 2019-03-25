#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif


#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_gen/PolyTrajGen.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <QThread>
#include <QStringListModel>

class QNode : public QThread {
Q_OBJECT

public:
   QNode(int argc, char** argv, const std::string &name );
   ~QNode();

   bool on_init();
   void shutdown();
   void run();

   QStringListModel* loggingModel() { return &logging; }
   const std::string& nodeName() { return node_name; }

   // waypoints from rviz
   void waypoint_cb(const geometry_msgs::PoseStampedConstPtr&);

   // trajectory call
   bool traj_gen_call(double tf,
                     geometry_msgs::Twist v0,
                     geometry_msgs::Twist a0,
                     TrajGenOpts option);
   // initialize the waypoints in queue
   void wpnts_init();
   // load the saved waypoints
   void queue_file_load(int,vector<geometry_msgs::PoseStamped>&);
   // writing directory
   string write_path;

   std::vector<geometry_msgs::PoseStamped> queue; // the wpnts from user

   // flags
   bool is_connected = false; // to ros
   bool is_insert_permit; // is insert waypoints active
   bool is_in_session = false; // is in simulation session
   bool is_init = false;
   bool is_path = false; // was path computed
   double cur_spline_eval_time = 0;
   double total_duration = 5;
   double global_time_end;


   // params
   ros::Time pred_start_time;
   ros::Time session_ckp;
   double record_dt = 0.5;
   bool arrow_record_switch = true;


Q_SIGNALS:
   void loggingUpdated();
   void rosShutdown();
   void writeOnBoard(QString);

protected:
   void ros_comms_init();
   int init_argc;
   char** init_argv;
   QStringListModel logging;
   const std::string node_name;
   std::string target_wnpt_topic; // default: /target_wnpt1 in rviz



private:
   std_msgs::Header header;

   // planner object
   PathPlanner planner;

   // waypoint input from user

   ros::Subscriber wpnt_sub; // the pnts subscriber

   ros::Publisher wpnt_marker_pub; // the wpnts from user (vis)
   visualization_msgs::MarkerArray wpnt_markerArray;

   // generated global path
   traj_gen::PolySplineXYZ spline;

   ros::Publisher spline_path_pub;
   nav_msgs::Path spline_path;

   ros::Publisher spline_knot_pub;
   visualization_msgs::MarkerArray knot_markerArray; // knot viz


   // for publising path
   ros::Publisher target_goal_pub; // publisher for target goal
   string control_point_topic; // target goal point topic name
   geometry_msgs::PoseStamped target_goal; // goal point for control

};


#endif // QNODE_H

