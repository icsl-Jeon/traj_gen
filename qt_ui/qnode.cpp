#include "qnode.h"


// constructor 
QNode::QNode(int argc, char** argv, const std::string &name ) :
    init_argc(argc),
    init_argv(argv),
    node_name(name)
    {

    is_insert_permit = false;
    header.frame_id = "/world";
}

// destructor 
QNode::~QNode() {
    shutdown();
}

void QNode::shutdown() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::on_init(){

    ros::init(init_argc,init_argv,node_name);
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    ros_comms_init();
    start();
    return true;
}


void QNode::ros_comms_init(){
    
    ros::NodeHandle nh("~");
    nh.param("world_frame_id",header.frame_id,std::string("/world"));
    
    nh.param("waypoint_topic",target_wnpt_topic,std::string("/waypoint"));
    target_goal.header = header;    
    wpnt_sub = nh.subscribe(target_wnpt_topic,1,&QNode::waypoint_cb,this);
    wpnt_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints_marker",1);
    spline_path_pub = nh.advertise<nav_msgs::Path>("trajectory",1);
    spline_knot_pub = nh.advertise<visualization_msgs::Marker>("trajectory_knots",1);
    target_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("control_pose",1);
    safe_corridor_pub = nh.advertise<visualization_msgs::Marker>("safe_corridor",1);
    
}

bool QNode::traj_gen_call(double tf,
                     geometry_msgs::Twist v0,
                     geometry_msgs::Twist a0,
                     TrajGenOpts option){
    
                         
    nav_msgs::Path waypoints;

    // initialize the total time 
    previous_elapsed = 0;
    waypoints.poses = queue;
    TimeSeries knots(queue.size());
    knots.setLinSpaced(queue.size(),0,tf);
    
    planner.path_gen(knots,waypoints,v0,a0,option);                         

    is_path = planner.is_spline_valid();
    
    if(is_path){
        spline_path = planner.get_path();
        spline_path.header = header;
    }
    
    return is_path;
}


void QNode::queue_file_load(int target_idx, vector<geometry_msgs::PoseStamped>& wpnt_replace ){

    this->queue = wpnt_replace;
    wpnt_markerArray.markers.clear();
    Q_EMIT writeOnBoard(QString::fromStdString("loaded "+std::to_string(wpnt_replace.size())+" pnts"));

    
    for(auto it = wpnt_replace.begin();it<wpnt_replace.end();it++){
        
        visualization_msgs::Marker marker;

        marker.action = 0;
        marker.header = header;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = it->pose;
        
        std::cout<< it->pose.position.x <<" , "<< it->pose.position.y <<" , "<<it->pose.position.z<<std::endl;

        float scale = 0.1;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        if(target_idx == 0)
            marker.color.r = 1;
        else
            marker.color.b = 1;

        marker.color.a = 1;
        marker.id = wpnt_markerArray.markers.size();
        wpnt_markerArray.markers.push_back(marker);
        
    }

} 


void QNode::waypoint_cb(const geometry_msgs::PoseStampedConstPtr & pose){
    if (is_insert_permit){
        ROS_INFO("point received");
        queue.push_back(*pose);
        visualization_msgs::Marker marker;
        
        marker.action = 0;
        marker.header = header;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = pose->pose;
        float scale = 0.1;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = 1;
        marker.color.a = 1;
        marker.id = queue.size();
        wpnt_markerArray.markers.push_back(marker);
        
        // lets print in board  
        string line = "recieved point: " 
                        + to_string(pose->pose.position.x) + " , "
                        + to_string(pose->pose.position.y);
        
        Q_EMIT writeOnBoard(QString::fromStdString(line));
        
    }else{
         
        Q_EMIT writeOnBoard(QString::fromStdString("insertion not allowed"));
 
        // std::cout<<"insertion not allowed"<<std::endl;
    }
}


// we don't have to include run explicitly in the main function
void QNode::run(){
    ros::Rate loop_rate(50);

    while(ros::ok()){
        // the marker waypoints from user 
        wpnt_marker_pub.publish(wpnt_markerArray);
        
        // generated path  
        if(is_path){
            spline_path_pub.publish(spline_path);
            safe_corridor_pub.publish(planner.get_safe_corridor_marker());
            visualization_msgs::Marker knots = planner.get_knots_marker();
            knots.header.frame_id = spline_path.header.frame_id;
            spline_knot_pub.publish(knots);
            
            // control point publish
            if(is_in_session){
                double t_eval = (ros::Time::now() - button_click_time).toSec() + previous_elapsed;
                target_goal.pose.position = planner.point_eval_spline(t_eval);
                target_goal_pub.publish(target_goal);            
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    Q_EMIT rosShutdown();


}

