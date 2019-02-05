
#include <cardboard/common.h>
#include <cardboard/AlignModels.h>
#include <cardboard/SceneHypothesis.h>
#include <stdlib.h>


//-----Enter "detectThenTrack" to detect then track objects-----//


ros::ServiceClient align_models_client;
cardboard::AlignModels srv;
bool terminal_input = false;
bool detected = false;

/* handle an All_Hulls message */
void all_hulls_callback(furniture::All_Hulls msg)
{
  ROS_INFO("Got All_Hulls msg\n");
  geometry_msgs::Polygon table_polygon;

  if (msg.hulls.size() > 0) {
    // hack: assume first table is the one we want
    table_polygon.points.resize(msg.hulls[0].polygon.points.size());
    for (size_t i = 0; i < msg.hulls[0].polygon.points.size(); i++) {
      table_polygon.points[i].x = msg.hulls[0].polygon.points[i].x;
      table_polygon.points[i].y = msg.hulls[0].polygon.points[i].y;
      table_polygon.points[i].z = msg.hulls[0].polygon.points[i].z;
    }
  }


  srv.request.surface_polygons.push_back(table_polygon);

    if (terminal_input == true){

    	 if (align_models_client.call(srv))
    		  {
    		  ROS_INFO("OK");
    		  }
    		  else
    		  {
    		  ROS_ERROR("Failed to call service align_models");
    		  }
    }


}



void scene_hyp_callback(cardboard::SceneHypothesis H){
	if (detected == false){
        ROS_INFO("called scene hyp");
	// Add objects and poses from Scene Hypothesis
	for (uint i = 0; i < H.objects.size(); i++){
	ROS_INFO("Called");
		srv.request.initial_poses.push_back(H.objects[i].pose);
		srv.request.models.push_back(H.objects[i].name);
	}
	  srv.request.header = H.header;

	  srv.request.header.stamp.sec += 1.0;  //dbug -- test timeout
	  srv.request.timeout.sec = 15.0;

	  if (align_models_client.call(srv))
	  {
	  ROS_INFO("OK");
	  detected = true;
          ROS_INFO("Detected");
	  }
	  else
	  {
	  ROS_ERROR("Failed to call service align_models");
	  }
    // clear poses and models
    srv.request.initial_poses.clear();
    srv.request.models.clear();

}}

void alignment_hyp_callback(cardboard::SceneHypothesis H){
	if (detected == true){
		for (uint i = 0; i < H.objects.size(); i++){
			ROS_INFO("Aligning");
			srv.request.initial_poses.push_back(H.objects[i].pose);
			srv.request.models.push_back(H.objects[i].name);
		}
		srv.request.header = H.header;
		srv.request.header.stamp.sec += 1.0;
		srv.request.timeout.sec = 15.0;

		if (align_models_client.call(srv)){
			ROS_INFO("OK");
		}
		else{
			ROS_ERROR("Failed to call service align_models");
		}
		srv.request.initial_poses.clear();
		srv.request.models.clear();
	}
}


int main(int argc, char *argv[])
{
  // init ROS
  ros::init(argc, argv, "align_models_client");
  ros::NodeHandle nh;




  //-------input "model name" x y z qa qx qy qz ----------//


  if (argc == 1){
	  ROS_INFO("Align all");
  }


  if (argc > 2){
	  ROS_INFO("Terminal Input\n");
  }

  if(argc > 1){
	  ROS_INFO("Terminal Input\n");
	  terminal_input = true;
	  srv.request.models.push_back(argv[1]);
	  geometry_msgs::Pose pose;
	  double x = std::atof(argv[2]);
	  double y = std::atof(argv[3]);
	  double z = std::atof(argv[4]);
	  pose.position.x = x;
	  pose.position.y = y;
	  pose.position.z = z;
	  double f1 = std::atof(argv[5]);
	  double f2 = std::atof(argv[6]);
	  double f3 = std::atof(argv[7]);
	  double f4 = std::atof(argv[8]);
	  pose.orientation.w = f1;
	  pose.orientation.x = f2;
	  pose.orientation.y = f3;
	  pose.orientation.z = f4;

	  srv.request.initial_poses.push_back(pose);

	  ros::Time time;
	  srv.request.header.stamp = time.now();
	  srv.request.header.stamp.sec = 0;
	  srv.request.timeout.sec = 15.0;
	  srv.request.header.frame_id = "base_footprint";
	  ROS_INFO("Aligning to position (%2f ,%2f , %2f) with orientation (%2f, %2f, %2f, %2f )", x, y, z, f1, f2, f3, f4);
  }
  ros::Subscriber sub_scene_hyp = nh.subscribe("scene_hypothesis", 1, scene_hyp_callback);
  ros::Subscriber sub_all_hulls = nh.subscribe("/convex_hulls", 1, all_hulls_callback);
  ros::Subscriber align_scene_hyp = nh.subscribe("alignment_hypothesis", 1, alignment_hyp_callback);

  align_models_client = nh.serviceClient<cardboard::AlignModels>("align_models");

  ros::spin();

  return 0;
}
