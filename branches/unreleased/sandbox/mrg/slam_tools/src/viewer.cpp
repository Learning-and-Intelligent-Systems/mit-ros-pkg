#include "slam_tools/viewer.hpp"

//Viewer::Viewer(MapEstimate* m, lcm_t* lcm) : m_estimate(m), m_lcm(lcm)
Viewer::Viewer(lcm_t* lcm) : m_lcm(lcm)
{
}

Viewer::~Viewer()
{
}
void Viewer::sendCollection(const ObjectCollection & collection, bool reset)
{
    mrlcm_obj_collection_t objs;    
    size_t n = collection.size();
    if (n > 0) {
        objs.id = collection.id();
        objs.name = (char*) collection.name().c_str();
        objs.type = collection.type();
        objs.reset = reset;
        objs.nobjs = n;
        mrlcm_obj_t poses[n];
        for (size_t i = 0; i < n; i++) {
            const isam::Pose3d & pose = collection(i).pose;
            poses[i].id = collection(i).utime;
            poses[i].x = pose.x();
            poses[i].y = pose.y();
            poses[i].z = pose.z();
            poses[i].yaw = pose.yaw();
            poses[i].pitch = pose.pitch();
            poses[i].roll = pose.roll();
        }
        objs.objs = poses;
        mrlcm_obj_collection_t_publish(m_lcm, "OBJ_COLLECTION", &objs);
    }
}

void Viewer::sendCollection(const LinkCollection & collection, bool reset)
{ 
    mrlcm_link_collection_t linkCollection;
    int n = collection.size();
    if (n > 0) {
        linkCollection.id = collection.id();
        linkCollection.name = (char*) collection.name().c_str();
        linkCollection.type = 0; // unused
        linkCollection.reset = reset;
        linkCollection.nlinks = n;
        mrlcm_link_t links[n];
        for (int i = 0; i < n; i++) {
            const Link & link = collection(i);
            links[i].id = link.id;
            links[i].collection1 = link.collection1;
            links[i].id1 = link.id1;
            links[i].collection2 = link.collection2;
            links[i].id2 = link.id2;            
        }
        linkCollection.links = links; 
        mrlcm_link_collection_t_publish(m_lcm, "LINK_COLLECTION", &linkCollection);
    }        
}

void Viewer::sendCollection(const PointCloudCollection & collection, bool reset)
{
    //  mrlcm_points_collection_t points;
    mrlcm_point3d_list_collection_t point_lists;

//  std::queue<Scan*> & scans = scanDisplayQueue[mapEstimate];
    size_t m = collection.size();
    
    point_lists.id = collection.id();
    point_lists.name = (char *)collection.name().c_str(); // Use channel name?
    point_lists.type = MRLCM_POINT3D_LIST_COLLECTION_T_POINT;
    
    /// @todo reset if reset is requested also reset on first run
    point_lists.reset = reset;
    point_lists.nlists = m;
    mrlcm_point3d_list_t point_list[m];
    
    for(size_t i=0; i<collection.size(); i++)
    {
//        boost::shared_ptr<PointCloudPtr > dataInstance = collection(i);
//            
//        PointCloudPtr pointCloud = dataInstance->data;
        PointCloudPtr pointCloud =  *(collection(i));
                
        mrlcm_point3d_list_t* points = &(point_list[i]);
        int64_t scantime = pointCloud->utime(); // dataInstance->node->utime;
        points->ncolors = 0;
        points->colors = NULL;
        points->nnormals = 0;
        points->normals = NULL;
        points->npointids = 0;
        points->pointids = NULL;

        if (pointCloud) {
          size_t k = pointCloud->size();

          mrlcm_point3d_t* entries = new mrlcm_point3d_t[k];

          points->id = scantime;
          points->collection = collection.objectCollectionId();
          points->element_id = scantime;
          points->npoints = k;
          for (size_t j=0;j<k;j++) {
              entries[j].x = pointCloud->points()[j].x();
              entries[j].y = pointCloud->points()[j].y();
              entries[j].z = pointCloud->points()[j].z();           
          }
          points->points = entries;         
        }
        else          
        {
          points->id = 0;
          points->collection = 0;
          points->element_id = 0;
          points->npoints = 0;        
          points->points = 0;         
        }
    }
    
    point_lists.point_lists = point_list;
    mrlcm_point3d_list_collection_t_publish(m_lcm,"POINTS_COLLECTION",&point_lists);            
    for (int i=0;i<point_lists.nlists;i++) {
        delete point_lists.point_lists[i].points;
    }   
}

void Viewer::setConfig (const mrlcm_collection_config_t config)
{
    mrlcm_collection_config_t_publish(m_lcm, "COLLECTION_CONFIG", &config);  
}

void Viewer::update()
{
  /*
    mrlcm_obj_collection_t objs;
    //mrlcm_link_collection_t links;
    Pose2d pose;

    const std::map<int64_t, Pose3d_Node*> nodes; // & nodes = m_estimate->getNodes();    
    int n = nodes.size();
    if (n > 1) {
      objs.id = 0;
      objs.name = (char*) "Navigation";
      objs.type = MRLCM_OBJ_COLLECTION_T_POSE;
      objs.reset = true;
      objs.nobjs = n;
      mrlcm_obj_t poses[n];
      int i = 0;
      for (std::map<int64_t, Pose3d_Node*>::const_iterator it= nodes.begin(); 
           it != nodes.end(); 
           i++,it++) {
        
        Pose3d_Node* node = (*it).second;
        Pose3d pose = node->value();
        poses[i].id = (*it).first;
        poses[i].x = pose.x();
        poses[i].y = pose.y();
        poses[i].z = pose.z();;
        poses[i].yaw = pose.yaw();
        poses[i].pitch = pose.pitch();
        poses[i].roll = pose.roll();
      }
      objs.objs = poses;
      mrlcm_obj_collection_t_publish(m_lcm, "OBJ_COLLECTION", &objs);
    }
    */    
}

