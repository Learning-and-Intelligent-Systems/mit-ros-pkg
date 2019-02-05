#ifndef _HAUV_VIEWER_H
#define _HAUV_VIEWER_H
/**
 * @file viewer.h
 * @brief Handles the communication the the mr-viewer. Sending object collections and links.
 * @author Hordur Johannsson
 * @version $Id: $
 *
 * (c) 2010 Massachusetts Institute of Technology
 *
 */

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

#include <lcm/lcm.h>
#include <isam/isam.h>

#include "mrlcm_obj_collection_t.h"
#include "mrlcm_obj_t.h"
#include "mrlcm_link_collection_t.h"
#include "mrlcm_link_t.h"
#include "mrlcm_point3d_list_collection_t.h"
#include "mrlcm_point3d_list_t.h"
#include "mrlcm_point3d_t.h"
#include "mrlcm_collection_config_t.h"
#include "pointcloud.hpp"

/**
 * An implementation of the viewer interface that
 * communicates with the mr-viewer using LCM.
 */
#if 0
class Viewer
{
public:
    Viewer( lcm_t* lcm); //MapEstimate* m,
    ~Viewer();
    
    /**
     * Sends and updated view to the viewer
     */
    void update();
private:
   // MapEstimate* m_estimate;
    lcm_t* m_lcm;    
}
#endif

class ObjectCollection;
class LinkCollection;
class PointCloudCollection;

/**
 * Handles communication with a viewer to display data from the map database.
 */ 
class Viewer
{
public:
    Viewer(lcm_t* lcm); //MapEstimate* m,
  
    virtual ~Viewer();
    virtual void sendCollection(const ObjectCollection & collection, bool reset = false);
    virtual void sendCollection(const LinkCollection & collection, bool reset = false);
    virtual void sendCollection(const PointCloudCollection & collection, bool reset = false);

    void setConfig (const mrlcm_collection_config_t config);
    
    /**
     * Sends and updated view to the viewer
     */
    void update();
private:
   // MapEstimate* m_estimate;
    lcm_t* m_lcm;     
};

class Collection
{
public:
    Collection() {}
    Collection(int id, const std::string & name) : m_id(id), m_name(name) {}
    virtual ~Collection() {}
    int id() const {return m_id;}
    const std::string & name() const {return m_name;}
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) = 0;
    virtual void clear() = 0;
private:
    int m_id;
    std::string m_name;
};

/**
 * Contains a timed pose
 */
struct Pose3dTime
{
    Pose3dTime(int64_t utime, const isam::Pose3d & pose) : utime(utime), pose(pose) {}
    int64_t utime;
    isam::Pose3d pose;
};

struct Link
{
    Link(int64_t id, int32_t collection1, int64_t id1,
             int32_t collection2, int64_t id2)
             : id(id),
               collection1(collection1), id1(id1),
               collection2(collection2), id2(id2) {}
    int64_t id;
    int32_t collection1;
    int64_t id1;
    int32_t collection2;
    int64_t id2;
};

class ObjectCollection : public Collection
{
public:
    ObjectCollection(int id, const std::string & name, int type=MRLCM_OBJ_COLLECTION_T_AXIS3D) : Collection(id,name),m_first(true),m_type(type) {}
    virtual ~ObjectCollection() {}
    void add(const Pose3dTime & poseTime) { m_poses.push_back(poseTime); }
    void add(int64_t utime, const isam::Pose3d & pose) {
        add(Pose3dTime(utime, pose));
    }
    const std::vector<Pose3dTime> & poses() const {return m_poses;}
    std::vector<Pose3dTime> & poses() {return m_poses;}
    const Pose3dTime & operator()(size_t i) const {return m_poses[i];}
    virtual void clear() {m_poses.clear();}
    size_t size() const {return m_poses.size();}
    int type() const {return m_type;}
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        if (m_first)
        {
            reset = true;
            m_first = false;
        }

        viewer->sendCollection(*this,reset);
    };

    static boost::shared_ptr<ObjectCollection> makeCollection(int id, const std::string & name)
    {
        return boost::make_shared<ObjectCollection>(id, name);
    }
private:
    std::vector<Pose3dTime> m_poses;    
    bool m_first;
    int m_type; //MRLCM_OBJ_COLLECTION_T_AXIS3D;
};

class LinkCollection : public Collection
{
public:
    // LinkCollection() : m_first(true) {}
    LinkCollection(int id, const std::string & name) : Collection(id,name), m_first(true) {}
    virtual ~LinkCollection() {}
    
    void add(int64_t id, int collection1, int64_t id1,
             int collection2, int64_t id2) {
        m_links.push_back(Link(id, collection1, id1, collection2, id2));
    }
    const std::vector<Link> & links() const {return m_links;}
    std::vector<Link> & links() {return m_links;}
    const Link & operator()(size_t i) const {return m_links[i];}
    virtual void clear() {m_links.clear();}
    size_t size() const {return m_links.size();}
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        if (m_first)
        {
            reset = true;
            m_first = false;
        }
      
        viewer->sendCollection(*this,reset);
    };

    static boost::shared_ptr<LinkCollection> makeCollection(int id, const std::string & name)
    {
        return boost::make_shared<LinkCollection>(id, name);
    }   
private:
    std::vector<Link> m_links;  
    bool m_first;
};

class PointCloudCollection : public Collection
{
public:
    PointCloudCollection(int id, const std::string & name, int objectCollectionId) 
        : Collection(id,name), m_objectCollectionId(objectCollectionId), m_first(true) {}
    PointCloudCollection(const PointCloudCollection& collection) 
      : Collection(collection), 
        m_pointClouds(collection.m_pointClouds),
        m_objectCollectionId(collection.m_objectCollectionId),
        m_first(collection.m_first)
    { }

    virtual ~PointCloudCollection() {}
        
    void add(boost::shared_ptr<PointCloudPtr > pointCloud) {
        boost::mutex::scoped_lock scoped_lock(mutex);         
        m_pointClouds.push_back(pointCloud);
    }
    boost::shared_ptr<PointCloudPtr > operator()(size_t i) const {
      boost::mutex::scoped_lock scoped_lock(mutex); 
      return m_pointClouds[i];
    }
    virtual void clear() {
      boost::mutex::scoped_lock scoped_lock(mutex); 
      m_pointClouds.clear();
    }
    size_t size() const {
      boost::mutex::scoped_lock scoped_lock(mutex); 
      return m_pointClouds.size(); 
    }
    int objectCollectionId() const { return m_objectCollectionId; }
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        boost::mutex::scoped_lock scoped_lock(mutex);
        if (m_first)
        {
            reset = true;
            m_first = false;
        }
        PointCloudCollection collection(*this);
        scoped_lock.unlock();

        viewer->sendCollection(collection, reset);
        clear();
    };
    
    static boost::shared_ptr<PointCloudCollection> makeCollection(int id, const std::string & name, int objectCollectionId)
    {
        return boost::make_shared<PointCloudCollection>(id, name, objectCollectionId);
    }
private:    
    mutable boost::mutex mutex;  
    std::vector<boost::shared_ptr<PointCloudPtr > > m_pointClouds;
    const int m_objectCollectionId;
    bool m_first;
};

#endif
