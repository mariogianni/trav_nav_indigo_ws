#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "Exception.h"


DEFINE_EXCEPTION(TransformException, Exception)	


///	\class Transform
///	\author Alcor
///	\brief 
///	\note 
/// 	\todo 
///	\date
///	\warning
class Transform
{
public:
    
    Transform(const std::string& parent = "/map", const std::string& child="/base_link");
    ~Transform();
        
public: // getters 
    
    // get transform for input frames 
    tf::StampedTransform get(const std::string& parent, const std::string& child);

    // get transform for set frames 
    tf::StampedTransform get();
    
    bool isOk() const {return b_ok_;}
    
public: // setters 
    
    // set frames 
    void set(const std::string& parent, const std::string& child);
        
protected:
    tf::TransformListener *p_tf_listener_;
    
    std::string parent_;
    std::string child_;
    bool b_frames_set_; 
    bool b_ok_; // true if we received a valid transform, false otherwise 
};


#endif
