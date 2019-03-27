//
//  Created by Tyson YU on 3/25/2019
//

#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "camera.h"

namespace VO
{
    
// forward declare 
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
    Mat                            color_, depth_; // color and depth image 
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    float                          max_depth;
public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat(), pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ = nullptr);
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );

    //get depth map from point cloud
    void getDepthImage();
};

}

#endif // FRAME_H
