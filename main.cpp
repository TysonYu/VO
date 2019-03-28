//
// create by Tiezheng YU on 3/14/2019
//
/* usage:
./main  /home/icey/Desktop/project/VO/config/default.yaml
*/
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "config.h"
#include "visual_odometry.h"

int main(int argc, char **argv)
{
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1);//内参矩阵
    cv::Mat distortion_coefficients = cv::Mat(4,1, CV_32FC1);//畸变矩阵
    camera_matrix.at<float>(0,0) = 458.654;
    camera_matrix.at<float>(0,1) = 0;
    camera_matrix.at<float>(0,2) = 367.215;
    camera_matrix.at<float>(1,0) = 0;
    camera_matrix.at<float>(1,1) = 457.296;
    camera_matrix.at<float>(1,2) = 248.375;
    camera_matrix.at<float>(2,0) = 0;
    camera_matrix.at<float>(2,1) = 0;
    camera_matrix.at<float>(2,2) = 1;
    distortion_coefficients.at<float>(0) = -0.28340811;
    distortion_coefficients.at<float>(1) = 0.07395907;
    distortion_coefficients.at<float>(2) = 0.00019359;
    distortion_coefficients.at<float>(3) = 1.76187114e-05;

    //--------- input config ----------------------------------------------------
    if ( argc != 2 )
    {
        cout<<"usage: ./main default.yaml file path"<<endl;
        return 1;
    }
    VO::Config::setParameterFile ( argv[1] );

    VO::VisualOdometry::Ptr vo ( new VO::VisualOdometry );

    //--------- input images name and file path ---------------------------------
    string dataset_dir = VO::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    vector<string> rgb_files;
    vector<long> rgb_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file;
        fin>>rgb_time>>rgb_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );

        if ( fin.good() == false )
            break;
    }
    //--------- input point cloud -----------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (dataset_dir+"/data.pcd", *original_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file data.pcd \n");
    }
    vector<Vector3d> cloud;
    boost::timer timer;
    for(int i = 0; i < original_cloud->points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = original_cloud->points[i].x;
        point.y = original_cloud->points[i].y;
        point.z = original_cloud->points[i].z;
        xyz_cloud->points.push_back(point);
    }
    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
    cout << "cloud point size = " << cloud.size() << endl;
    //--------- create a camera object -----------------------------------------
    VO::Camera::Ptr camera ( new VO::Camera );

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    pcl::visualization::PCLVisualizer viewer("result");

    //--------- start VO ------------------------------------------------------
    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    for ( int i=120; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color_raw = cv::imread ( rgb_files[i] );
        Mat color;
        cv::undistort(color_raw, color, camera_matrix, distortion_coefficients);
        cv::imwrite("../data/test_image_refined.png",color);
        // Mat depth = cv::imread ( depth_files[i], -1 );//flag=-1时，8位深度，原通道
        if ( color.data==nullptr)
            break;
        VO::Frame::Ptr pFrame = VO::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        // pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];
        pFrame->point_cloud_ = xyz_cloud;

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;
        
        if ( vo->state_ == VO::VisualOdometry::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        Mat img_show = color.clone();
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        for ( auto& pt:vo->map_->map_points_ )
        {
            pcl::PointXYZ point;
            VO::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            point.x = p->pos_(0);
            point.y = p->pos_(1);
            point.z = p->pos_(2);
            temp_cloud->points.push_back(point);
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );

        // cv::waitKey(1000);
        // viewer.addPointCloud(vo->showResult(),to_string(i));
        // // viewer.addPointCloud(temp_cloud,to_string(i));
        // viewer.setBackgroundColor(0,0,0);
        // viewer.addCoordinateSystem();
        // viewer.spin();
        // viewer.removeCoordinateSystem();
        // viewer.removeAllPointClouds(); 
        cout<<endl;
    }

    return 0;
}