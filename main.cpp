//
// create by Tiezheng YU on 3/14/2019
//
/* usage:
./main  /home/icey/Desktop/project/VO/config/default.yaml
*/


#include <common_include.h>
#include <config.h>
#include <data_loader.h>
#include <camera.h>

int main(int argc, char **argv)
{
    //--------- input config ----------------------------------------------------
    if ( argc != 2 )
    {
        cout<<"usage: ./main default.yaml file path"<<endl;
        return 1;
    }
    VO::Config::setParameterFile ( argv[1] );

    // myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

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
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (dataset_dir+"/V1_pointcloud.pcd", *original_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file data.pcd \n");
    }
    vector<Vector3d> cloud;
    boost::timer timer;
    for(int i = 0; i < original_cloud->points.size(); i++)
    {
        Vector3d point;
        point(0) = original_cloud->points[i].x;
        point(1) = original_cloud->points[i].y;
        point(2) = original_cloud->points[i].z;
        cloud.push_back(point);
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



    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    // for ( int i=0; i<rgb_files.size(); i++ )
    // {
    //     cout<<"****** loop "<<i<<" ******"<<endl;
    //     Mat color = cv::imread ( rgb_files[i] );
    //     Mat depth = cv::imread ( depth_files[i], -1 );//flag=-1时，8位深度，原通道
    //     if ( color.data==nullptr || depth.data==nullptr )
    //         break;
    //     myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    //     pFrame->camera_ = camera;
    //     pFrame->color_ = color;
    //     pFrame->depth_ = depth;
    //     pFrame->time_stamp_ = rgb_times[i];

    //     boost::timer timer;
    //     vo->addFrame ( pFrame );
    //     cout<<"VO costs time: "<<timer.elapsed() <<endl;

    //     if ( vo->state_ == myslam::VisualOdometry::LOST )
    //         break;
    //     SE3 Twc = pFrame->T_c_w_.inverse();

    //     // show the map and the camera pose
    //     cv::Affine3d M (
    //         cv::Affine3d::Mat3 (
    //             Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
    //             Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
    //             Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
    //         ),
    //         cv::Affine3d::Vec3 (
    //             Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
    //         )
    //     );

    //     Mat img_show = color.clone();
    //     for ( auto& pt:vo->map_->map_points_ )
    //     {
    //         myslam::MapPoint::Ptr p = pt.second;
    //         Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
    //         cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
    //     }

    //     cv::imshow ( "image", img_show );
    //     cv::waitKey ( 1 );
    //     vis.setWidgetPose ( "Camera", M );
    //     vis.spinOnce ( 1, false );
    //     cout<<endl;
    // }

    return 0;
}