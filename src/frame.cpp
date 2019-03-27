/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "frame.h"

namespace VO
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), point_cloud_(point_cloud), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    float d = depth_.ptr<float>(y)[x];
    // if ( d! = max_depth )
    // {
    //     return double(d);
    // }
    // else 
    // {
    //     // check the nearby points 
    //     int dx[4] = {-1,0,1,0};
    //     int dy[4] = {0,-1,0,1};
    //     for ( int i=0; i<4; i++ )
    //     {
    //         d = depth_.ptr<float>( y+dy[i] )[x+dx[i]];
    //         if ( d!=0 )
    //         {
    //             return double(d);
    //         }
    //     }
    // }
    for(int i = x-2; i < x+3; i++)
        for(int j = y-2; j < y+3; j++)
        {
            if(depth_.ptr<float>(j)[i] < d)
                d = depth_.ptr<float>(j)[i];
        }
    return double(d);
    // return -1.0;
}

void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}

void Frame::getDepthImage()
{
    // ector<Vector3d> cloud;
    // boost::timer timer;
    // for(int i = 0; i < point_cloud_->points.size(); i++)
    // {
    //     Vector3d point;
    //     point(0) = point_cloud_->points[i].x;
    //     point(1) = point_cloud_->points[i].y;
    //     point(2) = point_cloud_->points[i].z;
    //     cloud.push_back(point);
    // }
    Eigen::Matrix4d Rt = T_c_w_.matrix();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud (*point_cloud_, *cloud, Rt);//image，Z上未归一化的像素坐标系
    //-- //滤掉后面的点
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 50);//delete all the point that z<0 && z>40
    pass.filter (*cloud);
    Eigen::Matrix4f intrisic;//相机内参
    intrisic << camera_->fx_, 0, camera_->cx_, 0,    0, camera_->fy_, camera_->cy_, 0,    0, 0, 1, 0,    0, 0, 0, 1;
    pcl::transformPointCloud (*cloud, *cloud, intrisic);//result，Z上未归一化的像素坐标系

    for(int i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].x = cloud->points[i].x / cloud->points[i].z;
        cloud->points[i].y = cloud->points[i].y / cloud->points[i].z;
        if(cloud->points[i].z > max_depth)
            max_depth = cloud->points[i].z;
    }
    cv::Mat M(color_.rows, color_.cols, CV_32F);//把点投影到M上
    cv::Mat P(color_.rows, color_.cols, CV_32F);//扩展投影点
    cv::MatIterator_<float>Mbegin,Mend;//遍历所有像素，初始化像素值
	for (Mbegin=M.begin<float>(),Mend=M.end<float>();Mbegin!=Mend;++Mbegin)
		*Mbegin=max_depth;
    for(int i=0;i<cloud->points.size();i++)//把深度值投影到图像M上
    {
        if(cloud->points[i].x>=0  && cloud->points[i].x<color_.cols && cloud->points[i].y>=0 && cloud->points[i].y<color_.rows)
        {
            if( cloud->points[i].z < M.at<float>(cloud->points[i].y,cloud->points[i].x))
                M.at<float>(cloud->points[i].y,cloud->points[i].x) = cloud->points[i].z;
        }
    }
    for(int count = 0; count < 9; count ++)
    {
        if (count%2 == 0) 
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(M.at<float>(i,j) == max_depth)
                    {
                        float temp = max_depth;
                        float sum = 0;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(M.at<float>(n,m) < temp )
                                {
                                    temp = M.at<float>(n,m);
                                }   
                            }
                        }
                        P.at<float>(i,j) = temp;
                    }
                    else
                        P.at<float>(i,j)  = M.at<float>(i,j);
		        }
            }
        }
        else
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(P.at<float>(i,j) == max_depth)
                    {
                        float temp = max_depth;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(P.at<float>(n,m) < temp)
                                {
                                    temp = P.at<float>(n,m);
                                }
                            }
                        }
                        M.at<float>(i,j) = temp;
                    }
                    else
                        M.at<float>(i,j)  = P.at<float>(i,j);
		        }
            }
        }
    }
    depth_ = M;
    
    // cv::Mat result(color_.rows, color_.cols, CV_8U);//把点投影到M上
    // for(int i = 0; i < M.rows; i++)
    //     for(int j = 0; j < M.cols; j++)
    //     {
    //         result.at<char>(i,j) =  M.at<float>(i,j)/max_depth *255;
    //     }

    // cv::imshow("depthmap", result);
    // cv::waitKey(0);
    // cv::destroyWindow("depthmap");
    // cout << M << endl;
    // pcl::visualization::PCLVisualizer viewer("Keypoints viewer");
    // viewer.addPointCloud(cloud, "sample cloud");
    // viewer.setBackgroundColor(0,0,0);
    // viewer.addCoordinateSystem();
    // viewer.addCoordinateSystem();
    // while(!viewer.wasStopped())
    //     viewer.spinOnce();

}


}
