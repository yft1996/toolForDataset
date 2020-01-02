#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
int main( int argc, char** argv )
{
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<std::string> depthfile;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿

    std::string data_path="/home/anshuai/yft/CLion/clion_project/qcit_depthmap_rmd/output";
    std::string method="REMODE";
    boost::format pose_path("%s/%s/%s_pose.txt");///位姿文件路径
    int image_num=6;
    ifstream fin((pose_path%data_path%method%method).str() );
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }

    boost::format image_path("%s/%s/%s/%04d.%s");

    for ( int i=0; i<image_num; i++ )
    {
        colorImgs.push_back( cv::imread( (image_path%data_path%method%"reference"%(i+1)%"png").str() ));
        depthfile.push_back((image_path%data_path%method%"depth"%(i+1)%"depth").str()) ;

        std::string imagename;
        fin>>imagename;
        std::cout<<imagename<<std::endl;
        double data[7] = {0};
        for ( auto& d:data )
            fin>>d;
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }

    // 计算点云并拼接
    // 相机内参
    double fx = 481.2 ;
    double fy = -480.0;
    double cx = 319.5 ;
    double cy = 239.5 ;

    //double fx = 535.4 ;
    //double fy = 539.2 ;
    //double cx = 320.1 ;
    //double cy = 247.6 ;

    //double depthScale = 1000.0;

    cout<<"正在将图像转换为点云..."<<endl;

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );
    for ( int i=0; i<image_num; i++ )
    {
        PointCloud::Ptr current( new PointCloud );
        cout<<"转换图像中: "<<i+1<<endl;
        cv::Mat color = colorImgs[i];
        Eigen::Isometry3d T = poses[i];

        std::ifstream depth(depthfile[i]);

        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                float d;
                depth>>d;
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d norm_point;

                norm_point[0]=(u-cx)/fx;
                norm_point[1]=(v-cy)/fy;
                norm_point[2]=1;
                float length = sqrt( norm_point[0]*norm_point[0]+
                                     norm_point[1]*norm_point[1]+
                                     norm_point[2]*norm_point[2]) ;
                norm_point[0]/=length;
                norm_point[1]/=length;
                norm_point[2]/=length;

                Eigen::Vector3d point=norm_point*d;
                Eigen::Vector3d pointWorld = T*point;

                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                current->points.push_back( p );
            }

        depth.close();
        // depth filter and statistical removal
        PointCloud::Ptr tmp ( new PointCloud );
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp );
        (*pointCloud) += *tmp;
    }

    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
/**/
    // voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       // resolution
    PointCloud::Ptr tmp ( new PointCloud );
    voxel_filter.setInputCloud( pointCloud );
    voxel_filter.filter( *tmp );
    tmp->swap( *pointCloud );

    cout<<"滤波之后，点云共有"<<pointCloud->size()<<"个点."<<endl;
/**/
    boost::format pcd_name("../output/d_%s.pcd");
    pcl::io::savePCDFileBinary((pcd_name%method).str(), *pointCloud );
    return 0;
}
