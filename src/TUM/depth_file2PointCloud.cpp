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


///////////////////////////////////////////////////////
///
///    读取depth文件生成点云
///
///////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    string file_path="/home/anshuai/yft/CLion/clion_project/qcit_depthmap_rmd/output/tum3";
    string pose_file="tum3_pose.txt";
    ///路径/图像文件夹/图像名/图像格式
    boost::format image_path("%s/%s/%s.%s");
    ///路径/位姿文件名
    boost::format pose_path("%s/%s");

    ifstream data_in( (pose_path%file_path%pose_file).str() );
    if(!data_in)
    {
        cout<<"位姿文件路径:"<< (pose_path%file_path%pose_file).str() <<endl;
        cout<<"位姿文件读取失败!"<<endl;
        return  -1;
    }
    int num=1;

    double fx = 525.0  ;//# focal length x
    double fy = 525.0  ;//# focal length y
    double cx = 319.5  ;//# optical center x
    double cy = 239.5  ;//# optical center y
    //double factor = 5000 ;//# for the 16-bit PNG files

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr pc( new PointCloud );

    //while( data_in.good() )
    for(int i=0;i<num;i++)
    {
        std::string pose_num;
        data_in>>pose_num;
        double data[7] = {0};
        for ( auto& d:data )
            data_in>>d;
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));

        cout<<"正在将图像转换为点云..."<<endl;
        cout<<"转换图像中: "<<i<<endl;
        PointCloud::Ptr pointCloud( new PointCloud );

        cv::Mat color = cv::imread( ( image_path %file_path%"reference"%pose_num%"png").str() );
        //cv::Mat depth = cv::imread( ( image_path %file_path%"depth"%pose_num%"png").str() );
        ifstream depthfile(( image_path %file_path%"depth"%pose_num%"depth").str());

        if(color.empty())
        {
            cout<<"编号 "<<pose_num<<" 读取彩色图失败!"<<endl;
            return -1;
        }
        if(depthfile.bad())
        {
            cout<<"编号 "<<pose_num<<" 读取深度文件失败!!"<<endl;
            return -1;
        }

        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                float d;
                depthfile>>d;
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
                pointCloud->points.push_back( p );
            }
        (*pc)+=*pointCloud;
            depthfile.close();
    }

    pc->is_dense = false;
    cout<<"点云共有"<<pc->size()<<"个点."<<endl;
    boost::format pcd_name("../output/d_%s.pcd");
    pcl::io::savePCDFileBinary((pcd_name%"tum3").str(), *pc );
    return 0;
}



