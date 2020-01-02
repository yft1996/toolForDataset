#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings

///////////////////////////////////////////////////////
///
///    读取深度图 写入到 深度文件
///
///////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    string file_path="/home/anshuai/Download/ICL/";
    string pose_file="ICL_pose.txt";
    ///路径/图像文件夹/图像名/图像格式
    boost::format image_path("%s/%s/%d.%s");
    ///路径/位姿文件名
    boost::format pose_path("%s/%s");

    ifstream data_in( (pose_path%file_path%pose_file).str() );
    if(!data_in)
    {
        cout<<"位姿文件路径:"<< (pose_path%file_path%pose_file).str() <<endl;
        cout<<"位姿文件读取失败!"<<endl;
        return  -1;
    }
    int num=3;

    double fx = 525.0  ;//# focal length x
    double fy = 525.0  ;//# focal length y
    double cx = 319.5  ;//# optical center x
    double cy = 239.5  ;//# optical center y
    double factor = 5000 ;//# for the 16-bit PNG files

    while( data_in.good() )
    //for(int i=0;i<num;i++)
    {

        int pose_num;
        data_in>>pose_num;
        double data[7] = {0};
        for ( auto& d:data )
            data_in>>d;

        cv::Mat color = cv::imread( ( image_path %file_path%"reference"%pose_num%"png").str() );
        cv::Mat depth = cv::imread( ( image_path %file_path%"depth"%pose_num%"png").str() );

        if(color.empty())
        {
            cout<<"编号 "<<pose_num<<" 读取彩色图失败!"<<endl;
            return -1;
        }
        if(depth.empty())
        {
            cout<<"编号 "<<pose_num<<" 读取深度图失败!!"<<endl;
            return -1;
        }

        boost::format output_path( "%s/%s/%d.%s" );
        ofstream fout( (output_path%file_path%"depth"%pose_num%"depth").str() );
        for ( int r=0; r<color.rows; r++ )
        {
            for ( int c=0; c<color.cols; c++ )
            {
                double d =(double) depth.ptr<unsigned short>(r)[c]/factor; // 深度值
                double norm_point[3];
                norm_point[0] = (c - cx) / fx;
                norm_point[1] = (r - cy) / fy;
                norm_point[2] = 1;

                double point[3];
                point[0] = norm_point[0] * d;
                point[1] = norm_point[1] * d;
                point[2] = norm_point[2] * d;

                float length = sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);

                fout << length << "\t";
            }
            fout<<"\n";
        }
    }

    return 0;
}
