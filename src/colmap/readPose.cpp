#include <iostream>
#include <fstream>
#include <boost/format.hpp>

using namespace std;
int main(int argc,char** argv)
{
    if(argc<4)
    {
        cout<<"输入参数不足!"<<endl;
        cout<<"./readColmapPose [文件所在路径] [输入文件名] [输出文件名]"<<endl;
        return -1;
    }

    string path=argv[1];
    string input_file_name=argv[2];
    string output_file_name=argv[3];

    ifstream fin(path+"/"+input_file_name);
    ofstream fout(path+"/"+output_file_name);

    string image_id,qw,qx,qy,qz,tx,ty,tz,camera_id,image_name;

    if(fin.bad())
    {
        cout<<"输入文件读取错误!"<<endl;
    }
    cout<<"--开始转移数据:"<<endl;
    while (fin.good())
    {
        fin>>image_id;
        fin>>qw;
        fin>>qx;
        fin>>qy;
        fin>>qz;
        fin>>tx;
        fin>>ty;
        fin>>tz;
        fin>>camera_id;
        fin>>image_name;

        ///数据格式：图像文件名 tx, ty, tz, qx, qy, qz, qw ???
        fout<<image_name<<"\t";
        fout<<tx<<"\t";
        fout<<ty<<"\t";
        fout<<tz<<"\t";
        fout<<qx<<"\t";
        fout<<qy<<"\t";
        fout<<qz<<"\t";
        fout<<qw<<"\t";
        fout<<"\n";
        cout<<"   --正在转移第"<<image_id<<"组数据...."<<endl;
    }
    cout<<"--数据转移结束"<<endl;


}
