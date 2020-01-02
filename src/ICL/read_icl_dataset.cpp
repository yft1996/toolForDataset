#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/opencv.hpp>
int main()
{
    string file_path="/home/anshuai/Download/ICL/";
    string associate_file="associations.txt";
    string pose_file="living_room_pose.txt";
    string output_file="output.txt";

    ifstream asso_in(file_path+associate_file);
    ifstream pose_in(file_path+pose_file);
    ofstream output(file_path+output_file);
    
    if(!pose_in)
    {
        cout<<"位姿文件读取失败!"<<endl;
        return  -1;
    }
    
    if(!asso_in)
    {
        cout<<"对应文件读取失败!"<<endl;
        return  -1;
    }

    while(pose_in.good())
    {
        float pose_num;
        float pose[7];
        
        float pose_num2;
        string depth_name;
        float pose_num3;
        string rgb_name;
        
        cv::Mat depth;
        cv::Mat rgb;
        
        pose_in>>pose_num;
        for(int i=0;i<7;i++)pose_in>>pose[i];
        
        bool nofound=true;
        
        while(nofound)
        {
            asso_in>>pose_num2;
            asso_in>>depth_name;
            asso_in>>pose_num3;
            asso_in>>rgb_name;
            if(pose_num==pose_num2) nofound=false;
        }       
        
        depth=cv::imread(file_path+depth_name);
        rgb=cv::imread(file_path+rgb_name);
        
        if(depth.empty())
        {
            cout<<"编号 "<<pose_num<<" 无对应深度图!"<<endl;
            continue;
        }

        if(rgb.empty())
        {
            cout<<"编号 "<<pose_num<<" 无对应彩色图!"<<endl;
            continue;
        }
        
        output<<pose_num<<"\t";
        for(int i=0;i<7;i++)output<<pose[i]<<"\t";
        output<<pose_num2<<"\t";
        output<<depth_name<<"\t";
        output<<pose_num3<<"\t";
        output<<rgb_name<<"\n";
    }


}
