#include <iostream>
#include <fstream>
#include <boost/format.hpp>

using namespace std;

int main()
{
    string path="/home/anshuai/yft/test_data/ICL/";
    string input_posefile="ICL_pose.txt";
    string output_posefile="pose.txt";

    ifstream fin(path+input_posefile);
    ofstream fout(path+output_posefile);

    while (fin.good())
    {
        boost::format image_name("%s.%s");

        string image_num;
        fin>>image_num;
        double data[7] = {0};
        for ( auto& d:data )
            fin>>d;

        fout<< (image_name%image_num%"png").str()<<"\t";
        for(int i=0;i<7;i++)fout<<data[i]<<"\t";
        fout<<"\n";
    }
    return 0;
}

