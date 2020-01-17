#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>

void compute_relative_error(cv::Mat& depth,std::string truth_depth_path)
{
    //打开真实深度文件
    std::ifstream fin(truth_depth_path);
    const int row=depth.rows;
    const int col=depth.cols;

    for(int r=0;r<row;r++)
    {
        for(int c=0;c<col;c++)
        {

        }
    }
}