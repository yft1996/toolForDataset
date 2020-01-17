#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>

/**
 * @func    计算估计的深度值与真实深度值之间的相对误差
 * @param depth     估计的深度图
 * @param truth_depth_path  真实的深度的文件路径
 * @return  相对误差
 */
float compute_relative_error(cv::Mat& depth,std::string truth_depth_path)
{
    //打开真实深度文件
    std::ifstream fin(truth_depth_path);
    const int row=depth.rows;
    const int col=depth.cols;

    int valid_counter=0;
    float sum_error=0.0f;
    for(int r=0;r<row;r++)
    {
        for(int c=0;c<col;c++)
        {
            float truth_depth=0.0f,est_depth=0.0f;

            fin>>truth_depth;
            est_depth=depth.at<float>(r,c);

            if(truth_depth==0)continue;
            if(est_depth<=0)continue;

            valid_counter++;

            sum_error+=fabs(truth_depth-est_depth)/truth_depth;
        }
    }
    sum_error/=valid_counter;
    return sum_error;
}

/**
 * @func    计算估计的深度的稠密度
 * @param depth   估计的深度图
 * @return      稠密程度
 */
float compute_density(cv::Mat& depth)
{
    const int row=depth.rows;
    const int col=depth.cols;

    int valid_counter=0;
    for(int r=0;r<row;r++)
    {
        for(int c=0;c<col;c++)
        {
            float est_depth=0.0f;
            est_depth=depth.at<float>(r,c);

            if(est_depth<=0)continue;

            valid_counter++;
        }
    }
    float density=(float)valid_counter/(row*col);
    return density;
}