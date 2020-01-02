#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

int main(int argc,char** argv) {
    std::string depthmap_path = "/home/anshuai/yft/TUM_datasets/rgbd_dataset_freiburg3_structure_texture_far/TUM/depth/TUM_0010.png";
    std::string output_path = "/home/anshuai/yft/TUM_datasets/rgbd_dataset_freiburg3_structure_texture_far/TUM/depth/TUM_0001.zepth";
    cv::Mat depthmap = cv::imread(depthmap_path, cv::IMREAD_ANYDEPTH);

    double fx = 535.4;
    double fy = 539.2;
    double cx = 320.1;
    double cy = 247.6;
    double depthScale = 5000;

    std::ofstream fout(output_path);

    for (int r = 0; r < depthmap.rows; r++)
    {       for (int c = 0; c < depthmap.cols; c++) {
            float depth = depthmap.at<short>(r, c) / depthScale;

            fout << depth << "\t";
        }
        fout<<"\n";
    }


    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("depth",640,480);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);
    std::ofstream fout1(output_path);
    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(depthmap.cols,depthmap.rows,fx,fy,cx,cy,0.2,100),
            pangolin::ModelViewLookAt(-2,-2,-2, 0.5,0,0, pangolin::AxisY)
    );

    //创建视角窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        //清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        pangolin::glDrawAxis(3);

        //点的创建
        glPointSize(1.0f);
        glBegin(GL_POINTS);


        //for (int r=0;r<_depth.rows;r++)
        int rows=depthmap.rows;
        int cols=depthmap.cols;
        for(int r=0;r<rows;r++)
        {
            for (int c=0;c<cols;c++)
                //for(int c=0;c<_depth.cols;c++)
            {

                float depth=depthmap.at<short>(r,c)/depthScale;
                float point[3];
                point[0]=(c-cx)*depth/fx;
                point[1]=(r-cy)*depth/fy;
                point[2]=depth;

                switch ((int)depth)
                {
                    case 0:
                        glColor3f(1,100,200);
                        break;
                    case 1:
                        glColor3f(50,150,250);
                        break;
                    case 2:
                        glColor3f(100,200,1);
                        break;
                    case 3:
                        glColor3f(150,250,50);
                        break;
                    case 4:
                        glColor3f(200,1,100);
                        break;
                    default:
                        glColor3f(250,50,150);
                        break;
                }

                glVertex3f(point[0],point[1],point[2]);
            }
        }

        glEnd();
        //交换帧和并推进事件
        pangolin::FinishFrame();
    }

}