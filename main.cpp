//#include <QCoreApplication>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/tracking.hpp>
#include "iostream"
#include "meshflow.h"

using namespace std;

int main()
{
    string link = "/media/eric/DATA/SSU research/Data set/video_test_case/regular_girl_2.avi";
    cv::Mat src = cv::imread("/home/eric/qt_project/test_warping/s.png");
    cv::Mat dst = cv::imread("/home/eric/qt_project/test_warping/t.png");

    const int max_corners = 335;
    const double quality_level = 0.05;
    double minDistance = 5;
    vector<cv::Point2f> pp1, pp2, prev_corner, cur_corner;
    vector<uchar> status;
    vector<float> err;
    cv::Mat srcGray, dstGray;

    cv::cvtColor(src,srcGray,CV_BGR2GRAY);
    cv::cvtColor(dst,dstGray,CV_BGR2GRAY);
    cv::goodFeaturesToTrack(srcGray, prev_corner, max_corners, quality_level, minDistance);
    cv::calcOpticalFlowPyrLK(srcGray, dstGray , prev_corner, cur_corner, status, err);
    for (int i = 0; i < status.size(); i++) {
        if (status[i]) {
            pp1.push_back(prev_corner[i]);
            pp2.push_back(cur_corner[i]);
        }
    }

//    int size =275;
//    pp1.resize(size);
//    pp2.resize(size);
//    ifstream LoadX;
//    LoadX.open("loadX.txt");
//    if (LoadX.is_open()) {
//        for(int i=0;i<size;i++) {
//        LoadX >> pp1.push_back(cv::Point2f(a;
//        cout<<output;

//     }
//    }
//    myReadFile.close();


    vector<cv::Mat>homos;
    double weight =1;
    int height = src.rows;
    int width  = src.cols;
    double quadWidth = width/16.0;
    double quadHeight = height/16.0;
    MeshFlow WarpingMethod(width, height, quadWidth, quadHeight, weight);
    WarpingMethod.SetControlPts(pp1,pp2);
    WarpingMethod.Solve();
    WarpingMethod.CalcHomos(homos);
    cv::Mat wrpImg = WarpingMethod.Warping(src,homos);

    cv::namedWindow( "Output",CV_WINDOW_AUTOSIZE);
    cv::moveWindow("Output", 800, 800);
    cv::imshow("Output", wrpImg);
    cv::imwrite("Output.jpg", wrpImg);

    cv::namedWindow( "Input",CV_WINDOW_AUTOSIZE);
    cv::moveWindow("Input", 400, 400);
    cv::imshow("Input", src);


//    cv::Point2f pt(247.9435, 162.0560);
//    Mesh m_mesh(height,width,quadWidth,quadHeight);
//    int  dataterm_element_i = floor(pt.y/quadHeight)+ 1;                                            //????????
//    int  dataterm_element_j = floor(pt.x/quadWidth) + 1;
//    Quad quad = m_mesh.getQuad(dataterm_element_i,dataterm_element_j);
//    vector<double> coefficient;
//    quad.getBilinearCoordinates(pt, coefficient );
//    double dataterm_element_V00 = coefficient[0];
//    double dataterm_element_V01 = coefficient[1];
//    double dataterm_element_V10 = coefficient[2];
//    double dataterm_element_V11 = coefficient[3];
//    double m_meshheight = m_mesh.meshHeight;
//    double m_meshwidth  = m_mesh.meshWidth;
//    vector<int> x_index;
//    vector<int> y_index;
//    for(int i=0;i<m_meshheight*m_meshwidth;i++){
//        x_index.push_back(i);
//        y_index.push_back(m_meshheight*m_meshwidth+i);
//    }

//    double tem1, tem2;
//    int i = dataterm_element_i;
//    int j = dataterm_element_j;
//    tem1 = y_index[(i-1)*m_meshwidth+j-1+1];
//    tem2 = dataterm_element_V00;

//    tem1 = y_index[(i-1)*m_meshwidth+j+1];
//    tem2 = dataterm_element_V01;

//    tem1 = y_index[i*m_meshwidth+j-1+1];
//    tem2 = dataterm_element_V10;

//    tem1 = y_index[i*m_meshwidth+j+1];
//    tem2 = dataterm_element_V11;

    cv::waitKey(0);
    return 0;
}
