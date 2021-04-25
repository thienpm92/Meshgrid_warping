#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
using namespace std;

#ifndef QUAD_H
#define QUAD_H


class Quad
{
public:
    cv::Point2f V00;
    cv::Point2f V01;
    cv::Point2f V10;
    cv::Point2f V11;

    Quad();
    Quad(const Quad &inQuad);
    Quad(const cv::Point2f &inV00, const cv::Point2f &inV01, const cv::Point2f &inV10, const cv::Point2f &inV11);
    ~Quad();

    void operator=(const Quad &inQuad);

    double getMinX() const;
    double getMaxX() const;
    double getMinY() const;
    double getMaxY() const;

    bool isPointIn(const cv::Point2f &pt) const;
    bool isPointInTriangular( const cv::Point2f &pt, const cv::Point2f &V0, const cv::Point2f &V1, const cv::Point2f &V2 ) const;
    bool getBilinearCoordinates(const cv::Point2f &pt, vector<double> &coefficient) const;
    bool getBilinearCoordinates(const cv::Point2f &pt, double* &coefficient) const;
    inline void printQuad(){
       printf("V00 = %f %f\n",V00);
       printf("V01 = %f %f\n",V01);
       printf("V10 = %f %f\n",V10);
       printf("V11 = %f %f\n",V11);
    }

    cv::Point2f getPointByBilinearCoordinates(const vector<double> &coefficient) const;
};

#endif // QUAD_H
