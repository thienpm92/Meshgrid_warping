#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include "quad.h"

using namespace std;

#ifndef MESH_H
#define MESH_H

class Mesh
{
private:
    cv::Mat xMat;
    cv::Mat yMat;

public:
    int imgRows;
    int imgCols;

    int meshWidth;
    int meshHeight;

    int quadWidth;
    int quadHeight;

    Mesh();
    Mesh(const Mesh &inMesh);
    Mesh(int rows, int cols);
    Mesh(int rows, int cols, double quadWidth, double quadHeight);
    ~Mesh();
    void operator=(const Mesh &inMesh);
    void buildMesh(double quadWidth, double quadHeight);
    void setVertex(int i, int j, const cv::Point2f &pos);
    cv::Point2f getVertex(int i, int j) const;
    Quad getQuad(int i,int j) const;
};

#endif // MESH_H

void myQuickSort(vector<float> &arr, int left, int right);
