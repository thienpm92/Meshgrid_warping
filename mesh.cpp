#include "mesh.h"
#include "quad.h"
Mesh::~Mesh()
{
}

Mesh::Mesh()
{
    imgRows = 0;
    imgCols = 0;
    meshWidth = 0;
    meshHeight = 0;
}

Mesh::Mesh( const Mesh &inMesh )
{
    imgRows = inMesh.imgRows;
    imgCols = inMesh.imgCols;
    meshWidth = inMesh.meshWidth;
    meshHeight = inMesh.meshHeight;
    xMat = cv::Mat::zeros(meshHeight, meshWidth, CV_64FC1);
    yMat = cv::Mat::zeros(meshHeight, meshWidth, CV_64FC1);
    for (int i = 0; i < meshHeight; i ++)
    {
        for (int j = 0; j < meshWidth; j ++)
        {
            setVertex(i, j, inMesh.getVertex(i,j));
        }
    }
}

Mesh::Mesh( int rows, int cols )
{
    imgRows = rows;
    imgCols = cols;
    meshWidth = 0;
    meshHeight = 0;
}

Mesh::Mesh( int rows, int cols, double quadWidth, double quadHeight )
{
    imgRows = rows;
    imgCols = cols;
    buildMesh(quadWidth, quadHeight);
    this->quadWidth = quadWidth;
    this->quadHeight = quadHeight;
}

void Mesh::buildMesh( double quadWidth, double quadHeight )
{
    vector<double> xSet;
    vector<double> ySet;

    for (double x = 0; imgCols - x > 0.5*quadWidth; x += quadWidth)
    {
        xSet.push_back(x);
    }
    xSet.push_back(imgCols-1);
    for (double y = 0; imgRows - y > 0.5*quadHeight; y += quadHeight)
    {
        ySet.push_back(y);
    }
    ySet.push_back(imgRows-1);

    meshWidth = xSet.size();
    meshHeight = ySet.size();

    xMat.create(meshHeight, meshWidth, CV_64FC1);
    yMat.create(meshHeight, meshWidth, CV_64FC1);

    for (int y = 0; y < meshHeight; y ++)
    {
        for (int x = 0; x < meshWidth; x ++)
        {
            xMat.at<double>(y,x) = xSet[x];
            yMat.at<double>(y,x) = ySet[y];
        }
    }
}


void Mesh::setVertex( int i, int j, const cv::Point2f &pos )
{
    xMat.at<double>(i,j) = pos.x;
    yMat.at<double>(i,j) = pos.y;
}

cv::Point2f Mesh::getVertex( int i, int j ) const
{
    double x;
    double y;

    x = xMat.at<double>(i,j);
    y = yMat.at<double>(i,j);

    return cv::Point2f(x,y);
}

Quad Mesh::getQuad(int i,int j) const
{
    cv::Point2f V00;
    cv::Point2f V01;
    cv::Point2f V10;
    cv::Point2f V11;

    V00 = getVertex(i-1,j-1);
    V01 = getVertex(i-1,j);
    V10 = getVertex(i,j-1);
    V11 = getVertex(i,j);

    Quad qd(V00,V01,V10,V11);

    return qd;
}

void Mesh::operator=( const Mesh &inMesh )
{
    imgRows = inMesh.imgRows;
    imgCols = inMesh.imgCols;
    meshWidth = inMesh.meshWidth;
    meshHeight = inMesh.meshHeight;
    xMat = cv::Mat::zeros(meshHeight, meshWidth, CV_64FC1);
    yMat = cv::Mat::zeros(meshHeight, meshWidth, CV_64FC1);
    for (int i = 0; i < meshHeight; i ++)
    {
        for (int j = 0; j < meshWidth; j ++)
        {
            setVertex(i, j, inMesh.getVertex(i,j));
        }
    }
}

//void Mesh::drawMesh( cv::Mat &targetImg){

//    cv::Mat temp = targetImg.clone();
//    //cv::Scalar color(0,0,0);
//    cv::Scalar color(255,255,255);
//    int gap = 0;
//    int lineWidth=3;

//    for (int i = 1; i < height; i ++)
//    {
//        for (int j = 1; j < width; j ++)
//        {
//            cv::Point2f pUp = getVertex(i-1, j);
//            cv::Point2f pLeft = getVertex(i, j-1);
//            cv::Point2f pCur = getVertex(i, j);

//            pUp.x += gap;
//            pUp.y += gap;
//            pLeft.x += gap;
//            pLeft.y += gap;
//            pCur.x += gap;
//            pCur.y += gap;

//            if(pUp.x > -9999.0 && pUp.y > -9999.0 && pCur.x > -9999.0 && pCur.y > -9999.0){
//                double dis = sqrt((pUp.x - pCur.x)*(pUp.x - pCur.x) + (pUp.y - pCur.y)*(pUp.y - pCur.y));
//                //if(dis<100){
//                    line(temp, cv::Point2f(pUp.x,pUp.y), cv::Point2f(pCur.x,pCur.y),color,lineWidth,CV_AA);
//                //}
//            }
//            if(pLeft.x > -9999.0 && pLeft.y > -9999.0 && pCur.x > -9999.0 && pCur.y > -9999.0){
//                double dis = sqrt((pLeft.x - pCur.x)*(pLeft.x - pCur.x) + (pLeft.y - pCur.y)*(pLeft.y - pCur.y));
//                //if(dis<100){
//                    line(temp, cv::Point2f(pLeft.x,pLeft.y),cv::Point2f(pCur.x,pCur.y), color,lineWidth,CV_AA);
//                //}
//            }
//            cv::circle(temp,cv::Point(pUp.x,pUp.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//            cv::circle(temp,cv::Point(pLeft.x,pLeft.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//            cv::circle(temp,cv::Point(pCur.x,pCur.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//        }
//    }

//    for (int i = 1; i < height; i ++)
//    {
//        cv::Point2f pLeft = getVertex(i, 0);
//        cv::Point2f pLeftUp = getVertex(i-1,0);

//        pLeftUp.x += gap;
//        pLeftUp.y += gap;
//        pLeft.x += gap;
//        pLeft.y += gap;

//        if (pLeft.x > -9999.0 && pLeft.y > -9999.0 && pLeftUp.x > -9999.0 && pLeftUp.y > -9999.0){
//            double dis = sqrt((pLeft.x - pLeftUp.x)*(pLeft.x - pLeftUp.x) + (pLeft.y - pLeftUp.y)*(pLeft.y - pLeftUp.y));
//            //if(dis<100){
//                line(temp, cv::Point2f(pLeft.x,pLeft.y), cv::Point2f(pLeftUp.x,pLeftUp.y),color,lineWidth,CV_AA);
//            //}
//        }
//        cv::circle(temp,cv::Point(pLeftUp.x,pLeftUp.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//        cv::circle(temp,cv::Point(pLeft.x,pLeft.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//    }

//    for (int j = 1; j < width; j++)
//    {
//        cv::Point2f pLeftUp = getVertex(0, j-1);
//        cv::Point2f pUp = getVertex(0, j);

//        pLeftUp.x += gap;
//        pLeftUp.y += gap;
//        pUp.x += gap;
//        pUp.y += gap;

//        if (pLeftUp.x > -9999.0 && pLeftUp.y > -9999.0 && pUp.x > -9999.0 && pUp.y > -9999.0){
//            double dis = sqrt((pLeftUp.x - pUp.x)*(pLeftUp.x - pUp.x) + (pLeftUp.y - pUp.y)*(pLeftUp.y - pUp.y));
//            //if(dis<100){
//                line(temp, cv::Point2f(pLeftUp.x,pLeftUp.y), cv::Point2f(pUp.x,pUp.y),color,lineWidth,CV_AA);
//            //}
//        }
//        cv::circle(temp,cv::Point(pUp.x,pUp.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//        cv::circle(temp,cv::Point(pLeftUp.x,pLeftUp.y),lineWidth+2,cv::Scalar(45,57,167),-1);
//    }
//    targetImg = (2.0/5 * targetImg + 3.0/5 *temp);
//}



bool isPointInTriangular( const cv::Point2f &pt, const cv::Point2f &V0, const cv::Point2f &V1, const cv::Point2f &V2 )
{
    double lambda1 = ((V1.y-V2.y)*(pt.x-V2.x) + (V2.x-V1.x)*(pt.y-V2.y)) / ((V1.y-V2.y)*(V0.x-V2.x) + (V2.x-V1.x)*(V0.y-V2.y));
    double lambda2 = ((V2.y-V0.y)*(pt.x-V2.x) + (V0.x-V2.x)*(pt.y-V2.y)) / ((V2.y-V0.y)*(V1.x-V2.x) + (V0.x-V2.x)*(V1.y-V2.y));
    double lambda3 = 1-lambda1-lambda2;
    if (lambda1 >= 0.0 && lambda1 <= 1.0 && lambda2 >= 0.0 && lambda2 <= 1.0 && lambda3 >= 0.0 && lambda3 <= 1.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

cv::Point2f matMyPointCVMat(const cv::Point2f &pt,const cv::Mat &H){
    cv::Mat cvPt = cv::Mat::zeros(3,1,CV_64F);
    cvPt.at<double>(0,0) = pt.x;
    cvPt.at<double>(1,0) = pt.y;
    cvPt.at<double>(2,0) = 1.0;

    cv::Mat cvResult = H*cvPt;

    cv::Point2f result;
    result.x = cvResult.at<double>(0,0)/cvResult.at<double>(2,0);
    result.y = cvResult.at<double>(1,0)/cvResult.at<double>(2,0);

    return result;
}







void quadWarp( const cv::Mat src, cv::Mat dst, const Quad &q1, const Quad &q2){
    int gap=0;

    int minx = max(0, (int)q2.getMinX());
    int maxx = min(dst.cols-1, (int)q2.getMaxX());
    int miny = max(0, (int)q2.getMinY());
    int maxy = min(dst.rows-1, (int)q2.getMaxY());

    /*
    int minx = (int)q2.getMinX();
    int maxx = (int)q2.getMaxX();
    int miny = (int)q2.getMinY();
    int maxy = (int)q2.getMaxY();
    */

    for (int i = miny; i <= maxy; i ++)
    {
        for (int j = minx; j <= maxx; j ++)
        {
            if (q2.isPointIn(cv::Point2f(j,i)))
            {
                vector<double> coe;
                bool flag = q2.getBilinearCoordinates(cv::Point2f(j, i), coe);
                if (flag)
                {
                    cv::Point2f ptInSrc = q1.getPointByBilinearCoordinates(coe);

                    cv::Point2f tmpV00(floor(ptInSrc.x), floor(ptInSrc.y));
                    cv::Point2f tmpV01(tmpV00.x+1, tmpV00.y);
                    cv::Point2f tmpV10(tmpV00.x, tmpV00.y+1);
                    cv::Point2f tmpV11(tmpV00.x+1, tmpV00.y+1);

                    Quad tmpQ(tmpV00, tmpV01, tmpV10, tmpV11);
                    vector<double> tmpCoe;
                    tmpQ.getBilinearCoordinates(ptInSrc, tmpCoe);

                    if (q1.isPointIn(ptInSrc))
                    {
                        for (int c = 0; c < 3; c ++)
                        {
                            bool f0 = q1.isPointIn(tmpV00);
                            bool f1 = q1.isPointIn(tmpV01);
                            bool f2 = q1.isPointIn(tmpV10);
                            bool f3 = q1.isPointIn(tmpV11);

                            dst.at<cv::Vec3b>(i+gap,j+gap)[c] = (tmpCoe[0]*f0*src.at<cv::Vec3b>(int(tmpV00.y), int(tmpV00.x))[c] +
                                                     tmpCoe[1]*f1*src.at<cv::Vec3b>(int(tmpV01.y), int(tmpV01.x))[c] +
                                                     tmpCoe[2]*f2*src.at<cv::Vec3b>(int(tmpV10.y), int(tmpV10.x))[c] +
                                                     tmpCoe[3]*f3*src.at<cv::Vec3b>(int(tmpV11.y), int(tmpV11.x))[c]) /
                                                    (tmpCoe[0]*f0 + tmpCoe[1]*f1 + tmpCoe[2]*f2 + tmpCoe[3]*f3);
                        }
                    }
                }
            }
        }
    }
}



void myQuickSort(vector<float> &arr, int left, int right){
      int i = left, j = right;
      float tmp;
      float pivot = arr[(left + right) / 2];

      while(i<=j){
          while(arr[i]<pivot)
              i++;
          while(arr[j]>pivot)
              j--;
          if(i<=j){
              tmp = arr[i];
              arr[i] = arr[j];
              arr[j] = tmp;
              i++;
              j--;
          }
      }
      if(left<j)myQuickSort(arr,left,j);
      if(i<right)myQuickSort(arr,i,right);
}
