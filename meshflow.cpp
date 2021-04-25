#include "meshflow.h"

MeshFlow::MeshFlow(int width, int height, double quadWidth, double quadHeight, double weight){

    m_imgHeight  = height;
    m_imgWidth   = width;
    m_quadWidth  = quadWidth;
    m_quadHeight = quadHeight;


//    m_mesh       = new Mesh(height,width,quadWidth,quadHeight);
//    m_warpedmesh = new Mesh(height,width,quadWidth,quadHeight);
    Mesh initMesh(height,width,quadWidth,quadHeight);
    m_mesh = initMesh;
    m_warpedmesh = initMesh;
    m_meshheight = m_mesh.meshHeight;
    m_meshwidth  = m_mesh.meshWidth;
    m_vertexMotion.resize(m_meshheight*m_meshwidth);


//    x_index.resize(m_meshheight*m_meshwidth);
//    y_index.resize(m_meshheight*m_meshwidth);
    for(int i=0;i<m_meshheight*m_meshwidth;i++){
        x_index.push_back(i);
        y_index.push_back(m_meshheight*m_meshwidth+i);
    }
    columns = m_meshheight*m_meshwidth*2;
    num_smooth_cons = (m_meshheight-2)*(m_meshwidth-2)*16 + ((m_meshwidth+m_meshheight)*2-8)*8 + 16;
    SmoothConstraints = cv::Mat::zeros(num_smooth_cons*5, 3, CV_64FC1);     //size calculate base on number of weight vertice in the code
    CreateSmoothCons(weight);
}

void MeshFlow::SetControlPts(vector<cv::Point2f> &prevPts, vector<cv::Point2f> &curPts){
    int len = prevPts.size();
    dataterm_element_i.resize(len,0);
    dataterm_element_j.resize(len,0);
    dataterm_element_V00.resize(len,0);
    dataterm_element_V01.resize(len,0);
    dataterm_element_V10.resize(len,0);
    dataterm_element_V11.resize(len,0);
    num_feature_pts = prevPts.size();

    dataterm_element_orgPt = prevPts;
    dataterm_element_desPt = curPts;

    for(int i=0;i<len;i++){
        cv::Point2f pt = prevPts[i];
        dataterm_element_i[i] = floor(pt.y/m_quadHeight)+ 1;                                            //????????
        dataterm_element_j[i] = floor(pt.x/m_quadWidth) + 1;
        Quad quad = m_mesh.getQuad(dataterm_element_i[i],dataterm_element_j[i]);
        vector<double> coefficient;
        quad.getBilinearCoordinates(pt, coefficient );
        dataterm_element_V00[i] = coefficient[0];
        dataterm_element_V01[i] = coefficient[1];
        dataterm_element_V10[i] = coefficient[2];
        dataterm_element_V11[i] = coefficient[3];
    }
}

void MeshFlow::Solve(){
    vector<double> b_temp = CreateDataCons();
    int N = SmoothConstraints.rows + DataConstraints.rows;
    int cc = 0;
ofstream Check("check_error.txt");


//    for(int i=0;i<SmoothConstraints.rows;i++){
//        locations(0,i) = (unsigned int)SmoothConstraints.at<double>(i,0);
//        locations(1,i) = (unsigned int)SmoothConstraints.at<double>(i,1);
//        values(i) = (unsigned int)SmoothConstraints.at<double>(i,2);
//Check<<i<<" "<<SmoothConstraints.at<double>(i,0)<<"-"<<SmoothConstraints.at<double>(i,1)<<" "<<SmoothConstraints.at<double>(i,2)<<endl;
//        cc++;
//    }
//    for(int i=0;i<DataConstraints.rows;i++){
//        locations(0,i+cc) = (unsigned int)DataConstraints.at<double>(i,0);
//        locations(1,i+cc) = (unsigned int)DataConstraints.at<double>(i,1);
//        values(i+cc) = (unsigned int)DataConstraints.at<double>(i,2);
//    }

    cv::Mat DataMat;
    DataMat= cv::Mat::zeros(N, N, CV_64FC1);
    for(int i=0;i<SmoothConstraints.rows;i++){
        int y = SmoothConstraints.at<double>(i,0)+1;
        int x = SmoothConstraints.at<double>(i,1);
        DataMat.at<double>(x,y) += SmoothConstraints.at<double>(i,2);
//Check<<i<<" "<<SmoothConstraints.at<double>(i,0)<<" "<<SmoothConstraints.at<double>(i,1)<<" "<<SmoothConstraints.at<double>(i,2)<<endl;
    }
    for(int i=0;i<DataConstraints.rows;i++){
        int y = DataConstraints.at<double>(i,0)+1;
        int x = DataConstraints.at<double>(i,1);
        DataMat.at<double>(x,y) += DataConstraints.at<double>(i,2);
Check<<i<<" "<<DataConstraints.at<double>(i,0)<<" "<<DataConstraints.at<double>(i,1)<<" "<<DataConstraints.at<double>(i,2)<<endl;
    }
    vector<cv::Point2f> LocationVec;
    vector<double> ValueVec;
    for(int i=0;i<DataMat.rows;i++){
        for(int j=0;j<DataMat.cols;j++){
            double val = DataMat.at<double>(i,j);
            if(val!=0){
                cv::Point2f pt(j,i);
                LocationVec.push_back(pt);
                ValueVec.push_back(val);
//Check<<cc<<" "<<j<<" "<<i<<" "<<val<<endl;
cc++;
            }
        }
    }

    arma::umat locations(2, ValueVec.size());
    arma::vec values(ValueVec);
    for(int i=0;i<ValueVec.size();i++){
        locations(1,i) = LocationVec[i].y;
        locations(0,i) = LocationVec[i].x;
        values(i) = ValueVec[i];
    }

    int a = b_temp.size();
    arma::mat B(a,1);
    arma::sp_mat A(locations,values);
    for(int i=0;i<b_temp.size();i++){
        B(i,0) = b_temp[i];
    }

    arma::mat A1 = arma::zeros(a, m_meshheight*m_meshwidth*2);
    for(int i=0;i<A1.n_rows;i++){
        for(int j=0;j<A1.n_cols;j++){
            A1(i,j) = A(i,j);
        }
    }
    arma::vec X = solve(A1,B);



//    arma::vec X = spsolve(A,B);

    int halfcolumns = columns/2;
    for(int i=0;i<m_meshheight;i++){
        for(int j=0;j<m_meshwidth;j++){
            cv::Point2f pt(X(i*m_meshwidth+j),X(halfcolumns+i*m_meshwidth+j));
            m_warpedmesh.setVertex(i,j,pt);
        }
    }
}

void MeshFlow::CalcHomos(vector<cv::Mat> &homos){
    homos.resize(m_meshwidth*m_meshheight);
    vector<cv::Point2f> source(4);
    vector<cv::Point2f> target(4);

    for(int i=1;i<m_meshheight;i++){
        for(int j=1;j<m_meshwidth;j++){
            Quad s = m_mesh.getQuad(i, j);
            Quad t = m_warpedmesh.getQuad(i, j);

            source[0] = s.V00;
            source[1] = s.V01;
            source[2] = s.V10;
            source[3] = s.V11;

            target[0] = t.V00;
            target[1] = t.V01;
            target[2] = t.V10;
            target[3] = t.V11;

            cv::Mat H = cv::findHomography(source, target, 0);
            homos.push_back(H);
        }
    }
}

cv::Mat MeshFlow::Warping(const cv::Mat srcImg, const vector<cv::Mat> homos){
    cv::Mat warpImg;
    warpImg = cv::Mat::zeros( srcImg.size(), CV_8UC3 );
    for (int i = 1; i < m_meshheight; i ++)	{
        for (int j = 1; j < m_meshwidth; j ++)		{
            cv::Point2f p0 = m_mesh.getVertex(i-1, j-1);
            cv::Point2f p1 = m_mesh.getVertex(i-1, j);
            cv::Point2f p2 = m_mesh.getVertex(i, j-1);
            cv::Point2f p3 = m_mesh.getVertex(i,j);

            cv::Point2f q0 = m_warpedmesh.getVertex(i-1, j-1);
            cv::Point2f q1 = m_warpedmesh.getVertex(i-1, j);
            cv::Point2f q2 = m_warpedmesh.getVertex(i, j-1);
            cv::Point2f q3 = m_warpedmesh.getVertex(i, j);

            Quad quad1(p0, p1, p2, p3);
            Quad quad2(q0, q1, q2, q3);

            quadWarp(srcImg, warpImg,homos[m_meshwidth*i+j], quad1, quad2);
        }
    }
    return warpImg;
}

void MeshFlow::quadWarp( const cv::Mat src, cv::Mat dst, const Quad &q1, const Quad &q2){
    int minx = max(0, (int)q2.getMinX());
    int maxx = min(dst.cols-1, (int)q2.getMaxX());
    int miny = max(0, (int)q2.getMinY());
    int maxy = min(dst.rows-1, (int)q2.getMaxY());

    vector<cv::Point2f> source(4);
    vector<cv::Point2f> target(4);
    source[0] = q1.V00;
    source[1] = q1.V01;
    source[2] = q1.V10;
    source[3] = q1.V11;

    target[0] = q2.V00;
    target[1] = q2.V01;
    target[2] = q2.V10;
    target[3] = q2.V11;
    cv::Mat H = cv::findHomography(source, target, 0);
    for (int i = miny; i < maxy; i ++) {
        for (int j = minx; j < maxx; j ++) {
            double X = H.at<double>(0, 0) * j + H.at<double>(0, 1) * i + H.at<double>(0, 2);
            double Y = H.at<double>(1, 0) * j + H.at<double>(1, 1) * i + H.at<double>(1, 2);
            double W = H.at<double>(2, 0) * j + H.at<double>(2, 1) * i + H.at<double>(2, 2);
            W = W ? 1.0 / W : 0;
            X = (int)X*W;
            Y = (int)Y*W;
            cv::Vec3b color = src.at<cv::Vec3b>(cv::Point(j,i));
            if(X>=0 && X<src.cols && Y>=0 && Y< src.rows)
            {
                dst.at<cv::Vec3b>(cv::Point(X,Y)) = color;
            }
        }
    }

}

vector<double> MeshFlow::CreateDataCons(){
    int len = dataterm_element_i.size();        //number of feature points
    num_data_cons = len*2;
    vector<double> b;
    b.resize( num_data_cons + num_smooth_cons,0);
    DataConstraints = cv::Mat::zeros(len*8, 3, CV_64FC1);       //size base on measure

    rowCount--;
    for(int k=0;k<len;k++){
        int i = dataterm_element_i[k];
        int j = dataterm_element_j[k];
        double v00 = dataterm_element_V00[k];
        double v01 = dataterm_element_V01[k];
        double v10 = dataterm_element_V10[k];
        double v11 = dataterm_element_V11[k];

        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = x_index[(i-1)*m_meshwidth+j-1];           //????????
        DataConstraints.at<double>(DCc,2) = v00;
double tem1, tem2;
tem1 = x_index[(i-1)*m_meshwidth+j-1+1];
tem2 = v00;
        DCc++;
        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = x_index[(i-1)*m_meshwidth+j];
        DataConstraints.at<double>(DCc,2) = v01;
tem1 = x_index[(i-1)*m_meshwidth+j-1+1];
tem2 = v01;
        DCc++;
        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = x_index[i*m_meshwidth+j-1];
        DataConstraints.at<double>(DCc,2) = v10;
tem1 = x_index[i*m_meshwidth+j-1+1];
tem2 = v10;
        DCc++;
        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = x_index[i*m_meshwidth+j];
        DataConstraints.at<double>(DCc,2) = v11;
tem1 = x_index[i*m_meshwidth+j+1];
tem2 = v11;
        DCc++;
        rowCount++;

        b[rowCount] = dataterm_element_desPt[k].x;

        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = y_index[(i-1)*m_meshwidth+j-1];
        DataConstraints.at<double>(DCc,2) = v00;
        DCc++;
        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = y_index[(i-1)*m_meshwidth+j];
        DataConstraints.at<double>(DCc,2) = v01;
        DCc++;
        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = y_index[i*m_meshwidth+j-1];
        DataConstraints.at<double>(DCc,2) = v10;
        DCc++;
        DataConstraints.at<double>(DCc,0) = rowCount;
        DataConstraints.at<double>(DCc,1) = y_index[i*m_meshwidth+j];
        DataConstraints.at<double>(DCc,2) = v11;
        DCc++;
        rowCount++;

        b[rowCount] = dataterm_element_desPt[k].y;
    }
    return b;
}

void MeshFlow::CreateSmoothCons(double weight){
    rowCount = 0;
    int i=0;
    int j=0;
    addCoefficient_5(i,j,weight);
    addCoefficient_6(i,j,weight);

    i=0;
    j = m_meshwidth-1;
    addCoefficient_7(i,j,weight);
    addCoefficient_8(i,j,weight);

    i=m_meshheight-1;
    j=0;
    addCoefficient_3(i,j,weight);
    addCoefficient_4(i,j,weight);

    i=m_meshheight-1;
    j=m_meshwidth-1;
    addCoefficient_1(i,j,weight);
    addCoefficient_2(i,j,weight);

    i=0;
    for(j=1;j<m_meshwidth-1;j++){
        addCoefficient_5(i,j,weight);
        addCoefficient_6(i,j,weight);
        addCoefficient_7(i,j,weight);
        addCoefficient_8(i,j,weight);
    }

    i=m_meshheight-1;
    for(j=1;j<m_meshwidth-1;j++){
        addCoefficient_1(i,j,weight);
        addCoefficient_2(i,j,weight);
        addCoefficient_3(i,j,weight);
        addCoefficient_4(i,j,weight);
    }

    j=0;
    for(i=1;i<m_meshheight-1;i++){
        addCoefficient_3(i,j,weight);
        addCoefficient_4(i,j,weight);
        addCoefficient_5(i,j,weight);
        addCoefficient_6(i,j,weight);
    }

    j=m_meshwidth-1;
    for(i=1;i<m_meshheight-1;i++){
        addCoefficient_1(i,j,weight);
        addCoefficient_2(i,j,weight);
        addCoefficient_7(i,j,weight);
        addCoefficient_8(i,j,weight);
    }

    for(i=1;i<m_meshheight-1;i++){
        for(j=1;j<m_meshwidth-1;j++){
            addCoefficient_1(i,j,weight);
            addCoefficient_2(i,j,weight);
            addCoefficient_3(i,j,weight);
            addCoefficient_4(i,j,weight);
            addCoefficient_5(i,j,weight);
            addCoefficient_6(i,j,weight);
            addCoefficient_7(i,j,weight);
            addCoefficient_8(i,j,weight);
        }
    }
}

void MeshFlow::getSmoothWeight(vector<double> &smoothWeight, const cv::Point2f &V1, const cv::Point2f &V2, const cv::Point2f &V3){
    double d1 = sqrt((V1.x - V2.x)*(V1.x - V2.x) + (V1.y - V2.y)*(V1.y - V2.y));
    double d3 = sqrt((V2.x - V3.x)*(V2.x - V3.x) + (V2.y - V3.y)*(V2.y - V3.y));

    cv::Point2f V21(V1.x-V2.x, V1.y-V2.y);
    cv::Point2f V23(V3.x-V2.x, V3.y-V2.y);

    double cosin = V21.x*V23.x + V21.y*V23.y;
    cosin = cosin/(d1*d3);
    double u_dis = cosin*d1;
    double u = u_dis/d3;

    double v_dis = sqrt(d1*d1 - u_dis*u_dis);
    double v = v_dis/d3;
    smoothWeight[0] = u;
    smoothWeight[1] = v;
}






void MeshFlow::addCoefficient_1(int i, int j, double weight){
    //  V3(i-1,j-1)
    //  |
    //  |
    //  |_________V1(i,j)
    //  V2
    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i,j-1);
    cv::Point2f V3 = m_mesh.getVertex(i-1,j-1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*i + j - 1;
    int coordv3 = m_meshwidth*(i-1) + j - 1;

//    coordv1 = coordv1 + 1;                                                              //????????
//    coordv2 = coordv2 + 1;
//    coordv3 = coordv3 + 1;


    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    double tem1, tem2;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_2(int i, int j, double weight){
    //  V3     V2
    //  _______
    //          |
    //          |
    //          |
    //          V1

    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i-1,j);
    cv::Point2f V3 = m_mesh.getVertex(i-1,j-1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*(i-1) + j;
    int coordv3 = m_meshwidth*(i-1) + j - 1;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    double tem1, tem2;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_3(int i, int j, double weight){
    //  V2     V3
    //  _______
    //  |
    //  |
    //  |
    //  V1(i,j)
    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i-1,j);
    cv::Point2f V3 = m_mesh.getVertex(i-1,j+1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*(i-1) + j;
    int coordv3 = m_meshwidth*(i-1) + j + 1;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    double tem1, tem2;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_4(int i, int j, double weight){
    //          V3 (i-1,j+1)
    //          |
    //          |
    // V1_______|V2

    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i,j+1);
    cv::Point2f V3 = m_mesh.getVertex(i-1,j+1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*i + j + 1;
    int coordv3 = m_meshwidth*(i-1) + j + 1;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    double tem1, tem2;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_5(int i, int j, double weight){
    //  V1     V2
    //  _______
    //          |
    //          |
    //          |
    //          V3
    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i,j+1);
    cv::Point2f V3 = m_mesh.getVertex(i+1,j+1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*i + j + 1;
    int coordv3 = m_meshwidth*(i+1) + j + 1;

    double tem1, tem2;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_6(int i, int j, double weight){
    //  V1
    //  |
    //  |
    //  |_________V3 (i+1,j+1)
    //  V2(i+1,j)
    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i+1,j);
    cv::Point2f V3 = m_mesh.getVertex(i+1,j+1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*(i+1) + j ;
    int coordv3 = m_meshwidth*(i+1) + j + 1;

    double tem1, tem2;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_7(int i, int j, double weight){
    //          V1 (i,j)
    //          |
    //          |
    //  ________|V2
    //  V3(i+1,j-1)
    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i+1,j);
    cv::Point2f V3 = m_mesh.getVertex(i+1,j-1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*(i+1) + j;
    int coordv3 = m_meshwidth*(i+1) + j - 1;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    double tem1, tem2;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}

void MeshFlow::addCoefficient_8(int i, int j, double weight){
    //  V2     V1(i,j)
    //  _______
    //  |
    //  |
    //  |
    //  V3(i+1,j-1)
    vector<double> smoothWeight;
    smoothWeight.resize(2);

    cv::Point2f V1 = m_mesh.getVertex(i,j);
    cv::Point2f V2 = m_mesh.getVertex(i,j-1);
    cv::Point2f V3 = m_mesh.getVertex(i+1,j-1);

    getSmoothWeight(smoothWeight,V1,V2,V3);
    double u = smoothWeight[0];
    double v = smoothWeight[1];
    int coordv1 = m_meshwidth*i + j;
    int coordv2 = m_meshwidth*i + j - 1;
    int coordv3 = m_meshwidth*(i+1) + j - 1;

    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    double tem1, tem2;
    tem1 = x_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = x_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = y_index[coordv3];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = y_index[coordv2];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = x_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
    //////////
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = (1.0-u)*weight;
    tem1 = y_index[coordv2];
    tem2 = (1.0-u)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = u*weight;
    tem1 = y_index[coordv3];
    tem2 = u*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv2];
    SmoothConstraints.at<double>(Scc,2) = v*weight;
    tem1 = x_index[coordv2];
    tem2 = v*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = x_index[coordv3];
    SmoothConstraints.at<double>(Scc,2) = (-1.0*v)*weight;
    tem1 = x_index[coordv3];
    tem2 = (-1.0*v)*weight;
    Scc++;
    SmoothConstraints.at<double>(Scc,0) = rowCount;
    SmoothConstraints.at<double>(Scc,1) = y_index[coordv1];
    SmoothConstraints.at<double>(Scc,2) = (-1.0)*weight;
    tem1 = y_index[coordv1];
    tem2 = (-1.0)*weight;
    Scc++;
    rowCount++;
}
