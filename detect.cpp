#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

//PnP（Perspective-n-Point）问题是通过一组3D空间点及其在图像上的2D投影点
//来估计相机姿态（位置和方向）的问题。
// -> 获取3d世界坐标点以及对应的2d图像坐标点，相机内参矩阵，畸变系数

/*Q：尝试获得识别到的装甲板到相机坐标系距离
  A：1、输入海康相机拍摄的装甲板图片
     2、识别装甲板4个角点（框出的区域）
     3、使用solvePNP计算距离
     4、在图片上显示*/

int main() {
    // 直接从ost.yaml文件中提取的相机参数
    Mat cameraMatrix = (Mat_<double>(3, 3) << //双精度浮点数的矩阵
        2422.61547, 0, 706.68406,
        0, 2420.80771, 564.29241,
        0, 0, 1);
    
    Mat distCoeffs = (Mat_<double>(1, 5) << 
        -0.018647, 0.084359, -0.000925, 0.000312, 0.000000);

    // 加载图片
    string imagePath = "/home/suxinlan/code/opencv/Resources/detect.bmp";
    Mat img = imread(imagePath);
    if (img.empty()) {
        cout << "错误: 无法加载图片文件" << endl;
        return -1;
    }
    
    Mat result = img.clone();

    // 检测装甲板
    Mat imgHSV, mask;
    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    
    Scalar lower(1, 174, 15);
    Scalar upper(30, 255, 255);
    
    inRange(imgHSV, lower, upper, mask);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    vector<Point> allPoints;
    for (int i = 0; i < contours.size(); i++) {
        int area = contourArea(contours[i]);
        if(area > 500) {
            allPoints.insert(allPoints.end(), contours[i].begin(), contours[i].end());
        }
    }
    
    if(!allPoints.empty()) {
        // 使用旋转矩形替代普通矩形
        RotatedRect rotatedRect = minAreaRect(allPoints);
        
        // 从旋转矩形获取角点
        Point2f vertices[4];//创建点数组
        //获取旋转矩形顶点(rotatedrect的成员函数)，但顺序不固定
        //传入数组首地址作为参数，通过指针操作填充
        rotatedRect.points(vertices);
        vector<Point2f> corners(vertices, vertices + 4);//将数组转换为vector
        
        // 绘制旋转矩形
        for (int i = 0; i < 4; i++) {
            line(result, vertices[i], vertices[(i+1)%4], Scalar(0, 255, 0), 3);
        }
        
        // 装甲板真实尺寸（毫米），定义3D模型
        double width = 135, height = 55;
        vector<Point3f> objectPoints = {
            Point3f(-width/2, -height/2, 0),
            Point3f(width/2, -height/2, 0),
            Point3f(width/2, height/2, 0),
            Point3f(-width/2, height/2, 0)
        };

        // 计算距离
        Mat rvec, tvec;
        //输入3D+2D+相机参数，输出rvec(旋转向量(3*1向量，不直观))+tvec(平移向量)
        solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
        double distance = norm(tvec);//norm(tvec) 计算的是向量的欧几里得范数（2-范数），即向量的长度

        // 显示结果
        string resultText = "distances:  " + to_string(int(distance)) + " mm";
        putText(result, resultText, Point(50, 50), 
               FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        
        imwrite("/home/suxinlan/code/opencv/Resources/result.jpg", result);
        imshow("distance",result);
        waitKey(0);
        
    } else {
        cout << "未检测到装甲板" << endl;
    }
    
    return 0;
}