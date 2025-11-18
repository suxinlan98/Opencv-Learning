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

cv::Point2f getPointAtRatio(const cv::Point2f& p1, const cv::Point2f& p2, float ratio) {
    return p1 + ratio * (p2 - p1);
}

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


    vector<cv::Point2f> imagePoints;
    
        // 绘制所有检测到的矩形
    for(int i = 0; i < contours.size(); i++) {
        cv::Rect boundRect = cv::boundingRect(contours[i]);
        cv::rectangle(result, boundRect, cv::Scalar(0,255,0), 0.5);
    }

        // 使用两个矩形的特定比例点
    if(contours.size() >= 2) {
            // 改为使用旋转矩形获取角点
        RotatedRect rotatedRect1 = cv::minAreaRect(contours[0]);
        RotatedRect rotatedRect2 = cv::minAreaRect(contours[1]);

            // 判断左右关系（x坐标小的在左边）
        RotatedRect leftRect, rightRect;
        if(rotatedRect1.center.x < rotatedRect2.center.x) {
            leftRect = rotatedRect1;
            rightRect = rotatedRect2;
        } else {
            leftRect = rotatedRect2;
            rightRect = rotatedRect1;
        }
            
            // 获取旋转矩形的四个角点
        Point2f leftCorners[4], rightCorners[4];
        leftRect.points(leftCorners);
        rightRect.points(rightCorners);

            // 计算左边矩形的四个角点
        Point2f leftTop = leftCorners[0];
        Point2f leftTopRight = leftCorners[1];
        Point2f leftBottomRight = leftCorners[2];
        Point2f leftBottom = leftCorners[3];

            // 计算右边矩形的四个角点
        Point2f rightTop = rightCorners[0];
        Point2f rightTopRight = rightCorners[1];
        Point2f rightBottomRight = rightCorners[2];
        Point2f rightBottom = rightCorners[3];

            // 计算所需的1/2比例点
            // 左边矩形上边：左上点往右上点的1/2处
        Point2f leftTopThird = getPointAtRatio(leftTop, leftTopRight, 1.0f/2);
            // 左边矩形下边：左下点往右下点的1/2处
        Point2f leftBottomThird = getPointAtRatio(leftBottom, leftBottomRight, 1.0f/2);
            // 右边矩形上边：左上点往右上点的1/2处
        Point2f rightTopThird = getPointAtRatio(rightTop, rightTopRight, 1.0f/2);
            // 右边矩形下边：左下点往右下点的1/2处
        Point2f rightBottomThird = getPointAtRatio(rightBottom, rightBottomRight, 1.0f/2);

            // 组合成4个2D点
        imagePoints = {
            leftTopThird,
            rightTopThird,
            rightBottomThird,
            leftBottomThird
        };

            // 标记所有使用的点
        vector<cv::Scalar> colors = {
            Scalar(255, 0, 0),
            Scalar(255, 0, 0),
            Scalar(255, 0, 0),
            Scalar(255, 0, 0)
        };

        for(int i = 0; i < imagePoints.size(); i++) {
            circle(result, imagePoints[i], 3, colors[i], -1);
            putText(result, std::to_string(i), imagePoints[i] + cv::Point2f(8, -8),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2);
        }

            // 连接点形成四边形
        for(int i = 0; i < imagePoints.size(); i++) {
            line(result, imagePoints[i], imagePoints[(i+1)%imagePoints.size()],
                    cv::Scalar(0, 0, 255), 1);
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
        solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        double distance = norm(tvec);//norm(tvec) 计算的是向量的欧几里得范数（2-范数），即向量的长度

        // 显示结果
        string resultText = "distances:  " + to_string(int(distance)) + " mm";
        putText(result, resultText, Point(50, 50), 
               FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        
        imwrite("/home/suxinlan/code/opencv/Resources/result.jpg", result);
        imshow("distance",result);
        imshow("mask",mask);
        waitKey(0);
        
    } else {
        cout << "未检测到装甲板" << endl;
    }
    
    return 0;
}