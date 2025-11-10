#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//Q：将任务视频进行图像处理，识别并框出装甲板 
/*A：1、获取灯条hsv数据，转化成mask
     2、在mask上获取灯条的所有点
     3、用rectangle框出*/

Mat img;

// <<hmin,smin,vmin,hmax,smax,vmax;
//vector<vector<int>> myColor{{0,0,255,33,236,255}};
//vector<Scalar> myColorValue{{0,0,255}};

int getContours(Mat imgMask)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;//每个vec4i元素对应一个轮廓，包含四个索引，形成链表结构
    
    findContours(imgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//轮廓集合，层次信息，轮廓检索模式：只检测最外层轮廓，轮廓近似方法：压缩水平、垂直和对角线段
    
    vector<Point> allPoints;
    
    for (int i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        cout << "轮廓 " << i << " 面积: " << area << endl;
        
        if(area > 500)
        {
            // 直接使用原始轮廓点
            allPoints.insert(allPoints.end(), contours[i].begin(), contours[i].end());
        }
    }
    
    // 计算整体外接矩形并绘制在全局变量img上
    if(!allPoints.empty())
    {
        Rect overallRect = boundingRect(allPoints);//获得矩形
        rectangle(img, overallRect.tl(), overallRect.br(), Scalar(0, 255, 0), 3);//通过mask获得轮廓，在原图上绘制
        
        cout << "整体矩形面积: " << overallRect.area() << endl;
    }
    
    return 0;
}

//转化为二值图像
int findColor(Mat img)
{
        Mat imgHSV;
        cvtColor(img,imgHSV,COLOR_BGR2HSV); 
        
        Scalar lower(0,0,255);
        Scalar upper(33,236,255);
        Mat mask;
        inRange(imgHSV,lower,upper,mask);//若hsv在lower和upper范围内，转化成白色，否，转化成黑色

        getContours(mask);

        return 0;
}

int main()
{
   string path = "/home/suxinlan/code/opencv/Resources/video.mp4";//获取视频
   VideoCapture cap(path);
   
   
   while(true)
   {
       cap.read(img);
       findColor(img);

       imshow("Image",img);
       waitKey(20);    
   }
   
   return 0;
}


//矩形合并法：
// int getContours(Mat imgMask)
// {
//     vector<vector<Point>> contours;
//     vector<Vec4i> hierarchy;
    
//     findContours(imgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
//     vector<Rect> validRects;//创建矩形vector
    
//     for (int i = 0; i < contours.size(); i++)
//     {
//         int area = contourArea(contours[i]);
//         if(area > 500)
//         {
//             validRects.push_back(boundingRect(contours[i]));
//         }
//     }
    
//     // 合并所有矩形
//     if(validRects.size() >= 1)  // 即使只有一个也绘制
//     {
//         Rect overallRect = validRects[0];
//         for(int i = 1; i < validRects.size(); i++)
//         {
//             overallRect = overallRect | validRects[i];  // 矩形合并：rect1 | rect2 表示计算两个矩形的最小包围矩形
//         }
        
//         rectangle(img, overallRect.tl(), overallRect.br(), Scalar(0, 255, 0), 3);
        
//         // 显示信息
//         string info = "Rects: " + to_string(validRects.size());
//         putText(img, info, Point(overallRect.x, overallRect.y-10), 
//                 FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
//     }
    
//     return 0;
// }