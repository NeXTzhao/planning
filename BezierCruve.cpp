#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using std::cout;
using std::endl;
using std::vector;

vector<Point2f> bezierCurve(vector<Point2f> src);
int main(int argc, char const *argv[])
{
    while (1)
    {
        vector<Point2f> path;
        CvRNG rng;
        rng = cvRNG(cvGetTickCount());
        for (int i = 1; i < 6; i++)
            path.push_back(Point2f(i * 800 / 6.0, cvRandInt(&rng) % 800));

        Mat img(800, 800, CV_8UC3);
        // img = 0;
        for (int i = 0; i < path.size(); i++)
            circle(img, path[i], 3, Scalar(0, 0, 255), 3); //BGR
        vector<Point2f> bezierPath = bezierCurve(path);
        for (int i = 0; i < bezierPath.size(); i++)
        {
            //circle(img, bezierPath[i], 3, Scalar(0, 255, 255), 3); //BGR
            img.at<cv::Vec3b>(cvRound(bezierPath[i].y), cvRound(bezierPath[i].x)) = {0, 255, 255};
        }
        imshow("black", img);
        if (waitKey(0) == 'q')
            break;
    }
    return 0;
}
vector<Point2f> bezierCurve(vector<Point2f> src)
{
    if (src.size() < 1) //这种情况是不允许出现的，出现只能证明程序出错了
        return src;
    const float step = 0.01; //采集100个点，即1.0/step
    vector<Point2f> res;
    if (src.size() == 1)
    { //递归结束条件，k=0
        for (float t = 0; t < 1; t += step)
            res.push_back(src[0]); //为了和其他情况保持一致，生成了1.0/step个一样的点
        return res;
    }
    vector<Point2f> src1;
    vector<Point2f> src2;
    src1.assign(src.begin(), src.end() - 1); //分成两部分，即Pi和Pi+1
    src2.assign(src.begin() + 1, src.end());
    for (int i = 0; i < src1.size(); i++)
        cout << src1[i] << endl;
    cout << endl;
    for (int i = 0; i < src2.size(); i++)
        cout << src2[i] << endl;
    cout << endl;
    vector<Point2f> pln1 = bezierCurve(src1);
    vector<Point2f> pln2 = bezierCurve(src2);
    for (float t = 0; t < 1; t += step)
    {
        Point2f temp;
        temp = (1.0 - t) * pln1[cvRound(1.0 / step * t)] + t * pln2[cvRound(1.0 / step * t)];
        res.push_back(temp);
    }
    return res;
}
