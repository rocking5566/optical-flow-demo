#ifndef flow_h__
#define flow_h__
#define MAX_DIRECTION 8

#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

class flow
{
    public:
        static flow* GetSingleton();

        void run(const Mat& src, Mat& dst);
    private:
        flow();
        ~flow();
        void RemoveStationaryPoint();
        bool IsMove(int i);
        void Draw(Mat& img);
        int Direction(float x, float y);
        void PrintDirection(int* direction);
    private:
        static flow* m_self;

        Mat m_CurFrame;
        Mat m_PrevFrame;
        vector<Point2f> m_CurCorner;
        vector<Point2f> m_PrevCorner;
        vector<Point2f> m_StartCorner;
        vector<uchar> status;
        vector<float> err;
        string m_dirName[MAX_DIRECTION];
};
#endif // flow_h__