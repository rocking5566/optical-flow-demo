#include "flow.h"
#define MIN_CORNER_COUNT 10
#define MAX_CORNER_COUNT 500
#define CORNER_QUALITY 0.01
#define CORNER_MIN_DIST 10.
#define TRUE_MOTION 2

flow* flow::m_self = NULL;

flow::~flow()
{
    delete m_self;
    m_self = NULL;
}

flow* flow::GetSingleton()
{
    if (m_self == NULL)
    {
        m_self = new flow();
    }

    return m_self;
}

void flow::run(const Mat& src, Mat& dst)
{
    cvtColor(src, m_CurFrame, CV_BGR2GRAY);
    src.copyTo(dst);

    if (m_PrevCorner.size() <= MIN_CORNER_COUNT)
    {
        vector<Point2f> corners;
        goodFeaturesToTrack(m_CurFrame, corners, MAX_CORNER_COUNT, CORNER_QUALITY, CORNER_MIN_DIST);
        m_PrevCorner.insert(m_PrevCorner.end(), corners.begin(), corners.end());
        m_StartCorner.insert(m_StartCorner.end(), corners.begin(), corners.end());
    }

    if (m_PrevFrame.empty())
    {
        m_CurFrame.copyTo(m_PrevFrame);
    }

    calcOpticalFlowPyrLK(m_PrevFrame, m_CurFrame, m_PrevCorner, m_CurCorner, status, err);
    RemoveStationaryPoint();
    Draw(dst);

    std::swap(m_CurCorner, m_PrevCorner);
    cv::swap(m_PrevFrame, m_CurFrame);
}

void flow::RemoveStationaryPoint()
{
    int k = 0;

    for (int i = 0; i < m_CurCorner.size(); i++)
    {
        if (IsMove(i))
        {
            m_StartCorner[k] = m_StartCorner[i];
            m_CurCorner[k++] = m_CurCorner[i];
        }
    }

    m_CurCorner.resize(k);
    m_StartCorner.resize(k);
}

bool flow::IsMove(int i)
{
    return status[i] &&
		abs(m_CurCorner[i].x - m_PrevCorner[i].x) + abs(m_CurCorner[i].y - m_PrevCorner[i].y) > TRUE_MOTION;
}

void flow::Draw(Mat& img)
{
    for (int i = 0; i < m_CurCorner.size(); i++)
    {
        line(img, m_StartCorner[i], m_CurCorner[i], Scalar::all(0));
        circle(img, m_CurCorner[i], 3, Scalar::all(0), (-1));
    }
}

