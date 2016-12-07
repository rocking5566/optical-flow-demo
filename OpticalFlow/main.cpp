#include "opencv2/opencv.hpp"
#include "flow.h"
#include <sstream>
#include <iomanip>
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    VideoCapture caputure;
    caputure.open(0);
    Mat frame, output;
    
    while(1)
    {
        caputure.read(frame);
        flow::GetSingleton()->run(frame, output);
        imshow("output", output);

        if (waitKey(1) == 27)
        {
            break;
        }
    }

    caputure.release();
    return 0;
}

