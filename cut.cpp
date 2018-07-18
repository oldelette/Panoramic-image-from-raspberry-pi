#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

/*void testrange(Mat &img)
{
    int m = img.rows;
    int n = img.cols;
    Mat temp = img(Range(0,m*0.8), Range(0,n*0.8));
    namedWindow("Range");
    imshow("Range", temp);
}*/

void testroi(Mat &img)
{
    int m = img.rows;
    int n = img.cols;
    Rect rect(500,0,1000,m);
    Mat temp(img, rect);
    imwrite("a2.jpg", temp);
}

int main()
{
    Mat img = imread("magic1.jpg");
    testroi(img);
    return 0;
}
