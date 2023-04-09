

cv::Mat image1,image2;

cv::namedWindow("Car",cv::WINDOW_AUTOSIZE);

cv::imshow("Car",image1);   

#include <opencv2/opencv.hpp>
int main()
{

cv::Mat afraid_img, excited_img, angry_img, sad_img;
afraid_img=cv::imread("/images/benz.jpeg");
//cv::namedWindow("AHHH",cv::WINDOW_AUTOSIZE);

cv::imshow("AHHH",image1);
//Car window is used 

cv::waitKey(0);
cv::destroyAllWindows();

return 0;
}