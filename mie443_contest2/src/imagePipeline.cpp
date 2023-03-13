#include <imagePipeline.h>
#include <iostream>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

// Returns an image 
ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}


int ImagePipeline::getTemplateID(Boxes& boxes) { // a copy of boxes is passed into this function
/*
Purpose (how this function is used):
This function finds the ID of what cereal box image that the robot is seeing.

Params:
- boxes: list of possible cereal box images it could see

Global? Vars:
- img: image object of what the robot sees

Output:
- Template ID of the cereal box image it sees

*/
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        /*
        - What is boxes.templates? 
        - What is img?
        - Where do we find img_scene (image taken from scene) and img_object (reference image)?
        */
 
        // Convert to gray
        cv::Mat grey_img;
        cv:cvtColor(img,grey_img,CV_BGR2GRAY);

        // Set object image and scene image
        Mat img_object = boxes.templates[0];
        Mat img_scene = img;

        //-- Step 1: Detect the keypoints using SURF Detector, compute the
        // descriptors
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create( minHessian );
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;

        detector->detectAndCompute( img_object, noArray(), keypoints_object,
        descriptors_object );

        detector->detectAndCompute( img_scene, noArray(), keypoints_scene,
        descriptors_scene );

    }  
    return template_id;
}
#endif