#include <imagePipeline.h>
#include <iostream>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#endif
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
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
        cout<< "Valid Image:" << endl;

        // Set object image and scene image
        // Image is in grayscale
        Mat img_object = boxes.templates[0]; // 0 = Raisin Bran, 1 = Cinnamon, 2 = Rice Krispies
        Mat img_scene = img;

        //Increase contrast of image by 2

        // Define new image
        Mat img_scene_high_contrast;
        Mat img_object_high_contrast;

        // Call .convertTo on original image.
        img_scene.convertTo(img_scene_high_contrast, -1, 2, 0); //increase the contrast by 2
        // Replace old image
        img_scene = img_scene_high_contrast;

        // Display windows
        String windowNameContrastHigh2 = "Contrast Increased by 2";
        namedWindow(windowNameContrastHigh2, WINDOW_NORMAL);   
        imshow(windowNameContrastHigh2, img_scene_high_contrast);     
        waitKey(0); // Wait for any key stroke

        destroyAllWindows(); //destroy all open windows

        //-- Step 1: Detect the keypoints using SURF Detector, compute the
        // descriptors
        int minHessian = 800; //400
        Ptr<SURF> detector = SURF::create( minHessian );
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;

        detector->detectAndCompute( img_scene, noArray(), keypoints_scene,
        descriptors_scene );

        // Find template id based on matches
        int max_matches = 0;
        int max_match_id = -1;
        int match_thresh = 60; // smallest number of matches to be considered similar images
        int num_good_matches;
        std::vector<DMatch> good_matches;


        for (int img_id = 0; img_id < 3; img_id++){

            //Load image
            img_object = boxes.templates[img_id];
            img_object.convertTo(img_object_high_contrast, -1, 2, 0);
            img_object = img_object_high_contrast;

            //SURF detection on image
            detector->detectAndCompute( img_object, noArray(), keypoints_object,
            descriptors_object );

            // Match Features
            Ptr<DescriptorMatcher> matcher =
            DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector< std::vector<DMatch> > knn_matches;
            matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2
            );
            //-- Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.75f;

            for (size_t i = 0; i < knn_matches.size(); i++)
            {
                if (knn_matches[i][0].distance < ratio_thresh *
                knn_matches[i][1].distance)
                {
                good_matches.push_back(knn_matches[i][0]);
                }
            }
            num_good_matches = good_matches.size();
            cout << "good matches: " << num_good_matches <<endl;
            cout << "box id: " << img_id <<endl;

            if ((num_good_matches > max_matches)&&(num_good_matches > match_thresh)){
                max_matches = num_good_matches;
                max_match_id = img_id;
            }
        }
        
        cout << "Max Matches, ID : " << max_matches << ", "<< max_match_id <<endl;

        //-- Draw matches
        Mat img_matches;
        img_object = boxes.templates[max_match_id];
        // img_object.convertTo(img_object_high_contrast, -1, 2, 0);
        // img_object = img_object_high_contrast;

        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
        good_matches, img_matches, Scalar::all(-1),
        Scalar::all(-1), std::vector<char>(),
        DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }

        Mat H = findHomography( obj, scene, RANSAC );

//-- Get the corners from the image_1 ( the object to be "detected" )
std::vector<Point2f> obj_corners(4);
obj_corners[0] = Point2f(0, 0);
obj_corners[1] = Point2f( (float)img_object.cols, 0 );
obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
obj_corners[3] = Point2f( 0, (float)img_object.rows );
std::vector<Point2f> scene_corners(4);
perspectiveTransform( obj_corners, scene_corners, H);

//-- Draw lines between the corners (the mapped object in the scene - image_2 )
line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//-- Show detected matches
imshow("Good Matches & Object detection", img_matches );
waitKey();
return max_match_id;

    }

    return template_id;
}