/*!
* \class ImageMatching
* \brief module which uses OpenCV image rectification
* \author Tobias Langner
*/

#include "ImageMatching.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <math/Types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

//#define PERFORMANCETEST


namespace aa
{
namespace modules
{
namespace vision
{

REGISTERTASKCONTEXT(ImageMatching);

//////////////////////////////////////////////////////
// Constructor:

ImageMatching::ImageMatching(std::string const & name)
	: util::RtTaskContext(name)
    , mImage1Out("Image1Out")
    , mImage2Out("Image2Out")
    , mMatchedImageOut("MatchedImageOut")
{
    addPort(mImage1Out);
    addPort(mImage2Out);
    addPort(mMatchedImageOut);
}

//////////////////////////////////////////////////////
// Destructor:

ImageMatching::~ImageMatching()
{
}

//////////////////////////////////////////////////////
// TaskContext interface

bool ImageMatching::startHook()
{
	RTT::Logger::In in(getName());


    //initialize Sift detector (number of features = 1000)
    cv::SiftFeatureDetector detector(1000);

    //find keypoints in first image.
    cv::Mat input1 = cv::imread("../../aa/resources/auto1.png", CV_LOAD_IMAGE_GRAYSCALE);
    std::vector<cv::KeyPoint> keypoints1;
    detector.detect(input1, keypoints1);

    //draw keypoints of first image.
    cv::Mat output1;
    cv::drawKeypoints(input1, keypoints1, output1, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    //find keypoints in second image.
    cv::Mat input2 = cv::imread("../../aa/resources/auto2.png", CV_LOAD_IMAGE_GRAYSCALE);
    std::vector<cv::KeyPoint> keypoints2;
    detector.detect(input2, keypoints2);

    //draw keypoints of second image.
    cv::Mat output2;
    cv::drawKeypoints(input2, keypoints2, output2, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //compute descriptors
    cv::SiftDescriptorExtractor extractor;
    cv::Mat descriptors1,descriptors2;
    extractor.compute(input1,keypoints1,descriptors1);
    extractor.compute(input2,keypoints2,descriptors2);

    //match descriptors
    cv::BFMatcher matcher;
    cv::vector<cv::DMatch> matches;
    matcher.match(descriptors1,descriptors2,matches);

    //draw matches
    cv::Mat output3;
    cv::drawMatches(input1,keypoints1,input2,keypoints2,matches,output3,cv::Scalar::all(-1),cv::Scalar::all(-1), cv::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //initialize images for display modules
    TimeStamp now;
    now.stamp();
    mImage1 = ::data::TimedImagePtr(now, ::data::ImagePtr(new ::data::Image));
    mImage2 = ::data::TimedImagePtr(now, ::data::ImagePtr(new ::data::Image));
    mMatchedImage = ::data::TimedImagePtr(now, ::data::ImagePtr(new ::data::Image));

    output1.copyTo(*mImage1);
    output2.copyTo(*mImage2);
    output3.copyTo(*mMatchedImage);


    ///TODO: implement RANSAC

    ///sample code for accessing matches/keypoints/coordinates
    //draw random match
    math::flt min = 0;
    math::flt max = matches.size()-1;
    math::flt randomvalue = (rand() * (max-min) / RAND_MAX)+min;
    int index = (int)randomvalue;
    cv::DMatch match = matches[index];

    //get corresponding keypoint in first image
    cv::KeyPoint kp1 = keypoints1[match.queryIdx];
    //get corresponding keypoint in second image
    cv::KeyPoint kp2 = keypoints2[match.trainIdx];

    //get x coordinate
    float kp1_x_coordinate = kp1.pt.x;

    ///end sample code


    return true;
}


void ImageMatching::updateHook()
{
    RTT::Logger::In in("ImageMatching");

    //write output images to ports
    mImage1Out.write(mImage1);
    mImage2Out.write(mImage2);
    mMatchedImageOut.write(mMatchedImage);
}

void ImageMatching::stopHook()
{
}

}
}
}


