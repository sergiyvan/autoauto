/*!
* \class ImageMatching
* \brief module which uses OpenCV image rectification
* \author Tobias Langner
*/

#include "ImageMatching.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <math/Types.h>
#include <math.h>
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



    ///TODO: implement RANSAC
    ///
    math::flt max;
    int index;
    cv::DMatch match;
    cv::DMatch temp_match;
    cv::DMatch best_match;
    std::vector<cv::DMatch> maybe_inliner;
    std::vector<cv::DMatch> better_inliner;
    std::vector<cv::DMatch> best_inliner;
    float offset;
    float best_offset;
    float temp_offset;
    int cur_num_inliner = 0;
    int best_num_inliner = 0;
    float e = 0.7;
    float p = 0.99;
    int N;

    do{
        N =(int) std::log((1. - p))  / std::log(1.-(1.-e));
        best_inliner.clear();
        best_inliner.insert(best_inliner.end(),better_inliner.begin(),better_inliner.end());
        better_inliner.clear();

        best_num_inliner = cur_num_inliner;
        int j = 0;
        while(j < N){
            //get new random matches
            max = matches.size()-1;
            index = (int)(rand() * (max) / RAND_MAX);
            //index = (int)(rand() * (max-min) / RAND_MAX)+min;
            match = matches[index];

           //get offset (only x direction)
           offset = abs(keypoints1[match.queryIdx].pt.x - keypoints2[match.trainIdx].pt.x);

           //loop over all matches
           for(int i=0; i<matches.size();i++){
               temp_match = matches[i];
               temp_offset= abs(keypoints1[temp_match.queryIdx].pt.x-keypoints2[temp_match.trainIdx].pt.x);

               //offset difference is smalles as one pixel
               if(abs(offset-temp_offset)< 1. &&abs(offset-temp_offset)> -1.){
                   maybe_inliner.push_back(temp_match);
               }
           }
           //more inliner -> better offset
           if(maybe_inliner.size()> better_inliner.size()){
               better_inliner.clear();
               better_inliner.insert(better_inliner.end(),maybe_inliner.begin(),maybe_inliner.end());
               best_match = match;
               best_offset = offset;
           }
           maybe_inliner.clear();
           j++;
        }
        std::cout << " " << std::endl;
        std::cout << "==========" << std::endl;
        std::cout << "Gewaehltes e: " << e << std::endl;
        std::cout << "Berechnetes N: " << N << std::endl;
        std::cout << "Ermitterlter Offset: " << best_offset << std::endl;
        std::cout << "Anzahl der Inlier: " << better_inliner.size() << std::endl;
        std::cout << "=========="  << std::endl;
        e = e-0.05;
        cur_num_inliner = better_inliner.size();
    }
    while(cur_num_inliner >= best_num_inliner);

    //draw matches -> draw image3
    cv::Mat output3;
    cv::drawMatches(input1,keypoints1,input2,keypoints2,best_inliner,output3,cv::Scalar::all(-1),cv::Scalar::all(-1), cv::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //cv::drawMatches(input1,keypoints1,input2,keypoints2,matches,output3,cv::Scalar::all(-1),cv::Scalar::all(-1), cv::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    //initialize images for display modules
    TimeStamp now;
    now.stamp();
    mImage1 = ::data::TimedImagePtr(now, ::data::ImagePtr(new ::data::Image));
    mImage2 = ::data::TimedImagePtr(now, ::data::ImagePtr(new ::data::Image));
    mMatchedImage = ::data::TimedImagePtr(now, ::data::ImagePtr(new ::data::Image));

    output1.copyTo(*mImage1);
    output2.copyTo(*mImage2);
    output3.copyTo(*mMatchedImage);



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


