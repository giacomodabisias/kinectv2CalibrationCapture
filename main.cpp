//Copyright 2014 Giacomo Dabisias
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//This is preliminary software and/or hardware and APIs are preliminary and subject to change.
#include "Kinect2Grabber.hpp"

int main(int argc, char *argv[])
{
  if(argc < 2){
    std::cout << "insert number of images to capture and rgb size and depth size (ex: 16 512 424 1920 1080)" <<std::endl;
    exit(0);
  }

  Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("../calibration/rgb_calibration.yaml", "../calibration/depth_calibration.yaml", "../calibration/pose_calibration.yaml");
  
  cv::Size boardSize(6,9);
  int count = 0;
  int counter = 0;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
  const cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
  boost::filesystem::path dir("./images");
  boost::filesystem::create_directory(dir);
  boost::filesystem::path dir0("./images_raw");
  boost::filesystem::create_directory(dir0);
  boost::filesystem::path dir1("./images/ir");
  boost::filesystem::create_directory(dir1);
  boost::filesystem::path dir2("./images_raw/ir");
  boost::filesystem::create_directory(dir2);
  boost::filesystem::path dir3("./images_raw/depth");
  boost::filesystem::create_directory(dir3);
  boost::filesystem::path dir4("./images_raw/rgb");
  boost::filesystem::create_directory(dir4);
  boost::filesystem::path dir5("./images/rgb");
  boost::filesystem::create_directory(dir5);

  while(true)
  {
    
    libfreenect2::FrameMap * frames = k2g.getRawFrames();
    libfreenect2::Frame *rgb = (*frames)[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = (*frames)[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = (*frames)[libfreenect2::Frame::Depth];

    cv::Mat ir_gray, ir_scaled, rgb_gray, rgb_scaled;

    cv::Mat rgb_image = cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data);
    cv::Mat ir_image = cv::Mat(ir->height, ir->width, CV_32FC1, ir->data);
    cv::Mat depth_image = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    //cv::resize(rgb_image, rgb_scaled, cv::Size(atoi(argv[1]), atoi(argv[2])), cv::INTER_CUBIC);

    cv::cvtColor(rgb_image, rgb_gray, 7); // rgb2gray
    //cv::resize(ir_image, ir_scaled, cv::Size(atoi(argv[3]), atoi(argv[4])), cv::INTER_CUBIC);
    //convertIr(ir_scaled, ir_gray, 0, 0x7FFF);
    ir_image.convertTo(ir_gray, CV_8U, 255.0 / 0x7FFF);
    clahe->apply(ir_gray, ir_gray);

    std::vector<cv::Point2f > camera1ImagePoints;
    bool found1 = cv::findChessboardCorners(rgb_gray, boardSize, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);
    
    std::vector<cv::Point2f> camera2ImagePoints;
    bool found2 = cv::findChessboardCorners(ir_gray, boardSize, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    counter++;
    int c = cv::waitKey(30);
    if( (char)c == 'f' ){
      counter = 0;
      std::string ir_name = std::string("./images/ir/ir_image_") + std::to_string(count) + std::string(".jpg");
      std::string rgb_name = std::string("./images/rgb/rgb_image_") + std::to_string(count) + std::string(".jpg");
      std::string depth_raw_name = std::string("./images_raw/depth/depth_image_") + std::to_string(count) + std::string(".bmp");
      std::string ir_raw_name = std::string("./images_raw/ir/depth_image_") + std::to_string(count) + std::string(".bmp");
      std::string rgb_raw_name = std::string("./images_raw/rgb/depth_image_") + std::to_string(count) + std::string(".jpg");

      //save calibration images
      std::cout << "saving image " << ++count << std::endl;
      imwrite( ir_name, ir_gray );
      imwrite( rgb_name, rgb_gray );

      //save raw images
      cv::Mat m(ir_image.rows, ir_image.cols, CV_8UC4, ir_image.data);
      cv::Mat m2(depth_image.rows, depth_image.cols, CV_8UC4, depth_image.data);
      imwrite( rgb_raw_name, rgb_image );
      imwrite( ir_raw_name, ir_image );
      imwrite( depth_raw_name, depth_image );

      if(count == atoi(argv[1]))
        break;
    }

    if(found1){
      cv::cornerSubPix(rgb_gray, camera1ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
      drawChessboardCorners(rgb_image, boardSize, camera1ImagePoints, found1);
    }

   if(found2){
      cv::cornerSubPix(ir_gray, camera2ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
      drawChessboardCorners(ir_gray, boardSize, camera2ImagePoints, found2);

    }
    cv::imshow("rgb", rgb_image);
    cv::imshow("ir", ir_gray  );  
  }

  k2g.shutDown();
  return 0;
}

