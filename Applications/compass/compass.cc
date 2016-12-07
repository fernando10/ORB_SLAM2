/**
* This file is part of Compass.
*
* Copyright (C) 2016 University of Colorado at Boulder
*
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include "GetPot"
#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Messages/Matrix.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_rig.h>

#include<opencv2/core/core.hpp>

#include<System.h>

hal::Camera camera_device;
calibu::Rig<double> rig;
std::shared_ptr<GetPot> cl;
std::shared_ptr<hal::Image> camera_img;
int image_width;
int image_height;
double image_timestamp;

using namespace std;

bool LoadCameras();

int main(int argc, char **argv)
{
  srand(0);
  cl = std::shared_ptr<GetPot>(new GetPot(argc, argv));

  cerr << "Initializing camera...";
  LoadCameras();

  std::string vocab_file = cl->follow("", "-vocab");
  std::string config_file = cl->follow("", "-config");


  //  if(argc != 5)
  //  {
  //    cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
  //    return 1;
  //  }

  // Retrieve paths to images
  //  vector<string> vstrImageFilenames;
  //  vector<double> vTimestamps;
  //  LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

  //  int nImages = vstrImageFilenames.size();

  //  if(nImages<=0)
  //  {
  //    cerr << "ERROR: Failed to load images" << endl;
  //    return 1;
  //  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(vocab_file,config_file,ORB_SLAM2::System::MONOCULAR,true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;

  // Main loop
  cv::Mat im;
  bool capture_success = false;
  std::shared_ptr<hal::ImageArray> images = hal::ImageArray::Create();
  std::vector<pangolin::GlTexture> gl_tex;
  size_t num_capture_failed = 0;

  while(num_capture_failed < 10)
  {
    capture_success = false;

    capture_success = camera_device.Capture(*images);

    if (capture_success){
      num_capture_failed = 0;
      // could be capturing many images
      gl_tex.resize(images->Size());

//      for (uint32_t cam_id = 0 ; cam_id < (unsigned int) images->Size() ; ++cam_id) {
//        if (!gl_tex[cam_id].tid) {
//          camera_img = images->at(cam_id);
////          GLint internal_format = (camera_img->Format() == GL_LUMINANCE ?
////                                     GL_LUMINANCE : GL_RGBA);
////          // Only initialise now we know format.
////          gl_tex[cam_id].Reinitialise(
////                camera_img->Width(), camera_img->Height(), internal_format,
////                false, 0, camera_img->Format(), camera_img->Type(), 0);
//        }
//      }

      camera_img = images->at(0);
      image_width = camera_img->Width();
      image_height = camera_img->Height();
      image_timestamp = camera_img->Timestamp();

      std::vector<cv::Mat> cvmat_images;
      for (int ii = 0; ii < images->Size() ; ++ii) {
        cvmat_images.push_back(images->at(ii)->Mat());
      }

      im = cvmat_images.at(0);

      if(im.empty())
      {
        cerr << endl << "Failed to load image." << endl;
        return 1;
      }


#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im,image_timestamp);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

      double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      //vTimesTrack[ni]=ttrack;

    }else{
      num_capture_failed++;
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  cout << "finished" << endl;
  return 0;
}

bool LoadCameras()
{

  //LoadCameraAndRig(*cl, camera_device, rig);
  std::string cam_string = cl->follow("", "-cam");

  try {
    camera_device = hal::Camera(hal::Uri(cam_string));
  }
  catch (hal::DeviceException& e) {
    cerr << "Error loading camera device: " << e.what() << std::endl;
    return false;
  }

  return true;
}


