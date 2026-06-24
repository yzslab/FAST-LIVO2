#ifndef LIV_CAMERA_H
#define LIV_CAMERA_H

#include <string>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "common_lib.h"

class LIVMapper;
struct ImageFuture;

void *run_bag_image_reader(void *arg);

struct CameraState
{
  LIVMapper &mapper;

  // camera properties

  std::string ns;

  std::string topic;

  int id;

  int frame_counter = 0;

  double last_timestamp_img = -1;

  // double img_time_offset = 0.;

  // treat as equal if time difference < time_tolerance
  // double time_tolerance = 0.05; // 50ms
  // double time_tolerance = 0.025; // 25ms
  double time_tolerance = 0.00001;  // 0

  // double exposure_time_init = 0.;

  bool exit_flag = false;

  // buffers

  deque<cv::Mat> img_buffer;

  deque<ImageTime> img_time_buffer;

  // synchronization

  std::mutex mtx_buffer;

  std::condition_variable sig_buffer;

  // interfaces

  CameraState(const std::string &ns, const std::string &topic, int, LIVMapper &);

  void message_callback(const sensor_msgs::ImageConstPtr &);

  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &);

  cv::Mat getImageFromMsg(const sensor_msgs::CompressedImageConstPtr &img_msg);

  bool has_time(double);

  /**
   * Pop an item from each buffer
   */
  void pop();

  /**
   * Pop an item from each buffer when the time in the `img_time_buffer` equal to the provided time
   */
  void pop_if_time_matches(double);

  template <typename T>
  void process_image_message(T &msg_header, const ros::Time msg_pub_time, uint64_t first_lidar_msg_actual_time, cv::Mat cv_image)
  {
    while (is_paused())
    {
      usleep(100000);
    }

    // Drop image comming before LiDAR
    if (msg_pub_time.toNSec() < first_lidar_msg_actual_time)
    {
      std::cout << "[" << topic << "]Drop an image comming before LiDAR: seq=" << msg_header.seq << ", time=" << msg_header.stamp << std::endl;
      return;
    }

    double msg_header_time = msg_header.stamp.toSec() + img_time_offset();

    // Drop image with time same as the previous one
    if (abs(msg_header_time - last_timestamp_img) < 0.001)
    {
      return;
    }

    if (msg_header_time < last_timestamp_img)
    {
      ROS_ERROR("[%s]image loop back. \n", ns.c_str());
      return;
    }

    double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

    if (img_time_correct - last_timestamp_img < 0.02)
    {
      ROS_WARN("[%s]Image need Jumps: %.6f", ns.c_str(), img_time_correct);
      return;
    }

    // auto cv_image = getImageFromMsg(msg_ptr);

    for (;;)
    {
      if (img_buffer.size() < 32)
      {
        {
          std::lock_guard<std::mutex> lk(mtx_buffer);
          img_buffer.push_back(cv_image);
          img_time_buffer.emplace_back(
              img_time_correct,
              msg_header.stamp.toNSec());
          last_timestamp_img = img_time_correct;
          sig_buffer.notify_all();
        }
        ROS_INFO("[%s]Get image, its header time: %.6f, %ld images in buffer", ns.c_str(), msg_header_time, img_buffer.size());
        break;
      }
      usleep(50000);
    }
  }

  bool empty();

  /**
   * wait until image buffer is not empty
   */
  // bool wait();

  void stop();

  // bag reader

  std::atomic<bool> has_image_extraction_finished{false};
  std::atomic<uint32_t> n_extracted_images{0};
  uint32_t n_consumed_images{0};

  void bag_image_reader(void);
  void image_decompressor_thread(void);
  void image_extractor(void *);
  
  std::queue<std::shared_ptr<ImageFuture>> future_queue;
  std::mutex future_queue_mtx;
  std::condition_variable future_queue_cv;  

  std::queue<std::shared_ptr<ImageFuture>> processed_image_queue;
  std::mutex processed_image_queue_mtx;
  std::condition_variable processed_image_queue_push_cv; // notify on pushing
  std::condition_variable processed_image_queue_pop_cv; // notify on poping

  // getters

  double last_timestamp_lidar();

  double img_time_offset();

  bool img_en();

  bool hilti_en();

  bool is_paused();

  bool is_finished();

  ImageTime get_first_img_time();
};
#endif