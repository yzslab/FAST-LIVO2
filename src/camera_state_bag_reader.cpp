#include "camera_state.h"
#include "LIVMapper.h"

void *run_bag_image_reader(void *arg)
{
  CameraState *cs_ptr = static_cast<CameraState *>(arg);
  cs_ptr->bag_image_reader();
  return nullptr;
}

static void *run_image_decompressor_thread(void *arg)
{
  CameraState *cs_ptr = static_cast<CameraState *>(arg);
  cs_ptr->image_decompressor_thread();
  return nullptr;
}

void CameraState::image_decompressor_thread(void)
{
  auto vio_manager = mapper.vio_managers[id];

  auto width = vio_manager->width;
  auto height = vio_manager->height;
  auto image_resize_factor = vio_manager->image_resize_factor;

  for (;;)
  {
    std::shared_ptr<ImageFuture> image_future;
    {
      std::unique_lock<std::mutex> lock(future_queue_mtx);
      future_queue_cv.wait(lock, [this]
                           { return !future_queue.empty(); });

      image_future = std::move(future_queue.front());
      future_queue.pop();
      if (image_future == nullptr)
      {
        break;
      }
    }

    image_future->image = getImageFromMsg(image_future->compressed_image_msg_ptr);
    if (width != image_future->image.cols || height != image_future->image.rows)
    {
      cv::resize(image_future->image, image_future->image, cv::Size(image_future->image.cols * image_resize_factor, image_future->image.rows * image_resize_factor), 0, 0, CV_INTER_LINEAR);
    }
    {
      std::unique_lock<std::mutex> lock(image_future->mtx);
      image_future->is_done = true;
    }
    image_future->cv.notify_one();
  }
  ROS_INFO("image_decompressor_thread exited");
}

struct ImageExtractorArgs
{
  CameraState *cs_ptr;
  rosbag::View *view_ptr;
};

static void *run_image_extractor(void *arg)
{
  ImageExtractorArgs *extractor_arg_ptr = static_cast<ImageExtractorArgs *>(arg);
  extractor_arg_ptr->cs_ptr->image_extractor(extractor_arg_ptr->view_ptr);
  return nullptr;
}

void CameraState::image_extractor(void *arg)
{
  rosbag::View *view_ptr = static_cast<rosbag::View *>(arg);

  // start decompressors
  std::vector<pthread_t> thread_ids;
  for (int i = 0; i < 4; ++i)
  {
    pthread_t thread_id;
    if (pthread_create(&thread_id, NULL, run_image_decompressor_thread, static_cast<void *>(this)) != 0)
    {
      perror("image_decompressor_thread pthread_create()");
      return;
    }
    pthread_detach(thread_id);
    thread_ids.push_back(thread_id);
  }

  for (const auto &i : *view_ptr)
  {
    if (!ros::ok())
    {
      break;
    }

    // wait until queue not full
    if (processed_image_queue.size() >= 8)
    {
      std::unique_lock<std::mutex> lock(processed_image_queue_mtx);
      processed_image_queue_pop_cv.wait(lock, [this]
                                        { return processed_image_queue.size() < 8; });
    }

    // for compressed image
    sensor_msgs::CompressedImageConstPtr compressed_image_msg_ptr = i.instantiate<sensor_msgs::CompressedImage>();
    if (compressed_image_msg_ptr != nullptr)
    {
      // increase the counter
      ++n_extracted_images;

      auto future_ptr = std::make_shared<ImageFuture>(
          compressed_image_msg_ptr,
          i.getTime(),
          compressed_image_msg_ptr->header);

      // pendding processing queue
      {
        std::unique_lock<std::mutex> lock(future_queue_mtx);
        future_queue.push(future_ptr);
      }
      future_queue_cv.notify_one();

      // processed queue
      {
        std::unique_lock<std::mutex> lock(processed_image_queue_mtx);
        processed_image_queue.push(future_ptr);
      }
      processed_image_queue_push_cv.notify_one();

      continue;
    }

    // for raw image
    sensor_msgs::ImageConstPtr image_msg_ptr = i.instantiate<sensor_msgs::Image>();
    if (image_msg_ptr != nullptr)
    {
      // increase the counter
      ++n_extracted_images;

      auto future_ptr = std::make_shared<ImageFuture>(
          nullptr,
          i.getTime(),
          image_msg_ptr->header);

      future_ptr->image = getImageFromMsg(image_msg_ptr);
      future_ptr->is_done = true;
      // processed queue
      {
        std::unique_lock<std::mutex> lock(processed_image_queue_mtx);
        processed_image_queue.push(future_ptr);
      }
      processed_image_queue_push_cv.notify_one();
      // process_image_message(image_msg_ptr, i, first_lidar_msg_actual_time);
      continue;
    }

    ROS_ERROR("Unsupported image type: %s", i.getDataType().c_str());
    break;
  }

  // send exit signal
  {
    std::unique_lock<std::mutex> lock(future_queue_mtx);
    for (auto &i : thread_ids)
    {
      future_queue.push(nullptr);
    }
  }
  future_queue_cv.notify_all();

  has_image_extraction_finished = true;

  ROS_INFO("Image iteration finished");
}

void CameraState::bag_image_reader(void)
{
  std::vector<rosbag::Bag> bags;
  mapper.open_bags(bags);

  // LiDAR
  uint64_t first_lidar_msg_actual_time;
  {
    rosbag::View lidar_view;
    for (auto &i : bags)
    {
      lidar_view.addQuery(i, rosbag::TopicQuery(mapper.lid_topic));
    }

    rosbag::View::const_iterator lidar_msg_it = lidar_view.begin();
    if (lidar_msg_it == lidar_view.end())
    {
      ROS_ERROR("LiDAR message not found");
      return;
    }

    first_lidar_msg_actual_time = (*lidar_msg_it).getTime().toNSec();
  }

  std::cout << "first_lidar_msg_actual_time=" << first_lidar_msg_actual_time << std::endl;

  rosbag::View view;
  for (const auto &i : bags)
  {
    view.addQuery(i, rosbag::TopicQuery(topic));
    view.addQuery(i, rosbag::TopicQuery(topic + "/compressed"));
  }

  auto thread_arg = ImageExtractorArgs{
      this,
      &view};
  pthread_t image_extractor_thread_id;
  if (pthread_create(&image_extractor_thread_id, NULL, run_image_extractor, static_cast<void *>(&thread_arg)) != 0)
  {
    perror("run_image_extractor pthread_create()");
    return;
  }
  pthread_detach(image_extractor_thread_id);

  for (;;)
  {
    std::shared_ptr<ImageFuture> image_future;
    // retrieve from processed queue
    {
      std::unique_lock<std::mutex> lock(processed_image_queue_mtx);
      processed_image_queue_push_cv.wait(lock, [this]
                                         { return !processed_image_queue.empty(); });

      image_future = std::move(processed_image_queue.front());
      processed_image_queue.pop();
    }
    processed_image_queue_pop_cv.notify_one();

    // wait finishing processing
    if (!image_future->is_done)
    {
      std::unique_lock<std::mutex> lock(image_future->mtx);
      image_future->cv.wait(lock, [image_future]
                            { return image_future->is_done; });
    }

    process_image_message(
      image_future->msg_header,
      image_future->msg_pub_time,
      first_lidar_msg_actual_time,
      image_future->image
    );
    ++n_consumed_images;

    // Has finished?
    if (has_image_extraction_finished && n_consumed_images == n_extracted_images)
    {
      break;
    }
  }

  ROS_INFO("Image processing completed");

  return;
}