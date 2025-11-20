#include "camera_state.h"
#include "LIVMapper.h"

// interfaces

CameraState::CameraState(const std::string &ns_in, const std::string &topic_in, int id_in, LIVMapper &mapper_in) : ns(ns_in), topic(topic_in), id(id_in), mapper(mapper_in)
{
}

void CameraState::message_callback(const sensor_msgs::ImageConstPtr &msg_in)
{
    if (!img_en())
        return;
    sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
    // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
    // {
    //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
    //   sync_jump_flag = true;
    //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
    // }

    // Hiliti2022 40Hz
    if (hilti_en())
    {
        frame_counter = 0;
        if (++frame_counter % 4 != 0)
            return;
    }
    // double msg_header_time =  msg->header.stamp.toSec();
    double msg_header_time = msg->header.stamp.toSec() + img_time_offset();
    if (abs(msg_header_time - last_timestamp_img) < 0.001)
    {
        return;
    }
    ROS_INFO("[%s]Get image, its header time: %.6f", ns.c_str(), msg_header_time);
    if (last_timestamp_lidar() < 0)
    {
        return;
    }

    if (msg_header_time < last_timestamp_img)
    {
        ROS_ERROR("[%s]image loop back. \n", ns.c_str());
        return;
    }

    mtx_buffer.lock();

    double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

    if (img_time_correct - last_timestamp_img < 0.02)
    {
        ROS_WARN("[%s]Image need Jumps: %.6f", ns.c_str(), img_time_correct);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    cv::Mat img_cur = getImageFromMsg(msg);
    img_buffer.push_back(img_cur);
    img_time_buffer.emplace_back(
        img_time_correct,
        msg->header.stamp.toNSec());

    // ROS_INFO("Correct Image time: %.6f", img_time_correct);

    last_timestamp_img = img_time_correct;
    // cv::imshow("img", img);
    // cv::waitKey(1);
    // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

cv::Mat CameraState::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    return mapper.getImageFromMsg(img_msg);
}

cv::Mat CameraState::getImageFromMsg(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    return mapper.getImageFromMsg(img_msg);
}

bool CameraState::has_time(double time)
{
    double diff = get_first_img_time().time - time;
    auto has_time = diff < time_tolerance;

    if (has_time)
    {
        ROS_INFO("[%s] has %f, with diff %f", ns.c_str(), time, diff);
    }
    else
    {
        ROS_INFO("[%s] does not have %f, with diff %f", ns.c_str(), time, diff);
    }

    return has_time;
}

void CameraState::pop()
{
    {
        std::unique_lock<std::mutex> lk(mtx_buffer);
        img_buffer.pop_front();
        img_time_buffer.pop_front();
    }
    // sig_buffer.notify_all();
}

void CameraState::pop_if_time_matches(double time)
{
    if (has_time(time) == false)
    {
        return;
    }
    pop();
}

bool CameraState::empty()
{
    return img_buffer.empty();
}

// bool CameraState::wait()
// {
//     if (!img_en())
//     {
//         return true;
//     }

//     std::unique_lock<std::mutex> lk(mtx_buffer);

//     sig_buffer.wait(lk, [this]
//                     { return !img_buffer.empty() || exit_flag; });

//     if (img_buffer.empty())
//     {
//         return false;
//     }

//     return true;
// }

void CameraState::stop()
{
    {
        std::unique_lock<std::mutex> lk(mtx_buffer);
        exit_flag = true;
    }
    sig_buffer.notify_all();
}

// getters

double CameraState::last_timestamp_lidar()
{
    return mapper.last_timestamp_lidar;
}

double CameraState::img_time_offset()
{
    return mapper.img_time_offset;
}

bool CameraState::img_en()
{
    return mapper.img_en;
}

bool CameraState::hilti_en()
{
    return mapper.hilti_en;
}

bool CameraState::is_paused()
{
    return mapper.is_paused;
}

ImageTime CameraState::get_first_img_time()
{
    assert(img_time_buffer.empty() == false);
    return img_time_buffer.front();
}
