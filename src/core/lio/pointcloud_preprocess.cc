#include "pointcloud_preprocess.h"
#include <execution>

#include <glog/logging.h>

namespace lightning {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg, PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;
        case LidarType::AVIA:
            livox_ros_skyland_handler(msg);
            break;
        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg,
                                   PointCloudType::Ptr &pcl_out) {
    cloud_out_.clear();
    cloud_full_.clear();

    int plsize = msg->point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<bool> is_valid_pt(plsize, false);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        if ((msg->points[i].line < num_scans_) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num_ == 0) {
                cloud_full_[i].x = msg->points[i].x;
                cloud_full_[i].y = msg->points[i].y;
                cloud_full_[i].z = msg->points[i].z;
                cloud_full_[i].intensity = msg->points[i].reflectivity;

                // use curvature as time of each laser points, curvature unit: ms
                cloud_full_[i].time = msg->points[i].offset_time / double(1000000);

                if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                    (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                    (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                        (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                             cloud_full_[i].z * cloud_full_[i].z >
                         (blind_ * blind_))) {
                    is_valid_pt[i] = true;
                }
            }
        }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].t / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}



void PointCloudPreprocess::livox_ros_skyland_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
  /* 清除之前的点云缓存 */
  cloud_out_.clear();
  cloud_full_.clear();

  pcl::PointCloud<livox_ros::PointSkyland> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);     // fromROSMsg读数据
  int plsize = pl_orig.points.size(); // 一帧点云中的点数
  if (plsize == 0){
    printf("Error: No points in the point cloud.\n");
    return;
  }

  // 分配空间
  cloud_out_.reserve(plsize);
  cloud_full_.resize(plsize);

  uint valid_num = 0; // 有效的点数

  // 不进行特征处理,分别对每个点进行处理
  for (uint i = 1; i < plsize; i++)
  {
    // 只取线数在0~N_SCANS内并且回波次序(tag标签bit5和bit4)为0或者1的点云
    if ((pl_orig.points[i].line < num_scans_) && ((pl_orig.points[i].tag & 0x30) == 0x10 || (pl_orig.points[i].tag & 0x30) == 0x00))
    {
      valid_num++; // 满足回波序列01和00的计数有效的点数
      // 等间隔降采样,选取降采样的点
      if (valid_num % point_filter_num_ == 0)
      {
        cloud_full_[i].x = pl_orig.points[i].x;                 // 点的x轴坐标
        cloud_full_[i].y = pl_orig.points[i].y;                 // 点的y轴坐标
        cloud_full_[i].z = pl_orig.points[i].z;                 // 点的z轴坐标
        cloud_full_[i].intensity = pl_orig.points[i].intensity; // 点的强度
        cloud_full_[i].time = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp)/1000000; // 点的曲率

        // 间距太小不利于特征提取,只有当当前点和上一点的间距>1e-7,并且在最小距离阈值之外,才认为是有用点,加入到pl_surf队列中
        if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) || (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) || (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) && (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y + cloud_full_[i].z * cloud_full_[i].z > (blind_ * blind_)))
        {
          cloud_out_.push_back(cloud_full_[i]); // 将当前点加入到对应line的pl_fuff队列中
        }
      }
    }
  }
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}



}  // namespace lightning
