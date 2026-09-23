/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef USE_ROS1
#ifndef USE_ROS2
#error "Please define either USE_ROS1 or USE_ROS2, but not both."
#endif
#endif

#ifdef USE_ROS1
#ifdef USE_ROS2
#error "Cannot use both USE_ROS1 and USE_ROS2 at the same time. Please define only one."
#endif
#endif

#ifdef USE_ROS2

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <super_utils/color_msg_utils.hpp>

namespace rog_map {
    using namespace super_utils;


    class ROGMapROS : public ROGMap {
        rclcpp::Node::SharedPtr nh_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> br_map_ego_;


        const double getSystemWalltimeNow() override {
            return nh_->get_clock()->now().seconds();
        }

        void getSystemWalltimeNow(rclcpp::Time& _in) {
            _in = nh_->get_clock()->now();
        };

        struct VisualizeMap {
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
                occ_pub, unknown_pub, esdf_neg_pub, esdf_occ_pub,
                occ_inf_pub, unknown_inf_pub, frontier_pub, esdf_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_arr_pub;
            rclcpp::TimerBase::SharedPtr viz_timer;
            rclcpp::CallbackGroup::SharedPtr viz_reen_cbk_group;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr proj2d_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr proj2d_sdf_pub;
            /// 上一次发出去的 Field2D 版本号，用于"每生成一帧地图只发一次"
            uint64_t published_proj2d_version{0};
            /// 上一次发出去的 ESDF 对应的地图帧号
            uint64_t published_esdf_frame_count{0};

        } vm_;

        struct ROSCallback {
            rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
            int unfinished_frame_cnt{0};
            Pose pc_pose;
            PointCloud pc;
            rclcpp::TimerBase::SharedPtr update_timer;
            mutex updete_lock;
        } rc_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            updateRobotState(std::make_pair(Vec3f(odom_msg->pose.pose.position.x,
                                                  odom_msg->pose.pose.position.y,
                                                  odom_msg->pose.pose.position.z),
                                            Quatf(odom_msg->pose.pose.orientation.w,
                                                  odom_msg->pose.pose.orientation.x,
                                                  odom_msg->pose.pose.orientation.y,
                                                  odom_msg->pose.pose.orientation.z)));


            if (cfg_.pub_odom_tf) {
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = odom_msg->header.stamp;
                transformStamped.header.frame_id = cfg_.frame_id;
                transformStamped.child_frame_id = odom_msg->child_frame_id;
                transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
                transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
                transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
                transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
                transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
                transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
                transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
                br_map_ego_->sendTransform(transformStamped);
            }
        }

        void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
            if (!robot_state_.rcv) {
                std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback." << RESET << std::endl;
                return;
            }
            double cbk_t = nh_->get_clock()->now().seconds();
            if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
                std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
                return;
            }
            PointCloud temp_pc;
            pcl::fromROSMsg(*cloud_msg, temp_pc);
            rc_.updete_lock.lock();
            rc_.pc = temp_pc;
            rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
            rc_.unfinished_frame_cnt++;
            map_empty_ = false;
            rc_.updete_lock.unlock();
        }

        void updateCallback() {
            if (map_empty_) {
                static double last_print_t = nh_->get_clock()->now().seconds();
                double cur_t = nh_->get_clock()->now().seconds();
                if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
                    std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET <<
                        std::endl;
                    last_print_t = cur_t;
                }
                return;
            }
            if (rc_.unfinished_frame_cnt == 0) {
                return;
            }

            if (rc_.unfinished_frame_cnt > 1) {
                std::cout << YELLOW <<
                    " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
                    << std::endl;
            }
            static PointCloud temp_pc;
            static Pose temp_pose;

            rc_.updete_lock.lock();
            temp_pc = rc_.pc;
            temp_pose = rc_.pc_pose;
            rc_.unfinished_frame_cnt = 0;
            rc_.updete_lock.unlock();

            updateProbMap(temp_pc, temp_pose);
            // 建图线程内同步发布，保证"生成一帧就发一帧"
            publishProj2D();
            publishESDF();

            writeTimeConsumingToLog(time_log_file_);
        }

        void vizCallback() {
            if (!cfg_.visualization_en) {
                return;
            }
            if (map_empty_) {
                return;
            }

            Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
            Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

            boundBoxByLocalMap(box_min, box_max);
            if ((box_max - box_min).minCoeff() <= 0) {
                cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << endl;
                return;
            }

            if (cfg_.pub_unknown_map_en && vm_.unknown_pub->get_subscription_count() >= 1) {
                vec_E<Vec3f> unknown_map, inf_unknown_map;
                boxSearch(box_min, box_max, UNKNOWN, unknown_map);
                sensor_msgs::msg::PointCloud2 cloud_msg;
                vecEVec3fToPC2(unknown_map, cloud_msg);
                cloud_msg.header.stamp = nh_->get_clock()->now();
                vm_.unknown_pub->publish(cloud_msg);
                if (cfg_.unk_inflation_en && vm_.unknown_inf_pub->get_subscription_count() >= 1) {
                    boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
                    vecEVec3fToPC2(inf_unknown_map, cloud_msg);
                    cloud_msg.header.stamp = nh_->get_clock()->now();
                    vm_.unknown_inf_pub->publish(cloud_msg);
                }
            }

            if (cfg_.frontier_extraction_en && vm_.frontier_pub->get_subscription_count() >= 1) {
                vec_E<Vec3f> frontier_map;
                boxSearch(box_min, box_max, FRONTIER, frontier_map);
                sensor_msgs::msg::PointCloud2 cloud_msg;
                vecEVec3fToPC2(frontier_map, cloud_msg);
                cloud_msg.header.stamp = nh_->get_clock()->now();
                vm_.frontier_pub->publish(cloud_msg);
            }

            vec_E<Vec3f> occ_map, inf_occ_map;
            sensor_msgs::msg::PointCloud2 cloud_msg;

            if (vm_.occ_pub->get_subscription_count() >= 1) {
                boxSearch(box_min, box_max, OCCUPIED, occ_map);
                vecEVec3fToPC2(occ_map, cloud_msg);
                vm_.occ_pub->publish(cloud_msg);
            }

            if (vm_.occ_inf_pub->get_subscription_count() >= 1) {
                boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
                vecEVec3fToPC2(inf_occ_map, cloud_msg);
                cloud_msg.header.stamp = nh_->get_clock()->now();
                vm_.occ_inf_pub->publish(cloud_msg);
            }

            /* visualize ESDF Map*/
            if (cfg_.esdf_en) {
                /* ESDF 点云的发布已改到 publishESDF()，随地图帧率走，见 updateCallback() */

                // if (vm_.esdf_neg_pub->get_subscription_count() >= 1) {
                //     PointCloud pc;
                //     esdf_map_->getNegativeESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
                //     pcl::toROSMsg(pc, cloud_msg);
                //     cloud_msg.header.frame_id = cfg_.frame_id;
                //     cloud_msg.header.stamp = nh_->get_clock()->now();
                //     vm_.esdf_neg_pub->publish(cloud_msg);
                // }

#ifdef ESDF_MAP_DEBUG
        esdf_map_->getESDFOccPC2(box_min, box_max,cloud_msg);
        cloud_msg.header.stamp = nh_->get_clock()->now();
        vm_.esdf_occ_pub->publish(cloud_msg);
#endif
            }


            /* Publish visualization range */
            visualization_msgs::msg::MarkerArray mkr_arr;
            visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, box_min, box_max,
                                 "Visualization Range", Color::Purple());
            visualizeText(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, "Visualization Range Text",
                          "Visualization Range", box_max + Vec3f(0, 0, 0.5),
                          Color::Purple(), 0.6, 0);

            /* Publish local map range */
            Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
            boundBoxByLocalMap(local_map_min, local_map_max);
            visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, local_map_min,
                                 local_map_max, "Local Map Range", Color::Orange());
            visualizeText(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, "Local Map Range Text",
                          "Local Map Range", local_map_max + Vec3f(0, 0, 1.0),
                          Color::Orange(), 0.6, 0);

            /* Publish Ray-casting range */
            visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id,
                                 raycast_data_.cache_box_min, raycast_data_.cache_box_max,
                                 "Updating Range", Color::Green());
            visualizeText(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, "Updating Range Text",
                          "Updating Range", raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
                          Color::Green(), 0.6, 0);

            /* Publish Local map origin */
            visualizePoint(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, local_map_origin_d_,
                           Color::Red(), "Local Map Origin", 0.2, 0);

            if (cfg_.esdf_en) {
                Vec3f esdf_box_max, esdf_box_min;
                esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
                visualizeText(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, "ESDF Map Text",
                              "ESDF Map", esdf_box_max + Vec3f(0, 0, 1.0),
                              Color::Blue(), 0.6, 0);
                visualizeBoundingBox(mkr_arr, nh_->get_clock()->now().seconds(), cfg_.frame_id, esdf_box_min,
                                     esdf_box_max, "ESDF Updating Range", Color::Blue());
            }

            vm_.mkr_arr_pub->publish(mkr_arr);
        }

        /* 发布 2D 投影：每生成一帧地图（updateProbMap 产出一帧新的 Field2D）就发一次，
           频率 = 建图帧率，不再受 viz_timer / visualization.time_rate 限制。
           由 updateCallback() 在建图线程里直接调用，和 buildField2D() 串行，
           因此不会出现"读到正被改写的 field_"的竞争。 */
        void publishProj2D() {
            if (!cfg_.projection.enable || !cfg_.esdf_en) {
                return;     // 不满足时 buildField2D() 也不会跑，没有新场可发
            }
            // visualization.enable = false 时下面两个发布者根本没创建，必须判空
            if (!vm_.proj2d_pub || !vm_.proj2d_sdf_pub) {
                return;
            }

            const Field2D &f = getField2D();
            if (f.version() == vm_.published_proj2d_version) {
                return;     // 这一帧没有产出新的 2D 场（如 batch 攒帧、odom 越界提前 return）
            }
            const int w = f.width(), h = f.height();
            if (w <= 0 || h <= 0) {
                return;
            }

            if (!cfg_.projection.output_esdf) {
                /* 2D 栅格占用图 */
                if (vm_.proj2d_pub->get_subscription_count() < 1) {
                    return;
                }
                nav_msgs::msg::OccupancyGrid grid;
                grid.header.frame_id = cfg_.frame_id;
                grid.header.stamp = nh_->get_clock()->now();
                grid.info.resolution = f.resolution();
                grid.info.width = w;
                grid.info.height = h;
                grid.info.origin.position.x = f.origin().x();
                grid.info.origin.position.y = f.origin().y();
                grid.info.origin.orientation.w = 1.0;

                grid.data.resize(static_cast<size_t>(w) * h);
                const auto &d = f.distances();
                const auto &occ = f.occupied();
                const auto &unk = f.unknown();
                for (size_t i = 0; i < d.size(); ++i) {
                    // -1 未知 / 0 空闲 / 100 致命障碍；中间给渐变代价，方便 costmap 直接用
                    if (unk[i]) {
                        grid.data[i] = -1;              // 该柱在 [z_min, z_max] 内无有效采样
                    } else if (d[i] >= cfg_.projection.max_distance - 1e-6) {
                        grid.data[i] = 0;               // 远离开阔区
                    } else if (occ[i]) {
                        grid.data[i] = 100;             // 致命
                    } else {
                        // 距离 < 1 m 的区域给递增代价（0~99），给局部规划器留安全边界
                        const double t = std::clamp(1.0 - d[i], 0.0, 1.0);
                        grid.data[i] = static_cast<int8_t>(t * 0.0);
                    }
                }
                vm_.proj2d_pub->publish(grid);
            } else {
                /* 2D 距离场 */
                if (vm_.proj2d_sdf_pub->get_subscription_count() < 1) {
                    return;
                }
                const double res = f.resolution();
                const auto &d = f.distances();
                pcl::PointCloud<pcl::PointXYZI> pc;
                pc.reserve(static_cast<size_t>(w) * h);
                for (int j = 0; j < h; ++j) {
                    for (int i = 0; i < w; ++i) {
                        pcl::PointXYZI pt;
                        pt.x = f.origin().x() + (i + 0.5) * res;
                        pt.y = f.origin().y() + (j + 0.5) * res;
                        pt.z = robot_state_.p.z();      // 画在车的高度，好和 occ 对比
                        pt.intensity = d[static_cast<size_t>(j) * w + i];   // rviz 按 intensity 上色
                        pc.push_back(pt);
                    }
                }
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(pc, msg);
                msg.header.frame_id = cfg_.frame_id;
                msg.header.stamp = nh_->get_clock()->now();
                vm_.proj2d_sdf_pub->publish(msg);
            }

            vm_.published_proj2d_version = f.version();
        }

        /* 发布 ESDF 点云：同样"每生成一帧地图发一次"，频率 = 建图帧率。
           ESDF 数据在 updateESDF3D() 里刷新，而它在每次 updateProbMap() 中都是
           无条件执行的，所以地图帧号就是 ESDF 的帧号。 */
        void publishESDF() {
            if (!cfg_.esdf_en) {
                return;
            }
            // visualization.enable = false 时发布者没创建，必须判空
            if (!vm_.esdf_pub) {
                return;
            }
            const uint64_t frame_cnt = mapFrameCount();
            if (frame_cnt == vm_.published_esdf_frame_count) {
                return;     // 这一帧没有新建图
            }
            if (vm_.esdf_pub->get_subscription_count() < 1) {
                return;
            }

            // 与 vizCallback 里一致的可视化范围：以机器人为中心的 visualization_range
            Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
            Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;
            boundBoxByLocalMap(box_min, box_max);
            if ((box_max - box_min).minCoeff() <= 0) {
                return;     // visualization_range 某一维 <= 0
            }

            PointCloud pc;
            esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(pc, cloud_msg);
            cloud_msg.header.frame_id = cfg_.frame_id;
            cloud_msg.header.stamp = nh_->get_clock()->now();
            vm_.esdf_pub->publish(cloud_msg);

            vm_.published_esdf_frame_count = frame_cnt;
        }

        void vecEVec3fToPC2(const vec_E<Vec3f>& points, sensor_msgs::msg::PointCloud2& cloud) {
            // 设置header信息
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl_cloud.resize(points.size());
            for (long unsigned int i = 0; i < points.size(); i++) {
                pcl_cloud[i].x = static_cast<float>(points[i][0]);
                pcl_cloud[i].y = static_cast<float>(points[i][1]);
                pcl_cloud[i].z = static_cast<float>(points[i][2]);
            }
            pcl::toROSMsg(pcl_cloud, cloud);
            cloud.header.stamp = nh_->get_clock()->now();
            cloud.header.frame_id = cfg_.frame_id;
        }

    public:
        typedef shared_ptr<ROGMapROS> Ptr;

        ROGMapROS(const rclcpp::Node::SharedPtr nh, const std::string& cfg_path): nh_(nh) {
            // TODO: The current implementation uses a lenient QoS configuration for message transmission.
            const rclcpp::QoS qos(rclcpp::QoS(1)
                                  .best_effort()
                                  .keep_last(1)
                                  .durability_volatile());
            //besteffort->reliable
            const rclcpp::QoS pub_qos(rclcpp::QoS(1)
                                      .reliable()
                                      .keep_last(1)
                                      .durability_volatile());

            cfg_ = rog_map::Config(cfg_path);
            // 创建 TransformBroadcaster
            br_map_ego_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);

            init();
            /// Initialize visualization module
            if (cfg_.visualization_en) {
                vm_.occ_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/occ", pub_qos);
                vm_.unknown_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/unk", pub_qos);
                vm_.occ_inf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", pub_qos);
                vm_.unknown_inf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_unk", pub_qos);

                if (cfg_.frontier_extraction_en) {
                    vm_.frontier_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/frontier", pub_qos);
                }

                if (cfg_.esdf_en) {
                    vm_.esdf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf", pub_qos);
                    // vm_.esdf_neg_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf/neg", pub_qos);
                    // vm_.esdf_occ_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf/occ", pub_qos);
                }

                if (cfg_.projection.enable) {
                    vm_.proj2d_pub = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("rog_map/proj2d", pub_qos);
                    vm_.proj2d_sdf_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/proj2d_sdf", pub_qos);
                }

                if (cfg_.viz_time_rate > 0) {
                    const int cbk_dt_ms = static_cast<int>(1.0 / cfg_.viz_time_rate * 1000);
                    vm_.viz_reen_cbk_group = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                    vm_.viz_timer = nh_->create_wall_timer(
                        std::chrono::milliseconds(cbk_dt_ms),
                        std::bind(&ROGMapROS::vizCallback, this),
                        vm_.viz_reen_cbk_group
                    );
                }
            }

            vm_.mkr_arr_pub = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("rog_map/map_bound", pub_qos);

            if (cfg_.ros_callback_en) {
                rc_.odom_me_cbk_group = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rc_.cloud_me_cbk_group = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions so;
                so.callback_group = rc_.odom_me_cbk_group;
                rc_.odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(
                    cfg_.odom_topic, qos, std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
                so.callback_group = rc_.cloud_me_cbk_group;
                rc_.cloud_sub = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cfg_.cloud_topic, qos, std::bind(&ROGMapROS::cloudCallback, this, std::placeholders::_1), so);
                rc_.update_cbk_group = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rc_.update_timer = nh_->create_wall_timer(
                    std::chrono::milliseconds(1), // 0.001秒，即1毫秒
                    std::bind(&ROGMapROS::updateCallback, this),
                    rc_.update_cbk_group
                );
            }
        }

    private:
        static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray& mkrarr,
                                         const double& stamp,
                                         const string& frame_id,
                                         const Vec3f& box_min,
                                         const Vec3f& box_max,
                                         const string& ns,
                                         const Color& color,
                                         const double& size_x = 0.1,
                                         const double& alpha = 1.0,
                                         const bool& print_ns = true) {
            Vec3f size = (box_max - box_min) / 2;
            Vec3f vis_pos_world = (box_min + box_max) / 2;
            double width = size.x();
            double length = size.y();
            double hight = size.z();

            //Publish Bounding box
            int id = 0;
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.stamp = rclcpp::Time(stamp);
            line_strip.header.frame_id = frame_id;
            line_strip.action = visualization_msgs::msg::Marker::ADD;
            line_strip.ns = ns;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = id++; //unique id, useful when multiple markers exist.
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP; //marker type
            line_strip.scale.x = size_x;


            line_strip.color = color;
            line_strip.color.a = alpha; //不透明度，设0则全透明
            geometry_msgs::msg::Point p[8];

            //vis_pos_world是目标物的坐标
            p[0].x = vis_pos_world(0) - width;
            p[0].y = vis_pos_world(1) + length;
            p[0].z = vis_pos_world(2) + hight;
            p[1].x = vis_pos_world(0) - width;
            p[1].y = vis_pos_world(1) - length;
            p[1].z = vis_pos_world(2) + hight;
            p[2].x = vis_pos_world(0) - width;
            p[2].y = vis_pos_world(1) - length;
            p[2].z = vis_pos_world(2) - hight;
            p[3].x = vis_pos_world(0) - width;
            p[3].y = vis_pos_world(1) + length;
            p[3].z = vis_pos_world(2) - hight;
            p[4].x = vis_pos_world(0) + width;
            p[4].y = vis_pos_world(1) + length;
            p[4].z = vis_pos_world(2) - hight;
            p[5].x = vis_pos_world(0) + width;
            p[5].y = vis_pos_world(1) - length;
            p[5].z = vis_pos_world(2) - hight;
            p[6].x = vis_pos_world(0) + width;
            p[6].y = vis_pos_world(1) - length;
            p[6].z = vis_pos_world(2) + hight;
            p[7].x = vis_pos_world(0) + width;
            p[7].y = vis_pos_world(1) + length;
            p[7].z = vis_pos_world(2) + hight;
            //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
            for (int i = 0; i < 8; i++) {
                line_strip.points.push_back(p[i]);
            }
            //为了保证矩形框的八条边都存在：
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[3]);
            line_strip.points.push_back(p[2]);
            line_strip.points.push_back(p[5]);
            line_strip.points.push_back(p[6]);
            line_strip.points.push_back(p[1]);
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[7]);
            line_strip.points.push_back(p[4]);
            mkrarr.markers.push_back(line_strip);
        }

        static void visualizeText(visualization_msgs::msg::MarkerArray& mkr_arr,
                                  const double& stamp,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  const std::string& text,
                                  const Vec3f& position,
                                  const Color& c = Color::White(),
                                  const double& size = 0.6,
                                  const int& id = -1) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = rclcpp::Time(stamp);
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns.c_str();
            if (id >= 0) {
                marker.id = id;
            }
            else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color = c;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        };

        static void visualizePoint(visualization_msgs::msg::MarkerArray& mkr_arr,
                                   const double& stamp,
                                   const std::string& frame_id,
                                   const Vec3f& pt,
                                   Color color = Color::Pink(),
                                   std::string ns = "pt",
                                   double size = 0.1, int id = -1,
                                   const bool& print_ns = true) {
            visualization_msgs::msg::Marker marker_ball;
            static int cnt = 0;
            Vec3f cur_pos = pt;
            if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
                return;
            }
            marker_ball.header.frame_id = frame_id;
            marker_ball.header.stamp = rclcpp::Time(stamp);
            marker_ball.ns = ns.c_str();
            marker_ball.id = id >= 0 ? id : cnt++;
            marker_ball.action = visualization_msgs::msg::Marker::ADD;
            marker_ball.pose.orientation.w = 1.0;
            marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
            marker_ball.scale.x = size;
            marker_ball.scale.y = size;
            marker_ball.scale.z = size;
            marker_ball.color = color;

            geometry_msgs::msg::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();

            marker_ball.pose.position = p;
            mkr_arr.markers.push_back(marker_ball);

            // add test
            if (print_ns) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = frame_id;
                marker.header.stamp = rclcpp::Time(stamp);
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.ns = ns + "_text";
                if (id >= 0) {
                    marker.id = id;
                }
                else {
                    static int id = 0;
                    marker.id = id++;
                }
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.scale.z = 0.6;
                marker.color = color;
                marker.text = ns;
                marker.pose.position.x = cur_pos.x();
                marker.pose.position.y = cur_pos.y();
                marker.pose.position.z = cur_pos.z() + 0.5;
                marker.pose.orientation.w = 1.0;
                mkr_arr.markers.push_back(marker);
            }
        }
    };
}
#endif // ROG_MAP_ROS_HPP
#endif // USE_ROS2
