/**
 * @file r1_dummy_map_node.cpp
 * @author OpenAI
 * @brief /initialpose を受けて map -> odom を更新するダミー localization ノード
 * @version 0.1
 * @date 2026-04-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <chrono>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  /**
   * @brief ダミー map ノードを初期化する。
   *
   * `/initialpose` から `map -> odom` を再計算するために、
   * TF listener / broadcaster と publish timer を生成する。
   */
  MyNode() : Node("r1_dummy_map_node")
  {
    this->declare_parameter<double>("publish_rate", 100.0);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    publish_rate_ = this->get_parameter("publish_rate").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    initialpose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&MyNode::initialpose_callback, this, std::placeholders::_1));

    // 起動直後は map と odom を一致させておく。
    map_to_odom_.header.frame_id = map_frame_;
    map_to_odom_.child_frame_id = odom_frame_;
    map_to_odom_.transform.rotation.w = 1.0;

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_), std::bind(&MyNode::timer_callback, this));
  }

private:
  /**
   * @brief `/initialpose` を受けて `map -> odom` を更新する。
   *
   * `/initialpose` は `map` 座標系での目標 `base_link` 姿勢として扱う。
   * 現在の `odom -> base_link` と組み合わせ、整合する `map -> odom`
   * を再計算して内部状態へ保存する。
   *
   * @param msg `map` 座標系で与えられた目標自己位置。
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // このノードでは /initialpose を「map 座標系で見た base_link の目標姿勢」として扱う。
    // つまり /initialpose を受けた瞬間に base_link を直接動かすのではなく、
    // その姿勢に見えるよう map->odom を更新する。
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        this->get_logger(), "Ignoring /initialpose in frame '%s'. expected '%s'.",
        msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    // まず現在の odom->base_link を取得する。
    // ここで得られるのは「ローカル座標系 odom から見た現在のロボット姿勢」。
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
      odom_to_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to lookup %s -> %s for /initialpose: %s", odom_frame_.c_str(),
        base_frame_.c_str(), ex.what());
      return;
    }

    // /initialpose の pose を map->base_link の目標姿勢として tf2::Transform に変換する。
    // covariance は使わず、位置と姿勢だけを利用する。
    tf2::Transform tf_map_to_base;
    tf_map_to_base.setOrigin(
      tf2::Vector3(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf2::Quaternion q_map_to_base;
    tf2::fromMsg(msg->pose.pose.orientation, q_map_to_base);
    q_map_to_base.normalize();
    tf_map_to_base.setRotation(q_map_to_base);

    tf2::Transform tf_odom_to_base;
    tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);

    // ここで欲しいのは map->odom。
    // 現在の odom->base_link は dummy odometry が持っているので、
    // 「map から見たロボットを /initialpose の場所にしたい」という条件
    //
    //   map->base_link_target = map->odom * odom->base_link_current
    //
    // を満たす map->odom を解く。
    //
    // よって
    //
    //   map->odom = map->base_link_target * inverse(odom->base_link_current)
    //
    // となる。これにより、odom 自体は据え置いたまま、
    // map から見たロボット位置だけを /initialpose に合わせられる。
    const tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse();

    // 計算結果を保持し、この後は timer_callback() が継続的に map->odom を publish する。
    map_to_odom_.header.frame_id = map_frame_;
    map_to_odom_.child_frame_id = odom_frame_;
    map_to_odom_.transform = tf2::toMsg(tf_map_to_odom);

    const double yaw = tf2::getYaw(map_to_odom_.transform.rotation);
    RCLCPP_INFO(
      this->get_logger(), "Updated map->odom from /initialpose: x=%.3f, y=%.3f, yaw=%.3f",
      map_to_odom_.transform.translation.x, map_to_odom_.transform.translation.y, yaw);
  }

  /**
   * @brief 現在保持している `map -> odom` を定期 publish する。
   */
  void timer_callback()
  {
    map_to_odom_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(map_to_odom_);
  }

  double publish_rate_ = 100.0;
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
};

/**
 * @brief `r1_dummy_map_node` のエントリーポイント。
 *
 * @param argc コマンドライン引数の数。
 * @param argv コマンドライン引数の配列。
 * @return 終了コード。
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
