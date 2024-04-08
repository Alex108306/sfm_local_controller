/**
 * Class to adapt the sensory input and detections to be used by the
 * sfm_local_controller
 *
 * Author: Noé Pérez-Higueras
 * Service Robotics Lab, Pablo de Olavide University 2021
 *
 * Software License Agreement (BSD License)
 *
 */

#ifndef SFMCONTROLLER_SENSOR_HPP_
#define SFMCONTROLLER_SENSOR_HPP_

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
// sensor input for obstacles
#include <sensor_msgs/msg/laser_scan.hpp>
// Detection input for social navigation
#include <dynamic_obstacle_detector/DynamicObstacles.h>
#include <people_msgs/msg/people.hpp>
// Social Force Model
#include <lightsfm/angle.hpp>
#include <lightsfm/sfm.hpp>
#include <lightsfm/vector2d.hpp>

#include <math.h>
#include <mutex>

namespace Upo {
namespace Navigation {
namespace sfm_controller {

class SFMSensorInterface {
public:
  /**
   * @brief  Default constructor
   * @param n pointer to a ros node handle to publish to topics
   * @param tfBuffer Pointer to tf2 buffer
   * @param robot_max_lin_speed the maximum robot linear speed [m/s]
   * @param robot_radius the radius of the robot circunference [m]
   * @param person_radius the approximated radius of the people body
   * @param robot_frame the coordinate frame of the robot base
   * @param odom_frame the coordinate frame of the robot odometry
   **/
  SFMSensorInterface(rclcpp::Node::SharedPtr &n, std::shared_ptr<tf2_ros::Buffer> &tfBuffer,
                     float robot_max_lin_speed, float robot_radius,
                     float person_radius, std::string robot_frame,
                     std::string odom_frame);

  /**
   * @brief  Destructor class
   */
  ~SFMSensorInterface();

  /**
   * @brief  Callback to process the laser scan sensory input.
   * @param laser laserScan message to be processed
   */
  void laserCb(const sensor_msgs::msg::LaserScan::SharedPtr laser);

  /**
   * @brief  Callback to process the people detected in the robot vecinity.
   * @param people message with the people to be processed
   */
  void peopleCb(const people_msgs::msg::People::SharedPtr people);

  /**
   * @brief  Callback to process the moving obstacles detected in the robot
   * vecinity.
   * @param obs messages with the obstacles to be processed.
   */
  void dynamicObsCb(
      const dynamic_obstacle_detector::DynamicObstacles::SharedPtr obs);

  /**
   * @brief  Callback to process the odometry messages with the robot movement.
   * @param odom messages with the obstacles to be processed.
   */
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr odom);

  /**
   * @brief  Tranform a coordinate vector from one frame to another
   * @param vector coordinate vector in the origin frame
   * @param from string with the name of the origin frame
   * @param to string with the name of the target frame
   * @return coordinate vector in the target frame
   */
  geometry_msgs::Vector3 transformVector(geometry_msgs::msg::Vector3 &vector,
                                         std::string from, std::string to);

  /**
   * @brief  returns the vector of sfm agents
   * @return agents vector
   */
  std::vector<sfm::Agent> getAgents();

  void setMaxLinVel(float max_lin_vel) {
    agents_mutex_.lock();
    agents_[0].desiredVelocity = max_lin_vel;
    agents_mutex_.unlock();
  }

private:
  /**
   * @brief  publish the transformed points in RViz
   * @param points vector with the coordinates of the points
   * @return none
   */
  void publish_obstacle_points(const std::vector<utils::Vector2d> &points);

  // void updateAgents();

  rclcpp::Node::SharedPtr nh_; // Pointer to the node node handle
  // ros::NodeHandle n_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // Pointer to the tfBuffer created in the node

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex odom_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::Subscription<dynamic_obstacle_detector::DynamicObstacles>::SharedPtr dyn_obs_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;

  std::vector<sfm::Agent> agents_; // 0: robot, 1..: Others
  // sfm::Agent robot_agent_;
  std::mutex agents_mutex_;
  std::vector<utils::Vector2d> obstacles_;

  bool laser_received_;
  rclcpp::Time last_laser_;

  people_msgs::msg::People people_;
  std::mutex people_mutex_;
  dynamic_obstacle_detector::DynamicObstacles dyn_obs_;
  std::mutex obs_mutex_;

  float person_radius_;
  float naive_goal_time_;
  float people_velocity_;

  float max_obstacle_dist_;

  std::string robot_frame_;
  std::string odom_frame_;

  // bool use_static_map_;
  // sfm::RosMap *static_map_;
};

} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo

#endif