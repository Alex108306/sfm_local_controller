
#include <sfm_local_controller/collision_checker.hpp>

namespace Upo {
namespace Navigation {
namespace sfm_controller {

CollisionChecker::CollisionChecker(rclcpp::Node::SharedPtr &n, std::shared_ptr<tf2_ros::Buffer> &tf,
                                   double max_lv, double max_av, double lin_acc,
                                   double ang_acc, double r_radius,
                                   std::string base_frame,
                                   std::string planner_frame)
    : nh_(n), tf_(tf), max_lin_vel_(max_lv), max_ang_vel_(max_av),
      max_lin_acc_(lin_acc), max_ang_acc_(ang_acc), robot_radius_(r_radius),
      robot_frame_(base_frame), planner_frame_(planner_frame) {

  robot_radius_aug_ = robot_radius_;

  use_laser_ = true;
  use_range_ = false;
  num_ranges_ = 0;
  ranges_ready_ = false;

  setup();
}

CollisionChecker::~CollisionChecker() {}

void CollisionChecker::setup() {

  double uncertainty;
  nh_->get_parameter_or("sensor_uncertainty", uncertainty, 0.05);
  robot_radius_aug_ = robot_radius_ + uncertainty;

  nh_->get_parameter_or("use_laser", use_laser_, true);
  std::string laser_topic;
  if (use_laser_) {
    nh_->get_parameter_or("laser_topic", laser_topic, std::string("scan"));
    laser_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic, rclcpp::QoS(1), std::bind(&CollisionChecker::laserCallback, this, std::placeholders::_1));
  }

  unsigned int i = 0;
  nh_->get_parameter_or("use_range", use_range_, false);
  if (use_range_) {
    bool ok = true;
    while (ok) {
      char buf[25];
      std::sprintf(buf, "range_topic_%u", i);
      std::string st = std::string(buf);

      if (nh_->has_parameter(st)) {
        std::string rt;
        nh_->get_parameter(st, rt);
        range_topics_.push_back(rt);
        // ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>(rt.c_str(),
        // 10, &CollisionDetection::rangeCallback, this);
        sub = nh_->create_subscription<sensor_msgs::msg::Range>(
            rt, rclcpp::QoS(1),
            std::bind(&CollisionChecker::rangeCallback, this, std::placeholders::_1));
        range_subscribers_.push_back(sub);
        RCLCPP_INFO(nh_->get_logger(), "%s. subscribed to topic: %s\n",
               nh_->get_name(), rt.c_str());
        i++;
      } else
        ok = false;
    }
    num_ranges_ = static_cast<int>i;
    ranges_initiated_.assign(num_ranges_, false);
    // range_frames_.assign(num_ranges_, "");
  }
}

void CollisionChecker::laserCallback(
    const sensor_msgs::msg::LaserScan::ConstPtr &msg) {
  RCLCPP_INFO_ONCE(nh_->getlogger(), "CollisionChecker: Laser received!");
  // IMPORTANT: the frame of the laser should be the center of the robot
  // (base_link) Otherwise we should include a shift to the center in the
  // calculations.

  std::vector<geometry_msgs::msg::Point> points = laser_polar2euclidean(msg.get());
  if (msg->header.frame_id != robot_frame_)
    transformPoints(points, msg->header.frame_id);

  laser_mutex_.lock();
  scanpoints_ = points;
  laser_mutex_.unlock();
}

std::vector<geometry_msgs::msg::Point>
CollisionChecker::laser_polar2euclidean(const sensor_msgs::msg::LaserScan *scan) {
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(scan->ranges.size());
  geometry_msgs::msg::Point p;
  p.z = 0.0;
  for (unsigned int i = 0; i < scan->ranges.size(); i++) {
    // laser measure polar coordinates
    float laser_d = scan->ranges[i];
    float laser_th = scan->angle_min + scan->angle_increment * i;
    // transform to x,y
    p.x = laser_d * cos(laser_th);
    p.y = laser_d * sin(laser_th);
    points.push_back(p);
  }
  return points;
}

void CollisionChecker::transformPoints(
    std::vector<geometry_msgs::msg::Point> &points, std::string input_frame) {

  // Transform points to
  // robot_frame_ if necessary
  geometry_msgs::msg::PointStamped out;
  for (unsigned int i = 0; i < points.size(); i++) {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = input_frame;
    in.header.stamp = tf2::TimePointZero;
    in.point.x = points[i].x;
    in.point.y = points[i].y;
    in.point.z = points[i].z;
    try {
      out = tf_->transform(in, robot_frame_);
      points[i].x = out.point.x;
      points[i].y = out.point.y;
      points[i].z = out.point.z;

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(nh_->get_logger(), "[%s] Could NOT transform laser point to %s: %s",
                    nh_->get_name(), robot_frame_.c_str(), ex.what());
      return;
    }
  }
}

void CollisionChecker::rangeCallback(const sensor_msgs::msg::Range::ConstPtr &msg) {
  RCLCPP_INFO_ONCE(nh_->get_logger(),
      "Collision detector: range received! Detecting configuration...");
  if (!ranges_ready_) {
    if (range_frames_.size() != (unsigned int)num_ranges_) {
      bool found = false;
      for (unsigned int i = 0; i < range_frames_.size(); i++) {
        if (range_frames_[i].compare(msg->header.frame_id) == 0) {
          found = true;
          break;
        }
      }
      if (!found) {
        range_frames_.push_back(msg->header.frame_id);

        range r;
        r.range = msg->range;
        r.id = msg->header.frame_id;
        r.min_dist = msg->min_range;
        r.max_dist = msg->max_range;
        r.fov = msg->field_of_view;
        // listen to the TF to know the position of the sonar range
        // tf::StampedTransform transform;
        geometry_msgs::msg::TransformStamped ts;
        try {
          // tf_->waitForTransform(base_frame_.c_str(), r.id.c_str(),
          // ros::Time(0),
          //                      ros::Duration(3.0));
          // tf_->lookupTransform(base_frame_.c_str(), r.id.c_str(),
          // ros::Time(0), transform);
          ts = tf_->lookupTransform(robot_frame_, r.id,
                                    tf2::TimePointZero, tf2::durationFromSec(3.0));
        } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(nh_->getlogger(), "%s", ex.what());
        }
        float x = ts.transform.translation.x;
        float y = ts.transform.translation.y;
        r.polar_dist = sqrt(x * x + y * y);
        // tf::Matrix3x3 m(ts.transform.rotation);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        tf2::Quaternion quat_tf;
        tf2::fromMsg(ts.transform.rotation, quat_tf);
        r.polar_angle = tf2::impl::getYaw(quat_tf);

        range_mutex_.lock();
        ranges_.push_back(r);
        range_mutex_.unlock();
        RCLCPP_INFO(nh_->getlogger(), "Collision detector: Range %s configured.\n",
               msg->header.frame_id.c_str());
      }
    } else {
      ranges_ready_ = true;
      RCLCPP_INFO(nh_->getlogger(), "Collision detector: all the range sensors configured!!!\n\n");
    }
  } else {
    range_mutex_.lock();
    for (int i = 0; i < (int)ranges_.size(); i++) {
      if (ranges_[i].id.compare(msg->header.frame_id) == 0) {
        ranges_[i].range = msg->range;
        break;
      }
    }
    range_mutex_.unlock();
  }
}

/*
void CollisionChecker::saturateVelocities(geometry_msgs::Twist *twist) {
  float lv = twist->linear.x;
  float av = twist->angular.z;

  float rvx = robot_vel_.getOrigin().getX();
  float rvy = robot_vel_.getOrigin().getY();
  float rvt = tf::getYaw(robot_vel_.getRotation());

  // acc linear
  if (fabs(rvx - lv) > max_lv_var_) {
    lv = (lv < rvx) ? (rvx - max_lv_var_) : (rvx + max_lv_var_);
  }
  // acc angular
  if (fabs(rvt - av) > max_av_var_) {
    av = (av < rvt) ? (rvt - max_av_var_) : (rvt + max_av_var_);
  }

  // Check maximum velocities
  if (lv > max_lin_vel_)
    lv = max_lin_vel_;
  else if (lv < (-max_lin_vel_))
    lv = max_lin_vel_ * (-1);

  if (av > max_ang_vel_)
    av = max_ang_vel_;
  else if (av < (-max_ang_vel_))
    av = max_ang_vel_ * (-1);

  twist->linear.x = lv;
  twist->angular.z = av;
}
*/

bool CollisionChecker::inCollision(
    float x, float y, const std::vector<geometry_msgs::msg::Point> &scanpoints) {
  if (use_range_ && inRangeCollision(x, y)) {
    RCLCPP_INFO(nh_->getlogger(), "Possible collision detected!");
    return true;
  }
  if (use_laser_ && inLaserCollision(x, y, scanpoints)) {
    return true;
  }
  return false;
}

bool CollisionChecker::inRangeCollision(float x, float y) {
  range_mutex_.lock();
  for (unsigned int i = 0; i < ranges_.size(); i++) {
    // Main point
    float rx = (ranges_[i].polar_dist + ranges_[i].range) *
               cos(ranges_[i].polar_angle);
    float ry = (ranges_[i].polar_dist + ranges_[i].range) *
               sin(ranges_[i].polar_angle);
    float dx = (x - rx);
    float dy = (y - ry);
    float dist = dx * dx + dy * dy;
    if (dist <= (robot_radius_aug_ * robot_radius_aug_)) {
      RCLCPP_INFO(nh_->getlogger(), "POSSIBLE COLLISION DETECTED IN FRAME: %s",
               ranges_[i].id.c_str());
      range_mutex_.unlock();
      return true;
    }

    if (ranges_[i].fov > 0.2) {
      // second point
      rx = (ranges_[i].polar_dist + ranges_[i].range) *
           cos(ranges_[i].polar_angle + (ranges_[i].fov / 2.0));
      ry = (ranges_[i].polar_dist + ranges_[i].range) *
           sin(ranges_[i].polar_angle + (ranges_[i].fov / 2.0));
      dx = (x - rx);
      dy = (y - ry);
      dist = dx * dx + dy * dy;
      if (dist <= (robot_radius_aug_ * robot_radius_aug_)) {
        RCLCPP_INFO(nh_->getlogger(), "POSSIBLE COLLISION DETECTED (p2) IN FRAME: %s",
                 ranges_[i].id.c_str());
        range_mutex_.unlock();
        return true;
      }

      // third point
      rx = (ranges_[i].polar_dist + ranges_[i].range) *
           cos(ranges_[i].polar_angle - (ranges_[i].fov / 2.0));
      ry = (ranges_[i].polar_dist + ranges_[i].range) *
           sin(ranges_[i].polar_angle - (ranges_[i].fov / 2.0));
      dx = (x - rx);
      dy = (y - ry);
      dist = dx * dx + dy * dy;
      if (dist <= (robot_radius_aug_ * robot_radius_aug_)) {
        RCLCPP_INFO(nh_->getlogger(), "POSSIBLE COLLISION DETECTED (p3) IN FRAME: %s",
                 ranges_[i].id.c_str());
        range_mutex_.unlock();
        return true;
      }
    }
  }
  range_mutex_.unlock();
  return false;
}

bool CollisionChecker::inLaserCollision(
    float x, float y, const std::vector<geometry_msgs::msg::Point> &scanpoints) {
  for (unsigned int i = 0; i < scanpoints.size(); i++) {
    float dx = (x - scanpoints[i].x);
    float dy = (y - scanpoints[i].y);
    // float dist = sqrt(dx*dx + dy*dy);
    // if(dist <= robot_radius_)
    float dist = dx * dx + dy * dy;
    if (dist <= (robot_radius_aug_ * robot_radius_aug_))
      return true;
  }
  return false;
}

/**
 * @brief  Generate and check a single trajectory
 * @param cvx The current x velocity of the robot
 * @param cvy The current y velocity of the robot
 * @param cvth The current angular velocity of the robot
 * @param tvx The x velocity used to seed the trajectory
 * @param tvy The y velocity used to seed the trajectory
 * @param tvth The theta velocity used to seed the trajectory
 * @param dt The time for project the movement
 * @return True if the trajectory is legal, false otherwise
 */
bool CollisionChecker::checkCommand(double cvx, double cvy, double cvth,
                                    double tvx, double tvy, double tvth,
                                    double dt) {

  // printf("\nCollisionDetection. CheckCommand with vels vx:%.2f, vy:%.2f
  // th:%.2f\n", tvx, tvy, tvth);

  // double max_lv = computeNewVelocity(tvx, cvx, max_lin_acc_, sim_time_);
  // float vel_mag = sqrt(max_lv * max_lv);
  // float steps = (vel_mag * sim_time_) / granularity_;
  // float dt = sim_time_ / steps;
  float x = 0.0, y = 0.0, th = 0.0;

  double lv = cvx;
  double av = cvth;

  // geometry_msgs::PoseStamped pose;
  // pose.header.stamp = ros::Time();                       // ros::Time::now();
  // pose.header.frame_id = features_->getRobotBaseFrame(); // base_frame_;

  std::vector<geometry_msgs::msg::Point> laser_points;
  if (use_laser_) {
    laser_mutex_.lock();
    laser_points = scanpoints_;
    laser_mutex_.unlock();
  }

  // if dt is higher than 0.1 secs, split it in steps
  params_mutex_.lock();
  double step = 0.1;
  int steps = std::ceil(dt / step);
  for (unsigned int i = 0; i < steps; i++) {
    lv = computeNewVelocity(tvx, lv, max_lin_acc_, step);
    av = computeNewVelocity(tvth, av, max_ang_acc_, step);

    // x = computeNewXPosition(x, lv, 0.0, av, step);
    // y = computeNewYPosition(y, lv, 0.0, av, step);
    // th = computeNewThetaPosition(th, av, step);

    float lin_dist = lv * step;
    th = th + (av * step);
    // normalization just in case
    th = normalizeAngle(th, -M_PI, M_PI);
    x = x + lin_dist * cos(th); // cos(th+av*dt/2.0)
    y = y + lin_dist * sin(th);

    if (inCollision(x, y, laser_points)) {
      params_mutex_.unlock();
      return false;
    }
  }

  // Now we check that we can stop without colliding
  while (lv > 0.0) {
    lv = computeNewVelocity(0.0, lv, max_lin_acc_, step);
    av = computeNewVelocity(0.0, av, max_ang_acc_, step);

    // x = computeNewXPosition(x, lv, 0.0, av, step);
    // y = computeNewYPosition(y, lv, 0.0, av, step);
    // th = computeNewThetaPosition(th, av, step);

    float lin_dist = lv * step;
    th = th + (av * step);
    // normalization just in case
    th = normalizeAngle(th, -M_PI, M_PI);
    x = x + lin_dist * cos(th); // cos(th+av*dt/2.0)
    y = y + lin_dist * sin(th);

    if (inCollision(x, y, laser_points)) {
      params_mutex_.unlock();
      return false;
    }
  }
  params_mutex_.unlock();
  return true;
}

} // namespace sfm_controller
} // namespace Navigation
} // namespace Upo
