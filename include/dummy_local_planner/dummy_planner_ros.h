#ifndef DUMMY_LOCAL_PLANNER_DUMMY_PLANNER_ROS_H_
#define DUMMY_LOCAL_PLANNER_DUMMY_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <dummy_local_planner/DummyPlannerConfig.h>

#include <angles/angles.h>  // not sure is this needed here?

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dummy_local_planner/dummy_planner.h>

namespace dummy_local_planner {
  class DummyPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DummyPlannerROS wrapper
       */
      DummyPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DummyPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }

    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DummyPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DummyPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<DummyPlannerConfig> *dsrv_;
      dummy_local_planner::DummyPlannerConfig default_config_;
      bool setup_;
      tf::Stamped<tf::Pose> current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
      template <typename T> int sgn(T val) {
         return (T(0) < val) - (val < T(0));
      }
      bool always_true(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples) {
        return true;
      }
  };  // end class DUMMYPlannerROS
};  // end namespace dummy_local_planner

#endif
