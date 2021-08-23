#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <folly/Synchronized.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Int32MultiArray.h>

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStart(const geometry_msgs::Pose &start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal(const geometry_msgs::Pose &goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan();

  static geometry_msgs::PoseArray path_interpolate_and_fix_orientation(
      const geometry_msgs::PoseArray &trajectory,
      const float &path_density);

  geometry_msgs::PoseArray path_clip(const geometry_msgs::PoseArray &path,
                                     const geometry_msgs::Pose &vehicle_pose);

  bool get_vehicle_pose(geometry_msgs::Pose &vehicle_pose);

  static geometry_msgs::PoseArray convert_path_to_pose_array(const nav_msgs::Path &path_to_convert);

  void calculate_footprint_base();

 private:
  /// The node handle
  ros::NodeHandle n;
  /// A publisher publishing the start position for RViz
  ros::Publisher pubStart;
  /// A subscriber for receiving map updates
  ros::Subscriber subMap;
  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;
  /// A listener that awaits transforms
  tf::TransformListener listener;
  /// A transform for moving start positions
  tf::StampedTransform transform;
  /// The path produced by the hybrid A* algorithm
  Path path;
  /// Global path
  geometry_msgs::PoseArray path_global;
  /// Vehicle footprint for extra stable hybrid A* collision check
  geometry_msgs::Polygon footprint_base;
  /// Vehicle footprint with safety margins for extra stable hybrid A* collision check
  geometry_msgs::Polygon footprint_with_safety_base;
  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  Visualize visualization;
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::PoseStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double grid_length_x_;
  double grid_length_y_;

  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  float *dubinsLookup =
      new float[Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

  // Sync subscribers
  using SubOccGrid = message_filters::Subscriber<nav_msgs::OccupancyGrid>;
  using SubPoseStamped = message_filters::Subscriber<geometry_msgs::PoseStamped>;

  using SyncPolicy = message_filters::sync_policies::ExactTime<nav_msgs::OccupancyGrid,
                                                               geometry_msgs::PoseStamped,
                                                               geometry_msgs::PoseStamped>;

  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::shared_ptr<SubOccGrid> sub_occ_grid_;
  std::shared_ptr<SubPoseStamped> sub_pose_stamped_start_;
  std::shared_ptr<SubPoseStamped> sub_pose_stamped_goal_;

  std::shared_ptr<Synchronizer> synchronizer_;

  std::shared_ptr<ros::Subscriber> sub_grid_map_;

  void callback_synchronizer(
      const nav_msgs::OccupancyGrid::ConstPtr &msg_occ_grid,
      const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_start,
      const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal);

  folly::Synchronized<grid_map::GridMap> sync_grid_map_;
  void callback_grid_map(const grid_map_msgs::GridMap::ConstPtr &msg_grid_map);

  static std_msgs::Int32MultiArray do_collision_check_on_grid_frame(const geometry_msgs::PoseArray &trajectory_on_grid_map,
                                                                    const grid_map::GridMap &grid_map,
                                                                    const geometry_msgs::Polygon &vehicle_polygon);

  bool plan(const nav_msgs::OccupancyGrid::Ptr &occupancy_grid,
            const geometry_msgs::Pose &msg_pose_stamped_start,
            const geometry_msgs::Pose &msg_pose_stamped_goal,
            geometry_msgs::PoseArray &path_planned);

//  static geometry_msgs::PoseArray transform_pose_array_to_base_link(const geometry_msgs::PoseArray &pose_array,
//                                                                    const geometry_msgs::TransformStamped &transform);
//
//  static Eigen::Matrix4d create_transform_matrix(const geometry_msgs::TransformStamped &transform);;

  geometry_msgs::PoseArray transform_pose_array(const geometry_msgs::PoseArray &trajectory,
                                                const std::string &target_frame_id);

  geometry_msgs::Pose transform_pose(const geometry_msgs::Pose &pose,
                                     const std::string &source_frame_id,
                                     const std::string &target_frame_id);

  bool get_affine(Eigen::Affine3f &affine,
                  const std::string &frame_source,
                  const std::string &frame_target,
                  const ros::Time &time);

  geometry_msgs::PoseArray transform_grid2hybrid(const geometry_msgs::PoseArray &pose_array) const;

  geometry_msgs::Pose transform_grid2hybrid(const geometry_msgs::Pose &pose) const;

  geometry_msgs::PoseArray transform_hybrid2grid(const geometry_msgs::PoseArray &pose_array) const;

  geometry_msgs::Pose transform_hybrid2grid(const geometry_msgs::Pose &pose) const;

  geometry_msgs::PoseArray transform_map2hybrid(const geometry_msgs::PoseArray &pose_array);

  geometry_msgs::Pose transform_map2hybrid(const geometry_msgs::Pose &pose);

  geometry_msgs::PoseArray transform_hybrid2map(const geometry_msgs::PoseArray &pose_array);

  geometry_msgs::Pose transform_hybrid2map(const geometry_msgs::Pose &pose);

};
}
#endif // PLANNER_H
