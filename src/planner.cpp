#include "planner.h"
#include <grid_map_msgs/GridMap.h>

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  sub_occ_grid_ = std::make_shared<SubOccGrid>(n, "/occupy/costmap", 1);
  sub_pose_stamped_start_ = std::make_shared<SubPoseStamped>(n, "/occupy/pose_stamped_start_hybrid", 1);
  sub_pose_stamped_goal_ = std::make_shared<SubPoseStamped>(n, "/occupy/pose_stamped_goal_hybrid", 1);
  sub_transform_stamped_ = std::make_shared<SubTransformStamped>(n, "/occupy/transform_stamped_hybrid_to_base_link", 1);

  sub_grid_map_ = std::make_shared<ros::Subscriber>(
      n.subscribe("/occupy/grid_map_map_builder",
                  1,
                  &Planner::callback_grid_map,
                  this));

  synchronizer_ = std::make_shared<Synchronizer>(
      SyncPolicy(10),
      *sub_occ_grid_,
      *sub_pose_stamped_start_,
      *sub_pose_stamped_goal_,
      *sub_transform_stamped_);
  synchronizer_->registerCallback(
      boost::bind(
          &Planner::callback_synchronizer,
          this,
          boost::placeholders::_1,
          boost::placeholders::_2,
          boost::placeholders::_3,
          boost::placeholders::_4));
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

void Planner::callback_synchronizer(
    const nav_msgs::OccupancyGrid::ConstPtr &msg_occ_grid,
    const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_start,
    const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal,
    const geometry_msgs::TransformStamped::ConstPtr &msg_transform_stamped) {

  nav_msgs::OccupancyGrid::Ptr msg_occ_grid_ptr = boost::make_shared<nav_msgs::OccupancyGrid>();
  *msg_occ_grid_ptr = *msg_occ_grid;

  geometry_msgs::PoseArray path_planned;
  bool valid_plan;
  path_global.poses.clear();
  if (path_global.poses.empty()) {
    valid_plan = plan(msg_occ_grid_ptr,
                      msg_pose_stamped_start,
                      msg_pose_stamped_goal,
                      path_planned);
    if (valid_plan) {
      path_global = transform_pose_array_to_base_link(path_planned,
                                                      *msg_transform_stamped);
    }
  } else {
    for (int i = 0; i < path_global.poses.size(); ++i) {
      auto const &pose_c = path_global.poses.at(i);

    }
  }

  if (!path_global.poses.empty()) {
    auto path_interpolated_and_ori_fixed =
        path_interpolate_and_fix_orientation(path_global, HybridAStar::Constants::path_density);
    smoothedPath.publishPath(path_interpolated_and_ori_fixed);
  }
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool **binMap;
  binMap = new bool *[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseStamped::ConstPtr &initial) {
  float x = initial->pose.position.x / Constants::cellSize;
  float y = initial->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.position;
  startN.pose.orientation = initial->pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr &end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D *nodes3D = new Node3D[length]();
    Node2D *nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.position.x / Constants::cellSize;
    y = start.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D *nSolution = Algorithm::hybridAStar(nStart,
                                               nGoal,
                                               nodes3D,
                                               nodes2D,
                                               width,
                                               height,
                                               configurationSpace,
                                               dubinsLookup,
                                               visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
//    path.publishPath();
//    path.publishPathNodes();
//    path.publishPathVehicles();
//    smoothedPath.publishPath();
//    smoothedPath.publishPathNodes();
//    smoothedPath.publishPathVehicles();
//    visualization.publishNode3DCosts(nodes3D, width, height, depth);
//    visualization.publishNode2DCosts(nodes2D, width, height);

    delete[] nodes3D;
    delete[] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}

void Planner::callback_grid_map(const grid_map_msgs::GridMap::ConstPtr &msg_grid_map) {
  auto locked_grid_map = sync_grid_map_.wlock();
  *locked_grid_map = msg_grid_map;
}

int Planner::check_and_get_index_collision(const geometry_msgs::PoseArray &poses,
                                           const grid_map_msgs::GridMap::ConstPtr &grid_map,
                                           const geometry_msgs::Polygon &polygon_footprint) {
  return 0;
}

bool Planner::plan(const nav_msgs::OccupancyGrid::Ptr &occupancy_grid,
                   const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_start,
                   const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal,
                   geometry_msgs::PoseArray &path_planned) {

  setMap(occupancy_grid);
  setStart(msg_pose_stamped_start);
  setGoal(msg_pose_stamped_goal);
//  path.setTransformMatrix(*msg_transform_stamped);
//  smoothedPath.setTransformMatrix(*msg_transform_stamped);
  plan();
  if (smoothedPath.path.poses.empty()) {
    return false;
  } else {
    path_planned = convert_path_to_pose_array(smoothedPath.path);
    std::reverse(path_planned.poses.begin(), path_planned.poses.end());
    return true;
  }
}

geometry_msgs::PoseArray Planner::path_interpolate_and_fix_orientation(
    const geometry_msgs::PoseArray &trajectory,
    const float &path_density = 1.0f) {

  geometry_msgs::PoseArray trajectory_fixed;
  trajectory_fixed.header = trajectory.header;

  auto poses = trajectory.poses;
  if (poses.size() < 2 || path_density == 0) {
    return trajectory_fixed;
  }

  // Interpolate the path
  double d = 0, a = 0;
  double margin = path_density * 0.01;
  double remaining = 0;
  int nPoints = 0;

  trajectory_fixed.poses.push_back(poses.at(0));
  for (unsigned int si = 0, ei = 1; ei < poses.size();) {
    d += hypot(poses.at(ei).position.x - poses.at(ei - 1).position.x,
               poses.at(ei).position.y - poses.at(ei - 1).position.y) + remaining;
    a = atan2(poses.at(ei).position.y - poses.at(si).position.y,
              poses.at(ei).position.x - poses.at(si).position.x);

    if (d < path_density - margin) // skip
    {
      ei++;
      remaining = 0;
    } else if (d > (path_density + margin)) // skip
    {
      geometry_msgs::Pose pm = poses.at(si);
      nPoints = d / path_density;
      for (int k = 0; k < nPoints; k++) {
        pm.position.x = pm.position.x + path_density * cos(a);
        pm.position.y = pm.position.y + path_density * sin(a);
        trajectory_fixed.poses.push_back(pm);
      }
      remaining = d - nPoints * path_density;
      si++;
      poses.at(si).position = pm.position;
      d = 0;
      ei++;
    } else {
      d = 0;
      remaining = 0;
      trajectory_fixed.poses.push_back(poses.at(ei));
      ei++;
      si = ei - 1;
    }
  }

  if (trajectory_fixed.poses.size() < 2) {
    trajectory_fixed.poses.clear();
    return trajectory_fixed;
  }

  // Calculate waypoint orientations
  auto array_size = trajectory_fixed.poses.size();
  for (int i = 0; i < array_size; i++) {
    auto &pose = trajectory_fixed.poses.at(i);

    if (i < array_size - 1) {
      auto const &pose_next = trajectory_fixed.poses.at(i + 1);

      auto angle = std::atan2(pose_next.position.y - pose.position.y, pose_next.position.x - pose.position.x);

      tf2::Quaternion q;
      q.setRPY(0, 0, angle);
      pose.orientation.w = q.getW();
      pose.orientation.x = q.getX();
      pose.orientation.y = q.getY();
      pose.orientation.z = q.getZ();
    } else {
      pose.orientation = trajectory_fixed.poses.at(array_size - 2).orientation;
    }
  }

  return trajectory_fixed;
}

geometry_msgs::PoseArray Planner::convert_path_to_pose_array(const nav_msgs::Path &path_to_convert) {
  geometry_msgs::PoseArray pose_array_converted;
  pose_array_converted.header = path_to_convert.header;
  for (auto const &pose_stamped:path_to_convert.poses) {
    pose_array_converted.poses.push_back(pose_stamped.pose);
  }
  return pose_array_converted;
}

geometry_msgs::PoseArray Planner::transform_pose_array_to_base_link(const geometry_msgs::PoseArray &pose_array,
                                                                    const geometry_msgs::TransformStamped &transform) {
  auto pose_to_matrix = [](const geometry_msgs::Pose &pose) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    const auto &pos = pose.position;
    const auto &ori = pose.orientation;
    Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
    mat.topLeftCorner<3, 3>() = quat.toRotationMatrix();
    mat.topRightCorner<3, 1>() = Eigen::Vector3d(pos.x, pos.y, pos.z);
    return mat;
  };

  auto transform_pose = [](const Eigen::Matrix4d &transformed_pose) {
    geometry_msgs::Pose pose_trans;
    Eigen::Quaterniond quat(transformed_pose.topLeftCorner<3, 3>());
    const Eigen::Vector3d &trans = transformed_pose.topRightCorner<3, 1>();
    pose_trans.position.x = trans.x();
    pose_trans.position.y = trans.y();
    pose_trans.position.z = trans.z();
    pose_trans.orientation.x = quat.x();
    pose_trans.orientation.y = quat.y();
    pose_trans.orientation.z = quat.z();
    pose_trans.orientation.w = quat.w();
    return pose_trans;
  };

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = "base_link";
  poses.header.stamp = transform.header.stamp;
  for (auto &pose : pose_array.poses) {
    auto pose_matrix = pose_to_matrix(pose);
    auto transformed_pose_matrix = create_transform_matrix(transform).inverse() * pose_matrix;
    auto transformed_pose = transform_pose(transformed_pose_matrix);
    poses.poses.push_back(transformed_pose);
  }
  return poses;
}

Eigen::Matrix4d Planner::create_transform_matrix(const geometry_msgs::TransformStamped &transform) {
  // Creates transform matrix from a transform

  Eigen::Quaterniond q;
  q.x() = transform.transform.rotation.x;
  q.y() = transform.transform.rotation.y;
  q.z() = transform.transform.rotation.z;
  q.w() = transform.transform.rotation.w;

  auto rotation_matrix_raw = q.normalized().toRotationMatrix();
  Eigen::Vector3d translation_vector;
  translation_vector << transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z;

  Eigen::Matrix4d transform_matrix;
  transform_matrix.setIdentity();

  transform_matrix.topLeftCorner(3, 3) = rotation_matrix_raw;
  transform_matrix.topRightCorner(3, 1) = translation_vector;

  return transform_matrix;
}
