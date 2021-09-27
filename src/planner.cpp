#include "planner.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_msgs/Int32MultiArray.h>
#include <cavc/polyline.hpp>
#include <cavc/polylineintersects.hpp>

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  sub_occ_grid_ = std::make_shared<SubOccGrid>(n, "/occupy/costmap", 1);
  sub_pose_stamped_start_ = std::make_shared<SubPoseStamped>(n, "/occupy/pose_stamped_start_hybrid", 1);
  sub_pose_stamped_goal_ = std::make_shared<SubPoseStamped>(n, "/occupy/pose_stamped_goal_hybrid", 1);

  sub_grid_map_ = std::make_shared<ros::Subscriber>(
      n.subscribe("/occupy/grid_map_map_builder",
                  1,
                  &Planner::callback_grid_map,
                  this));

  synchronizer_ = std::make_shared<Synchronizer>(
      SyncPolicy(10),
      *sub_occ_grid_,
      *sub_pose_stamped_start_,
      *sub_pose_stamped_goal_);
  synchronizer_->registerCallback(
      boost::bind(
          &Planner::callback_synchronizer,
          this,
          boost::placeholders::_1,
          boost::placeholders::_2,
          boost::placeholders::_3));

  calculate_footprint_base();
  path_global.poses.clear();

  replan_ = false;
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
    const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal) {

  std::cout << "&&&&&&&&&&&&&&&&&& Callback." << std::endl;

  auto locked_grid_map = sync_grid_map_.wlock();
  grid_map::GridMap grid_map = *locked_grid_map;

  if (grid_map.getLength().isZero(1e-3)) {
    // Collision check is not possible!
    return;
  }

  geometry_msgs::Pose vehicle_pose;
  if (!get_vehicle_pose(vehicle_pose)) {
    // Vehicle pose unknown!
    return;
  }

  nav_msgs::OccupancyGrid::Ptr msg_occ_grid_ptr = boost::make_shared<nav_msgs::OccupancyGrid>();
  *msg_occ_grid_ptr = *msg_occ_grid;

//  std::cout << "msg_pose_stamped_start->pose: " << msg_pose_stamped_start->pose << std::endl;
//  std::cout << "msg_pose_stamped_goal->pose: " << msg_pose_stamped_goal->pose << std::endl;

  auto start_hybrid = transform_grid2hybrid(msg_pose_stamped_start->pose);
  auto goal_hybrid = transform_grid2hybrid(msg_pose_stamped_goal->pose);

//  std::cout << "start_hybrid: " << start_hybrid << std::endl;
//  std::cout << "goal_hybrid: " << goal_hybrid << std::endl;

  if (path_global.poses.empty() || replan_) {

    geometry_msgs::PoseArray path_planned;
    auto valid_plan = plan(msg_occ_grid_ptr,
                           start_hybrid,
                           goal_hybrid,
                           path_planned);
    if (valid_plan) {
      path_planned.poses.insert(path_planned.poses.begin(), transform_map2hybrid(vehicle_pose));
      auto path_fixed = path_interpolate_and_fix_orientation(path_planned, 1.0);
      path_global.poses.clear();
      path_global = transform_hybrid2map(path_fixed);
    }
    replan_ = false;
  }

  auto plan_start_index = -99;
  auto plan_start_index_memo = -99;
  auto publish_path = false;
  std::cout << "********** While start." << std::endl;
  while (true) {

//    std::cout << "path_global.poses.size(): " << path_global.poses.size() << std::endl;

    if (plan_start_index == -1) {
      ROS_WARN("No viable solution exists!");
      // No viable solution exists!
      path_global.poses.erase(path_global.poses.begin() + plan_start_index_memo, path_global.poses.end());
      plan_start_index = -99;
      plan_start_index_memo = -99;
    }

    if (path_global.poses.empty()) {
      ROS_WARN("Collision check is not possible!");
      // Collision check is not possible!
      break;
    }

    auto path_global_grid_map = transform_pose_array(path_global, "grid_map");

    auto collision_check_result_array = do_collision_check_on_grid_frame(path_global_grid_map,
                                                                         grid_map,
                                                                         footprint_base);

    // Calculates distance between two waypoint
    auto calculate_dist = [](geometry_msgs::Point first_wp, geometry_msgs::Point second_wp) {
      auto dist_calc = std::sqrt(std::pow((second_wp.y - first_wp.y), 2) + std::pow((second_wp.x - first_wp.x), 2));
      return dist_calc;
    };

    auto dist_to_goal = calculate_dist(path_global.poses.back().position,
                                       transform_hybrid2map(goal_hybrid).position);

    auto dist_to_start = calculate_dist(path_global.poses.front().position,
                                        vehicle_pose.position);

    auto path_is_clear = true;
    for (int i = 0; i < collision_check_result_array.data.size(); ++i) {
      const int &collision_info = collision_check_result_array.data.at(i);
      if (collision_info == 1) {
        std::cout << "Collision found. Path is not clear!" << std::endl;
        if (plan_start_index == -99) {
          plan_start_index = i;
          plan_start_index_memo = i;
        }
        path_is_clear = false;
        break;
      }
    }

    if (dist_to_start >= 2.0) {
      replan_ = true;
    }
    if (!path_is_clear && plan_start_index != -1) {
      std::cout << "plan_start_index: " << plan_start_index << std::endl;
      auto recent_start_map = path_global.poses.at(plan_start_index);
      auto recent_start_hybrid = transform_map2hybrid(recent_start_map);

      geometry_msgs::PoseArray path_planned;
      auto valid_plan = plan(msg_occ_grid_ptr,
                             recent_start_hybrid,
                             goal_hybrid,
                             path_planned);
      if (valid_plan) {
        auto path_fixed = path_interpolate_and_fix_orientation(path_planned, 1.0);
        auto new_path_map = transform_hybrid2map(path_fixed);

        path_global.poses.erase(path_global.poses.begin() + plan_start_index, path_global.poses.end());
        path_global.poses.insert(path_global.poses.end(), new_path_map.poses.begin(), new_path_map.poses.end());
      }
      plan_start_index--;
    }
    if (path_is_clear) {
      if (dist_to_goal >= 5.0) {
        std::cout << "Trying to replan to connect the path to the goal." << std::endl;
        geometry_msgs::PoseArray path_planned;
        auto valid_plan = plan(msg_occ_grid_ptr,
                               transform_map2hybrid(path_global.poses.back()),
                               goal_hybrid,
                               path_planned);
        if (valid_plan) {
          auto path_fixed = path_interpolate_and_fix_orientation(path_planned, 1.0);
          auto new_path_map = transform_hybrid2map(path_fixed);

          path_global.poses.pop_back();
          path_global.poses.insert(path_global.poses.end(), new_path_map.poses.begin(), new_path_map.poses.end());
        }
      }
      publish_path = true;
      break;
    }
    std::cout << "----------- In while." << std::endl;
  }
  std::cout << "********** While end." << std::endl;

  if (publish_path) {
    std::cout << "********** Publish path." << std::endl;
    auto clipped_path = path_clip(path_global, vehicle_pose);
    if (clipped_path.poses.empty()) {
      path_global.poses.clear();
      return;
    }
    path_global.poses.clear();
    path_global.poses = clipped_path.poses;
    auto global_path_interpolated_and_ori_fixed =
        path_interpolate_and_fix_orientation(clipped_path, HybridAStar::Constants::path_density);

    if (Constants::path_density > 0.0) {
      auto clip_wp_count =
          static_cast<int>(std::floor(Constants::path_clip_dist_from_base_link / Constants::path_density));

      if (global_path_interpolated_and_ori_fixed.poses.size() > clip_wp_count) {
        global_path_interpolated_and_ori_fixed.poses.erase(global_path_interpolated_and_ori_fixed.poses.begin(),
                                                           global_path_interpolated_and_ori_fixed.poses.begin()
                                                               + clip_wp_count);
      }
    }

    global_path_interpolated_and_ori_fixed.header.stamp = msg_occ_grid->header.stamp;
    smoothedPath.publishPath(global_path_interpolated_and_ori_fixed);
  }
//  else {
//    path_global.poses.clear();
//  }

  std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&." << std::endl;
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
//  voronoiDiagram.visualize();
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::Pose &initial) {
  float x = initial.position.x / Constants::cellSize;
  float y = initial.position.y / Constants::cellSize;
  float t = tf::getYaw(initial.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial.position;
  startN.pose.orientation = initial.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start.pose = initial;

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::Pose &end) {
  // retrieving goal position
  float x = end.position.x / Constants::cellSize;
  float y = end.position.y / Constants::cellSize;
  float t = tf::getYaw(end.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal.pose = end;

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
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(*msg_grid_map, grid_map);
  *locked_grid_map = grid_map;

  grid_length_x_ = msg_grid_map->info.length_x;
  grid_length_y_ = msg_grid_map->info.length_y;
}

// Return collision check result array ["1" for collision, "0" for collision free]
std_msgs::Int32MultiArray Planner::do_collision_check_on_grid_frame(const geometry_msgs::PoseArray &trajectory_on_grid_map,
                                                                    const grid_map::GridMap &grid_map,
                                                                    const geometry_msgs::Polygon &vehicle_polygon) {

  // Creates transform matrix from a pose to the origin on the same frame
  auto transform_matrix_to_origin = [](const geometry_msgs::Pose &msg_pose) {
    Eigen::Quaterniond q;
    q.x() = msg_pose.orientation.x;
    q.y() = msg_pose.orientation.y;
    q.z() = msg_pose.orientation.z;
    q.w() = msg_pose.orientation.w;

    auto rotation_matrix_raw = q.normalized().toRotationMatrix();
    Eigen::Vector3d translation_vector;
    translation_vector << msg_pose.position.x, msg_pose.position.y, msg_pose.position.z;

    Eigen::Matrix4d transform_matrix;
    transform_matrix.setIdentity();

    transform_matrix.topLeftCorner(3, 3) = rotation_matrix_raw;
    transform_matrix.topRightCorner(3, 1) = translation_vector;
    return transform_matrix;
  };

  // Checks if a polygon on the grid_map frame is collision free
  auto is_polygon_clear = [grid_map](const geometry_msgs::Polygon &my_polygon) {
    grid_map::Polygon poly;
    for (auto const &point : my_polygon.points) {
      grid_map::Position vertex;
      vertex.x() = point.x;
      vertex.y() = point.y;
      poly.addVertex(vertex);
    }

    bool is_clear = true;
    for (grid_map::PolygonIterator iterator(grid_map, poly); !iterator.isPastEnd(); ++iterator) {
      auto cost = grid_map.at("cost", *iterator);
      if (cost >= 1.0f) {
        is_clear = false;
        break;
      }
    }
    return is_clear;
  };

  std_msgs::Int32MultiArray collision_check_result_array;
  visualization_msgs::MarkerArray collision_marker_array;
  for (int i = 0; i < trajectory_on_grid_map.poses.size(); ++i) {
    auto const &c_pose = trajectory_on_grid_map.poses.at(i);
    auto pose_transform_matrix = transform_matrix_to_origin(c_pose);

    geometry_msgs::Polygon transformed_polygon;
    for (auto &point : vehicle_polygon.points) {
      Eigen::Vector4d org_point_vector;
      org_point_vector << point.x, point.y, 0.0, 1.0;
      Eigen::Vector4d transformed_point_vector;
      transformed_point_vector = pose_transform_matrix * org_point_vector;
      geometry_msgs::Point32 transformed_point;
      transformed_point.x = transformed_point_vector(0);
      transformed_point.y = transformed_point_vector(1);
      transformed_polygon.points.push_back(transformed_point);
    }

//    color::rgb<float> marker_color = color::constant::aquamarine_t();
//    auto marker_alpha = 0.3f;
    if (!is_polygon_clear(transformed_polygon)) {
      collision_check_result_array.data.push_back(1);
//      marker_color = color::constant::tomato_t();
//      marker_alpha = 0.7f;
    } else {
      collision_check_result_array.data.push_back(0);
    }

    // Calculate & pub marker array
//    {
//      auto vehicle_length = std::abs(vehicle_polygon.points.at(0).x - vehicle_polygon.points.at(2).x);
//      auto vehicle_width = std::abs(vehicle_polygon.points.at(0).y - vehicle_polygon.points.at(2).y);
//
//      float vehicle_base_link_to_back = -1.0f;
//      for (auto point : vehicle_polygon.points) {
//        auto x_c = point.x;
//        if (x_c < 0.0f) {
//          vehicle_base_link_to_back = std::abs(x_c);
//          break;
//        }
//      }
//
//      if (vehicle_base_link_to_back > 0) {
//        auto marker_origin = c_pose;
//        auto shift_on_marker_origin_x = vehicle_length / 2.0 - vehicle_base_link_to_back;
//        Eigen::Vector4d org_shift_vector;
//        org_shift_vector << shift_on_marker_origin_x, 0.0, 0.0, 1.0;
//        Eigen::Vector4d transformed_shift_vector;
//        transformed_shift_vector = pose_transform_matrix * org_shift_vector;
//        marker_origin.position.x = transformed_shift_vector(0);
//        marker_origin.position.y = transformed_shift_vector(1);
//
//        auto box = marker_handler::make_cube(vehicle_length,
//                                             vehicle_width,
//                                             1.0f,
//                                             marker_origin,
//                                             marker_color,
//                                             marker_alpha,
//                                             "grid_map",
//                                             trajectory_on_grid_map.header.stamp,
//                                             38945 + i);
//
//        collision_marker_array.markers.push_back(box);
//      }
//    }
  }
//  pub_collision_check_marker_array_.publish(collision_marker_array);
  return collision_check_result_array;
}

bool Planner::plan(const nav_msgs::OccupancyGrid::Ptr &occupancy_grid,
                   const geometry_msgs::Pose &msg_pose_stamped_start,
                   const geometry_msgs::Pose &msg_pose_stamped_goal,
                   geometry_msgs::PoseArray &path_planned) {

  setMap(occupancy_grid);
  setStart(msg_pose_stamped_start);
  setGoal(msg_pose_stamped_goal);
//  path.setTransformMatrix(*msg_transform_stamped);
//  smoothedPath.setTransformMatrix(*msg_transform_stamped);
  plan();

  // Calculates distance between two waypoint
  auto calculate_dist = [](geometry_msgs::Point first_wp, geometry_msgs::Point second_wp) {
    auto dist_calc = std::sqrt(std::pow((second_wp.y - first_wp.y), 2) + std::pow((second_wp.x - first_wp.x), 2));
    return dist_calc;
  };

  if (smoothedPath.path.poses.empty()) {
    return false;
  }

  auto dist_to_goal = calculate_dist(smoothedPath.path.poses.begin()->pose.position,
                                     msg_pose_stamped_goal.position);

  auto reversed_path = smoothedPath.path;
  std::reverse(reversed_path.poses.begin(), reversed_path.poses.end());

  auto p_line_path = path2polyline(reversed_path);
  cavc::StaticSpatialIndex<double> spacial_index = cavc::createApproxSpatialIndex(p_line_path);

  // Check for self intersection
  std::vector<cavc::PlineIntersect<double>> self_intersection_result;
  cavc::allSelfIntersects(p_line_path,
                          self_intersection_result,
                          spacial_index);
  bool self_intersection_found = !self_intersection_result.empty();

  if (self_intersection_found) {
    ROS_WARN("Hybrid A*: Self intersection found! Skipping the calculated path.");
    return false;
  }

  if (dist_to_goal > 1.0) {
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

geometry_msgs::PoseArray Planner::path_clip(const geometry_msgs::PoseArray &path_to_clip,
                                            const geometry_msgs::Pose &vehicle_pose) {

  geometry_msgs::PoseArray clipped_path;
  clipped_path.header = path_to_clip.header;

  if (path_to_clip.poses.empty()) {
    return clipped_path;
  }

  cavc::Polyline<float> route;
  for (auto const &pose_c:path_to_clip.poses) {
    auto vertex = cavc::PlineVertex<float>(static_cast<float>(pose_c.position.x),
                                           static_cast<float>(pose_c.position.y),
                                           0.0f);
    route.vertexes().push_back(vertex);
  }

  cavc::ClosestPoint<float> point_closest(
      route,
      cavc::Vector2<float>(static_cast<float>(vehicle_pose.position.x),
                           static_cast<float>(vehicle_pose.position.y)));

  clipped_path.poses.insert(clipped_path.poses.begin(),
                            path_to_clip.poses.begin() + point_closest.index(),
                            path_to_clip.poses.end());

  return clipped_path;
}

bool Planner::get_vehicle_pose(geometry_msgs::Pose &vehicle_pose) {

  // Get affine_to_map
  Eigen::Affine3f affine_to_map;
  if (!get_affine(affine_to_map,
                  "base_link",
                  "map",
                  ros::Time())) {
    return false;
  }
  const auto &base_link_translation = affine_to_map.translation();
  const auto &base_link_rotation = affine_to_map.rotation();
  vehicle_pose.position.x = base_link_translation.x();
  vehicle_pose.position.y = base_link_translation.y();
  vehicle_pose.position.z = base_link_translation.z();

  Eigen::Quaternionf base_link_q(base_link_rotation);
  vehicle_pose.orientation.x = base_link_q.x();
  vehicle_pose.orientation.y = base_link_q.y();
  vehicle_pose.orientation.z = base_link_q.z();
  vehicle_pose.orientation.w = base_link_q.w();

  return true;
}

geometry_msgs::PoseArray Planner::convert_path_to_pose_array(const nav_msgs::Path &path_to_convert) {
  geometry_msgs::PoseArray pose_array_converted;
  pose_array_converted.header = path_to_convert.header;
  for (auto const &pose_stamped:path_to_convert.poses) {
    pose_array_converted.poses.push_back(pose_stamped.pose);
  }
  return pose_array_converted;
}

//geometry_msgs::PoseArray Planner::transform_pose_array_to_base_link(const geometry_msgs::PoseArray &pose_array,
//                                                                    const geometry_msgs::TransformStamped &transform) {
//  auto pose_to_matrix = [](const geometry_msgs::Pose &pose) {
//    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
//    const auto &pos = pose.position;
//    const auto &ori = pose.orientation;
//    Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
//    mat.topLeftCorner<3, 3>() = quat.toRotationMatrix();
//    mat.topRightCorner<3, 1>() = Eigen::Vector3d(pos.x, pos.y, pos.z);
//    return mat;
//  };
//
//  auto transform_pose = [](const Eigen::Matrix4d &transformed_pose) {
//    geometry_msgs::Pose pose_trans;
//    Eigen::Quaterniond quat(transformed_pose.topLeftCorner<3, 3>());
//    const Eigen::Vector3d &trans = transformed_pose.topRightCorner<3, 1>();
//    pose_trans.position.x = trans.x();
//    pose_trans.position.y = trans.y();
//    pose_trans.position.z = trans.z();
//    pose_trans.orientation.x = quat.x();
//    pose_trans.orientation.y = quat.y();
//    pose_trans.orientation.z = quat.z();
//    pose_trans.orientation.w = quat.w();
//    return pose_trans;
//  };
//
//  geometry_msgs::PoseArray poses;
//  poses.header.frame_id = "base_link";
//  poses.header.stamp = transform.header.stamp;
//  for (auto &pose : pose_array.poses) {
//    auto pose_matrix = pose_to_matrix(pose);
//    auto transformed_pose_matrix = create_transform_matrix(transform).inverse() * pose_matrix;
//    auto transformed_pose = transform_pose(transformed_pose_matrix);
//    poses.poses.push_back(transformed_pose);
//  }
//  return poses;
//}

//Eigen::Matrix4d Planner::create_transform_matrix(const geometry_msgs::TransformStamped &transform) {
//  // Creates transform matrix from a transform
//
//  Eigen::Quaterniond q;
//  q.x() = transform.transform.rotation.x;
//  q.y() = transform.transform.rotation.y;
//  q.z() = transform.transform.rotation.z;
//  q.w() = transform.transform.rotation.w;
//
//  auto rotation_matrix_raw = q.normalized().toRotationMatrix();
//  Eigen::Vector3d translation_vector;
//  translation_vector << transform.transform.translation.x,
//      transform.transform.translation.y,
//      transform.transform.translation.z;
//
//  Eigen::Matrix4d transform_matrix;
//  transform_matrix.setIdentity();
//
//  transform_matrix.topLeftCorner(3, 3) = rotation_matrix_raw;
//  transform_matrix.topRightCorner(3, 1) = translation_vector;
//
//  return transform_matrix;
//}

void Planner::calculate_footprint_base() {

  geometry_msgs::Point32 point_first;
  geometry_msgs::Point32 point_first_with_safety;
  point_first.x = Constants::vehicle_base_link_to_front;
  point_first_with_safety.x = point_first.x + Constants::vehicle_length_safety_dist;
  point_first.y = -Constants::vehicle_width / 2.0f;
  point_first_with_safety.y = point_first.y - Constants::vehicle_width_safety_dist;
  footprint_base.points.push_back(point_first);
  footprint_with_safety_base.points.push_back(point_first_with_safety);

  geometry_msgs::Point32 point_second;
  geometry_msgs::Point32 point_second_with_safety;
  point_second.x = -Constants::vehicle_base_link_to_back;
  point_second_with_safety.x = point_second.x - Constants::vehicle_length_safety_dist;
  point_second.y = -Constants::vehicle_width / 2.0f;
  point_second_with_safety.y = point_second.y - Constants::vehicle_width_safety_dist;
  footprint_base.points.push_back(point_second);
  footprint_with_safety_base.points.push_back(point_second_with_safety);

  geometry_msgs::Point32 point_third;
  geometry_msgs::Point32 point_third_with_safety;
  point_third.x = -Constants::vehicle_base_link_to_back;
  point_third_with_safety.x = point_third.x - Constants::vehicle_length_safety_dist;
  point_third.y = Constants::vehicle_width / 2.0f;
  point_third_with_safety.y = point_third.y + Constants::vehicle_width_safety_dist;
  footprint_base.points.push_back(point_third);
  footprint_with_safety_base.points.push_back(point_third_with_safety);

  geometry_msgs::Point32 point_fourth;
  geometry_msgs::Point32 point_fourth_with_safety;
  point_fourth.x = Constants::vehicle_base_link_to_front;
  point_fourth_with_safety.x = point_fourth.x + Constants::vehicle_length_safety_dist;
  point_fourth.y = Constants::vehicle_width / 2.0f;
  point_fourth_with_safety.y = point_fourth.y + Constants::vehicle_width_safety_dist;
  footprint_base.points.push_back(point_fourth);
  footprint_with_safety_base.points.push_back(point_fourth_with_safety);
}

geometry_msgs::PoseArray Planner::transform_pose_array(const geometry_msgs::PoseArray &trajectory,
                                                       const std::string &target_frame_id) {

  geometry_msgs::PoseArray transformed_trajectory;
  transformed_trajectory.header = trajectory.header;
  auto frame_id = trajectory.header.frame_id;
  if (frame_id == target_frame_id) {
    transformed_trajectory = trajectory;
    return transformed_trajectory;
  }

  Eigen::Affine3f affine_transform;
  if (!get_affine(affine_transform,
                  frame_id,
                  target_frame_id,
                  ros::Time())) {
    return transformed_trajectory;
  }

  auto pose_to_matrix = [](const geometry_msgs::Pose &pose) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    const auto &pos = pose.position;
    const auto &ori = pose.orientation;
    Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
    mat.topLeftCorner<3, 3>() = quat.toRotationMatrix();
    mat.topRightCorner<3, 1>() = Eigen::Vector3f(pos.x, pos.y, pos.z);
    return mat;
  };

  auto transform_pose = [&pose_to_matrix](const geometry_msgs::Pose &pose, const Eigen::Affine3f &affine) {
    geometry_msgs::Pose pose_trans;
    auto mat_pose = pose_to_matrix(pose);
    Eigen::Matrix4f mat_trans = affine.matrix() * mat_pose;
    Eigen::Quaternionf quat(mat_trans.topLeftCorner<3, 3>());
    const Eigen::Vector3f &trans = mat_trans.topRightCorner<3, 1>();
    pose_trans.position.x = trans.x();
    pose_trans.position.y = trans.y();
    pose_trans.position.z = trans.z();
    pose_trans.orientation.x = quat.x();
    pose_trans.orientation.y = quat.y();
    pose_trans.orientation.z = quat.z();
    pose_trans.orientation.w = quat.w();
    return pose_trans;
  };

  transformed_trajectory.poses.resize(trajectory.poses.size());
  std::transform(trajectory.poses.begin(),
                 trajectory.poses.end(),
                 transformed_trajectory.poses.begin(),
                 std::bind(transform_pose, std::placeholders::_1, affine_transform));

  transformed_trajectory.header.frame_id = target_frame_id;
  return transformed_trajectory;
}

geometry_msgs::Pose Planner::transform_pose(const geometry_msgs::Pose &pose,
                                            const std::string &source_frame_id,
                                            const std::string &target_frame_id) {
  geometry_msgs::Pose transformed_pose;

  Eigen::Affine3f affine_transform;
  if (!get_affine(affine_transform,
                  source_frame_id,
                  target_frame_id,
                  ros::Time())) {
    return transformed_pose;
  }

  auto pose_to_matrix = [](const geometry_msgs::Pose &pose) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    const auto &pos = pose.position;
    const auto &ori = pose.orientation;
    Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
    mat.topLeftCorner<3, 3>() = quat.toRotationMatrix();
    mat.topRightCorner<3, 1>() = Eigen::Vector3f(pos.x, pos.y, pos.z);
    return mat;
  };

  auto transform_pose = [&pose_to_matrix](const geometry_msgs::Pose &pose, const Eigen::Affine3f &affine) {
    geometry_msgs::Pose pose_trans;
    auto mat_pose = pose_to_matrix(pose);
    Eigen::Matrix4f mat_trans = affine.matrix() * mat_pose;
    Eigen::Quaternionf quat(mat_trans.topLeftCorner<3, 3>());
    const Eigen::Vector3f &trans = mat_trans.topRightCorner<3, 1>();
    pose_trans.position.x = trans.x();
    pose_trans.position.y = trans.y();
    pose_trans.position.z = trans.z();
    pose_trans.orientation.x = quat.x();
    pose_trans.orientation.y = quat.y();
    pose_trans.orientation.z = quat.z();
    pose_trans.orientation.w = quat.w();
    return pose_trans;
  };

  transformed_pose = transform_pose(pose, affine_transform);

  return transformed_pose;
}

bool Planner::get_affine(Eigen::Affine3f &affine,
                         const std::string &frame_source,
                         const std::string &frame_target,
                         const ros::Time &time) {

  auto get_transform_latest = [this, &time](const std::string &source,
                                            const std::string &target)
      -> geometry_msgs::TransformStamped::Ptr {
    geometry_msgs::TransformStamped::Ptr transform_stamped = nullptr;
    try {
      transform_stamped.reset(new geometry_msgs::TransformStamped);

      *transform_stamped = tf_buffer_->lookupTransform(
          target,
          source,
          time);
    }
    catch (tf2::TransformException &ex) {
      transform_stamped = nullptr;
      ROS_WARN("%s", ex.what());
    }
    return transform_stamped;
  };

  geometry_msgs::TransformStamped::Ptr trans_init_ = get_transform_latest(
      frame_source,
      frame_target);
  if (trans_init_ == nullptr) {
    return false;
  }

  affine = tf2::transformToEigen(*trans_init_).cast<float>();

  return true;
}

geometry_msgs::PoseArray Planner::transform_grid2hybrid(const geometry_msgs::PoseArray &pose_array) const {
  auto pose_array_transformed = pose_array;
  for (auto &pose:pose_array_transformed.poses) {
    pose.position.x += grid_length_x_ / 2.0;
    pose.position.y += grid_length_y_ / 2.0;
  }
  pose_array_transformed.header.frame_id = "map";
  return pose_array_transformed;
}

geometry_msgs::Pose Planner::transform_grid2hybrid(const geometry_msgs::Pose &pose) const {
  auto pose_transformed = pose;
  pose_transformed.position.x += grid_length_x_ / 2.0;
  pose_transformed.position.y += grid_length_y_ / 2.0;
  return pose_transformed;
}

geometry_msgs::PoseArray Planner::transform_hybrid2grid(const geometry_msgs::PoseArray &pose_array) const {
  auto pose_array_transformed = pose_array;
  for (auto &pose:pose_array_transformed.poses) {
    pose.position.x -= grid_length_x_ / 2.0;
    pose.position.y -= grid_length_y_ / 2.0;
  }
  pose_array_transformed.header.frame_id = "grid_map";
  return pose_array_transformed;
}

geometry_msgs::Pose Planner::transform_hybrid2grid(const geometry_msgs::Pose &pose) const {
  auto pose_transformed = pose;
  pose_transformed.position.x -= grid_length_x_ / 2.0;
  pose_transformed.position.y -= grid_length_y_ / 2.0;
  return pose_transformed;
}

geometry_msgs::PoseArray Planner::transform_map2hybrid(const geometry_msgs::PoseArray &pose_array) {
  auto pose_array_grid = transform_pose_array(pose_array, "grid_map");
  auto pose_array_hybrid = transform_grid2hybrid(pose_array_grid);
  return pose_array_hybrid;
}

geometry_msgs::Pose Planner::transform_map2hybrid(const geometry_msgs::Pose &pose) {
  auto pose_grid = transform_pose(pose, "map", "grid_map");
  auto pose_hybrid = transform_grid2hybrid(pose_grid);
  return pose_hybrid;
}

geometry_msgs::PoseArray Planner::transform_hybrid2map(const geometry_msgs::PoseArray &pose_array) {
  auto pose_array_grid_map = transform_hybrid2grid(pose_array);
  auto pose_array_map = transform_pose_array(pose_array_grid_map, "map");
  return pose_array_map;
}

geometry_msgs::Pose Planner::transform_hybrid2map(const geometry_msgs::Pose &pose) {
  auto pose_grid_map = transform_hybrid2grid(pose);
  auto pose_map = transform_pose(pose_grid_map, "grid_map", "map");
  return pose_map;
}

cavc::Polyline<double> Planner::path2polyline(const nav_msgs::Path &path_to_convert) {
  cavc::Polyline<double> output;

  for (auto const &pose: path_to_convert.poses) {
    output.addVertex(pose.pose.position.x, pose.pose.position.y, 0.0);
  }
  return output;
}

