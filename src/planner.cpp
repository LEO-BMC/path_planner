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

  sub_occupancy_grid_ = std::make_shared<ros::Subscriber>(
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

  setMap(msg_occ_grid_ptr);
  setStart(msg_pose_stamped_start);
  setGoal(msg_pose_stamped_goal);
  path.setTransformMatrix(*msg_transform_stamped);
  smoothedPath.setTransformMatrix(*msg_transform_stamped);
  plan();
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
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

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
                   geometry_msgs::PoseArray &poses_plan) {

  // Obtain things in their respective frames and transform them to the hybrid astar frame

  setMap(occupancy_grid);
  setStart(msg_pose_stamped_start);
  setGoal(msg_pose_stamped_goal);
//  path.setTransformMatrix(*msg_transform_stamped);
//  smoothedPath.setTransformMatrix(*msg_transform_stamped);
  plan();

  return false;
}
