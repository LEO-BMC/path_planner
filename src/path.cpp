#include "path.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(const std::vector<Node3D> &nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}

void Path::setTransformMatrix(const geometry_msgs::TransformStamped &transform) {
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

  transform_matrix_hybrid_to_base_link = transform_matrix;
  msg_stamp_ = transform.header.stamp;
}

// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D &node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Node3D &node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "map";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D &node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}

geometry_msgs::PoseArray Path::path_interpolate_and_fix_orientation(
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
