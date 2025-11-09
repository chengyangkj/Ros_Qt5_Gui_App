#include "tf2_rosbridge.h"
#include "point.h"
#include "logger/logger.h"

namespace rosbridge2cpp {

std::string TF2Rosbridge::NormalizeFrame(const std::string &frame) {
  std::string normalized = frame;
  if (!normalized.empty() && normalized[0] == '/') {
    normalized = normalized.substr(1);
  }
  return normalized;
}

void TF2Rosbridge::UpdateTF(const std::unordered_map<std::string, TransformData> &tf_cache) {
  adj_.clear();
  adj_transform_.clear();

  for (const auto &[child_frame, tf_data] : tf_cache) {
    std::string parent_frame = NormalizeFrame(tf_data.parent_frame);
    std::string normalized_child = NormalizeFrame(child_frame);

    if (parent_frame.empty() || normalized_child.empty()) {
      continue;
    }

    basic::RobotPose transform_pose;
    transform_pose.x = tf_data.x;
    transform_pose.y = tf_data.y;
    transform_pose.theta = tf_data.theta;

    adj_[parent_frame].insert(normalized_child);
    adj_[normalized_child].insert(parent_frame);

    adj_transform_[parent_frame][normalized_child] = transform_pose;
    
    basic::RobotPose inverse_pose;
    double cos_theta = std::cos(tf_data.theta);
    double sin_theta = std::sin(tf_data.theta);
    inverse_pose.x = -(cos_theta * tf_data.x + sin_theta * tf_data.y);
    inverse_pose.y = -(-sin_theta * tf_data.x + cos_theta * tf_data.y);
    inverse_pose.theta = -tf_data.theta;
    
    adj_transform_[normalized_child][parent_frame] = inverse_pose;
  }
}

std::vector<std::string> TF2Rosbridge::ShortestPath(const std::string &from, const std::string &to) {
  std::string normalized_from = NormalizeFrame(from);
  std::string normalized_to = NormalizeFrame(to);

  if (normalized_from == normalized_to) {
    return {normalized_from};
  }

  if (adj_.find(normalized_from) == adj_.end() || adj_.find(normalized_to) == adj_.end()) {
    return {};
  }

  std::unordered_map<std::string, std::string> parent;
  std::queue<std::string> queue;
  std::unordered_set<std::string> visited;

  queue.push(normalized_from);
  visited.insert(normalized_from);
  parent[normalized_from] = "";

  int search_depth = 0;
  while (!queue.empty()) {
    std::string current = queue.front();
    queue.pop();

    if (current == normalized_to) {
      std::vector<std::string> path;
      std::string node = normalized_to;
      while (!node.empty()) {
        path.insert(path.begin(), node);
        node = parent[node];
      }
      return path;
    }

    if (adj_.find(current) != adj_.end()) {
      for (const auto &next : adj_[current]) {
        if (visited.find(next) == visited.end()) {
          visited.insert(next);
          queue.push(next);
          parent[next] = current;
        }
      }
    }
    
    search_depth++;
    if (search_depth > 100) {
      break;
    }
  }

  return {};
}

basic::RobotPose TF2Rosbridge::LookUpForTransform(const std::string &from, const std::string &to) {
  basic::RobotPose result;
  result.x = 0.0;
  result.y = 0.0;
  result.theta = 0.0;

  std::vector<std::string> path = ShortestPath(from, to);
  if (path.empty()) {
    return result;
  }

  for (size_t i = 0; i < path.size() - 1; i++) {
    const std::string &curr = path[i];
    const std::string &next = path[i + 1];

    if (adj_transform_.find(curr) == adj_transform_.end()) {
      continue;
    }

    const auto &transform_map = adj_transform_[curr];
    if (transform_map.find(next) == transform_map.end()) {
      continue;
    }

    const basic::RobotPose &transform = transform_map.at(next);
    result = basic::absoluteSum(result, transform);
  }

  return result;
}

}  // namespace rosbridge2cpp


