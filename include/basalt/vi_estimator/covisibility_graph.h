#pragma once

#include <basalt/utils/common_types.h>
#include <unordered_set>

namespace basalt {

struct CovisibilityEdge {
  FrameId id1;  // Always the smaller id
  FrameId id2;  // Always the larger id

  CovisibilityEdge(FrameId a, FrameId b) : id1(std::min(a, b)), id2(std::max(a, b)) {}

  bool operator==(const CovisibilityEdge& other) const { return id1 == other.id1 && id2 == other.id2; }

  struct Hash {
    size_t operator()(const CovisibilityEdge& e) const {
      size_t h = std::hash<FrameId>{}(e.id1);
      h ^= std::hash<FrameId>{}(e.id2) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };
};

enum class PoseGraphEdgeType { SPANNING_TREE, LOOP_CLOSURE, HIGH_COVISIBILITY };

inline const char* toString(PoseGraphEdgeType type) {
  switch (type) {
    case PoseGraphEdgeType::SPANNING_TREE:
      return "spanning_tree";
    case PoseGraphEdgeType::LOOP_CLOSURE:
      return "loop_closure";
    case PoseGraphEdgeType::HIGH_COVISIBILITY:
      return "high_covisibility";
  }
  return "unknown";
}

// Edge of the pose graph that is optimized on a loop closure. id1 is the parent for spanning tree
// edges and the smaller id for the other edge types.
struct PoseGraphEdge {
  FrameId id1;
  FrameId id2;
  PoseGraphEdgeType type;
};

struct NodeScore {
  FrameId id;

  float graph_score;
  float loop_score;
};

class CovisibilityGraph {
 public:
  using Ptr = std::shared_ptr<CovisibilityGraph>;
  using GraphScoreChangedCb = std::function<void(FrameId, NodeScore)>;

  static constexpr FrameId invalid() { return FrameId(-1); }

  void setHighCovisibilityThreshold(size_t threshold);

  void setGraphScoreChangedCallback(GraphScoreChangedCb cb);

  const std::unordered_map<FrameId, int>& getCovisibility(FrameId id) const;

  const std::unordered_set<CovisibilityEdge, CovisibilityEdge::Hash>& getHighCovisibilityEdges() const;

  const FrameId& getParentId(FrameId id) const;

  const std::vector<FrameId>& getChildrenIds(FrameId id) const;

  const std::unordered_map<FrameId, std::vector<FrameId>>& getLoopClosures() const;

  bool hasNode(FrameId id) const;

  bool edgeExists(FrameId id1, FrameId id2) const;

  void setEdge(FrameId id1, FrameId id2, int weight);

  void incrementEdge(FrameId id1, FrameId id2, int weight);

  void decrementEdge(FrameId id1, FrameId id2, int weight);

  void addTreeNode(FrameId id, FrameId parent_id);

  void addLoopClosure(FrameId id1, FrameId id2);

  void setRoot(FrameId id);

  FrameId getRoot() const;

  void removeNode(FrameId id);

  std::vector<FrameId> getTopCovisible(FrameId id, size_t k) const;

  std::vector<FrameId> getCovisibleAbove(FrameId id, int weight_threshold) const;

  NodeScore computeGraphScore(FrameId id) const;

  CovisibilityGraph copyPoseGraph() const;

  // Edges that make up the pose graph optimized on a loop closure: the spanning tree, the loop
  // closures and the high covisibility edges that are not already spanning tree edges.
  std::vector<PoseGraphEdge> getPoseGraphEdges() const;

  void saveJson(const std::string& file_path) const;

  void print_stats() const;

 private:
  struct TreeNode {
    FrameId parent = invalid();
    std::vector<FrameId> children;
  };

  // Covisibility edges and weights
  std::unordered_map<FrameId, std::unordered_map<FrameId, int>> covis_;

  // Pose graph structures
  std::unordered_map<FrameId, TreeNode> tree_;
  std::unordered_map<FrameId, std::vector<FrameId>> loop_closures_;
  std::unordered_set<CovisibilityEdge, CovisibilityEdge::Hash> high_covisibility_edges_;

  size_t high_covisibility_threshold_;

  GraphScoreChangedCb graph_score_changed_cb_;

  size_t culled_nodes_count_ = 0;

  FrameId root_ = invalid();
};

}  // namespace basalt
