#include <ros/init.h>
#include <gtest/gtest.h>
#include <nodelet/nodelet.h>
#include <motion_planning/iox/gnn.hpp>

namespace mock
{
template <typename EigenVector>
struct metric
{
  double operator()(const EigenVector& a, const EigenVector& b) const
  {
    return (a - b).norm();
  }
};

}  // namespace mock

TEST(GNN, empty_gnn)
{
  using State = Eigen::Vector2d;
  using Metric = mock::metric<State>;
  using Node = motion_planning::nearest_neighbors::node_t<State, 10>;
  using Container = std::vector<Node>;
  using Gnn = motion_planning::nearest_neighbors::graph_t<Container, Metric>;

  Gnn gnn{ Metric() };

  EXPECT_TRUE(Gnn::current_nodes(gnn) == 0);
}

TEST(GNN, query_construction)
{
  using State = Eigen::Vector2d;
  using Metric = mock::metric<State>;
  using Node = motion_planning::nearest_neighbors::node_t<State, 10>;
  using Container = std::vector<Node>;
  using Gnn = motion_planning::nearest_neighbors::graph_t<Container, Metric>;
  using Queries = motion_planning::nearest_neighbors::queries_t<Node, 200>;

  // Gnn gnn{ Metric() };
  Queries gnn_queries{};

  SUCCEED();
}

TEST(GNN, add_node_to_gnn)
{
  using State = Eigen::Vector2d;
  using Metric = mock::metric<State>;
  using Node = motion_planning::nearest_neighbors::node_t<State, 10>;
  using Container = std::vector<Node>;
  using Gnn = motion_planning::nearest_neighbors::graph_t<Container, Metric>;
  using Queries = motion_planning::nearest_neighbors::queries_t<Node, 200>;

  Gnn gnn{ Metric() };
  Queries gnn_queries{};

  const State p0{ State(0, 0) };

  gnn.emplace_back(p0);
  Node& n0{ gnn.back() };
  gnn_queries.add_node(n0, gnn);

  EXPECT_TRUE(Gnn::current_nodes(gnn) == 1);
  EXPECT_TRUE(Node::added_to_metric(n0));
}

TEST(GNN, query_gnn_with_single_node)
{
  using State = Eigen::Vector2d;
  using Metric = mock::metric<State>;
  using Node = motion_planning::nearest_neighbors::node_t<State, 10>;
  using Container = std::vector<Node>;
  using Gnn = motion_planning::nearest_neighbors::graph_t<Container, Metric>;
  using Queries = motion_planning::nearest_neighbors::queries_t<Node, 200>;
  using SingleQueryResult = motion_planning::nearest_neighbors::single_query_result_t;

  Gnn gnn{ Metric() };
  Queries gnn_queries{};

  const State p0{ State(0, 0) };
  const State p0p{ State(0, 1) };  // P0^\prime

  gnn.emplace_back(p0);
  Node& n0{ gnn.back() };
  gnn_queries.add_node(n0, gnn);

  const SingleQueryResult result0{ gnn_queries.single_query(p0, gnn) };
  const SingleQueryResult result0p{ gnn_queries.single_query(p0p, gnn) };

  const std::size_t expected_idx{ Node::index(n0) };
  const double expected_dist0{ 0.0 };
  const double expected_dist0p{ 1.0 };

  EXPECT_EQ(result0.index, expected_idx);
  EXPECT_DOUBLE_EQ(result0.distance, expected_dist0);

  EXPECT_EQ(result0p.index, expected_idx);
  EXPECT_DOUBLE_EQ(result0p.distance, expected_dist0p);
}

TEST(GNN, query_gnn_with_single_node_high_dim)
{
  using State = Eigen::Vector<double, 10>;
  using Metric = mock::metric<State>;
  using Node = motion_planning::nearest_neighbors::node_t<State, 10>;
  using Container = std::vector<Node>;
  using Gnn = motion_planning::nearest_neighbors::graph_t<Container, Metric>;
  using Queries = motion_planning::nearest_neighbors::queries_t<Node, 200>;
  using SingleQueryResult = motion_planning::nearest_neighbors::single_query_result_t;

  Gnn gnn{ Metric() };
  Queries gnn_queries{};

  const State p0{ State::Zero() };
  State p0p{ State::Zero() };  // P0^\prime
  p0p[0] = 1.0;

  gnn.emplace_back(p0);
  Node& n0{ gnn.back() };
  gnn_queries.add_node(n0, gnn);

  const SingleQueryResult result0{ gnn_queries.single_query(p0, gnn) };
  const SingleQueryResult result0p{ gnn_queries.single_query(p0p, gnn) };

  const std::size_t expected_idx{ Node::index(n0) };
  const double expected_dist0{ 0.0 };
  const double expected_dist0p{ 1.0 };

  EXPECT_EQ(result0.index, expected_idx);
  EXPECT_DOUBLE_EQ(result0.distance, expected_dist0);

  EXPECT_EQ(result0p.index, expected_idx);
  EXPECT_DOUBLE_EQ(result0p.distance, expected_dist0p);
}

TEST(GNN, query_gnn_with_multiple_nodes)
{
  using State = Eigen::Vector2d;
  using Metric = mock::metric<State>;
  using Node = motion_planning::nearest_neighbors::node_t<State, 10>;
  using Container = std::vector<Node>;
  using Gnn = motion_planning::nearest_neighbors::graph_t<Container, Metric>;
  using Queries = motion_planning::nearest_neighbors::queries_t<Node, 200>;
  using SingleQueryResult = motion_planning::nearest_neighbors::single_query_result_t;

  Gnn gnn{ Metric() };
  Queries gnn_queries{};

  // Creating a square
  const State p0{ State(0, 0) };
  const State p1{ State(0, 1) };
  const State p2{ State(1, 0) };
  const State p3{ State(1, 1) };

  gnn.emplace_back(p0);
  gnn.emplace_back(p1);
  gnn.emplace_back(p2);
  gnn.emplace_back(p3);
  gnn_queries.add_node(gnn[0], gnn);
  gnn_queries.add_node(gnn[1], gnn);
  gnn_queries.add_node(gnn[2], gnn);
  gnn_queries.add_node(gnn[3], gnn);

  const SingleQueryResult result0{ gnn_queries.single_query(p0, gnn) };
  const SingleQueryResult result0p{ gnn_queries.single_query(State(0.49, 0.49), gnn) };  // closest to P0

  const std::size_t expected_idx{ Node::index(gnn[0]) };
  const double expected_dist0{ 0.0 };
  const double expected_dist0p{ std::sqrt(2.0 * 0.49 * 0.49) };

  EXPECT_EQ(result0.index, expected_idx);
  EXPECT_DOUBLE_EQ(result0.distance, expected_dist0);

  EXPECT_EQ(result0p.index, expected_idx);
  EXPECT_DOUBLE_EQ(result0p.distance, expected_dist0p);
}
int main(int argc, char** argv)
{
  // ros::Time::init();
  ros::init(argc, argv, "gnn_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}