#pragma once
#include <motion_planning/iox/gnn.hpp>

#include <iox/signal_watcher.hpp>
#include <iceoryx_posh/popo/listener.hpp>
#include <iceoryx_posh/popo/server.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
namespace motion_planning
{

namespace nearest_neighbors
{
using NodeDistancePair = std::pair<std::size_t, double>;

//! [request]
template <typename State, typename Container>
struct single_query_request_t
{
  single_query_request_t() noexcept {};
  State state;
  Container container;
};

template <typename State, typename Graph, typename QueryLTV>
struct gnn_iox_service_t
{
  using SingleQueryRequest = motion_planning::nearest_neighbors::single_query_request_t<State, Graph>;
  using SingleQueryResult = motion_planning::nearest_neighbors::single_query_result_t;

  //! [request callback]
  static void single_query(iox::popo::Server<SingleQueryRequest, SingleQueryResult>* server, QueryLTV* gnn_queries)
  {
    //! [take request]
    while (server->take().and_then([&](const auto& request) {
      std::cout << "[GNN] Got Request: " << request->state.transpose() << std::endl;

      //! [send response]
      server->loan(request)
          .and_then([&](auto& response) {
            const SingleQueryResult result{ gnn_queries->single_query(request->state, request->container) };

            response->distance = result.distance;
            response->index = result.index;
            response->success = result.success;
            std::cout << "[GNN] Sending Response: " << response->distance;
            std::cout << " " << response->index;
            std::cout << " " << response->success;

            response.send().or_else(
                [&](auto& error) { std::cout << "Could not send Response! Error: " << error << std::endl; });
          })
          .or_else([](auto& error) { std::cout << "Could not allocate Response! Error: " << error << std::endl; });
      //! [send response]
    }))
    {
    }
    //! [take request]
  }

private:
};
//! [request callback]

}  // namespace nearest_neighbors
}  // namespace motion_planning
