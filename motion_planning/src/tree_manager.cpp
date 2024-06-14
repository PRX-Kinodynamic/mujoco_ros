// Copyright (c) 2022 by Apex.AI Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

//! [iceoryx includes]
// #include "request_and_response_types.hpp"

#include <iox/signal_watcher.hpp>
#include <iceoryx_posh/popo/client.hpp>
#include <iceoryx_posh/popo/wait_set.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
//! [iceoryx includes]

#include <motion_planning/iox/gnn.hpp>
#include <motion_planning/iox/gnn_iox_utils.hpp>
#include <motion_planning/iox/metrics.hpp>

#include <iostream>

constexpr char APP_NAME[] = "tree_manager";

namespace LTV
{

using State = Eigen::Vector4d;
using Metric = motion_planning::metrics::eucleadian<State>;
using Node = motion_planning::nearest_neighbors::node_t<State, 200>;
using Container = iox::cxx::vector<Node, 100'000>;
using Graph = motion_planning::nearest_neighbors::graph_t<Container, Metric>;
using Query = motion_planning::nearest_neighbors::queries_t<Node, 2000>;
using SingleQueryRequest = motion_planning::nearest_neighbors::single_query_request_t<State, Graph>;
using SingleQueryResult = motion_planning::nearest_neighbors::single_query_result_t;

using IoxService = motion_planning::nearest_neighbors::gnn_iox_service_t<State, Graph, Query>;
}  // namespace LTV

int main()
{
  //! [initialize runtime]
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  //! [initialize runtime]

  //! [create waitset]
  iox::popo::WaitSet<> waitset;

  //! [create client]
  iox::popo::ClientOptions options;
  options.responseQueueCapacity = 2U;
  iox::popo::Client<LTV::SingleQueryRequest, LTV::SingleQueryResult> client(
      { "GNN-server", "Request-Response", "GNN-queries" }, options);
  //! [create client]

  // attach client to waitset
  waitset.attachState(client, iox::popo::ClientState::HAS_RESPONSE).or_else([](auto) {
    std::cerr << "failed to attach client" << std::endl;
    std::exit(EXIT_FAILURE);
  });
  //! [create waitset]

  // Creating a square
  const LTV::State p0{ LTV::State(0, 0) };
  const LTV::State p1{ LTV::State(0, 1) };
  const LTV::State p2{ LTV::State(1, 0) };
  const LTV::State p3{ LTV::State(1, 1) };

  LTV::Graph gnn{ LTV::Metric() };
  gnn.emplace_back(p0);
  gnn.emplace_back(p1);
  gnn.emplace_back(p2);
  gnn.emplace_back(p3);

  LTV::Query gnn_queries{};
  gnn_queries.add_node(gnn[0], gnn);
  gnn_queries.add_node(gnn[1], gnn);
  gnn_queries.add_node(gnn[2], gnn);
  gnn_queries.add_node(gnn[3], gnn);

  //! [mainloop]
  while (!iox::hasTerminationRequested())
  {
    //! [send request]
    client.loan()
        .and_then([&](auto& request) {
          // request.getRequestHeader().setSequenceId(ctx.requestSequenceId);
          // ctx.expectedResponseSequenceId = ctx.requestSequenceId;
          // ctx.requestSequenceId += 1;
          request->state = Eigen::Vector4d::Random();
          // request->container = gnn;
          std::cout << "[Tree] Send Request: " << request->state.transpose() << std::endl;
          request.send().or_else(
              [&](auto& error) { std::cout << "Could not send Request! Error: " << error << std::endl; });
        })
        .or_else([](auto& error) { std::cout << "Could not allocate Request! Error: " << error << std::endl; });
    //! [send request]

    // We block and wait for samples to arrive, when the time is up we send the request again
    //! [wait and check if the client triggered]
    auto notificationVector = waitset.timedWait(iox::units::Duration::fromSeconds(5));

    for (auto& notification : notificationVector)
    {
      if (notification->doesOriginateFrom(&client))
      {
        //! [take response]
        while (client.take().and_then([&](const auto& response) {
          auto receivedSequenceId = response.getResponseHeader().getSequenceId();
          std::cout << "[Tree] Response " << receivedSequenceId << ": " << response->distance;
          std::cout << " " << response->index;
          std::cout << " " << response->success;
        }))
        {
        }
        //! [take response]
      }
    }
    //! [wait and check if the client triggered]
    constexpr std::chrono::milliseconds SLEEP_TIME{ 2000U };
    std::this_thread::sleep_for(SLEEP_TIME);
  }
  //! [mainloop]

  std::cout << "shutting down" << std::endl;

  return (EXIT_SUCCESS);
}