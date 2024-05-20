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
#include <motion_planning/iox/gnn_iox_utils.hpp>
#include <motion_planning/iox/metrics.hpp>
#include <iox/signal_watcher.hpp>
#include <iceoryx_posh/popo/listener.hpp>
#include <iceoryx_posh/popo/server.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
//! [iceoryx includes]

#include <iostream>

constexpr char APP_NAME[] = "gnn_query_service";

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

//! [request callback]

int main()
{
  //! [initialize runtime]
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  //! [initialize runtime]

  iox::popo::Listener listener;

  //! [create server]
  iox::popo::ServerOptions options;
  options.requestQueueCapacity = 10U;
  iox::popo::Server<LTV::SingleQueryRequest, LTV::SingleQueryResult> server(
      { "GNN-server", "Request-Response", "GNN-queries" }, options);
  //! [create server]
  LTV::Query gnn_queries{};

  // LTV_gnn_server_t gnn_query_server{};
  //! [attach listener]
  listener
      .attachEvent(server, iox::popo::ServerEvent::REQUEST_RECEIVED,
                   iox::popo::createNotificationCallback(LTV::IoxService::single_query, gnn_queries))
      .or_else([](auto) {
        std::cerr << "unable to attach server" << std::endl;
        std::exit(EXIT_FAILURE);
      });
  //! [attach listener]

  //! [wait for termination]
  // iox::posix::waitForTerminationRequest();
  iox::SignalWatcher::getInstance().wasSignalTriggered();
  //! [wait for termination]

  //! [cleanup]
  listener.detachEvent(server, iox::popo::ServerEvent::REQUEST_RECEIVED);
  //! [cleanup]

  return EXIT_SUCCESS;
}