/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once
namespace karri {

// Initializes the request state for a new request.
    template<typename VehInputGraphT,
            typename VehCHEnvT>
    class RequestStateInitializer {

    public:
        RequestStateInitializer(const VehInputGraphT &vehInputGraph,
            const VehCHEnvT &vehChEnv)
                : vehInputGraph(vehInputGraph),
                  vehCh(vehChEnv.getCH()),
                  vehChQuery(vehChEnv.template getFullCHQuery<>()) {}


        std::pair<RequestState, stats::DispatchingPerformanceStats> initializeRequestState(const Request &req) {
            KaRRiTimer timer;

            RequestState requestState;
            stats::DispatchingPerformanceStats stats;
            requestState.reset();
            stats.clear();
            requestState.originalRequest = req;
            requestState.setEarliestDeparture(req.requestTime);

            // Calculate the direct distance between the requests origin and destination
            timer.restart();
            const auto source = vehCh.rank(vehInputGraph.edgeHead(req.origin));
            const auto target = vehCh.rank(vehInputGraph.edgeTail(req.destination));
            vehChQuery.run(source, target);
            requestState.originalReqDirectDist = vehChQuery.getDistance() + vehInputGraph.travelTime(req.destination);

            const auto directSearchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationStats.computeODDistanceTime = directSearchTime;

            return {requestState, stats};
        }


    private:

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const VehInputGraphT &vehInputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

    };
}