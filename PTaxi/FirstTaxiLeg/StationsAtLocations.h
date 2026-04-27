#pragma once
#include "../Station/Station.h"
#include "KARRI/DataStructures/Utilities/IteratorRange.h"
#include "kassert/kassert.hpp"

struct Station;

namespace parrot {

    // Allows query of which stations are located at a given location (edge in the vehicle road network).
    struct StationsAtLocations {

        StationsAtLocations(const PTStations &ptStations, int numVehEdges)
            : stationsAtVehEdgeIndex(numVehEdges + 1, 0) {
            initializeEdgeToStationMapping(ptStations);
        }

        // Returns the station ids of the stations located at the given edge in the vehicle road network.
        ConstantVectorRange<int> getIdsOfStationsAtVehEdge(int vehEdgeId) const {
            int startIndex = stationsAtVehEdgeIndex[vehEdgeId];
            int endIndex = stationsAtVehEdgeIndex[vehEdgeId + 1];
            return {stationsAtVehEdge.begin() + startIndex, stationsAtVehEdge.begin() + endIndex};
        }

    private:
        void initializeEdgeToStationMapping(const PTStations &ptStations) {
            // Count stations per vehicle edge to get offsets
            for (const auto &station: ptStations) {
                ++stationsAtVehEdgeIndex[station.vehEdgeId];
            }

            // Compute offsets as prefix sum of counts
            int offset = 0;
            for (size_t i = 0; i < stationsAtVehEdgeIndex.size(); ++i) {
                int count = stationsAtVehEdgeIndex[i];
                stationsAtVehEdgeIndex[i] = offset;
                offset += count;
            }

            // Fill stationsAtVehEdge according to offsets
            stationsAtVehEdge.resize(ptStations.size());
            for (const auto &station: ptStations) {
                int edgeId = station.vehEdgeId;
                int index = stationsAtVehEdgeIndex[edgeId]++;
                stationsAtVehEdge[index] = station.stationId;
            }

            // Restore correct offsets
            for (size_t i = stationsAtVehEdgeIndex.size() - 1; i > 0; --i) {
                stationsAtVehEdgeIndex[i] = stationsAtVehEdgeIndex[i - 1];
            }
            stationsAtVehEdgeIndex[0] = 0;
            KASSERT(stationsAtVehEdgeIndex.back() == ptStations.size());
        }


        // stationsAtVehEdge[stationsAtVehEdgeIndex[e]..stationsAtVehEdgeIndex[e+1]) are the stations at edge e.
        // This is used to efficiently find stations at the last stop edge of a vehicle.
        std::vector<int> stationsAtVehEdgeIndex;
        std::vector<int> stationsAtVehEdge;
    };
}
