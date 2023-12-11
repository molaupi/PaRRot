#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../../Algorithms/KaRRi/KaRRi/BaseObjects/Vehicle.h"
#include "../../../Helpers/Types.h"

namespace RIDERAPTOR {

class JourneyLeg {
 public:
  JourneyLeg(const Vertex from = noVertex, const Vertex to = noVertex,
             const int departureTime = never, const int arrivalTime = never,
             const bool usesRoute = true, const bool usesRide = false,
             const RouteId routeId = noRouteId)
      : from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesRoute(usesRoute),
        usesRide(usesRide),
        routeId(routeId) {}

  JourneyLeg(const Vertex from, const Vertex to, const int departureTime,
             const int arrivalTime, const bool usesRoute = false,
             const bool usesRide = false, const Edge edge = noEdge)
      : from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesRoute(usesRoute),
        usesRide(usesRide),
        transferId(edge) {}

  JourneyLeg(const Vertex from, const Vertex to, const int departureTime,
             const int arrivalTime, const bool usesRoute = false,
             const bool usesRide = true, const int vehicleId = INFTY)
      : from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesRoute(usesRoute),
        usesRide(usesRide),
        vehicleId(vehicleId) {}

  inline int transferTime() const noexcept {
    return usesRoute || usesRide ? 0 : arrivalTime - departureTime;
  }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const JourneyLeg &leg) noexcept {
    return out << "from: " << leg.from << ", to: " << leg.to
               << ", dep-Time: " << leg.departureTime
               << ", arr-Time: " << leg.arrivalTime
               << (leg.usesRoute  ? ", route: "
                   : leg.usesRide ? ", ride: "
                                  : ", transfer: ")
               << leg.routeId;
  }

 public:
  Vertex from;
  Vertex to;
  int departureTime;
  int arrivalTime;
  bool usesRoute;
  bool usesRide;
  union {
    RouteId routeId;
    Edge transferId;
    int vehicleId;
  };
};

using Journey = std::vector<JourneyLeg>;

inline std::vector<Vertex> journeyToPath(const Journey &journey) noexcept {
  std::vector<Vertex> path;
  for (const JourneyLeg &leg : journey) {
    path.emplace_back(leg.from);
  }
  path.emplace_back(journey.back().to);
  return path;
}

inline int totalTransferTime(const Journey &journey) noexcept {
  int transferTime = 0;
  for (const JourneyLeg &leg : journey) {
    transferTime += leg.transferTime();
  }
  return transferTime;
}

inline int intermediateTransferTime(const Journey &journey) noexcept {
  int transferTime = 0;
  for (size_t i = 1; i < journey.size() - 1; i++) {
    transferTime += journey[i].transferTime();
  }
  return transferTime;
}

inline int initialTransferTime(const Journey &journey) noexcept {
  if (journey.empty()) return 0;
  int transferTime = journey[0].transferTime();
  if (journey.size() > 1) {
    transferTime += journey.back().transferTime();
  }
  return transferTime;
}

inline size_t countTrips(const Journey &journey) noexcept {
  size_t numTrips = 0;
  for (const JourneyLeg &leg : journey) {
    if (leg.usesRoute && leg.routeId != noRouteId) numTrips++;
  }
  return numTrips;
}

inline bool usesRide(const Journey &journey) noexcept {
  for (const JourneyLeg &leg : journey) {
    if (leg.usesRide) return true;
  }
  return false;
}

inline bool usesInitialRide(const Journey &journey) noexcept {
  const auto initialLeg = journey[0];
  return initialLeg.usesRide;
}

inline bool usesFinalRide(const Journey &journey) noexcept {
  const auto finalLeg = journey.back();
  return finalLeg.usesRide;
}

}  // namespace RIDERAPTOR
