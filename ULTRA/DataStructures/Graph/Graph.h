#pragma once

#include "Classes/DynamicGraph.h"
#include "Classes/EdgeList.h"
#include "Classes/GraphInterface.h"
#include "Classes/StaticGraph.h"
#include "Utils/Utils.h"

using NoVertexAttributes = List<>;
using WithCoordinates = List<Attribute<Coordinates, Geometry::Point>>;
using WithSize = List<Attribute<Size, size_t>>;

using NoEdgeAttributes = List<>;
using WithTravelTime = List<Attribute<TravelTime, int>>;
using WithTravelTimeAndDistance = List<Attribute<TravelTime, int>, Attribute<Distance, int>>;
using WithTravelTimeAndEdgeFlags = List<Attribute<TravelTime, int>, Attribute<EdgeFlags, std::vector<bool>>>;
using WithTravelTimeAndBundleSize = List<Attribute<TravelTime, int>, Attribute<BundleSize, int>>;
using WithReverseEdges = List<Attribute<ReverseEdge, Edge>>;
using WithCapacity = List<Attribute<Capacity, int>>;
using WithWeight = List<Attribute<Weight, int>>;
using WithViaVertex = List<Attribute<ViaVertex, Vertex>>;
using WithViaVertexAndWeight = List<Attribute<ViaVertex, Vertex>, Attribute<Weight, int>>;
using WithReverseEdgesAndViaVertex = List<Attribute<ReverseEdge, Edge>, Attribute<ViaVertex, Vertex>>;
using WithReverseEdgesAndWeight = List<Attribute<ReverseEdge, Edge>, Attribute<Weight, int>>;
using WithReverseEdgesAndCapacity = List<Attribute<ReverseEdge, Edge>, Attribute<Capacity, int>>;

using StrasserGraph = StaticGraph<NoVertexAttributes, WithTravelTimeAndDistance>;
using StrasserGraphWithCoordinates = StaticGraph<WithCoordinates, WithTravelTimeAndDistance>;

using TransferGraph = StaticGraph<WithCoordinates, WithTravelTime>;
using DynamicTransferGraph = DynamicGraph<WithCoordinates, WithTravelTime>;
using TransferEdgeList = EdgeList<WithCoordinates, WithTravelTime>;
using EdgeFlagsTransferGraph = DynamicGraph<WithCoordinates, WithTravelTimeAndEdgeFlags>;

using SimpleDynamicGraph = DynamicGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleStaticGraph = StaticGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleEdgeList = EdgeList<NoVertexAttributes, NoEdgeAttributes>;

using DynamicFlowGraph = DynamicGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;
using StaticFlowGraph = StaticGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;

using CHConstructionGraph = EdgeList<NoVertexAttributes, WithViaVertexAndWeight>;
using CHCoreGraph = DynamicGraph<NoVertexAttributes, WithViaVertexAndWeight>;
using CHGraph = StaticGraph<NoVertexAttributes, WithViaVertexAndWeight>;

using DimacsGraph = EdgeList<NoVertexAttributes, WithTravelTime>;
using DimacsGraphWithCoordinates = EdgeList<WithCoordinates, WithTravelTime>;

using TravelTimeGraph = StaticGraph<NoVertexAttributes, WithTravelTime>;

using CondensationGraph = DynamicGraph<WithSize, WithTravelTime>;

using BundledGraph = StaticGraph<WithCoordinates, WithTravelTimeAndBundleSize>;
using DynamicBundledGraph = DynamicGraph<WithCoordinates, WithTravelTimeAndBundleSize>;

// ********************************
// currently, we take the defs from louis
// ********************************
// First the Attributes for the Graph
// using WithPositionAndDistanceLabelsForToAndFrom =
//     List<Attribute<Position, size_t>, Attribute<DistanceLabelTo, size_t>,
//          Attribute<DistanceLabelFrom, size_t>>;

// Define the Ride Transfer Graph (for now, as dynamic graph => maybe in the
// future change to vector<vector<>>?)
// using RideTransferGraph = DynamicGraph<NoVertexAttributes,
// WithPositionAndDistanceLabelsForToAndFrom>;

// Helper for the KaRRiCHGraph
using WithWeightAndUnpackingInfo = List<Attribute<Weight, int>, Attribute<UnpackingInfo, std::pair<int, int>>>;
using KaRRiCHGraph = StaticGraph<NoVertexAttributes, WithWeightAndUnpackingInfo>;
using KaRRiGraph = StaticGraph<WithCoordinates,
    List<Attribute<FromVertex, Vertex>, Attribute<TravelTime, int>,
        Attribute<EdgeId, int>>>;

using ConstructionGraph = DynamicGraph<List<Attribute<VehicleId, int>>, List<Attribute<Weight, int>, Attribute<InsertionInfo, StopInsertionInfo>>>;
using RideTransferGraph = DynamicGraph<List<Attribute<VehicleId, int>>, List<Attribute<Weight, int>, Attribute<InsertionInfo, StopInsertionInfo>>>;
// ********************************

#include "Utils/Conversion.h"
#include "Utils/IO.h"
