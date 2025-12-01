#pragma once

#include "Classes/GraphInterface.h"

#include "Utils/Utils.h"

#include "Classes/DynamicGraph.h"
#include "Classes/StaticGraph.h"
#include "Classes/EdgeList.h"

using NoVertexAttributes = List<>;
using WithCoordinates = List<Attribute<Coordinates, Geometry::Point>>;
using WithSize = List<Attribute<Size, size_t>>;

using NoEdgeAttributes = List<>;
using WithTravelTime = List<Attribute<TravelTime, int>>;
using WithTravelTimeAndDistance = List<Attribute<TravelTime, int>, Attribute<Distance, int>>;
using WithTravelTimeAndDelays = List<Attribute<TravelTime, int>, Attribute<MinOriginDelay, int>, Attribute<MaxOriginDelay, int>>;
using WithReverseEdges = List<Attribute<ReverseEdge, Edge>>;
using WithCapacity = List<Attribute<Capacity, int>>;
using WithWeight = List<Attribute<Weight, int>>;
using WithViaVertex = List<Attribute<ViaVertex, Vertex>>;
using WithViaVertexAndWeight = List<Attribute<ViaVertex, Vertex>, Attribute<Weight, int>>;
using WithReverseEdgesAndViaVertex = List<Attribute<ReverseEdge, Edge>, Attribute<ViaVertex, Vertex>>;
using WithReverseEdgesAndWeight = List<Attribute<ReverseEdge, Edge>, Attribute<Weight, int>>;
using WithReverseEdgesAndCapacity = List<Attribute<ReverseEdge, Edge>, Attribute<Capacity, int>>;

using TransferGraph = ULTRAStaticGraph<WithCoordinates, WithTravelTime>;
using DynamicTransferGraph = ULTRADynamicGraph<WithCoordinates, WithTravelTime>;
using TransferEdgeList = EdgeList<WithCoordinates, WithTravelTime>;

using SimpleDynamicGraph = ULTRADynamicGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleStaticGraph = ULTRAStaticGraph<NoVertexAttributes, NoEdgeAttributes>;
using SimpleEdgeList = EdgeList<NoVertexAttributes, NoEdgeAttributes>;

using DynamicFlowGraph = ULTRADynamicGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;
using StaticFlowGraph = ULTRAStaticGraph<NoVertexAttributes, WithReverseEdgesAndCapacity>;

using CHConstructionGraph = EdgeList<NoVertexAttributes, WithViaVertexAndWeight>;
using CHCoreGraph = ULTRADynamicGraph<NoVertexAttributes, WithViaVertexAndWeight>;
using CHGraph = ULTRAStaticGraph<NoVertexAttributes, WithViaVertexAndWeight>;

using DimacsGraph = EdgeList<NoVertexAttributes, WithTravelTime>;
using DimacsGraphWithCoordinates = EdgeList<WithCoordinates, WithTravelTime>;

using TravelTimeGraph = ULTRAStaticGraph<NoVertexAttributes, WithTravelTime>;

using CondensationGraph = ULTRADynamicGraph<WithSize, WithTravelTime>;

using DelayGraph = ULTRAStaticGraph<WithCoordinates, WithTravelTimeAndDelays>;
using DynamicDelayGraph = ULTRADynamicGraph<WithCoordinates, WithTravelTimeAndDelays>;

#include "Utils/Conversion.h"
#include "Utils/IO.h"
