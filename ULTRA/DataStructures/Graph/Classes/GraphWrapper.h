#pragma once

// ******************************
// I took this class from Louis Repo
// ******************************

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

#include "../../../Helpers/Ranges/IndirectEdgeRange.h"
#include "../../../Helpers/Ranges/Range.h"
#include "../../../Helpers/Ranges/SubRange.h"
#include "../../Attributes/AttributeNames.h"
#include "../../Geometry/Point.h"
#include "../../KaRRi/Geometry/LatLng.h"
#include "../Graph.h"
#include "../Utils/Conversion.h"
#include "../Utils/Utils.h"
#include "GraphInterface.h"

// Wrapps Graph to work for imported KaRRi classes
template <typename GRAPH>
class GraphWrapper {
 public:
  using ListOfVertexAttributes = typename GRAPH::ListOfVertexAttributes;
  using ListOfEdgeAttributes = typename GRAPH::ListOfEdgeAttributes;
  using Type = typename GRAPH::Type;

  using VertexAttributes = typename GRAPH::VertexAttributes;
  using EdgeAttributes = typename GRAPH::EdgeAttributes;

  using VertexHandle = typename GRAPH::VertexHandle;
  using EdgeHandle = typename GRAPH::EdgeHandle;

  using ListOfRecordVertexAttributes =
      typename GRAPH::ListOfRecordVertexAttributes;
  using ListOfRecordEdgeAttributes = typename GRAPH::ListOfRecordEdgeAttributes;
  using VertexRecord = typename GRAPH::VertexRecord;
  using EdgeRecord = typename GRAPH::EdgeRecord;

  using ListOfAllAttributes = typename GRAPH::ListOfAllAttributes;

  template <typename FROM_GRAPH_TYPE, typename TO_GRAPH_TYPE,
            typename... ATTRIBUTE_NAME_CHANGES>
  friend inline void Graph::copy(
      const FROM_GRAPH_TYPE &from, TO_GRAPH_TYPE &to,
      const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) noexcept;

  template <AttributeNameType ATTRIBUTE_NAME>
  inline constexpr static bool HasVertexAttribute(
      const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
    return VertexAttributes::HasAttribute(
        AttributeNameWrapper<ATTRIBUTE_NAME>());
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  using VertexAttributeType =
      typename VertexAttributes::template AttributeType<ATTRIBUTE_NAME>;

  template <AttributeNameType ATTRIBUTE_NAME>
  inline constexpr static bool HasEdgeAttribute(
      const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
    return EdgeAttributes::HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>());
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  using EdgeAttributeType =
      typename EdgeAttributes::template AttributeType<ATTRIBUTE_NAME>;

  template <AttributeNameType ATTRIBUTE_NAME>
  using AttributeType =
      typename Meta::FindAttributeType<ATTRIBUTE_NAME, ListOfAllAttributes>;

  template <AttributeNameType ATTRIBUTE_NAME>
  using AttributeReferenceType = typename std::vector<
      Meta::FindAttributeType<ATTRIBUTE_NAME, ListOfAllAttributes>>::reference;

  template <AttributeNameType ATTRIBUTE_NAME>
  using AttributeConstReferenceType =
      typename std::vector<Meta::FindAttributeType<
          ATTRIBUTE_NAME, ListOfAllAttributes>>::const_reference;

 public:
  GraphWrapper() = default;

  GraphWrapper(GRAPH &graph) : graph(graph) {}

  GraphWrapper(TransferGraph &constructionGraph) {
    Graph::move(std::move(constructionGraph), graph);
    generateNeededAttributes();
  }

  // Access
  inline size_t numVertices() const noexcept { return graph.numVertices(); }

  inline size_t numEdges() const noexcept { return graph.numEdges(); }

  inline size_t edgeLimit() const noexcept { return graph.edgeLimit(); }

  inline bool isVertex(const Vertex vertex) const noexcept {
    return graph.isVertex(vertex);
  }

  inline bool isEdge(const Edge edge) const noexcept {
    return graph.isEdge(edge);
  }

  inline size_t outDegree(const Vertex vertex) const noexcept {
    return graph.outDegree(vertex);
  }

  inline Range<Vertex> vertices() const noexcept { return graph.vertices(); }

  inline Range<Edge> edgesFrom(const Vertex vertex) const noexcept {
    return graph.edgesFrom(vertex);
  }

  inline Edge beginEdgeFrom(const Vertex vertex) const noexcept {
    return graph.beginEdgeFrom(vertex);
  }

  inline Edge endEdgeFrom(const Vertex vertex) const noexcept {
    return graph.endEdgeFrom(vertex);
  }

  inline Range<Edge> edges() const noexcept { return graph.edges(); }

  inline IndirectEdgeRange<Type> edgesWithFromVertex() const noexcept {
    return graph.edgesWithFromVertex();
  }

  inline SubRange<std::vector<Vertex>> outgoingNeighbors(
      const Vertex vertex) const noexcept {
    return graph.outgoingNeighbors(vertex);
  }

  inline Edge findEdge(const Vertex from, const Vertex to) const noexcept {
    return graph.findEdge(from, to);
  }

  inline bool hasEdge(const Vertex from, const Vertex to) const noexcept {
    return graph.hasEdge(from, to);
  }

  inline bool empty() const noexcept { return graph.empty(); }

  inline long long byteSize() const noexcept { return graph.byteSize(); }

  inline long long memoryUsageInBytes() const noexcept {
    return graph.memoryUsageInBytes();
  }

  // Manipulation:
  inline void clear() noexcept { graph.clear(); }

  inline void reserve(const size_t numVertices,
                      const size_t numEdges) noexcept {
    graph.reserve(numVertices, numEdges);
  }

  inline Vertex addVertex() noexcept { return graph.addVertex(); }

  inline void addVertices(const size_t n = 1) noexcept { graph.addVertices(n); }

  inline Vertex addVertex(const VertexRecord &record) noexcept {
    return graph.addVertex(record);
  }

  inline void addVertices(const size_t n, const VertexRecord &record) noexcept {
    graph.addVertices(n, record);
  }

  inline EdgeHandle addEdge(const Vertex from, const Vertex to) noexcept {
    return graph.addEdge(from, to);
  }

  inline EdgeHandle addEdge(const Vertex from, const Vertex to,
                            const EdgeRecord &record) noexcept {
    return graph.addEdge(from, to, record);
  }

  inline void redirectEdge(const Edge edge, const Vertex oldFrom,
                           const Vertex newTo) noexcept {
    graph.redirectEdge(edge, oldFrom, newTo);
  }

  template <typename DELETE_VERTEX>
  inline void deleteVertices(const DELETE_VERTEX &deleteVertex) noexcept {
    graph.deleteVertices(deleteVertex);
  }

  template <typename T>
  inline void deleteVertices(const std::vector<T> &vertexMap,
                             const T &deleteValue) noexcept {
    graph.deleteVertices(vertexMap, deleteValue);
  }

  inline void deleteVertices(const std::vector<Vertex> &vertexList) noexcept {
    graph.deleteVertices(vertexList);
  }

  template <typename DELETE_EDGE>
  inline void deleteEdges(const DELETE_EDGE &deleteEdge) noexcept {
    graph.deleteEdges(deleteEdge);
  }

  inline void applyVertexPermutation(const Permutation &permutation) noexcept {
    graph.applyVertexPermutation(permutation);
  }

  inline void applyVertexOrder(const Order &order) noexcept {
    graph.applyVertexOrder(order);
  }

  inline void revert() noexcept { graph.revert(); }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline void sortEdges(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
    graph.sortEdges(attributeName);
  }

  // Attributes:
  template <AttributeNameType ATTRIBUTE_NAME>
  inline std::vector<AttributeType<ATTRIBUTE_NAME>> &operator[](
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
    return get(attributeName);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline const std::vector<AttributeType<ATTRIBUTE_NAME>> &operator[](
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) const noexcept {
    return get(attributeName);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline std::vector<AttributeType<ATTRIBUTE_NAME>> &get(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
    return graph.get(attributeName);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline const std::vector<AttributeType<ATTRIBUTE_NAME>> &get(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) const noexcept {
    return graph.get(attributeName);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline AttributeReferenceType<ATTRIBUTE_NAME> get(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
      const Vertex vertex) noexcept {
    return graph.get(attributeName, vertex);
  }
  template <AttributeNameType ATTRIBUTE_NAME>
  inline AttributeReferenceType<ATTRIBUTE_NAME> get(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
      const Edge edge) noexcept {
    return graph.get(attributeName, edge);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline AttributeConstReferenceType<ATTRIBUTE_NAME> get(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
      const Vertex vertex) const noexcept {
    return graph.get(attributeName, vertex);
  }
  template <AttributeNameType ATTRIBUTE_NAME>
  inline AttributeConstReferenceType<ATTRIBUTE_NAME> get(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
      const Edge edge) const noexcept {
    return graph.get(attributeName, edge);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline void set(
      const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
      const std::vector<VertexAttributeType<ATTRIBUTE_NAME>> &values) noexcept {
    return graph.set(attributeName, values);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
                  const Vertex vertex,
                  const VertexAttributeType<ATTRIBUTE_NAME> &value) noexcept {
    return graph.set(attributeName, vertex, value);
  }
  template <AttributeNameType ATTRIBUTE_NAME>
  inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName,
                  const Edge edge,
                  const EdgeAttributeType<ATTRIBUTE_NAME> &value) noexcept {
    return graph.set(attributeName, edge, value);
  }

  inline VertexRecord vertexRecord(const Vertex vertex) const noexcept {
    return graph.vertexRecord(vertex);
  }

  inline EdgeRecord edgeRecord(const Edge edge) const noexcept {
    return graph.edgeRecord(edge);
  }

  inline void setVertexAttributes(const Vertex vertex,
                                  const VertexRecord &record) noexcept {
    graph.setVertexAttributes(vertex, record);
  }

  inline void setEdgeAttributes(const Edge edge,
                                const EdgeRecord &record) noexcept {
    graph.setEdgeAttributes(edge, record);
  }

  inline EdgeHandle edge(const Edge edge) noexcept { return graph.edge(edge); }

  inline VertexHandle vertex(const Vertex vertex) noexcept {
    return graph.vertex(vertex);
  }

  // IO:
  inline void writeBinary(const std::string &fileName,
                          const std::string &separator = ".") const noexcept {
    graph.writeBinary(fileName, separator);
  }

  inline void readBinary(const std::string &fileName,
                         const std::string &separator = ".",
                         const bool debug = true) noexcept {
    graph.readBinary(fileName, separator, debug);
  }

  inline void printAnalysis(std::ostream &out = std::cout) const noexcept {
    graph.printAnalysis(out);
  }

  inline void printAdjacencyList(std::ostream &out = std::cout) const noexcept {
    graph.printAdjacencyList(out);
  }

  inline void checkVectorSize() const noexcept { graph.checkVectorSize(); }

  inline bool satisfiesInvariants() const noexcept {
    return graph.satisfiesInvariants();
  }

  // Appended methods for loud algorithm:

  inline Vertex appendVertex() noexcept { return graph.addVertex(); }

  EdgeHandle appendEdge(const int toVertex) noexcept {
    return graph.addEdge(Vertex(graph.numVertices() - 1), Vertex(toVertex));
  }

  inline bool containsEdge(const int tail, const int head) const {
    return graph.hasEdge(Vertex(tail), Vertex(head));
  }

  inline Vertex edgeHead(const int e) { return edgeHead(Edge(e)); }

  inline Vertex edgeHead(const Edge edge) {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(ToVertex),
              "Missing graph attribute ToVertex");

    return graph.get(ToVertex, edge);
  }

  inline Vertex edgeHead(const int e) const { return edgeHead(Edge(e)); }

  inline Vertex edgeHead(const Edge edge) const {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(ToVertex),
              "Missing graph attribute ToVertex");

    return graph.get(ToVertex, edge);
  }

  inline Vertex edgeTail(const int e) { return edgeTail(Edge(e)); }

  inline Vertex edgeTail(const Edge edge) {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");

    if constexpr (graph.HasEdgeAttribute(FromVertex)) {
      return graph.get(FromVertex, edge);
    } else {
      // very greedy algorithm -> will not get called
      for (const Vertex fromVertex : graph.vertices()) {
        for (const Edge e : graph.EdgesFrom(fromVertex)) {
          if (e == edge) {
            return fromVertex;
          }
        }
      }
    }

    return noVertex;
  }

  inline Vertex edgeTail(const int e) const { return edgeTail(Edge(e)); }

  inline Vertex edgeTail(const Edge edge) const {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");

    if constexpr (graph.HasEdgeAttribute(FromVertex)) {
      return graph.get(FromVertex, edge);
    } else {
      // very greedy algorithm -> will not get called
      for (const Vertex fromVertex : graph.vertices()) {
        for (const Edge e : graph.EdgesFrom(fromVertex)) {
          if (e == edge) {
            return fromVertex;
          }
        }
      }
    }

    return noVertex;
  }

  inline int travelTime(const int e) { return travelTime(Edge(e)); }

  inline int travelTime(const Edge edge) {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(TravelTime),
              "Missing graph attribute TravelTime");

    return graph.get(TravelTime, edge);
  }

  inline int travelTime(const int e) const { return travelTime(Edge(e)); }

  inline int travelTime(const Edge edge) const {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(TravelTime),
              "Missing graph attribute TravelTime");

    return graph.get(TravelTime, edge);
  }

  inline int traversalCost(const int e) { return traversalCost(Edge(e)); }

  inline int traversalCost(const Edge edge) {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(Weight), "Missing graph attribute Weight");

    return graph.get(Weight, edge);
  }

  inline int traversalCost(const int e) const { return traversalCost(Edge(e)); }

  inline int traversalCost(const Edge edge) const {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(Weight), "Missing graph attribute Weight");

    return graph.get(Weight, edge);
  }

  inline int edgeId(const int e) { return edgeId(Edge(e)); }

  inline int edgeId(const Edge edge) {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(EdgeId), "Missing graph attribute EdgeId");

    return graph.get(EdgeId, edge);
  }

  inline int edgeId(const int e) const { return edgeId(Edge(e)); }

  inline int edgeId(const Edge edge) const {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(EdgeId), "Missing graph attribute EdgeId");

    return graph.get(EdgeId, edge);
  }

  inline LatLng latLng(const int v) { return latLng(Vertex(v)); }

  inline LatLng latLng(const Vertex vertex) {
    AssertMsg(graph.isVertex(vertex), vertex << " is not a valid vertex!");
    AssertMsg(graph.HasEdgeAttribute(Coordinates),
              "Missing graph attribute Coordinates");

    Geometry::Point &coordinate = graph.get(Coordinates, vertex);
    return LatLng(coordinate.latitude, coordinate.longitude);
  }

  inline LatLng latLng(const int v) const { return latLng(Vertex(v)); }

  inline LatLng latLng(const Vertex vertex) const {
    AssertMsg(graph.isVertex(vertex), vertex << " is not a valid vertex!");
    AssertMsg(graph.HasEdgeAttribute(Coordinates),
              "Missing graph attribute Coordinates");

    Geometry::Point &coordinate = graph.get(Coordinates, vertex);
    return LatLng(coordinate.latitude, coordinate.longitude);
  }

  inline std::pair<int, int> unpackingInfo(const int e) {
    return unpackingInfo(Edge(e));
  }

  inline std::pair<int, int> unpackingInfo(const Edge edge) {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(UnpackingInfo),
              "Missing graph attribute UnpackingInfo");

    return graph.get(UnpackingInfo, edge);
  }

  const std::pair<int, int> unpackingInfo(const int e) const {
    return unpackingInfo(Edge(e));
  }

  const std::pair<int, int> unpackingInfo(const Edge edge) const {
    AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
    AssertMsg(graph.HasEdgeAttribute(UnpackingInfo),
              "Missing graph attribute UnpackingInfo");

    return graph.get(UnpackingInfo, edge);
  }

  GraphWrapper<GRAPH> getReverseGraph() const {
    GRAPH reverseGraph;
    Graph::copy(graph, reverseGraph);
    reverseGraph.revert();
    return GraphWrapper<GRAPH>(reverseGraph);
  }

  const GRAPH &getGraph() const noexcept { return graph; }

 private:
  // Generating FromVertex/EdgeId Attributes
  void generateNeededAttributes() {
    for (const Vertex v : graph.vertices()) {
      for (const Vertex u : graph.outgoingNeighbors(v)) {
        const Edge e = graph.findEdge(v, u);
        graph.set(FromVertex, e, v);
        graph.set(EdgeId, e, e);
      }
    }
  }

 private:
  GRAPH graph;
};

#define FORALL_VERTICES(G, u) for (const Vertex v : G.vertices())
#define FORALL_EDGES(G, e) for (const Edge e : G.edges())
#define FORALL_INCIDENT_EDGES(G, u, e) for (const Edge e : G.edgesFrom(u))
