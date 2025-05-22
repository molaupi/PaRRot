#include <vector>
#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h>

namespace karri {
    
template <typename RelevantPdLocsFilterT>
class FirstTaxiSharingLeg {
    public:
        FirstTaxiSharingLeg(std::vector<int> &vertexIdOfStations,
                            RelevantPdLocsFilterT &relevantPdLocsFilter)
            : vertexIdOfStations(vertexIdOfStations), 
              relevantPdLocsFilter(relevantPdLocsFilter){}
    
    
        void runOneToMany(Request &request) {
            // 1. PALS
            runPALS(request);    
            // 2. Ordinary
            runOrdinary(request);
        }

    private:

        // Representation: Assignment + Stations
        // Best (cost/) earliest arrival for each station 1 assignment
        // TODO later: Pareto front for both criterium
        void runPALS(Request &request) {
            // BCH orig -> all stations
            // Individual BCH pruning ausschalten? 
            // Dropofss -> Stations
            // PDDistances überall raus -> Distances zu den stations
        }
        
        void runOrdinary(Request &request) {
            // Find relevant stops for pickup -> filterOrdinaryPickups & filterPickupsBeforeNextStop
            
            // Elliptic buckets for nachkommende stops of relevant vehicles
            // Buckets for stations (vor & rückwarts)
            // HubLabeling: Buckets abgleichen -> in Ellipse? or not
            // iterate the tree (CH upward search + 1pruning: no bucket -> prune, 2pruning: scan stations -> return false)

            // BCH stops of relevant vehicles -> all stations
            // DALS kann auch vorkommen
        }

        std::vector<int> &vertexIdOfStations; // Vertex IDs of the stations
        RelevantPdLocsFilterT &relevantPdLocsFilter; // Filter to find relevant stations as pickups
};

}