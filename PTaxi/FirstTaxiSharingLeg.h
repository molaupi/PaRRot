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

        void runPALS(Request &request) {
            // BCH orig -> all stations
            // Individual BCH pruning ausschalten? 
            // Dropofss -> Stations
            // PDDistances überall raus -> Distances zu den stations
        }
        
        void runOrdinary(Request &request) {
            // Find relevant stations for pickup -> filterOrdinaryPickups & filterPickupsBeforeNextStop
            // Build buckets for stops of relevant vehicles
            // BCH stops of relevant vehicles -> all stations
            // DALS kann auch vorkommen
        }

        std::vector<int> &vertexIdOfStations; // Vertex IDs of the stations
        RelevantPdLocsFilterT &relevantPdLocsFilter; // Filter to find relevant stations as pickups
};

}