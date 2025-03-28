#pragma once

#include "PTAndTaxiTriple.h"


namespace karri {


    // Core of the PTaxi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<typename AssignmentFinderT>
    class PTAndTaxiTripFinder {

    public:
    
        PTAndTaxiTripFinder(AssignmentFinderT &assignmentFinder, RequestState &requestState)
                : assignmentFinder(assignmentFinder), requestState(requestState){}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // First taxi leg
            const RequestState &taxiOnlyResult = assignmentFinder.findBestAssignment(req);
            
            // Return the combined results
            return PTAndTaxiTriple(taxiOnlyResult, PTResult(false), requestState);
        }

    private:

        void initializeForRequest(const Request &req) {
            // initialize for PT requests
        }

        AssignmentFinderT &assignmentFinder;
        const RequestState &requestState;

    };
}