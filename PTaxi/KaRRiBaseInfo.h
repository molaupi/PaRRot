#pragma once
#include "KARRI/Algorithms/KaRRi/EllipticBCH/FeasiblePDLocs.h"
#include "KARRI/Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {
    struct KaRRiBaseInfo {
        PDLocs pdLocs;
        PDDistances pdDistances;

        FeasiblePDLocs feasiblePickups;
        FeasiblePDLocs feasibleDropoffs;

        // Filtered pickup and dropoff candidates for non-paired ordinary and PBNS assignments
        RelevantPDLocs relOrdinaryPickups;
        RelevantPDLocs relOrdinaryDropoffs;
        RelevantPDLocs relPickupsBeforeNextStop;
    };
}