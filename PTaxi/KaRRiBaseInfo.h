#pragma once
#include "KARRI/Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {
    struct KaRRiBaseInfo {
        PDLocs pdLocs;
        PDDistances pdDistances;
        RelevantPDLocs relOrdinaryPickups;
        RelevantPDLocs relPickupsBeforeNextStop;
        RelevantPDLocs relOrdinaryDropoffs;
        RelevantPDLocs relDropoffsBeforeNextStop;
    };
}