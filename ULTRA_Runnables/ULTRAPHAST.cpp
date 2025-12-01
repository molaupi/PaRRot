#include "../include/Common/Constants.h"
#include "Commands/BenchmarkULTRAPHAST.h"

#include "../include/ULTRA/Helpers/Console/CommandLineParser.h"
#include "../include/ULTRA/Helpers/MultiThreading.h"

#include "../include/ULTRA/Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;

    new RunOneToAllDijkstraCSAQueriesToVertices(shell);
    new RunOneToManyDijkstraCSAQueriesToStops(shell);
    new RunUPCSAQueries(shell);
    new RunOneToAllDijkstraRAPTORQueriesToVertices(shell);
    new RunOneToManyDijkstraRAPTORQueriesToStops(shell);
    new RunUPRAPTORQueries(shell);
    new RunUPTBQueries(shell);
    new CreateBallTargetSets(shell);
    new BuildCoreCHForTargetSets(shell);
    new BuildUPCHForTargetSets(shell);
    new RunOneToManyDijkstraCSAQueriesToBall(shell);
    new RunUPCSAQueriesToBall(shell);
    new RunOneToManyDijkstraRAPTORQueriesToBall(shell);
    new RunUPRAPTORQueriesToBall(shell);

    shell.run();
    return 0;
}
