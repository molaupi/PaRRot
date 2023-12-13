#include <sched.h>

#include "../Helpers/Console/CommandLineParser.h"
#include "../Shell/Shell.h"
#include "Commands/CH.h"
#include "Commands/KaRRi.h"

using namespace Shell;

int main(int argc, char** argv)
{
    CommandLineParser clp(argc, argv);
    checkAsserts();
    Shell::Shell shell;
    new RunRideRaptor(shell);
    new RunRideRaptorQueries(shell);
    new RunPreprocessing(shell);
    new RunMultiplePreprocessing(shell);
    new RunQueriesOnMultiplePreprocessings(shell);
    new BuildRideTransferGraph(shell);
    new BuildDistanceMatrix(shell);
    new BuildLoudCH(shell);
    new ExtendGraph(shell);
    new BuildCH(shell);
    new BuildCoreCH(shell);
    new RunRAPTORQueries(shell);
    new RunQueriesForEarliestArrival(shell);
    shell.run();
    return 0;
}
