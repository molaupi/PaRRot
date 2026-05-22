#include "../include/Common/Constants.h"
#include "Commands/CH.h"
#include "Commands/ULTRAPreprocessing.h"

#include "Commands/BenchmarkULTRA.h"
#include "Commands/BenchmarkMcULTRA.h"
#include "Commands/BenchmarkMultimodal.h"

#include "../include/ULTRA/Helpers/Console/CommandLineParser.h"
#include "../include/ULTRA/Helpers/MultiThreading.h"

#include "../include/ULTRA/Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    // CommandLineParser clp(argc, argv);
    // pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;
    new BuildCH(shell);
    new BuildCoreCH(shell);

    //Preprocessing
    new BuildFreeTransferGraph(shell);
    new ComputeStopToStopShortcuts(shell);
    new ComputeMcStopToStopShortcuts(shell);
    new ComputeMultimodalMcStopToStopShortcuts(shell);
    new RAPTORToTripBased(shell);
    new ComputeEventToEventShortcuts(shell);
    new ComputeDelayEventToEventShortcuts(shell);
    new ComputeMcEventToEventShortcuts(shell);
    new ComputeMultimodalMcEventToEventShortcuts(shell);
    new AugmentTripBasedShortcuts(shell);
    new ValidateStopToStopShortcuts(shell);
    new ValidateEventToEventShortcuts(shell);
    new TransformKaRRiRequestsToULTRAQueries(shell);

    //ULTRA
    new RunTransitiveCSAQueries(shell);
    new RunDijkstraCSAQueries(shell);
    new RunHLCSAQueries(shell);
    new RunULTRACSAQueries(shell);
    new RunTransitiveRAPTORQueries(shell);
    new RunDijkstraRAPTORQueries(shell);
    new RunHLRAPTORQueries(shell);
    new RunULTRARAPTORQueries(shell);
    new RunTransitiveTBQueries(shell);
    new RunULTRATBQueries(shell);
    new RunULTRARAPTORWithGivenQueries(shell);

    //McULTRA
    new RunTransitiveMcRAPTORQueries(shell);
    new RunMCRQueries(shell);
    new RunULTRAMcRAPTORQueries(shell);
    new RunULTRAMcTBQueries(shell);
    new RunTransitiveBoundedMcRAPTORQueries(shell);
    new RunUBMRAPTORQueries(shell);
    new RunUBMRAPTORWithGivenQueries(shell);
    new RunUBMTBQueries(shell);
    new RunUBMHydRAQueries(shell);
    new ComputeTransferTimeSavings(shell);

    //Multiple transfer modes
    new RunMultimodalMCRQueries(shell);
    new RunMultimodalULTRAMcRAPTORQueries(shell);
    new RunMultimodalUBMRAPTORQueries(shell);
    new RunMultimodalUBMHydRAQueries(shell);

    if (argc == 1) {
        // If no arguments, enter shell mode
        std::cout << "Entering interactive shell mode. Type 'help' for a list of available commands." << std::endl;
        shell.run();
    } else {
        // If arguments exist, they should be a valid ULTRA command and its parameters
        std::stringstream ss;
        for (int i = 1; i < argc; i++) {
            ss << argv[i];
            if (i < argc - 1) ss << " ";
        }
        const std::string line = ss.str();
        shell.printPrompt();
        shell << line << ::Shell::newLine;
        shell.interpretCommand(line);
    }
    return 0;
}
