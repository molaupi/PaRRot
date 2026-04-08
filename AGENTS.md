# AGENTS.md

## Scope and source of truth
- This file is for AI coding agents working in `PARROT/`.
- Existing AI-specific instruction files were not found in this repo (`AGENT.md`, `AGENTS.md`, `CLAUDE.md`, Copilot/Cursor/Windsurf/Cline rules); only `README.md` files exist.
- Trust build settings in `CMakeLists.txt`, `Runners/CMakeLists.txt`, `RawData/CMakeLists.txt`, `External/CMakeLists.txt` over older prose docs.

## Big picture architecture
- Main executable is `PTaxi` (`Runners/RunPTaxi.cpp`), orchestrating KaRRi ride-pooling + PT (ULTRA RAPTOR) + mode choice in one event simulation.
- Core algorithm code is mostly header-template based under `include/KARRI/**` and `PTaxi/**`.
- Runtime flow in `RunPTaxi.cpp`: parse CLI -> load vehicle/passenger graphs and fleet/requests -> build/load CH/CCH -> wire assignment/search components -> run `EventSimulation`.
- Taxi assignment strategies are composed explicitly: ORD + PBNS + PALS + DALS (`PTaxi/TaxiTripFinder.h`), with compile-time strategy switches.
- Combined taxi+PT journey logic is in `PTaxi/PTAndTaxiTripFinder.h`; plain PT-only journey logic is in `PTaxi/PTJourneyFinder.h`.

## Build and run workflows
- Configure/build from project root (single-generator workflow):
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
  - `cmake --build build --target PTaxi -j`
- External dependency note: `External/CMakeLists.txt` expects a prebuilt RoutingKit library (`External/RoutingKit/lib/libroutingkit.so`).
- If missing, bootstrap submodules + RoutingKit first:
  - `git submodule update --init --recursive`
  - `make -C External/RoutingKit lib/libroutingkit.so`
- Useful preprocessing targets live in `RawData/` (e.g., `BuildStaticBuckets`, `TransformLocations`, `TransformRequests`).
- `ULTRA_Runnables/` is a separate CMake project; do not assume it is part of the root build graph.

## Key runtime inputs and outputs
- `PTaxi` requires road graphs (`-veh-g`, `-psg-g`), requests (`-r`), vehicles (`-v`), RAPTOR data (`-raptor-data`), station mapping (`-station-mapping`), and station buckets (`-station-buckets`, `-psg-station-buckets`).
- CH/CCH behavior is compile-time + optional file input (`-veh-h`/`-psg-h` or `-veh-d`/`-psg-d`) in `RunPTaxi.cpp`.
- Logging uses `LogManager` (`include/KARRI/Tools/Logging/LogManager.h`) with base prefix set by `-o`; many CSVs are emitted via `stats::*::LOGGER_NAME` (e.g., `perf_overall.csv`, `perf_request_receive.csv`).

## Project-specific coding patterns
- Heavy use of compile-time flags from `Runners/KaRRi_compile_params_PTaxi.cmake` (e.g., `KARRI_PALS_STRATEGY`, `KARRI_DALS_STRATEGY`, `KARRI_USE_CCHS`, SIMD toggles). Keep new code compatible with both branches of `#if` switches.
- Assertion style is `KASSERT` with custom levels (`include/KARRI/Tools/custom_assertion_levels.h`): light in non-Debug, heavy in Debug.
- Time units are deciseconds in many paths (`* 10` in `RunPTaxi.cpp`); preserve unit conventions when adding fields/metrics.
- CSV parsing uses `fast-cpp-csv-parser` and often supports both default and LOUD schemas (`-csv-in-LOUD-format`).
- Prefer extending existing stats structs/log rows in `include/KARRI/Algorithms/KaRRi/Stats/PerformanceStats.h` instead of ad-hoc prints.

## Integration boundaries and dependencies
- External libs: RoutingKit, kassert, fast-cpp-csv-parser, vectorclass, nlohmann_json; system libs: OpenMP, PROJ.
- Python helper scripts (`prepare_csv.py`) require `pandas` (see `python_requirements.txt`).
- OSM preprocessing scripts in `RawData/Scripts/*.sh` expect `osmium` and specific folder conventions under a provided base directory.

## Practical agent guidance
- When modifying algorithms, trace both wiring (`RunPTaxi.cpp`) and the template implementation headers; behavior is rarely localized to one file.
- When changing CLI/input semantics, update `printUsage()` in `RunPTaxi.cpp` and keep CSV headers/schemas consistent.
- When adding performance metrics, update all three: struct fields, `LOGGER_COLS`, and `getLoggerRow()` in the corresponding stats struct.
- Validate compile-time option interactions by building at least one default config and one changed-strategy config (for example toggling `KARRI_PALS_STRATEGY`).

