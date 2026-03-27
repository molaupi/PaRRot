/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <cstdint>

namespace karri::stats {

    struct IntermediateResultStats {

        static constexpr auto LOGGER_NAME = "intermediate_results.csv";
        static constexpr auto LOGGER_COLS =
            "cost,"
            "cost_1st_taxi_leg,"
            "cost_pt_leg,"
            "cost_2nd_taxi_leg,"
            "1st_taxi_leg_type,"
            "arrival_time\n";
    };

    
    struct FirstTaxiLegResultStats {

        static constexpr auto LOGGER_NAME = "first_taxi_leg_results.csv";
        static constexpr auto LOGGER_COLS = 
            "average_cost," 
            "average_arrival_time,"
            "pals_count,"
            "dals_count,"
            "dals_pbns_count,"
            "ordinary_count,"
            "pbns_count,"
            "undefined_count,"
            "valid_results_count\n";
    };

    struct PTResultStats {

        static constexpr auto LOGGER_NAME = "pt_results.csv";
        static constexpr auto LOGGER_COLS = 
            "pt_only_cost," 
            "pt_only_leg_count," 
            "pt_with_taxi_cost," 
            "pt_with_taxi_leg_count\n";
    };
}