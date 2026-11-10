#pragma once

#include "fusionforce/simulation.hpp"

#include <iosfwd>
#include <string>

namespace fusionforce {

// Open file streams in binary mode to preserve CSV's CRLF record terminators.
class SimulationCsvWriter {
public:
    SimulationCsvWriter(std::ostream& output, bool lightweight);
    void write(const SimulationFrame& frame);

private:
    std::ostream& output_;
    bool lightweight_;
    std::string row_;
};

} // namespace fusionforce
