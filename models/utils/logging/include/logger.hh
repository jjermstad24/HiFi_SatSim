/*******************************************************************************
Purpose:
  Simulation data logger supporting CSV and Parquet formats using Apache Arrow.
  Enhanced to support multiple Trick data types and units.

Library dependencies:
  ((../src/logger.cpp))
*******************************************************************************/

#pragma once

#include <vector>
#include <string>

namespace gnc {

class DataLogger {
public:
    enum Format {
        CSV_FORMAT,
        PARQUET_FORMAT
    };

    struct VarInfo {
        std::string name;
        void* ptr;
        int type; // Trick TRICK_TYPE enum value
        std::string units;
    };

    DataLogger();
    ~DataLogger();

    /**
     * @brief Add a variable to be logged.
     * @param name Column name.
     * @param ptr Memory address of the variable.
     * @param type Trick data type (from TRICK_TYPE enum).
     * @param units Units of measurement.
     */
    void add_variable(const std::string& name, void* ptr, int type, const std::string& units = "");
    
    void update(double sim_time);
    void write_to_file(const std::string& filename, Format format);
    void set_log_interval(double interval) { log_interval = interval; }

private:
    double log_interval; 
    double next_log_time;

    std::vector<VarInfo> variables;

    void* time_builder_ptr;
    void* builders_ptrs; 

    bool initialized;
    void ensure_initialized();
};

} // namespace gnc
