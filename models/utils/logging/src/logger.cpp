#include "utils/logging/include/logger.hh"
#include <iostream>
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>
#include <arrow/csv/api.h>
#include "trick/parameter_types.h"

namespace gnc {

DataLogger::DataLogger() : log_interval(0.0), next_log_time(0.0), initialized(false) {
    time_builder_ptr = new arrow::DoubleBuilder();
    builders_ptrs = new std::vector<arrow::DoubleBuilder*>();
}

DataLogger::~DataLogger() {
    delete (arrow::DoubleBuilder*)time_builder_ptr;
    auto builders = (std::vector<arrow::DoubleBuilder*>*)builders_ptrs;
    for (auto b : *builders) delete b;
    delete builders;
}

void DataLogger::add_variable(const std::string& name, void* ptr, int type, const std::string& units) {
    variables.push_back({name, ptr, type, units});
    auto builders = (std::vector<arrow::DoubleBuilder*>*)builders_ptrs;
    builders->push_back(new arrow::DoubleBuilder());
}

void DataLogger::ensure_initialized() {
    if (initialized) return;
    initialized = true;
}

void DataLogger::update(double sim_time) {
    if (sim_time < next_log_time) return;
    
    ensure_initialized();

    auto time_builder = (arrow::DoubleBuilder*)time_builder_ptr;
    (void)time_builder->Append(sim_time);

    auto builders = (std::vector<arrow::DoubleBuilder*>*)builders_ptrs;
    for (size_t i = 0; i < variables.size(); ++i) {
        double val = 0.0;
        void* p = variables[i].ptr;
        
        switch (variables[i].type) {
            case TRICK_DOUBLE:             val = *(double*)p; break;
            case TRICK_FLOAT:              val = *(float*)p; break;
            case TRICK_INTEGER:            val = *(int*)p; break;
            case TRICK_UNSIGNED_INTEGER:   val = *(unsigned int*)p; break;
            case TRICK_SHORT:              val = *(short*)p; break;
            case TRICK_UNSIGNED_SHORT:     val = *(unsigned short*)p; break;
            case TRICK_LONG:               val = *(long*)p; break;
            case TRICK_UNSIGNED_LONG:      val = *(unsigned long*)p; break;
            case TRICK_LONG_LONG:          val = *(long long*)p; break;
            case TRICK_UNSIGNED_LONG_LONG: val = *(unsigned long long*)p; break;
            case TRICK_CHARACTER:          val = *(char*)p; break;
            case TRICK_UNSIGNED_CHARACTER: val = *(unsigned char*)p; break;
            case TRICK_BOOLEAN:            val = *(bool*)p ? 1.0 : 0.0; break;
            case TRICK_ENUMERATED:         val = *(int*)p; break;
            default:
                val = 0.0;
                break;
        }
        
        (void)(*builders)[i]->Append(val);
    }

    if (log_interval > 0) {
        next_log_time += log_interval;
    }
}

void DataLogger::write_to_file(const std::string& filename, Format format) {
    auto builders = (std::vector<arrow::DoubleBuilder*>*)builders_ptrs;
    if (variables.empty()) {
        std::cout << "[DataLogger] Warning: No data collected, file not written: " << filename << std::endl;
        return;
    }

    std::vector<std::shared_ptr<arrow::Array>> arrays;
    std::vector<std::shared_ptr<arrow::Field>> fields;
    
    // Preparation for Schema Metadata (Global)
    std::vector<std::string> meta_keys;
    std::vector<std::string> meta_values;

    // Time column
    std::shared_ptr<arrow::Array> time_array;
    (void)((arrow::DoubleBuilder*)time_builder_ptr)->Finish(&time_array);
    arrays.push_back(time_array);
    auto time_meta = arrow::key_value_metadata({"units"}, {"s"});
    fields.push_back(arrow::field("sim_time", arrow::float64(), time_meta));
    meta_keys.push_back("sim_time_units");
    meta_values.push_back("s");

    // Variable columns
    for (size_t i = 0; i < variables.size(); ++i) {
        std::shared_ptr<arrow::Array> var_array;
        (void)(*builders)[i]->Finish(&var_array);
        arrays.push_back(var_array);
        
        std::shared_ptr<arrow::KeyValueMetadata> metadata = nullptr;
        if (!variables[i].units.empty()) {
            metadata = arrow::key_value_metadata({"units"}, {variables[i].units});
            meta_keys.push_back(variables[i].name + "_units");
            meta_values.push_back(variables[i].units);
        }
        fields.push_back(arrow::field(variables[i].name, arrow::float64(), metadata));
    }

    auto schema_metadata = std::make_shared<arrow::KeyValueMetadata>(meta_keys, meta_values);
    auto schema = std::make_shared<arrow::Schema>(fields, schema_metadata);
    auto table = arrow::Table::Make(schema, arrays);

    if (format == PARQUET_FORMAT) {
        auto open_res = arrow::io::FileOutputStream::Open(filename);
        if (!open_res.ok()) {
            std::cerr << "[DataLogger] Error opening file for Parquet: " << open_res.status().ToString() << std::endl;
            return;
        }
        auto outfile = open_res.ValueOrDie();
        
        // Use ArrowWriterProperties to ensure schema is stored
        // Note: ArrowWriterProperties is in the 'parquet' namespace, NOT 'parquet::arrow'.
        auto arrow_props = parquet::ArrowWriterProperties::Builder().store_schema()->build();
        auto status = parquet::arrow::WriteTable(*table, arrow::default_memory_pool(), outfile, table->num_rows(), 
                                                 parquet::default_writer_properties(), arrow_props);
        
        if (status.ok()) {
            std::cout << "[DataLogger] Successfully wrote " << table->num_rows() << " rows to " << filename << " (Parquet)" << std::endl;
        } else {
            std::cerr << "[DataLogger] Error writing Parquet: " << status.ToString() << std::endl;
        }
    } else if (format == CSV_FORMAT) {
        auto open_res = arrow::io::FileOutputStream::Open(filename);
        if (!open_res.ok()) return;
        auto outfile = open_res.ValueOrDie();
        
        auto write_options = arrow::csv::WriteOptions::Defaults();
        auto status = arrow::csv::WriteCSV(*table, write_options, outfile.get());
        if (status.ok()) {
            std::cout << "[DataLogger] Successfully wrote " << table->num_rows() << " rows to " << filename << " (CSV)" << std::endl;
        }
    }
}

} // namespace gnc
