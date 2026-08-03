#ifndef ROSCO_DEBUG_WRITER_HPP
#define ROSCO_DEBUG_WRITER_HPP

#include <string>
#include <vector>
#include <memory>

// Output format enumeration matching CntrPar.OutputFormat values
enum class OutputFormat : int {
    Text = 0,
    HDF5 = 1
};

// Abstract base class for debug output writers.
// Implementations: TextDebugWriter (always available), HDF5DebugWriter (optional).
class DebugWriter {
public:
    virtual ~DebugWriter() = default;

    // Open output file and write metadata/headers.
    // var_names and var_units must have 'n_vars' elements.
    virtual void open(const std::string& filepath,
                      const char* const* var_names,
                      const char* const* var_units,
                      int n_vars) = 0;

    // Append one timestep row of data.
    virtual void write_row(double time, const double* data, int n) = 0;

    // Flush and close the output file.
    virtual void close() = 0;

    // Check if the writer is currently open.
    virtual bool is_open() const = 0;

    // Optional: write avrSWAP as a single 2-D dataset ("/avrSWAP") in the same
    // file, with column_labels stored as an attribute. No-op by default —
    // TextDebugWriter keeps the separate .dbg3 file handled directly in debug.cpp.
    virtual void open_avrswap(const char* const* column_labels, int n_vars) {}
    virtual void write_avrswap_row(const double* data, int n) {}

    // Factory: create the appropriate writer based on format.
    static std::unique_ptr<DebugWriter> create(OutputFormat fmt);

    // Resolve a requested format to one provided by this build.
    static OutputFormat effective_format(OutputFormat fmt);
};

#endif // ROSCO_DEBUG_WRITER_HPP
