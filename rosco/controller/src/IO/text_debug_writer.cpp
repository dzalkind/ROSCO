#include "debug_writer.hpp"
#include <fstream>
#include <cstdio>
#include <ctime>

static const char* ROSCO_VERSION_STR = "2.10.1";

// Text-based debug writer — writes column-formatted .dbg/.dbg2/.dbg3 files.
class TextDebugWriter : public DebugWriter {
public:
    void open(const std::string& filepath,
              const char* const* var_names,
              const char* const* var_units,
              int n_vars) override {
        file_.open(filepath);
        if (!file_.is_open()) return;

        // Header line 1: generation info
        time_t now = time(nullptr);
        struct tm* t = localtime(&now);
        char datebuf[32], timebuf[32];
        strftime(datebuf, sizeof(datebuf), "%d-%b-%Y", t);
        strftime(timebuf, sizeof(timebuf), "%H:%M:%S", t);
        file_ << " Generated on " << datebuf << " at "
               << timebuf << " using ROSCO-" << ROSCO_VERSION_STR << "\n";

        // Header line 2: variable names
        char hdr[32];
        snprintf(hdr, sizeof(hdr), "%20s", "Time");
        file_ << hdr;
        for (int i = 0; i < n_vars; i++) {
            snprintf(hdr, sizeof(hdr), "     %20s", var_names[i]);
            file_ << hdr;
        }
        file_ << "\n";

        // Header line 3: units
        snprintf(hdr, sizeof(hdr), "%20s", "(sec)");
        file_ << hdr;
        for (int i = 0; i < n_vars; i++) {
            snprintf(hdr, sizeof(hdr), "     %20s", var_units[i]);
            file_ << hdr;
        }
        file_ << "\n";
    }

    void write_row(double time, const double* data, int n) override {
        char buf[32];
        snprintf(buf, sizeof(buf), "%20.5f", time);
        file_ << buf;
        for (int i = 0; i < n; i++) {
            file_ << "     ";
            snprintf(buf, sizeof(buf), "%20.5E", data[i]);
            file_ << buf;
        }
        file_ << "\n";
    }

    void close() override {
        if (file_.is_open()) file_.close();
    }

    bool is_open() const override {
        return file_.is_open();
    }

private:
    std::ofstream file_;
};

// Factory implementation
std::unique_ptr<DebugWriter> DebugWriter::create(OutputFormat fmt) {
    switch (fmt) {
        case OutputFormat::HDF5:
#ifdef ROSCO_HDF5
            // HDF5 backend implemented in hdf5_debug_writer.cpp
            extern std::unique_ptr<DebugWriter> create_hdf5_writer();
            return create_hdf5_writer();
#else
            // Fall back to text if HDF5 not compiled in
            return std::make_unique<TextDebugWriter>();
#endif
        case OutputFormat::Text:
        default:
            return std::make_unique<TextDebugWriter>();
    }
}
