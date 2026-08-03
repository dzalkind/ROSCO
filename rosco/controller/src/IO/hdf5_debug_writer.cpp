#ifdef ROSCO_HDF5

#include "debug_writer.hpp"
#include <hdf5.h>
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>

// HDF5 debug writer — writes chunked, gzip-compressed datasets.
// Layout: one dataset per variable (1-D, extendible), plus a "Time" dataset.
// This gives good compression and allows individual variable reads.
class HDF5DebugWriter : public DebugWriter {
public:
    ~HDF5DebugWriter() override { close(); }

    void open(const std::string& filepath,
              const char* const* var_names,
              const char* const* var_units,
              int n_vars) override {
        n_vars_ = n_vars;

        // Create HDF5 file (overwrite if exists)
        file_id_ = H5Fcreate(filepath.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        if (file_id_ < 0) return;

        // Dataspace: 1-D unlimited
        hsize_t dims[1] = {0};
        hsize_t maxdims[1] = {H5S_UNLIMITED};
        hid_t space = H5Screate_simple(1, dims, maxdims);

        // Chunked dataset creation property with gzip compression
        hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
        hsize_t chunk[1] = {CHUNK_SIZE};
        H5Pset_chunk(dcpl, 1, chunk);
        H5Pset_deflate(dcpl, 1);  // gzip level 1 for speed

        // Create Time dataset
        hid_t time_ds = H5Dcreate2(file_id_, "Time", H5T_NATIVE_DOUBLE,
                                    space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
        // Attach unit attribute
        write_string_attr(time_ds, "units", "sec");
        datasets_.push_back(time_ds);

        // Create one dataset per variable
        for (int i = 0; i < n_vars; i++) {
            hid_t ds = H5Dcreate2(file_id_, var_names[i], H5T_NATIVE_DOUBLE,
                                   space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
            write_string_attr(ds, "units", var_units[i]);
            datasets_.push_back(ds);
        }

        H5Pclose(dcpl);
        H5Sclose(space);
        row_buffer_data_.resize(n_vars);
        open_ = true;
        row_count_ = 0;
    }

    void write_row(double time, const double* data, int n) override {
        if (!open_) return;

        // Buffer rows and flush in chunks
        row_buffer_time_.push_back(time);
        for (int i = 0; i < n; i++) {
            row_buffer_data_[i].push_back(data[i]);
        }
        row_count_++;

        if (row_buffer_time_.size() >= CHUNK_SIZE) {
            flush_buffer();
        }
    }

    void close() override {
        if (!open_) return;

        // Flush remaining buffered data
        if (!row_buffer_time_.empty()) {
            flush_buffer();
        }
        if (!avr_row_buffer_.empty()) {
            flush_avrswap_buffer();
        }
        if (avr_dataset_ >= 0) {
            H5Dclose(avr_dataset_);
            avr_dataset_ = -1;
        }

        for (auto ds : datasets_) {
            H5Dclose(ds);
        }
        datasets_.clear();
        H5Fclose(file_id_);
        file_id_ = -1;
        open_ = false;
    }

    bool is_open() const override { return open_; }

    // avrSWAP: single 2-D extendible dataset (rows x n_vars) in the same file,
    // with column labels ("AvrSWAP(1)".."AvrSWAP(N)") stored as an attribute.
    void open_avrswap(const char* const* column_labels, int n_vars) override {
        if (!open_) return;
        avr_n_vars_ = n_vars;

        hsize_t dims[2] = {0, (hsize_t)n_vars};
        hsize_t maxdims[2] = {H5S_UNLIMITED, (hsize_t)n_vars};
        hid_t space = H5Screate_simple(2, dims, maxdims);

        hid_t dcpl = H5Pcreate(H5P_DATASET_CREATE);
        hsize_t chunk[2] = {CHUNK_SIZE, (hsize_t)n_vars};
        H5Pset_chunk(dcpl, 2, chunk);
        H5Pset_deflate(dcpl, 1);

        avr_dataset_ = H5Dcreate2(file_id_, "avrSWAP", H5T_NATIVE_DOUBLE,
                                   space, H5P_DEFAULT, dcpl, H5P_DEFAULT);
        write_string_array_attr(avr_dataset_, "column_labels", column_labels, n_vars);

        H5Pclose(dcpl);
        H5Sclose(space);
        avr_row_count_ = 0;
        avr_row_buffer_.clear();
    }

    void write_avrswap_row(const double* data, int n) override {
        if (!open_ || avr_dataset_ < 0) return;
        avr_row_buffer_.insert(avr_row_buffer_.end(), data, data + n);
        avr_row_count_++;

        if (avr_row_buffer_.size() >= CHUNK_SIZE * (size_t)avr_n_vars_) {
            flush_avrswap_buffer();
        }
    }

private:
    static constexpr hsize_t CHUNK_SIZE = 512;

    hid_t file_id_ = -1;
    std::vector<hid_t> datasets_;  // [0] = Time, [1..n_vars] = variables
    int n_vars_ = 0;
    hsize_t row_count_ = 0;
    bool open_ = false;

    // Buffers for chunked writing
    std::vector<double> row_buffer_time_;
    std::vector<std::vector<double>> row_buffer_data_;

    // avrSWAP 2-D dataset state
    hid_t avr_dataset_ = -1;
    int avr_n_vars_ = 0;
    hsize_t avr_row_count_ = 0;
    std::vector<double> avr_row_buffer_;  // row-major, flattened

    void flush_avrswap_buffer() {
        if (avr_dataset_ < 0) return;
        hsize_t n_rows = avr_row_buffer_.size() / avr_n_vars_;
        if (n_rows == 0) return;
        hsize_t new_size[2] = {avr_row_count_, (hsize_t)avr_n_vars_};
        H5Dset_extent(avr_dataset_, new_size);

        hsize_t dims[2] = {n_rows, (hsize_t)avr_n_vars_};
        hid_t memspace = H5Screate_simple(2, dims, nullptr);
        hid_t filespace = H5Dget_space(avr_dataset_);
        hsize_t start[2] = {avr_row_count_ - n_rows, 0};
        hsize_t cnt[2] = {n_rows, (hsize_t)avr_n_vars_};
        H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, cnt, nullptr);
        H5Dwrite(avr_dataset_, H5T_NATIVE_DOUBLE, memspace, filespace, H5P_DEFAULT,
                 avr_row_buffer_.data());
        H5Sclose(filespace);
        H5Sclose(memspace);

        avr_row_buffer_.clear();
    }

    void flush_buffer() {
        hsize_t n_rows = row_buffer_time_.size();
        hsize_t new_size = row_count_;

        // Extend and write Time dataset
        H5Dset_extent(datasets_[0], &new_size);
        write_slab(datasets_[0], row_count_ - n_rows, n_rows, row_buffer_time_.data());

        // Extend and write each variable dataset
        for (int i = 0; i < n_vars_; i++) {
            H5Dset_extent(datasets_[i + 1], &new_size);
            write_slab(datasets_[i + 1], row_count_ - n_rows, n_rows,
                       row_buffer_data_[i].data());
        }

        row_buffer_time_.clear();
        for (auto& v : row_buffer_data_) v.clear();
    }

    void write_slab(hid_t dataset, hsize_t offset, hsize_t count, const double* buf) {
        hsize_t dims[1] = {count};
        hid_t memspace = H5Screate_simple(1, dims, nullptr);
        hid_t filespace = H5Dget_space(dataset);
        hsize_t start[1] = {offset};
        hsize_t cnt[1] = {count};
        H5Sselect_hyperslab(filespace, H5S_SELECT_SET, start, nullptr, cnt, nullptr);
        H5Dwrite(dataset, H5T_NATIVE_DOUBLE, memspace, filespace, H5P_DEFAULT, buf);
        H5Sclose(filespace);
        H5Sclose(memspace);
    }

    void write_string_attr(hid_t obj, const char* name, const char* value) {
        hid_t atype = H5Tcopy(H5T_C_S1);
        H5Tset_size(atype, strlen(value) + 1);
        hid_t aspace = H5Screate(H5S_SCALAR);
        hid_t attr = H5Acreate2(obj, name, atype, aspace, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(attr, atype, value);
        H5Aclose(attr);
        H5Sclose(aspace);
        H5Tclose(atype);
    }

    // Fixed-length string array attribute — used for avrSWAP column labels.
    void write_string_array_attr(hid_t obj, const char* name,
                                  const char* const* values, int n) {
        size_t maxlen = 1;
        for (int i = 0; i < n; i++) maxlen = std::max(maxlen, strlen(values[i]) + 1);

        std::vector<char> buf(maxlen * n, '\0');
        for (int i = 0; i < n; i++) {
            strncpy(&buf[i * maxlen], values[i], maxlen - 1);
        }

        hid_t atype = H5Tcopy(H5T_C_S1);
        H5Tset_size(atype, maxlen);
        hsize_t dims[1] = {(hsize_t)n};
        hid_t aspace = H5Screate_simple(1, dims, nullptr);
        hid_t attr = H5Acreate2(obj, name, atype, aspace, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(attr, atype, buf.data());
        H5Aclose(attr);
        H5Sclose(aspace);
        H5Tclose(atype);
    }
};

// Factory function called by DebugWriter::create()
std::unique_ptr<DebugWriter> create_hdf5_writer() {
    return std::make_unique<HDF5DebugWriter>();
}

#endif // ROSCO_HDF5
