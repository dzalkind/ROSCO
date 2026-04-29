// rosco_array.hpp — array view and owning parameter array types.
//
// ArrayView: lightweight non-owning view over a double array.
//   Intentionally trivial (no constructor, no destructor, plain data) so it
//   can be passed into GPU kernels (CUDA/HIP) without modification.
//
// ParamArray: owning parameter array that implicitly converts to ArrayView.
//   Stores data in a std::vector<double> on the host. At call sites, no
//   explicit wrapping is needed — ParamArray converts to ArrayView automatically:
//
//     interp1d(CntrPar.PC_GS_angles, CntrPar.PC_GS_KP, xq, ErrVar);
//
//   GPU migration path: copy storage to device memory, then construct an
//   ArrayView from the device pointer and pass it to the kernel directly.

#pragma once
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <vector>

// ---------------------------------------------------------------------------
// ArrayView — non-owning view, GPU-passable
// ---------------------------------------------------------------------------
struct ArrayView {
    double* data;
    int     size;

    // Element access — checked in debug builds (ASan), unchecked in release.
#ifdef NDEBUG
    double& operator[](int i)       { return data[i]; }
    double  operator[](int i) const { return data[i]; }
#else
    double& operator[](int i)       {
        if (!data || i < 0 || i >= size) {
            std::fprintf(stderr, "ArrayView: out-of-bounds access [%d] on array of size %d\n", i, size);
            std::abort();
        }
        return data[i];
    }
    double  operator[](int i) const {
        if (!data || i < 0 || i >= size) {
            std::fprintf(stderr, "ArrayView: out-of-bounds access [%d] on array of size %d\n", i, size);
            std::abort();
        }
        return data[i];
    }
#endif
};

// ---------------------------------------------------------------------------
// ParamArray — owning host array that behaves like ArrayView at call sites
// ---------------------------------------------------------------------------
struct ParamArray {
    std::vector<double> storage;

    // Interface used by the TOML loader (rosco_types_io.cpp)
    void   clear()                { storage.clear(); }
    void   push_back(double v)    { storage.push_back(v); }
    void   resize(size_t n)       { storage.resize(n); }
    int    size()  const          { return (int)storage.size(); }
    bool   empty() const          { return storage.empty(); }
    double* data()                { return storage.data(); }
    const double* data() const    { return storage.data(); }
    double& operator[](int i)     { return storage[i]; }
    double  operator[](int i) const { return storage[i]; }

    // Implicit conversion to ArrayView — allows passing ParamArray directly
    // wherever ArrayView is expected, with no extra syntax at call sites.
    operator ArrayView() const {
        return {const_cast<double*>(storage.data()), (int)storage.size()};
    }
};
