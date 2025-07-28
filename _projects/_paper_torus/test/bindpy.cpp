#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(my_module, m) {
    m.doc() = "pybind11 example plugin";

    // Bind the C++ add function to Python
    m.def("add", &add, "A function that adds two numbers");
}