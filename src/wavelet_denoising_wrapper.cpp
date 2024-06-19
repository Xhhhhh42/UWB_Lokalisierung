#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <vector>


namespace py = pybind11;

std::vector<float> wavelet_denoising( const std::vector<float> signal ) 
{
    py::scoped_interpreter guard{}; // Start the interpreter and keep it alive
    py::module wavelet = py::module::import("wavelet_denoising");

    py::array_t<float> signal_np(signal.size(), signal.data());
    py::array_t<float> result_np = wavelet.attr("wavelet_denoising")(signal_np).cast<py::array_t<float>>();
    
    std::vector<float> result(result_np.size());
    std::copy(result_np.data(), result_np.data() + result_np.size(), result.begin());
    return result;
}

PYBIND11_MODULE(wavelet_denoising_wrapper, m) {
    m.def("wavelet_denoising", &wavelet_denoising, "A function to denoise signal using wavelet");
}
