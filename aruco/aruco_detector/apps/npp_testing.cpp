// File for playing around with NPP functionality.

#include <chrono>
#include <cstdlib>
#include <driver_types.h>
#include <iostream>
#include <string>
#include <thread>

#include <cuda_runtime.h>

#include <nppcore.h>
#include <nppi_arithmetic_and_logical_operations.h>
#include <nppi_color_conversion.h>
#include <nppi_filtering_functions.h>
#include <nppi_support_functions.h>
#include <nppi_threshold_and_compare_operations.h>

#include <opencv2/core/cuda.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

namespace {

inline void check_cuda(cudaError_t err, const char *context) {
  if (err != cudaSuccess) {
    std::cerr << "CUDA error: " << context << ": " << cudaGetErrorString(err)
              << '\n';
    std::exit(EXIT_FAILURE);
  }
}

inline void check_npp(NppStatus status, const char *context) {
  if (status != NPP_SUCCESS) {
    const std::string type{status < 0 ? "error" : "warning"};
    std::cerr << "NPP " << type << ": " << context << ": code " << status
              << '\n';
    std::exit(EXIT_FAILURE);
  }
}

template <typename T> inline T *check_alloc(T *ptr, const char *context) {
  if (!ptr) {
    std::cerr << "Memory allocation failure: " << context << '\n';
    std::exit(EXIT_FAILURE);
  }
  return ptr;
}

#define CHECK_CUDA(x) check_cuda(x, #x)
#define CHECK_NPP(x) check_npp(x, #x)
#define CHECK_ALLOC(x) check_alloc(x, #x)

class ImageProcessor {
public:
  ImageProcessor(NppStreamContext npp_stream_context, NppiSize image_size)
      : npp_stream_context_{npp_stream_context}, image_size_{image_size},
        device_color_image_{
            CHECK_ALLOC(nppiMalloc_8u_C3(image_size.width, image_size.height,
                                         &device_color_image_line_step_))},
        device_gray_image_{
            CHECK_ALLOC(nppiMalloc_8u_C1(image_size.width, image_size.height,
                                         &device_gray_image_line_step_))},
        device_filtered_image_{
            CHECK_ALLOC(nppiMalloc_8u_C1(image_size.width, image_size.height,
                                         &device_filtered_image_line_step_))},
        device_binary_images_{},
        readbacks_{
            cv::cuda::HostMem(image_size_.height, image_size_.width, CV_8UC1),
            cv::cuda::HostMem(image_size_.height, image_size_.width, CV_8UC1),
            cv::cuda::HostMem(image_size_.height, image_size_.width, CV_8UC1),
            cv::cuda::HostMem(image_size_.height, image_size_.width, CV_8UC1)}

  // device_label_image_{},
  // device_labeling_scratch_buffer_{},
  // device_label_compress_scratch_buffer_{}
  {
    // CHECK_CUDA(
    //     cudaMalloc(&device_label_image_,
    //                image_size.height * image_size.width * sizeof(Npp32u)));

    // int labeling_scratch_buffer_size{};
    // CHECK_NPP(nppiLabelMarkersUFGetBufferSize_32u_C1R(
    //     image_size_, &labeling_scratch_buffer_size));
    // CHECK_CUDA(cudaMalloc(&device_labeling_scratch_buffer_,
    //                       labeling_scratch_buffer_size));

    // int label_compress_scratch_buffer_size{};
    // CHECK_NPP(nppiCompressMarkerLabelsGetBufferSize_32u_C1R(
    //     image_size_.width * image_size.height,
    //     &label_compress_scratch_buffer_size));
    // CHECK_CUDA(cudaMalloc(&device_label_compress_scratch_buffer_,
    //                       label_compress_scratch_buffer_size));

    for (auto &image : device_binary_images_) {
      image = CHECK_ALLOC(nppiMalloc_8u_C1(image_size.width, image_size.height,
                                           &device_binary_image_line_step_));
    }
  }

  void process(cv::cuda::HostMem host_image) {
    // int max_label_id = 2000;
    // Npp32u *test{};
    // CHECK_CUDA(cudaMalloc(&test, max_label_id * sizeof(Npp32u)));
    // Npp32u *test2 = static_cast<Npp32u *>(
    //     CHECK_ALLOC(malloc(max_label_id * sizeof(Npp32u))));

    std::array<cudaStream_t, 4> streams{};
    for (auto &stream : streams) {
      CHECK_CUDA(cudaStreamCreate(&stream));
    }

    const auto clock_start{std::chrono::steady_clock::now()};
    CHECK_CUDA(cudaMemcpy2DAsync(
        device_color_image_, device_color_image_line_step_, host_image.data,
        host_image.step, 3 * image_size_.width, image_size_.height,
        cudaMemcpyHostToDevice, npp_stream_context_.hStream));

    // TODO: check bgr order
    const std::array<const Npp32f, 3> to_gray_coefficients{0.114F, 0.587F,
                                                           0.299F};
    CHECK_NPP(nppiColorToGray_8u_C3C1R_Ctx(
        device_color_image_, device_color_image_line_step_, device_gray_image_,
        device_gray_image_line_step_, image_size_, to_gray_coefficients.data(),
        npp_stream_context_));

#if 0
  {
    cv::Mat readback(image_size.height, image_size.width, CV_8UC1);
    CHECK_CUDA(cudaMemcpy2DAsync(
        readback.ptr(), readback.step[0], device_gray_image,
        device_gray_image_line_step, image_size.width, image_size.height,
        cudaMemcpyDeviceToHost, npp_stream_context.hStream));
    CHECK_CUDA(cudaStreamSynchronize(npp_stream_context.hStream));
    cv::imwrite("out_gray.png", readback);
  }
#endif

    CHECK_CUDA(cudaStreamSynchronize(npp_stream_context_.hStream));
    for (int i{0}; i < static_cast<int>(device_binary_images_.size()); ++i) {
      auto stream_context{npp_stream_context_};
      stream_context.hStream = streams[i];
      const int window_size{3 + 10 * i};
      constexpr Npp32f threshold_constant{7.0F};
      CHECK_NPP(nppiFilterThresholdAdaptiveBoxBorder_8u_C1R_Ctx(
          device_gray_image_, device_gray_image_line_step_, image_size_, {0, 0},
          device_binary_images_[i], device_binary_image_line_step_, image_size_,
          {window_size, window_size}, threshold_constant, 0, 255,
          NPP_BORDER_REPLICATE, stream_context));
    }
    // CHECK_NPP(nppiCopyReplicateBorder_8u_C1R_Ctx(
    //     device_gray_image, device_gray_image_line_step, image_size,
    //     device_extended_gray_image, device_extended_gray_image_line_step,
    //     extended_image_size, filter_size / 2, filter_size / 2,
    //     npp_stream_context));
    // constexpr int window_size{23};

    // CHECK_NPP(nppiFilterBoxBorder_8u_C1R_Ctx(
    //     device_gray_image_, device_gray_image_line_step_, image_size_, {0,
    //     0}, device_filtered_image_, device_filtered_image_line_step_,
    //     image_size_, {window_size, window_size}, {window_size / 2,
    //     window_size / 2}, NPP_BORDER_REPLICATE, npp_stream_context_));
    // CHECK_NPP(nppiSubC_8u_C1IRSfs_Ctx(
    //     threshold_constant, device_filtered_image_,
    //     device_filtered_image_line_step_, image_size_, 1,
    //     npp_stream_context_));
    // CHECK_NPP(nppiCompare_8u_C1R_Ctx(
    //     device_gray_image_, device_gray_image_line_step_,
    //     device_filtered_image_, device_filtered_image_line_step_,
    //     device_binary_image_, device_binary_image_line_step_, image_size_,
    //     NPP_CMP_LESS_EQ, npp_stream_context_));

#if 1
    {
      // CHECK_CUDA(cudaStreamSynchronize(npp_stream_context_.hStream));
      for (int i{0}; i < static_cast<int>(device_binary_images_.size()); ++i) {
        CHECK_CUDA(cudaMemcpy2DAsync(
            readbacks_[i].data, readbacks_[i].step, device_binary_images_[i],
            device_binary_image_line_step_, image_size_.width,
            image_size_.height, cudaMemcpyDeviceToHost, streams[i]));
      }
      CHECK_CUDA(cudaMemcpy2DAsync(
          readbacks_[3].data, readbacks_[3].step, device_gray_image_,
          device_gray_image_line_step_, image_size_.width, image_size_.height,
          cudaMemcpyDeviceToHost, streams[3]));
      for (auto &stream : streams) {
        CHECK_CUDA(cudaStreamSynchronize(stream));
        CHECK_CUDA(cudaStreamDestroy(stream));
      }

      const auto clock_end{std::chrono::steady_clock::now()};
      std::cout << "Time: "
                << std::chrono::duration_cast<std::chrono::microseconds>(
                       clock_end - clock_start)
                       .count()
                << " µs \n";

      for (int i{0}; i < static_cast<int>(device_binary_images_.size()) + 1; ++i) {
        cv::imshow("out" + std::to_string(i), readbacks_[i].createMatHeader());
      }
      cv::waitKey(1);
      // cv::imwrite("out_binary.png", readback);
    }
#endif

    // TODO: correct norm
    // CHECK_NPP(nppiLabelMarkersUF_8u32u_C1R_Ctx(
    //     device_binary_image_, device_binary_image_line_step_,
    //     device_label_image_, image_size_.width * sizeof(Npp32u), image_size_,
    //     nppiNormL1, device_labeling_scratch_buffer_, npp_stream_context_));
    // // int max_label_id{};
    // CHECK_NPP(nppiCompressMarkerLabelsUF_32u_C1IR_Ctx(
    //     device_label_image_, image_size_.width * sizeof(Npp32u), image_size_,
    //     image_size_.width * image_size_.height, &max_label_id,
    //     device_label_compress_scratch_buffer_, npp_stream_context_));
    // std::cout << max_label_id << '\n';

    // unsigned int info_size{};
    // CHECK_NPP(nppiCompressedMarkerLabelsUFGetInfoListSize_32u_C1R(max_label_id,
    //                                                               &info_size));
    // std::cout << info_size << '\n';
    // NppiCompressedMarkerLabelsInfo *device_marker_info{};
    // CHECK_CUDA(cudaMalloc(&device_marker_info, info_size));

    // NppiContourTotalsInfo contour_totals{};
    // CHECK_NPP(nppiCompressedMarkerLabelsUFInfo_32u_C1R_Ctx(
    //     device_label_image_, image_size_.width * sizeof(Npp32u), image_size_,
    //     max_label_id, device_marker_info, nullptr, 0, nullptr, 0,
    //     nullptr /*&contour_totals*/, test, test2, nullptr, nullptr,
    //     npp_stream_context_));
    // std::cout << contour_totals.nTotalImagePixelContourCount << '\n';
  }

private:
  NppStreamContext npp_stream_context_;
  NppiSize image_size_;
  int device_color_image_line_step_{};
  int device_gray_image_line_step_{};
  // TODO: drop?
  int device_filtered_image_line_step_{};
  int device_binary_image_line_step_{};
  Npp8u *device_color_image_;
  Npp8u *device_gray_image_;
  Npp8u *device_filtered_image_;
  std::array<Npp8u *, 3> device_binary_images_;
  std::array<cv::cuda::HostMem, 4> readbacks_;

  // Npp32u *device_label_image_;
  // Npp8u *device_labeling_scratch_buffer_;
  // Npp8u *device_label_compress_scratch_buffer_;
};

} // namespace

int main() {
  NppStreamContext npp_stream_context{};
  // TODO(vainiovano): Make use of streams?
  npp_stream_context.hStream = 0;
  if (cudaGetDevice(&npp_stream_context.nCudaDeviceId) != cudaSuccess) {
    std::cerr << "No devices supporting CUDA found\n";
    return EXIT_FAILURE;
  }

  CHECK_CUDA(cudaDeviceGetAttribute(
      &npp_stream_context.nCudaDevAttrComputeCapabilityMajor,
      cudaDevAttrComputeCapabilityMajor, npp_stream_context.nCudaDeviceId));

  CHECK_CUDA(cudaDeviceGetAttribute(
      &npp_stream_context.nCudaDevAttrComputeCapabilityMinor,
      cudaDevAttrComputeCapabilityMinor, npp_stream_context.nCudaDeviceId));
  CHECK_CUDA(cudaStreamGetFlags(npp_stream_context.hStream,
                                &npp_stream_context.nStreamFlags));

  cudaDeviceProp device_properties;
  CHECK_CUDA(cudaGetDeviceProperties(&device_properties,
                                     npp_stream_context.nCudaDeviceId));

  npp_stream_context.nMultiProcessorCount =
      device_properties.multiProcessorCount;
  npp_stream_context.nMaxThreadsPerMultiProcessor =
      device_properties.maxThreadsPerMultiProcessor;
  npp_stream_context.nMaxThreadsPerBlock = device_properties.maxThreadsPerBlock;
  npp_stream_context.nSharedMemPerBlock = device_properties.sharedMemPerBlock;

  cv::VideoCapture capture{"../aruco/aruco/real_world_1/oakd-testvideo.mp4",
                           cv::CAP_FFMPEG};
  const NppiSize image_size{
      static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT))};
  // const auto host_image{cv::imread("test.jpg")};
  // const NppiSize image_size{host_image.size[1], host_image.size[0]};
  ImageProcessor processor{npp_stream_context, image_size};

  cv::cuda::HostMem host_image;
  const auto start_time{std::chrono::system_clock::now()};
  while (true) {
    const bool got_frame{capture.read(host_image)};
    if (!got_frame) {
      break;
    }
    const std::chrono::nanoseconds target_after_start{
        1'000'000 *
        static_cast<std::int64_t>(capture.get(cv::CAP_PROP_POS_MSEC))};
    std::this_thread::sleep_until(start_time + target_after_start);
    processor.process(host_image);
  }
}
