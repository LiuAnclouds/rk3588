#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rknn_api.h"

static std::vector<uint8_t> read_file(const std::string& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("failed to open model: " + path);
  }
  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> data(static_cast<size_t>(size));
  if (!file.read(reinterpret_cast<char*>(data.data()), size)) {
    throw std::runtime_error("failed to read model: " + path);
  }
  return data;
}

static const char* tensor_type_name(rknn_tensor_type type) {
  switch (type) {
    case RKNN_TENSOR_FLOAT32:
      return "float32";
    case RKNN_TENSOR_FLOAT16:
      return "float16";
    case RKNN_TENSOR_INT8:
      return "int8";
    case RKNN_TENSOR_UINT8:
      return "uint8";
    case RKNN_TENSOR_INT16:
      return "int16";
    default:
      return "other";
  }
}

static void print_attr(const char* prefix, const rknn_tensor_attr& attr) {
  std::cout << prefix << "[" << attr.index << "] name=" << attr.name
            << " dims=[";
  for (uint32_t i = 0; i < attr.n_dims; ++i) {
    if (i) std::cout << ",";
    std::cout << attr.dims[i];
  }
  std::cout << "] n_elems=" << attr.n_elems << " size=" << attr.size
            << " fmt=" << attr.fmt << " type=" << tensor_type_name(attr.type)
            << " qnt=" << attr.qnt_type << " scale=" << attr.scale
            << " zp=" << attr.zp << "\n";
}

int main(int argc, char** argv) {
  const std::string model_path =
      argc > 1 ? argv[1]
               : "/home/orangepi/moonxkj/mine_pedestrian/models/"
                 "yolov8s_seg_rk3588_fp.rknn";

  try {
    const auto model = read_file(model_path);

    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, const_cast<uint8_t*>(model.data()), model.size(),
                        0, nullptr);
    if (ret != RKNN_SUCC) {
      std::cerr << "rknn_init failed: " << ret << "\n";
      return 2;
    }

    rknn_sdk_version version;
    std::memset(&version, 0, sizeof(version));
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(version));
    if (ret == RKNN_SUCC) {
      std::cout << "api_version=" << version.api_version
                << " drv_version=" << version.drv_version << "\n";
    }

    rknn_input_output_num io_num;
    std::memset(&io_num, 0, sizeof(io_num));
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC) {
      std::cerr << "RKNN_QUERY_IN_OUT_NUM failed: " << ret << "\n";
      rknn_destroy(ctx);
      return 3;
    }
    std::cout << "inputs=" << io_num.n_input
              << " outputs=" << io_num.n_output << "\n";

    std::vector<rknn_tensor_attr> input_attrs(io_num.n_input);
    for (uint32_t i = 0; i < io_num.n_input; ++i) {
      std::memset(&input_attrs[i], 0, sizeof(rknn_tensor_attr));
      input_attrs[i].index = i;
      ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attrs[i],
                       sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC) {
        std::cerr << "RKNN_QUERY_INPUT_ATTR failed: " << ret << "\n";
        rknn_destroy(ctx);
        return 4;
      }
      print_attr("input", input_attrs[i]);
    }

    for (uint32_t i = 0; i < io_num.n_output; ++i) {
      rknn_tensor_attr attr;
      std::memset(&attr, 0, sizeof(attr));
      attr.index = i;
      ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &attr,
                       sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC) {
        std::cerr << "RKNN_QUERY_OUTPUT_ATTR failed: " << ret << "\n";
        rknn_destroy(ctx);
        return 5;
      }
      print_attr("output", attr);
    }

    if (io_num.n_input != 1) {
      std::cerr << "expected one input, got " << io_num.n_input << "\n";
      rknn_destroy(ctx);
      return 6;
    }

    const auto& input_attr = input_attrs[0];
    uint32_t height = 640;
    uint32_t width = 640;
    uint32_t channels = 3;
    if (input_attr.n_dims == 4) {
      if (input_attr.fmt == RKNN_TENSOR_NHWC) {
        height = input_attr.dims[1];
        width = input_attr.dims[2];
        channels = input_attr.dims[3];
      } else {
        channels = input_attr.dims[1];
        height = input_attr.dims[2];
        width = input_attr.dims[3];
      }
    }
    const size_t input_size =
        static_cast<size_t>(height) * width * channels;
    std::vector<uint8_t> input_data(input_size, 114);

    rknn_input input;
    std::memset(&input, 0, sizeof(input));
    input.index = 0;
    input.buf = input_data.data();
    input.size = input_data.size();
    input.pass_through = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;

    ret = rknn_inputs_set(ctx, 1, &input);
    if (ret != RKNN_SUCC) {
      std::cerr << "rknn_inputs_set failed: " << ret << "\n";
      rknn_destroy(ctx);
      return 7;
    }

    ret = rknn_run(ctx, nullptr);
    if (ret != RKNN_SUCC) {
      std::cerr << "rknn_run failed: " << ret << "\n";
      rknn_destroy(ctx);
      return 8;
    }

    std::vector<rknn_output> outputs(io_num.n_output);
    for (auto& out : outputs) {
      std::memset(&out, 0, sizeof(out));
      out.want_float = 1;
    }
    ret = rknn_outputs_get(ctx, io_num.n_output, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) {
      std::cerr << "rknn_outputs_get failed: " << ret << "\n";
      rknn_destroy(ctx);
      return 9;
    }
    for (uint32_t i = 0; i < io_num.n_output; ++i) {
      std::cout << "output_runtime[" << i << "] size=" << outputs[i].size
                << "\n";
    }
    rknn_outputs_release(ctx, io_num.n_output, outputs.data());
    rknn_destroy(ctx);
    std::cout << "RKNN_SMOKE_OK\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
