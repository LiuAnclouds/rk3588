#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rknn_api.h"

namespace {

constexpr int kInputSize = 640;
constexpr int kNumClasses = 80;
constexpr int kPersonClass = 0;
constexpr int kRegMax = 16;
constexpr int kMaskDim = 32;
constexpr int kProtoSize = 160;
constexpr float kScoreThresh = 0.25f;
constexpr float kNmsThresh = 0.45f;
constexpr float kMaskThresh = 0.5f;
constexpr float kApproxHFovDeg = 70.0f;

struct LetterboxInfo {
  float scale = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
  int resized_w = 0;
  int resized_h = 0;
};

struct Candidate {
  cv::Rect2f box640;
  float score = 0.0f;
  int cls = 0;
  std::vector<float> mask_coeff;
};

struct Detection {
  cv::Rect box;
  float score = 0.0f;
  cv::Mat mask;
  cv::Point footpoint;
  std::string bearing_zone;
  std::string range_zone;
  float bearing_deg = 0.0f;
};

std::vector<uint8_t> read_file(const std::string& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) throw std::runtime_error("failed to open: " + path);
  const auto size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<uint8_t> data(static_cast<size_t>(size));
  if (!file.read(reinterpret_cast<char*>(data.data()), size)) {
    throw std::runtime_error("failed to read: " + path);
  }
  return data;
}

cv::Mat letterbox_rgb(const cv::Mat& bgr, LetterboxInfo* info) {
  const int src_w = bgr.cols;
  const int src_h = bgr.rows;
  const float scale =
      std::min(kInputSize / static_cast<float>(src_w),
               kInputSize / static_cast<float>(src_h));
  const int new_w = static_cast<int>(std::round(src_w * scale));
  const int new_h = static_cast<int>(std::round(src_h * scale));
  const int pad_x = (kInputSize - new_w) / 2;
  const int pad_y = (kInputSize - new_h) / 2;

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  cv::Mat canvas(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));

  cv::Mat rgb;
  cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

  info->scale = scale;
  info->pad_x = pad_x;
  info->pad_y = pad_y;
  info->resized_w = new_w;
  info->resized_h = new_h;
  return rgb;
}

float sigmoid(float x) {
  if (x >= 0) {
    const float z = std::exp(-x);
    return 1.0f / (1.0f + z);
  }
  const float z = std::exp(x);
  return z / (1.0f + z);
}

float dfl_value(const float* box_data, int side, int y, int x, int h, int w) {
  float max_v = -1e30f;
  float logits[kRegMax];
  for (int i = 0; i < kRegMax; ++i) {
    const int c = side * kRegMax + i;
    const float v = box_data[(c * h + y) * w + x];
    logits[i] = v;
    max_v = std::max(max_v, v);
  }

  float sum = 0.0f;
  float weighted = 0.0f;
  for (int i = 0; i < kRegMax; ++i) {
    const float e = std::exp(logits[i] - max_v);
    sum += e;
    weighted += e * static_cast<float>(i);
  }
  return weighted / std::max(sum, 1e-12f);
}

float iou(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float inter = (a & b).area();
  const float uni = a.area() + b.area() - inter;
  return uni <= 0.0f ? 0.0f : inter / uni;
}

std::vector<int> nms(const std::vector<Candidate>& candidates) {
  std::vector<int> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return candidates[a].score > candidates[b].score;
  });

  std::vector<int> keep;
  std::vector<uint8_t> removed(candidates.size(), 0);
  for (size_t oi = 0; oi < order.size(); ++oi) {
    const int idx = order[oi];
    if (removed[idx]) continue;
    keep.push_back(idx);
    for (size_t oj = oi + 1; oj < order.size(); ++oj) {
      const int other = order[oj];
      if (removed[other]) continue;
      if (iou(candidates[idx].box640, candidates[other].box640) >
          kNmsThresh) {
        removed[other] = 1;
      }
    }
  }
  return keep;
}

void add_branch_candidates(const float* box_data, const float* cls_data,
                           const float* coeff_data, int h, int w,
                           std::vector<Candidate>* candidates) {
  const float stride = kInputSize / static_cast<float>(h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float person_score = cls_data[(kPersonClass * h + y) * w + x];
      if (person_score < kScoreThresh) continue;

      const float l = dfl_value(box_data, 0, y, x, h, w);
      const float t = dfl_value(box_data, 1, y, x, h, w);
      const float r = dfl_value(box_data, 2, y, x, h, w);
      const float b = dfl_value(box_data, 3, y, x, h, w);

      const float cx = x + 0.5f;
      const float cy = y + 0.5f;
      float x1 = (cx - l) * stride;
      float y1 = (cy - t) * stride;
      float x2 = (cx + r) * stride;
      float y2 = (cy + b) * stride;

      x1 = std::clamp(x1, 0.0f, static_cast<float>(kInputSize - 1));
      y1 = std::clamp(y1, 0.0f, static_cast<float>(kInputSize - 1));
      x2 = std::clamp(x2, 0.0f, static_cast<float>(kInputSize - 1));
      y2 = std::clamp(y2, 0.0f, static_cast<float>(kInputSize - 1));
      if (x2 <= x1 || y2 <= y1) continue;

      Candidate cand;
      cand.box640 = cv::Rect2f(cv::Point2f(x1, y1), cv::Point2f(x2, y2));
      cand.score = person_score;
      cand.cls = kPersonClass;
      cand.mask_coeff.resize(kMaskDim);
      for (int k = 0; k < kMaskDim; ++k) {
        cand.mask_coeff[k] = coeff_data[(k * h + y) * w + x];
      }
      candidates->push_back(std::move(cand));
    }
  }
}

cv::Rect unletterbox_box(const cv::Rect2f& box640, const LetterboxInfo& lb,
                         const cv::Size& orig_size) {
  float x1 = (box640.x - lb.pad_x) / lb.scale;
  float y1 = (box640.y - lb.pad_y) / lb.scale;
  float x2 = (box640.x + box640.width - lb.pad_x) / lb.scale;
  float y2 = (box640.y + box640.height - lb.pad_y) / lb.scale;
  x1 = std::clamp(x1, 0.0f, static_cast<float>(orig_size.width - 1));
  y1 = std::clamp(y1, 0.0f, static_cast<float>(orig_size.height - 1));
  x2 = std::clamp(x2, 0.0f, static_cast<float>(orig_size.width - 1));
  y2 = std::clamp(y2, 0.0f, static_cast<float>(orig_size.height - 1));
  return cv::Rect(cv::Point(static_cast<int>(std::round(x1)),
                            static_cast<int>(std::round(y1))),
                  cv::Point(static_cast<int>(std::round(x2)),
                            static_cast<int>(std::round(y2))));
}

cv::Mat build_original_mask(const Candidate& cand, const float* proto,
                            const LetterboxInfo& lb,
                            const cv::Size& orig_size) {
  cv::Mat mask160(kProtoSize, kProtoSize, CV_32F);
  for (int y = 0; y < kProtoSize; ++y) {
    for (int x = 0; x < kProtoSize; ++x) {
      float v = 0.0f;
      const int offset = y * kProtoSize + x;
      for (int k = 0; k < kMaskDim; ++k) {
        v += cand.mask_coeff[k] * proto[k * kProtoSize * kProtoSize + offset];
      }
      mask160.at<float>(y, x) = sigmoid(v);
    }
  }

  cv::Mat mask640;
  cv::resize(mask160, mask640, cv::Size(kInputSize, kInputSize), 0, 0,
             cv::INTER_LINEAR);

  cv::Mat cropped640 = cv::Mat::zeros(kInputSize, kInputSize, CV_32F);
  cv::Rect crop = cand.box640 & cv::Rect2f(0, 0, kInputSize, kInputSize);
  cv::Rect crop_i(static_cast<int>(std::floor(crop.x)),
                  static_cast<int>(std::floor(crop.y)),
                  static_cast<int>(std::ceil(crop.width)),
                  static_cast<int>(std::ceil(crop.height)));
  crop_i &= cv::Rect(0, 0, kInputSize, kInputSize);
  if (crop_i.area() > 0) {
    mask640(crop_i).copyTo(cropped640(crop_i));
  }

  cv::Rect valid(lb.pad_x, lb.pad_y, lb.resized_w, lb.resized_h);
  valid &= cv::Rect(0, 0, kInputSize, kInputSize);

  cv::Mat mask_orig_f;
  cv::resize(cropped640(valid), mask_orig_f, orig_size, 0, 0, cv::INTER_LINEAR);

  cv::Mat mask_orig;
  cv::threshold(mask_orig_f, mask_orig, kMaskThresh, 255, cv::THRESH_BINARY);
  mask_orig.convertTo(mask_orig, CV_8U);
  return mask_orig;
}

cv::Point footpoint_from_mask_or_box(const cv::Mat& mask, const cv::Rect& box) {
  cv::Rect safe_box = box & cv::Rect(0, 0, mask.cols, mask.rows);
  if (safe_box.area() <= 0) {
    return cv::Point(box.x + box.width / 2, box.y + box.height);
  }

  int bottom_y = -1;
  int sum_x = 0;
  int count = 0;
  const int y_start = safe_box.y + safe_box.height - 1;
  const int y_end = std::max(safe_box.y, y_start - 12);
  for (int y = y_start; y >= y_end; --y) {
    const uint8_t* row = mask.ptr<uint8_t>(y);
    for (int x = safe_box.x; x < safe_box.x + safe_box.width; ++x) {
      if (row[x]) {
        bottom_y = std::max(bottom_y, y);
        sum_x += x;
        ++count;
      }
    }
    if (count > 0) break;
  }
  if (count > 0) {
    return cv::Point(sum_x / count, bottom_y);
  }
  return cv::Point(box.x + box.width / 2, box.y + box.height);
}

std::string bearing_zone(float x_norm) {
  if (x_norm < 0.33f) return "left";
  if (x_norm > 0.67f) return "right";
  return "front";
}

std::string range_zone(float y_norm) {
  if (y_norm > 0.72f) return "near";
  if (y_norm > 0.45f) return "mid";
  return "far";
}

cv::Scalar zone_color(const std::string& zone) {
  if (zone == "near") return cv::Scalar(40, 40, 230);
  if (zone == "mid") return cv::Scalar(30, 190, 230);
  return cv::Scalar(80, 220, 80);
}

void draw_detections(cv::Mat* image, const std::vector<Detection>& detections) {
  cv::Mat dimmed;
  image->convertTo(dimmed, -1, 0.72, 0);
  dimmed.copyTo(*image);

  const cv::Point camera_origin(image->cols / 2, image->rows - 1);
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& det = detections[i];
    const cv::Scalar color = zone_color(det.range_zone);

    cv::Mat color_layer(image->size(), image->type(), color);
    color_layer.copyTo(*image, det.mask);
    cv::addWeighted(*image, 0.78, color_layer, 0.22, 0, *image);

    cv::rectangle(*image, det.box, color, 2);
    cv::circle(*image, det.footpoint, 5, color, -1);
    cv::line(*image, camera_origin, det.footpoint, color, 2);

    std::ostringstream label;
    label << "person#" << (i + 1) << " " << std::fixed
          << std::setprecision(2) << det.score << " " << det.range_zone
          << " " << det.bearing_zone << " " << std::setprecision(1)
          << det.bearing_deg << "deg";
    int baseline = 0;
    cv::Size text_size =
        cv::getTextSize(label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.55, 1,
                        &baseline);
    const int tx = std::max(0, std::min(det.box.x, image->cols - text_size.width));
    const int ty = std::max(text_size.height + 4, det.box.y - 6);
    cv::rectangle(*image,
                  cv::Rect(tx, ty - text_size.height - 4,
                           text_size.width + 6, text_size.height + 8),
                  cv::Scalar(20, 20, 20), -1);
    cv::putText(*image, label.str(), cv::Point(tx + 3, ty),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
  }
}

class RknnModel {
 public:
  explicit RknnModel(const std::string& model_path) {
    model_data_ = read_file(model_path);
    int ret = rknn_init(&ctx_, model_data_.data(), model_data_.size(), 0, nullptr);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("rknn_init failed: " + std::to_string(ret));
    }
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("RKNN_QUERY_IN_OUT_NUM failed: " +
                               std::to_string(ret));
    }
    output_attrs_.resize(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
      std::memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
      output_attrs_[i].index = i;
      ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i],
                       sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC) {
        throw std::runtime_error("RKNN_QUERY_OUTPUT_ATTR failed: " +
                                 std::to_string(ret));
      }
    }
  }

  ~RknnModel() {
    if (ctx_ != 0) rknn_destroy(ctx_);
  }

  std::vector<Detection> infer(const cv::Mat& bgr) {
    LetterboxInfo lb;
    cv::Mat input_rgb = letterbox_rgb(bgr, &lb);

    rknn_input input;
    std::memset(&input, 0, sizeof(input));
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = input_rgb.total() * input_rgb.elemSize();
    input.buf = input_rgb.data;

    int ret = rknn_inputs_set(ctx_, 1, &input);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("rknn_inputs_set failed: " + std::to_string(ret));
    }
    ret = rknn_run(ctx_, nullptr);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("rknn_run failed: " + std::to_string(ret));
    }

    std::vector<rknn_output> outputs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
      std::memset(&outputs[i], 0, sizeof(rknn_output));
      outputs[i].index = i;
      outputs[i].want_float = 1;
    }
    ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) {
      throw std::runtime_error("rknn_outputs_get failed: " +
                               std::to_string(ret));
    }

    std::vector<Candidate> candidates;
    add_branch_candidates(reinterpret_cast<float*>(outputs[0].buf),
                          reinterpret_cast<float*>(outputs[1].buf),
                          reinterpret_cast<float*>(outputs[3].buf), 80, 80,
                          &candidates);
    add_branch_candidates(reinterpret_cast<float*>(outputs[4].buf),
                          reinterpret_cast<float*>(outputs[5].buf),
                          reinterpret_cast<float*>(outputs[7].buf), 40, 40,
                          &candidates);
    add_branch_candidates(reinterpret_cast<float*>(outputs[8].buf),
                          reinterpret_cast<float*>(outputs[9].buf),
                          reinterpret_cast<float*>(outputs[11].buf), 20, 20,
                          &candidates);

    const auto keep = nms(candidates);
    const float* proto = reinterpret_cast<float*>(outputs[12].buf);
    std::vector<Detection> detections;
    for (int idx : keep) {
      const Candidate& cand = candidates[idx];
      Detection det;
      det.box = unletterbox_box(cand.box640, lb, bgr.size());
      if (det.box.area() <= 0) continue;
      det.score = cand.score;
      det.mask = build_original_mask(cand, proto, lb, bgr.size());
      det.footpoint = footpoint_from_mask_or_box(det.mask, det.box);
      det.footpoint.x = std::clamp(det.footpoint.x, 0, bgr.cols - 1);
      det.footpoint.y = std::clamp(det.footpoint.y, 0, bgr.rows - 1);
      const float x_norm = det.footpoint.x / static_cast<float>(bgr.cols);
      const float y_norm = det.footpoint.y / static_cast<float>(bgr.rows);
      det.bearing_zone = bearing_zone(x_norm);
      det.range_zone = range_zone(y_norm);
      det.bearing_deg = (x_norm - 0.5f) * kApproxHFovDeg;
      detections.push_back(std::move(det));
    }

    rknn_outputs_release(ctx_, io_num_.n_output, outputs.data());
    return detections;
  }

 private:
  rknn_context ctx_ = 0;
  rknn_input_output_num io_num_{};
  std::vector<rknn_tensor_attr> output_attrs_;
  std::vector<uint8_t> model_data_;
};

void print_usage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " <model.rknn> <input_image> <output_image>\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    print_usage(argv[0]);
    return 1;
  }

  const std::string model_path = argv[1];
  const std::string image_path = argv[2];
  const std::string output_path = argv[3];

  try {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      throw std::runtime_error("failed to read image: " + image_path);
    }

    RknnModel model(model_path);
    const auto detections = model.infer(image);

    std::cout << "{ \"image\": \"" << image_path << "\", \"targets\": [";
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto& d = detections[i];
      if (i) std::cout << ",";
      std::cout << "\n  {\"id\":" << (i + 1)
                << ",\"type\":\"person\""
                << ",\"confidence\":" << std::fixed << std::setprecision(3)
                << d.score << ",\"bbox\":[" << d.box.x << "," << d.box.y
                << "," << (d.box.x + d.box.width) << ","
                << (d.box.y + d.box.height) << "]"
                << ",\"footpoint_px\":[" << d.footpoint.x << ","
                << d.footpoint.y << "]"
                << ",\"bearing_deg\":" << std::setprecision(1)
                << d.bearing_deg << ",\"bearing_zone\":\"" << d.bearing_zone
                << "\",\"range_zone\":\"" << d.range_zone
                << "\",\"mask_available\":true}";
    }
    if (!detections.empty()) std::cout << "\n";
    std::cout << "] }\n";

    draw_detections(&image, detections);
    if (!cv::imwrite(output_path, image)) {
      throw std::runtime_error("failed to write image: " + output_path);
    }
    std::cout << "saved=" << output_path << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 2;
  }
}
