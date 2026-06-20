#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "MvCameraControl.h"
#include "rknn_api.h"

namespace {

namespace fs = std::filesystem;

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
constexpr int kDisplayMaxWidth = 800;
constexpr int kJpegQuality = 70;

std::atomic<bool> g_running{true};

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
  std::array<float, kMaskDim> mask_coeff{};
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

struct InferTimes {
  double preprocess_ms = 0.0;
  double rknn_ms = 0.0;
  double postprocess_ms = 0.0;
};

struct FrameTimes {
  double grab_ms = 0.0;
  double draw_write_ms = 0.0;
  double frame_ms = 0.0;
};

struct VisualPacket {
  cv::Mat frame;
  std::vector<Detection> detections;
  InferTimes infer_times;
  FrameTimes frame_times;
  double fps = 0.0;
  int frame_id = 0;
};

struct StreamState {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<uchar> jpeg;
  int frame_id = -1;
  bool enabled = true;
};

struct VisualState {
  std::mutex mutex;
  std::condition_variable cv;
  VisualPacket packet;
  int frame_id = -1;
  std::atomic<double> encode_ms{0.0};
};

void signal_handler(int) {
  g_running = false;
}

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

std::string ip_to_string(unsigned int ip) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", (ip >> 24) & 0xff,
                (ip >> 16) & 0xff, (ip >> 8) & 0xff, ip & 0xff);
  return buf;
}

void print_error(const char* what, int ret) {
  std::cerr << what << " failed, ret=0x" << std::hex << ret << std::dec
            << "\n";
}

float sigmoid(float x) {
  if (x >= 0) {
    const float z = std::exp(-x);
    return 1.0f / (1.0f + z);
  }
  const float z = std::exp(x);
  return z / (1.0f + z);
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
  cv::resize(bgr, resized, cv::Size(new_w, new_h));
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
      if (iou(candidates[idx].box640, candidates[other].box640) > kNmsThresh) {
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
      const float score = cls_data[(kPersonClass * h + y) * w + x];
      if (score < kScoreThresh) continue;

      const float l = dfl_value(box_data, 0, y, x, h, w);
      const float t = dfl_value(box_data, 1, y, x, h, w);
      const float r = dfl_value(box_data, 2, y, x, h, w);
      const float b = dfl_value(box_data, 3, y, x, h, w);
      const float cx = x + 0.5f;
      const float cy = y + 0.5f;
      float x1 = std::clamp((cx - l) * stride, 0.0f,
                            static_cast<float>(kInputSize - 1));
      float y1 = std::clamp((cy - t) * stride, 0.0f,
                            static_cast<float>(kInputSize - 1));
      float x2 = std::clamp((cx + r) * stride, 0.0f,
                            static_cast<float>(kInputSize - 1));
      float y2 = std::clamp((cy + b) * stride, 0.0f,
                            static_cast<float>(kInputSize - 1));
      if (x2 <= x1 || y2 <= y1) continue;

      Candidate cand;
      cand.box640 = cv::Rect2f(cv::Point2f(x1, y1), cv::Point2f(x2, y2));
      cand.score = score;
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

cv::Mat build_box_mask(const Candidate& cand, const float* proto,
                       const cv::Rect& box) {
  if (box.empty()) return {};

  cv::Rect crop640(static_cast<int>(std::floor(cand.box640.x)),
                   static_cast<int>(std::floor(cand.box640.y)),
                   static_cast<int>(std::ceil(cand.box640.width)),
                   static_cast<int>(std::ceil(cand.box640.height)));
  crop640 &= cv::Rect(0, 0, kInputSize, kInputSize);
  if (crop640.empty()) return {};

  const float proto_scale = kProtoSize / static_cast<float>(kInputSize);
  int px1 = std::clamp(static_cast<int>(std::floor(crop640.x * proto_scale)), 0,
                       kProtoSize - 1);
  int py1 = std::clamp(static_cast<int>(std::floor(crop640.y * proto_scale)), 0,
                       kProtoSize - 1);
  int px2 = std::clamp(static_cast<int>(std::ceil((crop640.x + crop640.width) * proto_scale)),
                       px1 + 1, kProtoSize);
  int py2 = std::clamp(static_cast<int>(std::ceil((crop640.y + crop640.height) * proto_scale)),
                       py1 + 1, kProtoSize);

  cv::Mat mask_proto(py2 - py1, px2 - px1, CV_32F);
  for (int y = py1; y < py2; ++y) {
    float* dst = mask_proto.ptr<float>(y - py1);
    for (int x = px1; x < px2; ++x) {
      float v = 0.0f;
      const int offset = y * kProtoSize + x;
      for (int k = 0; k < kMaskDim; ++k) {
        v += cand.mask_coeff[k] * proto[k * kProtoSize * kProtoSize + offset];
      }
      dst[x - px1] = sigmoid(v);
    }
  }

  cv::Mat mask_box_f;
  cv::resize(mask_proto, mask_box_f, box.size(), 0, 0, cv::INTER_LINEAR);
  cv::Mat mask_box;
  cv::threshold(mask_box_f, mask_box, kMaskThresh, 255, cv::THRESH_BINARY);
  mask_box.convertTo(mask_box, CV_8U);
  return mask_box;
}

cv::Point footpoint_from_box_mask_or_box(const cv::Mat& mask, const cv::Rect& box) {
  if (mask.empty()) return cv::Point(box.x + box.width / 2, box.y + box.height);
  int bottom_y = -1;
  int sum_x = 0;
  int count = 0;
  const int y_start = mask.rows - 1;
  const int y_end = std::max(0, y_start - 12);
  for (int y = y_start; y >= y_end; --y) {
    const uint8_t* row = mask.ptr<uint8_t>(y);
    for (int x = 0; x < mask.cols; ++x) {
      if (row[x]) {
        bottom_y = std::max(bottom_y, y);
        sum_x += x;
        ++count;
      }
    }
    if (count > 0) break;
  }
  if (count > 0) return cv::Point(box.x + sum_x / count, box.y + bottom_y);
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

std::string zone_upper(const std::string& zone) {
  if (zone == "near") return "NEAR";
  if (zone == "mid") return "MID";
  if (zone == "far") return "FAR";
  return zone;
}

std::string bearing_upper(const std::string& bearing) {
  if (bearing == "left") return "LEFT";
  if (bearing == "front") return "FRONT";
  if (bearing == "right") return "RIGHT";
  return bearing;
}

void fill_rect_alpha(cv::Mat* image, const cv::Rect& rect,
                     const cv::Scalar& color, double alpha) {
  cv::Rect safe = rect & cv::Rect(0, 0, image->cols, image->rows);
  if (safe.empty()) return;
  cv::Mat roi = (*image)(safe);
  cv::Mat layer(roi.size(), roi.type(), color);
  cv::addWeighted(layer, alpha, roi, 1.0 - alpha, 0, roi);
}

void draw_text(cv::Mat* image, const std::string& text, cv::Point org,
               double scale, const cv::Scalar& color, int thickness = 1) {
  cv::putText(*image, text, org, cv::FONT_HERSHEY_SIMPLEX, scale, color,
              thickness, cv::LINE_AA);
}

void draw_label_box(cv::Mat* image, const std::string& title,
                    const std::string& detail, cv::Point anchor,
                    const cv::Scalar& accent) {
  int base1 = 0;
  int base2 = 0;
  cv::Size title_size =
      cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, 0.52, 2, &base1);
  cv::Size detail_size =
      cv::getTextSize(detail, cv::FONT_HERSHEY_SIMPLEX, 0.43, 1, &base2);
  const int width = std::max(title_size.width, detail_size.width) + 20;
  const int height = 48;
  int x = std::clamp(anchor.x, 8, std::max(8, image->cols - width - 8));
  int y = anchor.y - height;
  if (y < 66) y = anchor.y + 8;
  y = std::clamp(y, 66, std::max(66, image->rows - height - 86));
  cv::Rect box(x, y, width, height);
  fill_rect_alpha(image, box, cv::Scalar(8, 12, 15), 0.78);
  cv::rectangle(*image, box, cv::Scalar(55, 68, 76), 1, cv::LINE_AA);
  cv::line(*image, cv::Point(x, y), cv::Point(x, y + height), accent, 3, cv::LINE_AA);
  draw_text(image, title, cv::Point(x + 10, y + 20), 0.52,
            cv::Scalar(245, 248, 250), 2);
  draw_text(image, detail, cv::Point(x + 10, y + 39), 0.43,
            cv::Scalar(190, 204, 212), 1);
}

int overlap_area(const cv::Rect& rect, const std::vector<cv::Rect>& others) {
  int area = 0;
  for (const auto& other : others) area += (rect & other).area();
  return area;
}

void draw_side_card(cv::Mat* image, const Detection& det, size_t index,
                    const cv::Scalar& accent,
                    const std::vector<cv::Rect>& avoid_boxes,
                    std::vector<cv::Rect>* placed_cards, bool compact) {
  const int card_w = compact ? 176 : 258;
  const int card_h = compact ? 64 : 88;
  const int gap = 8;
  const int top_limit = 66;
  const int bottom_limit = std::max(top_limit, image->rows - 90 - card_h);

  struct CardCandidate {
    int x;
    int y;
    double score;
  };

  std::vector<CardCandidate> candidates;
  auto add_candidate = [&](int ideal_x, int ideal_y, int base_score) {
    int x = std::clamp(ideal_x, 8, std::max(8, image->cols - card_w - 8));
    int y = std::clamp(ideal_y, top_limit, bottom_limit);
    cv::Rect r(x, y, card_w, card_h);
    const int move_penalty = std::abs(x - ideal_x) + std::abs(y - ideal_y);
    const int box_overlap = overlap_area(r, avoid_boxes);
    const int card_overlap = placed_cards ? overlap_area(r, *placed_cards) : 0;
    double score = static_cast<double>(base_score) - move_penalty * 0.42 -
                   box_overlap * 0.055 - card_overlap * 0.18;
    if (card_overlap > 0) score -= 1500.0;
    candidates.push_back({x, y, score});
  };

  const int y_top = std::clamp(det.box.y + 8, top_limit, std::max(top_limit, bottom_limit));
  const int y_mid = std::clamp(det.box.y + det.box.height / 2 - card_h / 2,
                               top_limit, std::max(top_limit, bottom_limit));
  const int y_low = std::clamp(det.box.y + det.box.height - card_h - 8,
                               top_limit, std::max(top_limit, bottom_limit));
  const int x_right = det.box.x + det.box.width + gap;
  const int x_left = det.box.x - card_w - gap;
  const int x_center = det.box.x + det.box.width / 2 - card_w / 2;
  add_candidate(x_right, y_mid, 520);
  add_candidate(x_left, y_mid, 510);
  add_candidate(x_right, y_top, 470);
  add_candidate(x_left, y_top, 460);
  add_candidate(x_right, y_low, 430);
  add_candidate(x_left, y_low, 420);
  add_candidate(x_center, det.box.y - card_h - gap, 250);
  add_candidate(x_center, det.box.y + det.box.height + gap, 220);
  add_candidate(det.box.x + det.box.width - card_w - 10, y_top, 120);
  add_candidate(det.box.x + 10, y_top, 110);

  CardCandidate best{std::clamp(det.box.x - card_w - gap, 8,
                                 std::max(8, image->cols - card_w - 8)),
                     y_top, -100000.0};
  for (const auto& c : candidates) {
    if (c.score > best.score) best = c;
  }

  const int x = best.x;
  const int y = best.y;
  cv::Rect card(x, y, card_w, card_h);
  if (placed_cards) placed_cards->push_back(card);

  cv::Rect halo(x - 2, y - 2, card_w + 4, card_h + 4);
  fill_rect_alpha(image, halo, cv::Scalar(0, 0, 0), 0.32);
  fill_rect_alpha(image, card, cv::Scalar(4, 9, 12), 0.90);
  cv::rectangle(*image, card, cv::Scalar(116, 144, 156), 1, cv::LINE_AA);
  cv::line(*image, cv::Point(x, y), cv::Point(x + card_w, y), accent, 2, cv::LINE_AA);
  cv::rectangle(*image, cv::Rect(x + 1, y + 1, 6, card_h - 2), accent, -1);

  cv::Point box_anchor;
  cv::Point card_anchor;
  const int det_center_x = det.box.x + det.box.width / 2;
  const int card_center_x = x + card_w / 2;
  if (card_center_x >= det_center_x) {
    box_anchor = cv::Point(det.box.x + det.box.width,
                            std::clamp(y + card_h / 2, det.box.y, det.box.y + det.box.height));
    card_anchor = cv::Point(x, y + card_h / 2);
  } else {
    box_anchor = cv::Point(det.box.x,
                           std::clamp(y + card_h / 2, det.box.y, det.box.y + det.box.height));
    card_anchor = cv::Point(x + card_w, y + card_h / 2);
  }
  cv::line(*image, box_anchor, card_anchor, cv::Scalar(5, 10, 13), 5, cv::LINE_AA);
  cv::line(*image, box_anchor, card_anchor, accent, 2, cv::LINE_AA);
  cv::circle(*image, box_anchor, 5, cv::Scalar(5, 10, 13), -1, cv::LINE_AA);
  cv::circle(*image, box_anchor, 4, accent, -1, cv::LINE_AA);
  cv::circle(*image, card_anchor, 5, cv::Scalar(5, 10, 13), -1, cv::LINE_AA);
  cv::circle(*image, card_anchor, 4, accent, -1, cv::LINE_AA);

  std::ostringstream title;
  title << "P" << std::setw(2) << std::setfill('0') << (index + 1);
  draw_text(image, title.str(), cv::Point(x + 18, y + (compact ? 29 : 34)),
            compact ? 0.70 : 0.86,
            cv::Scalar(245, 249, 250), 2);

  std::ostringstream conf;
  conf << std::fixed << std::setprecision(0) << det.score * 100.0f << "%";
  int base = 0;
  cv::Size conf_size =
      cv::getTextSize(conf.str(), cv::FONT_HERSHEY_SIMPLEX,
                      compact ? 0.50 : 0.66, 2, &base);
  cv::Rect conf_box(x + card_w - conf_size.width - 30, y + 12,
                    conf_size.width + 20, compact ? 26 : 32);
  fill_rect_alpha(image, conf_box, accent, 0.64);
  cv::rectangle(*image, conf_box, accent, 1, cv::LINE_AA);
  draw_text(image, conf.str(), cv::Point(conf_box.x + 10, conf_box.y + (compact ? 19 : 23)),
            compact ? 0.50 : 0.66, cv::Scalar(255, 255, 255), 2);

  std::ostringstream status_line;
  status_line << zone_upper(det.range_zone) << "  " << bearing_upper(det.bearing_zone);
  std::ostringstream angle_line;
  angle_line << "ANGLE  "
               << std::fixed << std::setprecision(1) << det.bearing_deg << " deg";
  draw_text(image, status_line.str(), cv::Point(x + 18, y + (compact ? 54 : 65)),
            compact ? 0.48 : 0.62, cv::Scalar(226, 238, 242), 1);
  if (!compact) {
    draw_text(image, angle_line.str(), cv::Point(x + 18, y + 84), 0.54,
              cv::Scalar(226, 238, 242), 1);
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
    ret = rknn_set_core_mask(ctx_, RKNN_NPU_CORE_0_1_2);
    if (ret != RKNN_SUCC) {
      std::cerr << "warning: rknn_set_core_mask failed, ret=" << ret << "\n";
    }
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (ret != RKNN_SUCC) throw std::runtime_error("query io failed");
  }

  ~RknnModel() {
    if (ctx_ != 0) rknn_destroy(ctx_);
  }

  std::vector<Detection> infer(const cv::Mat& bgr, InferTimes* times) {
    const auto t_pre0 = std::chrono::steady_clock::now();
    LetterboxInfo lb;
    cv::Mat input_rgb = letterbox_rgb(bgr, &lb);
    const auto t_pre1 = std::chrono::steady_clock::now();
    times->preprocess_ms =
        std::chrono::duration<double, std::milli>(t_pre1 - t_pre0).count();

    rknn_input input;
    std::memset(&input, 0, sizeof(input));
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    input.size = input_rgb.total() * input_rgb.elemSize();
    input.buf = input_rgb.data;

    auto t_rknn0 = std::chrono::steady_clock::now();
    int ret = rknn_inputs_set(ctx_, 1, &input);
    if (ret != RKNN_SUCC) throw std::runtime_error("inputs_set failed");
    ret = rknn_run(ctx_, nullptr);
    if (ret != RKNN_SUCC) throw std::runtime_error("rknn_run failed");
    std::vector<rknn_output> outputs(io_num_.n_output);
    for (uint32_t i = 0; i < io_num_.n_output; ++i) {
      std::memset(&outputs[i], 0, sizeof(rknn_output));
      outputs[i].index = i;
      outputs[i].want_float = 1;
    }
    ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) throw std::runtime_error("outputs_get failed");
    auto t_rknn1 = std::chrono::steady_clock::now();
    times->rknn_ms =
        std::chrono::duration<double, std::milli>(t_rknn1 - t_rknn0).count();

    std::vector<Candidate> candidates;
    candidates.reserve(128);
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
      det.mask = build_box_mask(cand, proto, det.box);
      det.footpoint = footpoint_from_box_mask_or_box(det.mask, det.box);
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
    const auto t_post1 = std::chrono::steady_clock::now();
    times->postprocess_ms =
        std::chrono::duration<double, std::milli>(t_post1 - t_rknn1).count();
    return detections;
  }

 private:
  rknn_context ctx_ = 0;
  rknn_input_output_num io_num_{};
  std::vector<uint8_t> model_data_;
};

void draw_detections(cv::Mat* image, const std::vector<Detection>& detections,
                     double fps, const InferTimes& infer_times,
                     const FrameTimes& frame_times, int frame_id) {
  image->convertTo(*image, -1, 0.86, 2);
  const cv::Point camera_origin(image->cols / 2, image->rows - 1);
  int near_count = 0;
  int mid_count = 0;
  int far_count = 0;
  for (const auto& det : detections) {
    if (det.range_zone == "near") ++near_count;
    else if (det.range_zone == "mid") ++mid_count;
    else ++far_count;
  }

  fill_rect_alpha(image, cv::Rect(0, 0, image->cols, 58), cv::Scalar(8, 12, 15), 0.82);
  cv::line(*image, cv::Point(0, 58), cv::Point(image->cols, 58),
           cv::Scalar(70, 86, 96), 1, cv::LINE_AA);
  draw_text(image, "MINE PEDESTRIAN VISION", cv::Point(18, 36), 0.72,
            cv::Scalar(235, 245, 248), 2);

  std::ostringstream top;
  top << "FPS " << std::fixed << std::setprecision(1) << fps
      << "   RKNN " << infer_times.rknn_ms << "ms"
      << "   TARGETS " << detections.size()
      << "   FRAME " << frame_id;
  int baseline = 0;
  cv::Size top_size =
      cv::getTextSize(top.str(), cv::FONT_HERSHEY_SIMPLEX, 0.54, 1, &baseline);
  draw_text(image, top.str(),
            cv::Point(std::max(18, image->cols - top_size.width - 18), 36),
            0.54, cv::Scalar(85, 225, 235), 1);

  cv::line(*image, cv::Point(image->cols / 3, 58),
           cv::Point(image->cols / 3, image->rows), cv::Scalar(45, 58, 64), 1, cv::LINE_AA);
  cv::line(*image, cv::Point(image->cols * 2 / 3, 58),
           cv::Point(image->cols * 2 / 3, image->rows), cv::Scalar(45, 58, 64), 1, cv::LINE_AA);
  draw_text(image, "LEFT", cv::Point(image->cols / 6 - 24, image->rows - 18),
            0.5, cv::Scalar(130, 146, 154), 1);
  draw_text(image, "FRONT", cv::Point(image->cols / 2 - 30, image->rows - 18),
            0.5, cv::Scalar(130, 146, 154), 1);
  draw_text(image, "RIGHT", cv::Point(image->cols * 5 / 6 - 30, image->rows - 18),
            0.5, cv::Scalar(130, 146, 154), 1);

  std::vector<cv::Rect> avoid_boxes;
  avoid_boxes.reserve(detections.size());
  for (const auto& det : detections) {
    cv::Rect safe_box = det.box & cv::Rect(0, 0, image->cols, image->rows);
    if (!safe_box.empty()) avoid_boxes.push_back(safe_box);
  }

  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& det = detections[i];
    const cv::Scalar color = zone_color(det.range_zone);
    cv::Rect box = det.box & cv::Rect(0, 0, image->cols, image->rows);
    if (!box.empty() && !det.mask.empty()) {
      cv::Mat roi = (*image)(box);
      cv::Mat color_roi(roi.size(), roi.type(), color);
      cv::Mat blended;
      cv::addWeighted(roi, 0.54, color_roi, 0.46, 0, blended);
      blended.copyTo(roi, det.mask);
    }

    cv::rectangle(*image, det.box, color, 2, cv::LINE_AA);
    const int corner = 18;
    const cv::Point tl = det.box.tl();
    const cv::Point tr(det.box.x + det.box.width, det.box.y);
    const cv::Point bl(det.box.x, det.box.y + det.box.height);
    const cv::Point br(det.box.x + det.box.width, det.box.y + det.box.height);
    cv::line(*image, tl, tl + cv::Point(corner, 0), color, 4, cv::LINE_AA);
    cv::line(*image, tl, tl + cv::Point(0, corner), color, 4, cv::LINE_AA);
    cv::line(*image, tr, tr + cv::Point(-corner, 0), color, 4, cv::LINE_AA);
    cv::line(*image, tr, tr + cv::Point(0, corner), color, 4, cv::LINE_AA);
    cv::line(*image, bl, bl + cv::Point(corner, 0), color, 4, cv::LINE_AA);
    cv::line(*image, bl, bl + cv::Point(0, -corner), color, 4, cv::LINE_AA);
    cv::line(*image, br, br + cv::Point(-corner, 0), color, 4, cv::LINE_AA);
    cv::line(*image, br, br + cv::Point(0, -corner), color, 4, cv::LINE_AA);

    cv::circle(*image, det.footpoint, 6, cv::Scalar(8, 12, 15), -1, cv::LINE_AA);
    cv::circle(*image, det.footpoint, 5, color, -1, cv::LINE_AA);
  }

  std::vector<size_t> card_order(detections.size());
  std::iota(card_order.begin(), card_order.end(), 0);
  std::sort(card_order.begin(), card_order.end(),
            [&](size_t a, size_t b) {
              const int area_a = detections[a].box.area();
              const int area_b = detections[b].box.area();
              if (area_a != area_b) return area_a > area_b;
              return detections[a].score > detections[b].score;
            });

  const size_t ray_count = std::min<size_t>(detections.size(), detections.size() >= 4 ? 3 : detections.size());
  for (size_t rank = 0; rank < ray_count; ++rank) {
    const auto& det = detections[card_order[rank]];
    const cv::Scalar color = zone_color(det.range_zone);
    cv::line(*image, camera_origin, det.footpoint, cv::Scalar(5, 10, 13), 4, cv::LINE_AA);
    cv::line(*image, camera_origin, det.footpoint, color, 2, cv::LINE_AA);
    cv::circle(*image, det.footpoint, 6, cv::Scalar(8, 12, 15), -1, cv::LINE_AA);
    cv::circle(*image, det.footpoint, 5, color, -1, cv::LINE_AA);
  }

  std::vector<cv::Rect> placed_cards;
  placed_cards.reserve(detections.size());
  const bool compact_cards = detections.size() >= 4;
  for (size_t rank = 0; rank < card_order.size(); ++rank) {
    const size_t i = card_order[rank];
    const auto& det = detections[i];
    const bool compact = (compact_cards && rank >= 3) || det.box.width < image->cols / 9;
    draw_side_card(image, det, rank, zone_color(det.range_zone), avoid_boxes,
                   &placed_cards, compact);
  }

  const int panel_h = 78;
  fill_rect_alpha(image, cv::Rect(0, image->rows - panel_h, image->cols, panel_h),
                  cv::Scalar(8, 12, 15), 0.78);
  cv::line(*image, cv::Point(0, image->rows - panel_h),
           cv::Point(image->cols, image->rows - panel_h), cv::Scalar(70, 86, 96), 1, cv::LINE_AA);

  std::string state = detections.empty() ? "CLEAR" : "TARGET LOCK";
  cv::Scalar state_color = detections.empty() ? cv::Scalar(80, 220, 80) : cv::Scalar(30, 190, 230);
  if (near_count > 0) state_color = cv::Scalar(40, 40, 230);
  draw_text(image, state, cv::Point(18, image->rows - 42), 0.78, state_color, 2);

  std::ostringstream zones;
  zones << "NEAR " << near_count << "   MID " << mid_count << "   FAR " << far_count;
  draw_text(image, zones.str(), cv::Point(18, image->rows - 16), 0.52,
            cv::Scalar(218, 228, 232), 1);

  std::ostringstream timing;
  timing << "grab " << std::fixed << std::setprecision(1) << frame_times.grab_ms
         << "ms   pre " << infer_times.preprocess_ms
         << "ms   post " << infer_times.postprocess_ms
         << "ms   stream " << frame_times.draw_write_ms << "ms";
  cv::Size timing_size =
      cv::getTextSize(timing.str(), cv::FONT_HERSHEY_SIMPLEX, 0.48, 1, &baseline);
  draw_text(image, timing.str(),
            cv::Point(std::max(18, image->cols - timing_size.width - 18), image->rows - 20),
            0.48, cv::Scalar(180, 195, 204), 1);
}

std::string detections_json(const std::vector<Detection>& detections, double fps,
                            const InferTimes& infer_times,
                            const FrameTimes& frame_times, int frame_id) {
  std::ostringstream os;
  os << "{\n  \"frame_id\": " << frame_id << ",\n"
     << "  \"fps\": " << std::fixed << std::setprecision(2) << fps << ",\n"
     << "  \"infer_ms\": " << std::setprecision(2) << infer_times.rknn_ms << ",\n"
     << "  \"timing_ms\": {"
     << "\"grab\":" << std::setprecision(2) << frame_times.grab_ms
     << ",\"preprocess\":" << infer_times.preprocess_ms
     << ",\"rknn\":" << infer_times.rknn_ms
     << ",\"postprocess\":" << infer_times.postprocess_ms
     << ",\"draw_write\":" << frame_times.draw_write_ms
     << ",\"frame\":" << frame_times.frame_ms << "},\n"
     << "  \"targets\": [";
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& d = detections[i];
    if (i) os << ",";
    os << "\n    {\"id\":" << (i + 1) << ",\"type\":\"person\""
       << ",\"confidence\":" << std::setprecision(3) << d.score
       << ",\"bbox\":[" << d.box.x << "," << d.box.y << ","
       << (d.box.x + d.box.width) << "," << (d.box.y + d.box.height) << "]"
       << ",\"footpoint_px\":[" << d.footpoint.x << "," << d.footpoint.y
       << "]"
       << ",\"bearing_deg\":" << std::setprecision(1) << d.bearing_deg
       << ",\"bearing_zone\":\"" << d.bearing_zone << "\""
       << ",\"range_zone\":\"" << d.range_zone << "\""
       << ",\"mask_available\":true}";
  }
  if (!detections.empty()) os << "\n  ";
  os << "]\n}\n";
  return os.str();
}

bool frame_to_bgr(void* handle, const MV_FRAME_OUT& frame, cv::Mat* bgr) {
  const auto& info = frame.stFrameInfo;
  const unsigned int width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
  const unsigned int height = info.nExtendHeight ? info.nExtendHeight : info.nHeight;
  const unsigned int src_len =
      info.nFrameLenEx ? static_cast<unsigned int>(info.nFrameLenEx) : info.nFrameLen;

  if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
    cv::Mat tmp(height, width, CV_8UC3, frame.pBufAddr);
    tmp.copyTo(*bgr);
    return true;
  }
  if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
    cv::Mat rgb(height, width, CV_8UC3, frame.pBufAddr);
    cv::cvtColor(rgb, *bgr, cv::COLOR_RGB2BGR);
    return true;
  }
  if (info.enPixelType == PixelType_Gvsp_Mono8) {
    cv::Mat mono(height, width, CV_8UC1, frame.pBufAddr);
    cv::cvtColor(mono, *bgr, cv::COLOR_GRAY2BGR);
    return true;
  }

  std::vector<unsigned char> converted(width * height * 3);
  MV_CC_PIXEL_CONVERT_PARAM_EX cvt;
  std::memset(&cvt, 0, sizeof(cvt));
  cvt.nWidth = width;
  cvt.nHeight = height;
  cvt.enSrcPixelType = info.enPixelType;
  cvt.pSrcData = frame.pBufAddr;
  cvt.nSrcDataLen = src_len;
  cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  cvt.pDstBuffer = converted.data();
  cvt.nDstBufferSize = converted.size();
  int ret = MV_CC_ConvertPixelTypeEx(handle, &cvt);
  if (ret != MV_OK) {
    print_error("MV_CC_ConvertPixelTypeEx", ret);
    return false;
  }
  cv::Mat tmp(height, width, CV_8UC3, converted.data());
  tmp.copyTo(*bgr);
  return true;
}

class MvsCamera {
 public:
  explicit MvsCamera(unsigned int index) {
    int ret = MV_CC_Initialize();
    if (ret != MV_OK) throw std::runtime_error("MVS initialize failed");

    MV_CC_DEVICE_INFO_LIST devices;
    std::memset(&devices, 0, sizeof(devices));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
    if (ret != MV_OK) throw std::runtime_error("MVS enum failed");
    if (devices.nDeviceNum == 0 || index >= devices.nDeviceNum) {
      throw std::runtime_error("no camera found");
    }
    auto* dev = devices.pDeviceInfo[index];
    if (dev->nTLayerType == MV_GIGE_DEVICE) {
      std::cout << "camera=" << dev->SpecialInfo.stGigEInfo.chModelName
                << " ip=" << ip_to_string(dev->SpecialInfo.stGigEInfo.nCurrentIp)
                << "\n";
    }
    ret = MV_CC_CreateHandle(&handle_, dev);
    if (ret != MV_OK) throw std::runtime_error("MVS create handle failed");
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) throw std::runtime_error("MVS open failed");
    if (dev->nTLayerType == MV_GIGE_DEVICE) {
      int packet_size = MV_CC_GetOptimalPacketSize(handle_);
      if (packet_size > 0) MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet_size);
    }
    MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Continuous");
    MV_CC_SetEnumValueByString(handle_, "GainAuto", "Continuous");
    MV_CC_SetEnumValueByString(handle_, "BalanceWhiteAuto", "Continuous");
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) throw std::runtime_error("MVS start grabbing failed");
  }

  ~MvsCamera() {
    if (handle_) {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      MV_CC_Finalize();
    }
  }

  bool grab(cv::Mat* bgr, int timeout_ms) {
    MV_FRAME_OUT frame;
    std::memset(&frame, 0, sizeof(frame));
    int ret = MV_CC_GetImageBuffer(handle_, &frame, timeout_ms);
    if (ret != MV_OK) return false;
    bool ok = frame_to_bgr(handle_, frame, bgr);
    MV_CC_FreeImageBuffer(handle_, &frame);
    return ok;
  }

 private:
  void* handle_ = nullptr;
};

void atomic_write_text(const std::string& path, const std::string& text) {
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp);
    out << text;
  }
  std::rename(tmp.c_str(), path.c_str());
}

void atomic_write_image(const std::string& path, const cv::Mat& image) {
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
  cv::Mat out = image;
  cv::Mat resized;
  if (image.cols > kDisplayMaxWidth) {
    const double scale = kDisplayMaxWidth / static_cast<double>(image.cols);
    cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    out = resized;
  }
  const std::string tmp = path + ".tmp.jpg";
  cv::imwrite(tmp, out, params);
  std::rename(tmp.c_str(), path.c_str());
}

std::vector<uchar> encode_display_jpeg(const cv::Mat& image) {
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
  cv::Mat out = image;
  cv::Mat resized;
  if (image.cols > kDisplayMaxWidth) {
    const double scale = kDisplayMaxWidth / static_cast<double>(image.cols);
    cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    out = resized;
  }
  std::vector<uchar> jpeg;
  cv::imencode(".jpg", out, jpeg, params);
  return jpeg;
}

std::vector<std::string> collect_images(const std::string& image_dir) {
  std::vector<std::string> images;
  for (const auto& entry : fs::directory_iterator(image_dir)) {
    if (!entry.is_regular_file()) continue;
    std::string ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
      images.push_back(entry.path().string());
    }
  }
  std::sort(images.begin(), images.end());
  return images;
}

bool send_all(int fd, const void* data, size_t size) {
  const char* ptr = static_cast<const char*>(data);
  while (size > 0) {
    ssize_t sent = send(fd, ptr, size, MSG_NOSIGNAL);
    if (sent <= 0) return false;
    ptr += sent;
    size -= static_cast<size_t>(sent);
  }
  return true;
}

bool send_all(int fd, const std::string& text) {
  return send_all(fd, text.data(), text.size());
}

void close_fd(int fd) {
  if (fd >= 0) close(fd);
}

void handle_stream_client(int client_fd, StreamState* state) {
  char request[1024];
  recv(client_fd, request, sizeof(request), 0);

  const std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Server: mine-pedestrian-mjpeg\r\n"
      "Connection: close\r\n"
      "Cache-Control: no-cache, no-store, must-revalidate\r\n"
      "Pragma: no-cache\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  if (!send_all(client_fd, header)) {
    close_fd(client_fd);
    return;
  }

  int last_frame = -1;
  while (g_running) {
    std::vector<uchar> jpeg;
    int frame_id = -1;
    {
      std::unique_lock<std::mutex> lock(state->mutex);
      state->cv.wait_for(lock, std::chrono::milliseconds(1000), [&] {
        return !g_running || state->frame_id != last_frame;
      });
      if (!g_running) break;
      if (state->jpeg.empty() || state->frame_id == last_frame) continue;
      jpeg = state->jpeg;
      frame_id = state->frame_id;
    }

    std::ostringstream part;
    part << "--frame\r\n"
         << "Content-Type: image/jpeg\r\n"
         << "Content-Length: " << jpeg.size() << "\r\n"
         << "X-Frame-Id: " << frame_id << "\r\n\r\n";
    if (!send_all(client_fd, part.str())) break;
    if (!send_all(client_fd, jpeg.data(), jpeg.size())) break;
    if (!send_all(client_fd, "\r\n")) break;
    last_frame = frame_id;
  }
  close_fd(client_fd);
}

void run_mjpeg_server(StreamState* state, int port) {
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    std::cerr << "MJPEG socket failed errno=" << errno << "\n";
    return;
  }
  int yes = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "MJPEG bind failed port=" << port << " errno=" << errno << "\n";
    close_fd(server_fd);
    return;
  }
  if (listen(server_fd, 4) < 0) {
    std::cerr << "MJPEG listen failed errno=" << errno << "\n";
    close_fd(server_fd);
    return;
  }
  std::cout << "mjpeg_stream=http://0.0.0.0:" << port << "/stream\n";

  while (g_running) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(server_fd, &rfds);
    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    int ready = select(server_fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ready <= 0) continue;
    sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &len);
    if (client_fd < 0) continue;
    std::thread(handle_stream_client, client_fd, state).detach();
  }
  close_fd(server_fd);
}

void run_visual_worker(VisualState* visual_state, StreamState* stream_state,
                       const std::string& out_image, bool write_snapshot) {
  int last_frame = -1;
  while (g_running) {
    VisualPacket packet;
    {
      std::unique_lock<std::mutex> lock(visual_state->mutex);
      visual_state->cv.wait_for(lock, std::chrono::milliseconds(1000), [&] {
        return !g_running || visual_state->frame_id != last_frame;
      });
      if (!g_running) break;
      if (visual_state->frame_id == last_frame || visual_state->packet.frame.empty()) {
        continue;
      }
      packet = visual_state->packet;
      last_frame = visual_state->frame_id;
    }

    cv::Mat vis = packet.frame.clone();
    const auto t0 = std::chrono::steady_clock::now();
    draw_detections(&vis, packet.detections, packet.fps, packet.infer_times,
                    packet.frame_times, packet.frame_id);
    auto jpeg = encode_display_jpeg(vis);
    const auto t1 = std::chrono::steady_clock::now();
    visual_state->encode_ms.store(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
    {
      std::lock_guard<std::mutex> lock(stream_state->mutex);
      stream_state->jpeg = std::move(jpeg);
      stream_state->frame_id = packet.frame_id;
    }
    stream_state->cv.notify_all();
    if (write_snapshot) {
      atomic_write_image(out_image, vis);
    }
  }
  stream_state->cv.notify_all();
}

std::string stem_from_path(const std::string& path) {
  return fs::path(path).stem().string();
}

int run_benchmark(const std::string& model_path, const std::string& image_dir,
                  const std::string& output_dir, int repeats) {
  fs::create_directories(output_dir);
  const auto images = collect_images(image_dir);
  if (images.empty()) {
    throw std::runtime_error("no test images found in: " + image_dir);
  }

  RknnModel model(model_path);

  double sum_rknn = 0.0;
  double sum_pre = 0.0;
  double sum_post = 0.0;
  double sum_total = 0.0;
  double sum_best_conf = 0.0;
  int timed_runs = 0;
  int detected_images = 0;
  int total_targets = 0;

  std::cout << "BENCH_MODEL path=" << model_path << " images=" << images.size()
            << " repeats=" << repeats << "\n";

  for (const auto& image_path : images) {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      std::cerr << "BENCH_WARN failed_to_read image=" << image_path << "\n";
      continue;
    }

    std::vector<Detection> first_detections;
    InferTimes first_times;
    double image_rknn = 0.0;
    double image_pre = 0.0;
    double image_post = 0.0;
    double image_total = 0.0;

    for (int r = 0; r < repeats; ++r) {
      InferTimes times;
      const auto t0 = std::chrono::steady_clock::now();
      auto detections = model.infer(image, &times);
      const auto t1 = std::chrono::steady_clock::now();
      const double total_ms =
          std::chrono::duration<double, std::milli>(t1 - t0).count();

      image_rknn += times.rknn_ms;
      image_pre += times.preprocess_ms;
      image_post += times.postprocess_ms;
      image_total += total_ms;
      sum_rknn += times.rknn_ms;
      sum_pre += times.preprocess_ms;
      sum_post += times.postprocess_ms;
      sum_total += total_ms;
      ++timed_runs;

      if (r == 0) {
        first_detections = std::move(detections);
        first_times = times;
      }
    }

    float best_conf = 0.0f;
    for (const auto& d : first_detections) {
      best_conf = std::max(best_conf, d.score);
    }
    if (!first_detections.empty()) {
      ++detected_images;
      sum_best_conf += best_conf;
    }
    total_targets += static_cast<int>(first_detections.size());

    FrameTimes draw_times;
    cv::Mat vis = image.clone();
    draw_detections(&vis, first_detections, 0.0, first_times, draw_times, 0);
    const std::string out_path = output_dir + "/" + stem_from_path(image_path) + ".jpg";
    atomic_write_image(out_path, vis);

    const double div = std::max(1, repeats);
    std::cout << "BENCH_IMAGE image=" << fs::path(image_path).filename().string()
              << " targets=" << first_detections.size()
              << " best_conf=" << std::fixed << std::setprecision(3)
              << best_conf
              << " avg_pre_ms=" << std::setprecision(2) << image_pre / div
              << " avg_rknn_ms=" << image_rknn / div
              << " avg_post_ms=" << image_post / div
              << " avg_total_ms=" << image_total / div
              << " out=" << out_path << "\n";
  }

  const double div = std::max(1, timed_runs);
  const double avg_total = sum_total / div;
  const double avg_conf = detected_images > 0 ? sum_best_conf / detected_images : 0.0;
  std::cout << "BENCH_SUMMARY"
            << " model=" << fs::path(model_path).filename().string()
            << " images=" << images.size()
            << " detected_images=" << detected_images
            << " total_targets=" << total_targets
            << " avg_best_conf=" << std::fixed << std::setprecision(3)
            << avg_conf
            << " avg_pre_ms=" << std::setprecision(2) << sum_pre / div
            << " avg_rknn_ms=" << sum_rknn / div
            << " avg_post_ms=" << sum_post / div
            << " avg_total_ms=" << avg_total
            << " infer_fps=" << (avg_total > 0.0 ? 1000.0 / avg_total : 0.0)
            << "\n";

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string project = "/home/orangepi/moonxkj/mine_pedestrian";

  if (argc >= 2 && std::string(argv[1]) == "--bench") {
    if (argc < 5) {
      std::cerr << "Usage: " << argv[0]
                << " --bench <model.rknn> <image_dir> <output_dir> [repeats]\n";
      return 1;
    }
    const std::string model_path = argv[2];
    const std::string image_dir = argv[3];
    const std::string output_dir = argv[4];
    const int repeats = argc >= 6 ? std::max(1, std::atoi(argv[5])) : 3;
    try {
      return run_benchmark(model_path, image_dir, output_dir, repeats);
    } catch (const std::exception& e) {
      std::cerr << "ERROR: " << e.what() << "\n";
      return 2;
    }
  }

  std::string model_path = project + "/models/current_seg.rknn";
  std::string out_image = project + "/web/live_result.jpg";
  std::string out_json = project + "/web/live_targets.json";
  unsigned int camera_index = 0;
  int timeout_ms = 3000;
  int mjpeg_port = 8090;
  bool write_snapshot = false;

  if (argc >= 2) camera_index = static_cast<unsigned int>(std::strtoul(argv[1], nullptr, 10));
  if (argc >= 3) out_image = argv[2];
  if (argc >= 4) out_json = argv[3];
  if (argc >= 5) model_path = argv[4];
  if (argc >= 6) mjpeg_port = std::atoi(argv[5]);
  if (argc >= 7) write_snapshot = std::atoi(argv[6]) != 0;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    RknnModel model(model_path);
    MvsCamera camera(camera_index);
    StreamState stream_state;
    VisualState visual_state;
    std::thread stream_thread(run_mjpeg_server, &stream_state, mjpeg_port);
    std::thread visual_thread(run_visual_worker, &visual_state, &stream_state,
                              out_image, write_snapshot);

    cv::Mat frame;
    for (int i = 0; i < 8; ++i) camera.grab(&frame, timeout_ms);

    int frame_id = 0;
    double fps = 0.0;
    auto last = std::chrono::steady_clock::now();
    FrameTimes last_frame_times;
    while (g_running) {
      const auto t_frame0 = std::chrono::steady_clock::now();
      const auto t_grab0 = std::chrono::steady_clock::now();
      if (!camera.grab(&frame, timeout_ms) || frame.empty()) {
        std::cerr << "grab timeout\n";
        continue;
      }
      const auto t_grab1 = std::chrono::steady_clock::now();

      InferTimes infer_times;
      auto detections = model.infer(frame, &infer_times);
      FrameTimes frame_times;
      frame_times.grab_ms =
          std::chrono::duration<double, std::milli>(t_grab1 - t_grab0).count();
      frame_times.draw_write_ms = visual_state.encode_ms.load();
      frame_times.frame_ms = last_frame_times.frame_ms;

      const auto now = std::chrono::steady_clock::now();
      frame_times.frame_ms =
          std::chrono::duration<double, std::milli>(now - t_frame0).count();

      const double dt = std::chrono::duration<double>(now - last).count();
      last = now;
      const double inst_fps = dt > 0 ? 1.0 / dt : 0.0;
      fps = fps == 0.0 ? inst_fps : fps * 0.85 + inst_fps * 0.15;

      atomic_write_text(out_json,
                        detections_json(detections, fps, infer_times, frame_times, frame_id));
      last_frame_times = frame_times;

      VisualPacket packet;
      packet.frame = frame.clone();
      packet.detections = detections;
      packet.infer_times = infer_times;
      packet.frame_times = frame_times;
      packet.fps = fps;
      packet.frame_id = frame_id;
      {
        std::lock_guard<std::mutex> lock(visual_state.mutex);
        visual_state.packet = std::move(packet);
        visual_state.frame_id = frame_id;
      }
      visual_state.cv.notify_one();

      if (frame_id % 10 == 0) {
        std::cout << "frame=" << frame_id << " fps=" << std::fixed
                  << std::setprecision(2) << fps
                  << " grab_ms=" << frame_times.grab_ms
                  << " pre_ms=" << infer_times.preprocess_ms
                  << " rknn_ms=" << infer_times.rknn_ms
                  << " post_ms=" << infer_times.postprocess_ms
                  << " draw_ms=" << frame_times.draw_write_ms
                  << " targets=" << detections.size() << "\n";
      }
      ++frame_id;
    }
    visual_state.cv.notify_all();
    stream_state.cv.notify_all();
    if (visual_thread.joinable()) visual_thread.join();
    if (stream_thread.joinable()) stream_thread.join();
    std::cout << "stopped\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 2;
  }
}
