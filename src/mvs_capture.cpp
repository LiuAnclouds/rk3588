#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "MvCameraControl.h"

namespace {

void print_error(const char* what, int ret) {
  std::cerr << what << " failed, ret=0x" << std::hex << ret << std::dec
            << "\n";
}

void warn_if_failed(const char* what, int ret) {
  if (ret != MV_OK) print_error(what, ret);
}

std::string ip_to_string(unsigned int ip) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", (ip >> 24) & 0xff,
                (ip >> 16) & 0xff, (ip >> 8) & 0xff, ip & 0xff);
  return buf;
}

void print_device(int index, MV_CC_DEVICE_INFO* info) {
  std::cout << "[device " << index << "]\n";
  if (!info) {
    std::cout << "  null device info\n";
    return;
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    const auto& gige = info->SpecialInfo.stGigEInfo;
    std::cout << "  type: GigE\n";
    std::cout << "  model: " << gige.chModelName << "\n";
    std::cout << "  user: " << gige.chUserDefinedName << "\n";
    std::cout << "  serial: " << gige.chSerialNumber << "\n";
    std::cout << "  ip: " << ip_to_string(gige.nCurrentIp) << "\n";
  } else if (info->nTLayerType == MV_USB_DEVICE) {
    const auto& usb = info->SpecialInfo.stUsb3VInfo;
    std::cout << "  type: USB3\n";
    std::cout << "  model: " << usb.chModelName << "\n";
    std::cout << "  user: " << usb.chUserDefinedName << "\n";
    std::cout << "  serial: " << usb.chSerialNumber << "\n";
  } else {
    std::cout << "  type: unsupported " << info->nTLayerType << "\n";
  }
}

bool is_bgr_or_rgb(MvGvspPixelType type) {
  return type == PixelType_Gvsp_BGR8_Packed ||
         type == PixelType_Gvsp_RGB8_Packed;
}

bool frame_to_bgr(void* handle, const MV_FRAME_OUT& frame, cv::Mat* bgr) {
  const auto& info = frame.stFrameInfo;
  const unsigned int width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
  const unsigned int height =
      info.nExtendHeight ? info.nExtendHeight : info.nHeight;
  const unsigned int src_len =
      info.nFrameLenEx ? static_cast<unsigned int>(info.nFrameLenEx)
                       : info.nFrameLen;

  if (width == 0 || height == 0 || frame.pBufAddr == nullptr) {
    std::cerr << "invalid frame\n";
    return false;
  }

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
    std::cerr << "source pixel type=0x" << std::hex << info.enPixelType
              << std::dec << " width=" << width << " height=" << height
              << "\n";
    return false;
  }

  cv::Mat tmp(height, width, CV_8UC3, converted.data());
  tmp.copyTo(*bgr);
  return true;
}

void configure_camera(void* handle) {
  warn_if_failed("Set TriggerMode Off",
                 MV_CC_SetEnumValue(handle, "TriggerMode", 0));
  warn_if_failed("Set AcquisitionFrameRateEnable false",
                 MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false));
  warn_if_failed("Set ExposureAuto Continuous",
                 MV_CC_SetEnumValueByString(handle, "ExposureAuto", "Continuous"));
  warn_if_failed("Set GainAuto Continuous",
                 MV_CC_SetEnumValueByString(handle, "GainAuto", "Continuous"));
  warn_if_failed("Set BalanceWhiteAuto Continuous",
                 MV_CC_SetEnumValueByString(handle, "BalanceWhiteAuto", "Continuous"));
  // Many color GigE cameras stream Bayer by default. We convert to BGR below.
  MV_CC_SetEnumValueByString(handle, "PixelFormat", "BGR8");
}

int capture_one(const std::string& output_path, unsigned int index,
                int timeout_ms) {
  int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    print_error("MV_CC_Initialize", ret);
    return 2;
  }

  void* handle = nullptr;
  MV_CC_DEVICE_INFO_LIST devices;
  std::memset(&devices, 0, sizeof(devices));

  ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
  if (ret != MV_OK) {
    print_error("MV_CC_EnumDevices", ret);
    MV_CC_Finalize();
    return 3;
  }

  std::cout << "device_count=" << devices.nDeviceNum << "\n";
  for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
    print_device(i, devices.pDeviceInfo[i]);
  }

  if (devices.nDeviceNum == 0) {
    std::cerr << "Find No Devices\n";
    MV_CC_Finalize();
    return 4;
  }
  if (index >= devices.nDeviceNum) {
    std::cerr << "camera index out of range: " << index << "\n";
    MV_CC_Finalize();
    return 5;
  }

  MV_CC_DEVICE_INFO* selected = devices.pDeviceInfo[index];
  if (!MV_CC_IsDeviceAccessible(selected, MV_ACCESS_Exclusive)) {
    std::cerr << "device is not accessible in exclusive mode\n";
  }

  ret = MV_CC_CreateHandle(&handle, selected);
  if (ret != MV_OK) {
    print_error("MV_CC_CreateHandle", ret);
    MV_CC_Finalize();
    return 6;
  }

  ret = MV_CC_OpenDevice(handle);
  if (ret != MV_OK) {
    print_error("MV_CC_OpenDevice", ret);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 7;
  }

  if (selected->nTLayerType == MV_GIGE_DEVICE) {
    int packet_size = MV_CC_GetOptimalPacketSize(handle);
    if (packet_size > 0) {
      ret = MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", packet_size);
      if (ret != MV_OK) print_error("Set GevSCPSPacketSize", ret);
      std::cout << "packet_size=" << packet_size << "\n";
    } else {
      std::cerr << "GetOptimalPacketSize failed: " << packet_size << "\n";
    }
  }

  configure_camera(handle);

  ret = MV_CC_StartGrabbing(handle);
  if (ret != MV_OK) {
    print_error("MV_CC_StartGrabbing", ret);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 8;
  }

  for (int i = 0; i < 8; ++i) {
    MV_FRAME_OUT warmup;
    std::memset(&warmup, 0, sizeof(warmup));
    int warm_ret = MV_CC_GetImageBuffer(handle, &warmup, timeout_ms);
    if (warm_ret == MV_OK) {
      MV_CC_FreeImageBuffer(handle, &warmup);
    }
  }

  MV_FRAME_OUT frame;
  std::memset(&frame, 0, sizeof(frame));
  ret = MV_CC_GetImageBuffer(handle, &frame, timeout_ms);
  if (ret != MV_OK) {
    print_error("MV_CC_GetImageBuffer", ret);
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 9;
  }

  const auto& info = frame.stFrameInfo;
  const unsigned int width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
  const unsigned int height =
      info.nExtendHeight ? info.nExtendHeight : info.nHeight;
  std::cout << "frame width=" << width << " height=" << height
            << " frame_num=" << info.nFrameNum << " pixel=0x" << std::hex
            << info.enPixelType << std::dec << "\n";

  cv::Mat bgr;
  bool ok = frame_to_bgr(handle, frame, &bgr);
  MV_CC_FreeImageBuffer(handle, &frame);

  MV_CC_StopGrabbing(handle);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);
  MV_CC_Finalize();

  if (!ok) return 10;
  cv::Scalar mean_bgr = cv::mean(bgr);
  std::cout << "mean_bgr=[" << mean_bgr[0] << "," << mean_bgr[1] << ","
            << mean_bgr[2] << "]\n";
  if (!cv::imwrite(output_path, bgr)) {
    std::cerr << "failed to write " << output_path << "\n";
    return 11;
  }
  std::cout << "saved=" << output_path << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string output = "outputs/camera_frame.jpg";
  unsigned int index = 0;
  int timeout_ms = 3000;

  if (argc >= 2) output = argv[1];
  if (argc >= 3) index = static_cast<unsigned int>(std::strtoul(argv[2], nullptr, 10));
  if (argc >= 4) timeout_ms = std::atoi(argv[3]);

  std::cout << "Usage: " << argv[0]
            << " [output.jpg] [camera_index=0] [timeout_ms=3000]\n";
  return capture_one(output, index, timeout_ms);
}
