#include <arpa/inet.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "MvCameraControl.h"

namespace {

void print_error(const char* what, int ret) {
  std::cerr << what << " failed, ret=0x" << std::hex << ret << std::dec
            << "\n";
}

bool parse_ip(const std::string& text, unsigned int* out) {
  unsigned int a, b, c, d;
  char tail = 0;
  std::istringstream iss(text);
  if (!(iss >> a)) return false;
  if (iss.get() != '.') return false;
  if (!(iss >> b)) return false;
  if (iss.get() != '.') return false;
  if (!(iss >> c)) return false;
  if (iss.get() != '.') return false;
  if (!(iss >> d)) return false;
  if (iss >> tail) return false;
  if (a > 255 || b > 255 || c > 255 || d > 255) return false;
  *out = (a << 24) | (b << 16) | (c << 8) | d;
  return true;
}

std::string ip_to_string(unsigned int ip) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", (ip >> 24) & 0xff,
                (ip >> 16) & 0xff, (ip >> 8) & 0xff, ip & 0xff);
  return buf;
}

void print_device(unsigned int index, MV_CC_DEVICE_INFO* info) {
  if (!info || info->nTLayerType != MV_GIGE_DEVICE) return;
  const auto& gige = info->SpecialInfo.stGigEInfo;
  std::cout << "[" << index << "] model=" << gige.chModelName
            << " serial=" << gige.chSerialNumber
            << " ip=" << ip_to_string(gige.nCurrentIp) << "\n";
}

int set_ip(unsigned int index, unsigned int ip, unsigned int mask,
           unsigned int gateway) {
  int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    print_error("MV_CC_Initialize", ret);
    return 2;
  }

  MV_CC_DEVICE_INFO_LIST devices;
  std::memset(&devices, 0, sizeof(devices));
  ret = MV_CC_EnumDevices(MV_GIGE_DEVICE, &devices);
  if (ret != MV_OK) {
    print_error("MV_CC_EnumDevices", ret);
    MV_CC_Finalize();
    return 3;
  }

  std::cout << "device_count=" << devices.nDeviceNum << "\n";
  for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
    print_device(i, devices.pDeviceInfo[i]);
  }
  if (devices.nDeviceNum == 0 || index >= devices.nDeviceNum) {
    std::cerr << "invalid camera index " << index << "\n";
    MV_CC_Finalize();
    return 4;
  }

  void* handle = nullptr;
  MV_CC_DEVICE_INFO* dev = devices.pDeviceInfo[index];
  ret = MV_CC_CreateHandle(&handle, dev);
  if (ret != MV_OK) {
    print_error("MV_CC_CreateHandle", ret);
    MV_CC_Finalize();
    return 5;
  }

  std::cout << "setting ip=" << ip_to_string(ip)
            << " mask=" << ip_to_string(mask)
            << " gateway=" << ip_to_string(gateway) << "\n";

  bool accessible = MV_CC_IsDeviceAccessible(dev, MV_ACCESS_Exclusive);
  if (accessible) {
    ret = MV_GIGE_SetIpConfig(handle, MV_IP_CFG_STATIC);
    if (ret != MV_OK) print_error("MV_GIGE_SetIpConfig before force", ret);
  }

  ret = MV_GIGE_ForceIpEx(handle, ip, mask, gateway);
  if (ret != MV_OK) {
    print_error("MV_GIGE_ForceIpEx", ret);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 6;
  }
  std::cout << "force ip ok\n";

  MV_CC_DestroyHandle(handle);
  handle = nullptr;

  dev->SpecialInfo.stGigEInfo.nCurrentIp = ip;
  dev->SpecialInfo.stGigEInfo.nCurrentSubNetMask = mask;
  dev->SpecialInfo.stGigEInfo.nDefultGateWay = gateway;
  ret = MV_CC_CreateHandle(&handle, dev);
  if (ret != MV_OK) {
    print_error("MV_CC_CreateHandle after force", ret);
    MV_CC_Finalize();
    return 7;
  }

  ret = MV_GIGE_SetIpConfig(handle, MV_IP_CFG_STATIC);
  if (ret != MV_OK) {
    print_error("MV_GIGE_SetIpConfig static", ret);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 8;
  }
  std::cout << "static ip config ok\n";

  MV_CC_DestroyHandle(handle);
  MV_CC_Finalize();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  unsigned int index = 0;
  std::string ip_text = "192.168.10.2";
  std::string mask_text = "255.255.255.0";
  std::string gateway_text = "192.168.10.1";

  if (argc >= 2) ip_text = argv[1];
  if (argc >= 3) mask_text = argv[2];
  if (argc >= 4) gateway_text = argv[3];
  if (argc >= 5) index = static_cast<unsigned int>(std::strtoul(argv[4], nullptr, 10));

  unsigned int ip = 0, mask = 0, gateway = 0;
  if (!parse_ip(ip_text, &ip) || !parse_ip(mask_text, &mask) ||
      !parse_ip(gateway_text, &gateway)) {
    std::cerr << "Usage: " << argv[0]
              << " [ip=192.168.10.2] [mask=255.255.255.0]"
                 " [gateway=192.168.10.1] [camera_index=0]\n";
    return 1;
  }

  return set_ip(index, ip, mask, gateway);
}
