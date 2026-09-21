#include "meta_service.hpp"

#include <algorithm>
#include <cstring>

namespace {
// mower_comms_v2's OnFirmwareInfoChanged() only checks major == 1; the version string is
// informational only, shown in logs/UI.
constexpr char kSimulatedFirmwareVersion[] = "sim-1.0.0";
constexpr uint16_t kSimulatedMajorVersion = 1;
}  // namespace

void MetaService::RPCGetFirmwareVersion(uint16_t call_id, char* data, uint16_t* response_length) {
  const size_t len = std::min(sizeof(kSimulatedFirmwareVersion) - 1, static_cast<size_t>(*response_length));
  memcpy(data, kSimulatedFirmwareVersion, len);
  *response_length = static_cast<uint16_t>(len);
  SendRpcResponse(call_id, xbot::datatypes::RpcStatus::SUCCESS, data, len * sizeof(char));
}

void MetaService::RPCGetMajorVersion(uint16_t call_id) {
  const uint16_t major = kSimulatedMajorVersion;
  SendRpcResponse(call_id, xbot::datatypes::RpcStatus::SUCCESS, &major, sizeof(major));
}
