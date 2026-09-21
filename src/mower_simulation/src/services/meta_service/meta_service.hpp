//
// MetaService simulation: reports a firmware version to mower_comms_v2 so its
// firmware-compatibility gate (which requires major version 1) opens and motion/mowing
// get enabled. Real firmware answers this over the same xbot-service protocol; the
// simulated board needs to answer it too or mower_comms_v2 waits forever.
//

#ifndef META_SERVICE_HPP
#define META_SERVICE_HPP

#include <MetaServiceBase.hpp>

using namespace xbot::service;

class MetaService : public MetaServiceBase {
 public:
  explicit MetaService(uint16_t service_id) : MetaServiceBase(service_id) {
  }

 protected:
  void RPCGetFirmwareVersion(uint16_t call_id, char* data, uint16_t* response_length) override;
  void RPCGetMajorVersion(uint16_t call_id) override;
};

#endif  // META_SERVICE_HPP
