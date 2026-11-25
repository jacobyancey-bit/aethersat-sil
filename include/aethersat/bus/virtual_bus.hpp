#pragma once

#include <cstdint>
#include <string>

namespace aethersat::bus {

struct BusStats {
    uint64_t tx_frames{0};
    uint64_t rx_frames{0};
    uint64_t error_count{0};
};

class VirtualBus {
public:
    virtual ~VirtualBus() = default;

    virtual std::string name() const = 0;
    virtual void reset() = 0;
    virtual BusStats stats() const noexcept = 0;
};

}  // namespace aethersat::bus
