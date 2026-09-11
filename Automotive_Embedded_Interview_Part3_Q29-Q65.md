# C/C++ Automotive Embedded Systems Interview Guide - Part 3
## Questions 29-65 with Code Examples

---

## Advanced Automotive Protocols

### Q29: Explain LIN (Local Interconnect Network) protocol and differences from CAN.

**Answer:**

```cpp
// LIN: Single-master, multiple-slave serial protocol
// Used for: Body control, lighting, motors (simpler than CAN)

// CAN vs LIN comparison:
/*
Feature          | CAN         | LIN
Speed            | 500K-1Mbps  | 9.6K-20Kbps
Cost             | Higher      | Lower
Complexity       | Complex     | Simple
Use Case         | Critical    | Non-critical
Wiring           | 2 wires     | 1 wire + GND
Nodes            | 32-64       | 2-16
Latency          | Lower       | Higher
*/

class LIN_Frame {
public:
    uint8_t pid;              // Protected ID (6-bit)
    uint8_t dlc;              // Data length (0-8)
    uint8_t data[8];
    uint16_t checksum;
};

class LINController {
private:
    volatile uint32_t* lin_ctrl_reg = (uint32_t*)0x40100000;
    volatile uint32_t* lin_status_reg = (uint32_t*)(0x40100000 + 0x04);
    volatile uint32_t* lin_data_reg = (uint32_t*)(0x40100000 + 0x08);
    
    // LIN timing parameters
    static constexpr uint16_t LIN_BAUDRATE = 9600;
    static constexpr uint16_t BREAK_LENGTH = 13;  // bit times
    static constexpr uint16_t DELIMITER_LENGTH = 1;
    
    enum LIN_Mode {
        LIN_SLEEP = 0,
        LIN_IDLE = 1,
        LIN_MASTER = 2,
        LIN_SLAVE = 3
    };
    
public:
    void initialize_master() {
        // Configure as LIN master
        *lin_ctrl_reg = (BREAK_LENGTH << 8) | LIN_MASTER;
        set_baudrate(LIN_BAUDRATE);
    }
    
    void initialize_slave() {
        // Configure as LIN slave
        *lin_ctrl_reg = LIN_SLAVE;
        set_baudrate(LIN_BAUDRATE);
    }
    
    // LIN frame transmission (master only)
    void send_header(uint8_t pid) {
        // Send break field (13 bit times at low)
        *lin_ctrl_reg |= 0x00000001;  // Send break
        
        // Send sync byte (0x55)
        uint32_t sync = 0x55;
        *lin_data_reg = sync;
        
        // Send PID (Protected ID)
        // PID includes parity bits (P0 and P1)
        uint8_t parity_pid = calculate_parity(pid);
        *lin_data_reg = parity_pid;
    }
    
    void send_data(uint8_t pid, const uint8_t* data, uint8_t dlc) {
        send_header(pid);
        
        // Send data bytes
        uint16_t checksum = calculate_checksum(pid, data, dlc);
        
        for (uint8_t i = 0; i < dlc; i++) {
            *lin_data_reg = data[i];
            checksum += data[i];
        }
        
        // Send checksum
        *lin_data_reg = checksum & 0xFF;
    }
    
    bool receive_frame(LIN_Frame& frame) {
        if (!(*lin_status_reg & 0x00000001)) {
            return false;  // No complete frame
        }
        
        // Read received frame data
        // Implementation similar to transmission but reverse
        return true;
    }
    
    void enter_sleep_mode() {
        *lin_ctrl_reg = (*lin_ctrl_reg & ~0x03) | LIN_SLEEP;
    }
    
private:
    void set_baudrate(uint16_t baudrate) {
        uint32_t prescaler = (CLOCK_SPEED / baudrate) - 1;
        *lin_ctrl_reg &= ~0xFF00;
        *lin_ctrl_reg |= (prescaler << 8);
    }
    
    uint8_t calculate_parity(uint8_t pid) {
        // PID = 6-bit ID + 2 parity bits
        uint8_t p0 = (((pid >> 0) & 1) + ((pid >> 1) & 1) + 
                     ((pid >> 2) & 1) + ((pid >> 4) & 1)) & 1;
        uint8_t p1 = (((pid >> 1) & 1) + ((pid >> 3) & 1) + 
                     ((pid >> 4) & 1) + ((pid >> 5) & 1)) & 1;
        
        return (p1 << 7) | (p0 << 6) | (pid & 0x3F);
    }
    
    uint16_t calculate_checksum(uint8_t pid, const uint8_t* data, 
                               uint8_t dlc) {
        uint16_t checksum = 0;
        checksum += pid;  // Include PID in checksum for LIN 2.x
        
        for (uint8_t i = 0; i < dlc; i++) {
            checksum += data[i];
        }
        
        // Handle overflow
        while (checksum > 255) {
            checksum = (checksum & 0xFF) + (checksum >> 8);
        }
        
        return (~checksum) & 0xFF;  // One's complement
    }
};

// LIN cluster configuration example
class LINCluster {
private:
    struct LINNode {
        uint8_t node_id;
        LINController* controller;
        uint8_t pids[16];  // Handled PIDs
    };
    
    LINNode master;
    LINNode slaves[8];
    uint8_t slave_count = 0;
    
public:
    void initialize_as_master() {
        master.controller->initialize_master();
    }
    
    void add_slave(LINController* controller, uint8_t node_id) {
        if (slave_count < 8) {
            slaves[slave_count].controller = controller;
            slaves[slave_count].node_id = node_id;
            slaves[slave_count].controller->initialize_slave();
            slave_count++;
        }
    }
    
    void configure_schedule() {
        // LIN uses time-triggered scheduling
        // Master sends headers at precise intervals
        
        // Example: 10ms cycle
        /*
        T0.0ms: Send header for PID 0x00 (Speed)
        T2.5ms: Send header for PID 0x01 (Temperature)
        T5.0ms: Send header for PID 0x02 (Status)
        T7.5ms: Send header for PID 0x03 (Commands)
        T10.0ms: Cycle repeats
        */
    }
};

// LIN message example: Motor control
class LINMotorControl {
private:
    LINController lin;
    
public:
    struct MotorCommand {
        uint8_t speed;        // 0-100%
        uint8_t direction;    // 0=off, 1=forward, 2=reverse
        uint8_t status;
    };
    
    void send_motor_command(const MotorCommand& cmd) {
        uint8_t data[3] = {cmd.speed, cmd.direction, cmd.status};
        lin.send_data(0x10, data, 3);  // PID 0x10 = motor control
    }
    
    void receive_motor_feedback(MotorCommand& feedback) {
        LIN_Frame frame;
        
        if (lin.receive_frame(frame)) {
            if (frame.pid == 0x10) {
                feedback.speed = frame.data[0];
                feedback.direction = frame.data[1];
                feedback.status = frame.data[2];
            }
        }
    }
};
```

---

### Q30: Explain Ethernet in automotive (Automotive Ethernet / 100BASE-T1).

**Answer:**

```cpp
// Automotive Ethernet: 100 Mbps, shielded single-twisted-pair
// Used in: ADAS, infotainment, diagnostics

// Why Automotive Ethernet?
// - High bandwidth (100Mbps vs 1Mbps CAN)
// - Lower latency
// - Single twisted pair (vs CAN 2-wire)
// - Can carry video/audio
// - Supports TCP/IP protocol stack

class Automotive_MAC_Address {
public:
    uint8_t octets[6];  // 48-bit MAC address
    
    void print() {
        printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
               octets[0], octets[1], octets[2],
               octets[3], octets[4], octets[5]);
    }
};

// Ethernet frame structure
class EthernetFrame {
public:
    Automotive_MAC_Address dest_mac;
    Automotive_MAC_Address src_mac;
    uint16_t ethertype;        // Protocol type (IPv4, IPv6, etc.)
    uint8_t payload[1500];     // MTU = 1500 bytes
    uint16_t payload_length;
    uint32_t fcs;              // Frame Check Sequence
};

// Physical layer driver (100BASE-T1)
class EthernetPHY {
private:
    volatile uint32_t* phy_control_reg = (uint32_t*)0x40200000;
    volatile uint32_t* phy_status_reg = (uint32_t*)(0x40200000 + 0x04);
    volatile uint32_t* phy_tx_reg = (uint32_t*)(0x40200000 + 0x08);
    volatile uint32_t* phy_rx_reg = (uint32_t*)(0x40200000 + 0x0C);
    
public:
    enum LinkSpeed {
        LINK_SPEED_NONE = 0,
        LINK_SPEED_100M = 1
    };
    
    void initialize() {
        // Reset PHY
        *phy_control_reg |= 0x00008000;
        
        // Wait for reset
        while (*phy_control_reg & 0x00008000) {}
        
        // Configure for 100BASE-T1
        uint16_t control = 0x0000;
        control |= 0x1000;  // Speed 100Mbps
        control |= 0x0100;  // Full duplex
        
        *phy_control_reg = control;
    }
    
    LinkSpeed get_link_speed() {
        uint32_t status = *phy_status_reg;
        
        if (status & 0x00000001) {  // Link up
            return LINK_SPEED_100M;
        }
        
        return LINK_SPEED_NONE;
    }
    
    bool is_link_up() {
        return (*phy_status_reg & 0x00000001) != 0;
    }
};

// MAC layer driver
class EthernetMAC {
private:
    EthernetPHY phy;
    
    volatile uint32_t* mac_ctrl_reg = (uint32_t*)0x40201000;
    volatile uint32_t* mac_addr_high = (uint32_t*)(0x40201000 + 0x04);
    volatile uint32_t* mac_addr_low = (uint32_t*)(0x40201000 + 0x08);
    volatile uint32_t* mac_tx_desc = (uint32_t*)(0x40201000 + 0x10);
    volatile uint32_t* mac_rx_desc = (uint32_t*)(0x40201000 + 0x14);
    
    // DMA descriptors
    static constexpr uint8_t NUM_TX_DESC = 2;
    static constexpr uint8_t NUM_RX_DESC = 4;
    
    struct DMA_Descriptor {
        uint32_t own : 1;           // Owner: 1=DMA, 0=CPU
        uint32_t ic : 1;            // Interrupt on completion
        uint32_t ls : 1;            // Last segment
        uint32_t fs : 1;            // First segment
        uint32_t reserved : 28;
        uint32_t buffer1_addr;
        uint32_t buffer1_size : 13;
        uint32_t buffer2_addr : 19;
    };
    
public:
    void initialize(const Automotive_MAC_Address& mac) {
        phy.initialize();
        
        // Set MAC address
        uint32_t addr_high = (mac.octets[5] << 8) | mac.octets[4];
        uint32_t addr_low = (mac.octets[3] << 24) | (mac.octets[2] << 16) |
                           (mac.octets[1] << 8) | mac.octets[0];
        
        *mac_addr_high = addr_high | 0x80000000;  // Valid bit
        *mac_addr_low = addr_low;
        
        // Enable MAC
        *mac_ctrl_reg = 0x0000000B;  // TX/RX enable
    }
    
    bool transmit(const EthernetFrame& frame) {
        // Check if TX descriptor available
        if (!is_tx_available()) {
            return false;
        }
        
        // Setup DMA descriptor
        // ...implementation...
        
        // Start transmission
        *mac_ctrl_reg |= 0x00000002;  // Start TX
        
        return true;
    }
    
    bool receive(EthernetFrame& frame) {
        // Check if RX descriptor has data
        if (!is_rx_available()) {
            return false;
        }
        
        // Read from DMA descriptor
        // ...implementation...
        
        return true;
    }
    
private:
    bool is_tx_available() {
        // Check TX descriptor
        return true;  // Simplified
    }
    
    bool is_rx_available() {
        // Check RX descriptor
        return false;  // Simplified
    }
};

// IPv4 stack (simplified)
class IPv4_Header {
public:
    uint8_t version : 4;
    uint8_t ihl : 4;
    uint8_t dscp : 6;
    uint8_t ecn : 2;
    uint16_t total_length;
    uint16_t identification;
    uint16_t flags_offset;
    uint8_t ttl;
    uint8_t protocol;
    uint16_t checksum;
    uint32_t src_ip;
    uint32_t dest_ip;
};

// UDP socket (for diagnostics, etc.)
class UDPSocket {
private:
    uint16_t local_port;
    uint16_t remote_port;
    uint32_t remote_ip;
    EthernetMAC& ethernet;
    
public:
    UDPSocket(EthernetMAC& eth, uint16_t port) 
        : local_port(port), ethernet(eth) {}
    
    bool send(const uint8_t* data, uint16_t length) {
        // Build UDP/IP/Ethernet frame
        // ...implementation...
        return true;
    }
    
    bool receive(uint8_t* data, uint16_t& length) {
        // Receive and parse UDP/IP/Ethernet frame
        // ...implementation...
        return false;
    }
};

// Automotive Ethernet use case: AUTOSAR Ethernet
class AUTOSAR_SocketExample {
private:
    // AUTOSAR PDU (Protocol Data Unit)
    struct AUTOSAR_PDU {
        uint16_t pdu_id;
        uint8_t data[256];
        uint16_t length;
    };
    
public:
    void send_pdu_over_ethernet(const AUTOSAR_PDU& pdu) {
        // AUTOSAR TP (Transport Protocol) layer
        // Maps PDU to UDP/IPv4/Ethernet
    }
};
```

---

## Safety and Functional Safety

### Q31: Explain ISO 26262 (Functional Safety) and ASIL levels.

**Answer:**

```cpp
// ISO 26262: Functional Safety of electrical/electronic systems in vehicles

/*
ASIL (Automotive Safety Integrity Level) Classification:

ASIL QM: No safety requirement (quality management only)
ASIL A:  Low safety requirement
ASIL B:  Medium safety requirement
ASIL C:  Medium-high safety requirement
ASIL D:  Highest safety requirement (most critical)

Determined by: Severity × Exposure × Controllability

Example: Loss of power steering
- Severity: High (serious injury/death possible)
- Exposure: Medium (not continuous)
- Controllability: Low (driver has limited control)
→ ASIL D (highest level)

Safety Mechanisms by ASIL:
ASIL D: Requires multiple redundant checks, cross-monitoring
ASIL C: Single-level safety mechanisms usually not sufficient
ASIL B: Single-level mechanisms may be adequate
ASIL A: Basic checks sufficient
*/

// Example: ASIL D - Fuel pump safety mechanism

class FuelPumpController {
private:
    // ASIL D requires dual-channel architecture
    enum FuelPumpState {
        PUMP_OFF = 0,
        PUMP_ON = 1,
        PUMP_FAULT = 2
    };
    
    // Primary control channel
    FuelPumpState primary_state = PUMP_OFF;
    
    // Secondary (watchdog) channel
    FuelPumpState secondary_state = PUMP_OFF;
    
    // Diagnostics state
    struct DiagnosticsInfo {
        uint8_t primary_failures;
        uint8_t secondary_failures;
        uint32_t fault_code;
    } diag_info;
    
    static constexpr uint16_t FUEL_PUMP_TIMEOUT = 100;  // ms
    uint32_t pump_control_timestamp = 0;
    
public:
    void enable_fuel_pump() {
        // Primary channel activation
        activate_pump_primary();
        primary_state = PUMP_ON;
        
        // Secondary channel monitor
        secondary_state = PUMP_ON;
        pump_control_timestamp = get_ticks();
        
        // Diagnostic: Both channels agree
        if (verify_dual_channel_agreement()) {
            // Safe to enable pump
        } else {
            // Mismatch detected - ASIL D violation
            disable_fuel_pump();
            record_fault(FUEL_PUMP_DUAL_CHANNEL_MISMATCH);
        }
    }
    
    void disable_fuel_pump() {
        // Dual-channel shutdown
        deactivate_pump_primary();
        primary_state = PUMP_OFF;
        
        deactivate_pump_secondary();
        secondary_state = PUMP_OFF;
    }
    
    void monitor_fuel_pump_health() {
        // ASIL D monitoring: Called every 10ms
        
        // Check timeout
        if ((get_ticks() - pump_control_timestamp) > FUEL_PUMP_TIMEOUT) {
            if (primary_state == PUMP_ON) {
                record_fault(FUEL_PUMP_COMMUNICATION_TIMEOUT);
                disable_fuel_pump();
            }
        }
        
        // Cross-channel monitoring
        if (primary_state != secondary_state) {
            record_fault(FUEL_PUMP_DUAL_CHANNEL_MISMATCH);
            disable_fuel_pump();
        }
        
        // Primary feedback verification
        if (!verify_pump_feedback()) {
            diag_info.primary_failures++;
            if (diag_info.primary_failures > 3) {
                record_fault(FUEL_PUMP_PRIMARY_FAILURE);
                disable_fuel_pump();
            }
        }
        
        // Secondary watchdog monitoring
        if (!verify_secondary_watchdog()) {
            diag_info.secondary_failures++;
            if (diag_info.secondary_failures > 1) {
                record_fault(FUEL_PUMP_SECONDARY_FAILURE);
                disable_fuel_pump();
            }
        }
    }
    
    uint32_t get_fault_code() {
        return diag_info.fault_code;
    }
    
private:
    void activate_pump_primary() {
        // Write to fuel pump relay control
        GPIO_SET_OUTPUT(FUEL_PUMP_PORT, FUEL_PUMP_PIN, 1);
    }
    
    void deactivate_pump_primary() {
        GPIO_SET_OUTPUT(FUEL_PUMP_PORT, FUEL_PUMP_PIN, 0);
    }
    
    void deactivate_pump_secondary() {
        // Secondary deactivation via different mechanism
        SET_FUEL_PUMP_FET_OFF();
    }
    
    bool verify_dual_channel_agreement() {
        return primary_state == secondary_state;
    }
    
    bool verify_pump_feedback() {
        // Read fuel pump current/pressure feedback
        uint16_t pump_current = read_fuel_pump_current();
        
        if (primary_state == PUMP_ON) {
            return pump_current > MIN_PUMP_CURRENT;
        } else {
            return pump_current < MAX_IDLE_CURRENT;
        }
    }
    
    bool verify_secondary_watchdog() {
        // Secondary watchdog uses different signal chain
        return read_secondary_pump_status();
    }
    
    void record_fault(uint32_t fault_code) {
        diag_info.fault_code = fault_code;
        
        // Log to safety memory (protected)
        log_safety_event(fault_code, get_ticks());
        
        // Notify other ECUs
        send_fault_message(fault_code);
    }
};

// ASIL B Example: Engine RPM Sensor Safety

class EngineRPMSensor_ASILB {
private:
    struct SensorReading {
        uint16_t rpm;
        bool is_valid;
        uint32_t timestamp;
    };
    
    SensorReading current_reading;
    SensorReading previous_reading;
    
    static constexpr uint16_t MAX_RPM_CHANGE_PER_10MS = 500;
    static constexpr uint16_t MAX_RPM = 7500;
    static constexpr uint16_t MIN_RPM = 0;
    
public:
    uint16_t read_engine_rpm() {
        // ASIL B: Range check + rate-of-change check
        
        uint16_t raw_rpm = read_rpm_sensor();
        
        // Range check
        if (raw_rpm > MAX_RPM) {
            current_reading.is_valid = false;
            return 0;  // Error
        }
        
        // Rate-of-change check (plausibility)
        uint16_t rpm_change = abs(raw_rpm - previous_reading.rpm);
        
        if (rpm_change > MAX_RPM_CHANGE_PER_10MS) {
            // Unrealistic change - sensor failure?
            record_plausibility_fault();
            current_reading.is_valid = false;
            return previous_reading.rpm;  // Use previous value
        }
        
        current_reading.rpm = raw_rpm;
        current_reading.is_valid = true;
        current_reading.timestamp = get_ticks();
        
        // Update history
        previous_reading = current_reading;
        
        return raw_rpm;
    }
    
private:
    void record_plausibility_fault() {
        // ASIL B: Record fault for diagnostics
    }
};

// Safety-critical function attributes
#define SAFETY_CRITICAL __attribute__((section(".safety_critical")))

class SafetyCriticalExample {
public:
    // Must be in protected memory region
    SAFETY_CRITICAL void emergency_brake_apply() {
        // Maximum 100ms response time
        apply_all_brake_actuators();
        
        // Verify activation
        if (!verify_brake_pressure()) {
            trigger_limp_home();
        }
    }
    
    // MISRA-C compliant (no dynamic memory, no recursion)
    SAFETY_CRITICAL uint32_t calculate_brake_force(uint16_t pedal_position) {
        if (pedal_position > MAX_PEDAL) {
            return 0;  // Fail-safe
        }
        
        uint32_t force = pedal_position * BRAKE_GAIN;
        
        if (force > MAX_BRAKE_FORCE) {
            return MAX_BRAKE_FORCE;  // Clamp
        }
        
        return force;
    }
};
```

---

### Q32: Explain fault tolerance and defensive programming in automotive.

**Answer:**

```cpp
// Defensive Programming Principles for Automotive:
// 1. Fail-safe defaults
// 2. Redundancy and cross-checking
// 3. Watchdogs and monitoring
// 4. Graceful degradation
// 5. Comprehensive error handling

// Example: Transmission shift solenoid control

class TransmissionShiftControl {
private:
    enum SolenoidState {
        SOLENOID_OFF = 0,
        SOLENOID_ON = 1,
        SOLENOID_FAULT = 2
    };
    
    struct SolenoidMonitor {
        SolenoidState desired_state;
        SolenoidState actual_state;
        uint16_t fault_count;
        uint32_t last_check_time;
    };
    
    SolenoidMonitor sol_a;  // Solenoid A (shift 1-2)
    SolenoidMonitor sol_b;  // Solenoid B (shift 2-3)
    
    static constexpr uint16_t SOLENOID_SETTLE_TIME = 20;  // ms
    static constexpr uint8_t FAULT_THRESHOLD = 3;
    
public:
    void shift_to_gear(uint8_t target_gear) {
        // Defensive check: Validate input
        if (target_gear > 6) {
            record_fault(INVALID_GEAR_REQUEST);
            return;  // Fail-safe
        }
        
        // Calculate solenoid pattern
        bool sol_a_desired = calculate_sol_a(target_gear);
        bool sol_b_desired = calculate_sol_b(target_gear);
        
        // Dual-channel command
        set_solenoid_a_primary(sol_a_desired);
        set_solenoid_a_secondary(sol_a_desired);
        
        set_solenoid_b_primary(sol_b_desired);
        set_solenoid_b_secondary(sol_b_desired);
        
        // Record desired state
        sol_a.desired_state = sol_a_desired ? SOLENOID_ON : SOLENOID_OFF;
        sol_b.desired_state = sol_b_desired ? SOLENOID_ON : SOLENOID_OFF;
        
        sol_a.last_check_time = get_ticks();
        sol_b.last_check_time = get_ticks();
    }
    
    void monitor_solenoids() {
        // Monitor EVERY 10ms
        
        // Check solenoid A
        if ((get_ticks() - sol_a.last_check_time) >= SOLENOID_SETTLE_TIME) {
            verify_solenoid_a();
        }
        
        // Check solenoid B
        if ((get_ticks() - sol_b.last_check_time) >= SOLENOID_SETTLE_TIME) {
            verify_solenoid_b();
        }
        
        // Check for latent faults
        if (sol_a.fault_count >= FAULT_THRESHOLD) {
            enter_limp_home(SOLENOID_A_FAULT);
        }
        
        if (sol_b.fault_count >= FAULT_THRESHOLD) {
            enter_limp_home(SOLENOID_B_FAULT);
        }
    }
    
private:
    void verify_solenoid_a() {
        // Read actual solenoid state via feedback
        bool is_energized = read_solenoid_a_feedback();
        
        sol_a.actual_state = is_energized ? SOLENOID_ON : SOLENOID_OFF;
        
        // Check agreement
        if (sol_a.actual_state != sol_a.desired_state) {
            sol_a.fault_count++;
            
            // Log for diagnostics
            log_solenoid_fault(SOLENOID_A_MISMATCH);
        } else {
            // Clear fault count on success
            if (sol_a.fault_count > 0) {
                sol_a.fault_count--;
            }
        }
    }
    
    void verify_solenoid_b() {
        bool is_energized = read_solenoid_b_feedback();
        
        sol_b.actual_state = is_energized ? SOLENOID_ON : SOLENOID_OFF;
        
        if (sol_b.actual_state != sol_b.desired_state) {
            sol_b.fault_count++;
            log_solenoid_fault(SOLENOID_B_MISMATCH);
        } else {
            if (sol_b.fault_count > 0) {
                sol_b.fault_count--;
            }
        }
    }
    
    void enter_limp_home(uint8_t fault_reason) {
        // Safe state: Lock in current gear
        disable_all_solenoids();
        
        // Notify driver and other systems
        set_check_engine_light();
        send_fault_to_can_bus(fault_reason);
        
        // Limit speed to allow safe parking
        set_max_speed_limit(50);  // 50 km/h
    }
    
    bool calculate_sol_a(uint8_t gear) {
        // Shift pattern lookup table (defensive)
        const bool shift_pattern[7][2] = {
            {0, 0},  // P/N
            {0, 0},  // R
            {0, 1},  // D
            {1, 0},  // D (1st)
            {1, 0},  // D (2nd)
            {1, 1},  // D (3rd)
            {0, 1}   // D (4th)
        };
        
        return shift_pattern[gear & 0x07][0];
    }
    
    bool calculate_sol_b(uint8_t gear) {
        const bool shift_pattern[7][2] = {
            {0, 0}, {0, 0}, {0, 1}, {1, 0}, {1, 0}, {1, 1}, {0, 1}
        };
        
        return shift_pattern[gear & 0x07][1];
    }
};

// Redundancy management
class DualChannelBraking {
private:
    enum BrakingMode {
        NORMAL = 0,
        PRIMARY_FAILED = 1,
        SECONDARY_FAILED = 2,
        BOTH_FAILED = 3
    };
    
    BrakingMode current_mode = NORMAL;
    
    struct BrakeChannel {
        uint8_t requested_force;
        uint8_t actual_force;
        bool is_healthy;
        uint32_t failure_time;
    };
    
    BrakeChannel primary;
    BrakeChannel secondary;
    
public:
    void apply_brakes(uint8_t force) {
        // Bound input
        force = MIN(force, 100);
        
        // Apply to both channels
        apply_primary_brake(force);
        apply_secondary_brake(force);
        
        primary.requested_force = force;
        secondary.requested_force = force;
    }
    
    void monitor_brake_health() {
        uint8_t primary_actual = read_primary_brake_pressure();
        uint8_t secondary_actual = read_secondary_brake_pressure();
        
        primary.actual_force = primary_actual;
        secondary.actual_force = secondary_actual;
        
        // Determine channel health
        primary.is_healthy = (primary_actual >= (primary.requested_force - 5));
        secondary.is_healthy = (secondary_actual >= (secondary.requested_force - 5));
        
        // Update mode
        if (primary.is_healthy && secondary.is_healthy) {
            current_mode = NORMAL;
        } else if (!primary.is_healthy && secondary.is_healthy) {
            current_mode = PRIMARY_FAILED;
        } else if (primary.is_healthy && !secondary.is_healthy) {
            current_mode = SECONDARY_FAILED;
        } else {
            current_mode = BOTH_FAILED;
            trigger_emergency_stop();
        }
    }
    
    BrakingMode get_current_mode() { return current_mode; }
    
private:
    void trigger_emergency_stop() {
        // Limp-home: Reduce speed gradually
        set_engine_idle_only();
    }
};
```

---

## Signal Processing and Filtering

### Q33: Explain sensor filtering and noise reduction in embedded systems.

**Answer:**

```cpp
// Sensor noise: High-frequency random variations
// Filtering: Smooth data without losing important information

// Example: Engine RPM sensor filtering

class RPMSensorFilter {
private:
    static constexpr uint8_t FILTER_ORDER = 4;
    uint16_t filter_buffer[FILTER_ORDER];
    uint8_t buffer_index = 0;
    
public:
    // Moving Average Filter (simplest)
    uint16_t moving_average_filter(uint16_t raw_rpm) {
        filter_buffer[buffer_index] = raw_rpm;
        buffer_index = (buffer_index + 1) % FILTER_ORDER;
        
        uint32_t sum = 0;
        for (uint8_t i = 0; i < FILTER_ORDER; i++) {
            sum += filter_buffer[i];
        }
        
        return sum / FILTER_ORDER;
    }
};

// More sophisticated: Kalman Filter
class KalmanFilter {
private:
    // State variables
    float state_estimate;        // Current estimate
    float state_covariance;      // Estimation uncertainty
    float measurement_noise;     // Sensor noise
    float process_noise;         // System noise
    
public:
    KalmanFilter(float initial_estimate, float meas_noise, float proc_noise)
        : state_estimate(initial_estimate),
          measurement_noise(meas_noise),
          process_noise(proc_noise),
          state_covariance(1.0f) {}
    
    float update(float measurement) {
        // Predict phase
        float prior_estimate = state_estimate;
        float prior_covariance = state_covariance + process_noise;
        
        // Update phase
        float kalman_gain = prior_covariance / 
                          (prior_covariance + measurement_noise);
        
        state_estimate = prior_estimate + 
                        kalman_gain * (measurement - prior_estimate);
        
        state_covariance = (1.0f - kalman_gain) * prior_covariance;
        
        return state_estimate;
    }
    
    float get_estimate() { return state_estimate; }
};

// Real example: Engine RPM with Kalman
class RealTimeRPMFilter {
private:
    KalmanFilter kf_rpm{800.0f, 100.0f, 10.0f};
    
public:
    uint16_t get_filtered_rpm(uint16_t raw_rpm) {
        float filtered = kf_rpm.update((float)raw_rpm);
        return (uint16_t)filtered;
    }
};

// Exponential Smoothing (good balance: simple + effective)
class ExponentialFilter {
private:
    float alpha;              // Smoothing factor (0.0-1.0)
    float filtered_value;
    
public:
    ExponentialFilter(float smoothing_factor, float initial_value)
        : alpha(smoothing_factor), filtered_value(initial_value) {}
    
    float update(float new_measurement) {
        filtered_value = (alpha * new_measurement) + 
                        ((1.0f - alpha) * filtered_value);
        return filtered_value;
    }
    
    /*
    alpha = 0.1:  Heavy smoothing (slow response)
    alpha = 0.5:  Moderate smoothing
    alpha = 0.9:  Light smoothing (fast response)
    
    Automotive choice: 0.2-0.5 (balance noise vs response time)
    */
};

// Median Filter (great for outlier rejection)
class MedianFilter {
private:
    static constexpr uint8_t WINDOW_SIZE = 5;
    uint16_t values[WINDOW_SIZE];
    uint8_t index = 0;
    
public:
    uint16_t update(uint16_t new_value) {
        values[index] = new_value;
        index = (index + 1) % WINDOW_SIZE;
        
        // Sort to find median
        uint16_t sorted[WINDOW_SIZE];
        memcpy(sorted, values, sizeof(values));
        
        // Bubble sort (for small array)
        for (int i = 0; i < WINDOW_SIZE - 1; i++) {
            for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {
                if (sorted[j] > sorted[j + 1]) {
                    uint16_t temp = sorted[j];
                    sorted[j] = sorted[j + 1];
                    sorted[j + 1] = temp;
                }
            }
        }
        
        // Return median (middle value)
        return sorted[WINDOW_SIZE / 2];
    }
};

// Combined filtering strategy
class AdaptiveFilter {
private:
    MedianFilter median;          // Outlier rejection
    ExponentialFilter exponential{0.3f, 0.0f};  // Smoothing
    
public:
    uint16_t process_sensor(uint16_t raw_value) {
        // Step 1: Remove outliers with median filter
        uint16_t outlier_removed = median.update(raw_value);
        
        // Step 2: Smooth with exponential filter
        float smoothed = exponential.update((float)outlier_removed);
        
        return (uint16_t)smoothed;
    }
};

// Low-pass filter implementation
class LowPassFilter {
private:
    float cutoff_frequency;  // Hz
    float dt;                // Time step (seconds)
    float filtered_value = 0;
    
    float calculate_rc_constant() {
        return 1.0f / (2.0f * 3.14159f * cutoff_frequency);
    }
    
public:
    LowPassFilter(float freq, float time_step)
        : cutoff_frequency(freq), dt(time_step) {}
    
    float update(float raw_value) {
        float rc = calculate_rc_constant();
        float alpha = dt / (rc + dt);
        
        filtered_value = (alpha * raw_value) + 
                        ((1.0f - alpha) * filtered_value);
        
        return filtered_value;
    }
};

// IIR Filter (Infinite Impulse Response)
class IIRFilter {
private:
    // Second-order IIR coefficients
    float b[3];  // Numerator
    float a[3];  // Denominator
    float x[2] = {0};  // Input history
    float y[2] = {0};  // Output history
    
public:
    IIRFilter(float* numerator, float* denominator) {
        memcpy(b, numerator, 3 * sizeof(float));
        memcpy(a, denominator, 3 * sizeof(float));
    }
    
    float update(float input) {
        // Shift history
        x[1] = x[0];
        x[0] = input;
        
        // Compute output
        float output = (b[0]*x[0] + b[1]*x[1] + b[2]*y[1]) / a[0];
        output -= (a[1]*y[0] + a[2]*y[1]) / a[0];
        
        // Update history
        y[1] = y[0];
        y[0] = output;
        
        return output;
    }
};

// Filter selection guide:
/*
Sensor Type            | Noise Level | Recommended Filter
Temperature            | Low         | Exponential (0.1-0.2)
Throttle Position      | Medium      | Median + Exponential
RPM                    | High        | Kalman or IIR
Pressure               | Low         | Moving Average
Acceleration (G-force) | High        | Kalman
CAN Bus Data          | Low         | Simple averaging
*/
```

---

### Q34: Explain ADC (Analog-to-Digital Converter) sampling and quantization.

**Answer:**

```cpp
// ADC converts analog voltage to digital value
// Resolution: typically 8-bit, 10-bit, 12-bit, or 16-bit

/*
12-bit ADC: 0-4095 values for 0-5V
Resolution: 5V / 4096 = 1.22 mV per LSB

Nyquist Theorem: Sample rate ≥ 2 × signal bandwidth
If signal has 1kHz component, need ≥ 2kHz sample rate
*/

class ADC_Config {
private:
    volatile uint32_t* adc_ctrl_reg = (uint32_t*)0x40100000;
    volatile uint32_t* adc_seq_reg = (uint32_t*)(0x40100000 + 0x04);
    volatile uint32_t* adc_result_reg = (uint32_t*)(0x40100000 + 0x08);
    
    enum ADC_Resolution {
        RES_8BIT = 0,      // 0-255
        RES_10BIT = 1,     // 0-1023
        RES_12BIT = 2,     // 0-4095
        RES_16BIT = 3      // 0-65535
    };
    
    enum ADC_SamplingRate {
        SAMPLE_1kHz = 0,
        SAMPLE_10kHz = 1,
        SAMPLE_100kHz = 2
    };
    
public:
    void initialize(ADC_Resolution res, ADC_SamplingRate rate) {
        // Set resolution
        *adc_ctrl_reg &= ~0x00000003;
        *adc_ctrl_reg |= res & 0x03;
        
        // Set sampling rate
        *adc_ctrl_reg &= ~0x0000000C;
        *adc_ctrl_reg |= (rate << 2) & 0x0C;
        
        // Enable ADC
        *adc_ctrl_reg |= 0x00000001;
    }
    
    uint16_t read_adc(uint8_t channel) {
        // Select channel
        *adc_seq_reg = channel & 0x0F;
        
        // Start conversion
        *adc_ctrl_reg |= 0x00000002;
        
        // Wait for conversion complete
        while (!(*adc_ctrl_reg & 0x00000004)) {}
        
        // Read result
        return *adc_result_reg & 0x0FFF;  // 12-bit
    }
};

// Quantization error
class QuantizationAnalysis {
    /*
    12-bit ADC: Resolution = Vref / 2^12 = 5V / 4096 ≈ 1.22 mV
    
    Raw reading: 2048
    Voltage: 2048 * 1.22 mV ≈ 2.5V
    
    Max quantization error: ±0.61 mV (half LSB)
    
    Effect in automotive:
    - Engine temp sensor: 1.22 mV acceptable (good)
    - High-precision pressure sensor: 1.22 mV might be too coarse
    → May need 16-bit ADC (0.076 mV)
    */
};

// Multi-channel sequential ADC
class MultiChannelADC {
private:
    static constexpr uint8_t NUM_CHANNELS = 8;
    uint16_t adc_values[NUM_CHANNELS];
    uint8_t current_channel = 0;
    
public:
    void scan_all_channels() {
        // DMA-driven scanning (most efficient)
        // Configure DMA to read ADC sequentially
        // Results stored in adc_values[] automatically
    }
    
    uint16_t get_channel(uint8_t channel) {
        if (channel < NUM_CHANNELS) {
            return adc_values[channel];
        }
        return 0;
    }
};

// ADC with oversampling (improve resolution)
class OversamplingADC {
private:
    static constexpr uint8_t OVERSAMPLE_RATE = 16;  // 16x
    static constexpr uint8_t EFFECTIVE_BITS = 14;   // 12 + 2 (log2(16))
    
public:
    uint16_t read_with_oversampling(uint8_t channel) {
        uint32_t sum = 0;
        
        // Take 16 samples and average
        for (int i = 0; i < OVERSAMPLE_RATE; i++) {
            sum += read_adc_raw(channel);
        }
        
        // Average (divide by 16 = right shift 4)
        uint16_t average = sum >> 4;
        
        // Effective resolution improved from 12-bit to ~14-bit
        return average;
    }
    
    /*
    Benefits:
    - Effectively increases resolution
    - Cost: N×M slower (N channels, M oversample rate)
    
    12-bit ADC + 4× oversampling = ~14-bit effective
    12-bit ADC + 16× oversampling = ~14-bit effective
    
    Typical automotive:
    - Basic analog signals: 1× (no oversampling)
    - Sensor inputs: 2-4× oversampling
    - High-precision: 8-16× oversampling
    */
};

// Successive Approximation ADC (typical implementation)
class SAR_ADC {
private:
    static constexpr uint16_t DAC_REF_VOLTAGE = 5000;  // mV
    
public:
    uint16_t sar_conversion(uint16_t analog_voltage_mv) {
        uint16_t result = 0;
        uint16_t guess = 0x800;  // Start with MSB (halfway)
        
        for (int bit = 11; bit >= 0; bit--) {
            // Set bit
            result |= guess;
            
            // Convert back to voltage
            uint16_t dac_voltage = (result * DAC_REF_VOLTAGE) >> 12;
            
            // Compare
            if (dac_voltage > analog_voltage_mv) {
                result &= ~guess;  // Clear bit if too high
            }
            
            guess >>= 1;  // Shift to next bit
        }
        
        return result;
    }
    
    /*
    Process: 12 clock cycles for 12-bit result
    - Bit 11: Check if > Vref/2
    - Bit 10: Check if > Vref/4 or 3×Vref/4
    - ...
    - Bit 0: Final bit
    
    Speed: Fast (good for real-time)
    Power: Moderate
    */
};

// Practical automotive ADC application
class EngineControlADC {
private:
    // Engine sensors
    ADC_Config adc;
    MultiChannelADC multi_adc;
    
    // Channel assignments
    enum SensorChannels {
        THROTTLE_POSITION = 0,
        MANIFOLD_PRESSURE = 1,
        ENGINE_TEMPERATURE = 2,
        OXYGEN_SENSOR = 3,
        BATTERY_VOLTAGE = 4,
        FUEL_PRESSURE = 5
    };
    
    // Filters
    ExponentialFilter throttle_filter{0.3f, 0.0f};
    ExponentialFilter map_filter{0.2f, 0.0f};
    ExponentialFilter temp_filter{0.1f, 0.0f};
    
public:
    void initialize() {
        adc.initialize(ADC_Config::RES_12BIT, 
                      ADC_Config::SAMPLE_100kHz);
    }
    
    struct SensorReadings {
        uint16_t throttle_position;     // 0-100%
        uint16_t manifold_pressure;     // kPa
        int16_t engine_temperature;     // °C
        uint16_t oxygen_sensor;         // 0-1023
        uint16_t battery_voltage;       // 10-16V
        uint16_t fuel_pressure;         // bar
    };
    
    SensorReadings read_all_sensors() {
        SensorReadings readings;
        
        // Read raw ADC values
        uint16_t raw_throttle = multi_adc.get_channel(THROTTLE_POSITION);
        uint16_t raw_map = multi_adc.get_channel(MANIFOLD_PRESSURE);
        uint16_t raw_temp = multi_adc.get_channel(ENGINE_TEMPERATURE);
        
        // Convert and filter
        readings.throttle_position = (throttle_filter.update(raw_throttle) / 40.95f);  // 0-100%
        readings.manifold_pressure = (map_filter.update(raw_map) * 250 / 4096);  // 0-250 kPa
        readings.engine_temperature = (temp_filter.update(raw_temp) * 200 / 4096) - 40;  // -40 to 160°C
        
        return readings;
    }
};
```

---

*[Content continues with more advanced topics...]*

