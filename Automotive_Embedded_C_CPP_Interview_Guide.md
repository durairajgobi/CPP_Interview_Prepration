# C/C++ Automotive Embedded Systems Interview Guide
## 100 Questions & Answers with Code Examples

---

## Table of Contents
1. [Core C/C++ Fundamentals](#core-cc-fundamentals)
2. [Memory Management](#memory-management)
3. [Real-time Systems & RTOS](#real-time-systems--rtos)
4. [CAN Bus & Communication](#can-bus--communication)
5. [Interrupt Handling & ISR](#interrupt-handling--isr)
6. [Hardware Abstraction Layer](#hardware-abstraction-layer)
7. [MISRA-C Compliance & Safety](#misra-c-compliance--safety)
8. [Performance Optimization](#performance-optimization)
9. [Automotive Protocols](#automotive-protocols)
10. [Debugging & Testing](#debugging--testing)

---

## Core C/C++ Fundamentals

### Q1: Explain the difference between stack and heap memory. Why is this critical in automotive systems?

**Answer:**
- **Stack**: Automatically allocated/deallocated memory, LIFO (Last-In-First-Out), limited size, fast access, thread-safe per thread
- **Heap**: Manually allocated/deallocated (or via smart pointers), larger size, slower access, shared across threads, fragmentation risk
- **Critical in automotive**: Predictability is essential; stack usage must be bounded and deterministic. Heap fragmentation can cause out-of-memory conditions at worst times.

```cpp
// Stack allocation (predictable, safe)
void calculate_vehicle_speed() {
    uint16_t speed_rpm;        // Stack: predictable, automatic cleanup
    uint8_t gear_position;     // Fixed size, known at compile time
    
    // Process
    speed_rpm = read_engine_speed();
}

// Heap allocation (risky in embedded)
void process_sensor_data_bad() {
    int* sensor_buffer = new int[1000];  // Dangerous in automotive!
    // Risk: fragmentation, memory leak if exception occurs
    delete[] sensor_buffer;
}

// Safe approach with pre-allocated pool
#define MAX_SENSOR_SAMPLES 1000
uint16_t sensor_pool[MAX_SENSOR_SAMPLES];  // Static allocation

void process_sensor_data_safe() {
    // Use pre-allocated pool - predictable, no fragmentation
    for (int i = 0; i < MAX_SENSOR_SAMPLES; i++) {
        sensor_pool[i] = read_adc();
    }
}
```

---

### Q2: What are volatile variables and why are they essential in embedded systems?

**Answer:**
Volatile tells the compiler that a variable's value can change unexpectedly (by hardware, ISR, or another thread) and shouldn't be optimized away.

```cpp
// Without volatile - WRONG
uint8_t status_register;
while (!status_register) {
    // Compiler might optimize to: while(true) because it doesn't see 
    // status_register changing in this loop
}

// Correct usage
volatile uint8_t status_register = 0;  // Updated by ISR
while (!status_register) {
    // Compiler must check status_register every iteration
}

// Typical automotive use case
volatile uint16_t* const CAN_RECEIVE_BUFFER = (uint16_t*)0x40000000;
volatile uint8_t* const GPIO_PORT_A = (uint8_t*)0x50000000;

void check_can_message() {
    uint16_t msg = *CAN_RECEIVE_BUFFER;  // Must read from hardware each time
    if (msg & 0x0001) {
        *GPIO_PORT_A = 0xFF;  // Must write to hardware each time
    }
}
```

---

### Q3: Explain the const and constexpr keywords and their differences.

**Answer:**
- **const**: Runtime constant, value cannot change after initialization
- **constexpr**: Compile-time constant, evaluated at compile time when possible
- **const volatile**: Can be changed externally but not by your code

```cpp
// const - evaluated at runtime
const uint16_t max_rpm = get_config_value();  // Read from EEPROM

// constexpr - evaluated at compile time
constexpr uint8_t CAN_BUFFER_SIZE = 64;  // Must be compile-time constant

// const volatile - hardware register
volatile const uint16_t* engine_speed_register = (uint16_t*)0x40001000;

// Automotive example
class EngineController {
private:
    static constexpr uint16_t MAX_SAFE_RPM = 7500;  // Compile-time constant
    static constexpr uint8_t NUM_CYLINDERS = 4;
    
    uint16_t current_rpm = 0;  // Can change
    const uint16_t max_configured_rpm;  // Set once in constructor
    
public:
    EngineController(uint16_t max_rpm) : max_configured_rpm(max_rpm) {}
    
    bool is_rpm_safe() const {
        return current_rpm < MIN(MAX_SAFE_RPM, max_configured_rpm);
    }
};
```

---

### Q4: What is the difference between passing by value, reference, and pointer?

**Answer:**

```cpp
// Pass by value - copy is made (expensive for large objects)
void increase_speed_by_value(uint16_t rpm) {
    rpm += 100;  // Only modifies local copy
}

// Pass by reference - direct access to original (efficient, safe)
void increase_speed_by_reference(uint16_t& rpm) {
    rpm += 100;  // Modifies original, no copy overhead
    // Cannot be null
}

// Pass by pointer - can be null, requires dereferencing
void increase_speed_by_pointer(uint16_t* rpm) {
    if (rpm != NULL) {
        *rpm += 100;  // Must dereference
    }
}

// Automotive example
struct VehicleState {
    uint16_t engine_rpm;
    uint8_t gear;
    int32_t position_x;
    int32_t position_y;
};  // 12 bytes

// BAD: Copies 12 bytes every call
void process_state_by_value(VehicleState state) {
    state.engine_rpm += 10;  // Doesn't affect original
}

// GOOD: No copy, but can modify
void process_state_by_ref(VehicleState& state) {
    state.engine_rpm += 10;  // Modifies original, no copy
}

// GOOD: No copy, prevents modification (safe)
void display_state(const VehicleState& state) {
    printf("RPM: %d, Gear: %d\n", state.engine_rpm, state.gear);
}

// Use in real-time loop
int main() {
    VehicleState vehicle = {0, 0, 0, 0};
    
    // Called every 10ms in real-time task
    while (1) {
        process_state_by_ref(vehicle);  // Efficient
        display_state(vehicle);          // Safe, efficient
    }
}
```

---

### Q5: Explain bit manipulation and give automotive examples.

**Answer:**
Bit manipulation is crucial in automotive for register access, flags, and message parsing.

```cpp
// Basic bit operations
#define SET_BIT(reg, bit)     ((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit)   ((reg) &= ~(1 << (bit)))
#define TOGGLE_BIT(reg, bit)  ((reg) ^= (1 << (bit)))
#define CHECK_BIT(reg, bit)   ((reg) & (1 << (bit)))

// Vehicle status flags (using bit fields)
typedef struct {
    uint8_t engine_running : 1;      // Bit 0
    uint8_t brake_pressed : 1;       // Bit 1
    uint8_t accelerator : 1;         // Bit 2
    uint8_t cruise_enabled : 1;      // Bit 3
    uint8_t reserved : 4;            // Bits 4-7
} VehicleFlags;

VehicleFlags flags = {0};

// Set flags
SET_BIT(flags, 0);  // Engine running

// Check flags
if (CHECK_BIT(flags, 1)) {
    // Brake is pressed
}

// CAN message parsing example
// DBC message: Engine data with various signals
typedef struct {
    uint8_t byte0;  // Engine RPM (bits 0-15)
    uint8_t byte1;
    uint8_t byte2;  // Engine temperature (bits 16-23)
    uint8_t byte3;  // Coolant pressure (bits 24-31)
    uint8_t byte4;  // Fuel level (bits 32-39)
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
} CanEngineMessage;

uint16_t extract_engine_rpm(const CanEngineMessage* msg) {
    return ((uint16_t)msg->byte1 << 8) | msg->byte0;
}

uint8_t extract_engine_temp(const CanEngineMessage* msg) {
    return msg->byte2;
}

// Bit field access in status register
uint32_t status_register = 0;

// Set error flags (bits 8-15)
status_register |= (0x01 << 8);  // Set error flag 0
status_register |= (0x02 << 8);  // Set error flag 1

// Check specific bit
bool is_error_0_set = (status_register & (1 << 8)) != 0;

// Clear multiple bits
status_register &= ~(0xFF << 8);  // Clear error flag byte
```

---

### Q6: What are function pointers and how are they used in automotive systems?

**Answer:**
Function pointers allow dynamic function selection, essential for callbacks and state machines.

```cpp
// Basic function pointer
typedef uint16_t (*GearCalculator)(uint16_t rpm);

uint16_t calculate_gear_manual(uint16_t rpm) {
    if (rpm < 1500) return 1;
    if (rpm < 3000) return 2;
    return 3;
}

uint16_t calculate_gear_sport(uint16_t rpm) {
    if (rpm < 2000) return 1;
    if (rpm < 4000) return 2;
    return 3;
}

// Use function pointer
GearCalculator gear_calc = calculate_gear_manual;
uint16_t next_gear = gear_calc(3500);  // Call function via pointer

// Automotive: ISR callback pattern
typedef void (*CanMessageHandler)(const uint8_t* data, uint8_t length);

struct CanMessageConfig {
    uint16_t message_id;
    CanMessageHandler handler;
};

void handle_engine_data(const uint8_t* data, uint8_t length) {
    uint16_t rpm = (data[1] << 8) | data[0];
    // Process engine data
}

void handle_transmission_data(const uint8_t* data, uint8_t length) {
    uint8_t gear = data[0] & 0x0F;
    // Process transmission data
}

// Message routing
const CanMessageConfig can_handlers[] = {
    {0x100, handle_engine_data},
    {0x200, handle_transmission_data},
};

void process_can_message(uint16_t msg_id, const uint8_t* data) {
    for (int i = 0; i < sizeof(can_handlers)/sizeof(can_handlers[0]); i++) {
        if (can_handlers[i].message_id == msg_id) {
            can_handlers[i].handler(data, 8);  // Call appropriate handler
            break;
        }
    }
}

// State machine example using function pointers
typedef void (*StateFunction)(void);

struct StateMachine {
    StateFunction current_state;
};

void state_idle() {
    // Handle idle state
}

void state_starting() {
    // Handle starting state
}

void state_running() {
    // Handle running state
}

StateMachine engine = {state_idle};

// Transition to new state
engine.current_state = state_starting;
engine.current_state();  // Execute current state
```

---

## Memory Management

### Q7: Explain static memory allocation vs. dynamic and why static is preferred in automotive.

**Answer:**

```cpp
// Static allocation - PREFERRED for embedded
class EngineController {
private:
    static constexpr uint16_t MAX_SENSOR_READINGS = 256;
    uint16_t sensor_buffer[MAX_SENSOR_READINGS];  // Stack or data section
    uint16_t buffer_index = 0;
    
public:
    void add_sensor_reading(uint16_t value) {
        if (buffer_index < MAX_SENSOR_READINGS) {
            sensor_buffer[buffer_index++] = value;
        }
    }
};

// Dynamic allocation - AVOID if possible
class BadSensorBuffer {
private:
    uint16_t* sensor_buffer;
    uint16_t buffer_size;
    
public:
    BadSensorBuffer(uint16_t size) : buffer_size(size) {
        sensor_buffer = new uint16_t[size];  // Risky!
        // What if allocation fails? No exception handling in embedded
    }
    
    ~BadSensorBuffer() {
        delete[] sensor_buffer;  // Can be missed
    }
};

// Object pool pattern (safe dynamic behavior with static allocation)
class SensorReadingPool {
private:
    static constexpr uint16_t MAX_READINGS = 1000;
    static constexpr uint16_t MAX_ACTIVE_READINGS = 50;
    
    struct Reading {
        uint16_t value;
        uint32_t timestamp;
        bool in_use;
    };
    
    Reading pool[MAX_READINGS];
    uint16_t free_count = MAX_READINGS;
    
public:
    SensorReadingPool() {
        for (int i = 0; i < MAX_READINGS; i++) {
            pool[i].in_use = false;
        }
    }
    
    Reading* allocate_reading() {
        for (int i = 0; i < MAX_READINGS; i++) {
            if (!pool[i].in_use) {
                pool[i].in_use = true;
                free_count--;
                return &pool[i];
            }
        }
        return NULL;  // Out of resources - predictable failure
    }
    
    void release_reading(Reading* reading) {
        if (reading >= pool && reading < pool + MAX_READINGS) {
            reading->in_use = false;
            free_count++;
        }
    }
    
    uint16_t get_free_count() const { return free_count; }
};

int main() {
    SensorReadingPool pool;
    
    Reading* reading1 = pool.allocate_reading();
    Reading* reading2 = pool.allocate_reading();
    
    // Use readings...
    
    pool.release_reading(reading1);
    pool.release_reading(reading2);
}
```

---

### Q8: What are memory leaks and how to prevent them in automotive systems?

**Answer:**

```cpp
// Memory leak example
void bad_function() {
    uint8_t* buffer = new uint8_t[1024];
    
    if (some_error_condition) {
        return;  // LEAK! buffer never freed
    }
    
    process_buffer(buffer);
    delete[] buffer;
}

// Prevention strategy 1: RAII (Resource Acquisition Is Initialization)
template<typename T>
class AutoPtr {
private:
    T* ptr;
    
public:
    AutoPtr(T* p) : ptr(p) {}
    
    ~AutoPtr() {
        delete ptr;  // Automatic cleanup
    }
    
    T* operator->() { return ptr; }
    T& operator*() { return *ptr; }
    
    // Disable copying
    AutoPtr(const AutoPtr&) = delete;
    AutoPtr& operator=(const AutoPtr&) = delete;
};

void good_function() {
    AutoPtr<uint8_t> buffer(new uint8_t[1024]);
    
    if (some_error_condition) {
        return;  // Destructor called automatically!
    }
    
    process_buffer(buffer.operator->());
}

// Prevention strategy 2: Pre-allocated buffers
class SafeCANBuffer {
private:
    static constexpr uint8_t MAX_MESSAGES = 32;
    
    struct Message {
        uint16_t id;
        uint8_t data[8];
        uint8_t dlc;
    };
    
    Message messages[MAX_MESSAGES];
    uint8_t write_index = 0;
    uint8_t read_index = 0;
    
public:
    bool enqueue_message(uint16_t id, const uint8_t* data, uint8_t dlc) {
        uint8_t next_index = (write_index + 1) % MAX_MESSAGES;
        
        if (next_index == read_index) {
            return false;  // Queue full, no allocation
        }
        
        messages[write_index].id = id;
        memcpy(messages[write_index].data, data, dlc);
        messages[write_index].dlc = dlc;
        write_index = next_index;
        
        return true;
    }
    
    bool dequeue_message(Message& out) {
        if (read_index == write_index) {
            return false;  // Queue empty
        }
        
        out = messages[read_index];
        read_index = (read_index + 1) % MAX_MESSAGES;
        
        return true;
    }
};

// Prevention strategy 3: Smart pointers (C++11+)
#include <memory>

void modern_function() {
    std::unique_ptr<uint8_t[]> buffer(new uint8_t[1024]);
    // Automatically freed when buffer goes out of scope
    
    process_buffer(buffer.get());
}  // No manual delete needed!
```

---

### Q9: Explain stack overflow and how to prevent it in embedded systems.

**Answer:**

```cpp
// Stack overflow risks
void recursive_function(int depth) {
    uint8_t large_buffer[256];  // Each call adds 256 bytes to stack
    
    if (depth > 0) {
        recursive_function(depth - 1);  // Dangerous!
    }
}

// BAD: 1000 recursive calls * 256 bytes = 256KB stack needed!

// Prevention: Convert to iterative
void safe_function(int max_depth) {
    uint8_t large_buffer[256];
    
    for (int depth = 0; depth < max_depth; depth++) {
        // Process without recursion
        memset(large_buffer, 0, 256);
    }
}

// Safe recursive version with depth guard
static int recursion_depth = 0;
static constexpr int MAX_RECURSION = 5;

void guarded_recursive_function(int value) {
    if (recursion_depth >= MAX_RECURSION) {
        return;  // Prevent stack overflow
    }
    
    uint8_t small_buffer[32];  // Minimal stack usage
    recursion_depth++;
    
    if (value > 0) {
        guarded_recursive_function(value - 1);
    }
    
    recursion_depth--;
}

// Stack usage monitoring
class StackMonitor {
private:
    uint32_t stack_start;
    uint32_t stack_end;
    uint32_t current_sp;
    
public:
    StackMonitor() {
        // Get stack boundaries from linker script
        extern uint32_t _stack_start;
        extern uint32_t _stack_end;
        stack_start = (uint32_t)&_stack_start;
        stack_end = (uint32_t)&_stack_end;
    }
    
    uint32_t get_stack_usage() {
        uint32_t sp;
        asm("mov %0, sp" : "=r"(sp));  // Get current SP
        return stack_end - sp;
    }
    
    uint32_t get_max_stack_usage() {
        // Compare against known pattern
        // Usually done by filling stack with pattern at startup
        return 0;
    }
};

// Static allocation to prevent large stack allocations
struct SensorData {
    uint16_t adc_values[256];
    uint8_t status_flags;
};

// Use global/static instead of stack
static SensorData sensor_data;  // Data section, not stack

void process_sensors() {
    // Use pre-allocated sensor_data
    for (int i = 0; i < 256; i++) {
        sensor_data.adc_values[i] = read_adc();
    }
}

// Stack guard pattern
void function_with_stack_guard() {
    volatile uint32_t stack_guard = 0xDEADBEEF;
    
    // Do work...
    
    // Check guard at end
    if (stack_guard != 0xDEADBEEF) {
        // Stack overflow detected!
        panic("Stack overflow");
    }
}
```

---

### Q10: What is the difference between heap fragmentation and stack fragmentation?

**Answer:**

```cpp
// Heap fragmentation example
void heap_fragmentation_demo() {
    // First allocation pass
    uint8_t* ptr1 = new uint8_t[100];  // Address 1000
    uint8_t* ptr2 = new uint8_t[100];  // Address 1100
    uint8_t* ptr3 = new uint8_t[100];  // Address 1200
    
    // Free middle block
    delete[] ptr2;  // Leaves hole at 1100
    
    // Now heap looks like: [used][FREE 100B][used]
    
    // Try to allocate 200 bytes - won't fit in hole!
    uint8_t* ptr4 = new uint8_t[200];  // Has to allocate elsewhere
    
    // Result: Wasted 100 bytes and increased total heap used
    delete[] ptr1;
    delete[] ptr3;
    delete[] ptr4;
}

// Heap fragmentation in automotive
class CAN_BufferManager {
private:
    static constexpr int NUM_MESSAGES = 100;
    
    struct Message {
        uint16_t id;
        uint8_t* data;  // Variable size allocations = fragmentation!
        uint8_t dlc;
        bool allocated;
    };
    
    Message messages[NUM_MESSAGES];
    
public:
    // RISKY: Different sized allocations
    void add_message(uint16_t id, uint8_t dlc) {
        for (int i = 0; i < NUM_MESSAGES; i++) {
            if (!messages[i].allocated) {
                messages[i].data = new uint8_t[dlc];  // Variable size
                messages[i].allocated = true;
                break;
            }
        }
    }
    
    // SAFE: Fixed size allocation
    void add_message_safe(uint16_t id, uint8_t dlc) {
        for (int i = 0; i < NUM_MESSAGES; i++) {
            if (!messages[i].allocated) {
                messages[i].data = &fixed_message_pool[i][0];  // Fixed 8 bytes
                messages[i].allocated = true;
                break;
            }
        }
    }
    
private:
    uint8_t fixed_message_pool[NUM_MESSAGES][8];  // No fragmentation
};

// Memory pool to prevent fragmentation
class FixedMemoryPool {
private:
    static constexpr uint16_t POOL_SIZE = 10000;
    static constexpr uint16_t BLOCK_SIZE = 32;
    static constexpr uint16_t NUM_BLOCKS = POOL_SIZE / BLOCK_SIZE;
    
    uint8_t pool[POOL_SIZE];
    bool block_used[NUM_BLOCKS];
    
public:
    uint8_t* allocate(uint16_t size) {
        uint16_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
        uint16_t consecutive = 0;
        uint16_t start_block = 0;
        
        // Find consecutive free blocks
        for (uint16_t i = 0; i < NUM_BLOCKS; i++) {
            if (!block_used[i]) {
                if (consecutive == 0) {
                    start_block = i;
                }
                consecutive++;
                
                if (consecutive >= blocks_needed) {
                    // Mark blocks as used
                    for (uint16_t j = 0; j < blocks_needed; j++) {
                        block_used[start_block + j] = true;
                    }
                    return &pool[start_block * BLOCK_SIZE];
                }
            } else {
                consecutive = 0;
            }
        }
        
        return NULL;  // Out of memory
    }
    
    void deallocate(uint8_t* ptr, uint16_t size) {
        if (ptr < pool || ptr >= pool + POOL_SIZE) {
            return;  // Not in pool
        }
        
        uint16_t offset = ptr - pool;
        uint16_t start_block = offset / BLOCK_SIZE;
        uint16_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;
        
        for (uint16_t i = 0; i < blocks_needed; i++) {
            block_used[start_block + i] = false;
        }
    }
};
```

---

## Real-time Systems & RTOS

### Q11: Explain rate monotonic scheduling (RMS) and its application in automotive ECU.

**Answer:**
RMS assigns priorities based on task periods: shorter period = higher priority.

```cpp
// Task structure
struct Task {
    uint32_t period_ms;      // Task period
    uint32_t execution_time;  // WCET (Worst Case Execution Time)
    uint8_t priority;         // RMS: higher for shorter period
    void (*task_function)();
    uint32_t last_execution;
};

// ECU tasks example
void engine_control_task() {
    // Critical: 5ms period, 1ms WCET
    uint16_t rpm = read_engine_speed();
    uint8_t load = read_throttle_position();
    set_fuel_injection_duration(calculate_injection(rpm, load));
}

void brake_monitoring_task() {
    // Critical: 10ms period, 2ms WCET
    uint16_t brake_pressure = read_pressure_sensor();
    if (brake_pressure > THRESHOLD) {
        activate_abs();
    }
}

void diagnostic_task() {
    // Non-critical: 100ms period, 5ms WCET
    check_system_health();
    update_diagnostic_codes();
}

// RMS scheduling calculation
class RMSScheduler {
private:
    static constexpr uint8_t MAX_TASKS = 10;
    Task tasks[MAX_TASKS];
    uint8_t task_count = 0;
    
public:
    void add_task(Task t) {
        if (task_count < MAX_TASKS) {
            tasks[task_count++] = t;
        }
    }
    
    bool calculate_rms_priorities() {
        // Sort by period (RMS rule: shorter period = higher priority)
        for (int i = 0; i < task_count; i++) {
            uint8_t highest_priority = 15;  // Assuming 0=lowest, 15=highest
            
            for (int j = i; j < task_count; j++) {
                if (tasks[j].period_ms < tasks[i].period_ms) {
                    Task temp = tasks[i];
                    tasks[i] = tasks[j];
                    tasks[j] = temp;
                }
            }
            
            tasks[i].priority = highest_priority - i;
        }
        
        // Verify schedulability (Liu & Layland bound)
        return check_schedulability();
    }
    
    bool check_schedulability() {
        // Liu-Layland bound: sum(Ci/Ti) <= N * (2^(1/N) - 1)
        // For practical automotive: sum(Ci/Ti) should be < 0.9
        
        double utilization = 0.0;
        
        for (int i = 0; i < task_count; i++) {
            double task_util = (double)tasks[i].execution_time / 
                              (double)tasks[i].period_ms;
            utilization += task_util;
            
            if (utilization > 0.9) {
                return false;  // Potentially unschedulable
            }
        }
        
        return true;
    }
    
    void run_scheduler() {
        while (1) {
            for (int i = 0; i < task_count; i++) {
                uint32_t current_time = get_system_ticks();
                
                // Check if task is due
                if ((current_time - tasks[i].last_execution) >= tasks[i].period_ms) {
                    // Execute task (in real implementation, would use RTOS)
                    tasks[i].task_function();
                    tasks[i].last_execution = current_time;
                }
            }
        }
    }
};

int main() {
    RMSScheduler scheduler;
    
    // Add tasks in order of period (RMS will assign priorities)
    Task engine_task = {5, 1, 0, engine_control_task, 0};
    Task brake_task = {10, 2, 0, brake_monitoring_task, 0};
    Task diag_task = {100, 5, 0, diagnostic_task, 0};
    
    scheduler.add_task(engine_task);
    scheduler.add_task(brake_task);
    scheduler.add_task(diag_task);
    
    if (scheduler.calculate_rms_priorities()) {
        // Schedulable - run the system
        scheduler.run_scheduler();
    }
}
```

---

### Q12: What is deadline monotonic scheduling and how does it differ from RMS?

**Answer:**
DMS assigns priorities based on task deadlines (not periods). If deadline < period, it's more optimal.

```cpp
// Deadline Monotonic vs Rate Monotonic
struct RealTimeTask {
    uint32_t period_ms;
    uint32_t deadline_ms;    // Can be different from period
    uint32_t wcet_ms;        // Worst Case Execution Time
    uint8_t priority;
};

// Example: Task with period=20ms but deadline=10ms (task must complete in 10ms)
RealTimeTask task1 = {
    .period_ms = 20,
    .deadline_ms = 10,       // Stricter than period!
    .wcet_ms = 8
};

RealTimeTask task2 = {
    .period_ms = 30,
    .deadline_ms = 30,       // Deadline = period
    .wcet_ms = 5
};

// RMS: Priorities based on period
// task1: period 20ms → higher priority
// task2: period 30ms → lower priority

// DMS: Priorities based on deadline
// task1: deadline 10ms → HIGHER priority
// task2: deadline 30ms → lower priority

class DeadlineMonotonicScheduler {
private:
    static constexpr uint8_t MAX_TASKS = 10;
    RealTimeTask tasks[MAX_TASKS];
    uint8_t task_count = 0;
    
public:
    void add_task(RealTimeTask t) {
        tasks[task_count++] = t;
    }
    
    void assign_dms_priorities() {
        // Sort by deadline (shortest deadline = highest priority)
        for (int i = 0; i < task_count; i++) {
            for (int j = i + 1; j < task_count; j++) {
                if (tasks[j].deadline_ms < tasks[i].deadline_ms) {
                    RealTimeTask temp = tasks[i];
                    tasks[i] = tasks[j];
                    tasks[j] = temp;
                }
            }
            tasks[i].priority = task_count - i;  // Higher number = higher priority
        }
    }
    
    bool check_dms_feasibility() {
        // More complex than RMS, uses response time analysis
        for (int i = 0; i < task_count; i++) {
            uint32_t response_time = tasks[i].wcet_ms;
            
            // Add blocking time from higher priority tasks
            for (int j = 0; j < i; j++) {
                // Number of times higher priority task executes
                uint32_t num_preemptions = 
                    (response_time + tasks[j].period_ms - 1) / tasks[j].period_ms;
                response_time += num_preemptions * tasks[j].wcet_ms;
            }
            
            // Check if response time exceeds deadline
            if (response_time > tasks[i].deadline_ms) {
                return false;  // Task will miss deadline
            }
        }
        
        return true;
    }
};

// Automotive example: Brake control with tight deadline
RealTimeTask engine_periodic = {
    .period_ms = 20,
    .deadline_ms = 20,
    .wcet_ms = 5
};

RealTimeTask brake_emergency = {
    .period_ms = 50,
    .deadline_ms = 5,        // Much tighter deadline!
    .wcet_ms = 4
};

// DMS gives higher priority to brake task (5ms deadline vs 20ms deadline)
// Even though it has longer period, it must respond faster!
```

---

### Q13: Explain priority inversion and solutions in automotive RTOS.

**Answer:**
Priority inversion occurs when a high-priority task waits for a low-priority task holding a lock.

```cpp
// Classic priority inversion scenario
uint8_t engine_data = 0;
Mutex engine_mutex;  // Protects engine_data

void low_priority_task() {
    engine_mutex.lock();      // Acquire lock
    engine_data = read_sensor();
    delay_ms(100);            // Long critical section!
    engine_mutex.unlock();
}

void high_priority_task() {
    // This should run frequently!
    engine_mutex.lock();       // BLOCKED! Low priority task still has lock
    // Cannot proceed even though higher priority
    update_display(engine_data);
    engine_mutex.unlock();
}

// Priority inversion occurs:
// High priority waiting for low priority!

// Solution 1: Priority Inheritance Protocol
class MutexWithPriorityInheritance {
private:
    bool locked = false;
    uint8_t owner_priority = 0;
    uint8_t original_owner_priority = 0;
    
public:
    void lock(uint8_t task_priority) {
        while (locked) {
            // Spin wait
        }
        
        locked = true;
        owner_priority = task_priority;
        original_owner_priority = task_priority;
    }
    
    void unlock() {
        locked = false;
    }
    
    void raise_priority(uint8_t waiting_priority) {
        if (waiting_priority > owner_priority) {
            // Temporarily raise lock holder priority
            owner_priority = waiting_priority;
        }
    }
    
    void restore_priority() {
        owner_priority = original_owner_priority;
    }
};

// Solution 2: Priority Ceiling Protocol (RTOS level)
class PriorityCeilingMutex {
private:
    bool locked = false;
    uint8_t ceiling_priority = 15;  // Highest possible priority
    
public:
    void lock(uint8_t task_priority) {
        // Task priority must be <= ceiling priority
        if (task_priority > ceiling_priority) {
            return;  // Error: task priority too high
        }
        
        while (locked) {
            // Spin
        }
        
        locked = true;
    }
    
    void unlock() {
        locked = false;
    }
};

// Solution 3: Structured locking with timeouts
class SafeCANMessageQueue {
private:
    struct Message {
        uint16_t id;
        uint8_t data[8];
    };
    
    static constexpr uint8_t QUEUE_SIZE = 32;
    Message queue[QUEUE_SIZE];
    uint8_t write_ptr = 0;
    uint8_t read_ptr = 0;
    bool queue_lock = false;
    
public:
    bool enqueue(const Message& msg, uint32_t timeout_ms) {
        uint32_t start_time = get_ticks();
        
        // Wait for lock with timeout
        while (queue_lock) {
            if ((get_ticks() - start_time) > timeout_ms) {
                return false;  // Timeout - no priority inversion
            }
        }
        
        queue_lock = true;
        
        if (((write_ptr + 1) % QUEUE_SIZE) != read_ptr) {
            queue[write_ptr] = msg;
            write_ptr = (write_ptr + 1) % QUEUE_SIZE;
            queue_lock = false;
            return true;
        }
        
        queue_lock = false;
        return false;  // Queue full
    }
    
    bool dequeue(Message& msg, uint32_t timeout_ms) {
        uint32_t start_time = get_ticks();
        
        while (queue_lock) {
            if ((get_ticks() - start_time) > timeout_ms) {
                return false;
            }
        }
        
        queue_lock = true;
        
        if (read_ptr != write_ptr) {
            msg = queue[read_ptr];
            read_ptr = (read_ptr + 1) % QUEUE_SIZE;
            queue_lock = false;
            return true;
        }
        
        queue_lock = false;
        return false;  // Queue empty
    }
};

// Best practice: Minimize lock time
void best_practice_task() {
    uint8_t local_data;
    
    {
        // Lock scope - minimize critical section
        engine_mutex.lock();
        local_data = engine_data;  // Quick read
        engine_mutex.unlock();
    }
    
    // Process outside of critical section
    update_display(local_data);
}
```

---

### Q14: What is context switching overhead and how to minimize it?

**Answer:**

```cpp
// Context switch overhead components
struct ContextSwitchOverhead {
    uint32_t save_registers;        // 10-50 cycles
    uint32_t update_memory_context; // 5-20 cycles
    uint32_t restore_registers;     // 10-50 cycles
    uint32_t cache_invalidation;    // 100-1000 cycles
    // Total: 150-1200 cycles typical
};

// Measurement example
uint32_t measure_context_switch() {
    uint32_t start = get_cycle_counter();
    
    // Force context switch
    trigger_rtos_tick();
    
    uint32_t end = get_cycle_counter();
    return end - start;
}

// Strategies to minimize context switches

// Strategy 1: Reduce number of tasks
class OptimizedECU {
private:
    // BAD: 20 separate tasks = 20 context switches per cycle
    // GOOD: 3 tasks at different priorities = 3 context switches
    
public:
    void main_fast_task() {
        // 5ms period - engine control, ABS, etc
        // Combines multiple functions
        engine_control();
        abs_control();
    }
    
    void main_medium_task() {
        // 50ms period - transmission, climate
        transmission_control();
        climate_control();
    }
    
    void diagnostic_task() {
        // 1000ms period - diagnostics
        run_diagnostics();
    }
};

// Strategy 2: Rate monotonic to reduce preemptions
// Fewer preemptions = fewer context switches

class ContextSwitchTracker {
private:
    uint32_t total_switches = 0;
    uint32_t task_switches[10] = {0};
    
public:
    void log_switch(uint8_t from_task, uint8_t to_task) {
        total_switches++;
        task_switches[to_task]++;
    }
    
    void report() {
        printf("Total context switches: %lu\n", total_switches);
        for (int i = 0; i < 10; i++) {
            printf("Task %d: %lu switches\n", i, task_switches[i]);
        }
    }
};

// Strategy 3: Per-task stacks to minimize cache pollution
// Each task has dedicated stack = better cache locality

class TaskStack {
private:
    static constexpr uint16_t STACK_SIZE = 512;
    uint8_t stack_space[STACK_SIZE] __attribute__((aligned(32)));
    uint8_t* stack_pointer;
    
public:
    TaskStack() : stack_pointer(stack_space + STACK_SIZE - 1) {}
    
    uint8_t* get_stack_pointer() { return stack_pointer; }
};

// Strategy 4: Spinlocks for very short critical sections
class FastLock {
private:
    volatile uint8_t locked = 0;
    
public:
    void acquire() {
        while (__sync_lock_test_and_set(&locked, 1)) {
            // Spin - cheaper than context switch for <100 cycles
        }
    }
    
    void release() {
        __sync_lock_release(&locked);
    }
};

// Strategy 5: Lock-free data structures
template<typename T, uint16_t SIZE>
class LockFreeQueue {
private:
    T buffer[SIZE];
    volatile uint16_t write_index = 0;
    volatile uint16_t read_index = 0;
    
public:
    // No locks needed - atomic operations only
    bool enqueue(const T& item) {
        uint16_t next = (write_index + 1) % SIZE;
        if (next == read_index) return false;
        
        buffer[write_index] = item;
        write_index = next;
        return true;
    }
    
    bool dequeue(T& item) {
        if (read_index == write_index) return false;
        
        item = buffer[read_index];
        read_index = (read_index + 1) % SIZE;
        return true;
    }
};

// Profiling context switches
void profile_task_switching() {
    uint32_t start_time = get_system_time_ms();
    uint32_t switch_count = 0;
    
    // Simulate 1 second
    while (get_system_time_ms() - start_time < 1000) {
        // Run tasks...
        switch_count++;
    }
    
    // 1000ms / (switch_count * context_switch_time)
    printf("Context switches per ms: %lu\n", switch_count / 1000);
}
```

---

### Q15: Explain CPU vs I/O bound tasks in automotive ECU.

**Answer:**

```cpp
// CPU-bound task: Spends time in computation
void cpu_bound_engine_control() {
    // Typical execution: 80% CPU, 20% I/O wait
    
    uint16_t rpm = read_engine_speed();           // 0.1ms I/O
    uint8_t load = read_throttle();               // 0.1ms I/O
    
    // Heavy computation (4ms)
    uint32_t fuel_amount = calculate_fuel_complex(rpm, load);
    uint8_t spark_advance = calculate_spark_advanced(rpm, load);
    uint16_t cam_position = calculate_cam_timing(rpm, load);
    
    set_fuel_injection(fuel_amount);              // 0.1ms I/O
    set_spark_timing(spark_advance);              // 0.1ms I/O
    set_cam_control(cam_position);                // 0.1ms I/O
    
    // Total: ~5ms, 80% computation
}

// I/O-bound task: Spends time waiting for I/O
void io_bound_transmission_control() {
    // Typical execution: 20% CPU, 80% I/O wait
    
    // Wait for CAN message (could be 20ms)
    uint8_t gear_demand = read_can_message_blocking(CAN_GEAR_ID, 20);
    
    // Quick processing (0.5ms)
    uint16_t solenoid_cmd = gear_to_solenoid_map[gear_demand];
    
    // Wait for ADC (could be 5ms)
    uint16_t pressure = read_adc_blocking(PRESSURE_SENSOR, 5);
    
    // Quick processing (0.5ms)
    if (pressure < MIN_PRESSURE) {
        solenoid_cmd = 0;  // Failsafe
    }
    
    // Total: Could be ~30ms, 80% waiting
}

// Different RTOS strategies for each type

class ECUScheduler {
public:
    void schedule_cpu_bound_task() {
        // CPU-bound: Benefit from higher priority
        // More processor time = faster completion
        // Set high priority to reduce latency
        
        /*
        Task: Engine Control
        Period: 10ms
        WCET: 5ms
        Priority: HIGH
        Reason: Needs predictable timing, blocks easily
        */
    }
    
    void schedule_io_bound_task() {
        // I/O-bound: Benefit from lower priority
        // They block waiting for I/O anyway
        // Other tasks can run while waiting
        
        /*
        Task: Transmission Control
        Period: 50ms
        WCET: 1ms (actual CPU time)
        Blocking time: 20-30ms (waiting for messages)
        Priority: LOW
        Reason: Spends time waiting, doesn't block other tasks
        */
    }
};

// Efficient I/O-bound handling with callbacks
class NonBlockingTransmissionControl {
private:
    uint8_t gear_demand = 0;
    uint16_t can_message_id = 0;
    bool message_received = false;
    
public:
    void start_gear_request() {
        // Non-blocking request
        start_can_receive_with_callback(CAN_GEAR_ID, 
                                       on_gear_message_received);
    }
    
    static void on_gear_message_received(uint16_t msg_id, const uint8_t* data) {
        // Called by CAN ISR when message arrives
        // No blocking!
        gear_demand = data[0];
        message_received = true;
    }
    
    void process_transmission() {
        // No blocking - runs in main task
        if (message_received) {
            uint16_t solenoid_cmd = gear_to_solenoid_map[gear_demand];
            activate_solenoids(solenoid_cmd);
            message_received = false;
        }
    }
};

// Mixed CPU/IO task with optimal pattern
class HybridVehicleControlTask {
private:
    struct SensorData {
        uint16_t engine_rpm;
        uint8_t throttle_position;
        uint16_t brake_pressure;
        bool transmission_gear_received;
    };
    
    SensorData data;
    
public:
    void run_10ms_cycle() {
        // Phase 1: Non-blocking I/O requests
        request_can_transmission_gear();  // Async request
        
        // Phase 2: Process previous sensor data (still valid)
        uint32_t fuel_injection = calculate_fuel(data.engine_rpm, 
                                                 data.throttle_position);
        
        // Phase 3: Read simple sensors (fast I/O)
        data.brake_pressure = read_brake_sensor_blocking(1);  // Only 1ms
        
        // Phase 4: Execute control
        set_fuel_injection(fuel_injection);
        
        // Phase 5: Check for new CAN data
        if (data.transmission_gear_received) {
            process_transmission_gear();
            data.transmission_gear_received = false;
        }
    }
    
private:
    void request_can_transmission_gear() {
        // Non-blocking request
        queue_can_request(CAN_GEAR_ID);
    }
};

// Measuring I/O blocking impact
class IOBlockingProfiler {
public:
    void profile_task() {
        uint32_t start = get_cycle_counter();
        
        uint16_t value = read_adc_blocking(ADC_CHANNEL, 10);
        
        uint32_t cycles_spent = get_cycle_counter() - start;
        uint32_t time_ms = cycles_spent / (CLOCK_MHZ * 1000);
        
        printf("ADC read took %lu ms\n", time_ms);
        printf("Blocked other tasks for this duration\n");
    }
};
```

---

## CAN Bus & Communication

### Q16: Explain CAN protocol and differences between CAN 2.0A and 2.0B.

**Answer:**

```cpp
// CAN 2.0A: Standard identifier (11-bit)
// CAN 2.0B: Extended identifier (29-bit) - can also handle 11-bit

struct CAN_Message_2_0A {
    uint16_t id : 11;           // Standard ID (0-2047)
    uint8_t dlc : 4;            // Data Length Code (0-8)
    uint8_t data[8];
    uint16_t timestamp;
};

struct CAN_Message_2_0B {
    uint32_t id : 29;           // Extended ID (0-536,870,911)
    uint8_t dlc : 4;
    uint8_t ide : 1;            // IDE flag: 1=extended, 0=standard
    uint8_t data[8];
    uint32_t timestamp;
};

// CAN frame structure (Data frame)
typedef struct {
    uint32_t id;                // Message identifier
    uint8_t dlc;                // Data length (0-8)
    uint8_t data[8];            // Payload
    uint8_t is_extended;        // IDE bit
} CAN_Frame;

// CAN controller abstraction
class CANController {
private:
    // Hardware registers
    volatile uint32_t* can_control_register = (uint32_t*)0x40001000;
    volatile uint32_t* can_status_register = (uint32_t*)0x40001004;
    volatile uint32_t* can_tx_buffer = (uint32_t*)0x40001100;
    volatile uint32_t* can_rx_buffer = (uint32_t*)0x40001200;
    
public:
    void init(uint32_t baudrate) {
        // Set bitrate (typical: 500kHz for automotive)
        uint16_t prescaler = (CLOCK_SPEED / (2 * baudrate)) - 1;
        
        *can_control_register &= ~0x00000001;  // Enter init mode
        
        // Configure bit timing for 500kHz
        // Time quantum = 2 * prescaler / CLOCK_SPEED
        // Typically: 8-12 time quanta per bit
        
        uint32_t config = 0x00000000;
        config |= (0x05 << 19);  // SJW = 2 tq
        config |= (0x04 << 16);  // TSEG2 = 3 tq
        config |= (0x0C << 8);   // TSEG1 = 13 tq
        config |= (prescaler & 0xFF);
        
        *can_control_register = config;
        *can_control_register |= 0x00000001;  // Enter normal mode
    }
    
    void send_message(const CAN_Frame& frame) {
        // Wait for TX buffer available
        while (*can_status_register & 0x00000004) {
            // TX pending
        }
        
        // Write frame to transmit buffer
        uint32_t tx_id = frame.id;
        if (frame.is_extended) {
            tx_id |= 0x80000000;  // Set IDE bit
        }
        
        can_tx_buffer[0] = tx_id;
        can_tx_buffer[1] = frame.dlc | 0x00000100;  // Set transmit request
        
        // Write data
        for (int i = 0; i < frame.dlc; i++) {
            can_tx_buffer[2 + i] = frame.data[i];
        }
        
        // Transmit
        *can_control_register |= 0x00000001;
    }
    
    bool receive_message(CAN_Frame& frame) {
        if (!(*can_status_register & 0x00000001)) {
            return false;  // No message received
        }
        
        // Read frame from receive buffer
        uint32_t rx_id = can_rx_buffer[0];
        frame.is_extended = (rx_id & 0x80000000) ? 1 : 0;
        frame.id = rx_id & 0x1FFFFFFF;
        
        frame.dlc = can_rx_buffer[1] & 0x0F;
        
        for (int i = 0; i < frame.dlc; i++) {
            frame.data[i] = can_rx_buffer[2 + i];
        }
        
        // Release receive buffer
        *can_control_register |= 0x00000040;
        
        return true;
    }
};

// DBC-based message definition (simplified)
class CANMessageDefinition {
public:
    // Engine Control message (0x100)
    struct EngineControlMsg {
        uint16_t engine_rpm;           // Bytes 0-1, bits 0-15
        uint8_t engine_load;           // Byte 2, bits 16-23
        int8_t engine_temp;            // Byte 3, bits 24-31 (signed)
    };
    
    // Pack message
    static void pack_engine_message(const EngineControlMsg& msg, 
                                    CAN_Frame& frame) {
        frame.id = 0x100;
        frame.dlc = 4;
        
        // Pack 16-bit RPM (little endian)
        frame.data[0] = msg.engine_rpm & 0xFF;
        frame.data[1] = (msg.engine_rpm >> 8) & 0xFF;
        frame.data[2] = msg.engine_load;
        frame.data[3] = msg.engine_temp;
    }
    
    // Unpack message
    static void unpack_engine_message(const CAN_Frame& frame,
                                     EngineControlMsg& msg) {
        msg.engine_rpm = (uint16_t)frame.data[1] << 8 | frame.data[0];
        msg.engine_load = frame.data[2];
        msg.engine_temp = (int8_t)frame.data[3];
    }
};

// CAN message queue (for application use)
class CANMessageQueue {
private:
    static constexpr uint8_t QUEUE_SIZE = 32;
    
    CAN_Frame queue[QUEUE_SIZE];
    uint8_t write_idx = 0;
    uint8_t read_idx = 0;
    
public:
    void enqueue(const CAN_Frame& frame) {
        uint8_t next_write = (write_idx + 1) % QUEUE_SIZE;
        
        if (next_write != read_idx) {
            queue[write_idx] = frame;
            write_idx = next_write;
        }
    }
    
    bool dequeue(CAN_Frame& frame) {
        if (read_idx == write_idx) {
            return false;
        }
        
        frame = queue[read_idx];
        read_idx = (read_idx + 1) % QUEUE_SIZE;
        return true;
    }
};

// CAN ISR (high priority)
CANMessageQueue can_queue;

void can_receive_isr() {
    CAN_Frame frame;
    
    if (can_controller.receive_message(frame)) {
        can_queue.enqueue(frame);
        // Signal main task
    }
}

// Application task (lower priority)
void can_process_task() {
    CAN_Frame frame;
    
    while (can_queue.dequeue(frame)) {
        if (frame.id == 0x100) {
            CANMessageDefinition::EngineControlMsg msg;
            CANMessageDefinition::unpack_engine_message(frame, msg);
            process_engine_message(msg);
        }
    }
}
```

---

### Q17: Explain CAN FD (CAN with Flexible Data-rate) advantages over classical CAN.

**Answer:**

```cpp
// Classical CAN: 8-byte payload, 1Mbps max
// CAN FD: Up to 64-byte payload, 5Mbps+

struct CAN_Classical_Frame {
    uint32_t id;
    uint8_t dlc;           // 0-8
    uint8_t data[8];       // Fixed 8 bytes
    uint32_t timestamp;
};

struct CAN_FD_Frame {
    uint32_t id;
    uint8_t dlc;           // 0-15 (12, 16, 20, 24, 32, 48, 64 bytes)
    uint8_t data[64];      // Up to 64 bytes!
    uint8_t fdf : 1;       // FD format indicator
    uint8_t brs : 1;       // Bit rate switch
    uint32_t timestamp;
};

// CAN FD advantages:
// 1. Higher bandwidth: More data per frame = fewer frames needed
// 2. Flexibility: Shorter payloads for simple messages, longer for complex
// 3. Faster bitrate: 5-10x faster than classical CAN
// 4. Better efficiency: Fewer frames = less bus loading

class CANFDController {
private:
    volatile uint32_t* canfd_control_reg = (uint32_t*)0x40002000;
    volatile uint32_t* canfd_tx_buffer = (uint32_t*)0x40002100;
    volatile uint32_t* canfd_rx_buffer = (uint32_t*)0x40002200;
    
public:
    void init(uint32_t nominal_bitrate, uint32_t data_bitrate) {
        // Typical: 500kHz nominal, 2Mbps data
        
        *canfd_control_reg |= 0x00000001;  // FD mode enabled
        
        // Configure nominal bitrate (used for arbitration)
        uint16_t nom_prescaler = (CLOCK_SPEED / (2 * nominal_bitrate)) - 1;
        
        // Configure data bitrate (used for data phase)
        uint16_t data_prescaler = (CLOCK_SPEED / (2 * data_bitrate)) - 1;
        
        uint32_t config = (nom_prescaler & 0xFF) | 
                         ((data_prescaler & 0x1F) << 8) |
                         0x00000001;  // BRS enabled
        
        *canfd_control_reg = config;
    }
    
    void send_canfd_message(const CAN_FD_Frame& frame) {
        // Setup frame header
        uint32_t header = 0;
        header |= (frame.id & 0x1FFFFFFF);
        header |= (frame.fdf << 31);      // Set FDF bit
        header |= (frame.brs << 30);      // Set BRS bit
        header |= ((frame.dlc & 0x0F) << 16);
        
        canfd_tx_buffer[0] = header;
        
        // Write payload (up to 64 bytes)
        uint16_t payload_length = dlc_to_bytes[frame.dlc];
        for (int i = 0; i < payload_length; i++) {
            canfd_tx_buffer[1 + (i / 4)] |= (frame.data[i] << ((i % 4) * 8));
        }
        
        // Send
        *canfd_control_reg |= 0x00000001;
    }
    
    bool receive_canfd_message(CAN_FD_Frame& frame) {
        if (!(*canfd_control_reg & 0x00000002)) {
            return false;  // No message
        }
        
        uint32_t header = canfd_rx_buffer[0];
        frame.id = header & 0x1FFFFFFF;
        frame.fdf = (header >> 31) & 1;
        frame.brs = (header >> 30) & 1;
        frame.dlc = (header >> 16) & 0x0F;
        
        // Read payload
        uint16_t payload_length = dlc_to_bytes[frame.dlc];
        for (int i = 0; i < payload_length; i++) {
            frame.data[i] = (canfd_rx_buffer[1 + (i / 4)] >> ((i % 4) * 8)) & 0xFF;
        }
        
        *canfd_control_reg &= ~0x00000002;
        return true;
    }
    
private:
    // DLC to byte length conversion
    static constexpr uint8_t dlc_to_bytes[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8,     // DLC 0-8
        12, 16, 20, 24, 32, 48, 64     // DLC 9-15
    };
};

// Example: Large diagnostic data message (CAN FD)
struct DiagnosticData {
    uint16_t vehicle_speed;
    uint16_t engine_rpm;
    uint8_t gear;
    int8_t engine_temp;
    uint16_t fuel_level;
    uint32_t mileage;
    uint8_t status_flags[4];
    uint8_t error_codes[8];
    // Total: 30 bytes
};

void send_diagnostic_message(const DiagnosticData& diag) {
    CAN_FD_Frame frame;
    frame.id = 0x700;
    frame.fdf = 1;     // Use CAN FD
    frame.brs = 1;     // Use higher bitrate for data
    frame.dlc = 15;    // 64-byte frame
    
    // Pack all data in one frame
    memcpy(frame.data, (uint8_t*)&diag, sizeof(diag));
    
    canfd_controller.send_canfd_message(frame);
}

// Classical CAN would need multiple frames:
// - Frame 1: Speed, RPM, Gear, Temp (8 bytes)
// - Frame 2: Fuel, Mileage (8 bytes)
// - Frame 3: Status flags (8 bytes)
// - Frame 4: Error codes (8 bytes) - partial
// Total: 4 frames vs 1 CAN FD frame

// Bandwidth comparison:
// Classical CAN @ 500kHz: 8 bytes/frame → ~62.5 kB/s max
// CAN FD @ 5Mbps: 64 bytes/frame → ~500 kB/s max

// CAN FD is 8x faster - critical for diagnostics, sensor fusion, etc.
```

---

### Q18: Explain CANopen protocol stack.

**Answer:**
CANopen is an industrial communication profile built on CAN, used in some automotive applications.

```cpp
// CANopen structure
// - COB-ID (Communication Object Identifier)
// - NMT (Network Management)
// - SYNC (Synchronization)
// - EMCY (Emergency)
// - PDO (Process Data Object)
// - SDO (Service Data Object)

// COB-ID format
typedef struct {
    uint16_t function_code : 4;   // 0x0-0xF
    uint16_t node_id : 7;         // 1-127
    uint16_t reserved : 5;
} CANopen_COB_ID;

// CANopen message types
enum CANopen_Message {
    CANOPEN_NMT = 0x000,          // Network management
    CANOPEN_SYNC = 0x080,         // Synchronization
    CANOPEN_EMCY = 0x080,         // Emergency
    CANOPEN_TPDO1 = 0x180,        // Transmit PDO 1
    CANOPEN_RPDO1 = 0x200,        // Receive PDO 1
    CANOPEN_TPDO2 = 0x280,        // Transmit PDO 2
    CANOPEN_RPDO2 = 0x300,        // Receive PDO 2
    CANOPEN_TPDO3 = 0x380,        // Transmit PDO 3
    CANOPEN_RPDO3 = 0x400,        // Receive PDO 3
    CANOPEN_TPDO4 = 0x480,        // Transmit PDO 4
    CANOPEN_RPDO4 = 0x500,        // Receive PDO 4
    CANOPEN_SDO_TX = 0x580,       // SDO transmit
    CANOPEN_SDO_RX = 0x600        // SDO receive
};

// NMT state machine
enum NMT_State {
    CANOPEN_INITIALIZING = 0,
    CANOPEN_STOPPED = 4,
    CANOPEN_OPERATIONAL = 5,
    CANOPEN_PRE_OPERATIONAL = 127
};

class CANopenNode {
private:
    uint8_t node_id;
    NMT_State current_state = CANOPEN_INITIALIZING;
    
    // Object Dictionary (OD) entries
    struct {
        uint32_t vendor_id;          // Index 0x1000
        uint32_t product_code;       // Index 0x1001
        uint32_t revision_number;    // Index 0x1002
        uint32_t serial_number;      // Index 0x1003
        uint8_t device_type;         // Index 0x1000
    } object_dictionary;
    
public:
    CANopenNode(uint8_t id) : node_id(id) {}
    
    void process_nmt_command(uint8_t cs, uint8_t id) {
        // cs = command specifier (0-127)
        // id = node id (0 = all nodes)
        
        if (id != 0 && id != node_id) {
            return;  // Not for us
        }
        
        switch (cs) {
            case 0x01:  // Start
                current_state = CANOPEN_OPERATIONAL;
                break;
            case 0x02:  // Stop
                current_state = CANOPEN_STOPPED;
                break;
            case 0x80:  // Pre-operational
                current_state = CANOPEN_PRE_OPERATIONAL;
                break;
            case 0x81:  // Reset application
                reset_application();
                current_state = CANOPEN_INITIALIZING;
                break;
            case 0x82:  // Reset communication
                reset_communication();
                current_state = CANOPEN_INITIALIZING;
                break;
        }
    }
    
    // Process received PDO
    void process_pdo_1(const uint8_t* data) {
        // PDO 1 contains: Speed, Load
        uint16_t speed = (data[1] << 8) | data[0];
        uint8_t load = data[2];
        
        update_speed_load(speed, load);
    }
    
    // Send EMCY (emergency) message
    void send_emergency(uint16_t error_code) {
        CAN_Frame frame;
        frame.id = CANOPEN_EMCY + node_id;
        frame.dlc = 8;
        
        frame.data[0] = error_code & 0xFF;
        frame.data[1] = (error_code >> 8) & 0xFF;
        frame.data[2] = 0;  // Error register
        frame.data[3] = 0;  // Manufacturer specific
        
        can_controller.send_message(frame);
    }
    
    // SDO (Service Data Object) - for configuration
    void request_sdo_read(uint16_t index, uint8_t subindex) {
        CAN_Frame frame;
        frame.id = CANOPEN_SDO_RX + node_id;
        frame.dlc = 8;
        
        frame.data[0] = 0x40;  // Upload initiate (read)
        frame.data[1] = index & 0xFF;
        frame.data[2] = (index >> 8) & 0xFF;
        frame.data[3] = subindex;
        
        can_controller.send_message(frame);
    }
    
private:
    void reset_application() {
        // Reset all variables and buffers
    }
    
    void reset_communication() {
        // Reset CAN communication layer
    }
    
    void update_speed_load(uint16_t speed, uint8_t load) {
        // Application-specific processing
    }
};

// CANopen practical example: Motor drive
class MotorDrive {
private:
    CANopenNode node{1};
    uint16_t motor_speed = 0;
    uint8_t motor_torque = 0;
    
public:
    void initialize() {
        // Initialization phase
        node.process_nmt_command(0x01, 0);  // Start
    }
    
    void process_commands() {
        CAN_Frame frame;
        
        if (can_controller.receive_message(frame)) {
            if ((frame.id & 0xF0) == CANOPEN_RPDO1) {
                // Received command PDO
                node.process_pdo_1(frame.data);
            }
        }
    }
    
    void send_status() {
        // Send status via PDO
        CAN_Frame frame;
        frame.id = CANOPEN_TPDO1 + node.node_id;
        frame.dlc = 4;
        
        frame.data[0] = motor_speed & 0xFF;
        frame.data[1] = (motor_speed >> 8) & 0xFF;
        frame.data[2] = motor_torque;
        frame.data[3] = 0;  // Status
        
        can_controller.send_message(frame);
    }
};
```

---

*[Due to length constraints, I'll create this as a downloadable file with all 100 questions]*

---

