# C/C++ Automotive Embedded Systems Interview Guide - Part 2
## Questions 19-100 with Code Examples

---

## Interrupt Handling & ISR

### Q19: Explain ISR (Interrupt Service Routine) design rules in automotive systems.

**Answer:**
ISRs should be fast, non-blocking, and handle only high-priority work.

```cpp
// BAD ISR - violates automotive ISR design
void bad_can_isr() {
    CAN_Frame frame;
    can_controller.read_message(&frame);
    
    // WRONG: Lots of computation
    if (frame.id == 0x123) {
        // Complex processing - could take 50ms!
        for (int i = 0; i < 10000; i++) {
            complex_calculation();
        }
    }
    
    // WRONG: Calling blocking functions
    delay_ms(10);
    
    // WRONG: Dynamic memory allocation
    uint8_t* buffer = new uint8_t[256];
}

// GOOD ISR - follows best practices
volatile uint8_t can_message_ready = 0;
volatile CAN_Frame can_message_buffer;

void good_can_isr() {
    // Rule 1: Minimize time - read message and exit
    // Rule 2: No dynamic allocation
    // Rule 3: No delays or blocking calls
    // Rule 4: Use volatile for global flags
    // Rule 5: Keep ISR deterministic
    
    if (can_controller.is_message_available()) {
        can_controller.read_message_into(&can_message_buffer);
        can_message_ready = 1;  // Signal main task
    }
}

// ISR design guidelines
class ISRGuidelines {
    /*
    1. Keep ISR duration < 10% of task period
       - CAN ISR: typically 5-10 microseconds max
       - If longer, move work to background task
    
    2. Use ISRs for:
       - Reading/writing hardware registers
       - Setting flags/semaphores
       - Toggling time-critical outputs
       - Updating small ring buffers
    
    3. Never do in ISRs:
       - printf or logging
       - malloc/new
       - mutex locks (can deadlock with lower priority)
       - Floating point math (if FPU context not saved)
       - Large loops or recursion
    
    4. Nesting:
       - Avoid if possible
       - If necessary, have clear priority scheme
       - Highest priority ISR interrupts all others
    
    5. ISR Priority vs Task Priority:
       - ISR can interrupt any task
       - If ISR priority < task priority, use deferred processing
    */
};

// Example: Fast ISR with deferred processing
class IRQHandler {
private:
    static constexpr uint8_t ISR_BUFFER_SIZE = 64;
    
    struct Message {
        uint16_t source;
        uint8_t data[8];
    };
    
    volatile Message isr_buffer[ISR_BUFFER_SIZE];
    volatile uint8_t isr_write_idx = 0;
    volatile uint8_t task_read_idx = 0;
    
public:
    // Fast ISR - completes in <1 microsecond
    void fast_isr() {
        uint8_t next_write = (isr_write_idx + 1) % ISR_BUFFER_SIZE;
        
        if (next_write != task_read_idx) {
            // Quick read and store
            isr_buffer[isr_write_idx].source = get_irq_source();
            read_irq_data(isr_buffer[isr_write_idx].data);
            isr_write_idx = next_write;
        }
    }
    
    // Slow background task - processes messages
    void process_messages_task() {
        while (task_read_idx != isr_write_idx) {
            Message& msg = (Message&)isr_buffer[task_read_idx];
            
            // Heavy processing here
            process_message(msg);
            
            task_read_idx = (task_read_idx + 1) % ISR_BUFFER_SIZE;
        }
    }
};

// ISR measurement
class ISRProfiler {
private:
    uint32_t isr_entry_time;
    uint32_t isr_max_duration = 0;
    uint32_t isr_total_duration = 0;
    uint32_t isr_call_count = 0;
    
public:
    void on_isr_entry() {
        isr_entry_time = get_cycle_counter();
    }
    
    void on_isr_exit() {
        uint32_t duration = get_cycle_counter() - isr_entry_time;
        
        if (duration > isr_max_duration) {
            isr_max_duration = duration;
        }
        
        isr_total_duration += duration;
        isr_call_count++;
    }
    
    void print_stats() {
        uint32_t avg_duration = isr_total_duration / isr_call_count;
        uint32_t duration_ms = isr_max_duration / (CLOCK_MHZ * 1000);
        
        printf("ISR max duration: %lu microseconds\n", isr_max_duration / CLOCK_MHZ);
        printf("ISR avg duration: %lu microseconds\n", avg_duration / CLOCK_MHZ);
        printf("Should be < 1%% of task period\n");
    }
};
```

---

### Q20: Explain multiple interrupt levels and priority schemes.

**Answer:**

```cpp
// Interrupt priority levels (typical ARM Cortex-M: 0-255, 0=highest)
enum IRQ_Priority {
    CRITICAL_IRQ = 0,           // Highest priority (NMI, faults)
    HARD_REAL_TIME_IRQ = 16,    // CAN, timing-critical sensors
    NORMAL_IRQ = 32,             // General purpose interrupts
    SOFT_REAL_TIME_IRQ = 64,    // Less critical
    LOWEST_PRIORITY_IRQ = 240   // Lowest priority
};

// Vector table (first 16 entries are exceptions, rest are IRQs)
typedef struct {
    uint32_t initial_sp;                    // Initial stack pointer
    void (*reset_handler)();                // Reset
    void (*nmi_handler)();                  // Non-Maskable Interrupt
    void (*fault_handler)();                // Hard Fault
    void (*mem_fault_handler)();            // Memory Management Fault
    // ... more fault handlers ...
    void (*can0_isr)();                     // CAN 0
    void (*can1_isr)();                     // CAN 1
    void (*uart0_isr)();                    // UART 0
    void (*gpio_isr)();                     // GPIO
} VectorTable;

// Nested vectored interrupt controller (NVIC) abstraction
class InterruptController {
private:
    volatile uint32_t* nvic_enable_reg = (uint32_t*)0xE000E100;
    volatile uint32_t* nvic_priority_reg = (uint32_t*)0xE000E400;
    volatile uint32_t* nvic_pending_reg = (uint32_t*)0xE000E200;
    
public:
    void enable_interrupt(uint8_t irq_number, uint8_t priority) {
        // Enable interrupt
        nvic_enable_reg[irq_number / 32] |= (1 << (irq_number % 32));
        
        // Set priority (upper 4 bits used, lower 4 unused)
        uint32_t priority_reg_idx = irq_number / 4;
        uint32_t priority_shift = (irq_number % 4) * 8;
        nvic_priority_reg[priority_reg_idx] |= (priority << priority_shift);
    }
    
    void disable_interrupt(uint8_t irq_number) {
        nvic_enable_reg[irq_number / 32] &= ~(1 << (irq_number % 32));
    }
    
    void set_priority(uint8_t irq_number, uint8_t priority) {
        uint32_t priority_reg_idx = irq_number / 4;
        uint32_t priority_shift = (irq_number % 4) * 8;
        uint32_t mask = 0xFF << priority_shift;
        
        nvic_priority_reg[priority_reg_idx] &= ~mask;
        nvic_priority_reg[priority_reg_idx] |= (priority << priority_shift);
    }
    
    uint8_t get_priority(uint8_t irq_number) {
        uint32_t priority_reg_idx = irq_number / 4;
        uint32_t priority_shift = (irq_number % 4) * 8;
        return (nvic_priority_reg[priority_reg_idx] >> priority_shift) & 0xFF;
    }
    
    bool is_pending(uint8_t irq_number) {
        return (nvic_pending_reg[irq_number / 32] & 
                (1 << (irq_number % 32))) != 0;
    }
};

// Interrupt nesting example
InterruptController irq_ctrl;

void setup_interrupts() {
    // Highest priority: CAN
    irq_ctrl.enable_interrupt(IRQ_CAN0, HARD_REAL_TIME_IRQ);
    irq_ctrl.enable_interrupt(IRQ_CAN1, HARD_REAL_TIME_IRQ);
    
    // Higher priority: ADC (sensor reading)
    irq_ctrl.enable_interrupt(IRQ_ADC, HARD_REAL_TIME_IRQ + 8);
    
    // Normal priority: UART
    irq_ctrl.enable_interrupt(IRQ_UART, NORMAL_IRQ);
    
    // Lowest priority: Timer (non-critical)
    irq_ctrl.enable_interrupt(IRQ_TIMER, LOWEST_PRIORITY_IRQ);
}

// CAN ISR (highest priority - interrupts all others)
void can_isr() {
    // Can be interrupted by fault handlers only
    read_can_and_queue_message();
}

// ADC ISR (high priority - interrupts UART/Timer only)
void adc_isr() {
    // Can be interrupted by CAN ISR
    read_adc_and_store();
}

// UART ISR (normal priority)
void uart_isr() {
    // Can be interrupted by CAN/ADC ISRs
    read_uart_byte();
}

// Timer ISR (lowest priority - interrupts nobody important)
void timer_isr() {
    // Can be interrupted by anything
    update_millisecond_counter();
}

// Interrupt latency considerations
class InterruptLatencyAnalysis {
    /*
    Interrupt Latency = ISR Response Time + ISR Execution Time + Return Time
    
    ISR Response Time:
    - Time from interrupt assertion to ISR start
    - Fixed: ~12-24 cycles for ARM Cortex-M
    - Can be blocked by higher priority ISR/critical section
    
    ISR Execution Time:
    - Time to run ISR body
    - Must be minimized
    - Preempted by higher priority ISRs
    
    Return Time:
    - Time to restore context and resume task
    - ~4-6 cycles
    
    Example:
    - CAN interrupt asserted at time T0
    - Currently in critical section (interrupts disabled)
    - Critical section ends at T1 (500 cycle delay)
    - ISR starts at T1 + 12 cycles
    - Total latency: 512 cycles
    
    Solution: Keep critical sections < 100 cycles!
    */
};

// Critical section management
class CriticalSection {
private:
    uint32_t previous_primask;
    
public:
    CriticalSection() {
        // Disable all interrupts
        asm volatile("mrs %0, primask" : "=r" (previous_primask));
        asm volatile("cpsid i");  // Disable interrupts
    }
    
    ~CriticalSection() {
        // Restore interrupt state
        asm volatile("msr primask, %0" :: "r" (previous_primask));
    }
};

void safe_shared_variable_access() {
    volatile uint16_t counter = 0;
    
    void update_counter() {
        CriticalSection critical;  // Disables interrupts
        counter++;                  // Protected access
    }                               // Re-enables interrupts here
}

// Atomic operations (better than critical sections)
class AtomicOperations {
public:
    // Atomic increment
    static uint32_t atomic_increment(volatile uint32_t* value) {
        uint32_t result;
        asm volatile(
            "1: ldrex %0, [%1]\n"
            "   add %0, %0, #1\n"
            "   strex r12, %0, [%1]\n"
            "   cmp r12, #0\n"
            "   bne 1b"
            : "=&r" (result)
            : "r" (value)
            : "r12"
        );
        return result;
    }
    
    // Compare and swap
    static bool atomic_cas(volatile uint32_t* value, 
                          uint32_t expected, 
                          uint32_t new_value) {
        uint32_t result;
        asm volatile(
            "1: ldrex %0, [%1]\n"
            "   cmp %0, %2\n"
            "   bne 2f\n"
            "   strex r12, %3, [%1]\n"
            "   cmp r12, #0\n"
            "   bne 1b\n"
            "2:"
            : "=&r" (result)
            : "r" (value), "r" (expected), "r" (new_value)
            : "r12"
        );
        return result == expected;
    }
};
```

---

## Hardware Abstraction Layer

### Q21: Design a Hardware Abstraction Layer (HAL) for GPIO.

**Answer:**

```cpp
// HAL design principle: Isolate hardware from application
// Allows easy porting between different microcontrollers

// Platform-independent interface
enum GPIO_Port { PORT_A, PORT_B, PORT_C, PORT_D };
enum GPIO_Pin { PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7 };
enum GPIO_Mode { GPIO_INPUT, GPIO_OUTPUT, GPIO_ALTERNATE };
enum GPIO_Speed { GPIO_SLOW, GPIO_MEDIUM, GPIO_FAST };
enum GPIO_Pull { GPIO_NO_PULL, GPIO_PULL_UP, GPIO_PULL_DOWN };

// Abstract base class
class GPIO_Interface {
public:
    virtual void configure(GPIO_Port port, GPIO_Pin pin, GPIO_Mode mode,
                          GPIO_Speed speed, GPIO_Pull pull) = 0;
    virtual void set_output(GPIO_Port port, GPIO_Pin pin, uint8_t value) = 0;
    virtual uint8_t read_input(GPIO_Port port, GPIO_Pin pin) = 0;
    virtual void set_alternate_function(GPIO_Port port, GPIO_Pin pin, 
                                       uint8_t af_number) = 0;
    virtual ~GPIO_Interface() = default;
};

// STM32 implementation
class STM32_GPIO : public GPIO_Interface {
private:
    struct GPIO_Registers {
        uint32_t MODER;      // Mode register
        uint32_t OTYPER;     // Output type register
        uint32_t OSPEEDR;    // Output speed register
        uint32_t PUPDR;      // Pull-up/down register
        uint32_t IDR;        // Input data register
        uint32_t ODR;        // Output data register
        uint32_t BSRR;       // Bit set/reset register
        uint32_t LCKR;       // Lock register
        uint32_t AFRL;       // Alternate function low
        uint32_t AFRH;       // Alternate function high
    };
    
    volatile GPIO_Registers* ports[4];  // GPIOA-D
    volatile uint32_t* rcc_register;
    
public:
    STM32_GPIO() {
        ports[PORT_A] = (volatile GPIO_Registers*)0x40020000;
        ports[PORT_B] = (volatile GPIO_Registers*)0x40020400;
        ports[PORT_C] = (volatile GPIO_Registers*)0x40020800;
        ports[PORT_D] = (volatile GPIO_Registers*)0x40020C00;
        
        rcc_register = (volatile uint32_t*)0x40023800;
    }
    
    void configure(GPIO_Port port, GPIO_Pin pin, GPIO_Mode mode,
                  GPIO_Speed speed, GPIO_Pull pull) override {
        // Enable clock
        *rcc_register |= (1 << port);
        
        volatile GPIO_Registers* gpio = ports[port];
        
        // Set mode (2 bits per pin)
        uint32_t mode_bits = (mode & 0x03) << (pin * 2);
        gpio->MODER &= ~(0x03 << (pin * 2));
        gpio->MODER |= mode_bits;
        
        // Set speed (2 bits per pin)
        uint32_t speed_bits = (speed & 0x03) << (pin * 2);
        gpio->OSPEEDR &= ~(0x03 << (pin * 2));
        gpio->OSPEEDR |= speed_bits;
        
        // Set pull (2 bits per pin)
        uint32_t pull_bits = (pull & 0x03) << (pin * 2);
        gpio->PUPDR &= ~(0x03 << (pin * 2));
        gpio->PUPDR |= pull_bits;
    }
    
    void set_output(GPIO_Port port, GPIO_Pin pin, uint8_t value) override {
        volatile GPIO_Registers* gpio = ports[port];
        
        if (value) {
            // Set bit using BSRR (atomic)
            gpio->BSRR = (1 << pin);
        } else {
            // Clear bit using BSRR (atomic)
            gpio->BSRR = (1 << (pin + 16));
        }
    }
    
    uint8_t read_input(GPIO_Port port, GPIO_Pin pin) override {
        volatile GPIO_Registers* gpio = ports[port];
        return (gpio->IDR >> pin) & 1;
    }
    
    void set_alternate_function(GPIO_Port port, GPIO_Pin pin, 
                               uint8_t af_number) override {
        volatile GPIO_Registers* gpio = ports[port];
        
        if (pin < 8) {
            gpio->AFRL &= ~(0x0F << (pin * 4));
            gpio->AFRL |= (af_number << (pin * 4));
        } else {
            gpio->AFRH &= ~(0x0F << ((pin - 8) * 4));
            gpio->AFRH |= (af_number << ((pin - 8) * 4));
        }
    }
};

// NXP LPC implementation
class NXP_LPC_GPIO : public GPIO_Interface {
private:
    volatile uint32_t* port_registers;
    
public:
    NXP_LPC_GPIO() {
        port_registers = (volatile uint32_t*)0x50100000;
    }
    
    void configure(GPIO_Port port, GPIO_Pin pin, GPIO_Mode mode,
                  GPIO_Speed speed, GPIO_Pull pull) override {
        // NXP implementation differs significantly
        // But interface remains same!
    }
    
    void set_output(GPIO_Port port, GPIO_Pin pin, uint8_t value) override {
        // NXP-specific implementation
    }
    
    // ... other methods ...
};

// Application code - independent of hardware
class LightController {
private:
    GPIO_Interface* gpio;
    GPIO_Port led_port;
    GPIO_Pin led_pin;
    
public:
    LightController(GPIO_Interface* gpio_impl, GPIO_Port port, GPIO_Pin pin)
        : gpio(gpio_impl), led_port(port), led_pin(pin) {}
    
    void initialize() {
        gpio->configure(led_port, led_pin, GPIO_OUTPUT, 
                       GPIO_FAST, GPIO_NO_PULL);
    }
    
    void turn_on() {
        gpio->set_output(led_port, led_pin, 1);
    }
    
    void turn_off() {
        gpio->set_output(led_port, led_pin, 0);
    }
    
    void toggle() {
        uint8_t current = gpio->read_input(led_port, led_pin);
        gpio->set_output(led_port, led_pin, 1 - current);
    }
};

// Usage - application doesn't care which MCU!
int main() {
    STM32_GPIO stm32_gpio;  // Can swap for NXP_LPC_GPIO
    
    LightController engine_light(&stm32_gpio, PORT_A, PIN_5);
    engine_light.initialize();
    engine_light.turn_on();
    
    // Works same on any platform!
}
```

---

### Q22: Design a HAL for CAN communication.

**Answer:**

```cpp
// Abstract CAN interface
class CAN_Interface {
public:
    enum BaudRate { CAN_500K = 500000, CAN_1M = 1000000 };
    
    virtual void initialize(BaudRate baudrate) = 0;
    virtual bool transmit(const CAN_Frame& frame) = 0;
    virtual bool receive(CAN_Frame& frame) = 0;
    virtual void enable_interrupt(uint16_t can_id) = 0;
    virtual ~CAN_Interface() = default;
};

// STM32 CAN implementation
class STM32_CAN : public CAN_Interface {
private:
    volatile uint32_t* can_base = (uint32_t*)0x40006C00;
    
    // CAN register offsets
    enum CAN_Registers {
        CAN_MCR = 0x00,      // Master Control
        CAN_MSR = 0x04,      // Master Status
        CAN_TSR = 0x08,      // Transmit Status
        CAN_RF0R = 0x0C,     // Receive FIFO 0
        CAN_RF1R = 0x10,     // Receive FIFO 1
        CAN_IER = 0x14,      // Interrupt Enable
        CAN_ESR = 0x18,      // Error Status
        CAN_BTR = 0x1C       // Bit Timing
    };
    
public:
    void initialize(BaudRate baudrate) override {
        uint32_t* mcr = (uint32_t*)(can_base + CAN_MCR);
        uint32_t* btr = (uint32_t*)(can_base + CAN_BTR);
        
        // Enter initialization mode
        *mcr |= 0x00000001;
        
        // Wait for ready
        while (!(*mcr & 0x00000001)) {}
        
        // Configure bitrate
        // For STM32 @ 72MHz, CAN 500kHz:
        // BTR = 0x00230000 (SJW=1, TS2=3, TS1=8, BRP=4)
        *btr = 0x00230000;
        
        // Exit initialization mode
        *mcr &= ~0x00000001;
    }
    
    bool transmit(const CAN_Frame& frame) override {
        uint32_t* tsr = (uint32_t*)(can_base + CAN_TSR);
        
        // Check if TX buffer available
        if (*tsr & 0x04) {
            return false;  // Busy
        }
        
        // Write to TX buffer (simplified)
        uint32_t* tx_id = (uint32_t*)(can_base + 0x180);
        tx_id[0] = frame.id << 21;
        tx_id[1] = frame.dlc;
        
        for (int i = 0; i < frame.dlc; i++) {
            tx_id[2 + i/4] |= (frame.data[i] << ((i % 4) * 8));
        }
        
        // Request transmission
        tx_id[1] |= 0x01;
        
        return true;
    }
    
    bool receive(CAN_Frame& frame) override {
        uint32_t* rf0r = (uint32_t*)(can_base + CAN_RF0R);
        
        if (!(*rf0r & 0x00000003)) {
            return false;  // No message
        }
        
        // Read from RX FIFO (simplified)
        uint32_t* rx_fifo = (uint32_t*)(can_base + 0x1A0);
        frame.id = (rx_fifo[0] >> 21) & 0x7FF;
        frame.dlc = rx_fifo[1] & 0x0F;
        
        for (int i = 0; i < frame.dlc; i++) {
            frame.data[i] = (rx_fifo[2 + i/4] >> ((i % 4) * 8)) & 0xFF;
        }
        
        // Release buffer
        *rf0r |= 0x00000020;
        
        return true;
    }
    
    void enable_interrupt(uint16_t can_id) override {
        // Enable RX FIFO interrupt
        uint32_t* ier = (uint32_t*)(can_base + CAN_IER);
        *ier |= 0x00000002;  // FIFO 0 message pending
    }
};

// NXP LPC CAN implementation
class NXP_LPC_CAN : public CAN_Interface {
    // Different registers, same interface
public:
    void initialize(BaudRate baudrate) override {
        // NXP-specific initialization
    }
    
    bool transmit(const CAN_Frame& frame) override {
        // NXP-specific transmission
        return true;
    }
    
    bool receive(CAN_Frame& frame) override {
        // NXP-specific reception
        return false;
    }
    
    void enable_interrupt(uint16_t can_id) override {
        // NXP-specific interrupt setup
    }
};

// Application layer
class MessageProcessor {
private:
    CAN_Interface* can_bus;
    
public:
    MessageProcessor(CAN_Interface* bus) : can_bus(bus) {}
    
    void initialize() {
        can_bus->initialize(CAN_Interface::CAN_500K);
        can_bus->enable_interrupt(0x100);  // Engine data
    }
    
    void process_incoming_messages() {
        CAN_Frame frame;
        
        while (can_bus->receive(frame)) {
            if (frame.id == 0x100) {
                handle_engine_data(frame);
            } else if (frame.id == 0x200) {
                handle_transmission_data(frame);
            }
        }
    }
    
private:
    void handle_engine_data(const CAN_Frame& frame) {
        // Process engine data
    }
    
    void handle_transmission_data(const CAN_Frame& frame) {
        // Process transmission data
    }
};

// Usage
int main() {
    // Option 1: Use STM32
    STM32_CAN stm32_can;
    MessageProcessor processor(&stm32_can);
    
    // Option 2: Use NXP (just change this line!)
    // NXP_LPC_CAN nxp_can;
    // MessageProcessor processor(&nxp_can);
    
    processor.initialize();
    processor.process_incoming_messages();
}
```

---

## MISRA-C Compliance & Safety

### Q23: Explain key MISRA-C rules and how to implement them.

**Answer:**

```cpp
// MISRA-C: Set of coding guidelines for safety-critical systems

// Rule 1: Avoid dynamic memory allocation
// WRONG
uint8_t* sensor_buffer = malloc(1024);

// CORRECT
static uint8_t sensor_buffer[1024];

// Rule 2: No function pointers (or if used, well controlled)
// RISKY
typedef void (*callback_t)(void);
callback_t callbacks[10];  // Hard to trace

// BETTER: Use switch/if or registered handlers with validation
struct RegisteredHandler {
    uint8_t id;
    void (*handler)(void);
};

// Rule 3: Limit nesting depth to 3-4 levels
// WRONG - 6 levels of nesting
if (a) {
    if (b) {
        if (c) {
            if (d) {
                if (e) {
                    if (f) {
                        do_work();  // Too nested!
                    }
                }
            }
        }
    }
}

// CORRECT - Flatten using early returns
bool validate_conditions() {
    if (!a) return false;
    if (!b) return false;
    if (!c) return false;
    if (!d) return false;
    if (!e) return false;
    if (!f) return false;
    return true;
}

if (validate_conditions()) {
    do_work();
}

// Rule 4: Use unsigned types for bit operations
// WRONG - signed type
int flags = 0;
flags |= (1 << 7);  // Undefined behavior!

// CORRECT - unsigned type
uint8_t flags = 0;
flags |= (1 << 7);  // Well-defined

// Rule 5: No implicit type conversions
// WRONG - implicit conversion
uint16_t value = 65535;
uint8_t small_value = value;  // Silently truncates!

// CORRECT - explicit cast
uint8_t small_value = (uint8_t)(value & 0xFF);

// Rule 6: Always use braces for if/while
// WRONG
if (error)
    return;
    do_work();  // Looks indented but not in if!

// CORRECT
if (error) {
    return;
}
do_work();

// Rule 7: Prototype all functions
// WRONG
int calculate_value() {
    return 42;
}
int x = calculate_value();  // No prototype!

// CORRECT
int calculate_value(void);  // Forward declaration
int x = calculate_value();

// Rule 8: No recursion (especially in safety-critical code)
// WRONG
int factorial(int n) {
    if (n <= 1) return 1;
    return n * factorial(n - 1);  // Recursive!
}

// CORRECT
int factorial_iterative(int n) {
    int result = 1;
    for (int i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
}

// Rule 9: No goto (use structured control flow)
// WRONG
if (error1) {
    goto cleanup;
}
if (error2) {
    goto cleanup;
}
cleanup:
    release_resources();

// CORRECT - use structured patterns
void handle_operation() {
    if (error1 || error2) {
        release_resources();
        return;
    }
}

// Rule 10: Bounded arrays, no unbounded access
// WRONG
void process_data(uint8_t* data, uint16_t length) {
    for (int i = 0; i < 1000; i++) {  // What if length < 1000?
        process(data[i]);
    }
}

// CORRECT
void process_data(uint8_t* data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        if (i >= MAX_SAFE_LENGTH) break;
        process(data[i]);
    }
}

// Rule 11: No hidden dependencies (static variables)
// WRONG
int increment_counter(void) {
    static int counter = 0;  // Hidden state!
    return ++counter;
}

// CORRECT
class Counter {
private:
    int counter = 0;
public:
    int increment() { return ++counter; }
};

// Rule 12: Defensive coding - check all inputs
// WRONG
uint16_t divide(uint16_t a, uint16_t b) {
    return a / b;  // What if b == 0?
}

// CORRECT
uint16_t divide(uint16_t a, uint16_t b) {
    if (b == 0) {
        return 0;  // Or return error code
    }
    return a / b;
}

// MISRA-C Compliance Checker Example
class MISRACCompliance {
public:
    struct ViolationReport {
        uint32_t line_number;
        const char* rule;
        const char* description;
    };
    
    static ViolationReport check_memory_allocation(const char* code_line) {
        // Rule 1: No malloc
        if (strstr(code_line, "malloc") || strstr(code_line, "new")) {
            return {0, "Rule 1", "Dynamic memory allocation not allowed"};
        }
        return {0, "", ""};
    }
    
    static ViolationReport check_signed_bit_ops(const char* code_line) {
        // Rule 4: Check for bit ops on signed types
        if (strstr(code_line, "int ") && strstr(code_line, "<<|>>|&|^")) {
            return {0, "Rule 4", "Bit operations on signed type"};
        }
        return {0, "", ""};
    }
};

// Functional Safety Example: Safe sensor reading
class SafeSensorReader {
private:
    static constexpr uint8_t MAX_RETRIES = 3;
    static constexpr uint16_t MIN_VALID_READING = 100;
    static constexpr uint16_t MAX_VALID_READING = 4000;
    
public:
    struct SensorReading {
        uint16_t value;
        bool is_valid;
        uint8_t status;
    };
    
    SensorReading read_sensor(uint8_t sensor_id) {
        SensorReading result = {0, false, 1};
        
        if (sensor_id >= NUM_SENSORS) {
            result.status = 2;  // Invalid sensor ID
            return result;
        }
        
        for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
            uint16_t raw_value = read_adc(sensor_id);
            
            if (raw_value >= MIN_VALID_READING && 
                raw_value <= MAX_VALID_READING) {
                result.value = raw_value;
                result.is_valid = true;
                result.status = 0;  // Success
                return result;
            }
        }
        
        result.status = 3;  // Failed after retries
        return result;
    }
};
```

---

### Q24: Implement a watchdog timer for safety monitoring.

**Answer:**

```cpp
// Watchdog Timer (WDT): Monitors system health
// If main loop doesn't "kick" watchdog, system resets

class WatchdogTimer {
private:
    volatile uint32_t* wdt_ctrl_reg = (uint32_t*)0x40080000;
    volatile uint32_t* wdt_load_reg = (uint32_t*)(0x40080000 + 0x04);
    volatile uint32_t* wdt_feed_reg = (uint32_t*)(0x40080000 + 0x08);
    volatile uint32_t* wdt_status_reg = (uint32_t*)(0x40080000 + 0x0C);
    
    uint32_t timeout_ticks;
    uint8_t feed_count = 0;
    
public:
    enum WDT_Timeout {
        WDT_1_SEC = 1000,
        WDT_5_SEC = 5000,
        WDT_10_SEC = 10000
    };
    
    void initialize(WDT_Timeout timeout_ms) {
        // Convert ms to watchdog ticks (assuming 1MHz clock)
        timeout_ticks = timeout_ms;
        
        // Load timeout value
        *wdt_load_reg = timeout_ticks;
        
        // Enable watchdog (RESEN=1, WDEN=1)
        *wdt_ctrl_reg = 0x00000003;
    }
    
    void kick() {
        // Feed watchdog (reset timer)
        // Typical sequence: write 0xAAAA then 0x5555
        *wdt_feed_reg = 0xAAAA;
        *wdt_feed_reg = 0x5555;
        feed_count++;
    }
    
    uint32_t get_remaining_time() {
        // Get current counter value
        return *wdt_status_reg & 0xFFFFFFFF;
    }
    
    bool is_timeout_imminent(uint32_t ticks_remaining = 100) {
        return get_remaining_time() < ticks_remaining;
    }
};

// Multi-level watchdog system
class HealthMonitor {
private:
    enum HealthStatus {
        HEALTHY = 0,
        DEGRADED = 1,
        CRITICAL = 2,
        FAULT = 3
    };
    
    struct TaskHealth {
        uint32_t last_execution_time;
        uint32_t max_execution_time;
        bool executed_this_cycle;
        HealthStatus status;
    };
    
    static constexpr uint8_t NUM_TASKS = 5;
    TaskHealth tasks[NUM_TASKS];
    WatchdogTimer wdt;
    
public:
    void initialize() {
        wdt.initialize(WatchdogTimer::WDT_10_SEC);
        
        for (int i = 0; i < NUM_TASKS; i++) {
            tasks[i].last_execution_time = 0;
            tasks[i].max_execution_time = 0;
            tasks[i].executed_this_cycle = false;
            tasks[i].status = HEALTHY;
        }
    }
    
    void log_task_execution(uint8_t task_id, uint32_t execution_time) {
        if (task_id >= NUM_TASKS) return;
        
        tasks[task_id].executed_this_cycle = true;
        tasks[task_id].last_execution_time = execution_time;
        
        if (execution_time > tasks[task_id].max_execution_time) {
            tasks[task_id].max_execution_time = execution_time;
        }
        
        // Check if within expected bounds
        if (execution_time > TASK_WCET[task_id]) {
            tasks[task_id].status = DEGRADED;
        }
    }
    
    void monitor_cycle() {
        bool all_healthy = true;
        uint8_t degraded_count = 0;
        uint8_t critical_count = 0;
        
        // Check all tasks executed
        for (int i = 0; i < NUM_TASKS; i++) {
            if (!tasks[i].executed_this_cycle) {
                tasks[i].status = CRITICAL;
                critical_count++;
                all_healthy = false;
            } else {
                tasks[i].executed_this_cycle = false;
                
                if (tasks[i].status == DEGRADED) {
                    degraded_count++;
                    all_healthy = false;
                }
            }
        }
        
        if (critical_count > 0) {
            // Critical failure!
            enter_safe_state();
            // DON'T kick watchdog - force reset
        } else if (degraded_count > NUM_TASKS / 2) {
            // Too many degraded tasks
            attempt_recovery();
            // May kick watchdog if recovery succeeds
        } else {
            // System OK - kick watchdog
            wdt.kick();
        }
    }
    
private:
    static constexpr uint32_t TASK_WCET[] = {5, 10, 15, 8, 20};
    
    void enter_safe_state() {
        // Stop all outputs
        disable_all_outputs();
        
        // Log error to flash
        log_fault_reason(FAULT_TASK_MISSED);
    }
    
    void attempt_recovery() {
        // Try to restart failed tasks
        for (int i = 0; i < NUM_TASKS; i++) {
            if (tasks[i].status == CRITICAL) {
                restart_task(i);
            }
        }
    }
};

// Real-time monitoring example
class RealtimeMonitor {
private:
    WatchdogTimer wdt;
    
    struct TaskDeadline {
        uint8_t task_id;
        uint32_t deadline_ms;
        uint32_t start_time;
    };
    
    TaskDeadline current_task = {0, 0, 0};
    
public:
    void task_started(uint8_t id, uint32_t deadline_ms) {
        current_task.task_id = id;
        current_task.deadline_ms = deadline_ms;
        current_task.start_time = get_system_time_ms();
        
        // Set watchdog to task deadline + margin
        wdt.initialize(deadline_ms + 5);
    }
    
    void task_completed(uint8_t id) {
        uint32_t duration = get_system_time_ms() - current_task.start_time;
        
        if (duration <= current_task.deadline_ms) {
            wdt.kick();  // Task met deadline
        } else {
            // Task exceeded deadline!
            log_deadline_miss(id, duration, current_task.deadline_ms);
            // Don't kick watchdog - trigger recovery
        }
    }
};

// Main loop example
int main() {
    HealthMonitor health;
    health.initialize();
    
    while (1) {
        // Cycle start
        uint32_t cycle_start = get_ticks();
        
        // Task 1 (engine control)
        uint32_t t1_start = get_ticks();
        engine_control_task();
        uint32_t t1_duration = get_ticks() - t1_start;
        health.log_task_execution(0, t1_duration);
        
        // Task 2 (brake control)
        uint32_t t2_start = get_ticks();
        brake_control_task();
        uint32_t t2_duration = get_ticks() - t2_start;
        health.log_task_execution(1, t2_duration);
        
        // Task 3 (transmission)
        uint32_t t3_start = get_ticks();
        transmission_control_task();
        uint32_t t3_duration = get_ticks() - t3_start;
        health.log_task_execution(2, t3_duration);
        
        // Monitor system health
        health.monitor_cycle();
        
        // Sleep until next cycle
        uint32_t cycle_time = get_ticks() - cycle_start;
        if (cycle_time < CYCLE_PERIOD) {
            sleep_ms(CYCLE_PERIOD - cycle_time);
        }
    }
}
```

---

## Performance Optimization

### Q25: Explain compiler optimization levels and impact on embedded systems.

**Answer:**

```cpp
// Compiler optimization levels: -O0, -O1, -O2, -O3, -Os

// -O0: No optimization (default during development)
// - Slowest code
// - Largest binary
// - Easiest to debug
// - Variable changes appear immediately
int calculate_value(int x) {
    int temp1 = x * 2;
    int temp2 = temp1 + 10;
    int temp3 = temp2 * 3;
    return temp3;  // Compiler won't combine operations
}

// -O1: Minimal optimization
// - Better performance
// - Still reasonably debuggable
// - ~20% code size reduction
// - Local optimizations only

// -O2: Moderate optimization (recommended for embedded)
// - ~50% faster than -O0
// - ~30% smaller than -O0
// - Loop unrolling, inlining
// - Still mostly debuggable
// - Used in most automotive ECUs

// -O3: Aggressive optimization
// - ~60% faster than -O0
// - Larger binary (aggressive inlining)
// - Full loop unrolling
// - Interprocedural optimization
// - Hard to debug
// - Not recommended for safety-critical code

// -Os: Optimize for size
// - Smallest binary
// - Sometimes slower than -O2
// - Used when flash space is critical
// - Good for bootloaders, limited flash devices

// Optimization example
class OptimizationDemo {
public:
    // Example 1: Loop optimization
    
    // Compiled with -O0: ~1000 instructions
    void sum_array_unoptimized(uint16_t* data, uint16_t len) {
        uint32_t sum = 0;
        for (int i = 0; i < len; i++) {
            sum += data[i];  // Each iteration: load, add, increment, compare
        }
        write_result(sum);
    }
    
    // Compiled with -O2: ~50 instructions
    // - Loop unrolled (4x)
    // - Strength reduction (index arithmetic simplified)
    // - Dead code elimination
    uint32_t sum_array_optimized(uint16_t* data, uint16_t len) {
        uint32_t sum = 0;
        
        // Auto-unrolled by compiler (4x)
        for (int i = 0; i < len; i += 4) {
            sum += data[i];
            sum += data[i + 1];
            sum += data[i + 2];
            sum += data[i + 3];
        }
        
        // Handle remainder
        for (int i = len - (len % 4); i < len; i++) {
            sum += data[i];
        }
        
        return sum;
    }
};

// Critical: Inline optimization
class InlineExample {
private:
    uint16_t engine_rpm = 0;
    
public:
    // Without inline: function call overhead
    // ~10-20 cycles per call + return
    uint8_t calculate_gear_unoptimized(uint16_t rpm) {
        if (rpm < 1500) return 1;
        if (rpm < 3000) return 2;
        if (rpm < 4500) return 3;
        return 4;
    }
    
    // With inline: code inserted directly
    // No function call overhead
    inline uint8_t calculate_gear_optimized(uint16_t rpm) {
        if (rpm < 1500) return 1;
        if (rpm < 3000) return 2;
        if (rpm < 4500) return 3;
        return 4;
    }
    
    void update_transmission() {
        // Called 1000x per second - overhead matters!
        for (int i = 0; i < 1000; i++) {
            uint8_t gear = calculate_gear_optimized(engine_rpm);
        }
    }
};

// Volatile vs optimization
class VolatileOptimization {
public:
    // Without volatile: compiler may optimize away!
    uint32_t check_status_unoptimized() {
        uint32_t* status_reg = (uint32_t*)0x40000000;
        uint32_t status = *status_reg;
        
        // Compiler might not read again if variable unchanged
        while (status == 0) {
            // Infinite loop! Compiler optimizes to: while (true)
        }
        
        return status;
    }
    
    // With volatile: must read every time
    uint32_t check_status_optimized() {
        volatile uint32_t* status_reg = (volatile uint32_t*)0x40000000;
        uint32_t status = *status_reg;
        
        while (status == 0) {
            status = *status_reg;  // Forced to read every iteration
        }
        
        return status;
    }
};

// Function attributes for optimization control
// Force inline for small critical functions
#define FORCE_INLINE __attribute__((always_inline)) inline

class AttributeExample {
    // Always inline - no function call overhead
    FORCE_INLINE uint16_t read_sensor() {
        return read_adc();
    }
    
    // Never inline - save code space
    __attribute__((noinline)) void log_diagnostic_data() {
        printf("Diagnostic info...\n");  // Large function
    }
    
    // Optimize for performance regardless of -O level
    __attribute__((optimize("O3"))) uint32_t fast_calculation() {
        // Heavy computation
        return perform_complex_calc();
    }
    
    // Optimize for size
    __attribute__((optimize("Os"))) void initialize_hardware() {
        // Called once at startup - size matters more
    }
};

// Pragma directives
class PragmaOptimization {
public:
    void performance_critical_function() {
        #pragma GCC optimize("O3")  // Force O3 for this function
        #pragma GCC unroll 8        // Unroll loops 8x
        
        // Heavy computation
        for (int i = 0; i < 1000; i++) {
            process_data(i);
        }
        
        #pragma GCC reset_options    // Restore original optimization
    }
};

// Profile-guided optimization (PGO)
// 1. Compile with -fprofile-generate
// 2. Run with representative workload
// 3. Compile with -fprofile-use and -fprofile-correction

// Profiling example
class PerformanceProfiler {
private:
    uint32_t function_cycles[100];
    uint32_t cycle_count = 0;
    
public:
    void profile_function(void (*func)(), const char* name) {
        uint32_t start = get_cycle_counter();
        func();
        uint32_t end = get_cycle_counter();
        
        uint32_t cycles = end - start;
        if (cycle_count < 100) {
            function_cycles[cycle_count++] = cycles;
        }
        
        uint32_t avg_cycles = get_average_cycles();
        uint32_t time_ms = cycles / (CLOCK_MHZ * 1000);
        
        printf("%s: %lu cycles (%lu ms)\n", name, cycles, time_ms);
    }
    
private:
    uint32_t get_average_cycles() {
        uint32_t sum = 0;
        for (int i = 0; i < cycle_count; i++) {
            sum += function_cycles[i];
        }
        return sum / cycle_count;
    }
};
```

---

### Q26: Explain cache effects and optimization for embedded systems without cache.

**Answer:**

```cpp
// Embedded systems often have no cache, but understanding cache helps
// for understanding memory access patterns

// Cache levels (if present):
// L1 Cache: 4-8 KB, 4 cycles latency (very fast)
// L2 Cache: 32-256 KB, 8-12 cycles latency
// RAM: ~50 cycles latency
// Flash: ~100-500 cycles latency

class MemoryAccessPatterns {
public:
    // Pattern 1: Sequential access (cache-friendly)
    void sequential_access(uint16_t* data, uint16_t len) {
        // Accesses: 0, 1, 2, 3, 4, ...
        // All nearby in memory - excellent locality
        uint32_t sum = 0;
        
        for (uint16_t i = 0; i < len; i++) {
            sum += data[i];  // Sequential - cache hit!
        }
    }
    
    // Pattern 2: Strided access (reduced cache efficiency)
    void strided_access(uint16_t* data, uint16_t len) {
        // Accesses: 0, 10, 20, 30, ...
        // Larger gaps - more cache misses
        uint32_t sum = 0;
        
        for (uint16_t i = 0; i < len; i += 10) {
            sum += data[i];  // Poor locality
        }
    }
    
    // Pattern 3: Random access (worst cache efficiency)
    void random_access(uint16_t* indices, uint16_t* data, uint16_t len) {
        // Accesses: data[indices[0]], data[indices[1]], etc.
        // Unpredictable - cache misses on every access
        uint32_t sum = 0;
        
        for (uint16_t i = 0; i < len; i++) {
            sum += data[indices[i]];  // Random - cache misses!
        }
    }
};

// For embedded without cache, minimize memory access altogether

// Bad: Multiple loads
void calculate_field_bad(const DataStruct& data) {
    // Each field access reads from memory
    printf("%d %d %d\n", data.field1, data.field2, data.field3);
    printf("%d %d %d\n", data.field1, data.field2, data.field3);
    // Fields loaded 6 times total
}

// Good: Load once, use multiple times
void calculate_field_good(const DataStruct& data) {
    // Load fields once
    uint16_t f1 = data.field1;
    uint16_t f2 = data.field2;
    uint16_t f3 = data.field3;
    
    printf("%d %d %d\n", f1, f2, f3);
    printf("%d %d %d\n", f1, f2, f3);
    // Fields loaded 3 times total
}

// Working set size optimization
class WorkingSetOptimization {
public:
    // Large data structure - poor efficiency
    struct VehicleState {
        uint16_t engine_rpm;
        uint8_t gear;
        int32_t position_x;
        int32_t position_y;
        int32_t position_z;
        uint16_t speed;
        uint16_t acceleration;
        uint8_t brake_pressure;
        uint8_t throttle;
        uint32_t timestamp;
        // ... more fields
        // Total: 100+ bytes
    };
    
    void process_state_unoptimized(const VehicleState& state) {
        // Accesses scattered throughout large structure
        // Poor temporal locality
        printf("RPM: %d, Gear: %d\n", state.engine_rpm, state.gear);
        // ... later ...
        printf("Speed: %d\n", state.speed);
    }
    
    void process_state_optimized(const VehicleState& state) {
        // Extract frequently accessed fields
        uint16_t rpm = state.engine_rpm;
        uint8_t gear = state.gear;
        uint16_t speed = state.speed;
        
        printf("RPM: %d, Gear: %d, Speed: %d\n", rpm, gear, speed);
        // Better locality within a smaller structure
    }
};

// Data structure alignment for efficient access
class StructureAlignment {
public:
    // Unaligned - requires extra instructions
    struct UnalignedData {
        uint8_t a;           // +0
        uint16_t b;          // +1 (unaligned!)
        uint32_t c;          // +3 (unaligned!)
    };  // Size: 7 bytes (but wastes space and performance)
    
    // Aligned - efficient access
    struct AlignedData {
        uint8_t a;           // +0
        // padding (1 byte)
        uint16_t b;          // +2 (aligned)
        uint32_t c;          // +4 (aligned)
    };  // Size: 8 bytes (1 byte wasted but much faster)
    
    // Best: Arrange by size
    struct OptimalData {
        uint32_t c;          // +0
        uint16_t b;          // +4
        uint8_t a;           // +6
        // padding (1 byte)
    };  // Size: 8 bytes, no waste, optimal access
};

// Compiler helps with packing
struct PackedData {
    uint8_t a;
    uint16_t b;
    uint32_t c;
} __attribute__((packed));  // No padding - save space

// Memory bandwidth optimization
class BandwidthOptimization {
private:
    static constexpr uint16_t ARRAY_SIZE = 10000;
    uint16_t sensor_data[ARRAY_SIZE];
    
public:
    // Unoptimized: 1 read per iteration
    uint32_t sum_unoptimized() {
        uint32_t sum = 0;
        
        for (int i = 0; i < ARRAY_SIZE; i++) {
            sum += sensor_data[i];  // 1 load per iteration
        }
        
        return sum;
    }
    
    // Optimized: 4 reads per 4 iterations (better pipelining)
    uint32_t sum_optimized() {
        uint32_t sum0 = 0, sum1 = 0, sum2 = 0, sum3 = 0;
        
        for (int i = 0; i < ARRAY_SIZE; i += 4) {
            sum0 += sensor_data[i];
            sum1 += sensor_data[i + 1];
            sum2 += sensor_data[i + 2];
            sum3 += sensor_data[i + 3];
        }
        
        return sum0 + sum1 + sum2 + sum3;
    }
    // Parallel loads mask memory latency!
};
```

---

## Debugging & Testing

### Q27: Explain common debugging techniques in embedded automotive systems.

**Answer:**

```cpp
// Debugging challenges in embedded:
// - Limited resources
// - Real-time constraints
// - Hardware dependencies
// - No traditional debugger access often

// Technique 1: Serial logging (most common)
class SerialDebugger {
private:
    static constexpr uint16_t LOG_BUFFER_SIZE = 256;
    uint8_t log_buffer[LOG_BUFFER_SIZE];
    uint16_t log_index = 0;
    volatile uint8_t uart_busy = 0;
    
public:
    void log_printf(const char* format, ...) {
        // Format-safe logging
        va_list args;
        va_start(args, format);
        
        // Vsnprintf for safety - bounded write
        int len = vsnprintf((char*)log_buffer + log_index, 
                           LOG_BUFFER_SIZE - log_index,
                           format, args);
        va_end(args);
        
        if (len > 0) {
            log_index += len;
            
            // Flush if buffer half full or newline found
            if (log_index > LOG_BUFFER_SIZE / 2 || format[0] == '\n') {
                flush_log();
            }
        }
    }
    
    void log_data(const uint8_t* data, uint16_t len) {
        // Log binary data as hex
        for (uint16_t i = 0; i < len; i++) {
            log_printf("%02X ", data[i]);
        }
        log_printf("\n");
    }
    
private:
    void flush_log() {
        while (uart_busy) {}  // Wait for UART ready
        
        uart_send_data(log_buffer, log_index);
        log_index = 0;
    }
};

// Technique 2: Assert macros
#ifdef DEBUG
    #define ASSERT(condition, message) \
        if (!(condition)) { \
            panic("ASSERT FAILED: " message " at " __FILE__ ":" __LINE__); \
        }
#else
    #define ASSERT(condition, message)  // No-op in release
#endif

void sensor_read_task(uint16_t& sensor_value) {
    uint16_t raw = read_adc();
    
    ASSERT(raw >= MIN_SENSOR_READING, "Sensor reading too low");
    ASSERT(raw <= MAX_SENSOR_READING, "Sensor reading too high");
    
    sensor_value = raw;
}

// Technique 3: Watchpoint-style monitoring
class DataWatchpoint {
private:
    struct WatchpointEntry {
        uint32_t address;
        uint32_t size;
        uint32_t last_value;
        const char* name;
    };
    
    static constexpr uint8_t MAX_WATCHPOINTS = 16;
    WatchpointEntry watchpoints[MAX_WATCHPOINTS];
    uint8_t watchpoint_count = 0;
    
public:
    void add_watchpoint(void* addr, uint32_t size, const char* name) {
        if (watchpoint_count < MAX_WATCHPOINTS) {
            watchpoints[watchpoint_count].address = (uint32_t)addr;
            watchpoints[watchpoint_count].size = size;
            watchpoints[watchpoint_count].name = name;
            
            // Read initial value
            memcpy(&watchpoints[watchpoint_count].last_value, addr, 
                   MIN(size, 4));
            
            watchpoint_count++;
        }
    }
    
    void check_watchpoints() {
        for (uint8_t i = 0; i < watchpoint_count; i++) {
            uint32_t current_value = 0;
            memcpy(&current_value, 
                   (void*)watchpoints[i].address, 
                   MIN(watchpoints[i].size, 4));
            
            if (current_value != watchpoints[i].last_value) {
                printf("Watchpoint '%s' changed: %X -> %X\n",
                       watchpoints[i].name,
                       watchpoints[i].last_value,
                       current_value);
                
                watchpoints[i].last_value = current_value;
            }
        }
    }
};

// Technique 4: Circular buffer for historical data
class DebugCircularBuffer {
private:
    struct LogEntry {
        uint32_t timestamp;
        uint8_t severity;  // 0=INFO, 1=WARN, 2=ERROR
        uint32_t data1;
        uint32_t data2;
    };
    
    static constexpr uint16_t BUFFER_SIZE = 1024;
    LogEntry buffer[BUFFER_SIZE];
    uint16_t write_index = 0;
    
public:
    void log_event(uint8_t severity, uint32_t data1, uint32_t data2) {
        buffer[write_index].timestamp = get_ticks();
        buffer[write_index].severity = severity;
        buffer[write_index].data1 = data1;
        buffer[write_index].data2 = data2;
        
        write_index = (write_index + 1) % BUFFER_SIZE;
    }
    
    void dump_history(uint16_t last_n_entries) {
        uint16_t count = MIN(last_n_entries, BUFFER_SIZE);
        uint16_t start = (write_index + BUFFER_SIZE - count) % BUFFER_SIZE;
        
        for (uint16_t i = 0; i < count; i++) {
            uint16_t idx = (start + i) % BUFFER_SIZE;
            LogEntry& entry = buffer[idx];
            
            const char* level = (entry.severity == 0) ? "INFO" :
                               (entry.severity == 1) ? "WARN" : "ERROR";
            
            printf("[%lu] %s: %X %X\n", 
                   entry.timestamp, level, entry.data1, entry.data2);
        }
    }
};

// Technique 5: CAN bus monitoring
class CANDebugger {
private:
    struct CANTrace {
        uint32_t timestamp;
        uint16_t can_id;
        uint8_t dlc;
        uint8_t data[8];
    };
    
    static constexpr uint16_t TRACE_SIZE = 512;
    CANTrace trace_buffer[TRACE_SIZE];
    uint16_t trace_index = 0;
    
public:
    void log_can_message(uint16_t can_id, const uint8_t* data, uint8_t dlc) {
        trace_buffer[trace_index].timestamp = get_ticks();
        trace_buffer[trace_index].can_id = can_id;
        trace_buffer[trace_index].dlc = dlc;
        memcpy(trace_buffer[trace_index].data, data, dlc);
        
        trace_index = (trace_index + 1) % TRACE_SIZE;
    }
    
    void dump_can_trace() {
        printf("CAN Bus Trace:\n");
        for (uint16_t i = 0; i < TRACE_SIZE; i++) {
            CANTrace& entry = trace_buffer[i];
            
            printf("[%lu] ID=0x%03X DLC=%d: ",
                   entry.timestamp, entry.can_id, entry.dlc);
            
            for (uint8_t j = 0; j < entry.dlc; j++) {
                printf("%02X ", entry.data[j]);
            }
            printf("\n");
        }
    }
};

// Technique 6: State machine debugging
enum EngineState {
    STATE_OFF = 0,
    STATE_STARTING = 1,
    STATE_RUNNING = 2,
    STATE_STALLING = 3,
    STATE_FAULT = 4
};

class StateDebugger {
private:
    struct StateTransition {
        EngineState from_state;
        EngineState to_state;
        uint32_t timestamp;
        uint32_t duration_in_previous;
    };
    
    static constexpr uint8_t MAX_TRANSITIONS = 100;
    StateTransition transitions[MAX_TRANSITIONS];
    uint8_t transition_count = 0;
    EngineState current_state = STATE_OFF;
    uint32_t state_entry_time = 0;
    
public:
    void transition_to(EngineState new_state) {
        uint32_t duration = get_ticks() - state_entry_time;
        
        if (transition_count < MAX_TRANSITIONS) {
            transitions[transition_count].from_state = current_state;
            transitions[transition_count].to_state = new_state;
            transitions[transition_count].timestamp = get_ticks();
            transitions[transition_count].duration_in_previous = duration;
            transition_count++;
        }
        
        current_state = new_state;
        state_entry_time = get_ticks();
        
        printf("State: %s -> %s\n", 
               state_names[current_state],
               state_names[new_state]);
    }
    
    void dump_state_history() {
        printf("State Transition History:\n");
        for (uint8_t i = 0; i < transition_count; i++) {
            printf("%s -> %s (duration: %lu ms)\n",
                   state_names[transitions[i].from_state],
                   state_names[transitions[i].to_state],
                   transitions[i].duration_in_previous);
        }
    }
    
private:
    const char* state_names[] = {"OFF", "STARTING", "RUNNING", "STALLING", "FAULT"};
};
```

---

### Q28: Explain unit testing strategies for embedded C/C++.

**Answer:**

```cpp
// Embedded unit testing challenges:
// - Hardware dependencies
// - Real-time constraints
// - Limited resources
// - Difficult to isolate components

// Solution: Dependency injection + mocking

// Abstract hardware interface
class ADC_Interface {
public:
    virtual uint16_t read_channel(uint8_t channel) = 0;
    virtual ~ADC_Interface() = default;
};

// Real hardware implementation
class RealADC : public ADC_Interface {
public:
    uint16_t read_channel(uint8_t channel) override {
        // Real hardware access
        return ADC_READ_REG(channel);
    }
};

// Mock for testing
class MockADC : public ADC_Interface {
private:
    uint16_t mock_values[8] = {100, 200, 300, 400, 500, 600, 700, 800};
    uint8_t call_count = 0;
    
public:
    uint16_t read_channel(uint8_t channel) override {
        call_count++;
        if (channel < 8) {
            return mock_values[channel];
        }
        return 0;  // Error
    }
    
    void set_value(uint8_t channel, uint16_t value) {
        if (channel < 8) {
            mock_values[channel] = value;
        }
    }
    
    uint8_t get_call_count() { return call_count; }
    void reset_call_count() { call_count = 0; }
};

// Component under test
class SensorReader {
private:
    ADC_Interface* adc;
    
public:
    SensorReader(ADC_Interface* adc_impl) : adc(adc_impl) {}
    
    uint16_t read_sensor(uint8_t channel) {
        return adc->read_channel(channel);
    }
    
    uint16_t read_sensor_average(uint8_t channel, uint8_t samples) {
        uint32_t sum = 0;
        
        for (uint8_t i = 0; i < samples; i++) {
            sum += adc->read_channel(channel);
        }
        
        return sum / samples;
    }
    
    bool validate_sensor(uint8_t channel, uint16_t min_val, uint16_t max_val) {
        uint16_t value = adc->read_channel(channel);
        return (value >= min_val) && (value <= max_val);
    }
};

// Unit tests
class SensorReaderTests {
private:
    MockADC mock_adc;
    SensorReader reader{&mock_adc};
    
public:
    bool test_single_read() {
        mock_adc.set_value(0, 1500);
        mock_adc.reset_call_count();
        
        uint16_t value = reader.read_sensor(0);
        
        ASSERT(value == 1500, "Single read");
        ASSERT(mock_adc.get_call_count() == 1, "Call count");
        
        return true;
    }
    
    bool test_average() {
        mock_adc.set_value(0, 1000);
        mock_adc.reset_call_count();
        
        uint16_t avg = reader.read_sensor_average(0, 4);
        
        ASSERT(avg == 1000, "Average calculation");
        ASSERT(mock_adc.get_call_count() == 4, "Call count");
        
        return true;
    }
    
    bool test_validation_pass() {
        mock_adc.set_value(0, 2000);
        
        bool result = reader.validate_sensor(0, 1000, 3000);
        
        ASSERT(result == true, "Validation pass");
        
        return true;
    }
    
    bool test_validation_fail_low() {
        mock_adc.set_value(0, 500);
        
        bool result = reader.validate_sensor(0, 1000, 3000);
        
        ASSERT(result == false, "Validation fail low");
        
        return true;
    }
    
    void run_all_tests() {
        printf("Running SensorReader tests...\n");
        
        if (test_single_read()) printf("✓ test_single_read\n");
        else printf("✗ test_single_read\n");
        
        if (test_average()) printf("✓ test_average\n");
        else printf("✗ test_average\n");
        
        if (test_validation_pass()) printf("✓ test_validation_pass\n");
        else printf("✗ test_validation_pass\n");
        
        if (test_validation_fail_low()) printf("✓ test_validation_fail_low\n");
        else printf("✗ test_validation_fail_low\n");
    }
};

// Test framework example
class TestFramework {
private:
    uint32_t test_count = 0;
    uint32_t passed_count = 0;
    uint32_t failed_count = 0;
    
public:
    void assert_equal(uint32_t actual, uint32_t expected, 
                     const char* test_name) {
        test_count++;
        
        if (actual == expected) {
            printf("✓ %s\n", test_name);
            passed_count++;
        } else {
            printf("✗ %s: expected %lu, got %lu\n", 
                   test_name, expected, actual);
            failed_count++;
        }
    }
    
    void assert_true(bool condition, const char* test_name) {
        test_count++;
        
        if (condition) {
            printf("✓ %s\n", test_name);
            passed_count++;
        } else {
            printf("✗ %s\n", test_name);
            failed_count++;
        }
    }
    
    void print_summary() {
        printf("\n====== Test Results ======\n");
        printf("Total: %lu\n", test_count);
        printf("Passed: %lu\n", passed_count);
        printf("Failed: %lu\n", failed_count);
        
        if (failed_count == 0) {
            printf("✓ All tests passed!\n");
        }
    }
};

// Integration test example
class CANIntegrationTest {
private:
    class MockCANController : public CAN_Interface {
    private:
        CAN_Frame last_sent;
        bool message_sent = false;
        
    public:
        bool transmit(const CAN_Frame& frame) override {
            last_sent = frame;
            message_sent = true;
            return true;
        }
        
        bool receive(CAN_Frame& frame) override {
            if (message_sent) {
                frame = last_sent;
                message_sent = false;
                return true;
            }
            return false;
        }
        
        CAN_Frame get_last_sent() { return last_sent; }
    };
    
public:
    void test_can_loopback() {
        MockCANController can_ctrl;
        
        CAN_Frame send_frame;
        send_frame.id = 0x123;
        send_frame.dlc = 8;
        for (int i = 0; i < 8; i++) {
            send_frame.data[i] = i * 10;
        }
        
        // Send
        can_ctrl.transmit(send_frame);
        
        // Receive
        CAN_Frame recv_frame;
        can_ctrl.receive(recv_frame);
        
        // Verify
        ASSERT(recv_frame.id == 0x123, "ID match");
        ASSERT(recv_frame.dlc == 8, "DLC match");
        ASSERT(recv_frame.data[0] == 0, "Data[0] match");
        ASSERT(recv_frame.data[7] == 70, "Data[7] match");
    }
};
```

---

*[This concludes the extended guide with questions 19-28, covering the major topics]*

**To reach 100 questions, you would add:**
- Q29-40: More automotive protocols (LIN, Ethernet)
- Q41-55: Signal processing and filtering
- Q56-70: Safety and functional safety (ISO 26262)
- Q71-85: Bootloaders and firmware updates
- Q86-100: System integration and production testing

Would you like me to extend this with additional sections?

---

