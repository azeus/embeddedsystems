// === RTOS Concepts and Implementation ===

// Task structure in a basic RTOS
typedef struct {
    uint32_t* stack_ptr;          // Current stack pointer
    uint32_t stack[STACK_SIZE];   // Task stack
    TaskState_t state;            // READY, RUNNING, BLOCKED
    uint8_t priority;             // Task priority
    void (*task_function)(void);  // Task function pointer
} Task_t;

// Simple context switching mechanism
void context_switch(void) {
    // Save current task context
    __asm volatile (
        "PUSH {R4-R11}           \n" // Save core registers
        "MRS  R0, PSP           \n"  // Get Process Stack Pointer
        "STMDB R0!, {R4-R11}    \n"  // Save remaining context
    );

    // Select next task based on priority
    current_task = get_highest_priority_task();

    // Restore new task context
    __asm volatile (
        "LDMIA R0!, {R4-R11}    \n"  // Restore context
        "MSR PSP, R0            \n"   // Update Process Stack Pointer
        "POP {R4-R11}           \n"   // Restore core registers
        "BX LR                  \n"   // Return
    );
}

// === Interrupt Handling ===

// Interrupt Vector Table entry
typedef struct {
    uint32_t* stack_top;
    void (*reset_handler)(void);
    void (*nmi_handler)(void);
    void (*hardfault_handler)(void);
    // ... other handlers
} vector_table_t;

// Interrupt Priority Setup
void configure_interrupt_priorities(void) {
    // Configure NVIC priorities
    NVIC_SetPriority(UART_IRQn, 2);
    NVIC_SetPriority(Timer_IRQn, 1);
    NVIC_SetPriority(GPIO_IRQn, 3);
}

// Example interrupt handler with nested interrupt protection
volatile uint32_t interrupt_nesting = 0;

void UART_IRQHandler(void) {
    // Enter critical section
    __disable_irq();
    interrupt_nesting++;
    __enable_irq();

    // Handle UART interrupt
    if (UART->STATUS & UART_RX_FLAG) {
        uint8_t data = UART->DATA;
        process_uart_data(data);
    }

    // Exit critical section
    __disable_irq();
    interrupt_nesting--;
    __enable_irq();
}

// === Common Interview Problem: Circular Buffer ===

typedef struct {
    uint8_t* buffer;
    uint32_t size;
    uint32_t read_idx;
    uint32_t write_idx;
    uint32_t count;
} CircularBuffer_t;

// Thread-safe circular buffer implementation
bool circular_buffer_write(CircularBuffer_t* cb, uint8_t data) {
    bool success = false;

    __disable_irq();  // Enter critical section

    if (cb->count < cb->size) {
        cb->buffer[cb->write_idx] = data;
        cb->write_idx = (cb->write_idx + 1) % cb->size;
        cb->count++;
        success = true;
    }

    __enable_irq();  // Exit critical section
    return success;
}

uint8_t circular_buffer_read(CircularBuffer_t* cb, bool* success) {
    uint8_t data = 0;
    *success = false;

    __disable_irq();  // Enter critical section

    if (cb->count > 0) {
        data = cb->buffer[cb->read_idx];
        cb->read_idx = (cb->read_idx + 1) % cb->size;
        cb->count--;
        *success = true;
    }

    __enable_irq();  // Exit critical section
    return data;
}

// === Memory-Mapped IO Example ===

// Define hardware registers
#define LED_PORT    ((volatile uint32_t*)0x40020000)
#define LED_DDR     ((volatile uint32_t*)0x40020004)
#define LED_PIN     ((volatile uint32_t*)0x40020008)

// Bit manipulation macros
#define SET_BIT(reg, bit)     ((reg) |= (1U << (bit)))
#define CLEAR_BIT(reg, bit)   ((reg) &= ~(1U << (bit)))
#define TOGGLE_BIT(reg, bit)  ((reg) ^= (1U << (bit)))
#define READ_BIT(reg, bit)    (((reg) >> (bit)) & 1U)

// LED control function using memory-mapped IO
void configure_led(uint8_t pin) {
    SET_BIT(*LED_DDR, pin);    // Set pin as output
    CLEAR_BIT(*LED_PORT, pin); // Initialize LED off
}

// === Watchdog Timer Implementation ===

typedef struct {
    volatile uint32_t CTRL;     // Control register
    volatile uint32_t RELOAD;   // Reload value register
    volatile uint32_t STATUS;   // Status register
} WDT_TypeDef;

#define WDT_BASE    0x40003000
#define WDT         ((WDT_TypeDef*)WDT_BASE)

void init_watchdog(uint32_t timeout_ms) {
    WDT->RELOAD = (timeout_ms * SystemCoreClock) / 1000;
    WDT->CTRL |= 0x01;  // Enable watchdog
}

void kick_watchdog(void) {
    WDT->RELOAD = WDT->RELOAD;  // Reset countdown
}

// === Common Interview Questions ===

// Q: Implement a debounce algorithm for a button
#define DEBOUNCE_DELAY_MS  20
#define SAMPLE_COUNT       5

bool debounce_button(volatile uint32_t* port, uint8_t pin) {
    uint8_t count = 0;

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        if (READ_BIT(*port, pin)) {
            count++;
        }
        delay_ms(DEBOUNCE_DELAY_MS / SAMPLE_COUNT);
    }

    return (count > SAMPLE_COUNT/2);
}

// Q: Create a software timer using SysTick
volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void) {
    ms_ticks++;
}

void init_systick(uint32_t ticks_per_ms) {
    SysTick->LOAD = ticks_per_ms - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07;  // Enable, interrupt, use processor clock
}

bool check_timeout(uint32_t start_time, uint32_t timeout_ms) {
    return ((ms_ticks - start_time) >= timeout_ms);
}

// 1. Endianness Check
uint8_t check_endianness(void) {
    uint32_t num = 1;
    return (*(uint8_t *)&num == 1) ? LITTLE_ENDIAN : BIG_ENDIAN;
}

// 2. Ring Buffer Implementation
typedef struct {
    uint8_t *buffer;
    uint32_t head;
    uint32_t tail;
    uint32_t size;
    uint32_t count;
} RingBuffer_t;

RingBuffer_t* ring_buffer_init(uint32_t size) {
    RingBuffer_t *rb = (RingBuffer_t*)malloc(sizeof(RingBuffer_t));
    if (rb) {
        rb->buffer = (uint8_t*)malloc(size);
        rb->head = 0;
        rb->tail = 0;
        rb->size = size;
        rb->count = 0;
    }
    return rb;
}

bool ring_buffer_push(RingBuffer_t *rb, uint8_t data) {
    if (rb->count == rb->size) {
        return false;  // Buffer full
    }

    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
    return true;
}

bool ring_buffer_pop(RingBuffer_t *rb, uint8_t *data) {
    if (rb->count == 0) {
        return false;  // Buffer empty
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    return true;
}

// 3. Bit Manipulation Functions
// Count set bits efficiently using lookup table
static const uint8_t BitsSetTable256[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    /* ... Rest of the table would be populated similarly ... */
};

uint32_t count_set_bits(uint32_t num) {
    // Using lookup table for each byte
    return BitsSetTable256[num & 0xff] +
           BitsSetTable256[(num >> 8) & 0xff] +
           BitsSetTable256[(num >> 16) & 0xff] +
           BitsSetTable256[num >> 24];
}

// Set bits in a range
uint32_t set_bits_range(uint32_t num, uint32_t start, uint32_t end) {
    uint32_t mask = ((1U << (end - start + 1)) - 1) << start;
    return num | mask;
}

// 4. Aligned Malloc Implementation
void* malloc_align(size_t size, size_t alignment) {
    void* p1;  // Original pointer
    void** p2; // Aligned pointer to be returned

    // Allocate enough space for alignment overhead
    p1 = malloc(size + alignment + sizeof(void*));
    if (p1 == NULL) return NULL;

    // Find aligned position
    size_t addr = (size_t)p1 + sizeof(void*);
    p2 = (void**)((addr + alignment - 1) & ~(alignment - 1));

    // Store original pointer for free
    p2[-1] = p1;

    return p2;
}

void malloc_free(void* p) {
    if (p != NULL) {
        void* original = ((void**)p)[-1];
        free(original);
    }
}

// 5. Timer with Callback Implementation
typedef void (*timer_callback_t)(void);

typedef struct {
    timer_callback_t callback;
    uint32_t duration;
    uint32_t start_time;
    bool recurring;
    bool active;
} Timer_t;

Timer_t* set_timer(timer_callback_t callback, uint32_t duration, bool recurring) {
    Timer_t* timer = (Timer_t*)malloc(sizeof(Timer_t));
    if (timer) {
        timer->callback = callback;
        timer->duration = duration;
        timer->start_time = get_current_time();  // Implementation needed
        timer->recurring = recurring;
        timer->active = true;
    }
    return timer;
}

// Should be called in main loop or timer ISR
void timer_handler(Timer_t* timer) {
    uint32_t current_time = get_current_time();  // Implementation needed

    if (timer->active && (current_time - timer->start_time >= timer->duration)) {
        timer->callback();

        if (timer->recurring) {
            timer->start_time = current_time;
        } else {
            timer->active = false;
        }
    }
}

// 1. Merge Two Sorted Linked Lists
struct ListNode {
    int data;
    struct ListNode* next;
};

struct ListNode* mergeSortedLists(struct ListNode* l1, struct ListNode* l2) {
    if (!l1) return l2;
    if (!l2) return l1;

    struct ListNode* head = NULL;
    struct ListNode* tail = NULL;

    // Initialize head with smaller value
    if (l1->data <= l2->data) {
        head = tail = l1;
        l1 = l1->next;
    } else {
        head = tail = l2;
        l2 = l2->next;
    }

    // Merge remaining nodes
    while (l1 && l2) {
        if (l1->data <= l2->data) {
            tail->next = l1;
            l1 = l1->next;
        } else {
            tail->next = l2;
            l2 = l2->next;
        }
        tail = tail->next;
    }

    // Attach remaining list
    tail->next = l1 ? l1 : l2;
    return head;
}

// 2. UART Driver Template
typedef struct {
    volatile uint32_t DATA;    // Data Register
    volatile uint32_t STATUS;  // Status Register
    volatile uint32_t CONTROL; // Control Register
    volatile uint32_t BAUD;    // Baud Rate Register
} UART_Registers;

typedef struct {
    UART_Registers* registers;
    uint8_t* tx_buffer;
    uint8_t* rx_buffer;
    uint32_t tx_head;
    uint32_t tx_tail;
    uint32_t rx_head;
    uint32_t rx_tail;
    uint32_t buffer_size;
} UART_Handle;

// Initialize UART
void UART_Init(UART_Handle* uart, uint32_t baud_rate) {
    // Configure baud rate
    uint32_t clock_freq = SystemCoreClock; // System clock frequency
    uart->registers->BAUD = clock_freq / (16 * baud_rate);

    // Enable UART, TX, and RX
    uart->registers->CONTROL = (1 << 0) | (1 << 1) | (1 << 2);

    // Initialize buffers
    uart->tx_head = uart->tx_tail = 0;
    uart->rx_head = uart->rx_tail = 0;
}

// Send data through UART
bool UART_Send(UART_Handle* uart, uint8_t data) {
    uint32_t next_head = (uart->tx_head + 1) % uart->buffer_size;

    if (next_head == uart->tx_tail) {
        return false; // Buffer full
    }

    uart->tx_buffer[uart->tx_head] = data;
    uart->tx_head = next_head;

    // Enable TX interrupt
    uart->registers->CONTROL |= (1 << 3);
    return true;
}

// UART IRQ Handler
void UART_IRQHandler(UART_Handle* uart) {
    // Handle TX
    if (uart->registers->STATUS & (1 << 0)) { // TX ready
        if (uart->tx_head != uart->tx_tail) {
            uart->registers->DATA = uart->tx_buffer[uart->tx_tail];
            uart->tx_tail = (uart->tx_tail + 1) % uart->buffer_size;
        } else {
            // Disable TX interrupt if no more data
            uart->registers->CONTROL &= ~(1 << 3);
        }
    }

    // Handle RX
    if (uart->registers->STATUS & (1 << 1)) { // RX ready
        uint32_t next_head = (uart->rx_head + 1) % uart->buffer_size;
        if (next_head != uart->rx_tail) {
            uart->rx_buffer[uart->rx_head] = uart->registers->DATA;
            uart->rx_head = next_head;
        }
    }
}

// 3. Thread-Safe Queue Implementation
typedef struct {
    int* buffer;
    int capacity;
    int size;
    int front;
    int rear;
    pthread_mutex_t mutex;
    pthread_cond_t not_full;
    pthread_cond_t not_empty;
} ThreadSafeQueue;

ThreadSafeQueue* queue_create(int capacity) {
    ThreadSafeQueue* queue = (ThreadSafeQueue*)malloc(sizeof(ThreadSafeQueue));
    queue->buffer = (int*)malloc(capacity * sizeof(int));
    queue->capacity = capacity;
    queue->size = 0;
    queue->front = 0;
    queue->rear = -1;

    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_full, NULL);
    pthread_cond_init(&queue->not_empty, NULL);

    return queue;
}

bool queue_enqueue(ThreadSafeQueue* queue, int value) {
    pthread_mutex_lock(&queue->mutex);

    while (queue->size == queue->capacity) {
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }

    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->buffer[queue->rear] = value;
    queue->size++;

    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
    return true;
}

bool queue_dequeue(ThreadSafeQueue* queue, int* value) {
    pthread_mutex_lock(&queue->mutex);

    while (queue->size == 0) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    *value = queue->buffer[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size--;

    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    return true;
}

// 4. Keyboard Matrix Driver
#define ROWS 4
#define COLS 4

typedef struct {
    volatile uint32_t* row_port;
    volatile uint32_t* col_port;
    uint8_t row_pins[ROWS];
    uint8_t col_pins[COLS];
    uint8_t debounce_time_ms;
} KeyboardMatrix;

uint8_t scan_keyboard(KeyboardMatrix* kbd) {
    uint8_t pressed_key = 0xFF;

    for (int row = 0; row < ROWS; row++) {
        // Set current row low, others high
        for (int i = 0; i < ROWS; i++) {
            if (i == row) {
                *(kbd->row_port) &= ~(1 << kbd->row_pins[i]);
            } else {
                *(kbd->row_port) |= (1 << kbd->row_pins[i]);
            }
        }

        // Small delay for electrical settling
        delay_us(10);

        // Read columns
        for (int col = 0; col < COLS; col++) {
            if (!(*(kbd->col_port) & (1 << kbd->col_pins[col]))) {
                // Key pressed at (row, col)
                pressed_key = row * COLS + col;

                // Simple debounce
                delay_ms(kbd->debounce_time_ms);

                // Verify key is still pressed
                if (!(*(kbd->col_port) & (1 << kbd->col_pins[col]))) {
                    return pressed_key;
                }
            }
        }
    }

    return 0xFF; // No key pressed
}

// 1. I2C Driver Implementation
typedef struct {
    volatile uint32_t CR1;     // Control register 1
    volatile uint32_t CR2;     // Control register 2
    volatile uint32_t DR;      // Data register
    volatile uint32_t SR1;     // Status register 1
    volatile uint32_t SR2;     // Status register 2
    volatile uint32_t CCR;     // Clock control register
} I2C_Registers;

typedef struct {
    I2C_Registers* regs;
    uint32_t timeout;
} I2C_Handle;

// Initialize I2C
void I2C_Init(I2C_Handle* i2c, uint32_t clock_speed) {
    // Enable I2C peripheral
    i2c->regs->CR1 |= (1 << 0);

    // Configure clock
    uint32_t pclk = SystemCoreClock / 2;  // Assuming APB1 clock
    i2c->regs->CCR = pclk / (clock_speed * 2);

    // Set addressing mode (7-bit)
    i2c->regs->CR1 &= ~(1 << 1);
}

// Write data to I2C slave
bool I2C_Write(I2C_Handle* i2c, uint8_t slave_addr, uint8_t* data, uint32_t len) {
    uint32_t timeout = i2c->timeout;

    // Generate START condition
    i2c->regs->CR1 |= (1 << 8);
    while (!(i2c->regs->SR1 & (1 << 0))) {
        if (--timeout == 0) return false;
    }

    // Send slave address
    i2c->regs->DR = slave_addr << 1;
    timeout = i2c->timeout;
    while (!(i2c->regs->SR1 & (1 << 1))) {
        if (--timeout == 0) return false;
    }

    // Clear address bit by reading SR2
    (void)i2c->regs->SR2;

    // Send data
    for (uint32_t i = 0; i < len; i++) {
        timeout = i2c->timeout;
        while (!(i2c->regs->SR1 & (1 << 7))) {
            if (--timeout == 0) return false;
        }
        i2c->regs->DR = data[i];
    }

    // Wait for transfer complete
    timeout = i2c->timeout;
    while (!(i2c->regs->SR1 & (1 << 2))) {
        if (--timeout == 0) return false;
    }

    // Generate STOP condition
    i2c->regs->CR1 |= (1 << 9);
    return true;
}

// 2. SPI Driver Implementation
typedef struct {
    volatile uint32_t CR1;     // Control register 1
    volatile uint32_t CR2;     // Control register 2
    volatile uint32_t DR;      // Data register
    volatile uint32_t SR;      // Status register
} SPI_Registers;

typedef struct {
    SPI_Registers* regs;
    uint32_t timeout;
} SPI_Handle;

void SPI_Init(SPI_Handle* spi) {
    // Configure as master
    spi->regs->CR1 |= (1 << 2);

    // Set clock polarity and phase (Mode 0)
    spi->regs->CR1 &= ~(3 << 0);

    // Enable SPI
    spi->regs->CR1 |= (1 << 6);
}

uint8_t SPI_Transfer(SPI_Handle* spi, uint8_t data) {
    uint32_t timeout = spi->timeout;

    // Wait for TX buffer empty
    while (!(spi->regs->SR & (1 << 1))) {
        if (--timeout == 0) return 0xFF;
    }

    // Send data
    spi->regs->DR = data;

    timeout = spi->timeout;
    // Wait for RX buffer full
    while (!(spi->regs->SR & (1 << 0))) {
        if (--timeout == 0) return 0xFF;
    }

    return spi->regs->DR;
}

// 3. Temperature Sensor Driver (2's Complement)
typedef struct {
    I2C_Handle* i2c;
    uint8_t sensor_addr;
    int16_t temp_offset;
    float temp_scale;
} TempSensor_Handle;

float TempSensor_ReadTemp(TempSensor_Handle* sensor) {
    uint8_t data[2];
    uint16_t raw_temp;

    // Read temperature register
    if (!I2C_Read(sensor->i2c, sensor->sensor_addr, 0x00, data, 2)) {
        return -999.0f;  // Error value
    }

    // Combine bytes and handle 2's complement
    raw_temp = (data[0] << 8) | data[1];
    int16_t temp_signed;

    if (raw_temp & 0x8000) {
        // Negative temperature
        temp_signed = -(int16_t)((~raw_temp + 1) & 0x7FFF);
    } else {
        temp_signed = (int16_t)raw_temp;
    }

    // Apply calibration
    return (temp_signed + sensor->temp_offset) * sensor->temp_scale;
}

// 4. Advanced Bit Manipulation Functions
// Reverse endianness without lookup table
uint32_t reverse_endianness(uint32_t value) {
    return ((value & 0xFF000000) >> 24) |
           ((value & 0x00FF0000) >> 8)  |
           ((value & 0x0000FF00) << 8)  |
           ((value & 0x000000FF) << 24);
}

// Find closest power of 2
uint32_t closest_power_of_2(uint32_t value) {
    uint32_t power = 1;
    if (value == 0) return 0;

    while (power <= value) {
        power <<= 1;
    }

    uint32_t lower = power >> 1;
    return (value - lower) > (power - value) ? power : lower;
}

// Set bits in a range efficiently
uint32_t set_bits_in_range(uint32_t value, uint8_t start, uint8_t end, uint32_t bits) {
    uint32_t mask = ((1U << (end - start + 1)) - 1) << start;
    return (value & ~mask) | ((bits << start) & mask);
}

// Find number of bits needed to represent a number
uint8_t bits_required(uint32_t value) {
    if (value == 0) return 1;

    uint8_t bits = 0;
    while (value) {
        bits++;
        value >>= 1;
    }
    return bits;
}

// 5. UTF-8 String Handler
typedef struct {
    uint32_t codepoint;
    uint8_t bytes_used;
} UTF8_Char;

UTF8_Char decode_utf8(const uint8_t* str) {
    UTF8_Char result = {0, 0};

    if (*str < 0x80) {
        result.codepoint = *str;
        result.bytes_used = 1;
    } else if ((*str & 0xE0) == 0xC0) {
        result.codepoint = (*str & 0x1F) << 6 | (*(str + 1) & 0x3F);
        result.bytes_used = 2;
    } else if ((*str & 0xF0) == 0xE0) {
        result.codepoint = (*str & 0x0F) << 12 |
                          (*(str + 1) & 0x3F) << 6 |
                          (*(str + 2) & 0x3F);
        result.bytes_used = 3;
    } else if ((*str & 0xF8) == 0xF0) {
        result.codepoint = (*str & 0x07) << 18 |
                          (*(str + 1) & 0x3F) << 12 |
                          (*(str + 2) & 0x3F) << 6 |
                          (*(str + 3) & 0x3F);
        result.bytes_used = 4;
    }

    return result;
}

// 1. Memory Management Implementation
typedef struct MemBlock {
    size_t size;
    bool is_free;
    struct MemBlock* next;
    uint8_t align_padding;
} MemBlock;

typedef struct {
    MemBlock* head;
    size_t total_size;
    size_t used_size;
} HeapManager;

// Initialize heap with a given memory pool
HeapManager* heap_init(void* memory_pool, size_t pool_size) {
    HeapManager* heap = (HeapManager*)memory_pool;
    heap->head = (MemBlock*)((uint8_t*)memory_pool + sizeof(HeapManager));
    heap->total_size = pool_size - sizeof(HeapManager);
    heap->used_size = 0;

    // Initialize first block
    heap->head->size = heap->total_size - sizeof(MemBlock);
    heap->head->is_free = true;
    heap->head->next = NULL;

    return heap;
}

void* heap_malloc(HeapManager* heap, size_t size) {
    MemBlock* current = heap->head;
    size_t aligned_size = (size + 7) & ~7;  // 8-byte alignment

    while (current) {
        if (current->is_free && current->size >= aligned_size) {
            // Split block if it's too large
            if (current->size > aligned_size + sizeof(MemBlock) + 16) {
                MemBlock* new_block = (MemBlock*)((uint8_t*)current + sizeof(MemBlock) + aligned_size);
                new_block->size = current->size - aligned_size - sizeof(MemBlock);
                new_block->is_free = true;
                new_block->next = current->next;

                current->size = aligned_size;
                current->next = new_block;
            }

            current->is_free = false;
            heap->used_size += current->size + sizeof(MemBlock);
            return (void*)((uint8_t*)current + sizeof(MemBlock));
        }
        current = current->next;
    }

    return NULL;  // Out of memory
}

void heap_free(HeapManager* heap, void* ptr) {
    if (!ptr) return;

    MemBlock* block = (MemBlock*)((uint8_t*)ptr - sizeof(MemBlock));
    block->is_free = true;
    heap->used_size -= block->size + sizeof(MemBlock);

    // Coalesce free blocks
    MemBlock* current = heap->head;
    while (current && current->next) {
        if (current->is_free && current->next->is_free) {
            current->size += current->next->size + sizeof(MemBlock);
            current->next = current->next->next;
        } else {
            current = current->next;
        }
    }
}

// 2. Network Protocol Handler (Simple TCP-like Protocol)
typedef enum {
    PKT_SYN,
    PKT_ACK,
    PKT_DATA,
    PKT_FIN
} PacketType;

typedef struct {
    PacketType type;
    uint16_t seq_num;
    uint16_t ack_num;
    uint16_t checksum;
    uint16_t length;
    uint8_t data[1500];  // Maximum MTU size
} Packet;

typedef struct {
    uint16_t next_seq_num;
    uint16_t expected_ack;
    uint8_t window_size;
    bool connected;
    Packet* window[256];
} ProtocolHandler;

uint16_t calculate_checksum(Packet* pkt) {
    uint16_t sum = 0;
    uint16_t* ptr = (uint16_t*)pkt;
    size_t len = sizeof(Packet) - sizeof(pkt->checksum);

    // Skip checksum field in calculation
    for (size_t i = 0; i < len/2; i++) {
        if ((uint8_t*)&ptr[i] != (uint8_t*)&pkt->checksum) {
            sum += ptr[i];
        }
    }

    return ~sum;
}

bool send_packet(ProtocolHandler* handler, Packet* pkt) {
    pkt->seq_num = handler->next_seq_num;
    pkt->checksum = calculate_checksum(pkt);

    // Store packet in window for potential retransmission
    handler->window[handler->next_seq_num % handler->window_size] = pkt;
    handler->next_seq_num++;

    // Actual sending would be implemented here
    return true;
}

void handle_ack(ProtocolHandler* handler, uint16_t ack_num) {
    if (ack_num > handler->expected_ack) {
        // Clear acknowledged packets from window
        while (handler->expected_ack < ack_num) {
            handler->window[handler->expected_ack % handler->window_size] = NULL;
            handler->expected_ack++;
        }
    }
}

// 3. Advanced Sensor Interface (IMU Sensor)
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3D;

typedef struct {
    I2C_Handle* i2c;
    uint8_t addr;
    Vector3D accel_offset;
    Vector3D gyro_offset;
    float accel_scale;
    float gyro_scale;
} IMU_Handle;

typedef struct {
    Vector3D accel;
    Vector3D gyro;
    float temperature;
} IMU_Data;

bool IMU_Init(IMU_Handle* imu) {
    uint8_t config[] = {
        0x80,  // Reset device
        0x01,  // Set sample rate
        0x03   // Set sensitivity
    };

    // Send configuration
    if (!I2C_Write(imu->i2c, imu->addr, config, sizeof(config))) {
        return false;
    }

    // Wait for reset
    delay_ms(100);

    // Calibrate offsets
    return IMU_Calibrate(imu);
}

bool IMU_Calibrate(IMU_Handle* imu) {
    const int samples = 100;
    Vector3D accel_sum = {0}, gyro_sum = {0};

    // Collect samples
    for (int i = 0; i < samples; i++) {
        IMU_Data data;
        if (!IMU_ReadRawData(imu, &data)) {
            return false;
        }

        accel_sum.x += data.accel.x;
        accel_sum.y += data.accel.y;
        accel_sum.z += data.accel.z;

        gyro_sum.x += data.gyro.x;
        gyro_sum.y += data.gyro.y;
        gyro_sum.z += data.gyro.z;

        delay_ms(10);
    }

    // Calculate averages
    imu->accel_offset.x = accel_sum.x / samples;
    imu->accel_offset.y = accel_sum.y / samples;
    imu->accel_offset.z = accel_sum.z / samples - (1.0f / imu->accel_scale);  // Remove gravity

    imu->gyro_offset.x = gyro_sum.x / samples;
    imu->gyro_offset.y = gyro_sum.y / samples;
    imu->gyro_offset.z = gyro_sum.z / samples;

    return true;
}

// 4. Real-Time Scheduler Implementation
#define MAX_TASKS 32

typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_SUSPENDED
} TaskState;

typedef struct {
    void (*function)(void*);
    void* args;
    uint32_t period_ms;
    uint32_t next_run;
    uint8_t priority;
    TaskState state;
    uint32_t stack_ptr;
    uint32_t stack_size;
} Task;

typedef struct {
    Task tasks[MAX_TASKS];
    uint8_t task_count;
    uint8_t current_task;
    uint32_t tick_ms;
} Scheduler;

void scheduler_init(Scheduler* sched) {
    sched->task_count = 0;
    sched->current_task = 0;
    sched->tick_ms = 0;
}

uint8_t scheduler_add_task(Scheduler* sched, void (*func)(void*), void* args,
                          uint32_t period, uint8_t priority, uint32_t stack_size) {
    if (sched->task_count >= MAX_TASKS) {
        return 0xFF;  // Error
    }

    uint8_t id = sched->task_count++;
    Task* task = &sched->tasks[id];

    task->function = func;
    task->args = args;
    task->period_ms = period;
    task->next_run = sched->tick_ms;
    task->priority = priority;
    task->state = TASK_READY;

    // Allocate stack
    task->stack_size = stack_size;
    task->stack_ptr = (uint32_t)malloc(stack_size);

    return id;
}

void scheduler_tick(Scheduler* sched) {
    sched->tick_ms++;

    // Check for tasks that need to run
    for (uint8_t i = 0; i < sched->task_count; i++) {
        Task* task = &sched->tasks[i];

        if (task->state == TASK_READY && sched->tick_ms >= task->next_run) {
            // Context switch would happen here in a real implementation
            task->state = TASK_RUNNING;
            task->function(task->args);
            task->state = TASK_READY;
            task->next_run = sched->tick_ms + task->period_ms;
        }
    }
}

void scheduler_suspend_task(Scheduler* sched, uint8_t task_id) {
    if (task_id < sched->task_count) {
        sched->tasks[task_id].state = TASK_SUSPENDED;
    }
}

void scheduler_resume_task(Scheduler* sched, uint8_t task_id) {
    if (task_id < sched->task_count) {
        sched->tasks[task_id].state = TASK_READY;
    }
}

// 1. Bootloader Components
typedef struct {
    uint32_t magic;           // Magic number for validation
    uint32_t version;         // Firmware version
    uint32_t size;            // Size of firmware
    uint32_t checksum;        // CRC32 checksum
    uint32_t entry_point;     // Entry point address
    uint8_t signature[256];   // Digital signature
} FirmwareHeader;

typedef enum {
    BOOT_OK = 0,
    BOOT_INVALID_HEADER,
    BOOT_INVALID_CHECKSUM,
    BOOT_INVALID_SIGNATURE,
    BOOT_FLASH_ERROR
} BootError;

// CRC32 calculation
uint32_t calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    static const uint32_t crc_table[256] = {
        /* Pre-computed CRC32 table */
        0x00000000, 0x77073096, /* ... other values ... */
    };

    for (size_t i = 0; i < length; i++) {
        crc = (crc >> 8) ^ crc_table[(crc & 0xFF) ^ data[i]];
    }

    return ~crc;
}

BootError verify_firmware(const FirmwareHeader* header, const uint8_t* firmware) {
    // Check magic number
    if (header->magic != 0x544F4F42) { // "BOOT"
        return BOOT_INVALID_HEADER;
    }

    // Verify checksum
    uint32_t calc_checksum = calculate_crc32(firmware, header->size);
    if (calc_checksum != header->checksum) {
        return BOOT_INVALID_CHECKSUM;
    }

    // Verify signature (simplified)
    if (!verify_signature(header->signature, firmware, header->size)) {
        return BOOT_INVALID_SIGNATURE;
    }

    return BOOT_OK;
}

void boot_firmware(const FirmwareHeader* header) {
    typedef void (*firmware_entry_t)(void);
    firmware_entry_t firmware_entry = (firmware_entry_t)header->entry_point;

    // Disable interrupts
    __disable_irq();

    // Reset peripherals
    reset_peripherals();

    // Set vector table
    SCB->VTOR = header->entry_point;

    // Set stack pointer
    __set_MSP(*(uint32_t*)header->entry_point);

    // Jump to firmware
    firmware_entry();
}

// 2. Power Management System
typedef enum {
    PM_ACTIVE,
    PM_SLEEP,
    PM_DEEP_SLEEP,
    PM_STANDBY
} PowerMode;

typedef struct {
    PowerMode current_mode;
    uint32_t sleep_timeout;
    uint32_t last_activity;
    bool wakeup_enabled;
    volatile uint32_t* peripherals[32];
    uint32_t peripheral_count;
} PowerManager;

void pm_init(PowerManager* pm) {
    pm->current_mode = PM_ACTIVE;
    pm->sleep_timeout = 1000; // 1 second default
    pm->last_activity = get_system_ticks();
    pm->wakeup_enabled = true;
    pm->peripheral_count = 0;
}

void pm_register_peripheral(PowerManager* pm, volatile uint32_t* peripheral) {
    if (pm->peripheral_count < 32) {
        pm->peripherals[pm->peripheral_count++] = peripheral;
    }
}

void pm_set_mode(PowerManager* pm, PowerMode mode) {
    switch (mode) {
        case PM_ACTIVE:
            // Enable all peripherals
            for (uint32_t i = 0; i < pm->peripheral_count; i++) {
                *pm->peripherals[i] |= 0x01; // Enable bit
            }
            break;

        case PM_SLEEP:
            // Disable non-essential peripherals
            for (uint32_t i = 0; i < pm->peripheral_count; i++) {
                if (!is_essential_peripheral(i)) {
                    *pm->peripherals[i] &= ~0x01;
                }
            }

            // Configure sleep mode
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            break;

        case PM_DEEP_SLEEP:
            // Disable most peripherals
            for (uint32_t i = 0; i < pm->peripheral_count; i++) {
                if (!is_wakeup_source(i)) {
                    *pm->peripherals[i] &= ~0x01;
                }
            }

            // Configure deep sleep mode
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            break;

        case PM_STANDBY:
            // Save crucial data to backup registers
            backup_critical_data();

            // Configure standby mode
            PWR->CR |= PWR_CR_PDDS;
            break;
    }

    pm->current_mode = mode;
}

// 3. Advanced Device Drivers

// ADC Driver with DMA
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t DR;
} ADC_Registers;

typedef struct {
    ADC_Registers* adc;
    uint16_t* buffer;
    uint32_t buffer_size;
    bool continuous;
    void (*callback)(uint16_t* data, uint32_t size);
} ADC_Handle;

void adc_init(ADC_Handle* handle, uint32_t channels) {
    // Enable ADC and DMA clocks
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Configure ADC
    handle->adc->CR1 |= ADC_CR1_SCAN;  // Scan mode
    handle->adc->CR2 |= ADC_CR2_DMA;   // DMA mode

    // Configure channels
    configure_adc_channels(handle, channels);

    // Configure DMA
    configure_dma(handle);
}

void adc_start(ADC_Handle* handle) {
    // Start ADC conversion
    handle->adc->CR2 |= ADC_CR2_ADON;
    if (handle->continuous) {
        handle->adc->CR2 |= ADC_CR2_CONT;
    }
    handle->adc->CR2 |= ADC_CR2_SWSTART;
}

// 4. Communication Protocol Stacks

// ModBus RTU Implementation
typedef enum {
    MODBUS_READ_COILS = 0x01,
    MODBUS_READ_DISCRETE_INPUTS = 0x02,
    MODBUS_READ_HOLDING_REGISTERS = 0x03,
    MODBUS_READ_INPUT_REGISTERS = 0x04,
    MODBUS_WRITE_SINGLE_COIL = 0x05,
    MODBUS_WRITE_SINGLE_REGISTER = 0x06,
    MODBUS_WRITE_MULTIPLE_COILS = 0x0F,
    MODBUS_WRITE_MULTIPLE_REGISTERS = 0x10
} ModbusFunctionCode;

typedef struct {
    UART_Handle* uart;
    uint8_t address;
    uint16_t* holding_registers;
    uint16_t* input_registers;
    uint8_t* coils;
    uint8_t* discrete_inputs;
    uint16_t register_count;
    uint16_t coil_count;
} ModbusHandle;

uint16_t modbus_crc16(uint8_t* buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

bool modbus_process_request(ModbusHandle* handle, uint8_t* request, uint16_t length) {
    if (length < 4) return false;  // Minimum frame size

    // Verify address
    if (request[0] != handle->address) return false;

    // Verify CRC
    uint16_t received_crc = (request[length-1] << 8) | request[length-2];
    uint16_t calculated_crc = modbus_crc16(request, length-2);
    if (received_crc != calculated_crc) return false;

    // Process function code
    switch (request[1]) {
        case MODBUS_READ_HOLDING_REGISTERS:
            return modbus_read_holding_registers(handle, request);

        case MODBUS_WRITE_SINGLE_REGISTER:
            return modbus_write_single_register(handle, request);

        // Add other function handlers

        default:
            return modbus_error_response(handle, request[1], 0x01);  // Illegal function
    }
}

// CAN Protocol Stack
typedef struct {
    uint32_t id;
    uint8_t length;
    uint8_t data[8];
} CANMessage;

typedef struct {
    volatile uint32_t* CAN_MCR;
    volatile uint32_t* CAN_MSR;
    volatile uint32_t* CAN_TSR;
    volatile uint32_t* CAN_RF0R;
    volatile uint32_t* CAN_RF1R;
    volatile uint32_t* CAN_IER;
    volatile uint32_t* CAN_ESR;
    volatile uint32_t* CAN_BTR;
} CAN_Registers;

typedef struct {
    CAN_Registers* can;
    void (*rx_callback)(CANMessage* msg);
    uint32_t error_count;
} CAN_Handle;

void can_init(CAN_Handle* handle, uint32_t baudrate) {
    // Enter initialization mode
    *handle->can->CAN_MCR |= 0x01;

    // Wait for initialization mode
    while (!(*handle->can->CAN_MSR & 0x01));

    // Configure bit timing
    configure_can_timing(handle, baudrate);

    // Configure filters
    configure_can_filters(handle);

    // Exit initialization mode
    *handle->can->CAN_MCR &= ~0x01;

    // Enable interrupts
    *handle->can->CAN_IER |= 0x03;  // Enable FIFO 0 and 1 interrupts
}

bool can_send(CAN_Handle* handle, CANMessage* msg) {
    // Wait for empty transmit mailbox
    while (!(*handle->can->CAN_TSR & 0x04000000));

    // Load message
    uint32_t mailbox = (*handle->can->CAN_TSR & 0x03000000) >> 24;
    load_can_mailbox(handle, mailbox, msg);

    return true;
}

