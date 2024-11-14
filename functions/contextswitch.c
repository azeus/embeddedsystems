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