#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 5

typedef struct {
    int buffer[BUFFER_SIZE];
    int head;
    int tail;
    bool full;
} CircularBuffer;

void cb_init(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->full = false;
}

bool cb_put(CircularBuffer *cb, int data) {
    if (cb->full) {
        return false;  // Buffer is full
    }

    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % BUFFER_SIZE;

    if (cb->head == cb->tail) {
        cb->full = true;  // Buffer becomes full
    }

    return true;
}

bool cb_get(CircularBuffer *cb, int *data) {
    if (cb->head == cb->tail && !cb->full) {
        return false;  // Buffer is empty
    }

    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->full = false;
    return true;
}

bool cb_is_empty(CircularBuffer *cb) {
    return (cb->head == cb->tail && !cb->full);
}

bool cb_is_full(CircularBuffer *cb) {
    return cb->full;
}

// Test function
void test_circular_buffer() {
    CircularBuffer cb;
    cb_init(&cb);

    int value;
    printf("Adding elements to the buffer...\n");
    for (int i = 1; i <= BUFFER_SIZE; ++i) {
        if (cb_put(&cb, i)) {
            printf("Added: %d\n", i);
        } else {
            printf("Failed to add: %d (Buffer Full)\n", i);
        }
    }

    printf("\nRetrieving elements from the buffer...\n");
    while (!cb_is_empty(&cb)) {
        if (cb_get(&cb, &value)) {
            printf("Retrieved: %d\n", value);
        }
    }
}