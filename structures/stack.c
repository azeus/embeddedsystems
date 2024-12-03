#include <stdio.h>
#include <stdbool.h>

#define MAX 10

typedef struct {
    int arr[MAX];
    int top;
} Stack;

void stack_init(Stack *s) {
    s->top = -1;
}

bool stack_push(Stack *s, int value) {
    if (s->top >= MAX - 1) {
        return false;  // Stack Overflow
    }
    s->arr[++s->top] = value;
    return true;
}

bool stack_pop(Stack *s, int *value) {
    if (s->top < 0) {
        return false;  // Stack Underflow
    }
    *value = s->arr[s->top--];
    return true;
}

bool stack_peek(Stack *s, int *value) {
    if (s->top < 0) {
        return false;  // Stack Empty
    }
    *value = s->arr[s->top];
    return true;
}

int stack_size(Stack *s) {
    return s->top + 1;
}

// Test function
void test_stack() {
    Stack s;
    stack_init(&s);

    printf("Pushing elements to the stack...\n");
    for (int i = 1; i <= 12; ++i) {
        if (stack_push(&s, i)) {
            printf("Pushed: %d\n", i);
        } else {
            printf("Failed to push: %d (Stack Full)\n", i);
        }
    }

    int value;
    if (stack_peek(&s, &value)) {
        printf("\nTop element: %d\n", value);
    }

    printf("\nPopping elements from the stack...\n");
    while (stack_pop(&s, &value)) {
        printf("Popped: %d\n", value);
    }
}