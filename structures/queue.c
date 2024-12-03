#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct Node {
    int data;
    struct Node *next;
} Node;

typedef struct {
    Node *front, *rear;
    int size;
} Queue;

void queue_init(Queue *q) {
    q->front = q->rear = NULL;
    q->size = 0;
}

void enqueue(Queue *q, int value) {
    Node *newNode = (Node *)malloc(sizeof(Node));
    newNode->data = value;
    newNode->next = NULL;

    if (q->rear == NULL) {
        q->front = q->rear = newNode;
    } else {
        q->rear->next = newNode;
        q->rear = newNode;
    }
    q->size++;
}

bool dequeue(Queue *q, int *value) {
    if (q->front == NULL) {
        return false;  // Queue empty
    }

    Node *temp = q->front;
    *value = temp->data;
    q->front = q->front->next;

    if (q->front == NULL) {
        q->rear = NULL;
    }

    free(temp);
    q->size--;
    return true;
}

bool queue_peek(Queue *q, int *value) {
    if (q->front == NULL) {
        return false;  // Queue empty
    }
    *value = q->front->data;
    return true;
}

int queue_size(Queue *q) {
    return q->size;
}

// Test function
void test_queue() {
    Queue q;
    queue_init(&q);

    printf("Enqueuing elements...\n");
    for (int i = 1; i <= 7; ++i) {
        enqueue(&q, i);
        printf("Enqueued: %d\n", i);
    }

    int value;
    if (queue_peek(&q, &value)) {
        printf("\nFront of the queue: %d\n", value);
    }

    printf("\nDequeuing elements...\n");
    while (dequeue(&q, &value)) {
        printf("Dequeued: %d\n", value);
    }
}