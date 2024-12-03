#include <stdio.h>
#include <stdlib.h>

typedef struct Node {
    int data;
    struct Node *next;
} Node;

Node *reverse(Node *head) {
    Node *prev = NULL, *current = head, *next = NULL;

    while (current != NULL) {
        next = current->next;
        current->next = prev;
        prev = current;
        current = next;
    }

    return prev;
}

// Utility function to print a linked list
void print_list(Node *head) {
    while (head != NULL) {
        printf("%d -> ", head->data);
        head = head->next;
    }
    printf("NULL\n");
}

// Test function for reverse
void test_reverse_linked_list() {
    Node *head = malloc(sizeof(Node));
    head->data = 1;
    head->next = malloc(sizeof(Node));
    head->next->data = 2;
    head->next->next = malloc(sizeof(Node));
    head->next->next->data = 3;
    head->next->next->next = NULL;

    printf("Original Linked List:\n");
    print_list(head);

    head = reverse(head);
    printf("Reversed Linked List:\n");
    print_list(head);
}