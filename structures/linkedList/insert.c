void insert_into_linked_list(Node **head, int pos, int value) {
    Node *new_node = malloc(sizeof(Node));
    new_node->data = value;
    new_node->next = NULL;

    if (pos == 0) {
        new_node->next = *head;
        *head = new_node;
        return;
    }

    Node *current = *head;
    for (int i = 0; i < pos - 1 && current != NULL; ++i) {
        current = current->next;
    }

    if (current == NULL) {
        printf("Position out of bounds.\n");
        free(new_node);
        return;
    }

    new_node->next = current->next;
    current->next = new_node;
}

// Test function for linked list insertion
void test_linked_list_insertion() {
    Node *head = NULL;

    insert_into_linked_list(&head, 0, 1);
    insert_into_linked_list(&head, 1, 3);
    insert_into_linked_list(&head, 1, 2);

    printf("Linked List After Insertion: ");
    print_list(head);
}