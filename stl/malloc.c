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