// Pattern detection in incoming bit stream
typedef struct {
    ShiftRegister sr;
    uint32_t pattern;       // Pattern to match
    uint32_t patternMask;   // Mask for valid pattern bits
} PatternDetector;

// Initialize pattern detector
PatternDetector initPatternDetector(uint8_t size, uint32_t pattern) {
    PatternDetector pd;
    pd.sr = initShiftRegister(size);
    pd.pattern = pattern;
    pd.patternMask = (1UL << size) - 1;
    return pd;
}

// Check if pattern is detected in current window
bool detectPattern(PatternDetector* pd) {
    return (pd->sr.data & pd->patternMask) == pd->pattern;
}