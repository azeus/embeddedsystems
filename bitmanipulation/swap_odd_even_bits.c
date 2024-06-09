#include <stdio.h>
#include <stdint.h>

// Function to swap odd and even bits
uint32_t swap_odd_even(unit32_t number){
  // Masks for odd and even bits
  uint32_t odd_mask = 0xAAAAAAAA;
  uint32_t even_mask = 0x55555555;

  // Get odd and even bits from the number using the mask
  uint32_t odd_bits = number & odd_mask;
  uint32_t even_bits = number & even_mask;

  // Shift the odd and even bits of the number by 1(right and left respectively) for swapping
  uint32_t shifted_odd_bits = odd_bits >> 1;
  uint32_t shifted_even_bits = even_bits << 1;

  // Combine the two to get the number with swapped even and Odd bits
  uint32_t swapped_number = shifted_odd_bits | shifted_even_bits;

  return swapped_number;
}

int main(){

  uint32_t number = 0x35;
  printf("the number to be swapped is: %u \n", number);
  uint32_t swapped_number = swap_odd_even(number);
  printf("the swapped number is: %u \n", swapped_number);

}
