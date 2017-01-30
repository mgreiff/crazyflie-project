#include <stdio.h>

int sum_example(int num_numbers, int *numbers) {
    int i;
    int sum;
    for (i = 0; i < num_numbers; i++) {
        sum += numbers[i];
    }
    printf("Calculated sum in the C program sum.c: ");
    return sum;
}

