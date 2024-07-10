#include "aditional.hpp"



int fractional (double a) {
    return int((a-floor(a))*1000000);
}

void convert(int x, uint8_t *data) {
    // data[0] = (uint8_t)(x >> 0);
    // data[1] = (uint8_t)(x >> 8);
    // data[2] = (uint8_t)(x >> 16);
    //data[3] = (uint8_t)(x >> 24);
    data[0] = x/10000;
    data[1]= (x%10000)/100;
    data[2] =x%100;
    //123456
}