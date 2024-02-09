/*
this is a small module for handling 4-byte floats that by far take unnecessary ram space, coming from cases such as temperature, humidity and pressure measurements where the sensor accuracy doesn't quite justify the capabilities of fp32 numbers
*/

#include <stdio.h>



