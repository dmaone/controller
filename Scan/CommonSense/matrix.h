/* Copyright (C) 2014-2015 by Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

// ----- Includes -----

// Project Includes
#include <matrix_setup.h>


// ----- Matrix Definition -----

// Freescale ARM MK20's support GPIO PTA, PTB, PTC, PTD and PTE 0..31
// Not all chips have access to all of these pins (most don't have 160 pins :P)
//
// NOTE:
// Before using a pin, make sure it supports being a GPIO *and* doesn't have a default pull-up/pull-down
// Checking this is completely on the ownness of the user

// Teensy3.2
// Teensy layout is STUPID. Avoid using it if you can.
//
// Rows (Strobe)
//  3 2 7 8 0 1 11 12
// PTA12 PTD0 PTD2 PTD3 PTC3 PTC4 PTC6 PTC7
//
// Columns (Sense)
// 23 22 21 20 19 18 17 16
// PTC2 PTC1 PTD6  PTD5  PTB2 PTB3 PTB1 PTB0
//
// 15 14 29 30 28 27 5(wired to A12) 4(wired to A10)
// PTC0 PTD1 PTC10 PTC11 PTC8 PTC9 PTD7 PTA13

// Define Columns (Strobes) and Rows (Sense)
// Note that matrix is transposed, because kiibohd assumes rows are sense pins.
GPIO_Pin Matrix_rows[] = { 
	gpio(C,2), gpio(C,1), gpio(D,6), gpio(D,5),
	gpio(B,2), gpio(B,3), gpio(B,1), gpio(B,0),
	gpio(C,0), gpio(D,1), gpio(C,10), gpio(C,11),
	gpio(C,8), gpio(C,9), gpio(D,7), gpio(A,13)
};

GPIO_Pin Matrix_cols[] = {
	gpio(A,12), gpio(D,0), gpio(D,2), gpio(D,3),
	gpio(C,3), gpio(C, 4), gpio(C,6), gpio(C,7)
};

Config Matrix_type = Config_Pulldown;
#define COMMONSENSE

// vim:ts=8:sts=8:sw=8:noet
