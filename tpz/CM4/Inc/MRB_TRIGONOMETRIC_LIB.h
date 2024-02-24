/*
 * MRB_TRIG_LIB.h
 *
 *  Created on: Feb 17, 2024
 *      Author: Maciek
 */

#ifndef INC_MRB_TRIGONOMETRIC_LIB_H_
#define INC_MRB_TRIGONOMETRIC_LIB_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MRB_TL_PI 3.1415926
#define MRB_TL_PI_BY_2 1.5707963
#define MRB_TL_1_BY_2PI 0.1591549
#define MRB_TL_PI_BY_3 1.0471975512
#define MRB_TL_LUT_LENGTH 4000



float sin_f(float arg);
float cos_f(float arg);
float asin_f(float arg);

#endif /* INC_MRB_TRIGONOMETRIC_LIB_H_ */
