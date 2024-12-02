/*
 * jingle.h
 */

#ifndef INC_JINGLE_H_
#define INC_JINGLE_H_

// Extern variables

extern TIM_HandleTypeDef htim1;

extern int melody[];
extern int durations[];
extern int pause[];
extern size_t melody_size;

// Defines

#define TIM_FREQ 240000000  // Frequency of the timer's clock

// Function prototypes

void playTone(int *tone, int *duration, int *pause, size_t size);
void noTone(void);

#endif /* __JINGLE_H */
