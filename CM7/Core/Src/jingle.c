/*
 * jingle.c
 */

#include "main.h"
#include "jingle.h"

// Define the melody, durations, and pause intervals

int melody[] = {
	131, 165, 196, 247, 262, 247, 196, 165, 131, 165, 196, 247, 262,
	247, 196, 165, 131, 165, 196, 247, 262, 247, 196, 165, 131, 165,
	196, 247, 262, 247, 196, 165, 131, 165, 196, 247, 262, 247, 196,
	165, 131, 165, 196, 247, 262, 247, 196, 165
};

int durations[] = {
	188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188,
	188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188,
	188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188,
	188, 188, 188, 188, 188, 188, 188, 188, 188
};

int pause[] = {
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
    50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50
};

size_t melody_size = sizeof(melody) / sizeof(melody[0]);

// Helper function to calculate the prescaler for the given frequency

static int presForFrequency(int frequency)
{
	if (frequency == 0) return 0;
    return ((TIM_FREQ / (1000 * frequency)) - 1);  // Calculate the prescaler based on the frequency
}

// Function to stop playing the tone (stop PWM)

void noTone(void)
{
    __HAL_TIM_SET_PRESCALER(&htim1, 0);  // Set prescaler to 0 to stop the PWM
}

// Function to play the melody

void playTone(int *tone, int *duration, int *pause, size_t size)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Start PWM for the buzzer

    for (int i = 0; i < size; i++)
    {
        int prescaler = presForFrequency(tone[i]);
        int dur = duration[i];
        int pauseBetweenTones = pause ? pause[i] : 0;

        // Set the prescaler to adjust the frequency

        __HAL_TIM_SET_PRESCALER(&htim1, prescaler);

        // Play the tone for the specified duration

        HAL_Delay(dur);

        // Stop the tone

        noTone();

        // Delay between tones

        HAL_Delay(pauseBetweenTones);
    }
}
