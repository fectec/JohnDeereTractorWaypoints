/*
 * CANRoutines.h
 */

#ifndef INC_CANROUTINES_H_
#define INC_CANROUTINES_H_

// Extern variables

extern FDCAN_HandleTypeDef hfdcan1;

// Function prototypes

uint16_t CAN_ReceiveEncoder(FDCAN_RxHeaderTypeDef *RxHeader);

#endif /* INC_CANROUTINES_H_ */
