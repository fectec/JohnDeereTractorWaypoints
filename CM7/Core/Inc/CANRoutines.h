/*
 * CANRoutines.h
 */

#ifndef INC_CANROUTINES_H_
#define INC_CANROUTINES_H_

extern FDCAN_HandleTypeDef hfdcan1;

// Function prototypes

int CAN_ReceiveEncoder(FDCAN_RxHeaderTypeDef *RxHeader);

#endif /* INC_CANROUTINES_H_ */
