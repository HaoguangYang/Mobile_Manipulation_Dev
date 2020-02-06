#ifndef CO_OBJECTS_H
#define CO_OBJECTS_H

/* Defintions for CANopen object indices */

#define COMM_CYCLE_PERIOD			(0x1006)
#define PROD_HEARTBEAT_TIME			(0x1017)
#define RPDO1_COMM					(0x1400)
#define RPDO2_COMM					(0x1401)
#define RPDO3_COMM					(0x1402)
#define RPDO4_COMM					(0x1403)
#define RPDO1_MAPPING				(0x1600)
#define RPDO2_MAPPING				(0x1601)
#define RPDO3_MAPPING				(0x1602)
#define RPDO4_MAPPING				(0x1603)
#define TPDO1_COMM					(0x1800)
#define TPDO2_COMM					(0x1801)
#define TPDO3_COMM					(0x1802)
#define TPDO4_COMM					(0x1803)
#define TPDO1_MAPPING				(0x1A00)
#define TPDO2_MAPPING				(0x1A01)
#define TPDO3_MAPPING				(0x1A02)
#define TPDO4_MAPPING				(0x1A03)
#define EXT_ONLINE_REF				(0x201C)
#define EXT_REF_TYPE				(0x201D)
#define CONTROL_WORD				(0x6040)
#define STATUS_WORD 				(0x6041)
#define MODES_OF_OPERATION			(0x6060)
#define HOME_OFFSET					(0x607C)
#define POS_FACTOR          		(0x6093)
#define VEL_NOTATION_IND			(0x608B)
#define VEL_DIMENSION_IND			(0x608C)
#define VEL_ENCODER_FACTOR 			(0x6094)
#define ACCEL_FACTOR        		(0x6097)
#define HOMING_METHOD				(0x6098)
#define HOMING_SPEEDS 				(0x6099)
#define HOMING_ACCEL				(0x609A)

#endif