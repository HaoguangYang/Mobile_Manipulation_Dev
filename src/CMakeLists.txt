set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MOBILE_MANIP_BINARY_DIR})

ADD_EXECUTABLE (vehicle main.cc caster.cc vehicle.cc motor.c OTG.cc pose_t265.cc CAN_utils.c CO_message.c RT_utils.c)
# ADD_EXECUTABLE (calibrate calibrate.cc caster.cc vehicle.cc motor.c CAN_utils.c CO_message.c RT_utils.c)
# ADD_EXECUTABLE (canDump_motor canDump_motor.cc caster.cc vehicle.cc motor.c CAN_utils.c CO_message.c RT_utils.c)
# ADD_EXECUTABLE (test main.cc caster.cc vehicle.cc motor.c CAN_utils.c CO_message.c RT_utils.c)
ADD_EXECUTABLE (joystick joystick.c caster.cc vehicle.cc motor.c CAN_utils.c CO_message.c RT_utils.c)


TARGET_LINK_LIBRARIES (vehicle ${OTG_LIBRARY} ${LIB_REALSENSE_LIBRARIES})
