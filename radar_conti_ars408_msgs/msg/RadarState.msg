#  ------------------------------------------------------------------------
#
#  Name:       RadarState
#  Id:         0x201
#  Length:     8 bytes
#  Cycle time: 50 ms
#  Senders:    ARS_ISF
#  Layout:
#
#                          Bit
#
#             7   6   5   4   3   2   1   0
#           +---+---+---+---+---+---+---+---+
#         0 |<-x|<-x|   |   |   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#             |   +-- RadarState_NVMReadStatus
#             +-- RadarState_NVMwriteStatus
#           +---+---+---+---+---+---+---+---+
#         1 |<------------------------------|
#           +---+---+---+---+---+---+---+---+
#         2 |------x|<-x|<-x|<-x|<-x|<-x|   |
#           +---+---+---+---+---+---+---+---+
#                 |   |   |   |   |   +-- RadarState_Voltage_Error
#                 |   |   |   |   +-- RadarState_Temporary_Error
#                 |   |   |   +-- RadarState_Temperature_Error
#                 |   |   +-- RadarState_Interference
#                 |   +-- RadarState_Persistent_Error
#                 +-- RadarState_MaxDistanceCfg
#           +---+---+---+---+---+---+---+---+
#     B   3 |   |   |   |   |   |   |<------|
#     y     +---+---+---+---+---+---+---+---+
#     t   4 |--x|<---------x|   |<---------x|
#     e     +---+---+---+---+---+---+---+---+
#             |           |               +-- RadarState_SensorID
#             |           +-- RadarState_SortIndex
#             +-- RadarState_RadarPowerCfg
#           +---+---+---+---+---+---+---+---+
#         5 |<-----x|<-x|<-x|<-----x|<-x|   |
#           +---+---+---+---+---+---+---+---+
#                 |   |   |       |   +-- RadarState_CtrlRelayCfg
#                 |   |   |       +-- RadarState_OutputTypeCfg
#                 |   |   +-- RadarState_SendQualityCfg
#                 |   +-- RadarState_SendExtInfoCfg
#                 +-- RadarState_MotionRxState
#           +---+---+---+---+---+---+---+---+
#         6 |   |   |   |   |   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#         7 |   |   |   |<---------x|   |   |
#           +---+---+---+---+---+---+---+---+
#                                 +-- RadarState_RCS_Threshold
#
#  Signal tree:
#
#    -- {root}
#       +-- RadarState_NVMwriteStatus
#       +-- RadarState_NVMReadStatus
#       +-- RadarState_MaxDistanceCfg
#       +-- RadarState_Persistent_Error
#       +-- RadarState_Interference
#       +-- RadarState_Temperature_Error
#       +-- RadarState_Temporary_Error
#       +-- RadarState_Voltage_Error
#       +-- RadarState_RadarPowerCfg
#       +-- RadarState_SortIndex
#       +-- RadarState_SensorID
#       +-- RadarState_MotionRxState
#       +-- RadarState_SendExtInfoCfg
#       +-- RadarState_SendQualityCfg
#       +-- RadarState_OutputTypeCfg
#       +-- RadarState_CtrlRelayCfg
#       +-- RadarState_RCS_Threshold
#
#  ------------------------------------------------------------------------
#
std_msgs/Header header
uint8 nvmwritestatus
uint8 nvmreadstatus
uint8 maxdistancecfg
uint8 persistent_error
uint8 interference
uint8 temperature_error
uint8 temporary_error
uint8 voltage_error
uint8 radarpowercfg
uint8 sortindex
uint8 sensorid
uint8 motionrxstate
uint8 sendextinfocfg
uint8 sendqualitycfg
uint8 outputtypecfg
uint8 ctrlrelaycfg
uint8 rcs_threshold