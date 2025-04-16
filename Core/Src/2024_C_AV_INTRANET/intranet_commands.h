//
// Created by marin on 08.03.2025.
//

/**
 * The communication between microcontrollers uses I2C protocol.
 * The following enums define the I2C buffer registers for each slave.
 * During a transaction, these addresses will be specified after the slave own address.
 * To simplify things, each transaction involves 4 bytes of data (32-bit)
 * 
 * Intranet reference: https://docs.google.com/spreadsheets/d/159JLAAPnOQsqaWHLeamU7fWXOPAsgL0OkVv0sn9sXFg/edit?gid=0#gid=0
 */

#ifndef AV_INTRANET_COMMANDS_H
#define AV_INTRANET_COMMANDS_H

/* Slaves addresses */
#define NET_ADDR_DPR_ETH 0x0E
#define NET_ADDR_DPR_LOX 0x0C
#define NET_ADDR_PRB     0x0F
#define NET_ADDR_TRB     0x0A

/* Engine valves map */
#define PRB_VO_NOC (1 << 5)
#define PRB_VE_NO  (1 << 4)  
#define PRB_IO_NCC (1 << 3)
#define PRB_IE_NC  (1 << 2)
#define PRB_MO_BC  (1 << 1)
#define PRB_ME_B   (1 << 0)

/* Pyro channels map */
#define TRB_PYRO1 (0xFF << 0)
#define TRB_PYRO2 (0xFF << 8)
#define TRB_PYRO3 (0xFF << 16) 

/* For boolean data type, such as WAKE_UP */
#define NET_CMD_ON  0xAC  // Activate
#define NET_CMD_OFF 0xDE  // Deactivate

/* Each transfer involves 4-bytes of data */
#define NET_XFER_SIZE 4

/* Havoc DPR */
enum NET_REG_DPR {
    DPR_TIMESTAMP_MAIN = 0x00, // -W
    DPR_WAKE_UP        = 0x01, // -W
    DPR_IS_WOKEN_UP    = 0x02, // R-
    DPR_PRESSURIZE     = 0x03, // -W
    DPR_ABORT          = 0x04, // -W
    DPR_PRESSURE       = 0x05, // R-
    DPR_DN_NC          = 0x06, // R-
    DPR_NB_REG
};

/* Prop Board */
enum NET_REG_PRB {
    PRB_TIMESTAMP_MAIN  = 0x00, // -W
    PRB_WAKE_UP         = 0x01, // -W
    PRB_IS_WOKEN_UP     = 0x02, // R-
    PRB_CLEAR_TO_IGNITE = 0x03, // -W
    PRB_FSM_PRB         = 0x04, // R-
    PRB_P_OIN           = 0x05, // R-
    PRB_T_OIN           = 0x06, // R-
    PRB_P_EIN           = 0x07, // R-
    PRB_T_EIN           = 0x08, // R-
    PRB_P_CCC           = 0x09, // R-
    PRB_T_CCC           = 0x0A, // R-
    PRB_P_CIG           = 0x0B, // R-
    PRB_T_CIG           = 0x0C, // R-
    PRB_VALVES_STATE    = 0x0D, // R/W
    PRB_NB_REG
};

/* Trigger Board */
enum NET_REG_TRB {
    TRB_TIMESTAMP_MAIN   = 0x00, // -W
    TRB_WAKE_UP          = 0x01, // -W
    TRB_IS_WOKEN_UP      = 0x02, // R-
    TRB_CLEAR_TO_TRIGGER = 0x03, // -W
    TRB_PYROS            = 0x04, // R/W
    TRB_HAS_TRIGGERED    = 0x05, // R-
    TRB_NB_REG
};

/* Cameras */
// enum NET_REG_CAM {
//     TIMESTAMP_MAIN = 0x00, // -W
//     WAKE_UP        = 0x01, // -W
//     IS_WOKEN_UP    = 0x02, // R-
//     RECORDING      = 0x03  // R/W
// };

#endif /* AV_INTRANET_COMMANDS_H */
