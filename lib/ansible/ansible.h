#ifndef _ANSIBLE_H
#define _ANSIBLE_H

#include <stdint.h>

#define I2C_ROLE_UNKNOWN            0x00
#define I2C_ROLE_LEADER_NODE        0x01
#define I2C_ROLE_COHORT_NODE        0x02

#define I2C_MAX_ADDRESS             0xA1
#define I2C_DEFAULT_ADDRESS         ( I2C_MAX_ADDRESS - 1 )

#define NODE_PROP_I2C_ADDRESS       0x01
#define NODE_PROP_DISPLAY_DATA      0x02
#define NODE_PROP_TRANSITION_TYPE   0x03
#define NODE_PROP_NEXT_NODE_ADDR    0x04
#define NODE_PROP_PREV_NODE_ADDR    0x05

typedef struct {
    uint8_t i2c_address;            // Node I2C address
    uint8_t node_index;             // Absolute (physical) position
    uint8_t display_data;           // What the node is assigned to display
    uint8_t transition_type;        // Transition type (if any)
    node_state_t* next_node;        // Pointer to the next node's state
} node_state_t;

void ansible_enumerate();
node_state_t ansible_get_state( uint8_t node_address );
node_state_t ansible_set_state( node_state_t node_state );

#endif // _ANSIBLE_H