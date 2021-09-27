#ifndef _ANSIBLE_H
#define _ANSIBLE_H

#include <stdint.h>
#include <stdlib.h>

#define I2C_MAX_ADDRESS             0x7F
#define I2C_LEADER_ADDRESS          0x0A
#define I2C_DEFAULT_ADDRESS         ( I2C_MAX_ADDRESS - 1 )

#define I2C_TRANSFER_NACK           0x20
#define I2C_TRANSFER_STOP           0x30

#define I2C_CMD_SET_ADDR            0xaa
#define I2C_CMD_SET_INDEX           0x02
#define I2C_CMD_GET_INDEX           0x03
#define I2C_CMD_SET_TRANSITION_TYPE 0x04
#define I2C_CMD_GET_TRANSITION_TYPE 0x05
#define I2C_CMD_SET_DISPLAY_DATA    0x06
#define I2C_CMD_GET_DISPLAY_DATA    0x07
#define I2C_CMD_SET_SIGNAL          0x08
#define I2C_CMD_GET_SIGNAL          0x09

#define NODE_PROP_I2C_ADDRESS       0x01
#define NODE_PROP_DISPLAY_DATA      0x02
#define NODE_PROP_TRANSITION_TYPE   0x03
#define NODE_PROP_NEXT_NODE_ADDR    0x04
#define NODE_PROP_PREV_NODE_ADDR    0x05

typedef struct node {
    uint8_t i2c_address;            // Node I2C address
    uint8_t node_index;             // Absolute (physical) position
    uint8_t display_data;           // What the node is assigned to display
    uint8_t transition_type;        // Transition type (if any)
    struct node* next_node;         // Pointer to the next node
} node_t;

node_t* add_node(void);

#endif // _ANSIBLE_H