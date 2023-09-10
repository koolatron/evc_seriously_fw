#include "ansible.h"

node_t* add_node(void) {
    node_t* new_node = (node_t*) malloc(sizeof(node_t));
    return new_node;
}

uint8_t last_node_index(node_t* root) {
    uint8_t last_index;
    node_t* node;

    node = root;
    last_index = root->node_index;

    while (node->next_node != NULL) {
        last_index++;
        node = node->next_node;
    }

    return last_index;
}