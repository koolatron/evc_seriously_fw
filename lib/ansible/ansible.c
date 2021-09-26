#include "ansible.h"

node_t* add_node(void) {
    node_t* new_node = (node_t*) malloc(sizeof(node_t));
    return new_node;
}