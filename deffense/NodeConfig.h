// Compile-time role selector for the flat Arduino sketch.
// Arduino builds every .ino tab, so only the selected role defines setup/loop.
#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

// Stable numeric values used by the conditional blocks in A.ino, B.ino, C.ino.
#define NODE_A 1
#define NODE_B 2
#define NODE_C 3

// Select exactly one firmware before Verify/Upload in Arduino IDE.
#ifndef ACTIVE_NODE
#define ACTIVE_NODE NODE_A
#endif

#if ACTIVE_NODE < NODE_A || ACTIVE_NODE > NODE_C
#error "ACTIVE_NODE must be NODE_A, NODE_B or NODE_C"
#endif

#endif
