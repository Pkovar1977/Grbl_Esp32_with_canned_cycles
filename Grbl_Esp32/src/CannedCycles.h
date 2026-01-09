#pragma once

#include "Grbl.h"

// Execute canned cycles. Returns GCUpdatePos to indicate how the parser should update
// the parser position after execution (Target/System/None).
GCUpdatePos canned_cycle_execute(parser_block_t* gc_block, plan_line_data_t* pl_data);
