#include "CannedCycles.h"
#include "MotionControl.h"
#include "Limits.h"
#include "Protocol.h"

GCUpdatePos canned_cycle_execute(parser_block_t* gc_block, plan_line_data_t* pl_data) {
    // ====== MODAL STATE TRACKING ======
    static struct {
        bool active;
        Motion cycle_type;
        float original_R;
        float original_Z;
        float original_Q;
    } modal_state = {false, Motion::None, 0.0f, 0.0f, 0.1f};  // ← Q=0.1mm default!
    
// Reset on G80
if (gc_block->modal.motion == Motion::None) {
    modal_state.active = false;
    modal_state.cycle_type = Motion::None;
    modal_state.original_R = 0.0f;
    modal_state.original_Z = 0.0f;
    modal_state.original_Q = 0.0f;  // Reset to 0
    // DON'T reset g73g83_q_initialized here! It stays true after power cycle
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Modal cycle CANCELLED (G80)");
    return GCUpdatePos::None;
}
    
    // Check if this is a canned cycle
    if (gc_block->modal.motion != Motion::G81 && gc_block->modal.motion != Motion::G82 &&
        gc_block->modal.motion != Motion::G83 && gc_block->modal.motion != Motion::G73) {
        return GCUpdatePos::None;
    }

    // Ensure planner buffer is clear and parser/planner positions are synchronized to real machine.
    protocol_buffer_synchronize();

    float target[MAX_N_AXIS];
    // Use a local position buffer to track commanded position immediately (don't rely on gc_state.position updating).
    float localPos[MAX_N_AXIS];
    memcpy(localPos, gc_state.position, sizeof(localPos));

    // For canned cycles, X/Y may be specified on the line to select the hole position.
    // If provided in the block use those values; otherwise use current position.
    target[X_AXIS] = (gc_block->values.xyz[X_AXIS] != 0.0f) ? gc_block->values.xyz[X_AXIS] : localPos[X_AXIS];
    target[Y_AXIS] = (gc_block->values.xyz[Y_AXIS] != 0.0f) ? gc_block->values.xyz[Y_AXIS] : localPos[Y_AXIS];
    target[Z_AXIS] = gc_block->values.xyz[Z_AXIS];

    // ======== Compute work coordinates for Z-related values =========
    float coord_sys = gc_state.coord_system[Z_AXIS];
    float coord_off = gc_state.coord_offset[Z_AXIS];
    float tool_off = gc_state.tool_length_offset;

    // DEBUG: Show raw input values
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                   "CYCLE RAW: block.Z=%.3f block.R=%.3f block.Q=%.3f",
                   gc_block->values.xyz[Z_AXIS], gc_block->values.r, gc_block->values.q);

// ====== MODAL LOGIC ======
// Track if G73/G83 Q has been initialized after power cycle
static bool g73g83_q_initialized = false;

if (!modal_state.active || modal_state.cycle_type != gc_block->modal.motion) {
    // New or different cycle type
    modal_state.active = true;
    modal_state.cycle_type = gc_block->modal.motion;
    modal_state.original_R = gc_block->values.r;
    modal_state.original_Z = gc_block->values.xyz[Z_AXIS] - coord_sys;
    
    // ====== CRITICAL Q PRESET FOR G73/G83 ======
    if (modal_state.cycle_type == Motion::G73 || modal_state.cycle_type == Motion::G83) {
        if (gc_block->values.q <= 0.0f && !g73g83_q_initialized) {
            // First G73/G83 after power cycle with no Q - PRESET!
            modal_state.original_Q = 0.1f;  // Safe default peck depth
            g73g83_q_initialized = true;    // Mark as initialized
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                           "G73/G83: First after power, preset Q=0.1mm");
        } else if (gc_block->values.q > 0.0f) {
            // G73/G83 with explicit Q
            modal_state.original_Q = gc_block->values.q;
            g73g83_q_initialized = true;    // Also mark as initialized
        } else {
            // G73/G83 with Q=0 but already initialized (shouldn't happen)
            modal_state.original_Q = 0.1f;  // Fallback
        }
    } else {
        // G81/G82 - just use whatever Q (will be 0)
        modal_state.original_Q = gc_block->values.q;
    }
    
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                   "Modal START: Type=%d Z=%.3f R=%.3f Q=%.3f",
                   (int)modal_state.cycle_type, 
                   modal_state.original_Z, 
                   modal_state.original_R, 
                   modal_state.original_Q);
}
    // Always use stored modal values
    float finalZ_work = modal_state.original_Z;
    float R_work = modal_state.original_R;  
    float q = modal_state.original_Q;

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                   "Using modal: Z=%.3f R=%.3f Q=%.3f",
                   finalZ_work, R_work, q);

    // Current Z position converted to work coordinates
    float startZ_work = gc_state.position[Z_AXIS] - coord_sys - coord_off - tool_off;

    // DEBUG: Verify calculations
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                   "WORK COORDS: R=%.3f finalZ=%.3f startZ=%.3f (coord_sys=%.3f)",
                   R_work, finalZ_work, startZ_work, coord_sys);

    // ============= Move XY to position at current Z (rapid) =============
    float move[MAX_N_AXIS];
    memcpy(move, localPos, sizeof(move));    // Copy current position
    move[X_AXIS] = target[X_AXIS];           // Set target X (from block)
    move[Y_AXIS] = target[Y_AXIS];           // Set target Y (from block)
    // Z remains at current height for this XY move

    pl_data->motion.rapidMotion = 1;         // Set rapid motion flag
    limitsCheckSoft(move);                   // Check soft limits

    // Execute the XY move
    if (!cartesian_to_motors(move, pl_data, localPos)) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Canned cycle XY move failed");
        return GCUpdatePos::None;            // Motion cancelled or failed
    }

    // Update local position tracker
    memcpy(localPos, move, sizeof(localPos));

    // =========== Rapid to R plane (retract plane) ============
    // Rapid move to R plane (retract plane)
    memcpy(move, localPos, sizeof(move));    // Start from current XY position

    // Convert R_work (work coordinates) to machine coordinates for motion
    move[Z_AXIS] = R_work + coord_sys + coord_off + tool_off;

    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, 
                   "Rapid to R: work=%.3f -> machine=%.3f", 
                   R_work, move[Z_AXIS]);

    pl_data->motion.rapidMotion = 1;         // Rapid motion
    limitsCheckSoft(move);                   // Check limits

    if (!cartesian_to_motors(move, pl_data, localPos)) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Canned cycle R plane move failed");
        return GCUpdatePos::None;
    }

    // Update position tracker
    memcpy(localPos, move, sizeof(localPos));

    // =========== Debug start ===========
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "CannedCycle start: R=%.4f finalZ=%.4f Q=%.4f startZ=%.4f", 
                   R_work, finalZ_work, q, startZ_work);
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                   "state: gc_state.position.Z=%.4f coord_system.Z=%.4f coord_offset.Z=%.4f tool_length_offset=%.4f",
                   gc_state.position[Z_AXIS],
                   gc_state.coord_system[Z_AXIS],
                   gc_state.coord_offset[Z_AXIS],
                   gc_state.tool_length_offset);

    // =========== Handle G81, G82 ===========
    if (gc_block->modal.motion == Motion::G81 || gc_block->modal.motion == Motion::G82) {
        // Feed to final Z
        memcpy(move, localPos, sizeof(move));
        move[Z_AXIS] = finalZ_work + coord_sys + coord_off + tool_off;
        pl_data->motion.rapidMotion = 0;
        pl_data->feed_rate = gc_block->values.f > 0.0f ? gc_block->values.f : pl_data->feed_rate;
        limitsCheckSoft(move);
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "feed plunge to Z=%.4f", finalZ_work);
        if (!cartesian_to_motors(move, pl_data, localPos)) {
            return GCUpdatePos::None;
        }
        memcpy(localPos, move, sizeof(localPos));

        // Dwell if G82
        if (gc_block->modal.motion == Motion::G82 && gc_block->values.p > 0.0f) {
            mc_dwell(int32_t(gc_block->values.p * 1000.0f));
        }

        // Retract to R
        memcpy(move, localPos, sizeof(move));
        move[Z_AXIS] = R_work + coord_sys + coord_off + tool_off;
        pl_data->motion.rapidMotion = 1;
        limitsCheckSoft(move);
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "final retract to R Z=%.4f (work %.4f)", move[Z_AXIS], R_work);
        if (!cartesian_to_motors(move, pl_data, localPos)) {
            return GCUpdatePos::None;
        }
        memcpy(localPos, move, sizeof(localPos));

    // =========== G73 - Chip-breaking Peck Cycle ===========
    } else if (gc_block->modal.motion == Motion::G73) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                       "G73 START: Chip-breaking peck R=%.3f Z=%.3f Q=%.3f",
                       R_work, finalZ_work, q);
        
        // G73 LOGIC - partial retracts (chip breaking)
        bool drilling_down = finalZ_work < R_work;
        float current_depth = R_work;
        
        while (true) {
            // Calculate next peck
            float next_target;
            if (drilling_down) {
                next_target = current_depth - q;
                if (next_target < finalZ_work) next_target = finalZ_work;
            } else {
                next_target = current_depth + q;
                if (next_target > finalZ_work) next_target = finalZ_work;
            }
            
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                           "G73: %.3f -> %.3f", current_depth, next_target);
            
            // Rapid to current depth (if needed)
            if (fabs(localPos[Z_AXIS] - (current_depth + coord_sys + coord_off + tool_off)) > 0.001f) {
                memcpy(move, localPos, sizeof(move));
                move[Z_AXIS] = current_depth + coord_sys + coord_off + tool_off;
                pl_data->motion.rapidMotion = 1;
                if (!cartesian_to_motors(move, pl_data, localPos)) break;
                memcpy(localPos, move, sizeof(localPos));
            }
            
            // Feed to next target
            memcpy(move, localPos, sizeof(move));
            move[Z_AXIS] = next_target + coord_sys + coord_off + tool_off;
            pl_data->motion.rapidMotion = 0;
            if (!cartesian_to_motors(move, pl_data, localPos)) break;
            memcpy(localPos, move, sizeof(localPos));
            
            current_depth = next_target;
            
            // Check if done
            if (fabs(current_depth - finalZ_work) < 0.001f) {
                // Final retract to R
                memcpy(move, localPos, sizeof(move));
                move[Z_AXIS] = R_work + coord_sys + coord_off + tool_off;
                pl_data->motion.rapidMotion = 1;
                cartesian_to_motors(move, pl_data, localPos);
                break;
            }
            
            // G73: PARTIAL RETRACT (1mm chip breaking) - stays in hole
            const float CHIP_BREAK = 1.0f;
            float retract_height;
            if (drilling_down) {
                retract_height = current_depth + CHIP_BREAK;
                if (retract_height > R_work) retract_height = R_work;
            } else {
                retract_height = current_depth - CHIP_BREAK;
                if (retract_height < R_work) retract_height = R_work;
            }
            
            memcpy(move, localPos, sizeof(move));
            move[Z_AXIS] = retract_height + coord_sys + coord_off + tool_off;
            pl_data->motion.rapidMotion = 1;
            if (!cartesian_to_motors(move, pl_data, localPos)) break;
            memcpy(localPos, move, sizeof(localPos));
        }

    // =========== G83 - Full-Retract Peck Cycle ===========
    } else if (gc_block->modal.motion == Motion::G83) {
        grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                       "G83 START: Full-retract peck R=%.3f Z=%.3f Q=%.3f",
                       R_work, finalZ_work, q);
        
        // G83 LOGIC - FULL RETRACTS to R plane between pecks
        bool drilling_down = finalZ_work < R_work;
        float current_depth = R_work;
        
        while (true) {
            // Calculate next peck
            float next_target;
            if (drilling_down) {
                next_target = current_depth - q;
                if (next_target < finalZ_work) next_target = finalZ_work;
            } else {
                next_target = current_depth + q;
                if (next_target > finalZ_work) next_target = finalZ_work;
            }
            
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info,
                           "G83: %.3f -> %.3f", current_depth, next_target);
            
            // Rapid to current depth (if needed) - for first peck, we're already at R
            if (fabs(localPos[Z_AXIS] - (current_depth + coord_sys + coord_off + tool_off)) > 0.001f) {
                memcpy(move, localPos, sizeof(move));
                move[Z_AXIS] = current_depth + coord_sys + coord_off + tool_off;
                pl_data->motion.rapidMotion = 1;
                if (!cartesian_to_motors(move, pl_data, localPos)) break;
                memcpy(localPos, move, sizeof(localPos));
            }
            
            // Feed to next target
            memcpy(move, localPos, sizeof(move));
            move[Z_AXIS] = next_target + coord_sys + coord_off + tool_off;
            pl_data->motion.rapidMotion = 0;
            if (!cartesian_to_motors(move, pl_data, localPos)) break;
            memcpy(localPos, move, sizeof(localPos));
            
            current_depth = next_target;
            
            // Check if done
            if (fabs(current_depth - finalZ_work) < 0.001f) {
                // Final retract to R
                memcpy(move, localPos, sizeof(move));
                move[Z_AXIS] = R_work + coord_sys + coord_off + tool_off;
                pl_data->motion.rapidMotion = 1;
                cartesian_to_motors(move, pl_data, localPos);
                break;
            }
            
            // G83: FULL RETRACT to R plane between pecks
            memcpy(move, localPos, sizeof(move));
            move[Z_AXIS] = R_work + coord_sys + coord_off + tool_off;
            pl_data->motion.rapidMotion = 1;
            if (!cartesian_to_motors(move, pl_data, localPos)) break;
            memcpy(localPos, move, sizeof(localPos));
        }
    }

    // Final planner sync to ensure motions complete before parser state update
    protocol_buffer_synchronize();

    // Parser should synchronize from the system (actual machine) positions
    return GCUpdatePos::System;
}