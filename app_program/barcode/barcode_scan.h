/**
 * @file barcode_scan.h
 * @brief Turn timestamped barcode-sensor edges into one scan's widths.
 *
 * A scan starts at the first black edge after white and collects each
 * element's width (time between edges). A white gap longer than
 * BARCODE_QUIET_MAX_US ends it; so does a black longer than that (bars
 * merged, the line, or the car stopped on a bar), at the bar before it.
 * The module is pure: no kernel calls.
 */

#ifndef BARCODE_SCAN_H
#define BARCODE_SCAN_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>
#include "barcode_config.h"

/* Public types, constants, and macros. */

/** @brief What a call found. */
typedef enum
{
    BARCODE_SCAN_BUSY = 0, /**< Nothing to decode yet. */
    BARCODE_SCAN_DONE      /**< widths[0..count-1] hold a finished scan;
                                decode it, then barcode_scan_reset(). */
} barcode_scan_result_t;

/** @brief Chatter filter state; see barcode_filter_edge(). */
typedef struct
{
    uint32_t last_us; /**< Time of the last edge, chatter included. */
    bool b_settled;   /**< Level read at that edge. */
} barcode_filter_t;

/** @brief Scan state; callers allocate it and read widths/count. */
typedef struct
{
    uint32_t widths[BARCODE_MAX_ELEMENTS]; /**< Element widths, in us. */
    uint32_t count;                        /**< Widths collected. */
    bool b_active;                         /**< Inside a scan. */
    bool b_black;                          /**< Level since last_us. */
    bool b_overflow;                       /**< Too many elements. */
    bool b_long_black;                     /**< Ended by an over-long black. */
    uint32_t last_us;                      /**< Time of the last edge. */
    uint32_t narrowest_us;                 /**< Narrowest width so far. */
} barcode_scan_t;

/* Public function declarations. */

/**
 * @brief Start the chatter filter at the current level.
 * @param p_filter State to set.
 * @param b_black Current level.
 * @param now_us Current time.
 */
void barcode_filter_init (barcode_filter_t * p_filter, bool b_black,
                          uint32_t now_us);

/**
 * @brief Pass one sensor edge through the chatter filter.
 * An edge after at least BARCODE_MIN_PULSE_US of quiet starts a real
 * transition: the level flips from the one last settled, because a level
 * read during chatter cannot be trusted. Edges closer together than that
 * are the same transition.
 * @param p_filter Filter state.
 * @param edge_us Time of the edge.
 * @param b_black Level read at the edge.
 * @param p_level Receives the new level when the edge is a transition.
 * @return True for a transition; false for chatter.
 * Runs in the edge interrupt; no kernel calls.
 */
bool barcode_filter_edge (barcode_filter_t * p_filter, uint32_t edge_us,
                          bool b_black, bool * p_level);

/**
 * @brief Between scans, take the sensor's level as the current one.
 * A scan ended on a long black can leave a stale level, and the next
 * barcode's first bar would then look like no change. Does nothing during
 * a scan.
 * @param p_scan Scan state.
 * @param b_black Sensor level now.
 */
void barcode_scan_sync (barcode_scan_t * p_scan, bool b_black);

/**
 * @brief Start idle at level @p b_black.
 * @param p_scan State to reset.
 * @param b_black Current level.
 * @param now_us Current time.
 */
void barcode_scan_init (barcode_scan_t * p_scan, bool b_black, uint32_t now_us);

/**
 * @brief Drop a finished or abandoned scan; keep the level and time.
 * @param p_scan State.
 */
void barcode_scan_reset (barcode_scan_t * p_scan);

/**
 * @brief Take one edge.
 * @param p_scan State.
 * @param edge_us Time of the edge.
 * @param b_black Level after it. The same level as before is a glitch
 *        (two edges merged) and is ignored.
 * @return BARCODE_SCAN_DONE if the white or black before this edge ended
 *         a scan; the edge was NOT taken: decode, reset, then pass it again.
 */
barcode_scan_result_t barcode_scan_edge (barcode_scan_t * p_scan,
                                         uint32_t edge_us, bool b_black);

/**
 * @brief Check the time since the last edge.
 * @param p_scan State.
 * @param now_us Current time. An edge already taken that is newer than
 *        it is not an error: the call then waits.
 * @return BARCODE_SCAN_DONE once the trailing white is a quiet zone, or
 *         the trailing black is too long for a bar.
 */
barcode_scan_result_t barcode_scan_poll (barcode_scan_t * p_scan,
                                         uint32_t now_us);

#endif /* BARCODE_SCAN_H */
