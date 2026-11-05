/**
 * @file barcode_code39.h
 * @brief Decode Code 39 from element widths, in either scan direction.
 *
 * A Code 39 character is 9 elements (bar, space, ... bar), 3 of them
 * wide, followed by a narrow gap before the next character. A symbol is
 * '*', the text, '*'. The module is pure: no kernel calls.
 */

#ifndef BARCODE_CODE39_H
#define BARCODE_CODE39_H

/* Public includes. */
#include <stdbool.h>
#include <stdint.h>

/* Public types, constants, and macros. */

/** Elements per character, without the gap after it. */
#define BARCODE_CODE39_ELEMENTS 9u

/** @brief Decode result. */
typedef enum
{
    BARCODE_CODE39_OK = 0,
    BARCODE_CODE39_ERR_LENGTH,  /**< Not 10 x characters - 1 elements, or
                                     fewer than 3 characters. */
    BARCODE_CODE39_ERR_RATIO,   /**< Wide and narrow not clearly apart. */
    BARCODE_CODE39_ERR_PATTERN, /**< Not a Code 39 character. */
    BARCODE_CODE39_ERR_FRAME,   /**< No '*' at both ends, or '*' inside. */
    BARCODE_CODE39_ERR_SPACE    /**< Output buffer too small. */
} barcode_code39_status_t;

/* Public function declarations. */

/**
 * @brief Decode one scan.
 * @param p_widths Element widths in any unit (us), first bar first, as
 * scanned; the quiet zones are not included.
 * @param count Number of widths.
 * @param p_text Receives the text between the '*'s, NUL-terminated.
 * @param text_size Size of @p p_text in bytes.
 * @param p_reversed Set true when the scan ran backwards over the symbol.
 * @return BARCODE_CODE39_OK, or why the forward reading failed (the
 * reversed reading's reason when the forward one had no '*' frame).
 */
barcode_code39_status_t barcode_code39_decode (uint32_t const * p_widths,
                                               uint32_t count, char * p_text,
                                               uint32_t text_size,
                                               bool * p_reversed);

/**
 * @brief Wide/narrow pattern of @p symbol, for tests: bit 8 is the first
 * element, 1 is wide.
 * @return The 9-bit pattern, or 0 for a character Code 39 lacks.
 */
uint32_t barcode_code39_pattern (char symbol);

/** @brief Short name of @p status for logs. */
char const * barcode_code39_status_name (barcode_code39_status_t status);

#endif /* BARCODE_CODE39_H */
