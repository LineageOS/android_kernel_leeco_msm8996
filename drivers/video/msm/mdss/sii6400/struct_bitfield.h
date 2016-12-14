/*
  SiI6400 Linux Driver

  Copyright (C) 2012-2013 Silicon Image, Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation version 2.

  This program is distributed "AS-IS" WITHOUT ANY WARRANTY of any
  kind, whether express or implied; INCLUDING without the implied warranty
  of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
  See the GNU General Public License for more details at
  http://www.gnu.org/licenses/gpl-2.0.html.
*/

/*
 * Macros to operate on bitfields in packed structures.
 * Inspired by Marc Viredaz's open source "bitfield.h" package on Koders.com
 */

#ifndef STRUCT_BITFIELD_H
#define STRUCT_BITFIELD_H

#include <linux/types.h>

#define UDATA(Data) ((uint32_t)(Data))

/*
 * NOTE: Bitfield Size and Shift values may be 0-32 bits for ease of handling
 *       so full 32 bit words can used for any field.
 *       Offset value restricted to 8 bits (0-255) for ease in packing - may
 *       revisit later if needed.
 *
 *       Endian issues, and access (byte_offset+base vs ptr+word_offset) up
 *       to user's choice.
 */

/*
 * MACRO: FIELD_DEFINE
 *
 * Purpose
 *    The macro "FIELD_DEFINE" encodes a bit field and structure offset value
 *    given its size and its shift value with respect to bit 0 and the structure
 *    offset.
 *
 * Note
 *    A more intuitive way to encode bit fields would have been to use their
 *    mask. However, extracting size and shift value information from a bit
 *    field's mask is cumbersome and might break the assembler (255-character
 *    line-size limit). The offset value is a nicety so that one FIELD_DEFINE
 *    tag encapsulates all relevant access data.
 *
 * Input
 *    Size              Size of the bit field, in number of bits.
 *    Shft              Shift value of the bit field with respect to bit 0.
 *    Off               Offset value in struct (handling up to programmer).
 *
 * Output
 *    FIELD_DEFINE      Encoded bit field.
 */
#define FIELD_DEFINE(Size, Shft, Off) \
	((((UDATA(Off)) & 0x000000FFUL) << 16) | \
	 (((Size) & 0x00FF) << 8) | ((Shft) & 0x00FF))

/*
 * MACROS: FIELD_SIZE, FIELD_SHIFT, FIELD_MASK, FIELD_ALIGNED_MASK,
 *         FIELD_1ST_BIT, FIELD_OFFSET
 *
 * Purpose
 *    The macros "FIELD_SIZE", "FIELD_SHIFT", "FIELD_MASK",
 *    "FIELD_ALIGNED_MASK", "FIELD_1ST_BIT", and "FIELD_OFFSET" return
 *    the size, shift value, mask, aligned mask, first bit, and stucture
 *    offset value of a bit field.
 *
 * Input
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_SIZE        Size of the bit field, in number of bits.
 *    FIELD_SHIFT       Shift value of the bit field with respect to bit 0.
 *    FIELD_MASK        Mask for the bit field.
 *    FIELD_ALIGNED_MASK Mask for the bit field, aligned on bit 0.
 *    FIELD_1ST_BIT     First bit of the bit field.
 *    FIELD_OFFSET      Access offset of word containing bit field in structure.
 */
#define FIELD_SIZE(Field) \
	(((Field) >> 8) & 0x000000FFUL)

#define FIELD_SHIFT(Field) \
({ \
	uint32_t tmp_field = (Field); \
	(tmp_field & 0x000000FFUL); \
})

#define FIELD_ALIGNED_MASK(Field) \
({ \
	uint32_t tmp_field_size = (32 - (FIELD_SIZE(Field))); \
	((UDATA(0xFFFFFFFFUL)) >> tmp_field_size); \
})

#define FIELD_MASK(Field) \
	((FIELD_ALIGNED_MASK(Field)) << (FIELD_SHIFT(Field)))

#define FIELD_1ST_BIT(Field) \
	((UDATA(1UL)) << (FIELD_SHIFT(Field)))

#define FIELD_OFFSET(Field) \
({ \
	uint32_t tmp_field = (Field); \
	((tmp_field >> 16) & 0x000000FFUL); \
})

/*
 * MACRO: FIELD_INSERT
 *
 * Purpose
 *    The macro "FIELD_INSERT" inserts a value into a bit field by shifting the
 *    former appropriately.
 *
 * Input
 *    Value             Bit-field value.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_INSERT      Bit-field value positioned appropriately.
 */
#define FIELD_INSERT(Value, Field) \
	(((UDATA(Value)) & (FIELD_ALIGNED_MASK(Field))) << (FIELD_SHIFT(Field)))

/*
 * MACRO: FIELD_EXTRACT
 *
 * Purpose
 *    The macro "FIELD_EXTRACT" extracts the value of a bit field by masking and
 *    shifting it appropriately.
 *
 * Input
 *    Data              Data containing the bit-field to be extracted.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    FExtr             Bit-field value.
 */
#define FIELD_EXTRACT(Data, Field) \
	(((UDATA(Data)) >> (FIELD_SHIFT(Field))) & (FIELD_ALIGNED_MASK(Field)))

/*
 * MACRO: FIELD_CLEAR
 *
 * Purpose
 *    The macro "FIELD_CLEAR" clears just the bits in the selected "Field" from
 *    the packed "Value".
 *
 * Input
 *    Value             Bit-field value.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    FIELD_CLEAR       "Value" with selected "Field" bits cleared.
 */
#define FIELD_CLEAR(Value, Field) \
	((UDATA(Value)) & ~(FIELD_MASK(Field)))

/*
 * MACRO: FIELD_SET
 *
 * Purpose
 *    The macro "FIELD_SET" inserts a value into an "Original" bit field by
 *    shifting the former appropriately and doing a logical-OR with the
 *    "Original" bitfield which has the field bits cleared.
 *
 * Input
 *    Value             Bit-field value.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FInsrt            Bit-field value positioned appropriately.
 */
#define FIELD_SET(Value, Original, Field) \
	(FIELD_CLEAR((Original), (Field)) | (FIELD_INSERT((Value), (Field))))

/*****************************************************************************/
/*****************************************************************************/

/*
 * Structure offset access macros
 * Assumes that "Base" is a pointer to an array of 32 bit unsigned integers that
 * can be indexed by the Offset field in the "Field" token.
 */

/*
 * MACROS: FIELD_UA_SHIFT, FIELD_UA_MASK, FIELD_UA_ALIGNED_MASK, FIELD_UA_OFFSET
 *
 * Purpose
 *    The macros "FIELD_UA_SHIFT", "FIELD_UA_MASK", "FIELD_UA_ALIGNED_MASK" and
 *    "FIELD_UA_OFFSET".
 *    The shift value, mask, aligned mask, and stucture offset value of a bit
 *    field relative to the first byte in the structure containing the field's
 *    low-order bit (assumes FIELD_OFFSET() returns 4-byte word offset).
 *
 * Input
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_UA_SHIFT    Shift value of the bit field with respect to bit 0 of
 *                      byte in structure containing the field.
 *    FIELD_UA_MASK     Mask for the bit field (relative to byte in structure
 *                      containing bit 0 of field).
 *    FIELD_UA_ALIGNED_MASK Mask for the bit field, aligned on bit 0 (same as
 *                      FIELD_AILIGNED_MASK()).
 *    FIELD_UA_OFFSET   Access offset of byte containing bit 0 of field in
 *                      the structure.
 */

/* Shift from base of byte containing low-order bit of field */
#define FIELD_UA_SHIFT(Field) \
	((FIELD_SHIFT(Field)) & 0x7)

#define FIELD_UA_MASK(Field) \
	((FIELD_ALIGNED_MASK(Field)) << (FIELD_UA_SHIFT(Field)))

/* Aligned mask is always the same - include here for completeness */
#define FIELD_UA_ALIGNED_MASK(Field) \
	(FIELD_ALIGNED_MASK(Field))

#define FIELD_UA_OFFSET(Field) \
	((FIELD_OFFSET(Field) << 2) + (FIELD_SHIFT(Field) >> 3))

/*
 * MACRO: FIELD_UA_ADDRESS
 *
 * Purpose
 *    The macro "FIELD_UA_ADDRESS" returns the address of the structure
 *    described by "Base" and "Field".
 *
 * Input
 *    Base              Base address of the structure as an array of 8 bit
 *                      integers (uint8_t *).
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_UA_ADDRESS  Address of (pointer to) 32 bit unsigned int containing
 *                      the desired field relative to the possibly unaligned
 *                      "Base". Note that all fields are still dealt with as
 *                      32 values, but this allows unaligned access.
 */
#define FIELD_UA_ADDRESS(Base, Field) \
	((uint8_t *)((uint8_t *)(Base) + (FIELD_OFFSET(Field) * 4)))

/*
 * MACRO: FIELD_UA_BYTE_ADDRESS
 *
 * Purpose
 *    The macro "FIELD_UA_BYTE_ADDRESS" returns the byte address of bit 0 of the
 *    field within the structure described by "Base" and "Field".
 *
 * Input
 *    Base              Base address of the structure as an array of 8 bit
 *                      integers (uint8_t *).
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_UA_BYTE_ADDRESS Address of (pointer to) 8 bit unsigned int
 *                      containing bit 0 of the desired field relative to the
 *                      possibly unaligned "Base" (usable with FIELD_UA_xxx
 *                      macros). Note that all fields are still dealt with as 32
 *                      values, but shifts, etc using the UA macros are relative
 *                      to this Byte address.
 */
#define FIELD_UA_BYTE_ADDRESS(Base, Field) \
	((uint8_t *)((uint8_t *)(Base) + (FIELD_UA_OFFSET(Field))))

/*
 * MACRO: FIELD_UA_INSERT
 *
 * Purpose
 *    The macro "FIELD_UA_INSERT" inserts a value into a bit field by shifting
 *    the former appropriately.
 *
 * Input
 *    Value             Bit-field value.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_UA_INSERT   Bit-field value positioned appropriately.
 */
#define FIELD_UA_INSERT(Value, Field) \
	(((UDATA(Value)) & (FIELD_UA_ALIGNED_MASK(Field))) \
	 << (FIELD_UA_SHIFT(Field)))

/*
 * MACRO: FIELD_UA_EXTRACT
 *
 * Purpose
 *    The macro "FIELD_UA_EXTRACT" extracts the value of a bit field by masking
 *    and shifting it appropriately.
 *
 * Input
 *    Data              Data containing the bit-field to be extracted.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    FExtr             Bit-field value.
 */
#define FIELD_UA_EXTRACT(Data, Field) \
	(((UDATA(Data)) >> (FIELD_UA_SHIFT(Field))) & \
	 (FIELD_UA_ALIGNED_MASK(Field)))

/*
 * MACRO: FIELD_UA_CLEAR
 *
 * Purpose
 *    The macro "FIELD_UA_CLEAR" clears just the bits in the selected "Field"
 *    from the packed "Value".
 *
 * Input
 *    Value             Bit-field value.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    FIELD_UA_CLEAR    "Value" with selected "Field" bits cleared.
 */
#define FIELD_UA_CLEAR(Value, Field) \
	((UDATA(Value)) & ~(FIELD_UA_MASK(Field)))

/*
 * MACRO: FIELD_UA_SET
 *
 * Purpose
 *    The macro "FIELD_UA_SET" inserts a value into an "Original" bit field by
 *    shifting the former appropriately and doing a logical-OR with the
 *    "Original" bitfield which has the field bits cleared.
 *
 * Input
 *    Value             Bit-field value.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FInsrt            Bit-field value positioned appropriately.
 */
#define FIELD_UA_SET(Value, Original, Field) \
	((FIELD_UA_CLEAR((Original), (Field))) | \
	 (FIELD_UA_INSERT((Value), (Field))))

/*****************************************************************************/

/*
 * MACRO: FIELD_A32_ADDRESS
 *
 * Purpose
 *    The macro "FIELD_A32_ADDRESS" returns the address of the structure
 *    described by "Base" and "Field".
 *
 * Input
 *    Base              Base address of the array of 32 bit integers.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_A32_ADDRESS Address of (pointer to) 32 bit unsigned int containing
 *                      the desired field.
 */
#define FIELD_A32_ADDRESS(Base, Field) \
	((uint32_t *)((uint32_t *)(Base) + (FIELD_OFFSET(Field))))

/*
 * MACRO: FIELD_A32_INSERT
 *
 * Purpose
 *    The macro "FIELD_A32_INSERT" inserts a value into a bit field by shifting
 *    the former appropriately. The bitfield is written to the structure pointed
 *    to by "Base".
 *
 *    WARNING - this will overwrite any other bitfields in the uint32_t to "0",
 *    so this macro should not be used except when this behavior is OK.
 *
 * Input
 *    Value             Bit-field value.
 *    Base              Base address of the array of 32 bit integers.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_INSERT      Bit-field value positioned appropriately in the struct.
 */
#define FIELD_A32_INSERT(Value, Base, Field) \
({ \
	*(FIELD_A32_ADDRESS((Base), (Field))) = \
			(FIELD_INSERT((Value), (Field))); \
})

/*
 * MACRO: FIELD_A32_EXTRACT
 *
 * Purpose
 *    The macro "FIELD_A32_EXTRACT" extracts the value of a bit field from the
 *    structure by indexing, masking and shifting it appropriately.
 *
 * Input
 *    Base              Base address of the array of 32 bit integers.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    FIELD_A32_EXTRACT Bit-field value.
 */
#define FIELD_A32_EXTRACT(Base, Field) \
	FIELD_EXTRACT((*(FIELD_A32_ADDRESS((Base), (Field)))), (Field))

/*
 * MACRO: FIELD_A32_CLEAR
 *
 * Purpose
 *    The macro "FIELD_A32_CLEAR" clears just the bits in the selected "Field"
 *    from the packed structure at "Base".
 *
 * Input
 *    Base              Base address of the structure/array.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    FIELD_A32_CLEAR   Structure at "Base" has selected "Field" bits cleared.
 */
#define FIELD_A32_CLEAR(Base, Field) \
{ \
	*(FIELD_A32_ADDRESS((Base), (Field))) &= ~(FIELD_MASK(Field)); \
}

/*
 * MACRO: FIELD_A32_SET
 *
 * Purpose
 *    The macro "FIELD_A32_SET" inserts a value into a "Base" bit field struct
 *    by indexing and shifting the value former appropriately and doing a
 *    logical-OR with the "Basel" bitfield which has the field bits cleared.
 *
 * Input
 *    Value             Bit-field value.
 *    Base              Base address of structure/array.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    FIELD_A32_SET     Bit-field value positioned appropriately.
 */
#define FIELD_A32_SET(Value, Base, Field) \
{ \
	register uint32_t *fieldPtr = (FIELD_A32_ADDRESS((Base), (Field))); \
	*fieldPtr = (FIELD_SET((UDATA(Value)), (*fieldPtr), (Field))); \
}

/******************************************************************************/
/******************************************************************************/

/*
 * Function: field_ua_extract()
 *
 * Purpose
 *    The function "field_ua_extract()" extracts the value of a bit field from
 *    an unaligned structure at using by indexing, masking and shifting it
 *    appropriately. Handled as a function (not an inline function) due
 *    to the need to.
 *
 * Input
 *    Base              Base address of the array of 32 bit integers.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    returns unsigned integer Bit-field value.
 */
uint32_t field_ua_extract(void *Base, uint32_t Field);

/*
 * Function: field_ua_clear()
 *
 * Purpose
 *    The function "field_ua_clear" clears just the bits in the selected "Field"
 *    from the packed unaligned structure at "Base".
 *
 * Input
 *      Base            Base address of the structure/array.
 *      Field           Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *      Unaligned structure at "Base" has selected "Field" bits cleared.
 */
void field_ua_clear(void *Base, uint32_t Field);

/*
 * Function: field_ua_set()
 *
 * Purpose
 *    The function "field_ua_set" inserts "Value" in an unaligned bit field
 *    structure at "Base" by indexing and shifting the value former
 *    appropriately and doing a logical-OR with the "Base"
 *    bitfield which has the field bits cleared.
 *
 * Input
 *    Value             Bit-field value.
 *    Base              Base address of structure/array.
 *    Field             Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    Unaligned structure at "Base" has the "Field" bits in the unaligned
 *    structure set correctly.
 */
void field_ua_set(uint32_t Value, void *Base, uint32_t Field);

/******************************************************************************/
/******************************************************************************/

#endif /* !STRUCT_BITFIELD_H */

