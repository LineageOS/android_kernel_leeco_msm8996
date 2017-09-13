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
 * Functions to operate on bitfields in packed structures.
 * Inspired by Marc Viredaz's open source "bitfield.h" package on Koders.com
 */
#include "struct_bitfield.h"

/*
 * Function: field_ua_extract()
 *
 * Purpose
 *    The function "field_ua_extract()" extracts the value of a bit field from
 *    an unaligned structure at using by indexing, masking and shifting it
 *    appropriately.  Handled as a function (not an inline function) due
 *    to the need to.
 *
 * Input
 *    Base      Base address of the array of 32 bit integers (uint32_t *).
 *    Field     Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    returns unsigned integer Bit-field value.
 */
uint32_t field_ua_extract(void *Base, uint32_t Field)
{
	uint32_t tempWord = 0;
	uint32_t maxOffInWord = (FIELD_SIZE(Field) + FIELD_SHIFT(Field)) - 1;

	if (NULL == Base)
		goto done;

	/* Only do more complex unaligned operations if needed.
	 * It is faster to do simple word access with no bits ovelapping.
	 */
	if (((uintptr_t)Base & 0x3L) || (maxOffInWord > 31)) {
		/* Simplify copy with local pointer */
		uint8_t *twPtr = (uint8_t *)&tempWord;
		uint8_t *tsPtr = (uint8_t *)FIELD_UA_BYTE_ADDRESS(Base, Field);

		/* Calculate highest bit in field and convert to byte offset.
		 * An 8 bit field at shift 0 would occupy bits 0-7,
		 * for instance.
		 * NOTE: highest bit of shifted field can be no higher than
		 *  bit 31. For now, just silently truncate if needed.
		 */
		maxOffInWord = (FIELD_SIZE(Field) + FIELD_UA_SHIFT(Field)) - 1;
		if (maxOffInWord > 31)
			maxOffInWord = 31;
		maxOffInWord /= 8;

		/* Copy data into word alignment. This is more efficient than
		 * calling memcpy() for a single word.
		 */
		tempWord = 0;
		twPtr[0] = tsPtr[0];
		if (maxOffInWord > 0)
			twPtr[1] = tsPtr[1];
		if (maxOffInWord > 1)
			twPtr[2] = tsPtr[2];
		if (maxOffInWord > 2)
			twPtr[3] = tsPtr[3];

		/* Basic code for LE over-the-air order,
		 * per WirelessHD 1.1 Chapter 8.1 */
		tempWord = FIELD_UA_EXTRACT(tempWord, Field);
	} else {
		tempWord = FIELD_A32_EXTRACT(Base, Field);
	}

done:
	return tempWord;
}

/*
 * Function: field_ua_clear()
 *
 * Purpose
 *    The function "field_ua_clear" clears just the bits in the selected "Field"
 *    from the packed unaligned structure at "Base".
 *
 * Input
 *    Base      Base address of the structure/array.
 *    Field     Encoded bit field (using the macro "FIELD_DEFINE"").
 *
 * Output
 *    Unaligned structure at "Base" has selected "Field" bits cleared.
 */
void field_ua_clear(void *Base, uint32_t Field)
{
	uint32_t tempWord = 0;
	uint32_t maxOffInWord = (FIELD_SIZE(Field) + FIELD_SHIFT(Field)) - 1;

	if (NULL == Base)
		return;

	/* Only do more complex unaligned operations if needed.
	 * It is faster to do simple word access with no bits ovelapping.
	 */
	if (((uintptr_t)Base & 0x3L) || (maxOffInWord > 31)) {
		/* Simplify copy with local pointer */
		uint8_t *twPtr = (uint8_t *)&tempWord;
		uint8_t *tsPtr = (uint8_t *)FIELD_UA_BYTE_ADDRESS(Base, Field);

		/* Calculate highest bit in field and convert to byte offset.
		 * An 8 bit field at shift 0 would occupy bits 0-7,
		 * for instance.
		 * NOTE: highest bit of shifted field can be no higher than
		 *  bit 31. For now, just silently truncate if needed.
		 */
		maxOffInWord = (FIELD_SIZE(Field) + FIELD_UA_SHIFT(Field)) - 1;
		if (maxOffInWord > 31)
			maxOffInWord = 31;
		maxOffInWord /= 8;

		/* Copy data into word alignment. This is more efficient than
		 * calling memcpy() for a single word.
		 */
		tempWord = 0;

		/* Copy only needed bytes. No need to include upper bytes of
		 * word if not in the current field.
		 */
		twPtr[0] = tsPtr[0];
		if (maxOffInWord > 0)
			twPtr[1] = tsPtr[1];
		if (maxOffInWord > 1)
			twPtr[2] = tsPtr[2];
		if (maxOffInWord > 2)
			twPtr[3] = tsPtr[3];

		tempWord = FIELD_UA_CLEAR(tempWord, Field);

		/* Write out the possibly modified bytes of the word. */
		tsPtr[0] = twPtr[0];
		if (maxOffInWord > 0)
			tsPtr[1] = twPtr[1];
		if (maxOffInWord > 1)
			tsPtr[2] = twPtr[2];
		if (maxOffInWord > 2)
			tsPtr[3] = twPtr[3];
	} else {
		FIELD_A32_CLEAR(Base, Field);
	}
}

/*
 * Function: field_ua_set()
 *
 * Purpose
 *    The function "field_ua_set" inserts "Value" in an unaligned bit field
 *    structure at "Base" by indexing and shifting the value former
 *    appropriately and doing a logical-OR with the "Base" bitfield which has
 *    the field bits cleared.
 *
 * Input
 *    Value     Bit-field value.
 *    Base      Base address of structure/array.
 *    Field     Encoded bit field (using the macro "FIELD_DEFINE").
 *
 * Output
 *    Unaligned structure at "Base" has the "Field" bits in the unaligned
 *    structure set correctly.
 */
void field_ua_set(uint32_t Value, void *Base, uint32_t Field)
{
	uint32_t tempWord = 0;
	uint32_t maxOffInWord = (FIELD_SIZE(Field) + FIELD_SHIFT(Field)) - 1;

	if (NULL == Base)
		return;

	/* Only do more complex unaligned operations if needed.
	 * It is faster to do simple word access with no bits ovelapping.
	 */
	if (((uintptr_t)Base & 0x3L) || (maxOffInWord > 31)) {
		/* Simplify copy with local pointer */
		uint8_t *twPtr = (uint8_t *)&tempWord;
		uint8_t *tsPtr = (uint8_t *)FIELD_UA_BYTE_ADDRESS(Base, Field);

		/* Calculate highest bit in field and convert to byte offset.
		 * An 8 bit field at shift 0 would occupy bits 0-7,
		 * for instance.
		 * NOTE: highest bit of shifted field can be no higher than
		 *  bit 31. For now, just silently truncate if needed.
		 */
		maxOffInWord = (FIELD_SIZE(Field) + FIELD_UA_SHIFT(Field)) - 1;
		if (maxOffInWord > 31)
			maxOffInWord = 31;
		maxOffInWord /= 8;

		/* Copy data into word alignment. This is more efficient than
		 * calling memcpy() for a single word.
		 */
		tempWord = 0;

		/* Copy only needed bytes. No need to include upper bytes of
		 * word if not in the current field.
		 */
		twPtr[0] = tsPtr[0];
		if (maxOffInWord > 0)
			twPtr[1] = tsPtr[1];
		if (maxOffInWord > 1)
			twPtr[2] = tsPtr[2];
		if (maxOffInWord > 2)
			twPtr[3] = tsPtr[3];

		tempWord = FIELD_UA_SET(Value, tempWord, Field);

		/* Write out the possibly modified bytes of the word. */
		tsPtr[0] = twPtr[0];
		if (maxOffInWord > 0)
			tsPtr[1] = twPtr[1];
		if (maxOffInWord > 1)
			tsPtr[2] = twPtr[2];
		if (maxOffInWord > 2)
			tsPtr[3] = twPtr[3];
	} else {
		FIELD_A32_SET(Value, Base, Field);
	}
}

