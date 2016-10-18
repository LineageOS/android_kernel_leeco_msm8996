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

#ifndef _HAL_H
#define _HAL_H

#include "sii_common.h"

/* HAL API */
enum sii_status SiiHalInit(void);
void SiiHalTerm(void);
enum sii_status SiiHalReset(bool reset);
enum sii_status SiiHalSpiOp(uint32_t *pTxData, uint32_t *pRxData,
				uint32_t size);
enum sii_status SiiHalBulkTxfr(uint32_t *pBuffer, uint32_t size);
void SiiHalIsr(void);
void SiiHalIsrDone(void);

#endif /* !_HAL_H */

