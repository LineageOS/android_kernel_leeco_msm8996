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

#ifndef _SII_COMMON_H
#define _SII_COMMON_H

#include <linux/types.h> /* to get 'bool' */

enum sii_status {
	SII_STATUS_SUCCESS = 0,
	SII_STATUS_ERR_INVALID_PARAM = -1,
	SII_STATUS_ERR_FAILED = -2,
};

#endif /* !_SII_COMMON_H */

