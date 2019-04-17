/* codec.cpp -- the encoder and decoder program for the thermal camera SDK

   This file is part of the thermal camera SDK.

   Copyright (C) 1996-2017 Issac Tseng
   All Rights Reserved.

   The thermal camera SDK is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of
   the License, or (at your option) any later version.

   The thermal camera SDK is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with the thermal camera SDK; see the file COPYING.
   If not, write to the Free Software Foundation, Inc.,
   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

   Issac Tseng
   <chinaybtseng@gmail.com>
 */

/*************************************************************************
// This program shows the basic usage of the codec.
// We will compress or decompress a block of data.
**************************************************************************/

#include <cstdio>
#include <cassert>
#include "codec.h"

// ----------------------------------------------------------------------------------
// Compress a block of data.
// ----------------------------------------------------------------------------------
lzo_int encoder(
	const lzo_bytep const in,
	lzo_uint in_len,
	lzo_bytep const out,
	lzo_uint &out_len
)	{
	int r;
	lzo_voidp wrkmem;
	assert(in);
	assert(in_len > 0);
	assert(out);
/*
 * Step 1: initialize the LZO library
 */
    if ((r = lzo_init()) != LZO_E_OK) {
		printf("ERROR: lzo_init()\n");
		return r;
	}	
/*
 * Step 2: allocate blocks and the work-memory
 */
	wrkmem = (lzo_voidp)xmalloc(LZO1X_1_MEM_COMPRESS);
    if (wrkmem == NULL)
    {
        printf("ERROR: xmalloc()\n");
		return LZO_E_OUT_OF_MEMORY;
    }
/*
 * Step 3: compress from 'in' to 'out' with LZO1X-1
 */
    r = lzo1x_1_compress(in, in_len, out, &out_len, wrkmem);
    if (r == LZO_E_OK) {
        // printf("Compressed %lu bytes into %lu bytes\n", (unsigned long) in_len, (unsigned long) out_len);
    } else {
        /* this should NEVER happen */
        printf("Internal error - compression failed: %d\n", r);
        return r;
    }
	lzo_free(wrkmem);
	return r;
}

// ----------------------------------------------------------------------------------
// Decompress a block of data.
// ----------------------------------------------------------------------------------
lzo_int decoder(
	const lzo_bytep const in,
	lzo_uint in_len,
	lzo_bytep const out,
	lzo_uint &out_len
)	{
	int r;
	assert(in);
	assert(in_len > 0);
	assert(out);
/*
 * Step 1: initialize the LZO library
 */
    if ((r = lzo_init()) != LZO_E_OK) {
		printf("ERROR: lzo_init()\n");
		return r;
	}	
/*
 * Step 2: decompress again, now going from 'out' to 'in'
 */
    r = lzo1x_decompress(in, in_len, out, &out_len, NULL);
    if (r == LZO_E_OK) {
        // printf("Decompressed %lu bytes back into %lu bytes\n", (unsigned long)in_len, (unsigned long)out_len);
    } else {
        /* this should NEVER happen */
        printf("Internal error - decompression failed: %d\n", r);
        return r;
    }
	return r;
}