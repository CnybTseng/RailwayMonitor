#ifndef _CODEC_H_
#define _CODEC_H_

/* We will be using the LZO1X-1 algorithm, so we have
 * to include <lzo/lzo1x.h>
 */

#include <lzo/lzoconf.h>
#include <lzo/lzo1x.h>

static const char *progname = NULL;
#define WANT_LZO_MALLOC 1
#define WANT_XMALLOC 1

#include <lzo/portab.h>

/**
 * Compress a block of data.
 * @param[in] in Input data buffer.
 * @param[in] in_len Input data length.
 * @param[out] out Output data buffer.
 * @param[out] out_len Output data length.
 * @return Compress flag.
 */
lzo_int encoder(
	const lzo_bytep const in,
	lzo_uint in_len,
	lzo_bytep const out,
	lzo_uint &out_len
);

/**
 * Decompress a block of data.
 * @param[in] in Input data buffer.
 * @param[in] in_len Input data length.
 * @param[out] out Output data buffer.
 * @param[out] out_len Output data length.
 * @return Decompress flag.
 */
lzo_int decoder(
	const lzo_bytep const in,
	lzo_uint in_len,
	lzo_bytep const out,
	lzo_uint &out_len
);

#endif