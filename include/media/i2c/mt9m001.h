#ifndef _MEDIA_MT9M001_H
#define _MEDIA_MT9M001_H

struct mt9m001_platform_data {
	unsigned int clk_pol:1;

	int ext_freq;

	const s64 *link_freqs;
	s64 link_def_freq;
};

#endif
