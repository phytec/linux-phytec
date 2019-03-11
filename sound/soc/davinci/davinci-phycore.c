/*
 * ASoC machine driver for Phytec phyCORE-AM335x based on davinci-evm
 *
 * Author:      Stefan Müller-Klieser, <s.mueller-klieser@phytec.de>
 * Copyright:   (C) 2015-2016 PHYTEC Messtechnik GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#define DRV_NAME "davinci_phycore"

struct snd_soc_card_drvdata_davinci {
	struct clk *mclk;
	unsigned sysclk;
	unsigned clk_gpio;
	struct snd_pcm_hw_constraint_list *rate_constraint;
};

static unsigned int phycore_get_bclk(struct snd_pcm_hw_params *params)
{
	int sample_size = params_width(params);
	int rate = params_rate(params);
	int channels = params_channels(params);

	return sample_size * channels * rate;
}

static int phycore_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_davinci *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

static void phycore_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_davinci *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

static int phycore_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct platform_device *pdev = to_platform_device(soc_card->dev);
	unsigned int pll_out, bclk_div, tx_bits;
	/* We run the codec in i2s mode. */
	unsigned int tdm_slots = 2;
	int ret = 0;
	unsigned int rate = params_rate(params);
	unsigned int bclk = phycore_get_bclk(params);
	unsigned sysclk = ((struct snd_soc_card_drvdata_davinci *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;

	/* set the mcasp system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't set CPU DAI sysclock: %d\n", ret);
		return ret;
	}

	/* The codec can operate either in clock slave or in clock master mode
	 * mixed clocked modes are not supported by the codec. We need to setup
	 * the clocking for those two cases.
	 */
	if ((soc_card->dai_link->dai_fmt & SND_SOC_DAIFMT_MASTER_MASK)
						== SND_SOC_DAIFMT_CBS_CFS) {
		/* Codec is clock slave */

		/* set mcasp MCLK div */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, 1);
		if (ret < 0)
			return ret;

		/* set mcasp BCLK */
		bclk_div = sysclk / bclk;
		ret = snd_soc_dai_set_clkdiv(cpu_dai, 1, bclk_div);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"can't set CPU DAI clock divider: %d\n", ret);
			return ret;
		}

		/* set mcasp BCLK/FS ratio */
		tx_bits = params_width(params) * tdm_slots;
		/* bclk_fs_ratio */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, 2, tx_bits);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"can't set CPU DAI clock divider: %d\n", ret);
			return ret;
		}

		/* calculate codec pll - mcasp cannot generate accurate frame
		 * sync in i2s mode maybe we can do better with DSP or DIT mode
		 * and use padding bits to get closed to the desired frequency.
		 * in the pll_out equation we have:
		 * 256 = fixed divider
		 * 2 = default of MCLKDIV (WM8974 datasheet)
		 * MCLKDIV does not get touched anywhere in the driver
		 */
		pll_out = sysclk / bclk_div / tx_bits * 256 * 2;
	} else {
		/* Codec is clock master:
		 * This is the preferred mode, as the clock is closer to the
		 * correct frequency and clock jitter is smaller so the DAC
		 * performance is much better
		 */
		pll_out = rate * 256 * 2;
	}

	/* set the codec pll */
	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, sysclk, pll_out);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't set codec pll_out, ret: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops phycore_ops = {
	.startup = phycore_startup,
	.shutdown = phycore_shutdown,
	.hw_params = phycore_hw_params,
};

static const struct snd_soc_dapm_widget wm8974_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Loudspeaker", NULL),
	SND_SOC_DAPM_SPK("Headphones", NULL),
	SND_SOC_DAPM_LINE("Mono Out Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static int phycore_wm8974_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	/* Add machine specific widgets */
	snd_soc_dapm_new_controls(&card->dapm, wm8974_dapm_widgets,
				  ARRAY_SIZE(wm8974_dapm_widgets));

	ret = snd_soc_of_parse_audio_routing(card, "ti,audio-routing");
	if (ret)
		return ret;

	/* not connected */
	snd_soc_dapm_nc_pin(&card->dapm, "MICP");
	snd_soc_dapm_nc_pin(&card->dapm, "AUX");

	return 0;
}

static struct snd_soc_dai_link phycore_dai_wm8974 = {
	.name           = "WM8974",
	.stream_name    = "WM8974",
	.codec_dai_name = "wm8974-hifi",
	.ops            = &phycore_ops,
	.init           = phycore_wm8974_init,
};

static const struct of_device_id davinci_phycore_dt_ids[] = {
	{
		.compatible = "phytec,am335x-phycore",
		.data = &phycore_dai_wm8974,
	},
	{},
};
MODULE_DEVICE_TABLE(of, davinci_phycore_dt_ids);

/* davinci phycore audio machine driver */
static struct snd_soc_card phycore_soc_card = {
	.owner = THIS_MODULE,
	.num_links = 1,
};

static int davinci_phycore_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(davinci_phycore_dt_ids), &pdev->dev);
	struct snd_soc_dai_link *dai = (struct snd_soc_dai_link *) match->data;
	struct snd_soc_card_drvdata_davinci *drvdata = NULL;
	struct clk *mclk;
	int ret = 0;

	phycore_soc_card.dai_link = dai;

	dai->codec_of_node = of_parse_phandle(np, "ti,audio-codec", 0);
	if (!dai->codec_of_node)
		return -EINVAL;

	dai->cpu_of_node = of_parse_phandle(np, "ti,mcasp-controller", 0);
	if (!dai->cpu_of_node)
		return -EINVAL;

	dai->platform_of_node = dai->cpu_of_node;

	/* dai_fmt will be set from standard dts bindings legacy parsing */
	dai->dai_fmt = snd_soc_of_parse_daifmt(np, "ti,", NULL, NULL);

	phycore_soc_card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&phycore_soc_card, "ti,model");
	if (ret)
		return ret;

	mclk = devm_clk_get(&pdev->dev, "mclk");
	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "mclk not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->mclk = mclk;

	ret = of_property_read_u32(np, "ti,codec-clock-rate", &drvdata->sysclk);

	if (ret < 0) {
		if (!drvdata->mclk) {
			dev_err(&pdev->dev,
				"No clock or clock rate defined.\n");
			return -EINVAL;
		}
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
	} else if (drvdata->mclk) {
		unsigned int requestd_rate = drvdata->sysclk;

		clk_set_rate(drvdata->mclk, drvdata->sysclk);
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
		if (drvdata->sysclk != requestd_rate)
			dev_warn(&pdev->dev,
				 "Could not get requested rate %u using %u.\n",
				 requestd_rate, drvdata->sysclk);
	}

	snd_soc_card_set_drvdata(&phycore_soc_card, drvdata);
	ret = devm_snd_soc_register_card(&pdev->dev, &phycore_soc_card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
}

static int davinci_phycore_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_card_drvdata_davinci *drvdata =
		(struct snd_soc_card_drvdata_davinci *)
		snd_soc_card_get_drvdata(card);

	if (drvdata->mclk)
		clk_put(drvdata->mclk);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver davinci_phycore_driver = {
	.probe          = davinci_phycore_probe,
	.remove         = davinci_phycore_remove,
	.driver         = {
		.name   = DRV_NAME,
		.pm     = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(davinci_phycore_dt_ids),
	},
};

module_platform_driver(davinci_phycore_driver);

MODULE_AUTHOR("Stefan Müller-Klieser");
MODULE_DESCRIPTION("Phytec phyCORE AM335x ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
