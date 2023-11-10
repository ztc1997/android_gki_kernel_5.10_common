/*
 * Cryptographic API.
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2022. All rights reserved.
 * Description: LZ4KD compression algorithm for ZRAM
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/crypto.h>
#include <linux/vmalloc.h>
#include <linux/lz4kd.h>


struct lz4kd_ctx {
	void *lz4kd_comp_mem;
};

static int lz4kd_init(struct crypto_tfm *tfm)
{
	struct lz4kd_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->lz4kd_comp_mem = vmalloc(lz4kd_encode_state_bytes_min());
	if (!ctx->lz4kd_comp_mem)
		return -ENOMEM;

	return 0;
}

static void lz4kd_exit(struct crypto_tfm *tfm)
{
	struct lz4kd_ctx *ctx = crypto_tfm_ctx(tfm);
	vfree(ctx->lz4kd_comp_mem);
}

static int lz4kd_compress_crypto(struct crypto_tfm *tfm, const u8 *src, unsigned int slen, u8 *dst,
				unsigned int *dlen)
{
	struct lz4kd_ctx *ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	ret = lz4kd_encode(ctx->lz4kd_comp_mem, src, dst, slen, *dlen, 0);
	if (ret < 0)
		return -EINVAL;

	if (ret)
		*dlen = ret;

	return 0;
}

static int lz4kd_decompress_crypto(struct crypto_tfm *tfm, const u8 *src, unsigned int slen, u8 *dst,
				unsigned int *dlen)
{
	int ret = 0;

	ret = lz4kd_decode(src, dst, slen, *dlen);
	if (ret <= 0)
		return -EINVAL;
	*dlen = ret;
	return 0;
}

static struct crypto_alg alg_lz4kd = {
	.cra_name		= "lz4kd",
	.cra_driver_name	= "lz4kd-generic",
	.cra_flags		= CRYPTO_ALG_TYPE_COMPRESS,
	.cra_ctxsize		= sizeof(struct lz4kd_ctx),
	.cra_module		= THIS_MODULE,
	.cra_init		= lz4kd_init,
	.cra_exit		= lz4kd_exit,
	.cra_u			= {
	.compress = {
			.coa_compress    = lz4kd_compress_crypto,
			.coa_decompress  = lz4kd_decompress_crypto
		}
	}
};

static int __init lz4kd_mod_init(void)
{
	return crypto_register_alg(&alg_lz4kd);
}

static void __exit lz4kd_mod_fini(void)
{
	crypto_unregister_alg(&alg_lz4kd);
}

module_init(lz4kd_mod_init);
module_exit(lz4kd_mod_fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LZ4KD Compression Algorithm");
MODULE_ALIAS_CRYPTO("lz4kd");
