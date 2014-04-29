#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>
#include <osmocom/core/utils.h>

#include "codes.h"
#include "noise.h"

#define MAX_LEN_BITS		9182
#define MAX_LEN_BYTES		(9182/8)
#define DEFAULT_ITER		10000
#define DEFAULT_THREADS		1
#define MAX_THREADS		32
#define MAX_CODES		2048

/* Parameters for soft symbol generation
 *     Signal-to-noise ratio specified in dB and symbol amplitude, which has a
 *     valid range from 0 (no signal) to 127 (saturation).
 */
#define DEFAULT_SOFT_SNR	8.0
#define DEFAULT_SOFT_AMP	32.0

/* Command line arguments
 *     iter     - Number of iterations
 *     threads  - Number of concurrent threads to launch for benchmark test
 *     bench    - Enable the benchmarking test
 *     length   - Enable length checks
 *     skip     - Skip baseline comparison
 *     ber      - Enable the bit-error-rate test
 *     num      - Run code number if specified 
 */
struct cmd_options {
	int iter;
	int threads;
	int bench;
	int length;
	int skip;
	int base;
	int ber;
	int num;
	float snr;
};

/* Argument passing struct for benchmark threads */
struct benchmark_thread_arg {
	const struct conv_test_vector *tst;
	struct osmo_conv_code *code;
	int base;
	int iter;
	int err;
};

/* Convolutional encoder (uses generator polynomials - not API compatible) */
int test_conv_encode(const struct osmo_conv_code *code,
		     const unsigned rgen, const unsigned *gen,
		     const ubit_t *input, ubit_t *output);

/* API drop-in replacement */
int test_conv_decode(const struct osmo_conv_code *code,
		     const sbit_t *input, ubit_t *output);

struct conv_test_vector {
	const char *name;
	const char *spec;
	const struct osmo_conv_code *code;
	unsigned rgen;
	unsigned gen[4];
	int in_len;
	int out_len;
	int has_vec;
	pbit_t vec_in[MAX_LEN_BYTES];
	pbit_t vec_out[MAX_LEN_BYTES];
};

static void enable_prio(float prio)
{
    int min, max;
    struct sched_param param;

    if (prio > 1.0)
        prio = 1.0;
    if (prio < 0)
        prio = 0.0;

    max = sched_get_priority_max(SCHED_FIFO);
    min = sched_get_priority_min(SCHED_FIFO);
    sched_getparam(0, &param);
    param.sched_priority = (int) ((max - min) * prio + min);
    sched_setscheduler(0, SCHED_FIFO, &param);
}

const struct conv_test_vector tests[] = {
	{
		.name = "GSM xCCH",
		.spec = "(N=2, K=5, non-recursive, flushed, not punctured)",
		.code = &gsm_conv_xcch,
		.rgen = 0,
		.gen = { 023, 033 },
		.in_len  = 224,
		.out_len = 456,
		.has_vec = 1,
		.vec_in  = { 0xf3, 0x1d, 0xb4, 0x0c, 0x4d, 0x1d, 0x9d, 0xae,
			     0xc0, 0x0a, 0x42, 0x57, 0x13, 0x60, 0x80, 0x96,
			     0xef, 0x23, 0x7e, 0x4c, 0x1d, 0x96, 0x24, 0x19,
			     0x17, 0xf2, 0x44, 0x99 },
		.vec_out = { 0xe9, 0x4d, 0x70, 0xab, 0xa2, 0x87, 0xf0, 0xe7,
			     0x04, 0x14, 0x7c, 0xab, 0xaf, 0x6b, 0xa1, 0x16,
			     0xeb, 0x30, 0x00, 0xde, 0xc8, 0xfd, 0x0b, 0x85,
			     0x80, 0x41, 0x4a, 0xcc, 0xd3, 0xc0, 0xd0, 0xb6,
			     0x26, 0xe5, 0x4e, 0x32, 0x49, 0x69, 0x38, 0x17,
			     0x33, 0xab, 0xaf, 0xb6, 0xc1, 0x08, 0xf3, 0x9f,
			     0x8c, 0x75, 0x6a, 0x4e, 0x08, 0xc4, 0x20, 0x5f,
			     0x8f },
	},
	{
		.name = "GPRS CS2",
		.spec = "(N=2, K=5, non-recursive, flushed, not punctured)",
		.code = &gsm_conv_cs2,
		.rgen = 0,
		.gen = { 023, 033 },
		.in_len  = 290,
		.out_len = 588,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GPRS CS3",
		.spec = "(N=2, K=5, non-recursive, flushed, not punctured)",
		.code = &gsm_conv_cs3,
		.rgen = 0,
		.gen = { 023, 033 },
		.in_len  = 334,
		.out_len = 676,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM RACH",
		.spec = "(N=2, K=5, non-recursive, flushed, not punctured)",
		.code = &gsm_conv_rach,
		.rgen = 0,
		.gen = { 023, 033 },
		.in_len  = 14,
		.out_len = 36,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/FR",
		.spec = "(N=2, K=5, non-recursive, flushed, not punctured)",
		.code = &gsm_conv_tch_fr,
		.rgen = 0,
		.gen = { 023, 033 },
		.in_len  = 185,
		.out_len = 378,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AFS 12.2",
		.spec = "(N=2, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_afs_12_2,
		.rgen = 023,
		.gen = { 020, 033 },
		.in_len  = 250,
		.out_len = 448,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AFS 10.2",
		.spec = "(N=3, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_afs_10_2,
		.rgen = 037,
		.gen = { 033, 025, 020 },
		.in_len  = 210,
		.out_len = 448,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AFS 7.95",
		.spec = "(N=3, K=7, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_afs_7_95,
		.rgen = 0133,
		.gen = { 0100, 0145, 0175 },
		.in_len  = 165,
		.out_len = 448,
		.has_vec = 1,
		.vec_in  = { 0x87, 0x66, 0xc3, 0x58, 0x09, 0xd4, 0x06, 0x59,
			     0x10, 0xbf, 0x6b, 0x7f, 0xc8, 0xed, 0x72, 0xaa,
			     0xc1, 0x3d, 0xf3, 0x1e, 0xb0 },
		.vec_out = { 0x92, 0xbc, 0xde, 0xa0, 0xde, 0xbe, 0x01, 0x2f,
			     0xbe, 0xe4, 0x61, 0x32, 0x4d, 0x4f, 0xdc, 0x41,
			     0x43, 0x0d, 0x15, 0xe0, 0x23, 0xdd, 0x18, 0x91,
			     0xe5, 0x36, 0x2d, 0xb7, 0xd9, 0x78, 0xb8, 0xb1,
			     0xb7, 0xcb, 0x2f, 0xc0, 0x52, 0x8f, 0xe2, 0x8c,
			     0x6f, 0xa6, 0x79, 0x88, 0xed, 0x0c, 0x2e, 0x9e,
			     0xa1, 0x5f, 0x45, 0x4a, 0xfb, 0xe6, 0x5a, 0x9c },
	},
	{
		.name = "GSM TCH/AFS 7.4",
		.spec = "(N=3, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_afs_7_4,
		.rgen = 037,
		.gen = { 033, 025, 020 },
		.in_len  = 154,
		.out_len = 448,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AFS 6.7",
		.spec = "(N=4, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_afs_6_7,
		.rgen = 037,
		.gen = { 033, 025, 020, 020 },
		.in_len  = 140,
		.out_len = 448,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AFS 5.9",
		.spec = "(N=4, K=7, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_afs_5_9,
		.rgen = 0175,
		.gen = { 0133, 0145, 0100, 0100 },
		.in_len  = 124,
		.out_len = 448,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AHS 7.95",
		.spec = "(N=2, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_ahs_7_95,
		.rgen = 023,
		.gen = { 020, 033 },
		.in_len  = 129,
		.out_len = 188,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AHS 7.4",
		.spec = "(N=2, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_ahs_7_4,
		.rgen = 023,
		.gen = { 020, 033 },
		.in_len  = 126,
		.out_len = 196,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AHS 6.7",
		.spec = "(N=2, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_ahs_6_7,
		.rgen = 023,
		.gen = { 020, 033 },
		.in_len  = 116,
		.out_len = 200,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AHS 5.9",
		.spec = "(N=2, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_ahs_5_9,
		.rgen = 023,
		.gen = { 020, 033 },
		.in_len  = 108,
		.out_len = 208,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AHS 5.15",
		.spec = "(N=3, K=5, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_ahs_5_15,
		.rgen = 037,
		.gen = { 033, 025, 020 },
		.in_len  = 97,
		.out_len = 212,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GSM TCH/AHS 4.75",
		.spec = "(N=3, K=7, recursive, flushed, punctured)",
		.code = &gsm_conv_tch_ahs_4_75,
		.rgen = 0133,
		.gen = { 0100, 0145, 0175 },
		.in_len  = 89,
		.out_len = 212,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "WiMax FCH",
		.spec = "(N=2, K=7, non-recursive, tail-biting, non-punctured)",
		.code = &wimax_conv_fch,
		.rgen = 0,
		.gen = { 0171, 0133 },
		.in_len  = 48,
		.out_len = 96,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "GMR-1 TCH3 Speech",
		.spec = "(N=2, K=7, non-recursive, tail-biting, punctured)",
		.code = &gmr1_conv_tch3_speech,
		.rgen = 0,
		.gen = { 0133, 0171 },
		.in_len  = 48,
		.out_len = 72,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "LTE PBCH",
		.spec = "(N=3, K=7, non-recursive, tail-biting, non-punctured)",
		.code = &lte_conv_pbch,
		.rgen = 0,
		.gen = { 0133, 0171, 0165 },
		.in_len  = 40,
		.out_len = 120,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{
		.name = "Random Code",
		.spec = "(N=2, K=5, non-recursive, truncated, non-punctured)",
		.code = &conv_trunc,
		.rgen = 0,
		.gen = { 023, 033 },
		.in_len  = 224,
		.out_len = 448,
		.has_vec = 0,
		.vec_in  = { },
		.vec_out = { },
	},
	{ /* end */ },
};

static void print_codes()
{
	int i = 1;
	const struct conv_test_vector *tst;

	printf("\n");
	printf("Code  0:  Test all codes\n");
	for (tst=tests; tst->name; tst++)
		printf("Code %2i:  %-18s %s\n", i++, tst->name, tst->spec);
	printf("\n");
}

static void fill_random(ubit_t *b, int n)
{
	int i, r, m, c;

	c = 0;
	r = rand();
	m = sizeof(int) - 1;

	for (i = 0; i < n; i++) {
		if (c++ == m) {
			r = rand();
			c = 0;
		}

		b[i] = (r >> (i % m)) & 0x01;
	}
}

/* Generate NRZ values of +/- 127 */
static void ubit_to_sbit(sbit_t *dst, ubit_t *src, int n)
{
	int i;

	for (i = 0; i < n; i++)
		dst[i] = src[i] * -254 + 127;
}

/* Generate soft bits with AWGN channel */
static int ubit_to_err(sbit_t *dst, ubit_t *src, int n, float snr)
{
	int i, err = 0;

	add_noise(src, dst, n, snr, DEFAULT_SOFT_AMP);

	for (i = 0; i < n; i++) {
		if ((src[i] && (dst[i] >= 0)) || (!src[i] && (dst[i] <= 0)))
			err++;
	}

	return err;
}

/* Generate binary symmetric channel based on sliced soft error bits */
static int ubit_to_xerr(sbit_t *dst, ubit_t *src, int n, float snr)
{
	int i, err;

	err = ubit_to_err(dst, src, n, snr);

	for (i = 0; i < n; i++)
		dst[i] = dst[i] < 0 ? -127 : 127;

	return err;
}

/* Output error input/output error rates */
static void print_error_results(const struct conv_test_vector *tst,
				int iber, int ober, int fer, int iter)
{
	printf("[..] Input BER.......................... %f\n",
	       (float) iber / (iter * tst->out_len));
	printf("[..] Output BER......................... %f\n",
	       (float) ober / (iter * tst->out_len));
	printf("[..] Output FER......................... %f ",
	       (float) fer / iter, fer);
	if (fer > 0)
		printf("(%i)\n", fer);
	else
		printf("\n");
}

/* Timed performance benchmark */
static double get_timed_results(struct timeval *tv0, struct timeval *tv1,
			        const struct conv_test_vector *tst,
				int iter, int threads)
{
	double elapsed;

	elapsed = (tv1->tv_sec - tv0->tv_sec);
	elapsed += (tv1->tv_usec - tv0->tv_usec) / 1e6;
	printf("[..] Elapsed time....................... %f secs\n", elapsed);
	printf("[..] Rate............................... %f Mbps\n",
	       (float) threads * tst->in_len * iter / elapsed / 1e6);

	return elapsed;
}

/* Bit error rate test */
static int error_test(const struct conv_test_vector *tst,
		      int iter, float snr, int base)
{
	int i, n, l, iber = 0, ober = 0, fer = 0;
	sbit_t *bs;
	ubit_t *bu0, *bu1;
	int (*decode) (const struct osmo_conv_code *, const sbit_t *, ubit_t *);

	bu0 = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bu1 = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bs  = malloc(sizeof(sbit_t) * MAX_LEN_BITS);

	if (base)
		decode = osmo_conv_decode;
	else
		decode = test_conv_decode;

	for (i = 0; i < iter; i++) {
		fill_random(bu0, tst->in_len);

		l = test_conv_encode(tst->code, tst->rgen, tst->gen, bu0, bu1);
		if (l != tst->out_len) {
			printf("ERROR !\n");
			fprintf(stderr, "[!] Failed encoding length check (%i)\n",
				l);
			return -1;
		}

		iber += ubit_to_err(bs, bu1, l, snr);
		decode(tst->code, bs, bu1);

		for (n = 0; n < tst->in_len; n++) {
                        if (bu0[n] != bu1[n])
                                ober++;
                }

		if (memcmp(bu0, bu1, tst->in_len))
			fer++;
	}

	print_error_results(tst, iber, ober, fer, iter);

	free(bs);
	free(bu1);
	free(bu0);

	return 0;
}

static int init_thread_arg(struct benchmark_thread_arg *arg,
			    const struct conv_test_vector *tst,
			    int iter, int base)
{
	sbit_t *bs;
	ubit_t *bu;
	int (*decode) (const struct osmo_conv_code *, const sbit_t *, ubit_t *);
	struct osmo_conv_code *code;

	code = (struct osmo_conv_code *) malloc(sizeof(struct osmo_conv_code));
	memcpy(code, tst->code, sizeof(struct osmo_conv_code));

	bu = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bs = malloc(sizeof(sbit_t) * MAX_LEN_BITS);

	if (base)
		decode = osmo_conv_decode;
	else
		decode = test_conv_decode;

	decode(code, bs, bu);

	arg->tst = tst;
	arg->base = base;
	arg->iter = iter;
	arg->code = code;
	arg->err = 0;

	free(bs);
	free(bu);

	return 0;
}

/* One benchmark benchmark thread with random valued input */
static void *thread_test(void *ptr)
{
	int i;
	sbit_t *bs;
	ubit_t *bu0, *bu1;
	struct benchmark_thread_arg *arg = (struct benchmark_thread_arg *) ptr;
	int (*decode) (const struct osmo_conv_code *, const sbit_t *, ubit_t *);

	bu0 = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bu1 = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bs  = malloc(sizeof(sbit_t) * MAX_LEN_BITS);

	enable_prio(0.5);

	if (arg->base)
		decode = osmo_conv_decode;
	else
		decode = test_conv_decode;

	for (i = 0; i < arg->iter; i++)
		decode(arg->code, bs, bu1);

	free(bs);
	free(bu1);
	free(bu0);

	pthread_exit(NULL);
}

/* Fire off benchmark threads and measure elapsed time */
static double run_benchmark(const struct conv_test_vector *tst,
			    struct benchmark_thread_arg *args,
			    int num_threads, int iter, int base)
{
	int i, rc, err = 0;
	void *status;
	struct timeval tv0, tv1;
	pthread_t threads[MAX_THREADS];

	for (i = 0; i < num_threads; i++) {
		rc = init_thread_arg(&args[i], tst, iter, base);
		if (rc < 0)
			return -1.0;
	}


	gettimeofday(&tv0, NULL);
	for (i = 0; i < num_threads; i++) {
		pthread_create(&threads[i], NULL,
			       thread_test, (void *) &args[i]);
	}
	for (i = 0; i < num_threads; i++) {
		pthread_join(threads[i], &status);
		err |= args[i].err;
	}
	gettimeofday(&tv1, NULL);

	if (err)
		return -1.0;

	return get_timed_results(&tv0, &tv1, tst, iter, num_threads);
}

/* Verify output values with predefined input */
static int value_test(const struct conv_test_vector *tst)
{
	int l;
	sbit_t *bs;
	ubit_t *bu0, *bu1;

	bu0 = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bu1 = malloc(sizeof(ubit_t) * MAX_LEN_BITS);
	bs  = malloc(sizeof(sbit_t) * MAX_LEN_BITS);

	osmo_pbit2ubit(bu0, tst->vec_in, tst->in_len);

	l = test_conv_encode(tst->code, tst->rgen, tst->gen, bu0, bu1);
	if (l != tst->out_len) {
		fprintf(stderr, "[!] Failed encoding length check (%i)\n", l);
		return -1;
	}

	osmo_pbit2ubit(bu0, tst->vec_out, tst->out_len);

	if (memcmp(bu0, bu1, tst->out_len)) {
		fprintf(stderr, "[!] Failed encoding: Results don't match\n");
		return -1;
	};

	printf("OK\n");
	printf("[..] Decoding base: \n");

	ubit_to_sbit(bs, bu0, l);
	l = osmo_conv_decode(tst->code, bs, bu1);
	if (l != 0) {
		fprintf(stderr, "[!] Failed decoding: non-zero path (%d)\n", l);
		return -1;
	}

	osmo_pbit2ubit(bu0, tst->vec_in, tst->in_len);
	if (memcmp(bu0, bu1, tst->in_len)) {
		fprintf(stderr, "[!] Failed decoding: Results don't match\n");
		return -1;
	}

	printf("[..] Decoding SIMD: \n");
	printf("[..] Code N %i\n", tst->code->N);
	printf("[..] Code K %i\n", tst->code->K);
	l = test_conv_decode(tst->code, bs, bu1);
	if (l != 0) {
		fprintf(stderr,
			"[!] Failed decoding: SIMD non-zero path (%d)\n", l);
		return -1;
	}

	osmo_pbit2ubit(bu0, tst->vec_in, tst->in_len);
	if (memcmp(bu0, bu1, tst->in_len)) {
		fprintf(stderr, "[!] Failed decoding: Results don't match\n");
		return -1;
	}

	free(bs);
	free(bu1);
	free(bu0);

	printf("[..] Decoding: OK\n");
	return 0;
}

/* Verify output lengths */
static int length_test(const struct conv_test_vector *tst)
{
	int l;

	l = osmo_conv_get_input_length(tst->code, 0);
	printf("[.] Input length  : ret = %3d  exp = %3d -> %s\n",
		l, tst->in_len, l == tst->in_len ? "OK" : "Bad !");

	if (l != tst->in_len) {
		fprintf(stderr, "[!] Failure for input length computation\n");
		return -1;
	}

	l = osmo_conv_get_output_length(tst->code, 0);
	printf("[.] Output length : ret = %3d  exp = %3d -> %s\n",
		l, tst->out_len, l == tst->out_len ? "OK" : "Bad !");

	if (l != tst->out_len) {
		fprintf(stderr, "[!] Failure for output length computation\n");
		return -1;
	}

	return 0;
}

static void print_help()
{
	fprintf(stdout, "Options:\n"
		"  -h    This text\n"
		"  -i    Number of iterations\n"
		"  -j    Number of threads for benchmark (EXPERIMENTAL)\n"
		"  -a    Run all tests\n"
		"  -b    Run benchmark tests\n"
		"  -n    Run length checks\n"
		"  -e    Run bit error rate tests\n"
		"  -s    Skip baseline decoder\n"
		"  -r    Specify SNR in dB (default %2.1f dB)\n"
		"  -o    Run baseline decoder only\n"
		"  -c    Test specific code\n"
		"  -l    List supported codes\n", DEFAULT_SOFT_SNR);
}

static void handle_options(int argc, char **argv, struct cmd_options *cmd)
{
	int option;

	cmd->iter = DEFAULT_ITER;
	cmd->threads = DEFAULT_THREADS;
	cmd->bench = 0;
	cmd->length = 0;
	cmd->skip = 0;
	cmd->base = 0;
	cmd->ber = 0;
	cmd->num = 0;
	cmd->snr = DEFAULT_SOFT_SNR;

	while ((option = getopt(argc, argv, "hi:baesoc:r:lj:")) != -1) {
		switch (option) {
		case 'h':
			print_help();
			exit(0);
			break;
		case 'i':
			cmd->iter = atoi(optarg);
			if (cmd->iter < 1) {
				printf("Iterations must be at least 1\n");
				exit(0);
			}
			break;
		case 'a':
			cmd->bench = 1;
			cmd->length = 1;
			cmd->ber = 1;
			break;
		case 'b':
			cmd->bench = 1;
			break;
		case 'n':
			cmd->length = 1;
			break;
		case 'e':
			cmd->ber = 1;
			break;
		case 's':
			cmd->skip = 1;
			break;
		case 'o':
			cmd->base = 1;
			break;
		case 'c':
			cmd->num = atoi(optarg);
			if (cmd->num < 0) {
				print_codes();
				exit(0);
			}
			break;
		case 'r':
			cmd->snr = atof(optarg);
			break;
		case 'l':
			print_codes();
			exit(0);
			break;
		case 'j':
			cmd->threads = atoi(optarg);
			if ((cmd->threads < 1) ||
			    (cmd->threads > MAX_THREADS)) {
				printf("Threads must be between 1 to %i\n",
				       MAX_THREADS);
				exit(0);
			}
			break;
		default:
			print_help();
			exit(0);
		}
	}

	if (!cmd->bench && !cmd->length && !cmd->ber) {
		cmd->length = 1;
		cmd->ber = 1;
	}
}

int main(int argc, char *argv[])
{
	int cnt = 0;
	const struct conv_test_vector *tst;
	double elapsed0 = 0.0, elapsed1 = 0.0;
	struct benchmark_thread_arg args[MAX_THREADS * 2];
	struct cmd_options cmd;

	handle_options(argc, argv, &cmd);

	srandom(time(NULL));

	for (tst=tests; tst->name; tst++) {
		if ((cmd.num > 0) && (cmd.num != ++cnt))
			continue;

		/* Test name */
		printf("\n=================================================\n");
		printf("[+] Testing: %s\n", tst->name);
		printf("[.] Specs: %s\n", tst->spec);

		/* Check length */
		if (length_test(tst) < 0)
			return -1;

		/* Check pre-computed vector */
		if (cmd.length && tst->has_vec) {
			printf("[.] Pre computed vector checks:\n");
			printf("[..] Encoding: ");

			if (value_test(tst) < 0)
				return -1;
		}

		/* BER tests */
		if (cmd.ber) {
			printf("\n[.] BER tests:\n");
			if (!cmd.skip) {
				printf("[..] Testing base:\n");
				if (error_test(tst, cmd.iter, cmd.snr, 1) < 0)
					return -1;
			}

			if (!cmd.base) {
				printf("[..] Testing SIMD:\n");
				if (error_test(tst, cmd.iter, cmd.snr, 0) < 0)
					return -1;
			}
		}

		if (!cmd.bench)
			continue;

		/* Timed benchmark tests */
		printf("\n[.] Performance benchmark:\n");
		printf("[..] Encoding / Decoding %i bursts on %i thread(s):\n",
		       cmd.iter * cmd.threads, cmd.threads);

		if (!cmd.skip) {
			printf("[..] Testing base:\n");
			elapsed0 = run_benchmark(tst, args,
						 cmd.threads, cmd.iter, 1);
			if (elapsed0 < 0.0)
				goto shutdown;
		}

		if (!cmd.base) {
			printf("[..] Testing SIMD:\n");
			elapsed1 = run_benchmark(tst, args,
						 cmd.threads, cmd.iter, 0);
			if (elapsed1 < 0.0)
				goto shutdown;
		}

		if (!cmd.skip && !cmd.base) {
			printf("[..] Speedup............................ %f\n",
			       elapsed0 / elapsed1);
		}
		printf("\n");
	}
	printf("\n");

shutdown:
	return 0;
}
