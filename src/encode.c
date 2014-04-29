/*
 * Viterbi encoder
 * Copyright (C) 2013-2014 Thomas Tsou <tom@tsou.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <errno.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/conv.h>

#define PARITY(X) __builtin_parity(X)
#define POPCNT(X) __builtin_popcount(X)

static int puncture(const struct osmo_conv_code *code,
		    ubit_t *unpunct, ubit_t *output)
{
	int len, i, j = 0, l = 0;
	const int *puncture = code->puncture;

	if (code->term == CONV_TERM_FLUSH)
		len = code->len + code->K - 1;
	else
		len = code->len;

	for (i = 0; i < len * code->N; i++) {
		if (i == puncture[j]) {
			j++;
			continue;
		}

		output[l++] = unpunct[i];
	}

	return l;
}

static int encode_n2(const struct osmo_conv_code *code,
		     const unsigned *gen, const ubit_t *input,
		     ubit_t *output)
{
	unsigned reg = 0;
	int len = code->len;
	int i, k = code->K;

	if (code->term == CONV_TERM_TAIL_BITING) {
		for (i = 0; i < k - 1; i++)
			reg |= input[len - 1 - i] << (k - 2 - i);
	}

	for (i = 0; i < len; i++) {
		reg |= input[i] << (k - 1);
		output[2 * i + 0] = PARITY(reg & gen[0]);
		output[2 * i + 1] = PARITY(reg & gen[1]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		output[2 * i + 0] = PARITY(reg & gen[0]);
		output[2 * i + 1] = PARITY(reg & gen[1]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_n3(const struct osmo_conv_code *code,
		     const unsigned *gen, const ubit_t *input,
		     ubit_t *output)
{
	unsigned reg = 0;
	int len = code->len;
	int i, k = code->K;

	if (code->term == CONV_TERM_TAIL_BITING) {
		for (i = 0; i < k - 1; i++)
			reg |= input[len - 1 - i] << (k - 2 - i);
	}

	for (i = 0; i < len; i++) {
		reg |= input[i] << (k - 1);
		output[3 * i + 0] = PARITY(reg & gen[0]);
		output[3 * i + 1] = PARITY(reg & gen[1]);
		output[3 * i + 2] = PARITY(reg & gen[2]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		output[3 * i + 0] = PARITY(reg & gen[0]);
		output[3 * i + 1] = PARITY(reg & gen[1]);
		output[3 * i + 2] = PARITY(reg & gen[2]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_n4(const struct osmo_conv_code *code,
		     const unsigned *gen, const ubit_t *input,
		     ubit_t *output)
{
	unsigned reg = 0;
	int len = code->len;
	int i, k = code->K;

	if (code->term == CONV_TERM_TAIL_BITING) {
		for (i = 0; i < k - 1; i++)
			reg |= input[len - 1 - i] << (k - 2 - i);
	}

	for (i = 0; i < len; i++) {
		reg |= input[i] << (k - 1);
		output[4 * i + 0] = PARITY(reg & gen[0]);
		output[4 * i + 1] = PARITY(reg & gen[1]);
		output[4 * i + 2] = PARITY(reg & gen[2]);
		output[4 * i + 3] = PARITY(reg & gen[2]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		output[4 * i + 0] = PARITY(reg & gen[0]);
		output[4 * i + 1] = PARITY(reg & gen[1]);
		output[4 * i + 2] = PARITY(reg & gen[2]);
		output[4 * i + 3] = PARITY(reg & gen[2]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_rec_n2(const struct osmo_conv_code *code,
			 const unsigned rgen, const unsigned *gen,
			 int pos, const ubit_t *input, ubit_t *output)
{
	unsigned reg = 0;
	int len = code->len;
	int i, m0, m1, k = code->K;

	m0 = pos;
	m1 = (pos + 1) % 2;

	for (i = 0; i < len; i++) {
		reg |= (PARITY((reg & rgen)) ^ input[i]) << (k - 1);
		output[2 * i + m0] = input[i];
		output[2 * i + m1] = PARITY(reg & gen[m1]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		output[2 * i + m0] = PARITY(reg & rgen);
		output[2 * i + m1] = PARITY(reg & gen[m1]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_rec_n3(const struct osmo_conv_code *code,
			 const unsigned rgen, const unsigned *gen,
			 int pos, const ubit_t *input, ubit_t *output)
{
	unsigned reg = 0;
	int len = code->len;
	int i, m0, m1, m2, k = code->K;

	m0 = pos;
	m1 = (pos + 1) % 3;
	m2 = (pos + 2) % 3;

	for (i = 0; i < len; i++) {
		reg |= (PARITY((reg & rgen)) ^ input[i]) << (k - 1);
		output[3 * i + m0] = input[i];
		output[3 * i + m1] = PARITY(reg & gen[m1]);
		output[3 * i + m2] = PARITY(reg & gen[m2]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		output[3 * i + m0] = PARITY(reg & rgen);
		output[3 * i + m1] = PARITY(reg & gen[m1]);
		output[3 * i + m2] = PARITY(reg & gen[m2]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_rec_n4(const struct osmo_conv_code *code,
			 const unsigned rgen, const unsigned *gen,
			 int pos, const ubit_t *input, ubit_t *output)
{
	unsigned reg = 0;
	int len = code->len;
	int i, m0, m1, m2, m3, k = code->K;

	m0 = pos;
	m1 = (pos + 1) % 4;
	m2 = (pos + 2) % 4;
	m3 = (pos + 3) % 4;

	for (i = 0; i < len; i++) {
		reg |= (PARITY((reg & rgen)) ^ input[i]) << (k - 1);
		output[4 * i + m0] = input[i];
		output[4 * i + m1] = PARITY(reg & gen[m1]);
		output[4 * i + m2] = PARITY(reg & gen[m2]);
		output[4 * i + m3] = PARITY(reg & gen[m2]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		output[4 * i + m0] = PARITY(reg & rgen);
		output[4 * i + m1] = PARITY(reg & gen[m1]);
		output[4 * i + m2] = PARITY(reg & gen[m2]);
		output[4 * i + m3] = PARITY(reg & gen[m2]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_gen(const struct osmo_conv_code *code,
		      const unsigned *gen, const ubit_t *input,
		      ubit_t *output)
{
	unsigned reg = 0;
	int n = code->N;
	int len = code->len;
	int i, j, k = code->K;

	if (code->term == CONV_TERM_TAIL_BITING) {
		for (i = 0; i < k - 1; i++)
			reg |= input[code->len - 1 - i] << (k - 2 - i);
	}

	for (i = 0; i < len; i++) {
		reg |= input[i] << (k - 1);
		for (j = 0; j < n; j++)
			output[n * i + j] = PARITY(reg & gen[j]);
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		for (j = 0; j < n; j++)
			output[n * i + j] = PARITY(reg & gen[j]);
		reg = reg >> 1;
	}

	return i;
}

static int encode_rec_gen(const struct osmo_conv_code *code,
			  const unsigned rgen, const unsigned *gen,
			  const ubit_t *input, ubit_t *output)
{
	unsigned reg = 0;
	int n = code->N;
	int len = code->len;
	int i, j, k = code->K;
	int p[n];

	for (i = 0; i < n; i++)
		p[i] = POPCNT(gen[i]) == 1;

	for (i = 0; i < len; i++) {
		reg |= (PARITY((reg & rgen)) ^ input[i]) << (k - 1);

		for (j = 0; j < n; j++) {
			if (p[j])
				output[n * i + j] = input[i];
			else
				output[n * i + j] = PARITY(reg & gen[j]);
		}
		reg = reg >> 1;
	}

	if (code->term != CONV_TERM_FLUSH)
		return i;

	for (i = len; i < len + k - 1; i++) {
		for (j = 0; j < n; j++) {
			if (p[j])
				output[n * i + j] = PARITY(reg & rgen);
			else
				output[n * i + j] = PARITY(reg & gen[j]);
		}
		reg = reg >> 1;
	}

	return i;
}

static int conv_encode(const struct osmo_conv_code *code,
		       const unsigned *gen, const ubit_t *input,
		       ubit_t *output)
{
	int l;
	ubit_t *_output;
	ubit_t unpunct[(code->len + code->K - 1) * code->N];

	if (code->puncture)
		_output = unpunct;
	else
		_output = output;

	switch (code->N) {
	case 2:
		l = encode_n2(code, gen, input, _output);
		break;
	case 3:
		l = encode_n3(code, gen, input, _output);
		break;
	case 4:
		l = encode_n4(code, gen, input, _output);
		break;
	default:
		l = encode_gen(code, gen, input, _output);
		break;
	}

	if (code->puncture)
		return puncture(code, _output, output);

	return code->N * l;
}

static int conv_encode_rec(const struct osmo_conv_code *code,
			   const unsigned rgen, const unsigned *gen,
			   const ubit_t *input, ubit_t *output)
{
	int l, pos = -1, cnt = 0;
	ubit_t *_output;
	ubit_t unpunct[(code->len + code->K - 1) * code->N];

	if (code->term == CONV_TERM_TAIL_BITING)
		return -ENOTSUP;

	if (code->puncture)
		_output = unpunct;
	else
		_output = output;

	for (l = 0; l < code->N; l++) {
		if (POPCNT(gen[l]) == 1) {
			pos = l;
			cnt++;
		}
	}

	if ((cnt > 1)) {
		l = encode_rec_gen(code, rgen, gen, input, _output);
		goto puncture;
	}
	if (cnt < 1)
		return -EPROTO;

	switch (code->N) {
	case 2:
		l = encode_rec_n2(code, rgen, gen, pos, input, _output);
		break;
	case 3:
		l = encode_rec_n3(code, rgen, gen, pos, input, _output);
		break;
	case 4:
		l = encode_rec_n4(code, rgen, gen, pos, input, _output);
		break;
	default:
		l = encode_rec_gen(code, rgen, gen, input, _output);
		break;
	}

puncture:
	if (code->puncture)
		return puncture(code, _output, output);

	return code->N * l;
}

int test_conv_encode(const struct osmo_conv_code *code,
		     const unsigned rgen, const unsigned *gen,
		     const ubit_t *input, ubit_t *output)
{
	if ((code->N < 2) || (code->K < 5))
		return -EINVAL;

	if (code->next_term_output)
		return conv_encode_rec(code, rgen, gen, input, output);

	return conv_encode(code, gen, input, output);
}
