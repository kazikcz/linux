#ifndef __NET_MAC80211_CODEL_H
#define __NET_MAC80211_CODEL_H

/*
 * Codel - The Controlled-Delay Active Queue Management algorithm
 *
 *  Copyright (C) 2011-2012 Kathleen Nichols <nichols@pollere.com>
 *  Copyright (C) 2011-2012 Van Jacobson <van@pollere.net>
 *  Copyright (C) 2016 Michael D. Taht <dave.taht@bufferbloat.net>
 *  Copyright (C) 2012 Eric Dumazet <edumazet@google.com>
 *  Copyright (C) 2015 Jonathan Morton <chromatix99@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the authors may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/skbuff.h>
#include <net/pkt_sched.h>
#include <net/inet_ecn.h>
#include <linux/reciprocal_div.h>

#include "codel_i.h"

/* Controlling Queue Delay (CoDel) algorithm
 * =========================================
 * Source : Kathleen Nichols and Van Jacobson
 * http://queue.acm.org/detail.cfm?id=2209336
 *
 * Implemented on linux by Dave Taht and Eric Dumazet
 */

/* CoDel5 uses a real clock, unlike codel */

static inline u64 codel_get_time(void)
{
	return ktime_get_ns();
}

static inline u32 codel_time_to_us(u64 val)
{
	do_div(val, NSEC_PER_USEC);
	return (u32)val;
}

/* sizeof_in_bits(rec_inv_sqrt) */
#define REC_INV_SQRT_BITS (8 * sizeof(u16))
/* needed shift to get a Q0.32 number from rec_inv_sqrt */
#define REC_INV_SQRT_SHIFT (32 - REC_INV_SQRT_BITS)

/* Newton approximation method needs more iterations at small inputs,
 * so cache them.
 */

static void codel_vars_init(struct codel_vars *vars)
{
	memset(vars, 0, sizeof(*vars));
}

/*
 * http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Iterative_methods_for_reciprocal_square_roots
 * new_invsqrt = (invsqrt / 2) * (3 - count * invsqrt^2)
 *
 * Here, invsqrt is a fixed point number (< 1.0), 32bit mantissa, aka Q0.32
 */
static inline void codel_Newton_step(struct codel_vars *vars)
{
	u32 invsqrt = ((u32)vars->rec_inv_sqrt) << REC_INV_SQRT_SHIFT;
	u32 invsqrt2 = ((u64)invsqrt * invsqrt) >> 32;
	u64 val = (3LL << 32) - ((u64)vars->count * invsqrt2);

	val >>= 2; /* avoid overflow in following multiply */
	val = (val * invsqrt) >> (32 - 2 + 1);

	vars->rec_inv_sqrt = val >> REC_INV_SQRT_SHIFT;
}

/*
 * CoDel control_law is t + interval/sqrt(count)
 * We maintain in rec_inv_sqrt the reciprocal value of sqrt(count) to avoid
 * both sqrt() and divide operation.
 */
static u64 codel_control_law(u64 t,
			     u64 interval,
			     u32 rec_inv_sqrt)
{
	return t + reciprocal_scale(interval, rec_inv_sqrt <<
				    REC_INV_SQRT_SHIFT);
}

/* Forward declaration of this for use elsewhere */

static inline u64
custom_codel_get_enqueue_time(struct sk_buff *skb);

static inline struct sk_buff *
custom_dequeue(struct codel_vars *vars, void *ptr);

static inline void
custom_drop(struct sk_buff *skb, void *ptr);

static bool codel_should_drop(struct sk_buff *skb,
			      __u32 *backlog,
			      __u32 backlog_thr,
			      struct codel_vars *vars,
			      const struct codel_params *p,
			      u64 now)
{
	if (!skb) {
		vars->first_above_time = 0;
		return false;
	}

	if (now - custom_codel_get_enqueue_time(skb) < p->target ||
	    *backlog <= backlog_thr) {
		/* went below - stay below for at least interval */
		vars->first_above_time = 0;
		return false;
	}

	if (vars->first_above_time == 0) {
		/* just went above from below; mark the time */
		vars->first_above_time = now + p->interval;

	} else if (now > vars->first_above_time) {
		return true;
	}

	return false;
}

static struct sk_buff *codel_dequeue(void *ptr,
				     __u32 *backlog,
				     __u32 backlog_thr,
				     struct codel_vars *vars,
				     struct codel_params *p,
				     u64 now,
				     bool overloaded)
{
	struct sk_buff *skb = custom_dequeue(vars, ptr);
	bool drop;

	if (!skb) {
		vars->dropping = false;
		return skb;
	}
	drop = codel_should_drop(skb, backlog, backlog_thr, vars, p, now);
	if (vars->dropping) {
		if (!drop) {
			/* sojourn time below target - leave dropping state */
			vars->dropping = false;
		} else if (now >= vars->drop_next) {
			/* It's time for the next drop. Drop the current
			 * packet and dequeue the next. The dequeue might
			 * take us out of dropping state.
			 * If not, schedule the next drop.
			 * A large backlog might result in drop rates so high
			 * that the next drop should happen now,
			 * hence the while loop.
			 */

			/* saturating increment */
			vars->count++;
			if (!vars->count)
				vars->count--;

			codel_Newton_step(vars);
			vars->drop_next = codel_control_law(vars->drop_next,
							    p->interval,
							    vars->rec_inv_sqrt);
			do {
				if (INET_ECN_set_ce(skb) && !overloaded) {
					vars->ecn_mark++;
					/* and schedule the next drop */
					vars->drop_next = codel_control_law(
						vars->drop_next, p->interval,
						vars->rec_inv_sqrt);
					goto end;
				}
				custom_drop(skb, ptr);
				vars->drop_count++;
				skb = custom_dequeue(vars, ptr);
				if (skb && !codel_should_drop(skb, backlog,
							      backlog_thr,
							      vars, p, now)) {
					/* leave dropping state */
					vars->dropping = false;
				} else {
					/* schedule the next drop */
					vars->drop_next = codel_control_law(
						vars->drop_next, p->interval,
						vars->rec_inv_sqrt);
				}
			} while (skb && vars->dropping && now >=
				 vars->drop_next);

			/* Mark the packet regardless */
			if (skb && INET_ECN_set_ce(skb))
				vars->ecn_mark++;
		}
	} else if (drop) {
		if (INET_ECN_set_ce(skb) && !overloaded) {
			vars->ecn_mark++;
		} else {
			custom_drop(skb, ptr);
			vars->drop_count++;

			skb = custom_dequeue(vars, ptr);
			drop = codel_should_drop(skb, backlog, backlog_thr,
						 vars, p, now);
			if (skb && INET_ECN_set_ce(skb))
				vars->ecn_mark++;
		}
		vars->dropping = true;
		/* if min went above target close to when we last went below
		 * assume that the drop rate that controlled the queue on the
		 * last cycle is a good starting point to control it now.
		 */
		if (vars->count > 2 &&
		    now - vars->drop_next < 8 * p->interval) {
			vars->count -= 2;
			codel_Newton_step(vars);
		} else {
			vars->count = 1;
			vars->rec_inv_sqrt = ~0U >> REC_INV_SQRT_SHIFT;
		}
		codel_Newton_step(vars);
		vars->drop_next = codel_control_law(now, p->interval,
						    vars->rec_inv_sqrt);
	}
end:
	return skb;
}
#endif
