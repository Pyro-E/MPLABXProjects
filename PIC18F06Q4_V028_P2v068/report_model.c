/* Host model of the PIC report boundary logic.
 *
 * Reproduces V022's SEND_IDLE / NotifyAck exactly as written, and V023's
 * frozen-cutoff version, then replays the same capture+session traffic through
 * both and checks the four invariants the RSP_DATA contract requires.
 *
 * This is not the firmware - it is the arithmetic of the firmware, lifted out
 * so the fix can be checked without a PIC.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SLOTS   1024
#define MARGIN  24

/* ---------------- shared "hardware" ---------------- */
typedef struct {
    uint16_t samples[SLOTS];    /* pulses stored per slot          */
    uint16_t write;
    uint32_t captures;
    uint32_t prev_total;        /* total at last COMPLETED capture */
    uint32_t live_total;        /* FlowMeter_GetTotal() equivalent  */
} Log;

static void log_init(Log *L) { memset(L, 0, sizeof(*L)); }

/* pulses arrive continuously; a capture closes and pushes a sample */
static void pulses(Log *L, uint32_t n) { L->live_total += n; }
static void capture(Log *L) {
    uint16_t d = (uint16_t)(L->live_total - L->prev_total);
    L->samples[L->write] = d;
    L->write = (uint16_t)((L->write + 1u) % SLOTS);
    L->captures++;
    L->prev_total = L->live_total;
}

/* ---------------- report state ---------------- */
typedef struct {
    uint16_t read, tx_read, end;
    uint32_t impulse_mark, caps_mark;
    uint32_t commit_total, commit_caps;
    bool     pending;
    /* frozen cutoff (V023 only) */
    bool     cut_valid;
    uint16_t cut_end;
    uint32_t cut_caps, cut_total;
    uint8_t  batch_seq, next_seq;
} Rep;

typedef struct {
    uint32_t caps, imp;      /* header total_captures / total_impulses */
    uint16_t count;          /* sample_count                          */
    uint32_t sum;            /* actual sum of the samples sent        */
    uint8_t  seq;
} Frame;

static void rep_init(Rep *R) { memset(R, 0, sizeof(*R)); R->next_seq = 1; }

static uint32_t span_sum(const Log *L, uint16_t from, uint16_t to) {
    uint32_t s = 0;
    for (uint16_t i = from; i != to; i = (uint16_t)((i + 1u) % SLOTS)) s += L->samples[i];
    return s;
}

/* ---- V022: rebuild from LIVE values on every REQ_DATA ---- */
static Frame v022_req(Log *L, Rep *R) {
    R->end = L->write;
    uint32_t cur_total = L->live_total;          /* FlowMeter_GetTotal() */
    uint32_t cur_caps  = L->captures;
    Frame f;
    f.caps  = cur_caps  - R->caps_mark;
    f.imp   = cur_total - R->impulse_mark;
    f.count = (uint16_t)((R->end - R->read + SLOTS) % SLOTS);
    f.sum   = span_sum(L, R->read, R->end);
    f.seq   = 0;
    R->tx_read = R->read;
    R->commit_total = cur_total;
    R->commit_caps  = cur_caps;
    R->pending = true;
    return f;
}
static void v022_ack(Rep *R) {
    if (!R->pending) return;
    R->read = R->end;
    R->impulse_mark = R->commit_total;
    R->caps_mark    = R->commit_caps;
    R->pending = false;
}

/* ---- V023: freeze at power-on, never rebuild before ACK ---- */
static Frame g_frozen;                            /* last built frame */
static void v023_freeze(const Log *L, Rep *R) {
    R->cut_end   = L->write;
    R->cut_caps  = L->captures;
    R->cut_total = L->prev_total;                 /* COMPLETED boundary */
    R->cut_valid = true;
}
static void v023_clear(Rep *R) { R->cut_valid = false; }

static Frame v023_req(Log *L, Rep *R) {
    if (R->pending) return g_frozen;              /* retransmit, unchanged */
    R->end = R->cut_end;
    uint32_t cur_total = R->cut_total;
    uint32_t cur_caps  = R->cut_caps;
    Frame f;
    f.caps  = cur_caps  - R->caps_mark;
    f.imp   = cur_total - R->impulse_mark;
    f.count = (uint16_t)((R->end - R->read + SLOTS) % SLOTS);
    f.sum   = span_sum(L, R->read, R->end);
    f.seq   = R->next_seq;
    R->batch_seq = R->next_seq;
    R->next_seq = R->next_seq + 1u ? R->next_seq + 1u : 1u;
    R->tx_read = R->read;
    R->commit_total = cur_total;
    R->commit_caps  = cur_caps;
    R->pending = true;
    g_frozen = f;
    return f;
}
static void v023_ack(Rep *R, uint8_t seq) {
    if (!R->pending || seq != R->batch_seq) return;
    R->read = R->end;
    R->impulse_mark = R->commit_total;
    R->caps_mark    = R->commit_caps;
    R->pending = false;
}

/* ---------------- invariant checks ---------------- */
static int fails = 0;
static void chk(bool ok, const char *what, const char *ver, int sess, int req) {
    if (!ok) { printf("  FAIL [%s] session %d req %d: %s\n", ver, sess, req, what); fails++; }
}

/* Replay: bench conditions from the 1.txt log.
 *   capture every 5.29 s at ~5 pulses; report due every 34 captures;
 *   the Photon retries REQ_DATA `retries` times before ACKing, and captures
 *   keep completing during the session (this is what V022 could not survive).
 */
static void run(const char *ver, bool v23, int retries, int caps_during_session) {
    Log L; Rep R; log_init(&L); rep_init(&R);
    uint32_t reported_pulses = 0;
    printf("%s (retries=%d, captures during session=%d)\n", ver, retries, caps_during_session);

    for (int sess = 1; sess <= 6; sess++) {
        for (int c = 0; c < 34; c++) { pulses(&L, 5); capture(&L); }   /* report period */

        if (v23) v023_freeze(&L, &R);
        uint32_t cutoff_pulses = L.prev_total;   /* what this report SHOULD cover */

        Frame first; memset(&first,0,sizeof(first)); bool have_first = false;
        for (int r = 0; r <= retries; r++) {
            /* captures keep completing while the Photon is awake */
            if (r > 0 && r <= caps_during_session) { pulses(&L, 5); capture(&L); }
            pulses(&L, 2);                       /* live meter runs ahead too */

            Frame f = v23 ? v023_req(&L, &R) : v022_req(&L, &R);
            if (!have_first) { first = f; have_first = true; }

            /* --- invariant B: samples can never exceed the header total --- */
            chk(f.sum <= f.imp, "sum(samples) > total_impulses", ver, sess, r);
            /* --- invariant C: no overflow here, so they must be equal --- */
            chk(f.sum == f.imp, "sum(samples) != total_impulses", ver, sess, r);
            /* --- invariant A: a retransmit is byte-identical --- */
            if (r > 0) {
                chk(f.count == first.count && f.caps == first.caps &&
                    f.imp == first.imp && f.sum == first.sum,
                    "batch changed before ACK", ver, sess, r);
            }
            /* --- invariant D: the batch covers exactly the frozen cutoff --- */
            if (v23) chk(R.commit_total == cutoff_pulses,
                         "batch does not end at the session cutoff", ver, sess, r);
        }
        reported_pulses += first.imp;
        if (v23) v023_ack(&R, R.batch_seq); else v022_ack(&R);

        /* after a full ACK the same session must have nothing left */
        Frame tail = v23 ? v023_req(&L, &R) : v022_req(&L, &R);
        chk(tail.count == 0 && tail.caps == 0 && tail.imp == 0,
            "post-ACK request not empty (n/caps/imp)", ver, sess, 99);
        if (v23) v023_ack(&R, R.batch_seq); else v022_ack(&R);
        if (v23) v023_clear(&R);
    }
    printf("  reported %u pulses; meter completed %u\n\n",
           reported_pulses, L.prev_total);
}

int main(void) {
    run("V022", false, 3, 2);
    int a = fails; fails = 0;
    run("V023 stress", true, 8, 8);
    printf("V023 stress violations: %d\n\n", fails); fails = a;
    int v022_fails = fails; fails = 0;
    run("V023", true, 3, 2);
    printf("V022 invariant violations: %d\nV023 invariant violations: %d\n",
           v022_fails, fails);
    return fails ? 1 : 0;
}
