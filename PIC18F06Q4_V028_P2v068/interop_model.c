/* V023 (PIC) <-> V064 (Photon) interoperation model.
 *
 * Mirrors the two firmwares' actual decision logic across a link that can lose
 * frames in either direction, and checks the properties the pair is supposed to
 * guarantee:
 *
 *   1. no loss        every captured pulse reaches the Photon's store exactly once
 *   2. no duplication a retransmitted batch is stored once, not twice
 *   3. no wedge       an ACK lost after storage still releases the PIC's batch
 *   4. no false skip  a genuinely NEW batch is never mistaken for a duplicate
 *
 * Property 4 is the one under test in the second scenario: the PIC's seq counter
 * restarts at 1 after a reset, and the Photon remembers only the LAST seq it
 * stored.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ================= PIC V023 ================= */
typedef struct {
    uint32_t captures, completed_total, live_total;
    uint32_t read_pulses;                 /* pulses already consumed (ACKed)   */
    uint32_t impulse_mark;
    /* frozen cutoff */
    bool     cut_valid; uint32_t cut_total, cut_caps;
    /* pending batch */
    bool     pending;
    uint32_t b_imp, b_caps; uint16_t b_n; uint8_t b_seq;
    uint8_t  next_seq;
} Pic;

static void pic_init(Pic *p){ memset(p,0,sizeof(*p)); p->next_seq=1; }
static void pic_capture(Pic *p, uint32_t pulses){
    p->live_total += pulses; p->completed_total = p->live_total; p->captures++;
}
static void pic_power_on(Pic *p){                       /* FreezeSessionCutoff */
    p->cut_total = p->completed_total; p->cut_caps = p->captures; p->cut_valid = true;
}
static void pic_power_off(Pic *p){ p->cut_valid = false; }  /* pending survives */

/* answer one REQ_DATA */
static void pic_req(Pic *p, uint32_t *imp, uint32_t *caps, uint16_t *n, uint8_t *seq){
    if (p->pending) {                                    /* retransmit, unchanged */
        *imp = p->b_imp; *caps = p->b_caps; *n = p->b_n; *seq = p->b_seq; return;
    }
    if (!p->cut_valid) pic_power_on(p);
    p->b_imp  = p->cut_total - p->impulse_mark;
    p->b_caps = p->cut_caps  - 0;                        /* caps mark folded below */
    p->b_n    = (uint16_t)(p->b_imp ? 1u : 0u);          /* 1 "sample" per batch  */
    p->b_seq  = p->next_seq;
    p->next_seq = (uint8_t)(p->next_seq + 1u); if (!p->next_seq) p->next_seq = 1u;
    p->pending = (p->b_n > 0u);      /* V023b: an empty batch is never pending */
    *imp = p->b_imp; *caps = p->b_caps; *n = p->b_n; *seq = p->b_seq;
}
static void pic_ack(Pic *p, uint8_t seq){
    if (!p->pending || seq != p->b_seq) return;          /* NotifyAckSeq */
    p->impulse_mark += p->b_imp;
    p->read_pulses  += p->b_imp;
    p->pending = false;
}
static void pic_reset(Pic *p){                           /* watchdog / brownout */
    p->cut_valid = false; p->pending = false; p->next_seq = 1;
    p->impulse_mark = p->completed_total;                /* FlowReport_Init baseline */
}

/* ================= Photon V064 ================= */
typedef struct {
    uint32_t stored_pulses;      /* total durably stored                        */
    uint32_t store_events;       /* how many blocks written                     */
    uint8_t  last_seq; bool last_seq_valid;   /* retained across power cuts     */
    /* V064+P-11 candidate: also remember the batch's shape                     */
    uint32_t last_imp, last_caps; uint16_t last_n;
    bool     strict;             /* true = compare content as well as seq       */
} Pho;

static void pho_init(Pho *h, bool strict){ memset(h,0,sizeof(*h)); h->strict = strict; }

/* returns true if an ACK should be sent */
static bool pho_rx(Pho *h, uint32_t imp, uint32_t caps, uint16_t n, uint8_t seq,
                   bool *stored){
    *stored = false;
    if (n == 0) return false;                            /* n=0: no ACK, healthy */
    bool dup = h->last_seq_valid && seq == h->last_seq;
    if (dup && h->strict)
        dup = (imp == h->last_imp && caps == h->last_caps && n == h->last_n);
    if (dup) return true;                                /* skip store, re-ACK   */
    h->stored_pulses += imp; h->store_events++;
    h->last_seq = seq; h->last_seq_valid = true;
    h->last_imp = imp; h->last_caps = caps; h->last_n = n;
    *stored = true;
    return true;
}

/* ================= scenarios ================= */
static int fails;
static void chk(bool ok, const char *what){ if(!ok){ printf("    FAIL: %s\n", what); fails++; } }

/* Scenario 1: ACK lost after durable storage, across a power cut. */
static void scen_ack_loss(bool strict){
    Pic p; Pho h; pic_init(&p); pho_init(&h, strict);
    uint32_t imp, caps; uint16_t n; uint8_t seq; bool stored;

    for (int s = 0; s < 5; s++) {
        for (int c = 0; c < 3; c++) pic_capture(&p, 5);
        pic_power_on(&p);
        pic_req(&p, &imp, &caps, &n, &seq);
        bool wantAck = pho_rx(&h, imp, caps, n, seq, &stored);
        if (s == 1 && wantAck) { /* ACK lost on the wire this session */ }
        else if (wantAck)       pic_ack(&p, seq);
        pic_power_off(&p);                               /* PIC cuts our power  */
    }
    /* drain */
    for (int k = 0; k < 4; k++) {
        pic_power_on(&p);
        pic_req(&p, &imp, &caps, &n, &seq);
        if (pho_rx(&h, imp, caps, n, seq, &stored)) pic_ack(&p, seq);
        pic_power_off(&p);
    }
    chk(h.stored_pulses == p.completed_total, "pulses lost or double-counted");
    chk(!p.pending, "PIC still wedged on an un-ACKed batch");
    printf("    stored=%u captured=%u store_events=%u\n",
           h.stored_pulses, p.completed_total, h.store_events);
}

/* Scenario 2: the PIC resets, so its seq counter restarts at 1, while the
 * Photon still remembers seq=1 from before. */
static void scen_seq_collision(bool strict){
    Pic p; Pho h; pic_init(&p); pho_init(&h, strict);
    uint32_t imp, caps; uint16_t n; uint8_t seq; bool stored;

    /* session 1: batch seq=1 stored and ACKed */
    for (int c = 0; c < 3; c++) pic_capture(&p, 5);
    pic_power_on(&p); pic_req(&p, &imp, &caps, &n, &seq);
    if (pho_rx(&h, imp, caps, n, seq, &stored)) pic_ack(&p, seq);
    pic_power_off(&p);

    /* PIC resets; its next batch is genuinely NEW but also carries seq=1 */
    pic_reset(&p);
    for (int c = 0; c < 4; c++) pic_capture(&p, 7);
    pic_power_on(&p); pic_req(&p, &imp, &caps, &n, &seq);
    printf("    after PIC reset: seq=%u (Photon remembers %u)\n", seq, h.last_seq);
    if (pho_rx(&h, imp, caps, n, seq, &stored)) pic_ack(&p, seq);
    pic_power_off(&p);

    chk(stored, "a genuinely NEW batch was skipped as a duplicate (silent loss)");
    chk(h.stored_pulses == p.completed_total, "pulse total wrong after PIC reset");
    printf("    stored=%u captured=%u\n", h.stored_pulses, p.completed_total);
}

int main(void){
    int t;
    printf("Scenario 1 - ACK lost after storage, across power cuts\n");
    printf("  V064 as delivered (seq only)\n");      fails=0; scen_ack_loss(false); t=fails;
    printf("  with P-11 (seq + content)\n");         fails=0; scen_ack_loss(true);
    printf("  -> failures: as-delivered %d, with P-11 %d\n\n", t, fails);

    printf("Scenario 2 - PIC reset restarts seq at 1\n");
    printf("  V064 as delivered (seq only)\n");      fails=0; scen_seq_collision(false); t=fails;
    printf("  with P-11 (seq + content)\n");         fails=0; scen_seq_collision(true);
    printf("  -> failures: as-delivered %d, with P-11 %d\n", t, fails);
    return 0;
}
