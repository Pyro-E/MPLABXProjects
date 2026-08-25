/* V024 protocol v2 audit model.
 *
 * The project's stated contract is that the PIC guarantees the AMOUNT of water
 * and the Photon places it in time (hourly.h). v2 makes that amount auditable by
 * carrying it as a POSITION on the PIC's lifetime pulse axis instead of a bare
 * delta.
 *
 * This model checks that the audit actually catches what it is supposed to, and
 * that it never fires on healthy traffic. It runs the same traffic through the
 * v1 receiver (delta only, no audit possible) and the v2 receiver.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---------------- PIC V024 ---------------- */
typedef struct {
    uint32_t pulses, caps;            /* lifetime axis (resets on PIC reset) */
    uint16_t boot_id;
    uint32_t mark_pul, mark_cap;      /* last ACKed position                 */
    bool     cut_valid; uint32_t cut_pul, cut_cap;
    bool     pending;
    uint32_t b_bpul, b_epul, b_bcap, b_ecap; uint16_t b_n;
} Pic;

typedef struct {                       /* one RSP_DATA frame                 */
    uint16_t boot_id;
    uint32_t bpul, epul, bcap, ecap;
    uint16_t n;
} Frame;

static void pic_boot(Pic *p, bool cold){
    uint16_t keep = p->boot_id;
    memset(p, 0, sizeof(*p));
    p->boot_id = (uint16_t)(cold ? 1u : keep + 1u);
}
static void pic_capture(Pic *p, uint32_t pulses){ p->pulses += pulses; p->caps++; }
static void pic_on(Pic *p){ p->cut_pul = p->pulses; p->cut_cap = p->caps; p->cut_valid = true; }
static void pic_off(Pic *p){ p->cut_valid = false; }

static Frame pic_req(Pic *p){
    Frame f;
    if (!p->pending) {
        if (!p->cut_valid) pic_on(p);
        p->b_bpul = p->mark_pul; p->b_epul = p->cut_pul;
        p->b_bcap = p->mark_cap; p->b_ecap = p->cut_cap;
        p->b_n    = (uint16_t)(p->b_ecap - p->b_bcap);
        p->pending = (p->b_n > 0u);        /* V023b: an empty batch is never pending */
    }
    f.boot_id = p->boot_id;
    f.bpul = p->b_bpul; f.epul = p->b_epul;
    f.bcap = p->b_bcap; f.ecap = p->b_ecap; f.n = p->b_n;
    return f;
}
static void pic_ack(Pic *p, uint32_t epul, uint32_t ecap){
    if (!p->pending) return;
    if (epul != p->b_epul || ecap != p->b_ecap) return;   /* names another batch */
    p->mark_pul = p->b_epul; p->mark_cap = p->b_ecap;
    p->pending = false;
}

/* ---------------- Photon receiver ---------------- */
typedef struct {
    uint32_t accounted;               /* pulses durably accounted            */
    uint32_t stores, dups, gaps, restarts;
    uint16_t last_boot; uint32_t last_bpul, last_epul, last_ecap; bool have_last;
    bool     v2;                      /* audit available                     */
} Pho;

static void pho_init(Pho *h, bool v2){ memset(h,0,sizeof(*h)); h->v2 = v2; }

/* returns true if an ACK should be sent */
static bool pho_rx(Pho *h, Frame f, uint32_t *gap_pulses){
    *gap_pulses = 0;
    uint32_t imp = f.epul - f.bpul;             /* DERIVED, cannot disagree   */
    if (f.n == 0u) return false;                /* nothing to store, no ACK   */

    if (!h->v2) {                               /* v1: store blindly          */
        h->accounted += imp; h->stores++;
        return true;
    }

    if (h->have_last) {
        if (f.boot_id != h->last_boot) {
            h->restarts++;                      /* axis replaced              */
            h->have_last = false;               /* nothing to join to         */
        } else if (f.bpul == h->last_bpul && f.epul == h->last_epul &&
                   f.ecap == h->last_ecap) {
            h->dups++;              /* identical endpoints = the SAME report   */
            return true;            /* skip the store, repeat the ACK          */
        } else if (f.bpul > h->last_epul) {
            *gap_pulses = f.bpul - h->last_epul;
            h->gaps++;                          /* hole of KNOWN size         */
        }
    }
    h->accounted += imp; h->stores++;
    h->last_boot = f.boot_id; h->last_bpul = f.bpul;
    h->last_epul = f.epul;    h->last_ecap = f.ecap;
    h->have_last = true;
    return true;
}

/* ---------------- scenarios ---------------- */
static int fails;
static void chk(bool ok, const char *w){ if(!ok){ printf("      FAIL: %s\n", w); fails++; } }

/* Healthy traffic + an ACK lost after storage + a PIC reset that restarts the
 * axis at 0 with a colliding position. */
static void run(bool v2, const char *tag){
    Pic p; Pho h; uint32_t gap;
    memset(&p,0,sizeof(p)); pic_boot(&p, true); pho_init(&h, v2);

    printf("    %s\n", tag);

    /* 4 clean sessions */
    for (int s = 0; s < 4; s++) {
        for (int c = 0; c < 3; c++) pic_capture(&p, 5);
        pic_on(&p);
        Frame f = pic_req(&p);
        if (pho_rx(&h, f, &gap)) pic_ack(&p, f.epul, f.ecap);
        chk(gap == 0, "spurious gap reported on healthy traffic");
        pic_off(&p);
    }

    /* session 5: stored, but the ACK is lost -> retransmitted next session */
    for (int c = 0; c < 3; c++) pic_capture(&p, 5);
    pic_on(&p); Frame f5 = pic_req(&p); pho_rx(&h, f5, &gap); /* ACK lost */
    pic_off(&p);

    for (int c = 0; c < 3; c++) pic_capture(&p, 5);
    pic_on(&p); Frame f6 = pic_req(&p);
    if (pho_rx(&h, f6, &gap)) pic_ack(&p, f6.epul, f6.ecap);
    pic_off(&p);

    /* drain whatever is left */
    for (int k = 0; k < 4; k++) {
        pic_on(&p); Frame f = pic_req(&p);
        if (pho_rx(&h, f, &gap)) pic_ack(&p, f.epul, f.ecap);
        pic_off(&p);
    }
    uint32_t before_reset = p.pulses;
    chk(h.accounted == before_reset, "pulses lost or double-counted");

    /* PIC reset: the axis restarts at 0 - the position that WOULD collide */
    uint32_t acc_before = h.accounted;
    pic_boot(&p, false);
    for (int c = 0; c < 4; c++) pic_capture(&p, 7);
    pic_on(&p); Frame fr = pic_req(&p);
    if (pho_rx(&h, fr, &gap)) pic_ack(&p, fr.epul, fr.ecap);
    pic_off(&p);

    chk(h.accounted == acc_before + p.pulses, "new data after a PIC reset was lost");
    if (v2) chk(h.restarts == 1, "the PIC restart was not detected");

    printf("      accounted=%u  stores=%u  duplicates=%u  gaps=%u  restarts=%u\n",
           h.accounted, h.stores, h.dups, h.gaps, h.restarts);
    printf("      (expected total = %u before reset + %u after = %u)\n",
           before_reset, p.pulses, before_reset + p.pulses);
}

int main(void){
    printf("Delivery under ACK loss and a PIC reset\n");
    fails = 0; run(false, "v1 receiver (delta only - nothing can be checked)");
    int v1 = fails;
    fails = 0; run(true,  "v2 receiver (position + audit)");
    printf("\n  v1 failures: %d      v2 failures: %d\n\n", v1, fails);

    /* Can the audit see a hole it is not told about?
     * Simulate the PIC losing 40 pulses of unreported data (a ring wrap that
     * outran the consumer), i.e. the mark advancing without delivery. */
    printf("An undelivered hole is MEASURED, not guessed\n");
    {
        Pic p; Pho h; uint32_t gap; memset(&p,0,sizeof(p));
        pic_boot(&p, true); pho_init(&h, true);
        for (int c = 0; c < 3; c++) pic_capture(&p, 5);
        pic_on(&p); Frame a = pic_req(&p);
        pho_rx(&h, a, &gap); pic_ack(&p, a.epul, a.ecap); pic_off(&p);

        p.mark_pul += 40; p.mark_cap += 8;      /* 40 pulses dropped, never sent */
        for (int c = 0; c < 3; c++) pic_capture(&p, 5);
        pic_on(&p); Frame b = pic_req(&p);
        pho_rx(&h, b, &gap); pic_ack(&p, b.epul, b.ecap); pic_off(&p);

        printf("    reported gap = %u pulses (actual = 40)  -> %s\n",
               gap, gap == 40u ? "EXACT" : "WRONG");
        printf("    a v1 receiver would have seen nothing at all.\n");
    }
    return 0;
}
