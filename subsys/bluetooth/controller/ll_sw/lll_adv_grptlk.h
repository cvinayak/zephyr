// int lll_adv_iso_init(void);
// int lll_adv_iso_reset(void);
void lll_adv_grptlk_create_prepare(void *param);
void lll_adv_grptlk_prepare(void *param);

extern struct lll_adv_iso_stream *ull_adv_grptlk_lll_stream_get(uint16_t handle);
extern void ll_iso_rx_put(memq_link_t *link, void *rx);

// extern void ull_adv_iso_lll_biginfo_fill(struct pdu_adv *pdu, struct lll_adv_sync *lll_sync);
