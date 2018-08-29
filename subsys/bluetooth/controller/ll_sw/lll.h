/* NOTE: Definitions used between ULL and LLL implementations */

#define TICKER_INSTANCE_ID_CTLR 0
#define TICKER_USER_ID_LLL      MAYFLY_CALL_ID_0
#define TICKER_USER_ID_ULL_HIGH MAYFLY_CALL_ID_1
#define TICKER_USER_ID_ULL_LOW  MAYFLY_CALL_ID_2
#define TICKER_USER_ID_THREAD   MAYFLY_CALL_ID_PROGRAM

#define EVENT_PIPELINE_MAX            4

#define ULL_HDR(p) ((void *)((u8_t *)(p) + sizeof(struct evt_hdr)))
#define EVT_HDR(lll) ((void *)((struct lll_hdr *)(lll))->parent)

#if defined(CONFIG_BT_CTLR_XTAL_ADVANCED)
#define XON_BITMASK BIT(31) /* XTAL has been retained from previous prepare */
#endif /* CONFIG_BT_CTLR_XTAL_ADVANCED */

#if defined(CONFIG_BT_BROADCASTER) && defined(CONFIG_BT_ADV_SET)
#define CONFIG_BT_ADV_MAX (CONFIG_BT_ADV_SET + 1)
#else
#define CONFIG_BT_ADV_MAX 1
#endif

enum {
	TICKER_ID_LLL_PREEMPT = 0,

#if defined(CONFIG_BT_TMP)
	TICKER_ID_TMP_BASE,
	TICKER_ID_TMP_LAST = ((TICKER_ID_TMP_BASE) + (CONFIG_BT_TMP_MAX) - 1),
#endif /* CONFIG_BT_TMP */

#if defined(CONFIG_BT_BROADCASTER)
	TICKER_ID_ADV_STOP,
	TICKER_ID_ADV_BASE,
#if defined(CONFIG_BT_CTLR_ADV_EXT) || defined(CONFIG_BT_HCI_MESH_EXT)
	TICKER_ID_ADV_LAST = ((TICKER_ID_ADV_BASE) + (CONFIG_BT_ADV_MAX) - 1),
#endif /* !CONFIG_BT_CTLR_ADV_EXT || !CONFIG_BT_HCI_MESH_EXT */
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_OBSERVER)
	TICKER_ID_SCAN_STOP,
	TICKER_ID_SCAN_BASE,
	TICKER_ID_SCAN_LAST = TICKER_ID_SCAN_BASE,
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CONN)
	TICKER_ID_CONN_BASE,
#endif /* CONFIG_BT_CONN */

	TICKER_ID_MAX,
};

#if defined(CONFIG_BT_BROADCASTER) && !defined(CONFIG_BT_CTLR_ADV_EXT) && \
    !defined(CONFIG_BT_HCI_MESH_EXT)
#define TICKER_ID_ADV_LAST TICKER_ID_ADV_BASE
#endif

#define TICKER_ID_ULL_BASE ((TICKER_ID_LLL_PREEMPT) + 1)

enum ull_status {
	ULL_STATUS_SUCCESS,
	ULL_STATUS_FAILURE,
	ULL_STATUS_BUSY,
};

struct evt_hdr {
	u32_t ticks_xtal_to_start;
	u32_t ticks_active_to_start;
	u32_t ticks_preempt_to_start;
	u32_t ticks_slot;
};

struct ull_hdr {
	u8_t ref;
	void (*disabled_cb)(void *param);
	void *disabled_param;
};

struct lll_hdr {
	void *parent;
};

struct lll_prepare_param {
	u32_t ticks_at_expire;
	u32_t remainder;
	u16_t lazy;
	void  *param;
};

typedef int (*lll_prepare_cb_t)(struct lll_prepare_param *prepare_param);
typedef int (*lll_is_abort_cb_t)(void *next, int prio, void *curr,
				 lll_prepare_cb_t *resume_cb, int *resume_prio);
typedef void (*lll_abort_cb_t)(struct lll_prepare_param *prepare_param,
			       void *param);

struct lll_event {
	struct lll_prepare_param prepare_param;
	lll_prepare_cb_t         prepare_cb;
	lll_is_abort_cb_t        is_abort_cb;
	lll_abort_cb_t           abort_cb;
	int                      prio;
	u8_t			 is_resume:1;
	u8_t			 is_aborted:1;
};

static inline void lll_hdr_init(void *lll, void *parent)
{
	struct lll_hdr *hdr = lll;

	hdr->parent = parent;
}

int lll_init(void);
int lll_prepare(lll_is_abort_cb_t is_abort_cb, lll_abort_cb_t abort_cb,
		lll_prepare_cb_t prepare_cb, int prio,
		struct lll_prepare_param *prepare_param);
void lll_resume(void *param);
void lll_disable(void *param);

int ull_prepare_enqueue(lll_is_abort_cb_t is_abort_cb,
			       lll_abort_cb_t abort_cb,
			       struct lll_prepare_param *prepare_param,
			       lll_prepare_cb_t prepare_cb, int prio,
			       u8_t is_resume);
void *ull_prepare_dequeue_get(void);
void *ull_prepare_dequeue_iter(u8_t *idx);
void *ull_pdu_rx_alloc_peek(u8_t count);
void *ull_pdu_rx_alloc(void);
void ull_rx_put(memq_link_t *link, void *rx);
void ull_rx_sched(void);
void *ull_event_done(void *param);
