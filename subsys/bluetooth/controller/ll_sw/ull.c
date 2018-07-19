#include <stddef.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/types.h>
#include <device.h>
#include <entropy.h>
#include <bluetooth/hci.h>

#include "hal/cntr.h"
#include "hal/ccm.h"
#include "hal/ticker.h"

#if defined(CONFIG_SOC_FAMILY_NRF)
#include "hal/radio.h"
#endif /* CONFIG_SOC_FAMILY_NRF */

#include "util/util.h"
#include "util/mem.h"
#include "util/mfifo.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "pdu.h"
#include "ll.h"
#include "ull_types.h"
#include "lll.h"
#include "lll_filter.h"
#include "lll_adv.h"
#include "lll_scan.h"
#include "lll_conn.h"
#include "lll_tmp.h"
#include "ull.h"
#include "ull_adv_types.h"
#include "ull_scan_types.h"
#include "ull_conn_types.h"
#include "ull_internal.h"
#include "ull_adv_internal.h"
#include "ull_scan_internal.h"
#include "ull_conn_internal.h"
#include "ull_tmp_internal.h"

#include "common/log.h"
#include "hal/debug.h"

/* Define ticker nodes and user operations */
#if defined(CONFIG_BT_CTLR_LOW_LAT) && \
    (CONFIG_BT_CTLR_LLL_PRIO == CONFIG_BT_CTLR_ULL_LOW_PRIO)
#define TICKER_USER_LLL_OPS      (3 + 1)
#else
#define TICKER_USER_LLL_OPS      (2 + 1)
#endif /* CONFIG_BT_CTLR_LOW_LAT */
#define TICKER_USER_ULL_HIGH_OPS (1 + 1)
#define TICKER_USER_ULL_LOW_OPS  (1 + 1)
#define TICKER_USER_THREAD_OPS   (1 + 1)

#if defined(CONFIG_BT_BROADCASTER)
#define BT_ADV_TICKER_NODES ((TICKER_ID_ADV_LAST) - (TICKER_ID_ADV_STOP) + 1)
#else
#define BT_ADV_TICKER_NODES 0
#endif

#if defined(CONFIG_BT_OBSERVER)
#define BT_SCAN_TICKER_NODES ((TICKER_ID_SCAN_LAST) - (TICKER_ID_SCAN_STOP) + 1)
#else
#define BT_SCAN_TICKER_NODES 0
#endif

#if defined(CONFIG_BT_TMP)
#define BT_TMP_TICKER_NODES ((TICKER_ID_TMP_LAST) - (TICKER_ID_TMP_BASE) + 1)
#else
#define BT_TMP_TICKER_NODES 0
#endif

#if defined(CONFIG_SOC_FLASH_NRF_RADIO_SYNC)
#define FLASH_TICKER_NODES        1 /* No. of tickers reserved for flashing */
#define FLASH_TICKER_USER_APP_OPS 1 /* No. of additional ticker operations */
#else
#define FLASH_TICKER_NODES        0
#define FLASH_TICKER_USER_APP_OPS 0
#endif

#define TICKER_NODES              (TICKER_ID_ULL_BASE + \
				   BT_ADV_TICKER_NODES + \
				   BT_SCAN_TICKER_NODES + \
				   BT_TMP_TICKER_NODES + \
				   FLASH_TICKER_NODES)
#define TICKER_USER_APP_OPS       (TICKER_USER_THREAD_OPS + \
				   FLASH_TICKER_USER_APP_OPS)
#define TICKER_USER_OPS           (TICKER_USER_LLL_OPS + \
				   TICKER_USER_ULL_HIGH_OPS + \
				   TICKER_USER_ULL_LOW_OPS + \
				   TICKER_USER_THREAD_OPS + \
				   FLASH_TICKER_USER_APP_OPS)

/* Memory for ticker nodes/instances */
static u8_t MALIGN(4) _ticker_nodes[TICKER_NODES][TICKER_NODE_T_SIZE];

/* Memory for users/contexts operating on ticker module */
static u8_t MALIGN(4) _ticker_users[MAYFLY_CALLER_COUNT][TICKER_USER_T_SIZE];

/* Memory for user/context simultaneous API operations */
static u8_t MALIGN(4) _ticker_user_ops[TICKER_USER_OPS][TICKER_USER_OP_T_SIZE];

/* Semaphire to wakeup thread on ticker API callback */
static struct k_sem sem_ticker_api_cb;

/* Semaphore to wakeup thread on Rx-ed objects */
static struct k_sem *sem_recv;

/* Entropy device */
static struct device *dev_entropy;

/* Rx memq configuration defines */
#define TODO_PDU_RX_SIZE_MIN  64
#define TODO_PDU_RX_SIZE_MAX  64
#define TODO_PDU_RX_COUNT_MAX 10
#define TODO_PDU_RX_POOL_SIZE ((TODO_PDU_RX_SIZE_MAX) * \
			       (TODO_PDU_RX_COUNT_MAX))

#define TODO_LINK_RX_COUNT_MAX ((TODO_PDU_RX_COUNT_MAX) + 2)

/* prepare and done event FIFOs */
static MFIFO_DEFINE(prep, sizeof(struct lll_event), EVENT_PIPELINE_MAX);
static MFIFO_DEFINE(done, sizeof(void *), EVENT_PIPELINE_MAX);

static struct {
	void *free;
	u8_t pool[sizeof(struct node_rx_event_done) * EVENT_PIPELINE_MAX];
} mem_done;

static struct {
	void *free;
	u8_t pool[sizeof(memq_link_t) * EVENT_PIPELINE_MAX];
} mem_link_done;

#define PDU_RX_CNT (CONFIG_BT_CTLR_RX_BUFFERS + 3)

static MFIFO_DEFINE(pdu_rx_free, sizeof(void *), PDU_RX_CNT);

#define PDU_RX_SIZE_MIN MROUND(offsetof(struct node_rx_pdu, pdu) + \
			       (PDU_AC_SIZE_MAX + PDU_AC_SIZE_EXTRA))

#if defined(CONFIG_BT_RX_BUF_LEN)
#define PDU_RX_OCTETS_MAX (CONFIG_BT_RX_BUF_LEN - 11)
#else
#define PDU_RX_OCTETS_MAX 0
#endif

#define PDU_RX_POOL_SIZE (MROUND(offsetof(struct node_rx_pdu, pdu) + \
				 max((PDU_AC_SIZE_MAX + PDU_AC_SIZE_EXTRA), \
				 (offsetof(struct pdu_data, lldata) + \
				  PDU_RX_OCTETS_MAX))) * PDU_RX_CNT)

static struct {
	u8_t size; /* Runtime (re)sized info */

	void *free;
	u8_t pool[PDU_RX_POOL_SIZE];
} mem_pdu_rx;

#if defined(CONFIG_BT_MAX_CONN)
#define CONFIG_BT_CTLR_MAX_CONN CONFIG_BT_MAX_CONN
#else
#define CONFIG_BT_CTLR_MAX_CONN 0
#endif

#define LINK_RX_POOL_SIZE (sizeof(memq_link_t) * ((PDU_RX_CNT + 2) + \
						  CONFIG_BT_CTLR_MAX_CONN))
static struct {
	u8_t quota_pdu;

	void *free;
	u8_t pool[LINK_RX_POOL_SIZE];
} mem_link_rx;

static MEMQ_DECLARE(ull_rx);
static MEMQ_DECLARE(ll_rx);

static void *mark;

static inline int _init_reset(void);
static inline void _done_alloc(void);
static inline void _rx_alloc(u8_t max);
static void _rx_demux(void *param);
#if defined(CONFIG_BT_TMP)
static inline void _rx_demux_tx_ack(u16_t handle, memq_link_t *link,
			     struct node_tx *node_tx);
#endif /* CONFIG_BT_TMP */
static inline void _rx_demux_rx(memq_link_t *link, struct node_rx_hdr *rx);
static inline void _rx_demux_event_done(memq_link_t *link,
					struct node_rx_hdr *rx);
void _disabled_cb(void *param);

int ll_init(struct k_sem *sem_rx)
{
	int err;

	/* Store the semaphore to be used to wakeup Thread context */
	sem_recv = sem_rx;
	/* Get reference to entropy device */
	dev_entropy = device_get_binding(CONFIG_ENTROPY_NAME);
	if (!dev_entropy) {
		return -ENODEV;
	}


	/* Initialize counter */
	/* TODO: Bind and use counter driver? */
	cntr_init();

	/* Initialize Mayfly */
	mayfly_init();

	/* Initialize Ticker */
	_ticker_users[MAYFLY_CALL_ID_0][0] = TICKER_USER_LLL_OPS;
	_ticker_users[MAYFLY_CALL_ID_1][0] = TICKER_USER_ULL_HIGH_OPS;
	_ticker_users[MAYFLY_CALL_ID_2][0] = TICKER_USER_ULL_LOW_OPS;
	_ticker_users[MAYFLY_CALL_ID_PROGRAM][0] = TICKER_USER_APP_OPS;

	err = ticker_init(TICKER_INSTANCE_ID_CTLR,
			  TICKER_NODES, &_ticker_nodes[0],
			  MAYFLY_CALLER_COUNT, &_ticker_users[0],
			  TICKER_USER_OPS, &_ticker_user_ops[0],
			  hal_ticker_instance0_caller_id_get,
			  hal_ticker_instance0_sched,
			  hal_ticker_instance0_trigger_set);
	LL_ASSERT(!err);

	/* Initialize semaphore for ticker API blocking wait */
	k_sem_init(&sem_ticker_api_cb, 0, 1);

	/* Initialize LLL */
	err = lll_init();
	if (err) {
		return err;
	}

	/* Initialize ULL internals */
	/* TODO: globals? */

	/* Common to init and reset */
	err = _init_reset();
	if (err) {
		return err;
	}

#if defined(CONFIG_BT_BROADCASTER)
	err = lll_adv_init();
	if (err) {
		return err;
	}

	err = ull_adv_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_OBSERVER)
	err = lll_scan_init();
	if (err) {
		return err;
	}

	err = ull_scan_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CONN)
	err = lll_conn_init();
	if (err) {
		return err;
	}

	err = ull_conn_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CONN */

	/* Initialize state/roles */
#if defined(CONFIG_BT_TMP)
	err = lll_tmp_init();
	if (err) {
		return err;
	}

	err = ull_tmp_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_TMP */

	return  0;
}

void ll_reset(void)
{
	int err;

#if defined(CONFIG_BT_BROADCASTER)
	/* Reset adv state */
	err = ull_adv_reset();
	LL_ASSERT(!err);
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_OBSERVER)
	/* Reset scan state */
	err = ull_scan_reset();
	LL_ASSERT(!err);
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CONN)
	/* Reset conn role */
	err = ull_conn_reset();
	LL_ASSERT(!err);
#endif /* CONFIG_BT_CONN */

#if defined(CONFIG_BT_TMP)
	/* Reset tmp */
	err = ull_tmp_reset();
	LL_ASSERT(!err);
#endif /* CONFIG_BT_TMP */

	/* Re-initialize ULL internals */

	/* Re-initialize the prep mfifo */
	MFIFO_INIT(prep);

	/* Re-initialize the free done mfifo */
	MFIFO_INIT(done);

	/* Re-initialize the free rx mfifo */
	MFIFO_INIT(pdu_rx_free);

	/* Common to init and reset */
	err = _init_reset();
	LL_ASSERT(!err);
}

u8_t ll_rx_get(void **node_rx, u16_t *handle)
{
	memq_link_t *link;
	u8_t cmplt = 0;
	void *rx;

	link = memq_peek(memq_ll_rx.head, memq_ll_rx.tail, &rx);
	if (link) {
		/* TODO: tx complete get a handle */
		if (!cmplt) {
			/* TODO: release tx complete for handles */
			*node_rx = rx;
		} else {
			*node_rx = NULL;
		}
	} else {
		/* TODO: tx complete get a handle */
		*node_rx = NULL;
	}

	return cmplt;
}

void ll_rx_dequeue(void)
{
	struct node_rx_hdr *node_rx = NULL;
	memq_link_t *link;

	link = memq_dequeue(memq_ll_rx.tail, &memq_ll_rx.head,
			    (void **)&node_rx);
	LL_ASSERT(link);

	mem_release(link, &mem_link_rx.free);

	/* handle object specific clean up */
	switch (node_rx->type) {
#if defined(CONFIG_BT_OBSERVER) || \
	defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY) || \
	defined(CONFIG_BT_CTLR_PROFILE_ISR) || \
	defined(CONFIG_BT_CTLR_ADV_INDICATION) || \
	defined(CONFIG_BT_CTLR_SCAN_INDICATION)
#if defined(CONFIG_BT_OBSERVER)
	case NODE_RX_TYPE_REPORT:
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	case NODE_RX_TYPE_EXT_1M_REPORT:
	case NODE_RX_TYPE_EXT_CODED_REPORT:
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
	case NODE_RX_TYPE_SCAN_REQ:
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	/* fallthrough */
	case NODE_RX_TYPE_PROFILE:
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

#if defined(CONFIG_BT_CTLR_ADV_INDICATION)
	case NODE_RX_TYPE_ADV_INDICATION:
#endif /* CONFIG_BT_CTLR_ADV_INDICATION */

#if defined(CONFIG_BT_CTLR_SCAN_INDICATION)
	case NODE_RX_TYPE_SCAN_INDICATION:
#endif /* CONFIG_BT_CTLR_SCAN_INDICATION */
		LL_ASSERT(mem_link_rx.quota_pdu < PDU_RX_CNT);

		mem_link_rx.quota_pdu++;
		break;
#endif /* CONFIG_BT_OBSERVER ||
	* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY ||
	* CONFIG_BT_CTLR_PROFILE_ISR ||
	* CONFIG_BT_CTLR_ADV_INDICATION ||
	* CONFIG_BT_CTLR_SCAN_INDICATION
	*/

#if defined(CONFIG_BT_CONN)
	case NODE_RX_TYPE_CONNECTION:
		break;
#endif /* CONFIG_BT_CONN */

	default:
		LL_ASSERT(0);
		break;
	}

	if (0) {
#if defined(CONFIG_BT_CONN)
	} else if (node_rx->type == NODE_RX_TYPE_CONNECTION) {
		struct node_rx_cc *cc;

		cc = (void *)((struct node_rx_pdu *)node_rx)->pdu;
		if ((cc->status == 0x3c) || cc->role) {
			struct ll_adv_set *adv;

			adv = ull_adv_is_enabled_get(0);
			LL_ASSERT(adv);

			if (cc->status == 0x3c) {
				LL_ASSERT(adv->lll.conn);

				ll_conn_release(adv->lll.conn->hdr.parent);
				adv->lll.conn = NULL;
			}

			adv->is_enabled = 0;
		} else {
			/* TODO: unset initiator enable flag */
		}

		if (IS_ENABLED(CONFIG_BT_CTLR_PRIVACY)) {
			u8_t bm;

			bm = (ull_scan_is_enabled(0) << 1) |
			     ull_adv_is_enabled(0);

			if (!bm) {
				ll_adv_scan_state_cb(0);
			}
		}
#endif /* CONFIG_BT_CONN */

#if defined(CONFIG_BT_HCI_MESH_EXT)
	} else if (node_rx->type == NODE_RX_TYPE_MESH_ADV_CPLT) {
		struct ll_adv_set *adv;
		struct ll_scan_set *scan;

		adv = ull_adv_is_enabled_get(0);
		LL_ASSERT(adv);
		adv->is_enabled = 0;

		scan = ull_scan_is_enabled_get(0);
		LL_ASSERT(scan);
		scan->is_enabled = 0;

		ll_adv_scan_state_cb(0);
#endif /* CONFIG_BT_HCI_MESH_EXT */
	}
}

void ll_rx_mem_release(void **node_rx)
{
	struct node_rx_hdr *_node_rx;

	_node_rx = *node_rx;
	while (_node_rx) {
		struct node_rx_hdr *_node_rx_free;

		_node_rx_free = _node_rx;
		_node_rx = _node_rx->next;

		switch (_node_rx_free->type) {
		case NODE_RX_TYPE_DC_PDU:
		case NODE_RX_TYPE_REPORT:

#if defined(CONFIG_BT_CTLR_ADV_EXT)
		case NODE_RX_TYPE_EXT_1M_REPORT:
		case NODE_RX_TYPE_EXT_CODED_REPORT:
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
		case NODE_RX_TYPE_SCAN_REQ:
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

#if defined(CONFIG_BT_CONN)
		case NODE_RX_TYPE_CONNECTION:
		case NODE_RX_TYPE_CONN_UPDATE:
		case NODE_RX_TYPE_ENC_REFRESH:

#if defined(CONFIG_BT_CTLR_LE_PING)
		case NODE_RX_TYPE_APTO:
#endif /* CONFIG_BT_CTLR_LE_PING */

		case NODE_RX_TYPE_CHAN_SEL_ALGO:

#if defined(CONFIG_BT_CTLR_PHY)
		case NODE_RX_TYPE_PHY_UPDATE:
#endif /* CONFIG_BT_CTLR_PHY */

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
		case NODE_RX_TYPE_RSSI:
#endif /* CONFIG_BT_CTLR_CONN_RSSI */
#endif /* CONFIG_BT_CONN */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
		case NODE_RX_TYPE_PROFILE:
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

#if defined(CONFIG_BT_CTLR_ADV_INDICATION)
		case NODE_RX_TYPE_ADV_INDICATION:
#endif /* CONFIG_BT_CTLR_ADV_INDICATION */

#if defined(CONFIG_BT_CTLR_SCAN_INDICATION)
		case NODE_RX_TYPE_SCAN_INDICATION:
#endif /* CONFIG_BT_CTLR_SCAN_INDICATION */

#if defined(CONFIG_BT_HCI_MESH_EXT)
		case NODE_RX_TYPE_MESH_ADV_CPLT:
		case NODE_RX_TYPE_MESH_REPORT:
#endif /* CONFIG_BT_HCI_MESH_EXT */

			mem_release(_node_rx_free, &mem_pdu_rx.free);
			break;

#if defined(CONFIG_BT_CONN)
		case NODE_RX_TYPE_TERMINATE:
			/* FIXME: */
#if 0
			struct connection *conn;

			conn = mem_get(_radio.conn_pool, CONNECTION_T_SIZE,
				       _node_rx_free->hdr.handle);

			mem_release(conn, &_radio.conn_free);
			break;
#endif
#endif /* CONFIG_BT_CONN */

		case NODE_RX_TYPE_NONE:
		case NODE_RX_TYPE_EVENT_DONE:
		default:
			LL_ASSERT(0);
			break;
		}
	}

	*node_rx = _node_rx;

	_rx_alloc(UINT8_MAX);
}

void *ll_rx_link_alloc(void)
{
	return mem_acquire(&mem_link_rx.free);
}

void ll_rx_link_release(void *link)
{
	mem_release(link, &mem_link_rx.free);
}

void *ll_rx_alloc(void)
{
	return mem_acquire(&mem_pdu_rx.free);
}

void ll_rx_release(void *node_rx)
{
	mem_release(node_rx, &mem_pdu_rx.free);
}

void ll_rx_put(memq_link_t *link, void *rx)
{
	/* TODO: link the tx complete */

	/* Enqueue the Rx object */
	memq_enqueue(link, rx, &memq_ll_rx.tail);
}

void ll_rx_sched(void)
{
	k_sem_give(sem_recv);
}

void ll_timeslice_ticker_id_get(u8_t * const instance_index,
				u8_t * const user_id)
{
	*instance_index = TICKER_INSTANCE_ID_CTLR;
	*user_id = (TICKER_NODES - FLASH_TICKER_NODES);
}

void ll_radio_state_abort(void)
{
	static memq_link_t _link;
	static struct mayfly _mfy = {0, 0, &_link, NULL, lll_disable};
	u32_t ret;

	ret = mayfly_enqueue(TICKER_USER_ID_THREAD, TICKER_USER_ID_LLL, 0,
			     &_mfy);
	LL_ASSERT(!ret);
}

u32_t ll_radio_state_is_idle(void)
{
	return radio_is_idle();
}

void ull_ticker_status_give(u32_t status, void *param)
{
	*((u32_t volatile *)param) = status;

	k_sem_give(&sem_ticker_api_cb);
}

u32_t ull_ticker_status_take(u32_t ret, u32_t volatile *ret_cb)
{
	if (ret == TICKER_STATUS_BUSY) {
		k_sem_take(&sem_ticker_api_cb, K_FOREVER);
	}

	return *ret_cb;
}

void *ull_disable_mark(void *param)
{
	if (!mark) {
		mark = param;
	}

	return mark;
}

void *ull_disable_unmark(void *param)
{
	if (mark && mark == param) {
		mark = NULL;
	}

	return param;
}

void *ull_disable_mark_get(void)
{
	return mark;
}

int ull_disable(void *lll)
{
	static memq_link_t _link;
	static struct mayfly _mfy = {0, 0, &_link, NULL, lll_disable};
	struct ull_hdr *hdr;
	struct k_sem sem;
	u32_t ret;

	hdr = ULL_HDR(((struct lll_hdr *)lll)->parent);
	if (!hdr || !hdr->ref) {
		return ULL_STATUS_SUCCESS;
	}

	k_sem_init(&sem, 0, 1);
	hdr->disabled_param = &sem;
	hdr->disabled_cb = _disabled_cb;

	_mfy.param = lll;
	ret = mayfly_enqueue(TICKER_USER_ID_THREAD, TICKER_USER_ID_LLL, 0,
			     &_mfy);
	LL_ASSERT(!ret);

	return k_sem_take(&sem, K_FOREVER);
}

void *ull_pdu_rx_alloc_peek(u8_t count)
{
	if (count > MFIFO_AVAIL_COUNT_GET(pdu_rx_free)) {
		return NULL;
	}

	return MFIFO_DEQUEUE_PEEK(pdu_rx_free);
}

void *ull_pdu_rx_alloc_peek_iter(u8_t *idx)
{
	return *(void **)MFIFO_DEQUEUE_ITER_GET(pdu_rx_free, idx);
}

void *ull_pdu_rx_alloc(void)
{
	return MFIFO_DEQUEUE(pdu_rx_free);
}

void ull_rx_put(memq_link_t *link, void *rx)
{
#if defined(CONFIG_BT_TMP)
	struct node_rx_hdr *rx_hdr = rx;

	/* Serialize Tx ack with Rx enqueue by storing reference to
	 * last element index in Tx ack FIFO.
	 */
	rx_hdr->ack_last = lll_tmp_ack_last_idx_get();
#endif /* CONFIG_BT_TMP */

	/* Enqueue the Rx object */
	memq_enqueue(link, rx, &memq_ull_rx.tail);
}

void ull_rx_sched(void)
{
	static memq_link_t _link;
	static struct mayfly _mfy = {0, 0, &_link, NULL, _rx_demux};

	/* Kick the ULL (using the mayfly, tailchain it) */
	mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_ULL_HIGH, 1, &_mfy);
}

int ull_prepare_enqueue(lll_is_abort_cb_t is_abort_cb,
			lll_abort_cb_t abort_cb,
			struct lll_prepare_param *prepare_param,
			lll_prepare_cb_t prepare_cb, int prio,
			u8_t is_resume)
{
	struct lll_event *e;
	u8_t idx;

	idx = MFIFO_ENQUEUE_GET(prep, (void **)&e);
	if (!e) {
		return -ENOBUFS;
	}

	memcpy(&e->prepare_param, prepare_param, sizeof(e->prepare_param));
	e->prepare_cb = prepare_cb;
	e->is_abort_cb = is_abort_cb;
	e->abort_cb = abort_cb;
	e->prio = prio;
	e->is_resume = is_resume;
	e->is_aborted = 0;

	MFIFO_ENQUEUE(prep, idx);

	return 0;
}

void *ull_prepare_dequeue_get(void)
{
	return MFIFO_DEQUEUE_GET(prep);
}

void *ull_prepare_dequeue_iter(u8_t *idx)
{
	return MFIFO_DEQUEUE_ITER_GET(prep, idx);
}

void *ull_event_done(void *param)
{
	struct node_rx_event_done *done;
	memq_link_t *link;

	done = MFIFO_DEQUEUE(done);
	if (!done) {
		return NULL;
	}

	link = done->hdr.link;
	done->hdr.link = NULL;

	done->hdr.type = NODE_RX_TYPE_EVENT_DONE;
	done->param = param;

	ull_rx_put(link, done);
	ull_rx_sched();

	return done;
}

u8_t ull_entropy_get(u8_t len, u8_t *rand)
{
	return entropy_get_entropy_isr(dev_entropy, rand, len, 0);
}

static inline int _init_reset(void)
{
	memq_link_t *link;

	/* Initialize done pool. */
	mem_init(mem_done.pool, sizeof(struct node_rx_event_done),
		 EVENT_PIPELINE_MAX, &mem_done.free);

	/* Initialize done link pool. */
	mem_init(mem_link_done.pool, sizeof(memq_link_t), EVENT_PIPELINE_MAX,
		 &mem_link_done.free);

	/* Allocate done buffers */
	_done_alloc();

	/* Initialize rx pool. */
	mem_pdu_rx.size = PDU_RX_SIZE_MIN;
	mem_init(mem_pdu_rx.pool, mem_pdu_rx.size,
		 sizeof(mem_pdu_rx.pool) / mem_pdu_rx.size,
		 &mem_pdu_rx.free);

	/* Initialize rx link pool. */
	mem_init(mem_link_rx.pool, sizeof(memq_link_t),
		 sizeof(mem_link_rx.pool) / sizeof(memq_link_t),
		 &mem_link_rx.free);

	/* Acquire a link to initialize ull rx memq */
	link = mem_acquire(&mem_link_rx.free);
	LL_ASSERT(link);

	/* Initialize ull rx memq */
	MEMQ_INIT(ull_rx, link);

	/* Acquire a link to initialize ll rx memq */
	link = mem_acquire(&mem_link_rx.free);
	LL_ASSERT(link);

	/* Initialize ll rx memq */
	MEMQ_INIT(ll_rx, link);

	/* Allocate rx free buffers */
	mem_link_rx.quota_pdu = PDU_RX_CNT;
	_rx_alloc(UINT8_MAX);

	return 0;
}

static inline void _done_alloc(void)
{
	u8_t idx;

	while (MFIFO_ENQUEUE_IDX_GET(done, &idx)) {
		memq_link_t *link;
		struct node_rx_hdr *rx;

		link = mem_acquire(&mem_link_done.free);
		if (!link) {
			break;
		}

		rx = mem_acquire(&mem_done.free);
		if (!rx) {
			mem_release(link, &mem_link_done.free);
			break;
		}

		rx->link = link;

		MFIFO_BY_IDX_ENQUEUE(done, idx, rx);
	}
}

static inline void *_done_release(memq_link_t *link,
				  struct node_rx_event_done *done)
{
	u8_t idx;

	done->hdr.link = link;

	if (!MFIFO_ENQUEUE_IDX_GET(done, &idx)) {
		return NULL;
	}

	MFIFO_BY_IDX_ENQUEUE(done, idx, done);

	return done;
}

static inline void _rx_alloc(u8_t max)
{
	u8_t idx;

	if (max > mem_link_rx.quota_pdu) {
		max = mem_link_rx.quota_pdu;
	}

	while ((max--) && MFIFO_ENQUEUE_IDX_GET(pdu_rx_free, &idx)) {
		memq_link_t *link;
		struct node_rx_hdr *rx;

		link = mem_acquire(&mem_link_rx.free);
		if (!link) {
			break;
		}

		rx = mem_acquire(&mem_pdu_rx.free);
		if (!rx) {
			mem_release(link, &mem_link_rx.free);
			break;
		}

		rx->link = link;

		MFIFO_BY_IDX_ENQUEUE(pdu_rx_free, idx, rx);

		mem_link_rx.quota_pdu--;
	}
}

static void _rx_demux(void *param)
{
	memq_link_t *link;

	do {
		struct node_rx_hdr *rx;

		link = memq_peek(memq_ull_rx.head, memq_ull_rx.tail,
				 (void **)&rx);
		if (link) {
#if defined(CONFIG_BT_TMP)
			struct node_tx *node_tx;
			memq_link_t *link_tx;
			u16_t handle;

			LL_ASSERT(rx);

			link_tx = lll_tmp_ack_by_last_peek(rx->ack_last,
							   &handle, &node_tx);
			if (link_tx) {
				_rx_demux_tx_ack(handle, link_tx, node_tx);
			} else {
#endif /* CONFIG_BT_TMP */
				_rx_demux_rx(link, rx);
#if defined(CONFIG_BT_TMP)
			}
		} else {
			struct node_tx *node_tx;
			u16_t handle;

			link = lll_tmp_ack_peek(&handle, &node_tx);
			if (link) {
				_rx_demux_tx_ack(handle, link, node_tx);
			}
#endif /* CONFIG_BT_TMP */
		}
	} while (link);
}

#if defined(CONFIG_BT_TMP)
static inline void _rx_demux_tx_ack(u16_t handle, memq_link_t *link,
			     struct node_tx *node_tx)
{
	lll_tmp_ack_dequeue();

	ull_tmp_link_tx_release(link);
}
#endif /* CONFIG_BT_TMP */

static inline void _rx_demux_rx(memq_link_t *link, struct node_rx_hdr *rx)
{
	/* NOTE: dequeue before releasing resources */
	memq_dequeue(memq_ull_rx.tail, &memq_ull_rx.head, NULL);

	/* Demux Rx objects */
	switch (rx->type) {
	case NODE_RX_TYPE_EVENT_DONE:
	{
		_rx_demux_event_done(link, rx);
	}
	break;

#if defined(CONFIG_BT_OBSERVER) || \
	defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY) || \
	defined(CONFIG_BT_CTLR_PROFILE_ISR) || \
	defined(CONFIG_BT_CTLR_ADV_INDICATION) || \
	defined(CONFIG_BT_CTLR_SCAN_INDICATION)
#if defined(CONFIG_BT_OBSERVER)
	case NODE_RX_TYPE_REPORT:
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	case NODE_RX_TYPE_EXT_1M_REPORT:
	case NODE_RX_TYPE_EXT_CODED_REPORT:
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_SCAN_REQ_NOTIFY)
	case NODE_RX_TYPE_SCAN_REQ:
#endif /* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	/* fallthrough */
	case NODE_RX_TYPE_PROFILE:
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

#if defined(CONFIG_BT_CTLR_ADV_INDICATION)
	case NODE_RX_TYPE_ADV_INDICATION:
#endif /* CONFIG_BT_CTLR_ADV_INDICATION */

#if defined(CONFIG_BT_CTLR_SCAN_INDICATION)
	case NODE_RX_TYPE_SCAN_INDICATION:
#endif /* CONFIG_BT_CTLR_SCAN_INDICATION */
	{
		ll_rx_put(link, rx);
		ll_rx_sched();
	}
	break;
#endif /* CONFIG_BT_OBSERVER ||
	* CONFIG_BT_CTLR_SCAN_REQ_NOTIFY ||
	* CONFIG_BT_CTLR_PROFILE_ISR ||
	* CONFIG_BT_CTLR_ADV_INDICATION ||
	* CONFIG_BT_CTLR_SCAN_INDICATION
	*/

#if defined(CONFIG_BT_CONN)
	/* fallthrough */
	case NODE_RX_TYPE_CONNECTION:
	{
		ull_conn_setup(link, rx);
	}
	break;

	case NODE_RX_TYPE_DC_PDU:
	{
		/* TODO: process and pass through to thread */
	}
	break;
#endif /* CONFIG_BT_CONN */

	default:
	{
		LL_ASSERT(0);
	}
	break;
	}
}

static inline void _rx_demux_event_done(memq_link_t *link,
					struct node_rx_hdr *rx)
{
	struct node_rx_event_done *done = (void *)rx;
	struct ull_hdr *ull_hdr;
	struct lll_event *next;

	/* Get the ull instance */
	ull_hdr = done->param;

	/* release done */
	_done_release(link, done);

	/* dequeue prepare pipeline */
	next = ull_prepare_dequeue_get();
	while (next) {
		u8_t is_resume = next->is_resume;

		if (!next->is_aborted) {
			static memq_link_t _link;
			static struct mayfly _mfy = {0, 0, &_link, NULL,
						     lll_resume};
			u32_t ret;

			_mfy.param = next;
			ret = mayfly_enqueue(TICKER_USER_ID_ULL_HIGH,
					     TICKER_USER_ID_LLL, 0, &_mfy);
			LL_ASSERT(!ret);
		}

		MFIFO_DEQUEUE(prep);

		next = ull_prepare_dequeue_get();

		if (!next || next->is_resume || !is_resume) {
			break;
		}
	}

	/* ull instance will resume, dont decrement ref */
	if (!ull_hdr) {
		return;
	}

	/* Decrement prepare reference */
	LL_ASSERT(ull_hdr->ref);
	ull_hdr->ref--;

	/* If disable initiated, signal the semaphore */
	if (!ull_hdr->ref && ull_hdr->disabled_cb) {
		ull_hdr->disabled_cb(ull_hdr->disabled_param);
	}
}

void _disabled_cb(void *param)
{
	k_sem_give(param);
}
