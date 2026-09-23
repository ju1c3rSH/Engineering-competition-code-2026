/* 底盘路线自测: 不接 Brain, 底盘自己按原版状态机的 nav 顺序逐条下发 REQ_NAV
 * (1→6→2→3→4→7→5, 不走 QR), 跑完一条自动发下一条, 到返回区停车。
 * 走 Nav_DoRequest 同一入口(接缝 2 在 Control_Car.c 展开), 核对的就是原版每张表。 */
#include "selftest.h"

#if SELFTEST_EN

#include "Control_Car.h"

/* 0=段表默认速度; 1/2/3=低/中/高强制覆盖。
 * 开停等(PAUSE_MS>0)必须非 0: 插桩只发生在拷贝路径上, 而 gear=0 走 flash 原表 */
#define SELFTEST_GEAR            0u

/* 停等时长: 0 = 不插 WAIT(纯比赛剖面); 非 0 时在每张大段末插 SEG_PAUSE 供读落点 */
#define SELFTEST_PAUSE_MS        0u
/* 插桩粒度: 0 = 只在大段末插一次; 1 = 逐段插(旧策略, 起停 18 次, 白走里程 3.5 倍) */
#define SELFTEST_PAUSE_EACH_SEG  0u

#if (SELFTEST_PAUSE_MS > 0u) && (SELFTEST_GEAR == 0u)
#error "停等插桩只在拷贝路径上生效: SELFTEST_PAUSE_MS>0 时必须把 SELFTEST_GEAR 设为非 0"
#endif

#define SEG_WAIT_MS(ms) { .type = MF_SEG_WAIT, .reset_pid = 0, .p.wait = { (ms) } }
#define SEG_PAUSE       SEG_WAIT_MS((float)SELFTEST_PAUSE_MS)

/* 自测命令序列: 原版状态机的 nav 顺序(不走 QR, 因为自测没有扫码结果) */
static const uint8_t selftest_seq[] =
{
	1u,   /* NAV_TO_BOMB         排爆区去程 */
	6u,   /* NAV_REBASE_BOMB     末段转弯+低速进作业点+航向重锚 */
	2u,   /* NAV_TO_BUCKET       空表(同点位, 无底盘位移) */
	3u,   /* NAV_TO_TARGET       打靶区 */
	4u,   /* NAV_TO_HOSTAGE      人质区 */
	7u,   /* NAV_REBASE_RETURN   返程前航向重锚 */
	5u,   /* NAV_TO_RETURN       返回区=完赛区 */
};
#define SELFTEST_SEQ_LEN  ((uint8_t)(sizeof(selftest_seq) / sizeof(selftest_seq[0])))

static uint8_t st_idx;
static uint8_t st_finished;   /* 1=序列已跑完(停在返回区) */

/* ---- 接缝 2: nav 跑完后排队下一条 ----
 * 返回 1 = 该起新段(调用方在同上下文调 Nav_DoRequest, 与 Brain 下发同一入口);
 * 返回 0 = 不接管(序列已跑完, 或刚完成的 nav 不是本序列的——手动下发的情况) */
uint8_t Selftest_NextNav(uint8_t done_nav_id, uint8_t *next_id, uint8_t *gear)
{
	if (st_idx >= SELFTEST_SEQ_LEN)            { return 0u; }
	if (done_nav_id != selftest_seq[st_idx])   { return 0u; }
	st_idx++;
	if (st_idx >= SELFTEST_SEQ_LEN)
	{
		st_finished = 1u;   /* 到返回区了, 主循环打一次 RT:FINISH */
		return 0u;
	}
	*next_id = selftest_seq[st_idx];
	*gear    = (uint8_t)SELFTEST_GEAR;
	return 1u;
}

/* ---- 接缝 3: 停等漂移采样(只测不改) ----
 * SEG=本段实际位移, PAUSE=停等期间车体挪动量(期望≈0), TOT=白走里程累计 */
static uint8_t  pd_in_wait;                     /* 1=当前在 WAIT 段内(entry 已抓) */
static uint8_t  pd_valid;                       /* 1=有一组新数据待主循环打印 */
static uint16_t pd_seq;                         /* 停等序号 */
static float    pd_entry_x, pd_entry_y;         /* WAIT 进段位姿 */
static float    pd_seg_x,  pd_seg_y;            /* 本段起点(=上次停等结束位姿) */
static float    pd_seg_dx, pd_seg_dy;           /* 本段实际位移 m */
static float    pd_pau_dx, pd_pau_dy;           /* 停等期间位移 m */
static float    pd_total_x, pd_total_y;         /* 白走里程累计(有符号分量 m):
                                                 * 单向漂移会线性累加, 来回抖动会互相抵消,
                                                 * 据此可区分"系统性溜车"与"随机抖动" */

void Selftest_DriftSample(const mf_ctx_t *m)
{
	const mf_pose_t *p = mf_get_pose(m);

	if (mf_is_waiting(m) != 0u)
	{
		if (pd_in_wait == 0u)               /* WAIT 进沿 */
		{
			pd_in_wait = 1u;
			pd_entry_x = p->x_m;
			pd_entry_y = p->y_m;
			pd_seg_dx  = p->x_m - pd_seg_x; /* 本段实际位移 */
			pd_seg_dy  = p->y_m - pd_seg_y;
		}
	}
	else if (pd_in_wait != 0u)              /* WAIT 出沿 */
	{
		pd_in_wait = 0u;
		pd_pau_dx  = p->x_m - pd_entry_x;   /* 停等白走 */
		pd_pau_dy  = p->y_m - pd_entry_y;
		pd_total_x += pd_pau_dx;            /* 有符号累加: 看漂移方向是否一致 */
		pd_total_y += pd_pau_dy;
		pd_seq++;
		pd_valid   = 1u;
		pd_seg_x   = p->x_m;                /* 下一段起点 */
		pd_seg_y   = p->y_m;
	}
}

/* ---- 接缝 4: 段表拷贝后的停等插桩 ----
 * src=原表, i=当前源下标, buf/o=目标缓冲与写指针(in-out), cap=缓冲段数 */
void Selftest_TablePost(const mf_seg_t *src, uint8_t i,
                        mf_seg_t *buf, uint8_t *o, uint8_t cap)
{
#if (SELFTEST_PAUSE_MS > 0u)
	const mf_seg_t pause_seg = SEG_PAUSE;

  #if SELFTEST_PAUSE_EACH_SEG
	/* 逐段停(旧策略, 仅 A/B 对照用) */
	if ((src[i].type != MF_SEG_WAIT) && (*o < cap))
	{
		buf[(*o)++] = pause_seg;
	}
  #else
	/* 默认: 只在本张大段(一条 nav 表)的最后一个运动段之后插一次(判据: 下一项已是 END) */
	if ((src[i].type != MF_SEG_WAIT)
	    && ((i + 1u) < cap)
	    && (src[i + 1u].type == MF_SEG_END)
	    && (*o < cap))
	{
		buf[(*o)++] = pause_seg;
	}
  #endif
#else
	(void)src; (void)i; (void)buf; (void)o; (void)cap;   /* 停等关闭: 不插桩 */
#endif
}

/* ---- 接缝 5: 主循环打印(唯一 printf 点) ---- */
void Selftest_DebugPoll(void)
{
	static uint8_t st_done_printed = 0u;

	if ((st_finished != 0u) && (st_done_printed == 0u))
	{
		st_done_printed = 1u;
		printf("RT:FINISH %u/%u\r\n", (unsigned)st_idx, (unsigned)SELFTEST_SEQ_LEN);
	}

	/* 停等漂移: 每次停等结束打一行, 单位 mm。
	 * SEG=本段实际位移(对核对表)  PAUSE=停等期间车体自己挪了多少(期望≈0)
	 * TOT=全程白走累计(单向溜车会持续累加, 来回抖动会互相抵消) */
	if (pd_valid != 0u)
	{
		pd_valid = 0u;
		printf("PD:%u SEG[%d,%d] PAUSE[%d,%d] TOT[%d,%d]\r\n",
		       (unsigned)pd_seq,
		       (int)(pd_seg_dx * 1000.0f), (int)(pd_seg_dy * 1000.0f),
		       (int)(pd_pau_dx * 1000.0f), (int)(pd_pau_dy * 1000.0f),
		       (int)(pd_total_x * 1000.0f), (int)(pd_total_y * 1000.0f));
	}
}

/* ---- 接缝 1: 上电起跑 ----
 * 必须投递到命令信箱: 直接调 Nav_DoRequest() 会在 mf_init 之前挂段表,
 * 结果车一动不动、3s 后报 NAV_FAULT(2026-09-20 实测) */
void Selftest_BootStart(void)
{
	st_idx      = 0u;
	st_finished = 0u;
	Car_Nav_MailPush(CAR_SUB_REQ_NAV, selftest_seq[0], (uint8_t)SELFTEST_GEAR);
}

#endif   /* SELFTEST_EN */
