#include "nav_routes.h"
#include "car_units.h"
#include "car_config.h"
#include "selftest.h"

/* 运动调优 (test/motion-tuning): 旧 TURN_ACC(45转/s²≈21.2m/s²) 等效阶跃,
 * 满目标速度 0.94m/s 仅 45ms —— 起步电流浪涌触发电源 BOR 复位(实测 RST=BOR)。
 * 直行/弧线统一降到 5转/s²(≈2.36m/s², 0.4s 拉满 0.94m/s), 削起步浪涌 */
#define SEG_ACC_MPS2  MF_M_ACC(5.0f)

/* 档位(2026-10-07 实车复测后)。mf_config.h 的 MF_WZ_RATE_OUT_HEADROOM 已从 1.8 抬到 3.0(=450dps),
 * 弯道角速度指令发得出去, 但**弯道速度不能给满**: 外侧轮需求 = v×(1+TRIM·轨距/(2R·k))
 * = v×1.3455, 给满 High(1.414) ⇒ 外轮要1.90m/s 远超v_free⇒ duty 顶格、实际车速被钳在
 * ~1.05, 而 wz 前馈仍按指令1.414 算 ⇒ 车速没上去角速度给多 ⇒ 半径偏小、转过头超调。
 * 实测两趟印证: 六个弧 R=937~959 均值943(名义942.5), 提速后无超调。
 * 直道无差速需求、外轮需求=v 本身, 可高于弯道。
 * 扫描依据(两种 v_free 口径都 PASS, 判据终点<=120mm): diag_speed_ceiling.py */
#define V_STRAIGHT   MF_M_SPEED(3.10f)        /* 直道1.461: 前段有弧, 出弧车速不足会带偏下一弧 */
#define V_ARC        MF_M_SPEED(2.20f)        /* 弯道1.036: 贴外轮饱和临界(v_free/1.3455)之下 */
/* 越障弧专用低速(seg5, 左90 flag4): 上障碍瞬间打滑/俯仰使里程虚高+转角亏,
 * 2026-10-10 两趟复现 R=984/EY=44(其余弧940±3, 出口判据±10)。降速给外轮差速留余量;
 * 物理落点需卷尺核对后再调档。注意: speed_gear≠0 时 Nav_GearTable 覆盖弧速, 本值仅 gear=0 生效 */
#define V_ARC_OBST   MF_M_SPEED(Low_speed)    /* 低速0.707: 越障段, 代价约+0.66s */
/* 越障前直道降速试验已否证(2026-10-10): V_STRAIGHT_OBST=2.60 一趟 ARC:5 EY=19/R=961,
 * 不优于只降弧速的 EY=12/R=957 —— 瓶颈不在入障初速, 残余 ~15mm 弧外是越障扰动的
 * 当前地板, 靠 LN:6/LN:8 直线吸收, 判据以物理落点为准。保留此注防再试 */
#define V_KEEP_MID   MF_M_SPEED(Middle_speed) /* 保持原速: 人质末段(见该表注) */
#define V_SLOW_ENTRY MF_M_SPEED(Middle_speed) /* 首段过减速带 */
#define V_SLOW_DOCK  MF_M_SPEED(Low_speed)    /* 末段低速进作业点, 横向精度靠它 */
/* 打靶直线(2026-10-07): 咬靶速度与段表**解耦** —— Tracking_LimitApproachSpeed() 每拍
 * 把 cur_speed 硬压到 TRACKING_SLOW_MPS=0.45, 咬靶区=本段最后 TRACKING_SLOW_ZONE_M=2.5m。
 * 那 2.5m 的 5.556s 是固定开销(任何档位都省不掉); 可省的只有前 4.286m。
 * 阶跃风险已核: 现 0.943->0.45 的轮速环duty 本就饱和在 -1.00 且实测稳定,提到 1.461 只是
 * err 从 -0.49 变 -1.01, duty 同样饱和 => 环路行为不变。 */
/* 2026-10-10 (方案A): 3.10->3.50(1.649m/s), 与 V_RETURN 同档 —— 该车速含刹停已在返程腿实车背书。
 * 可提速窗口只有咬靶区前 4.29m, 净省约 0.2~0.3s; 验收 = 咬靶停点极差不变 + 进咬靶区实刹无冲过靶 */
#define V_TARGET     MF_M_SPEED(3.50f)        /* 打靶直线大段: 与返程同档, 咬靶区自动压速 */

/* nav_id 段表: 原 flags 路线逐段复刻(基线 git 4f3ceb2 true_BOOM_Finish) */
#define SEG_MOVE(v, d)  { .type = MF_SEG_MOVE_DIST, .reset_pid = 1, \
                          .p.move_dist = { (v), (d), MF_KEEP, SEG_ACC_MPS2 } }
/* MOVE 段、但航向目标写死成**绝对角**(不用 MF_KEEP 的推算值) */
#define SEG_MOVE_HD(v, d, hdg) { .type = MF_SEG_MOVE_DIST, .reset_pid = 1, \
                          .p.move_dist = { (v), (d), (hdg), SEG_ACC_MPS2 } }
#define SEG_ARC(v, r)   { .type = MF_SEG_ARC, .reset_pid = 1, \
                          .p.arc = { (v), (r), ((r) > 0.0f ? 90.0f : -90.0f), SEG_ACC_MPS2 } }
/* 重锚转向的到位容差: 0.5°(段默认 1.0° 太松 —— 重锚角是后续各段的航向基准) */
#define SEG_TURN_TO(y)  { .type = MF_SEG_TURN_TO, .reset_pid = 1, \
                          .tol_yaw_deg = 0.5f, \
                          .p.turn = { (y), 0.0f } }
#define SEG_END         { .type = MF_SEG_END }

static const mf_seg_t tbl_qr[] =          /* NAV_TO_QR: flag0 出发首段(台架单段用, 长度随意改) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(47.0f)),
	SEG_END
};
static const mf_seg_t tbl_bomb[] =        /* NAV_TO_BOMB: 排爆区去程(末段转弯+末段直线已移入重锚表) */
/* 2026-09-30: 五条 MOVE 已按 0.96 缩(47/19/49/37/50 → 45.1/18.2/47.0/35.5/48.0)。
 * 直线物理长度 = (单位/15)×π×**真实**轮径, 而真实有效轮径 ≈156.3mm(固件假设 150)——
 * 同一比例也让弧半径变成 982(卷尺), 见 mf_config.h: MF_ARC_WZ_TRIM。两处同源, 要改一起改。 */
{
	SEG_MOVE(V_SLOW_ENTRY, MF_M_DIST(41.6f)),
	SEG_ARC(V_ARC,  TURN_RADIUS_M),   /* 左 90 (flag1) */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(18.2f)),
	SEG_ARC(V_ARC, -TURN_RADIUS_M),   /* 右 90 (flag2) */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(45.0f)),
	SEG_ARC(V_ARC_OBST, TURN_RADIUS_M),/* 左 90 (flag4): 越障弧, 见 V_ARC_OBST 注 */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(32.0f)),
	SEG_ARC(V_ARC, -TURN_RADIUS_M),   /* 右 90 (flag6) */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(47.0f)),
	SEG_END
};
static const mf_seg_t tbl_bucket[] =      /* NAV_TO_BUCKET: 桶与排爆作业点在车道两侧同一点位,
                                           * 到位后仅转台转向桶侧, 无底盘位移 */
{
	SEG_END
};
/* 打靶段停点标定(2026-09-27): 175 → 213 单位, 多出 38 单位(1.19m)让车能走到靶心(不延长则
 * 刹停判据永不成立)。与 tbl_hostage 首段**成对改**: 加多少减多少, 净和 223.5 不变, 否则
 * 90° 弧起点整体前移、落点横向偏同样距离。
 * 推导(E≈2300 反解/窗口公式)与验证法: docs/坑-运动与导航.md §6
 * 2026-10-07: 给 V_TARGET(3.10转/s=1.461)。此前按"要给跟瞄留收敛窗口"保持 Middle 是
 * 过度保守 —— 咬靶区那2.5m 由 TRACKING_SLOW_MPS=0.45 恒定压住, 与段表档位无关 */
/* 重锚角 = 几何真值(-90.0)。**不要再往这里塞补偿** —— 补偿统一走 MF_HEADING_BIAS_DEG。
 * 旧值 -90.3 / -89.5 是台架标定残差: 它随"跑了多久、改了什么行为"变, 且与 -180 那处
 * 符号相反、不能互推(Motion/README.md §11.5 的 K5)。单基准航向下整条路线是刚性旋转,
 * 一个 bias 就够; 段表里保持纯几何才能把误差归因到唯一一个数上 */
#define NAV_REBASE_BOMB_YAW_DEG     (-90.0f)
#define NAV_REBASE_TARGET_YAW_DEG   (-90.2f)
static const mf_seg_t tbl_target[] =      /* NAV_TO_TARGET: flag11~12 打靶区 */
{
	/* 入口重锚: 排爆作业期静止漂移(实测 -901→-905)在进长直前原地纠回。
	 * 注意: 上一表 TURN_TO 已把标称航向吸附到 -90(MF_HEADING_FRAME_EN 跨表不重定锚),
	 * 本段不改变长腿目标航向, 只是把 IE 提前清零 —— 对赛道系统性横偏要靠
	 * MF_HEADING_BIAS_DEG, 别指望这里 */
	SEG_TURN_TO(NAV_REBASE_TARGET_YAW_DEG),
	SEG_MOVE(V_TARGET, MF_M_DIST(216.0f)),
	SEG_END
};
static const mf_seg_t tbl_hostage[] =     /* NAV_TO_HOSTAGE: flag13~14 人质区(末段直线+重锚移入 rebase 表) */
{
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(6.0f)),   /* 48.5 − 38: 与 tbl_target 成对, 见上 */
	SEG_ARC(V_ARC, -TURN_RADIUS_M),   /* 右 90 (flag14) */
	SEG_END
};
/* 人质对坑前重锚 = 弧线出口先转绝对角、再按绝对航向走末段直线(标定项, 同 NAV_REBASE_BOMB_YAW_DEG)
 * 2026-10-07: 末段直线保持 Middle 不提速 —— 与 tbl_target 成对提速会一起破坏对坑落点 */
#define NAV_REBASE_HOSTAGE_YAW_DEG  (-180.0f)
static const mf_seg_t tbl_rebase_hostage[] = /* NAV_REBASE_HOSTAGE: 弧线出口重锚+末段直线进停点 */
{
	SEG_TURN_TO(NAV_REBASE_HOSTAGE_YAW_DEG),
	SEG_MOVE_HD(V_KEEP_MID, MF_M_DIST(77.0f), NAV_REBASE_HOSTAGE_YAW_DEG),
	SEG_END
};
/* 返程段/终点的绝对航向: 任务终点 YAW 严格 -180° —— 几何真值(GYRO_ZERO 定出发 0°, 反向即 -180°)。
 * 约束: 段表里**只允许出现几何量**; 基准系与真实几何的固定偏差已收口到 MF_HEADING_BIAS_DEG 一个旋钮
 * (旧 NAV_RETURN_TRIM_DEG=1.8 等散落补偿混在几何常量里分不出几何错还是补偿, 且不能互推)。详见
 * docs/坑-运动与导航.md §6 + Motion/README.md「返程与终点航向」。 */
#define NAV_RETURN_LEG_YAW_DEG     (-180.0f)
#define NAV_RETURN_FINAL_REBASE_EN 1u
/* 返程腿(2026-10-07): 纯直线无弧, 外轮需求=v 本身, 无饱和顾虑。
 * 必须**跨过饱和点**: 该腿末端接 SEG_TURN_TO(v_next=0), 全程受 v_stop 抛物线钳制,
 * v_stop=sqrt(2*acc*MF_STOP_DECEL_SCALE*remain)。巡航 v 只在 remain > v²/(2*1.414) 时有效,
 * 故提速收益随 v 增大而衰减(实测: 2.00->3.10 省1.82s, 3.10->3.90 只再省 0.09s 后饱和)。
 * 3.50 制动距离 0.962m, 占该腿 5.498m 的 18%, 余量充足。 */
#define V_RETURN     MF_M_SPEED(3.50f)        /* 返程1.649 */

static const mf_seg_t tbl_return[] =      /* NAV_TO_RETURN: flag17~18 返回区 */
{
	SEG_MOVE_HD(V_RETURN, MF_M_DIST(177.0f), NAV_RETURN_LEG_YAW_DEG),
#if NAV_RETURN_FINAL_REBASE_EN
	SEG_TURN_TO(NAV_RETURN_LEG_YAW_DEG),   /* 终点姿态收尾: 注入 -180° 绝对真值 */
#endif
	SEG_END
};
/* 返程重锚角 = 几何真值 -180.0(旧标定值 -179.8 / -179.5 已作废) */
#define NAV_REBASE_RETURN_YAW_DEG   (-180.0f)

static const mf_seg_t tbl_rebase_bomb[] =   /* NAV_REBASE_BOMB: 排爆区末段(原 tbl_bomb 表尾)+绝对航向重锚;
                                             * 末段转弯与低速直线挪进本表, 与转盘 90° 扫描位并行 —— 入口
                                             * 由底盘自行摆码 7(见 nav_entry_turn), 转台在末段转弯期间就转
                                             * (原工程 flag9 时点); 臂为收起位无干涉。若碍事把
                                             * nav_entry_turn[6] 置 0 = 主控 NAV_DONE 后再发(=串行) */
{
	SEG_ARC(V_ARC, -TURN_RADIUS_M),      /* 右 90: 原 tbl_bomb 表尾转弯 */
	SEG_MOVE(V_SLOW_DOCK, MF_M_DIST(47.0f)),/* 末段低速进作业点 —— 不许提速 */
	SEG_TURN_TO(NAV_REBASE_BOMB_YAW_DEG),                /* 对坑前重锚: 五个闭环 90° 弧的终点。
	                                                      * 几何真值, 不掺补偿 —— 基准偏差统一走
	                                                      * MF_HEADING_BIAS_DEG(见其定义处注释) */
	SEG_END
};
static const mf_seg_t tbl_rebase_return[] = /* NAV_REBASE_RETURN: flag17 携人质返程前重锚 */
{
	SEG_TURN_TO(NAV_REBASE_RETURN_YAW_DEG),
	SEG_END
};

const mf_seg_t *const nav_routes[CAR_NAV_ID_COUNT] =
{
	tbl_qr,             /* 0 NAV_TO_QR         */
	tbl_bomb,           /* 1 NAV_TO_BOMB       */
	tbl_bucket,         /* 2 NAV_TO_BUCKET     */
	tbl_target,         /* 3 NAV_TO_TARGET     */
	tbl_hostage,        /* 4 NAV_TO_HOSTAGE    */
	tbl_return,         /* 5 NAV_TO_RETURN     */
	tbl_rebase_bomb,    /* 6 NAV_REBASE_BOMB   */
	tbl_rebase_return,  /* 7 NAV_REBASE_RETURN */
	tbl_rebase_hostage, /* 8 NAV_REBASE_HOSTAGE */
};
typedef char nav_route_count_check[(sizeof(nav_routes) / sizeof(nav_routes[0])) == CAR_NAV_ID_COUNT ? 1 : -1];

/* NAV_GEAR_BUF_LEN: car_config.h 按 SELFTEST_EN 切换(比赛 12 / 自测 24) */

static float Nav_GearSpeed(uint8_t speed_gear)
{
	switch (speed_gear)
	{
	case 1u:  return MF_M_SPEED(Low_speed);
	case 2u:  return MF_M_SPEED(Middle_speed);
	case 3u:  return MF_M_SPEED(High_speed);
	default:  return 0.0f;
	}
}

/* speed_gear 覆盖需改写段速度, 拷入 RAM 缓冲后启动; gear=0 直接用 flash 原表。
 * gear=0 一律不拷贝, 让自测与比赛走同一条路径 —— 否则自测结论不可外推到比赛。
 * 代价: 自测插停等段(SEG_PAUSE)必须走拷贝路径, 故自测 gear 不能为 0
 * (见 selftest.c 的 SELFTEST_GEAR, 配错会 #error) */
const mf_seg_t *Nav_GearTable(uint8_t navigation_target_id, uint8_t speed_gear)
{
	static mf_seg_t buf[NAV_GEAR_BUF_LEN];
	const mf_seg_t *src = nav_routes[navigation_target_id];
	uint8_t i, o;
	float v;

	if (speed_gear == 0u)
	{
		return src;
	}

	v = Nav_GearSpeed(speed_gear);
	o = 0u;
	for (i = 0u; (i < NAV_GEAR_BUF_LEN) && (o < NAV_GEAR_BUF_LEN); i++)
	{
		buf[o] = src[i];
		if (buf[o].type == MF_SEG_END)
		{
			break;
		}
		if (buf[o].type == MF_SEG_MOVE_DIST)
		{
			buf[o].p.move_dist.speed_mps = v;
		}
		else if (buf[o].type == MF_SEG_ARC)
		{
			buf[o].p.arc.speed_mps = v;
		}
		o++;
		SELFTEST_TABLE_POST(src, i, buf, &o, NAV_GEAR_BUF_LEN);
	}
	/* 兜底: 缓冲耗尽时截断, 必须以 END 结束(否则段表会越过缓冲读飞) */
	if (o >= NAV_GEAR_BUF_LEN)
	{
		o = (uint8_t)(NAV_GEAR_BUF_LEN - 1u);
	}
	buf[o].type = MF_SEG_END;
	return buf;
}

/* 段入口自动转台: 启动该 nav_id 段时底盘自行摆到指定码位(0=不动); 现仅 NAV_REBASE_BOMB(6) -> 码 7 */
const uint8_t nav_entry_turn[CAR_NAV_ID_COUNT] =
{
	0u, 0u, 0u, 0u, 0u, 0u, 7u, 0u, 0u,   /* 仅 NAV_REBASE_BOMB(6) -> 回正 90° */
};

/* 段表自动链续: 该 nav 跑完由底盘自行续跑下一条(0=不续, 等主控下发)。
 * 自测形态必须置 0: SELFTEST_NEXT_NAV 与它同在"跑完一条"处接管, 同开会双触发。
 * 续表一律用 flash 原表 nav_routes[], 不走 RAM 缓冲(会把在跑 path 指针弄悬空)。
 * 拆表代价(表尾硬减速/跨表空窗)与链续收益: 见 docs/坑-运动与导航.md §5 */
const uint8_t nav_chain_next[CAR_NAV_ID_COUNT] =
{
	0u, NAV_CHAIN_1_TO, 0u, 0u, NAV_CHAIN_4_TO, 0u, 0u, 0u, 0u,
};
