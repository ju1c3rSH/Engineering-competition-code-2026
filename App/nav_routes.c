#include "nav_routes.h"
#include "car_units.h"
#include "car_config.h"
#include "selftest.h"

/* 5转/s²: 旧 45转/s²(≈21.2m/s²) 等效阶跃触发电源 BOR(坑-电源与复位.md) */
#define SEG_ACC_MPS2  MF_M_ACC(5.0f)

/* 弯道不能给满: 外轮需求 = v×(1+TRIM·轨距/(2R·k)) = v×1.3455, 给满则 duty 顶格、
 * wz 前馈按指令算 → 转过头(扫描依据 diag_speed_ceiling.py); 直道无差速需求, 可高于弯道 */
#define V_STRAIGHT   MF_M_SPEED(3.10f)        /* 直道1.461: 前段有弧, 出弧车速不足会带偏下一弧 */
#define V_ARC        MF_M_SPEED(2.20f)        /* 弯道1.036: 贴外轮饱和临界(v_free/1.3455)之下 */
/* 越障弧低速: 上障瞬间打滑/俯仰使里程虚高+转角亏, 降速给外轮差速留余量。
 * speed_gear≠0 时被 Nav_GearTable 覆盖, 本值仅 gear=0 生效 */
#define V_ARC_OBST   MF_M_SPEED(Low_speed)    /* 低速0.707: 越障段, 代价约+0.66s */
/* 越障前直道降速: 2026-10-10 只试了 V_STRAIGHT 降到 2.60 一档(=0.334m/s 差), 判据取 EY 且被
 * 弧速变量混淆, 结论不可用。2026-10 复查改用三点剂量-反应(VSRC_MEAS=0, 跨三夜):
 *   直1.461/弧1.036 => arc5 dA +9.3 / delta 21.2mm;  直1.461/弧0.707 => dA +5.0 / 15.9mm;
 *   直0.943/弧0.707 => dA -3.0 / 4.6mm (与干净弧同号 = 治好)。只降弧速仅拿 25%, 直道一起降拿 78%。
 * 机理: 激振幅值∝v²(进坎速度决定) + 衰减时间∝1/v(坎后速度决定), 只有直道速度同时管两头。 */
#define V_KEEP_MID   MF_M_SPEED(Middle_speed) /* 人质末段保持原速 */
/* 减速带区拆段: ZONE 中速压里程虚高 δ 的方差, POST 是干净里程; 三段和恒 45, 未标定前与不拆等价。
 * 标定: PRE=D1/32.74, POST=D2/32.74(1单位地面 = π×156.3/15 = 32.74mm), 详见 坑-运动与导航.md §7
 * ★2026-10-25 有意破掉"三段同档": ZONE 单独降到 0.50(POST/弧仍 0.707)。实测 0.707 过坎横漂
 * 趟间波动 ±6mm, 进越障弧被 R·Δθ 放大成 61mm 散布; 同车低速段(0.72)只漂 1~4mm。减速距离
 * (1.461²-0.50²)/(2·2.356·1.414)=283mm 仍在 PRE(628)内做完, 且不改变弧入口速度。代价 +0.13s */
#define V_OBST_ZONE    MF_M_SPEED(1.06f)      /* 坎带 0.50 m/s: 降速压坎上侧滑, 见上 */
#define V_OBST_SETTLE  MF_M_SPEED(Low_speed)  /* 出坎段: 保持与 V_ARC_OBST 同档, 见上 */
#define SEG4_TOTAL_U 45.0f
/* 2026-10-25 18:14 趟: 坎沿原本落在 PRE 末端(车带速撞坎), 已由**上文段2 缩短**解决,
 * 实测 PRE 出口航向从 -1.2° 变 0.0°, 坎区三段链 |YE| ≤0.2°(15 趟里最干净), 故此处保持 20.0。
 * ★不要再把这里也改小: 段2 与 PRE 是**同一件事的两个旋钮**, 一起改 = 边界推早 159mm, 白花 124ms。
 * 定位依据(供以后重标): PRE 出口航向 15 趟散布 4.2°, 而 ZONE(本该在坎上)只有 0.6°; 且 PRE 有
 * 7 个航向环时间常数, 能在出口留下 ±2° 的扰动只可能来自最后 1~2τ(90~180mm) => 坎沿在 PRE 末段。
 * 撞击速度 = 0.50(ZONE 档) + sqrt(2·acc·x), x = 坎沿到边界距离; x=150mm 时已是 0.98m/s,
 * 能量是最低档的 4 倍。规范做法: PRE = (弧出口到坎沿的真实距离 − 70) / 32.74, 70mm 留给减速
 * 在坎外做完 —— 若按此式算出来与 20.0 不同, 应同时把段2 改回几何真值, 不要只动一个。 */
#define SEG4_PRE_U   20.0f
#define SEG4_POST_U  18.0f                    /* TODO(卷尺标定): D2/32.74; 出弧仍偏外则修此数 */
#define SEG4_ZONE_U  (SEG4_TOTAL_U - SEG4_PRE_U - SEG4_POST_U)
#define V_SLOW_ENTRY MF_M_SPEED(Middle_speed) /* 首段过减速带 */
#define V_SLOW_DOCK  MF_M_SPEED(Low_speed)    /* 末段低速进作业点, 横向精度靠它 */
/* 咬靶速度与段表解耦: 末 2.5m 由 TRACKING_SLOW_MPS=0.45 恒压, 提速只省前段(净省 ~0.2s)。
 * 验收 = 咬靶停点极差不变 + 进咬靶区实刹无冲过靶 */
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
/* MOVE 续段: 只切里程清零点, 不复位 PID/不清 IE; 同速时与不拆等价 */
#define SEG_MOVE_CONT(v, d) { .type = MF_SEG_MOVE_DIST, .reset_pid = 0, \
                          .p.move_dist = { (v), (d), MF_KEEP, SEG_ACC_MPS2 } }
#define SEG_END         { .type = MF_SEG_END }

static const mf_seg_t tbl_qr[] =          /* NAV_TO_QR: flag0 出发首段(台架单段用, 长度随意改) */
{
	SEG_MOVE(MF_M_SPEED(Middle_speed), MF_M_DIST(47.0f)),
	SEG_END
};
static const mf_seg_t tbl_bomb[] =        /* NAV_TO_BOMB: 排爆区去程(末段转弯+末段直线已移入重锚表) */
/* MOVE 长度按真实轮径 156.3mm(固件假设 150)折算: 物理长度 = (单位/15)×π×真实轮径;
 * 与 MF_ARC_WZ_TRIM 同源, 要改一起改 */
{
	SEG_MOVE(V_SLOW_ENTRY, MF_M_DIST(41.6f)),
	SEG_ARC(V_ARC,  TURN_RADIUS_M),   /* 左 90 (flag1) */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(15.2f)), //这个是过坎转弯前的直线
	SEG_ARC(V_ARC, -TURN_RADIUS_M),   /* 右 90 (flag2) */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(SEG4_PRE_U)),
	SEG_MOVE_CONT(V_OBST_ZONE, MF_M_DIST(SEG4_ZONE_U)), /* 减速带区: 里程虚高 δ 死在本段, 不外传 */
	SEG_MOVE_CONT(V_OBST_SETTLE, MF_M_DIST(SEG4_POST_U)), /* 出坎段: 与越障弧同档, 见 V_OBST_SETTLE */
	/* 越障弧入口重锚(2026-10-25): 直线段的航向残差经 90° 弧按 R·Δθ 放大成**位置**误差
	 * (A 图实测 1.4° → 23mm, 且 TURN_TO 只纠角不纠位 ⇒ 一路带到末段), 必须在放大前清掉。
	 * 放 POST 之后而非 PRE 之后: 坎区三段(PRE/ZONE/POST)保持连续不被停车打断, 且此处残差
	 * 已被 ZONE/POST 吸收到 ±0.5° 量级, 重锚只需转半度、代价最小。 */
	
	SEG_ARC(V_ARC_OBST, TURN_RADIUS_M),/* 左 90 (flag4): 越障弧, 见 V_ARC_OBST 注 */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(32.0f)),
	SEG_ARC(V_ARC, -TURN_RADIUS_M),   /* 右 90 (flag6) */
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(47.0f)),
	SEG_END
};
static const mf_seg_t tbl_bucket[] =      /* NAV_TO_BUCKET: 桶与作业点同点位, 到位后仅转台转向, 无底盘位移 */
{
	SEG_END
};
/* 停点 213 单位(让车走到靶心): 与 tbl_hostage 首段成对改, 净和 223.5 不变; 推导见 坑-运动与导航.md §6 */
/* 重锚角 = 几何真值; 补偿统一走 MF_HEADING_BIAS_DEG, 勿在此塞常数 */
#define NAV_REBASE_BOMB_YAW_DEG     (-90.0f)
#define NAV_REBASE_TARGET_YAW_DEG   (-90.2f)
static const mf_seg_t tbl_target[] =      /* NAV_TO_TARGET: flag11~12 打靶区 */
{
	/* 入口重锚: 原地纠回作业期静止漂移, 只清 IE; 系统性横偏靠 MF_HEADING_BIAS_DEG */
	SEG_TURN_TO(NAV_REBASE_TARGET_YAW_DEG),
	SEG_MOVE(V_TARGET, MF_M_DIST(216.0f)),
	SEG_END
};
static const mf_seg_t tbl_hostage[] =     /* NAV_TO_HOSTAGE: flag13~14 人质区(末段直线+重锚移入 rebase 表) */
{
	SEG_MOVE(V_STRAIGHT, MF_M_DIST(7.0f)),   /* 48.5 − 38: 与 tbl_target 成对, 见上 */
	SEG_ARC(V_ARC, -TURN_RADIUS_M),   /* 右 90 (flag14) */
	SEG_END
};
/* 弧线出口先转绝对角再走末段直线; 末段不提速(与 tbl_target 成对, 提速破坏对坑落点) */
#define NAV_REBASE_HOSTAGE_YAW_DEG  (-180.0f)
static const mf_seg_t tbl_rebase_hostage[] = /* NAV_REBASE_HOSTAGE: 弧线出口重锚+末段直线进停点 */
{
	SEG_TURN_TO(NAV_REBASE_HOSTAGE_YAW_DEG),
	SEG_MOVE_HD(V_KEEP_MID, MF_M_DIST(77.0f), NAV_REBASE_HOSTAGE_YAW_DEG),
	SEG_END
};
/* 返程/终点航向 = 几何真值 -180°; 段表只允许几何量, 基准偏差统一走 MF_HEADING_BIAS_DEG
 * (坑-运动与导航.md §6 + Motion/README.md「返程与终点航向」) */
#define NAV_RETURN_LEG_YAW_DEG     (-180.0f)
#define NAV_RETURN_FINAL_REBASE_EN 1u
/* 返程腿纯直线无弧; 末端接刹停(v_stop=sqrt(2·acc·MF_STOP_DECEL_SCALE·remain), 巡航 v 只在
 * remain > v²/(2·1.414) 时有效) ⇒ 提速收益随 v 增大饱和 —— 3.50 已近饱和点 */
#define V_RETURN     MF_M_SPEED(3.50f)        /* 返程1.649 */

static const mf_seg_t tbl_return[] =      /* NAV_TO_RETURN: flag17~18 返回区 */
{
	SEG_MOVE_HD(V_RETURN, MF_M_DIST(177.0f), NAV_RETURN_LEG_YAW_DEG),
#if NAV_RETURN_FINAL_REBASE_EN
	SEG_TURN_TO(NAV_RETURN_LEG_YAW_DEG),   /* 终点姿态收尾: 注入 -180° 绝对真值 */
#endif
	SEG_END
};
/* 返程重锚角 = 几何真值 -180.0 */
#define NAV_REBASE_RETURN_YAW_DEG   (-180.0f)

static const mf_seg_t tbl_rebase_bomb[] =   /* NAV_REBASE_BOMB: 排爆区末段+绝对航向重锚; 入口摆码 7 与
                                             * 末段转弯并行(碍事则 nav_entry_turn[6]=0 改串行) */
{
	SEG_ARC(V_ARC, -TURN_RADIUS_M),      /* 右 90: 原 tbl_bomb 表尾转弯 */
	SEG_MOVE(V_SLOW_DOCK, MF_M_DIST(47.0f)),/* 末段低速进作业点 —— 不许提速 */
	SEG_TURN_TO(NAV_REBASE_BOMB_YAW_DEG),                /* 对坑前重锚: 几何真值, 补偿走 MF_HEADING_BIAS_DEG */
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

/* gear≠0: 段速改写进 RAM 缓冲; gear=0 用 flash 原表(自测/比赛同路径, 结论才可外推)。
 * 自测 gear 不能为 0(SEG_PAUSE 需拷贝路径, 配错会 #error, 见 selftest.c) */
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

/* 段表自动链续(0=不续, 等主控); 自测必须置 0(与 SELFTEST_NEXT_NAV 双触发)。
 * 续表走 flash 原表(RAM 缓冲会弄悬空 path 指针); 拆表代价见 坑-运动与导航.md §5 */
const uint8_t nav_chain_next[CAR_NAV_ID_COUNT] =
{
	0u, NAV_CHAIN_1_TO, 0u, 0u, NAV_CHAIN_4_TO, 0u, 0u, 0u, 0u,
};
