/**
 * @file    car_protocol.h
 * @brief   底盘 UART4 链路协议码表(纯契约, 零依赖)
 *
 * 唯一依据: serial_protocol/panzer_links_protocol.md。改帧格式先改文档再动三端代码。
 * 本头**不 include 任何模块头**, 故可被任一端(含被 Control_Car.h 聚合的子模块)直接引用,
 * 这是把协议码从 Control_Car.h 抽出来的唯一理由: 否则子模块要用码表就得反向
 * include Control_Car.h, 立即成环。
 */
#ifndef _CAR_PROTOCOL_H_
#define _CAR_PROTOCOL_H_

/* ================== 帧类型 (panzer_links_protocol.md 4.x) ================== */
#define FRAME_TYPE_REQUEST        0x20u   /* CMD_REQUEST */
#define FRAME_TYPE_ACK            0x21u   /* CMD_ACK */
#define FRAME_TYPE_NACK           0x22u   /* CMD_NACK */
#define FRAME_TYPE_STOP           0x23u   /* CMD_STOP(通用) */
#define FRAME_TYPE_DATA           0x24u   /* DATA_STREAM */

/* NACK 原因码枚举 (panzer_links_protocol.md v2.2 §1.4, 三链路统一) */
#define CAR_NACK_FORMAT       0x01u   /* 长度不足/非REQUEST/data_id 不支持 */
#define CAR_NACK_BUSY         0x02u   /* 忙/未就绪(预留) */
#define CAR_NACK_PARAM        0x03u   /* 子命令存在但参数越界 */
#define CAR_NACK_UNKNOWNSUB   0x04u   /* data_id 合法但 sub_cmd 不在定义表 */

#define CAR_DATA_ID_CTRL        0x21u   /* CMD_CAR_CTRL: 运动请求(下行 sub_cmd 式) */
#define CAR_DATA_ID_EVT         0x22u   /* EVT_CAR_EVENT: 底盘事件上报(上行) */

/* ================== 下行 sub_cmd (CMD_CAR_CTRL) ================== */
#define CAR_SUB_REQ_NAVIGATION  0x01u   /* arg0=navigation_target_id 0~8(6/7/8=重锚伪路线), arg1=speed_gear 0~3 */
#define CAR_SUB_REQ_STOP          0x02u   /* 急停安全停车, 清执行状态 */
#define CAR_SUB_REQ_TURNTABLE     0x03u   /* arg0=转台码 0~7: 1~3对坑(123/90/57°) 4~6对桶(270/270/90°CW反向) 7回正90°, 0=不动作; arg1=保留恒0 */
#define CAR_SUB_GYRO_ZERO         0x04u   /* 陀螺仪置零(须车静止) */
/* 打靶段内跟瞄 v3.0(刹停/续跑): 位移唯一来源是段表, 视觉只回答"是否已到靶位"。
 * 不变量: 跟瞄态一个字节的 vx/wz 都不写(零标定); 恢复后终点仍是"进段位置 + 段表里程" */
#define CAR_SUB_TRACKING_START    0x05u   /* arg0=保留(旧 speed_level, v3.0 不再生效),
                                             arg1=aim_axis(仅范围校验, v3.0 不再分轴) */
#define CAR_SUB_TRACKING_ERROR    0x06u   /* [error_low_byte,error_high_byte,target_found_flag], 定点=误差归一化*10000 小端 */
#define CAR_SUB_TRACKING_STOP     0x07u   /* arg0=tracking_stop_mode(0=就地刹停/1=放行段表走完当前段), arg1=0保留 */

/* 旧短名兼容别名: interurp.c 收发沿用, 值与全称一致, 新代码用全称 */
#define CAR_SUB_REQ_NAV         CAR_SUB_REQ_NAVIGATION

/* ================== 上行事件 (EVT_CAR_EVENT) ================== */
#define CAR_EVT_NAV_DONE          0x02u   /* 段序列全部完成 */
#define CAR_EVT_NAV_FAULT         0x03u   /* 段执行异常 */
#define CAR_EVT_TURN_DONE         0x04u   /* 转台到位 */
#define CAR_EVT_TRACKING_DONE     0x05u   /* 已刹停(暂停本段), 可发射; 等 0x07 STOP(1) 放行续跑 */
#define CAR_EVT_TRACKING_FAULT    0x06u   /* 本地判定瞄不上(reason 见 TRK_FAULT_*), Brain 容错定夺 */

/* NAV_FAULT 的 reason 码 */
#define CAR_FAULT_WATCHDOG      0x04u   /* 进度看门狗: 里程 3s 无变化 */

/* ================== 跟瞄入参 / reason ================== */
/* 收敛轴码表: v3.0 不再分轴(一律不写 vx/wz), 两个宏仅用于入参范围校验 */
#define TRK_AXIS_ROTATE           0x00u
#define TRK_AXIS_TRANSLATE        0x01u

/* TRACKING_FAULT(事件 0x06)的 reason 码: 上报给主控做决策(重试/容错发射) */
#define TRK_FAULT_NOT_CONVERGED         0x01u   /* 到位后长时间未收敛 */
#define TRK_FAULT_NO_TARGET             0x02u   /* 到位后丢失目标(遮挡/阈值/相机) */
#define TRK_FAULT_NO_PATH               0x03u   /* 收到 START 时无活动段表(未发 REQ_NAV/段启动失败) */

#endif /* _CAR_PROTOCOL_H_ */
