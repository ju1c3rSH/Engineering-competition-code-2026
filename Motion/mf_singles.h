/**
 * @file    mf_singles.h
 * @brief   单步便捷指令的段定义：保证单段路径自带 MF_SEG_END 哨兵
 *
 * 直接把 const 定义写在调用处会出现一个经典 bug：单段数组后面不跟哨兵，
 * mf_path_tick 读到段外内存。mf_motion.c 里的 mf_move_rel 系列用这个模板。
 */
#ifndef MF_SINGLES_H
#define MF_SINGLES_H

#include "mf_types.h"

/* 单步段模板宏：每个“便捷指令”内部展开为“1 段 + 哨兵” */
#define MF_SINGLE_BEGIN    static mf_seg_t seg[2]
#define MF_SINGLE_END      do { seg[1].type = MF_SEG_END; } while (0)

#endif /* MF_SINGLES_H */
