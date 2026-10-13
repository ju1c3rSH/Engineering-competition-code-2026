/**
 * @file    mf_singles.h
 * @brief   单步便捷指令段模板: 单段数组自带 END 哨兵 —— 缺哨兵 mf_path_tick 会读出段外内存
 */
#ifndef MF_SINGLES_H
#define MF_SINGLES_H

#include "mf_types.h"

/* 单步段模板宏：每个“便捷指令”内部展开为“1 段 + 哨兵” */
#define MF_SINGLE_BEGIN    static mf_seg_t seg[2]
#define MF_SINGLE_END      do { seg[1].type = MF_SEG_END; } while (0)

#endif /* MF_SINGLES_H */
