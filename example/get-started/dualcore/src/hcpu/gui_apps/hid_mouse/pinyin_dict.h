/* 拼音字典(音節→常用字,UTF-8)。資料抄自 LVGL lv_ime_pinyin 內建表
   (external 不動、LV_USE_IME_PINYIN 不開,只取資料)。 */
#ifndef PINYIN_DICT_H
#define PINYIN_DICT_H

typedef struct
{
    const char *py;    /* 音節(小寫字母) */
    const char *py_mb; /* 候選字串(UTF-8,頻序) */
} pinyin_dict_entry_t;

extern const pinyin_dict_entry_t g_pinyin_dict[];
extern const int g_pinyin_dict_count;

#endif /* PINYIN_DICT_H */
