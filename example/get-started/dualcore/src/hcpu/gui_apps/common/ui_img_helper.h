/**
 ******************************************************************************
 * @file   ui_img_helper.h
 * @author Skaiwalk software development team
 ******************************************************************************
 */
// #include "ui_img_helper.h"
#ifndef UI_IMG_HELPER_H
#define UI_IMG_HELPER_H

#ifdef FLASH_IMG
#define BTN_FLASHLIGHT "/assets/icons/btn_flashlight.bin"
#define BG_LODING "/assets/icons/bg_loading.bin"
#define IMG_FLASHLIGHT "/assets/icons/img_flashlight.bin"
#define IMG_ACTIVITY "/assets/icons/img_activity.bin"
#define IMG_ALARM_2 "/assets/icons/img_alarm_2.bin"
#define IMG_CALENDAR "/assets/icons/img_calendar.bin"
#define IMG_GAME "/assets/icons/img_game.bin"
#define IMG_GROUP "/assets/icons/img_group.bin"
#define IMG_PHOTO "/assets/icons/img_photo.bin"
#define IMG_RECORDER "/assets/icons/img_recorder.bin"
#define IMG_LOGO "/assets/icons/img_logo.bin"
#define IMG_NOTE "/assets/icons/img_note.bin"
#define IMG_WORKOUT "/assets/icons/img_workout.bin"
#define IMG_ITUNES "/assets/icons/img_itunes.bin"
#define IMG_HEART_RATE "/assets/icons/img_heart_rate.bin"
#define IMG_SLEEP "/assets/icons/icon_sleep_mode.bin"
#define IMG_CALCULATOR "/assets/icons/img_calculator.bin"
#define IMG_ALARM "/assets/icons/img_alarm.bin"
#define IMG_SETTINGS "/assets/icons/img_settings.bin"
#define IMG_MESSAGES "/assets/icons/img_messages.bin"
#define IMG_MOUSE "/assets/icons/img_mouse.bin"
#define SMALL_IMG_LOGO_MATTING "/assets/icons/small_img_logo_matting.bin"
#define UP_ARROW "/assets/icons/up_arrow.bin"
#define DOWN_ARROW "/assets/icons/down_arrow.bin"
#define PREVIOUS_ARROW "/assets/icons/previous_arrow.bin"
#define NEXT_ARROW "/assets/icons/next_arrow.bin"
#define ICON_APPLE_FACETIME "/assets/icons/icon_apple_facetime.bin"
#define ICON_GOOGLE_CALENDAR "/assets/icons/icon_google_calendar.bin"
#define ICON_FACEBOOK "/assets/icons/icon_facebook.bin"
#define ICON_INSTAGRAM "/assets/icons/icon_instagram.bin"
#define ICON_KAKAOTALK "/assets/icons/icon_kakaotalk.bin"
#define ICON_LINE "/assets/icons/icon_line.bin"
#define ICON_LINKEDIN "/assets/icons/icon_linkedin.bin"
#define ICON_APPLE_MAIL "/assets/icons/icon_apple_mail.bin"
#define ICON_MESSENGER "/assets/icons/icon_messenger.bin"
#define ICON_OTHER "/assets/icons/icon_other.bin"
#define ICON_QQ "/assets/icons/icon_qq.bin"
#define ICON_SKYPE "/assets/icons/icon_skype.bin"
#define ICON_SMS "/assets/icons/icon_sms.bin"
#define ICON_SNAP "/assets/icons/icon_snap.bin"
#define ICON_TWITTER "/assets/icons/icon_twitter.bin"
#define ICON_WECHAT "/assets/icons/icon_wechat.bin"
#define ICON_WHATSAPP "/assets/icons/icon_whatsapp.bin"
#define ICON_GMAIL "/assets/icons/icon_gmail.bin"
#define ICON_DINGTALK "/assets/icons/icon_dingtalk.bin"
#define ICON_GOOGLE_CHAT "/assets/icons/icon_google_chat.bin"
#define ICON_DISCORD "/assets/icons/icon_discord.bin"
#define ICON_YOUTUBE "/assets/icons/icon_youtube.bin"
#define ICON_PROHIBIT "/assets/icons/icon_prohibit.bin"
#define ICON_SAND "/assets/icons/icon_send.bin"
#define ICON_TRASH "/assets/icons/icon_trash.bin"
#define ICON_DND_MODE "/assets/icons/icon_dnd_mode.bin"
#define ICON_QRCODE "/assets/icons/icon_qrcode.bin"
#define MOUSE_MODE_ICON "/assets/icons/mouse_mode_icon.bin"
#define SKAIWALKICON "/assets/icons/skaiwalkicon.bin"
#define IMG_CHARGING "/assets/icons/img_charging.bin"
#define ICON_DELETE "/assets/icons/icon_delete.bin"
#define APP_ELC_5 "/assets/icons/app_elc_5.bin"
#define APP_ELC_20 "/assets/icons/app_elc_20.bin"
#define APP_ELC_40 "/assets/icons/app_elc_40.bin"
#define APP_ELC_60 "/assets/icons/app_elc_60.bin"
#define APP_ELC_80 "/assets/icons/app_elc_80.bin"
#define APP_ELC_100 "/assets/icons/app_elc_100.bin"
#define IMG_LOW_POWER "/assets/icons/img_low_power.bin"
#define ICON_BLUETOOTH_DISCONNECTION "/assets/icons/icon_bluetooth_disconnection.bin"
#define CHARGE_ICON "/assets/icons/charge_icon.bin"
#define ICON_X "/assets/icons/icon_x.bin"
#define CALCULATOR_ICON "/assets/icons/calculator_icon.bin"
#define FIND_PHONE "/assets/icons/find_phone.bin"
#define FLISHLIGHT_ICON "/assets/icons/flashlight_icon.bin"
#define IMG_MAIL "/assets/icons/img_mail.bin"

// #define GAUS_DEFAULT_PICTURE "/assets/gaus_images/gaus_default_picture.bin"
#define GAUS_CLOCK5_BG "/assets/gaus_images/gaus_clock5_bg.bin"
#define GAUS_CLOCK4_BG "/assets/gaus_images/gaus_clock4_bg.bin"
#define GAUS_CLOCK1_BG "/assets/gaus_images/gaus_clock1_bg.bin"
#else

LV_IMG_DECLARE(btn_flashlight);
LV_IMG_DECLARE(bg_loading);
LV_IMG_DECLARE(img_flashlight);
LV_IMG_DECLARE(img_activity);
LV_IMG_DECLARE(img_alarm_2);
LV_IMG_DECLARE(img_calendar);
LV_IMG_DECLARE(img_game);
LV_IMG_DECLARE(img_group);
LV_IMG_DECLARE(img_photo);
LV_IMG_DECLARE(img_recorder);
LV_IMG_DECLARE(img_logo);
LV_IMG_DECLARE(img_note);
LV_IMG_DECLARE(img_workout);
LV_IMG_DECLARE(img_itunes);
LV_IMG_DECLARE(img_heart_rate);
LV_IMG_DECLARE(img_calculator);
LV_IMG_DECLARE(img_alarm);
LV_IMG_DECLARE(img_settings);
LV_IMG_DECLARE(img_settings_app);
LV_IMG_DECLARE(img_messages);
LV_IMG_DECLARE(img_mouse);
LV_IMG_DECLARE(small_img_logo_matting);
LV_IMG_DECLARE(up_arrow);
LV_IMG_DECLARE(down_arrow);
LV_IMG_DECLARE(previous_arrow);
LV_IMG_DECLARE(next_arrow);
LV_IMG_DECLARE(icon_apple_facetime);
LV_IMG_DECLARE(icon_google_calendar);
LV_IMG_DECLARE(icon_facebook);
LV_IMG_DECLARE(icon_instagram);
LV_IMG_DECLARE(icon_kakaotalk);
LV_IMG_DECLARE(icon_line);
LV_IMG_DECLARE(icon_linkedin);
LV_IMG_DECLARE(icon_apple_mail);
LV_IMG_DECLARE(icon_messenger);
LV_IMG_DECLARE(icon_other);
LV_IMG_DECLARE(icon_qq);
LV_IMG_DECLARE(icon_skype);
LV_IMG_DECLARE(icon_sms);
LV_IMG_DECLARE(icon_snap);
LV_IMG_DECLARE(icon_twitter);
LV_IMG_DECLARE(icon_wechat);
LV_IMG_DECLARE(icon_whatsapp);
LV_IMG_DECLARE(icon_gmail);
LV_IMG_DECLARE(icon_dingtalk);
LV_IMG_DECLARE(icon_google_chat);
LV_IMG_DECLARE(icon_discord);
LV_IMG_DECLARE(icon_twitch);
LV_IMG_DECLARE(icon_tiktok);
LV_IMG_DECLARE(icon_telegram);
LV_IMG_DECLARE(icon_youtube);
LV_IMG_DECLARE(icon_prohibit);
LV_IMG_DECLARE(icon_send);
LV_IMG_DECLARE(icon_trash);
LV_IMG_DECLARE(icon_qrcode);
LV_IMG_DECLARE(icon_dnd_mode);
LV_IMG_DECLARE(icon_slack);
LV_IMG_DECLARE(icon_lark);
LV_IMG_DECLARE(icon_reddit);
LV_IMG_DECLARE(icon_skaiwalk);
LV_IMG_DECLARE(mouse_mode_icon);
LV_IMG_DECLARE(skaiwalkicon);
LV_IMG_DECLARE(img_charging);
LV_IMG_DECLARE(icon_delete);
LV_IMG_DECLARE(app_elc_5);
LV_IMG_DECLARE(app_elc_20);
LV_IMG_DECLARE(app_elc_40);
LV_IMG_DECLARE(app_elc_60);
LV_IMG_DECLARE(app_elc_80);
LV_IMG_DECLARE(app_elc_100);
LV_IMG_DECLARE(img_low_power);
LV_IMG_DECLARE(icon_bluetooth_disconnection);
LV_IMG_DECLARE(charge_icon);
LV_IMG_DECLARE(icon_x);
LV_IMG_DECLARE(calculator_icon);
LV_IMG_DECLARE(find_phone);
LV_IMG_DECLARE(flashlight_icon);
LV_IMG_DECLARE(img_mail);
// LV_IMG_DECLARE(gaus_clock5_bg);
LV_IMG_DECLARE(gaus_clock4_bg);
LV_IMG_DECLARE(gaus_clock1_bg);
LV_IMG_DECLARE(icon_sleep_mode);
LV_IMG_DECLARE(notification_img);

#define BTN_FLASHLIGHT btn_flashlight
#define IMG_FLASHLIGHT ((const void *)&img_flashlight)
#define IMG_ACTIVITY ((const void *)&img_activity)
#define IMG_ALARM_2 ((const void *)&img_alarm_2)
#define IMG_CALENDAR ((const void *)&img_calendar)
#define IMG_GAME ((const void *)&img_game)
#define IMG_GROUP ((const void *)&img_group)
#define IMG_PHOTO ((const void *)&img_photo)
#define IMG_RECORDER ((const void *)&img_recorder)
#define IMG_LOGO ((const void *)&img_logo)
#define IMG_NOTE ((const void *)&img_note)
#define IMG_WORKOUT ((const void *)&img_workout)
#define IMG_ITUNES ((const void *)&img_itunes)
#define IMG_HEART_RATE ((const void *)&img_heart_rate)
#define IMG_SLEEP ((const void *)&icon_sleep_mode)
#define IMG_CALCULATOR ((const void *)&img_calculator)
#define IMG_ALARM ((const void *)&img_alarm)
#define IMG_SETTINGS ((const void *)&img_settings)
#define IMG_SETTINGS_APP ((const void *)&img_settings_app)
#define IMG_MESSAGES ((const void *)&img_messages)
#define IMG_MOUSE ((const void *)&img_mouse)
#define SMALL_IMG_LOGO_MATTING ((const void *)&small_img_logo_matting)
#define UP_ARROW ((const void *)&up_arrow)
#define DOWN_ARROW ((const void *)&down_arrow)
#define PREVIOUS_ARROW ((const void *)&previous_arrow)
#define NEXT_ARROW ((const void *)&next_arrow)
#define ICON_APPLE_FACETIME ((const void *)&icon_apple_facetime)
#define ICON_GOOGLE_CALENDAR ((const void *)&icon_google_calendar)
#define ICON_FACEBOOK ((const void *)&icon_facebook)
#define ICON_INSTAGRAM ((const void *)&icon_instagram)
#define ICON_KAKAOTALK ((const void *)&icon_kakaotalk)
#define ICON_LINE ((const void *)&icon_line)
#define ICON_LINKEDIN ((const void *)&icon_linkedin)
#define ICON_APPLE_MAIL ((const void *)&icon_apple_mail)
#define ICON_MESSENGER ((const void *)&icon_messenger)
#define ICON_OTHER ((const void *)&icon_other)
#define ICON_QQ ((const void *)&icon_qq)
#define ICON_SKYPE ((const void *)&icon_skype)
#define ICON_TWITCH ((const void *)&icon_twitch)
#define ICON_TIKTOK ((const void *)&icon_tiktok)
#define ICON_TELEGRAM ((const void *)&icon_telegram)
#define ICON_SMS ((const void *)&icon_sms)
#define ICON_SNAP ((const void *)&icon_snap)
#define ICON_TWITTER ((const void *)&icon_twitter)
#define ICON_WECHAT ((const void *)&icon_wechat)
#define ICON_WHATSAPP ((const void *)&icon_whatsapp)
#define ICON_GMAIL ((const void *)&icon_gmail)
#define ICON_DINGTALK ((const void *)&icon_dingtalk)
#define ICON_GOOGLE_CHAT ((const void *)&icon_google_chat)
#define ICON_DISCORD ((const void *)&icon_discord)
#define ICON_YOUTUBE ((const void *)&icon_youtube)
#define ICON_PROHIBIT ((const void *)&icon_prohibit)
#define ICON_SAND ((const void *)&icon_send)
#define ICON_TRASH ((const void *)&icon_trash)
#define ICON_QRCODE ((const void *)&icon_qrcode)
#define ICON_DND_MODE ((const void *)&icon_dnd_mode)
#define ICON_SLEEP_MODE ((const void *)&icon_sleep_mode)
#define ICON_SLACK ((const void *)&icon_slack)
#define ICON_LARK ((const void *)&icon_lark)
#define ICON_REDDIT ((const void *)&icon_reddit)
#define ICON_SKAIWALK ((const void *)&icon_skaiwalk)
#define MOUSE_MODE_ICON ((const void *)&mouse_mode_icon)
#define SKAIWALKICON ((const void *)&skaiwalkicon)
#define IMG_CHARGING ((const void *)&img_charging)
#define ICON_DELETE ((const void *)&icon_delete)
#define APP_ELC_5 ((const void *)&app_elc_5)
#define APP_ELC_20 ((const void *)&app_elc_20)
#define APP_ELC_40 ((const void *)&app_elc_40)
#define APP_ELC_60 ((const void *)&app_elc_60)
#define APP_ELC_80 ((const void *)&app_elc_80)
#define APP_ELC_100 ((const void *)&app_elc_100)
#define IMG_LOW_POWER ((const void *)&img_low_power)
#define ICON_BLUETOOTH_DISCONNECTION ((const void *)&icon_bluetooth_disconnection)
#define CHARGE_ICON ((const void *)&charge_icon)
#define ICON_X ((const void *)&icon_x)
#define CALCULATOR_ICON ((const void *)&calculator_icon)
#define FIND_PHONE ((const void *)&find_phone)
#define FLISHLIGHT_ICON ((const void *)&flashlight_icon)

#define IMG_MAIL ((const void *)&img_mail)
#define NOTIFICATION_IMG ((const void *)&notification_img)
// #define GAUS_CLOCK5_BG (&gaus_clock5_bg)
#define GAUS_CLOCK5_BG "/assets/gaus_images/gaus_clock5_bg.bin"
#define GAUS_CLOCK4_BG (&gaus_clock4_bg)
#define GAUS_CLOCK1_BG (&gaus_clock1_bg)

#endif
extern char *GAUS_DEFAULT_PICTURE;
#define MEDIA_MASK "/assets/images/media_mask.bin"
extern char MEDIA_IMG[40];
extern char MEDIA_HEADER_IMG[40];
#endif

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/