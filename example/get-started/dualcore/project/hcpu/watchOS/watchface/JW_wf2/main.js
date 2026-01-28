import * as lv from "lv"
import * as os from "os"
import { app } from "lvapp"
import * as lv_enums from "/lv_enums.js"
import { analogclk } from "/analogclk.js"
import { label } from "/label.js"
import { lvsfezipa } from "/lvsfezipa.js"
import { img } from "/img.js"

class wf2 extends app {
    constructor() {
        super(1);		// Watch APP set 1 as parameter
    }

    refresh() {
        var cur_time = new Date();
        if (cur_time.getMinutes() != this.last_min) {
            this.hour.set_text(((cur_time.getHours() <= 9) ? "0" : "") + String(cur_time.getHours()))
            this.min.set_text(((cur_time.getMinutes() <= 9) ? "0" : "") + String(cur_time.getMinutes()));
            var weekdays = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
            var month = cur_time.getMonth() + 1;
            var day = cur_time.getDate();
            var weekday = weekdays[cur_time.getDay()];
            this.date.set_text(weekday + " " + ((month <= 9) ? "0" : "") + month + "/" + ((day <= 9) ? "0" : "") + day);
            // 檢查 /JW_wf2/picture.bin 是否存在及是否有變化
            var files = os.readdir("/JW_wf2");
            var hasPicture = false;
            var pictureSize = 0;
            if (files && files[0]) {
                for (var i = 0; i < files[0].length; i++) {
                    if (files[0][i] == "picture.bin") {
                        hasPicture = true;
                        // 取得檔案大小來判斷是否有變化
                        var stat = os.stat("/JW_wf2/picture.bin");
                        if (stat && stat[0]) {
                            pictureSize = stat[0].size;
                        }
                        break;
                    }
                }
            }
            // 只有在狀態變化時才刷新圖片：
            // 1. 從沒有 picture.bin 變成有 picture.bin
            // 2. 從有 picture.bin 變成沒有 picture.bin
            // 3. picture.bin 大小改變（表示檔案被更新）
            if (hasPicture != this.lastHasPicture || pictureSize != this.lastPictureSize) {
                print("Picture state changed for wf2: hasPicture=" + hasPicture + ", size=" + pictureSize);
                if (hasPicture) {
                    this.bg4.set_src("/JW_wf2/picture.bin");
                } else {
                    this.bg4.set_src("/JW_wf2/default_picture.bin");
                }
                this.lastHasPicture = hasPicture;
                this.lastPictureSize = pictureSize;
            }
        }
    }

    start() {
        this.bg4 = new img(this.root());
        this.bg4.align(lv_enums.ALIGN_CENTER, 0, 0);

        this.timebg = new img(this.root());
        this.timebg.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -25);
        this.timebg.set_src("/assets/images/time_bg.bin");

        // Dot
        this.dot = new label(this.root());
        this.dot.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -60);
        this.dot.set_text(":");
        this.dot.set_local_font(lv_enums.FONT_SUPER, lv_enums.LV_COLOR_WHITE);

        // Hour 
        this.hour = new label(this.root());
        this.hour.align_to(this.dot.nativeobj, lv_enums.ALIGN_OUT_LEFT_TOP, -75, 5);
        this.hour.set_local_font(lv_enums.FONT_SUPER, lv_enums.LV_COLOR_WHITE);

        // Minute
        this.min = new label(this.root());
        this.min.align_to(this.dot.nativeobj, lv_enums.ALIGN_OUT_RIGHT_TOP, 23, 5);
        this.min.set_local_font(lv_enums.FONT_SUPER, lv_enums.LV_COLOR_WHITE);

        // Date
        this.date = new label(this.root());
        this.date.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -30);
        this.date.set_local_font(lv_enums.FONT_TITLE, lv_enums.LV_COLOR_WHITE);

        this.last_min = -1;
        this.lastHasPicture = null;  // 記錄上次是否有 picture.bin
        this.lastPictureSize = -1;   // 記錄上次 picture.bin 的大小
        this.refresh();
    }
    pause() {
        this.task();
    }
    resume() {
        this.task(
            function () {
                this.refresh();
            }
            , 1000
        );
    }
}
globalThis.wf2 = wf2;
