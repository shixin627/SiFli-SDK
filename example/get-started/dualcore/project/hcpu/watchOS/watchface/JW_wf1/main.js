import * as lv from "lv"
import { app } from "lvapp"
import * as lv_enums from "/lv_enums.js"
import { analogclk } from "/analogclk.js"
import { idximg } from "/idximg.js"
import { label } from "/label.js"
import { img } from "/img.js"

class wf1 extends app {
    constructor() {
        super(1);		// Watch APP set 1 as parameter
    }

    refresh() {
        var cur_time = new Date();
        // print(cur_time.getHours());
        // print(cur_time.getMinutes());
        this.time_ul.select(cur_time.getHours() / 10);
        this.time_ur.select(cur_time.getHours() % 10);
        this.time_bl.select(cur_time.getMinutes() / 10);
        this.time_br.select(cur_time.getMinutes() % 10);
        var weekdays = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
        var month = cur_time.getMonth() + 1;
        var day = cur_time.getDate();
        var weekday = weekdays[cur_time.getDay()];
        this.date_label.set_text(weekday + " " + ((month <= 9) ? "0" : "") + month + "/" + ((day <= 9) ? "0" : "") + day);
    }
    start() {
        // // Upper bg
        // this.bg = new analogclk(this.root());
        // this.bg.set_pos(0, 0);
        // this.bg.img("/JW_wf3/bg.bin");
        // Upper left
        this.time_ul = new idximg(this.root());
        this.time_ul.set_pos(23, 110);
        this.time_ul.prefix("/JW_wf1/dig_1_");

        // Upper right
        this.time_ur = new idximg(this.root());
        this.time_ur.set_pos(128, 110);
        this.time_ur.prefix("/JW_wf1/dig_1_");

        // Bottom left
        this.time_bl = new idximg(this.root());
        this.time_bl.set_pos(238, 110);
        this.time_bl.prefix("/JW_wf1/dig_2_");

        // Bottom right
        this.time_br = new idximg(this.root());
        this.time_br.set_pos(343, 110);
        this.time_br.prefix("/JW_wf1/dig_2_");

        // Upper bg
        this.bg = new img(this.root());
        this.bg.set_pos(0, 160);
        this.bg.set_src("/JW_wf1/bg.bin");

        this.date_label = new label(this.root());
        this.date_label.align(lv_enums.ALIGN_IN_TOP_MID, 0, 30);
        this.date_label.set_local_font(lv_enums.FONT_TITLE, lv_enums.LV_COLOR_WHITE);
        this.refresh();
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
globalThis.wf1 = wf1;
