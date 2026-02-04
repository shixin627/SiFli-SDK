import * as lv from "lv"
import { app } from "lvapp"
import * as lv_enums from "/lv_enums.js"
import { label } from "/label.js"
import { img } from "/img.js"
import {rlottie} from "/rlottie.js"

var LV_HOR_RES_MAX = lv.get_hor_max();
var LV_VER_RES_MAX = lv.get_ver_max();

class app3 extends app {
	refresh() {
		// this.date.set_text(this.get_time(true));
		print("Task called ", this.count);
		this.count++;
	}

	get_time(sec_need) {
		var cur_time = new Date();
		var hour = cur_time.getHours();
		var min = cur_time.getMinutes();
		var sec = "";
		if (sec_need) {
			sec += ":" + String(cur_time.getSeconds()).padStart(2, '0');
		}

		print(String(hour) + ':' + String(min).padStart(2, '0') + sec);
		return String(hour) + ':' + String(min).padStart(2, '0') + sec;
	}

	modular_textField(par) {
		this.textField = new img(par);
		this.textField.set_src("/assets/images/text_card.bin");
		this.textField.align(lv_enums.ALIGN_CENTER, 0, 0);
		this.textField.set_event_cb(
			function (event) {
				print("textField event ", event);
				if (event == lv_enums.EVENT_CLICKED) {
					print("textField clicked");
					lv.gui_app_self_exit();
				}
			}
		);

		this.date = new label(par);
		this.date.align(lv_enums.ALIGN_CENTER, 0, 0);
		this.date.set_local_font(lv_enums.FONT_TITLE, lv_enums.LV_COLOR_WHITE);
		var message = "我能帮上什么忙？";
		this.date.set_text(message);
	}

	start() {
		// BG image
		this.bg = new img(this.root());
		this.bg.set_src("/assets/images/bg_main.bin");

		// logo icon
		this.logo = new img(this.root());
		this.logo.set_src("/assets/icons/logo.bin");
		this.logo.align(lv_enums.ALIGN_IN_TOP_MID, 0, 10);
		this.logo.set_event_cb(
			function (event) {
				print("logo event ", event);
				if (event == lv_enums.EVENT_CLICKED) {
					lv.gui_app_self_exit();
				}
			}
		);

		this.modular_textField(this.root());

		this.lottie = new rlottie(this.root());
		// this.lottie.set_size(LV_HOR_RES_MAX / 5, LV_HOR_RES_MAX / 5);
		// this.lottie.file("/assets/Lottie/Lego.json");
		// this.lottie.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -30);
		this.lottie.set_size(LV_HOR_RES_MAX / 3, LV_HOR_RES_MAX / 3);
		this.lottie.file("/assets/Lottie/cheney.json");
		this.lottie.align(lv_enums.ALIGN_CENTER, 0, 0);

		this.mic = new img(this.root());
		this.mic.set_src("/assets/icons/mic_fill.bin");
		this.mic.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -20);
		this.mic.set_event_cb(
			function (event) {
				print("mic event ", event);
				if (event == lv_enums.EVENT_CLICKED) {
					print("mic clicked");
				}
			}
		);
		this.count = 0;
		this.refresh();
	}
	pause() {
		this.task();
		this.lottie.play(0);
	}
	resume() {
		this.task(
			function () {
				this.refresh();
			}
			, 1000
		);
		this.lottie.play(1);
	}
}
globalThis.app3 = app3;
