import * as lv from "lv"
import * as os from "os"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {qrcode} from "/qrcode.js"
// import {lvsfbarcode} from "/lvsfbarcode.js"
import {label} from "/label.js"

function arrayBufferToString(buffer){
    var arr = new Uint8Array(buffer);
    var str = String.fromCharCode.apply(String, arr);
    return str;
}

class app1 extends app{	
	start() {
		// Step 1: Create an ArrayBuffer with the appropriate size
		const url = "https://skaiwalk.com/watch_id=";
		print("URL:", url);
		
		const buffer = new ArrayBuffer(50);
		
		this.f=os.open("/JA_app1/code.txt", os.O_RDONLY);
		os.read(this.f,buffer,0,50);
		var code = arrayBufferToString(buffer);
		print("Code:", code);
		var totol = url + code;
		os.close(this.f);
		
		// Demo for QRcode		
        this.qrcode = new qrcode(this.root());
        this.qrcode.setparam((466/2),lv_enums.LV_COLOR_WHITE, lv_enums.LV_COLOR_BLACK);
		this.qrcode.set_text(totol,2); 
		// print("ALIGN_IN_TOP_MID=",lv_enums.ALIGN_IN_TOP_MID)
        // this.qrcode.align(lv_enums.ALIGN_IN_TOP_MID, 0,(466/6));
		print("ALIGN_CENTER=", lv_enums.ALIGN_CENTER)
		this.qrcode.align(lv_enums.ALIGN_CENTER, 0, -20);
        this.qrcode.set_event_cb(
			function(event){
				print("qrcode event ", event);
				if (event==lv_enums.EVENT_CLICKED) {
					lv.gui_app_self_exit();
                }
			}
		);
		print("Qrcode created");
		// Demo for Barcode
        // this.barcode = new lvsfbarcode(this.root());
		// this.barcode.set_text("0705632441974");
        // this.barcode.align(lv_enums.ALIGN_IN_TOP_MID, 0, (466*2/3));
        // this.barcode.set_event_cb(
		// 	function(event){
		// 		print("barcode event ", event);
		// 	}
		// );  
		// print("Barcode created");

		// Text for code
		this.label=new label(this.root());
        this.label.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -70);
        this.label.set_local_font(lv_enums.FONT_BIGL, lv_enums.LV_COLOR_WHITE);
		this.label.set_text(code);
		// Demo for task
		this.count=0;
		this.task(
			function() {
				print("Task called ", this.count);
				this.count++;
			}
			, 1000
		);
        print("Started\n");
    }    
	resume() {
		// Demo for OS directory
		os.chdir("JA_app1");
		print(os.getcwd());
		var local_files=os.readdir("/");
		print(local_files[0].length);
		for (var i=0;i<local_files[0].length;i++)
		{ 		
			print(local_files[0][i]);
		}		
	}
}
globalThis.app1 = app1;
