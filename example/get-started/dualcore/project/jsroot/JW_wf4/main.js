import { app } from "lvapp"
import { analogclk } from "/analogclk.js"

class wf4 extends app {
    constructor() {
        super(1);
    }
    start() {
        this.anaclk = new analogclk(this.root());
        this.anaclk.img("/JW_wf4/bg.bin", "/JW_wf4/hour.bin", "/JW_wf4/minute.bin", "/JW_wf4/second.bin");
        this.rate = 30;
    }
    pause() {
        this.anaclk.refr_inteval(0);
    }
    resume() {
        this.anaclk.refr_inteval(this.rate);
    }
}
globalThis.wf4 = wf4;
