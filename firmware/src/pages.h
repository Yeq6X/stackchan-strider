// スマホのブラウザに出すページ(本体に内蔵。外部への通信はしない)
#pragma once
#include <Arduino.h>

// どちらのページでも使う見た目
#define STRIDER_PAGE_HEAD \
    "<!doctype html><html lang=\"ja\"><head><meta charset=\"utf-8\">" \
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,viewport-fit=cover\">" \
    "<style>" \
    ":root{--bg:#e9ecef;--card:#f8f9fa;--ink:#1d232c;--soft:#5b6573;--line:#cfd5dc;--red:#cf2e25;--red-ink:#fff}" \
    "@media(prefers-color-scheme:dark){:root{--bg:#12151a;--card:#1b2027;--ink:#e6e9ee;--soft:#98a2b1;--line:#2e3540;--red:#ff6257;--red-ink:#1f0705}}" \
    "*{box-sizing:border-box}body{margin:0;padding:20px 16px 32px;background:var(--bg);color:var(--ink);" \
    "font:16px/1.6 system-ui,-apple-system,'Hiragino Sans','Noto Sans JP',sans-serif}" \
    "main{max-width:26rem;margin:0 auto;display:flex;flex-direction:column;gap:14px}" \
    "h1{margin:0;font-size:22px;letter-spacing:.04em}h1 b{color:var(--red)}h2{margin:0;font-size:15px;color:var(--soft);font-weight:500}" \
    ".card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px;display:flex;flex-direction:column;gap:12px}" \
    "button{font:inherit;font-weight:700;border-radius:10px;border:1px solid var(--line);background:var(--bg);color:var(--ink);padding:14px;min-height:52px}" \
    "button.go{background:var(--red);border-color:var(--red);color:var(--red-ink)}button[aria-pressed=true]{outline:3px solid var(--red);outline-offset:2px}" \
    "button:disabled{opacity:.5}label{display:flex;flex-direction:column;gap:4px;font-size:14px;color:var(--soft)}" \
    "input,select{font:inherit;padding:12px;border-radius:8px;border:1px solid var(--line);background:var(--bg);color:var(--ink);width:100%}" \
    "input[type=range]{padding:0;accent-color:var(--red)}input[type=checkbox]{width:auto}" \
    ".row{display:grid;grid-template-columns:1fr 1fr;gap:10px}.wide{grid-column:1/-1}" \
    ".note{font-size:13px;color:var(--soft);margin:0}.state{font-variant-numeric:tabular-nums}" \
    "</style>"

// ---------------------------------------------------------------- 操作ページ(家の Wi-Fi につながっているとき / 直接つないでいるとき)
static const char CONTROL_PAGE[] PROGMEM = STRIDER_PAGE_HEAD R"HTML(
<title>Stackchan Strider</title>
<style>
/* 長押ししても文字が選択されたり、メニューが出たりしないようにする */
button,.step,.stick{-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;-webkit-tap-highlight-color:transparent;touch-action:manipulation}
main{min-height:calc(100dvh - 52px)}
.grow{flex:1}
.step{display:grid;grid-template-columns:1fr auto 1fr;gap:8px;align-items:center}
.step output{min-width:4.5em;text-align:center;font-variant-numeric:tabular-nums;font-weight:700}
/* スティック: 正方形の台の上で、つまみを指で動かす。左右の帯(台の幅の 3 割ずつ)に入ると旋回 */
.stick{position:relative;width:min(100%,20rem);aspect-ratio:1;margin:0 auto;border-radius:20px;border:1px solid var(--line);background:var(--card);touch-action:none;overflow:hidden}
.stick .zone{position:absolute;top:0;bottom:0;width:30%;background:var(--bg);display:flex;align-items:center;justify-content:center;font-size:13px;color:var(--soft);font-weight:700}
.stick .zone.on{background:var(--red);color:var(--red-ink)}
.stick .left{left:0;border-right:1px dashed var(--line)}.stick .right{right:0;border-left:1px dashed var(--line)}
.stick .tag{position:absolute;left:30%;right:30%;text-align:center;font-size:13px;color:var(--soft);font-weight:700}
.stick .tag.on{color:var(--red)}
.stick .top{top:8px}.stick .bottom{bottom:8px}
.stick .knob{position:absolute;left:50%;top:50%;width:26%;aspect-ratio:1;border-radius:50%;background:var(--red);transform:translate(-50%,-50%);pointer-events:none;box-shadow:0 2px 8px rgba(0,0,0,.25)}
.stick.held .knob{filter:brightness(.88)}
details summary{color:var(--soft);font-size:14px}
details .card{margin-top:10px}
</style></head><body><main>
<h1>STACKCHAN <b>STRIDER</b></h1>
<p class="note state" id="state">接続を確認しています…</p>
<div class="card">
  <h2>顔の上下</h2>
  <div class="step"><button id="down">下へ</button><output id="pitch-out"></output><button id="up">上へ</button></div>
</div>
<div class="grow"></div>
<div class="stick" id="stick">
  <div class="zone left" id="zone-left">左に回る</div>
  <div class="zone right" id="zone-right">右に回る</div>
  <div class="tag top" id="tag-fwd">前進</div>
  <div class="tag bottom" id="tag-back">後退</div>
  <div class="knob" id="knob"></div>
</div>
<p class="note">つまみを上下に動かすと歩き、大きく動かすほど速くなります。左右の帯に入れると、その場で向きを変えます。指を離すと止まります。</p>
<details>
  <summary>設定</summary>
  <div class="card">
    <button id="calib">旋回の調整をする</button>
    <p class="note" id="calib-note">左右に回れるようにするための調整です。平らな床に置いて押すと、8 秒ほど前に歩いて、同じだけ戻ります(前に 60 cm ほど空けてください)。歩行ベースを組んだあとに 1 回やれば、本体に保存されます。組み直したときは、やり直してください。</p>
    <button id="uncalib" hidden>旋回の調整を消す</button>
    <label style="color:var(--ink)">歩く速さの上限 <output id="limit-out"></output>
      <input type="range" id="limit" min="20" max="100" step="10" value="80"></label>
    <p class="note">つまみを端まで動かしたときの速さです。旋回の強さは変わりません。本体に保存されます。</p>
    <label style="flex-direction:row;align-items:center;gap:8px;color:var(--ink)"><input type="checkbox" id="invert">前進と後退を入れ替える</label>
    <p class="note">つまみを上に動かして後ろへ歩く場合にチェックします。本体に保存されます。</p>
    <label style="flex-direction:row;align-items:center;gap:8px;color:var(--ink)"><input type="checkbox" id="smile">歩いている間は笑顔にする</label>
    <button id="forget">スマホとのつなぎ方を選び直す</button>
    <p class="note">「直接つなぐ」と「家の Wi-Fi を使う」を選び直します。いまの設定は消えます。</p>
  </div>
</details>
</main>
<script>
const $ = (id) => document.getElementById(id);
let calibrating = false, pitch = 10, held = false, timer = null, stickX = 0, stickY = 0, sending = false, dirty = false;
const call = (path) => fetch(path, { cache: "no-store" }).then((r) => r.json());
const names = { "0": "停止中", "1": "前進中", "-1": "後退中", "2": "右に旋回中", "-2": "左に旋回中" };
function show(s) {
  $("invert").checked = !!s.invert;
  $("smile").checked = !!s.smile;
  if (document.activeElement !== $("limit")) { $("limit").value = s.limit; $("limit-out").textContent = s.limit + "%"; }
  const calib = ["旋回は未調整(設定で調整してください)", "旋回の調整中…", "", "旋回の調整に失敗しました。平らな床でやり直してください"][s.calib];
  $("state").textContent = (s.calib === 1 ? "" : names[s.drive] + " / ") + "電池 " + s.volt.toFixed(2) + " V" + (calib ? " / " + calib : "");
  $("calib").textContent = s.calib === 1 ? "調整をやめる" : s.center >= 0 ? "旋回の調整をやり直す" : "旋回の調整をする";
  calibrating = s.calib === 1;
  // 調整を消すボタンは、調整が済んでいて、調整中でないときだけ出す
  $("uncalib").hidden = s.center < 0 || calibrating;
  if ($("uncalib").hidden) { delete $("uncalib").dataset.armed; $("uncalib").textContent = "旋回の調整を消す"; }
}
function lost() { $("state").textContent = "スタックチャンと通信できません。本体の頭の上をさわると画面に QR が出るので、読み直してください"; }
// スティックの倒れを送る。前の送信の返事を待ってから次を送る(たまった古い指令が、あとから届かないようにする)
function send() {
  if (!held) return;
  if (sending) { dirty = true; return; }
  sending = true;
  dirty = false;
  call("/api/stick?x=" + stickX + "&y=" + stickY).then((s) => { if (held) show(s); }).catch(lost).finally(() => {
    sending = false;
    // 送っている間に指が離れていたら、止める指令をもう一度送る(この送信が、止める指令より後に届いた場合に備える)
    if (!held) call("/api/stop").then(show).catch(lost);
    else if (dirty) send();
  });
}
// つまみの位置(台の中心からの割合)を、倒れ(-100〜100。右と上が正)にする。左右の帯は台の幅の 3 割ずつで、倒れ 40 以上にあたる
function place(event) {
  const box = $("stick").getBoundingClientRect();
  const clamp = (v) => Math.max(-1, Math.min(1, v));
  const x = clamp((event.clientX - box.left) / box.width * 2 - 1), y = clamp(1 - (event.clientY - box.top) / box.height * 2);
  stickX = Math.round(x * 100);
  stickY = Math.round(y * 100);
  draw(x, y);
  send();
}
function draw(x, y) {
  // つまみが台からはみ出さないよう、動く範囲を台の 74% にする(つまみの直径が 26%)
  $("knob").style.left = (50 + x * 37) + "%";
  $("knob").style.top = (50 - y * 37) + "%";
  const turn = Math.abs(stickX) >= 40;
  $("zone-left").classList.toggle("on", turn && stickX < 0);
  $("zone-right").classList.toggle("on", turn && stickX > 0);
  $("tag-fwd").classList.toggle("on", !turn && stickY >= 15);
  $("tag-back").classList.toggle("on", !turn && stickY <= -15);
}
function stop() {
  if (!held) return;
  held = false;
  clearInterval(timer);
  stickX = stickY = 0;
  draw(0, 0);
  $("stick").classList.remove("held");
  call("/api/stop").then(show).catch(lost);
}
$("stick").addEventListener("pointerdown", (event) => {
  event.preventDefault();
  $("stick").setPointerCapture(event.pointerId);
  held = true;
  $("stick").classList.add("held");
  place(event);
  // 触れている間は、動かさなくても 0.3 秒ごとに送る。途切れると本体が自動で止まる
  clearInterval(timer);
  timer = setInterval(send, 300);
});
$("stick").addEventListener("pointermove", (event) => { if (held) place(event); });
["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => $("stick").addEventListener(name, stop));
$("stick").addEventListener("contextmenu", (event) => event.preventDefault());
// 画面を切り替えた、ほかのアプリへ移った、などのときも止める
document.addEventListener("visibilitychange", () => { if (document.hidden) stop(); });
window.addEventListener("blur", stop);
window.addEventListener("pagehide", stop);
document.addEventListener("selectstart", (event) => { if (event.target.closest("button,.step,.stick")) event.preventDefault(); });

function look(delta) {
  pitch = Math.min(60, Math.max(0, pitch + delta));
  $("pitch-out").textContent = pitch + "°";
  $("down").disabled = pitch <= 0;
  $("up").disabled = pitch >= 60;
  if (delta) call("/api/look?pitch=" + pitch).catch(lost);
}
$("down").onclick = () => look(-10);
$("up").onclick = () => look(10);
$("invert").onchange = () => call("/api/invert?on=" + ($("invert").checked ? 1 : 0)).then(show).catch(lost);
$("calib").onclick = () => call(calibrating ? "/api/stop" : "/api/calibrate").then(show).catch(lost);
$("uncalib").onclick = () => {
  if ($("uncalib").dataset.armed) call("/api/uncalibrate").then(show).catch(lost);
  else { $("uncalib").dataset.armed = "1"; $("uncalib").textContent = "もう一度押すと、旋回の調整を消します"; }
};
// 調整の間は、終わったかどうかを 1 秒ごとに確かめる
setInterval(() => { if (calibrating && !held) call("/api/state").then(show).catch(lost); }, 1000);
$("limit").oninput = () => { $("limit-out").textContent = $("limit").value + "%"; };
$("limit").onchange = () => call("/api/limit?v=" + $("limit").value).then(show).catch(lost);
$("smile").onchange = () => call("/api/smile?on=" + ($("smile").checked ? 1 : 0)).then(show).catch(lost);
$("forget").onclick = () => {
  if ($("forget").dataset.armed) { call("/api/forget").catch(() => {}); $("state").textContent = "つなぎ方の設定を消しました。スタックチャンの画面で、つなぎ方を選んでください"; }
  else { $("forget").dataset.armed = "1"; $("forget").textContent = "もう一度押すと、つなぎ方の設定を消します"; }
};
look(0);
call("/api/state").then(show).catch(lost);
</script></body></html>
)HTML";

// ---------------------------------------------------------------- 設定ページ(「家の Wi-Fi を使う」を選んだとき / 家の Wi-Fi につながらないとき)
static const char SETUP_PAGE[] PROGMEM = STRIDER_PAGE_HEAD R"HTML(
<title>Stackchan Strider の設定</title></head><body><main>
<h1>STACKCHAN <b>STRIDER</b></h1>
<form class="card" id="form">
  <h2>スタックチャンを家の Wi-Fi につなぐ</h2>
  <label>Wi-Fi の名前
    <select id="ssid"><option value="">探しています…</option></select></label>
  <label>パスワード
    <input id="pass" type="password" autocomplete="off" autocapitalize="off"></label>
  <button class="go" id="save">この Wi-Fi につなぐ</button>
  <button type="button" id="rescan">もう一度探す</button>
  <p class="note" id="msg">2.4GHz の Wi-Fi だけ使えます。パスワードはスタックチャンの中にだけ保存されます。</p>
</form>
<p class="note">家の Wi-Fi を使わずに操作したいときは、スタックチャンの画面の右上にある「つなぎ方を選び直す」をタッチして、「直接つなぐ」を選んでください。</p>
</main>
<script>
const $ = (id) => document.getElementById(id);
function scan() {
  $("ssid").innerHTML = '<option value="">探しています…</option>';
  fetch("/api/scan", { cache: "no-store" }).then((r) => r.json()).then((list) => {
    $("ssid").innerHTML = "";
    if (!list.length) { $("ssid").innerHTML = '<option value="">見つかりませんでした</option>'; return; }
    list.forEach((net) => { const o = document.createElement("option"); o.value = net.ssid; o.textContent = net.ssid + (net.lock ? "" : "(パスワードなし)"); $("ssid").appendChild(o); });
  }).catch(() => { $("msg").textContent = "Wi-Fi を探せませんでした。「もう一度探す」を押してください"; });
}
$("rescan").onclick = scan;
$("form").onsubmit = (event) => {
  event.preventDefault();
  if (!$("ssid").value) { $("msg").textContent = "Wi-Fi を選んでください"; return; }
  $("save").disabled = true;
  const body = new URLSearchParams({ ssid: $("ssid").value, pass: $("pass").value });
  fetch("/api/wifi", { method: "POST", body: body }).then(() => {
    $("msg").textContent = "保存しました。スタックチャンが再起動して、家の Wi-Fi につなぎます。つながるとスタックチャンの画面に QR が出るので、スマホで読んでください。操作ページが開きます。QR が出ない場合はパスワードを確かめて、もう一度この設定をしてください。";
  }).catch(() => { $("save").disabled = false; $("msg").textContent = "保存できませんでした。もう一度押してください"; });
};
scan();
</script></body></html>
)HTML";
