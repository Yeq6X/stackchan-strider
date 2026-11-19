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
button,.seg,.step{-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;-webkit-tap-highlight-color:transparent;touch-action:manipulation}
main{min-height:calc(100dvh - 52px)}
.grow{flex:1}
.seg{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.seg button[aria-pressed=true]{background:var(--ink);border-color:var(--ink);color:var(--bg);outline:0}
.step{display:grid;grid-template-columns:1fr auto 1fr;gap:8px;align-items:center}
.step output{min-width:4.5em;text-align:center;font-variant-numeric:tabular-nums;font-weight:700}
.pad{display:flex;flex-direction:column;gap:10px}
.pad button{min-height:104px;font-size:24px;touch-action:none}
.pad button.held{transform:scale(.98);filter:brightness(.88)}
.pad button[aria-pressed=true]{outline:0}
details summary{color:var(--soft);font-size:14px}
details .card{margin-top:10px}
</style></head><body><main>
<h1>STACKCHAN <b>STRIDER</b></h1>
<p class="note state" id="state">接続を確認しています…</p>
<div class="card">
  <h2>顔の上下</h2>
  <div class="step"><button id="down">下へ</button><output id="pitch-out"></output><button id="up">上へ</button></div>
  <h2>速さ</h2>
  <div class="seg" id="speeds">
    <button data-speed="20" aria-pressed="false">ゆっくり</button>
    <button data-speed="35" aria-pressed="true">ふつう</button>
    <button data-speed="55" aria-pressed="false">はやい</button>
  </div>
</div>
<div class="grow"></div>
<div class="pad">
  <button class="go" id="fwd">前進</button>
  <button id="back">後退</button>
</div>
<p class="note">ボタンを押している間だけ歩きます。離すと止まります。</p>
<details>
  <summary>設定</summary>
  <div class="card">
    <label style="flex-direction:row;align-items:center;gap:8px;color:var(--ink)"><input type="checkbox" id="invert">前進と後退を入れ替える</label>
    <p class="note">「前進」で後ろへ歩く場合にチェックします。本体に保存されます。</p>
    <label style="flex-direction:row;align-items:center;gap:8px;color:var(--ink)"><input type="checkbox" id="smile">歩いている間は笑顔にする</label>
    <button id="forget">スマホとのつなぎ方を選び直す</button>
    <p class="note">「直接つなぐ」と「家の Wi-Fi を使う」を選び直します。いまの設定は消えます。</p>
  </div>
</details>
</main>
<script>
const $ = (id) => document.getElementById(id);
let speed = 35, pitch = 10, held = 0, timer = null;
const call = (path) => fetch(path, { cache: "no-store" }).then((r) => r.json());
function show(s) {
  $("invert").checked = !!s.invert;
  $("smile").checked = !!s.smile;
  $("state").textContent = (s.dir ? (s.dir > 0 ? "前進中" : "後退中") : "停止中") + " / 電池 " + s.volt.toFixed(2) + " V";
}
function lost() { $("state").textContent = "スタックチャンと通信できません。本体の頭の上をさわると画面に QR が出るので、読み直してください"; }
// 押している間は 0.3 秒ごとに合図を送る。途切れると本体が自動で止まる
function start(dir, button) {
  held = dir;
  button.classList.add("held");
  call("/api/drive?dir=" + dir + "&speed=" + speed).then(show).catch(lost);
  clearInterval(timer);
  timer = setInterval(() => { if (held) call("/api/ping").then(show).catch(lost); }, 300);
}
function stop() {
  if (!held) return;
  held = 0;
  clearInterval(timer);
  document.querySelectorAll(".pad button").forEach((b) => b.classList.remove("held"));
  call("/api/stop").then(show).catch(lost);
}
[["fwd", 1], ["back", -1]].forEach(([id, dir]) => {
  const button = $(id);
  button.addEventListener("pointerdown", (event) => { event.preventDefault(); button.setPointerCapture(event.pointerId); start(dir, button); });
  ["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => button.addEventListener(name, stop));
  button.addEventListener("contextmenu", (event) => event.preventDefault());
});
// 画面を切り替えた、ほかのアプリへ移った、などのときも止める
document.addEventListener("visibilitychange", () => { if (document.hidden) stop(); });
window.addEventListener("blur", stop);
window.addEventListener("pagehide", stop);
document.addEventListener("selectstart", (event) => { if (event.target.closest("button,.seg,.step")) event.preventDefault(); });

$("speeds").addEventListener("click", (event) => {
  const button = event.target.closest("button[data-speed]");
  if (!button) return;
  speed = Number(button.dataset.speed);
  document.querySelectorAll("#speeds button").forEach((b) => b.setAttribute("aria-pressed", String(b === button)));
  if (held) call("/api/drive?dir=" + held + "&speed=" + speed).then(show).catch(lost);
});
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
