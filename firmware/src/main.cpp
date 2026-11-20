// Stackchan Strider: M5Stack 製品版スタックチャン(K151)を、首振りサーボの連続回転で歩かせるファームウェア。
//
//  - 画面はスタックチャンの顔(m5stack-avatar)。歩行中に笑顔にするかは設定で選べる(初期状態は変えない)
//  - はじめて起動すると、画面でつなぎ方を選ぶ(直接つなぐ / 家の Wi-Fi を使う)
//  - 直接つなぐ: 本体が Wi-Fi を出す。画面の QR をスマホで読んでつなぎ、そのまま操作する
//  - 家の Wi-Fi を使う: 設定用の Wi-Fi を出し、画面の QR をスマホで読んで家の Wi-Fi を設定する。
//    つながると画面に操作ページの QR を出す
//  - 操作はスマホのブラウザから。操作ページのスティックを上下に倒すと前進・後退(倒した分だけ速い)、
//    左右に倒すと、その場で左右に向きを変える(旋回)
//  - スティックに触れている間だけ動く。触れている間は 0.3 秒ごとに合図が来て、0.9 秒途切れたら自動で止まる
//
// 注意: この機体では土台が回り続けるので、首(ヨー)を「角度指定」で動かす命令(goHome / stop / moveX)は使わない。
//       それらは原点の角度まで首を回そうとして、脚が勝手に動く。止めるときは出力 0 の連続回転にする。
//
// サーボとの通信は、起動時の準備だけ公式ライブラリ(M5StackChan.Motion)に任せ、そのあとは自前のタスク(driveTask)
// だけが行う。旋回は首のサーボの生の位置を読みながら往復させるので、公式ライブラリの 20 ms ごとのタスクと
// 同じ通信線を取り合わないようにする(公式ライブラリの自動の脱力を切り、Motion の命令を呼ばなければ、通信しない)。
#include <Arduino.h>
#include <initializer_list>
#include <Avatar.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <M5StackChan.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <atomic>
#include <drivers/FTServo_Arduino/src/SCSCL.h>

#include "pages.h"

namespace {

constexpr int SPEED_MIN = 10;  // 歩く速さの下限 [%]
// 歩く速さの上限 [%](スティックを端まで倒したときの速さ)。操作ページの設定で SPEED_LIMIT_LOW〜SPEED_LIMIT_HIGH の間で選べる。
// 初期値の 80 は、実機で全開(100)まで試して決めた(2026-11-20)。シミュレーションでは、全開だと跳ねて進みが悪かった
constexpr int SPEED_LIMIT_DEFAULT = 80, SPEED_LIMIT_LOW = 20, SPEED_LIMIT_HIGH = 100;
constexpr int STICK_DEAD = 15;  // スティックの倒れ(-100〜100)がこれより小さければ、歩かない
constexpr int STICK_TURN = 40;  // スティックの左右の倒れがこれ以上なら、上下に関係なく旋回する

// 首(ヨー)とうなずき(ピッチ)のサーボ。公式ライブラリと同じ通信線(UART1)を使う
constexpr uint8_t YAW_ID = 1, PITCH_ID = 2;
constexpr int PITCH_ZERO_DEFAULT = 620;  // うなずきの原点の生の位置(公式ライブラリの初期値。機体ごとの値は NVS の servo / zero_pos_2)
constexpr int PITCH_SPEED = 400;  // うなずきの速さ [生の位置/秒]。1 = 0.3125°
constexpr uint32_t PITCH_RELEASE_MS = 1500;  // うなずきの指令からこの時間たったら脱力する(公式ライブラリは、止まると脱力する)

// 旋回(2026-11-20 に実機で確かめた往復。firmware/README.md の「試験用のファームウェア」)。
// 首を全開で往復させる。どちらに回るかは往復の中心で決まる。首のサーボは、出力が正のとき位置の値が減る。
// 生の位置 1022 と 3 の間(1 周の約 1 割)は位置が読めず、330〜415 前後などのでたらめな値が返る。端では 3 か 1022 が返る
constexpr int TURN_DUTY = 1000;
// 上から見て時計回り: 生の位置がこの 2 つを越えるたびに折り返す
constexpr int CW_LOW = 524, CW_HIGH = 812;
// 上から見て反時計回り: 往復が読めない区間をまたぐ。位置が増える向きは、読めない区間を抜けたのを見てから
// CCW_HIGH を下から越えたら折り返す。位置が減る向きは、読めない区間に入る手前(CCW_SYNC を下向きに通過)から
// CCW_DELAY_MS たったら折り返す
constexpr int CCW_HIGH = 260, CCW_SYNC = 40;
constexpr uint32_t CCW_DELAY_MS = 80;
constexpr int POS_STEP_MAX = 100;  // 続けて読んだ位置の差がこれより大きければ、読めない区間の値とみなす(全開で 1 回に約 26)
constexpr uint32_t CCW_DOWN_LIMIT_MS = 900;  // 位置が減る向きで、手前の通過を見ないままこの時間たったら折り返す
constexpr uint32_t CCW_BACK_LIMIT_MS = 1500;  // 位置が増える向きで、この時間たっても上端を越えなければ、やり直す
constexpr uint32_t CCW_SEEK_LIMIT_MS = 4000;
// 反時計回りを始めるとき、首がこの位置より上にあれば、位置が増える向きに回して読めない区間を抜けてから往復に入る
// (位置が減る向きに上端まで戻すより近い)。往復の読めない区間側の折り返しは、区間の向こう側の位置 1000 付近にあたり、
// そこまでの距離(1000 − 位置)と、上端までの距離(位置 − CCW_HIGH)が等しくなるのが 630
constexpr int CCW_UP_FROM = 630;
// 読めない区間にいるかの見分け。区間の端の近く(GAP_NEAR_LOW 以下か GAP_NEAR_HIGH 以上)から、続けて読んだ値が
// POS_STEP_MAX 以上飛んだら、区間に入ったとみなす。端の値(GAP_EDGE_LOW 以下か GAP_EDGE_HIGH 以上)を読んだら、出たとみなす
constexpr int GAP_NEAR_LOW = 30, GAP_NEAR_HIGH = 1000, GAP_EDGE_LOW = 10, GAP_EDGE_HIGH = 1015;
constexpr uint32_t COMMAND_TIMEOUT_MS = 900;  // スマホからの合図(0.3 秒ごと)がこの時間途切れたら止める
constexpr uint32_t CONNECT_TIMEOUT_MS = 20000;  // 家の Wi-Fi への接続を待つ時間
constexpr uint32_t QR_SHOW_MS = 15000;  // 操作ページの QR を出しておく時間
constexpr uint32_t FORGET_HOLD_MS = 5000;  // 頭の上をこの時間さわり続けると「つなぎ方の設定を消しますか」を出す
constexpr uint32_t CONFIRM_SHOW_MS = 10000;  // 確認の画面を出しておく時間。過ぎたら何もせず顔に戻る
constexpr int PITCH_MAX_DEG = 60;
const IPAddress SETUP_IP(192, 168, 4, 1);
const char* const MDNS_NAME = "strider";

// Choose: つなぎ方を選ぶ画面 / Setup: 家の Wi-Fi の設定待ち / Ready: 家の Wi-Fi につないで操作
// Direct: 本体が出す Wi-Fi にスマホを直接つないで操作
enum class Mode { Choose, Setup, Ready, Direct };

Preferences prefs;
WebServer server(80);
DNSServer dns;
m5avatar::Avatar avatar;

Mode mode = Mode::Choose;
String setupSsid;  // 本体が出す Wi-Fi の名前(家の Wi-Fi の設定用と、直接つなぐとき)
bool invertDirection = false;
bool smileWhileWalking = false;  // 歩いている間は笑顔にするか(設定で切り替える。初期状態は変えない)
int speedLimit = SPEED_LIMIT_DEFAULT;  // 歩く速さの上限 [%](設定で変える)
// 動き。操作ページからの指令で loop が決め、driveTask がサーボに伝える
enum class Drive { Stop, Walk, TurnRight, TurnLeft };  // TurnRight = 上から見て時計回り
std::atomic<Drive> driveMode{Drive::Stop};
std::atomic<int> walkDuty{0};  // 歩くときの首の出力(-1000〜1000。向きの入れ替えも済ませた値)
std::atomic<int> pitchRequest{-1};  // 0 以上なら、うなずきの目標(生の位置)。driveTask が送ったら -1 に戻す
std::atomic<uint32_t> turnLostCount{0};  // 反時計回りの旋回で、上端を見失ってやり直した回数(操作ページに出す)
std::atomic<int> yawPos{-1};  // 最後に読んだ首の生の位置(-1 = 読めなかった)。/api/state に出す(調べるとき用)
std::atomic<bool> yawInGap{false};  // 首が、位置の読めない区間にいるとみなしているか
int stickX = 0, stickY = 0;  // 操作ページのスティックの倒れ(-100〜100。右と上が正)
int pitchZero = PITCH_ZERO_DEFAULT;
SCSCL servoBus;
uint32_t lastCommandMs = 0;
uint32_t restartAtMs = 0;  // 0 以外なら、この時刻に再起動する
uint32_t qrUntilMs = 0;  // 0 以外なら、この時刻まで QR を表示している
int qrSecondsShown = -1;  // QR の画面にいま出している残り秒数(-1 = まだ出していない)
uint32_t confirmUntilMs = 0;  // 0 以外なら、この時刻まで「つなぎ方の設定を消しますか」を表示している
bool avatarRunning = false;

// ---------------------------------------------------------------- 歩行と旋回
// サーボと通信するのは、このタスクだけ
void driveTask(void*) {
    enum class Ccw { Start, Seek, Down, Back };
    int duty = 0;
    Drive current = Drive::Stop;
    Ccw ccw = Ccw::Start;
    uint32_t ccwSinceMs = 0, ccwSyncMs = 0, pitchReleaseAtMs = 0;
    int ccwArmed = 0;  // 0: まだ。1: 読めない区間の端の値(位置 10 以下)を見た。2: そのあと 10〜150 の値を見た(区間を抜けた)
    int prevPos = -1;  // 1 つ前に読めた位置(旋回を始めるたびに -1 に戻す)
    int trackPos = -1;  // 1 つ前に読めた位置(読めない区間の見分け用。戻さない)
    bool inGap = false;  // 首が、位置の読めない区間にいるとみなしているか
    const auto writeDuty = [&](int value) {
        duty = constrain(value, -1000, 1000);
        servoBus.WritePWM(YAW_ID, static_cast<int16_t>(duty * 1023 / 1000));
    };
    const auto enterCcw = [&](Ccw next, int nextDuty) {
        ccw = next;
        ccwSinceMs = millis();
        ccwSyncMs = 0;
        ccwArmed = 0;
        writeDuty(nextDuty);
    };
    for (;;) {
        const int pitch = pitchRequest.exchange(-1);
        if (pitch >= 0) {
            servoBus.WritePos(PITCH_ID, static_cast<uint16_t>(pitch), 0, PITCH_SPEED);
            pitchReleaseAtMs = millis() + PITCH_RELEASE_MS;
        } else if (pitchReleaseAtMs && millis() >= pitchReleaseAtMs) {
            pitchReleaseAtMs = 0;
            servoBus.EnableTorque(PITCH_ID, 0);
        }

        // 首の位置は、止まっている間も歩いている間も読み続ける(1 回に約 17 ms かかる)。
        // 旋回を始めるときに、首が読めない区間にいるかを知っておくため
        const bool got = servoBus.FeedBack(YAW_ID) >= 0;
        const int pos = got ? servoBus.ReadPos(-1) : -1;
        if (got) {
            if (pos <= GAP_EDGE_LOW || pos >= GAP_EDGE_HIGH) {
                inGap = false;
            } else if (trackPos >= 0 && (trackPos <= GAP_NEAR_LOW || trackPos >= GAP_NEAR_HIGH) && abs(pos - trackPos) >= POS_STEP_MAX) {
                inGap = true;
            }
            trackPos = pos;
        }
        yawPos = pos;
        yawInGap = inGap;
        vTaskDelay(pdMS_TO_TICKS(2));

        const Drive wanted = driveMode.load();
        const bool changed = wanted != current;
        current = wanted;
        if (current == Drive::Stop || current == Drive::Walk) {
            const int target = current == Drive::Walk ? walkDuty.load() : 0;
            if (changed || target != duty) {
                writeDuty(target);
            }
            continue;
        }

        // 旋回: 読んだ位置から、折り返すかを決める
        if (changed) {
            prevPos = -1;
            ccw = Ccw::Start;
            if (current == Drive::TurnRight) {
                // 往復の範囲より下にいれば、すぐ下の判定で、位置が増える向きに折り返す。
                // 読めない区間にいるときは、位置が減る向きのほうが範囲に近いので、区間を出るまで折り返さない
                writeDuty(TURN_DUTY);
            }
        }
        const uint32_t now = millis();
        if (current == Drive::TurnRight) {
            // 読めなかったときと、読めない区間にいる間は、向きを変えない
            if (!inGap && got && duty > 0 && pos <= CW_LOW) {
                writeDuty(-TURN_DUTY);
            } else if (got && duty < 0 && pos >= CW_HIGH) {
                writeDuty(TURN_DUTY);
            }
        } else {
            // 続けて読めた 2 つの位置が近いときだけ、通過したと判定する(読めない区間ではでたらめな値が返るため)
            const bool near = got && prevPos >= 0 && abs(pos - prevPos) < POS_STEP_MAX;
            if (got && pos <= 10 && ccwArmed == 0) {
                ccwArmed = 1;
            } else if (got && pos > 10 && pos < 150 && ccwArmed == 1) {
                ccwArmed = 2;
            }
            if (ccw == Ccw::Down && !ccwSyncMs && near && prevPos > CCW_SYNC && pos <= CCW_SYNC) {
                ccwSyncMs = now;
            }
            if (ccw == Ccw::Start && got) {
                if (inGap || pos > CCW_UP_FROM) {
                    // 読めない区間か、その手前にいる: 位置が増える向きに回し、区間を抜けたのを見てから上端で折り返す
                    enterCcw(Ccw::Back, -TURN_DUTY);
                } else if (pos > CCW_HIGH) {
                    enterCcw(Ccw::Seek, TURN_DUTY);  // 上端の少し上にいる: 上端まで下ろしてから往復を始める
                } else {
                    enterCcw(Ccw::Back, -TURN_DUTY);
                    ccwArmed = 2;  // 上端より下の読める位置から始めるので、読めない区間は通らない
                }
            } else if (ccw == Ccw::Seek && near && prevPos > CCW_HIGH && pos <= CCW_HIGH) {
                enterCcw(Ccw::Down, TURN_DUTY);
            } else if (ccw == Ccw::Down && ((ccwSyncMs && now - ccwSyncMs >= CCW_DELAY_MS) || now - ccwSinceMs >= CCW_DOWN_LIMIT_MS)) {
                enterCcw(Ccw::Back, -TURN_DUTY);
            } else if (ccw == Ccw::Back && ccwArmed == 2 && near && prevPos < CCW_HIGH && pos >= CCW_HIGH) {
                enterCcw(Ccw::Down, TURN_DUTY);
            } else if ((ccw == Ccw::Back && now - ccwSinceMs >= CCW_BACK_LIMIT_MS) || (ccw == Ccw::Seek && now - ccwSinceMs >= CCW_SEEK_LIMIT_MS)) {
                // 上端を見失った: 首を止めて、いまの位置から始め直す。回数は操作ページに出す
                turnLostCount++;
                writeDuty(0);
                ccw = Ccw::Start;
                prevPos = -1;
                continue;
            }
        }
        if (got) {
            prevPos = pos;
        }
    }
}

bool isDriving() {
    return driveMode.load() != Drive::Stop;
}

// 顔の表情。設定が ON のときだけ、動いている間を笑顔にする
void applyExpression() {
    if (!avatarRunning) {
        return;
    }
    const bool smile = smileWhileWalking && isDriving();
    avatar.setExpression(smile ? m5avatar::Expression::Happy : m5avatar::Expression::Neutral);
}

// スティックの倒れ(-100〜100。右と上が正)から、動きを決める。
// 左右に大きく倒れていれば、上下に関係なく旋回する(回りながら進むことは、この機体ではできない)
void setStick(int x, int y) {
    stickX = constrain(x, -100, 100);
    stickY = constrain(y, -100, 100);
    lastCommandMs = millis();
    if (abs(stickX) >= STICK_TURN) {
        driveMode = stickX > 0 ? Drive::TurnRight : Drive::TurnLeft;
    } else if (abs(stickY) >= STICK_DEAD) {
        const int speed = map(abs(stickY), STICK_DEAD, 100, SPEED_MIN, speedLimit);
        const int sign = (invertDirection ? -1 : 1) * (stickY > 0 ? 1 : -1);
        walkDuty = sign * speed * 10;  // 速さ % → -1000〜1000
        driveMode = Drive::Walk;
    } else {
        driveMode = Drive::Stop;
    }
    applyExpression();
}

void stopDrive() {
    setStick(0, 0);
}

// ---------------------------------------------------------------- 画面
// 顔の描画を止める。suspend() は描画タスクを画面への通信の途中で凍結しうるので使わず、
// 描画ループが 1 コマ描き終えて自分で終わる stop() を使い、終わるまで少し待つ
void stopAvatar() {
    if (avatarRunning) {
        avatar.stop();
        avatarRunning = false;
        delay(200);
    }
}

void showFace() {
    qrUntilMs = 0;
    M5.Display.fillScreen(TFT_BLACK);
    avatar.start();
    avatarRunning = true;
    applyExpression();
}

void drawCaption(const char* text, int x, int y) {
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.setTextDatum(textdatum_t::top_center);
    M5.Display.drawString(text, x, y);
}

// QR を左に大きく、右に「何をすればいいか」を文章で出す画面。lines は右側の文章(1 行ずつ)
void showQrScreen(const char* title, const String& qr, std::initializer_list<const char*> lines, const char* footer) {
    stopAvatar();
    M5.Display.fillScreen(TFT_WHITE);
    M5.Display.fillRect(0, 0, 320, 30, TFT_BLACK);
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.setTextDatum(textdatum_t::middle_left);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.drawString(title, 10, 15);
    M5.Display.qrcode(qr.c_str(), 8, 40, 164, 4);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.setTextDatum(textdatum_t::top_left);
    int y = 46;
    for (const char* line : lines) {
        M5.Display.drawString(line, 182, y);
        y += 24;
    }
    M5.Display.setTextDatum(textdatum_t::bottom_center);
    M5.Display.drawString(footer, 160, 236);
}

// 家の Wi-Fi の設定の画面の右上に出す「つなぎ方を選び直す」ボタンの位置
constexpr int RECHOOSE_LEFT = 166, RECHOOSE_RIGHT = 316, RECHOOSE_BOTTOM = 44;  // さわりやすいよう、黒い帯より少し下まで受け付ける

void drawRechooseButton() {
    M5.Display.drawRoundRect(RECHOOSE_LEFT, 3, RECHOOSE_RIGHT - RECHOOSE_LEFT, 24, 6, TFT_WHITE);
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.setTextDatum(textdatum_t::middle_center);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.drawString("つなぎ方を選び直す", (RECHOOSE_LEFT + RECHOOSE_RIGHT) / 2, 15);
}

// 家の Wi-Fi の設定の 1 歩目: スマホを設定用 Wi-Fi につないでもらう
void showSetupJoinScreen() {
    const String footer = "設定用の Wi-Fi: " + setupSsid;
    showQrScreen("家の Wi-Fi  1 / 2", "WIFI:T:nopass;S:" + setupSsid + ";;",
                 {"スマホのカメラで", "左の QR を読み、", "出てきた設定用の", "Wi-Fi につないで", "ください"}, footer.c_str());
    drawRechooseButton();
}

// 家の Wi-Fi の設定の 2 歩目: スマホがつながったら、設定ページの開き方を案内する
void showSetupPageScreen() {
    const String url = "http://" + SETUP_IP.toString() + "/";
    showQrScreen("家の Wi-Fi  2 / 2", url,
                 {"スマホに、家の", "Wi-Fi を選ぶ", "ページが開きます", "開かないときは", "左の QR を読んで", "ください"}, url.c_str());
    drawRechooseButton();
}

// 直接つなぐときの 1 歩目: スマホを本体の Wi-Fi につないでもらう
void showDirectJoinScreen() {
    const String footer = "本体の Wi-Fi: " + setupSsid;
    showQrScreen("スマホから操作する準備  1 / 2", "WIFI:T:nopass;S:" + setupSsid + ";;",
                 {"スマホのカメラで", "左の QR を読み、", "出てきた Wi-Fi に", "つないでください"}, footer.c_str());
}

// 操作ページの QR。直接つなぐときにスマホがまだつながっていなければ、先に Wi-Fi の QR を出す
void showControlQr() {
    if (mode == Mode::Direct && WiFi.softAPgetStationNum() == 0) {
        showDirectJoinScreen();
    } else if (mode == Mode::Direct) {
        const String url = "http://" + SETUP_IP.toString() + "/";
        showQrScreen("スマホから操作する準備  2 / 2", url,
                     {"つながりました。", "左の QR を読むと", "操作ページが", "開きます"}, url.c_str());
    } else {
        const String url = "http://" + WiFi.localIP().toString() + "/";
        showQrScreen("スマホから操作する", url, {"家の Wi-Fi に", "つないだスマホで", "左の QR を読むと", "操作できます"}, url.c_str());
    }
    qrUntilMs = millis() + QR_SHOW_MS;
    qrSecondsShown = -1;
}

// 操作ページの QR の画面に、顔へ戻るまでの残り秒数を出す(右の文章の下)。秒が変わったときだけ描き直す
void drawQrCountdown(uint32_t now) {
    const int seconds = static_cast<int>((qrUntilMs - now + 999) / 1000);
    if (seconds == qrSecondsShown) {
        return;
    }
    qrSecondsShown = seconds;
    char line[32];
    snprintf(line, sizeof(line), "あと %d 秒で", seconds);
    M5.Display.fillRect(180, 148, 140, 66, TFT_WHITE);
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.setTextDatum(textdatum_t::top_left);
    M5.Display.setTextColor(TFT_RED, TFT_WHITE);
    M5.Display.drawString(line, 182, 150);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.drawString("顔の画面に", 182, 172);
    M5.Display.drawString("戻ります", 182, 194);
}

void showMessage(const char* line1, const char* line2 = "") {
    stopAvatar();
    M5.Display.fillScreen(TFT_WHITE);
    drawCaption(line1, 160, 96);
    drawCaption(line2, 160, 124);
}

// 確認画面のボタンの位置(画面は 320×240)
constexpr int CONFIRM_BUTTON_TOP = 150, CONFIRM_BUTTON_BOTTOM = 220;
constexpr int CANCEL_LEFT = 16, CANCEL_RIGHT = 152, ERASE_LEFT = 168, ERASE_RIGHT = 304;

// 頭を長押ししたときの確認。画面の「消す」をタッチしたときだけ、つなぎ方の設定を消す
void showForgetConfirm() {
    stopAvatar();
    qrUntilMs = 0;
    M5.Display.fillScreen(TFT_WHITE);
    drawCaption("スマホとのつなぎ方の設定を消しますか?", 160, 36);
    drawCaption("消すと、つなぎ方を選ぶ画面に戻ります", 160, 70);
    drawCaption("下のボタンをタッチしてください", 160, 104);
    const int height = CONFIRM_BUTTON_BOTTOM - CONFIRM_BUTTON_TOP;
    M5.Display.drawRoundRect(CANCEL_LEFT, CONFIRM_BUTTON_TOP, CANCEL_RIGHT - CANCEL_LEFT, height, 10, TFT_BLACK);
    M5.Display.fillRoundRect(ERASE_LEFT, CONFIRM_BUTTON_TOP, ERASE_RIGHT - ERASE_LEFT, height, 10, TFT_RED);
    M5.Display.setFont(&fonts::lgfxJapanGothic_24);
    M5.Display.setTextDatum(textdatum_t::middle_center);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.drawString("やめる", (CANCEL_LEFT + CANCEL_RIGHT) / 2, (CONFIRM_BUTTON_TOP + CONFIRM_BUTTON_BOTTOM) / 2);
    M5.Display.setTextColor(TFT_WHITE, TFT_RED);
    M5.Display.drawString("消す", (ERASE_LEFT + ERASE_RIGHT) / 2, (CONFIRM_BUTTON_TOP + CONFIRM_BUTTON_BOTTOM) / 2);
    confirmUntilMs = millis() + CONFIRM_SHOW_MS;
}

// つなぎ方を選ぶ画面のボタンの位置
constexpr int CHOICE_LEFT = 12, CHOICE_RIGHT = 308;
constexpr int DIRECT_TOP = 62, DIRECT_BOTTOM = 136, HOME_TOP = 146, HOME_BOTTOM = 220;

void drawChoiceButton(int top, int bottom, uint16_t fill, uint16_t ink, const char* name, const char* note) {
    M5.Display.fillRoundRect(CHOICE_LEFT, top, CHOICE_RIGHT - CHOICE_LEFT, bottom - top, 10, fill);
    M5.Display.drawRoundRect(CHOICE_LEFT, top, CHOICE_RIGHT - CHOICE_LEFT, bottom - top, 10, ink == TFT_WHITE ? fill : ink);
    M5.Display.setTextDatum(textdatum_t::middle_center);
    M5.Display.setTextColor(ink, fill);
    M5.Display.setFont(&fonts::lgfxJapanGothic_24);
    M5.Display.drawString(name, 160, top + 24);
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.drawString(note, 160, top + 54);
}

// はじめて起動したとき(と、つなぎ方の設定を消したあと)に出す、つなぎ方を選ぶ画面
void showChooseScreen() {
    stopAvatar();
    M5.Display.fillScreen(TFT_WHITE);
    M5.Display.fillRect(0, 0, 320, 30, TFT_BLACK);
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.setTextDatum(textdatum_t::middle_left);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.drawString("スマホとのつなぎ方を選ぶ", 10, 15);
    drawCaption("スマホから操作します。どちらかをタッチ", 160, 38);
    drawChoiceButton(DIRECT_TOP, DIRECT_BOTTOM, TFT_RED, TFT_WHITE, "直接つなぐ", "設定なしで、すぐ使えます");
    drawChoiceButton(HOME_TOP, HOME_BOTTOM, TFT_WHITE, TFT_BLACK, "家の Wi-Fi を使う", "Wi-Fi のパスワードが要ります");
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.setTextDatum(textdatum_t::bottom_center);
    M5.Display.drawString("あとから選び直せます", 160, 239);
}

// ---------------------------------------------------------------- ページと API
void sendJsonState() {
    // drive: 0 停止 / 1 前進 / -1 後退 / 2 右に旋回 / -2 左に旋回(前進・後退は、向きの入れ替えの前の、スティックの向き)
    const Drive drive = driveMode.load();
    const int code = drive == Drive::TurnRight ? 2 : drive == Drive::TurnLeft ? -2 : drive == Drive::Walk ? (stickY > 0 ? 1 : -1) : 0;
    char json[260];
    snprintf(json, sizeof(json), "{\"drive\":%d,\"invert\":%d,\"smile\":%d,\"limit\":%d,\"volt\":%.2f,\"lost\":%u,\"pos\":%d,\"gap\":%d}", code,
             invertDirection ? 1 : 0, smileWhileWalking ? 1 : 0, speedLimit, M5StackChan.getBatteryVoltage(),
             static_cast<unsigned>(turnLostCount.load()), yawPos.load(), yawInGap.load() ? 1 : 0);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

// スティックの倒れ。触れている間は、動かさなくても 0.3 秒ごとに来る(途切れたら止めるための合図を兼ねる)
void handleStick() {
    setStick(server.arg("x").toInt(), server.arg("y").toInt());
    sendJsonState();
}

void handleStop() {
    stopDrive();
    sendJsonState();
}

void handleLook() {
    const int degrees = constrain(server.arg("pitch").toInt(), 0, PITCH_MAX_DEG);
    pitchRequest = constrain(pitchZero + degrees * 16 / 5, 0, 1000);  // 生の位置の 1 = 0.3125°(公式ライブラリと同じ換算)
    sendJsonState();
}

void handleInvert() {
    invertDirection = server.arg("on").toInt() != 0;
    prefs.putBool("invert", invertDirection);
    setStick(stickX, stickY);
    sendJsonState();
}

void handleLimit() {
    speedLimit = constrain(server.arg("v").toInt(), SPEED_LIMIT_LOW, SPEED_LIMIT_HIGH);
    prefs.putInt("limit", speedLimit);
    setStick(stickX, stickY);
    sendJsonState();
}

void handleSmile() {
    smileWhileWalking = server.arg("on").toInt() != 0;
    prefs.putBool("smile", smileWhileWalking);
    applyExpression();
    sendJsonState();
}

// つなぎ方の設定(家の Wi-Fi、直接つなぐ)を消す。次の起動で、つなぎ方を選ぶ画面に戻る
void forgetConnection() {
    prefs.remove("ssid");
    prefs.remove("pass");
    prefs.remove("direct");
}

void handleForget() {
    stopDrive();
    forgetConnection();
    server.send(200, "application/json", "{\"ok\":1}");
    restartAtMs = millis() + 800;
}

void handleScan() {
    const int count = WiFi.scanNetworks();
    String json = "[";
    for (int i = 0; i < count; i++) {
        String name = WiFi.SSID(i);
        if (name.isEmpty() || json.indexOf("\"" + name + "\"") >= 0) {
            continue;  // 名前のないものと重複は出さない
        }
        name.replace("\\", "\\\\");
        name.replace("\"", "\\\"");
        if (json.length() > 1) {
            json += ",";
        }
        json += "{\"ssid\":\"" + name + "\",\"lock\":" + (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "0" : "1") + "}";
    }
    json += "]";
    WiFi.scanDelete();
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

void handleSaveWifi() {
    const String ssid = server.arg("ssid");
    if (ssid.isEmpty()) {
        server.send(400, "application/json", "{\"error\":\"ssid\"}");
        return;
    }
    prefs.putString("ssid", ssid);
    prefs.putString("pass", server.arg("pass"));
    prefs.remove("direct");
    server.send(200, "application/json", "{\"ok\":1}");
    restartAtMs = millis() + 1200;
}

// スマホが「この Wi-Fi はログインが必要か」を確かめに来る通信を、本体のページへ案内する
void redirectToPage() {
    server.sendHeader("Location", "http://" + SETUP_IP.toString() + "/");
    server.send(302, "text/plain", "");
}

void startServer() {
    if (mode != Mode::Setup) {
        server.on("/", []() {
            if (qrUntilMs) {
                qrUntilMs = millis();  // 操作ページが開かれたら QR をしまって顔に戻す
            }
            server.send_P(200, "text/html; charset=utf-8", CONTROL_PAGE);
        });
        server.on("/api/stick", handleStick);
        server.on("/api/stop", handleStop);
        server.on("/api/state", sendJsonState);
        server.on("/api/look", handleLook);
        server.on("/api/invert", handleInvert);
        server.on("/api/smile", handleSmile);
        server.on("/api/limit", handleLimit);
        server.on("/api/forget", handleForget);
        if (mode == Mode::Direct) {
            server.onNotFound(redirectToPage);
        } else {
            server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
        }
    } else {
        server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", SETUP_PAGE); });
        server.on("/api/scan", handleScan);
        server.on("/api/wifi", HTTP_POST, handleSaveWifi);
        server.onNotFound(redirectToPage);
    }
    server.begin();
}

// ---------------------------------------------------------------- Wi-Fi
// 本体が Wi-Fi を出す(パスワードなし)。どの名前を引いても本体の IP を返し、スマホにページを開かせる
void startAccessPoint(wifi_mode_t wifiMode) {
    const uint64_t mac = ESP.getEfuseMac();
    char name[24];
    snprintf(name, sizeof(name), "Strider-%04X", static_cast<unsigned>((mac >> 32) & 0xFFFF));
    setupSsid = name;
    WiFi.mode(wifiMode);
    WiFi.softAPConfig(SETUP_IP, SETUP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(setupSsid.c_str());
    dns.start(53, "*", SETUP_IP);
}

void startSetupMode() {
    mode = Mode::Setup;
    startAccessPoint(WIFI_AP_STA);  // STA は周囲の Wi-Fi を探すために残す
    startServer();
    showSetupJoinScreen();
}

void startDirectMode() {
    mode = Mode::Direct;
    startAccessPoint(WIFI_AP);
    startServer();
    showDirectJoinScreen();  // スマホがつながるまで出したままにする(時間では消さない)
}

bool connectHomeWifi(const String& ssid, const String& pass) {
    showMessage("家の Wi-Fi につないでいます", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MDNS_NAME);
    WiFi.begin(ssid.c_str(), pass.c_str());
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < CONNECT_TIMEOUT_MS) {
        delay(200);
    }
    return WiFi.status() == WL_CONNECTED;
}

void startReadyMode() {
    mode = Mode::Ready;
    MDNS.begin(MDNS_NAME);
    MDNS.addService("http", "tcp", 80);
    startServer();
    showControlQr();
}

}  // namespace

void setup() {
    M5StackChan.begin();
    M5.Display.setBrightness(120);
    Serial.begin(115200);
    Serial.printf("[strider] board=%d rotation=%d width=%d height=%d\n", static_cast<int>(M5.getBoard()),
                  M5.Display.getRotation(), M5.Display.width(), M5.Display.height());
    // サーボ: 公式ライブラリに首(ヨー)を連続回転モードにさせて止め、自動の脱力(0.2 秒ごとにサーボと通信する)を切る。
    // このあと公式ライブラリの Motion の命令は呼ばない。サーボとの通信は driveTask だけが行う
    M5StackChan.Motion.setAutoTorqueReleaseEnabled(false);
    M5StackChan.Motion.rotateX(0);
    servoBus.uart_num = UART_NUM_1;  // 通信線の準備は公式ライブラリが済ませている
    {
        Preferences servoPrefs;
        if (servoPrefs.begin("servo", true)) {
            pitchZero = servoPrefs.getInt("zero_pos_2", PITCH_ZERO_DEFAULT);
            servoPrefs.end();
        }
    }
    xTaskCreate(driveTask, "drive", 4096, nullptr, 5, nullptr);
    prefs.begin("strider", false);
    invertDirection = prefs.getBool("invert", false);
    smileWhileWalking = prefs.getBool("smile", false);
    speedLimit = constrain(prefs.getInt("limit", SPEED_LIMIT_DEFAULT), SPEED_LIMIT_LOW, SPEED_LIMIT_HIGH);
    const String ssid = prefs.getString("ssid", "");
    if (prefs.getBool("direct", false)) {
        startDirectMode();
    } else if (ssid.isEmpty()) {
        showChooseScreen();  // mode は Choose のまま。Wi-Fi はまだ使わない
    } else if (connectHomeWifi(ssid, prefs.getString("pass", ""))) {
        startReadyMode();
    } else {
        showMessage("家の Wi-Fi につながりません", "家の Wi-Fi の設定の画面に戻ります");
        delay(2500);
        startSetupMode();
    }
}

void loop() {
    M5StackChan.update();
    server.handleClient();
    const uint32_t now = millis();

    if (mode == Mode::Choose) {
        // 画面のボタンをタッチして、つなぎ方を選ぶ
        const auto touch = M5.Touch.getDetail();
        if (touch.wasPressed() && touch.x >= CHOICE_LEFT && touch.x <= CHOICE_RIGHT) {
            if (touch.y >= DIRECT_TOP && touch.y <= DIRECT_BOTTOM) {
                prefs.putBool("direct", true);
                startDirectMode();
            } else if (touch.y >= HOME_TOP && touch.y <= HOME_BOTTOM) {
                startSetupMode();
            }
        }
    } else if (mode == Mode::Setup) {
        dns.processNextRequest();
        // 画面の右上の「つなぎ方を選び直す」: 保存してある設定を消して、選ぶ画面からやり直す
        const auto touch = M5.Touch.getDetail();
        if (touch.wasPressed() && touch.y <= RECHOOSE_BOTTOM && touch.x >= RECHOOSE_LEFT && touch.x <= RECHOOSE_RIGHT && !restartAtMs) {
            forgetConnection();
            showMessage("つなぎ方を選ぶ画面に戻ります", "再起動します");
            restartAtMs = now + 1200;
        }
        // スマホが設定用 Wi-Fi につながったら 2 歩目の画面へ、切れたら 1 歩目へ戻す
        static bool phoneJoined = false;
        const bool joined = WiFi.softAPgetStationNum() > 0;
        if (joined != phoneJoined && !restartAtMs) {
            phoneJoined = joined;
            if (joined) {
                showSetupPageScreen();
            } else {
                showSetupJoinScreen();
            }
        }
    } else {
        if (mode == Mode::Direct) {
            dns.processNextRequest();
            // スマホが本体の Wi-Fi につながったら、操作ページの QR を出す
            static bool phoneJoined = false;
            const bool joined = WiFi.softAPgetStationNum() > 0;
            if (joined != phoneJoined) {
                phoneJoined = joined;
                if (joined && !confirmUntilMs && !restartAtMs) {
                    showControlQr();
                }
            }
        }
        // スマホからの合図が途切れたら止める(電波が切れた、ページを閉じた、など)
        if (isDriving() && now - lastCommandMs > COMMAND_TIMEOUT_MS) {
            stopDrive();
        }
        if (qrUntilMs && now >= qrUntilMs) {
            showFace();
        } else if (qrUntilMs) {
            drawQrCountdown(now);
        }
        // 頭の上をさわって離すと操作ページの QR を出す。さわり続けると「つなぎ方の設定を消しますか」を出す
        static bool holdHandled = false;  // 長押しで確認を出したあと、指を離すまで次の操作を受け付けない
        if (confirmUntilMs) {
            const auto touch = M5.Touch.getDetail();
            const bool inButtonRow = touch.y >= CONFIRM_BUTTON_TOP && touch.y <= CONFIRM_BUTTON_BOTTOM;
            if (touch.wasPressed() && inButtonRow && touch.x >= ERASE_LEFT && touch.x <= ERASE_RIGHT && !restartAtMs) {
                confirmUntilMs = 0;
                forgetConnection();
                showMessage("つなぎ方の設定を消しました", "再起動します");
                restartAtMs = now + 1500;
            } else if ((touch.wasPressed() && inButtonRow && touch.x >= CANCEL_LEFT && touch.x <= CANCEL_RIGHT) || now >= confirmUntilMs) {
                confirmUntilMs = 0;
                showFace();
            }
        } else if (!restartAtMs) {
            if (M5StackChan.TouchSensor.pressedFor(FORGET_HOLD_MS) && !holdHandled) {
                holdHandled = true;
                stopDrive();
                showForgetConfirm();
            } else if (M5StackChan.TouchSensor.wasReleased() && !holdHandled && !qrUntilMs) {
                showControlQr();
            }
        }
        if (holdHandled && !M5StackChan.TouchSensor.isPressed()) {
            holdHandled = false;
        }
    }

    if (restartAtMs && now >= restartAtMs) {
        stopDrive();
        delay(100);
        ESP.restart();
    }
    delay(5);
}
