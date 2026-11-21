// 試験用: 首(ヨー)サーボを連続回転(PWM)させながら、サーボの生の位置が読めるかと、往復で機体が回るかを確かめる。
// 歩行用のファームウェアとは別の環境(pio run -e yawprobe)でビルドする。USB シリアルから 1 行ずつ指令を送る。
//
//   p <出力 -1000〜1000> <ミリ秒>             その出力で回し、時間が来たら止める
//   r <下限> <上限> <出力 0〜1000> <ミリ秒>   生の位置が下限・上限を越えるたびに向きを変えて往復する
//   h <上端> <出力 0〜1000> <片道ミリ秒> <ミリ秒>
//                                             読めない区間(位置 1022 と 3 の間)をまたぐ往復。位置が増える向きは、
//                                             上端を下から越えたら折り返す。位置が減る向きは、読めない区間に入るので、
//                                             上端で折り返してから <片道ミリ秒> たったら折り返す。
//                                             始めに首が上端より上にあれば、出力 30% で上端まで戻してから始める。
//                                             戻りで 1.5 秒たっても上端を越えなければ、首を止める(出力 0 のまま記録は続く)
//   w <上端> <下端> <出力 0〜1000> <ミリ秒>   読めない区間をまたぐ往復を、両端とも位置で折り返す。<下端> は読めない区間の
//                                             向こう側の位置(900〜1022)。位置が減る向きは、読めない区間を抜けて位置が
//                                             <下端> 以下になったら折り返す(0.9 秒たっても読めなければ、そこで折り返す)。
//                                             位置が増える向きは、読めない区間を抜けた印(位置 10 以下)を見たあとで
//                                             <上端> を下から越えたら折り返す。h と同じく、始めの位置合わせと 1.5 秒の打ち切りがある
//   v <上端> <遅れミリ秒> <出力 0〜1000> <ミリ秒>
//                                             読めない区間をまたぐ往復。位置が減る向きは、読めない区間に入る手前(位置 40 を
//                                             下向きに通過)から <遅れミリ秒> たったら折り返す(0.9 秒たっても通過を
//                                             見なければ、そこで折り返す)。位置が増える向きは w と同じ
//   e <中心> <半幅> <出力 0〜1000> <ミリ秒>   往復の中心がどこにあっても同じ手順で折り返す往復(どの機体でも使える往復の試験)。
//                                             <中心> は 1 周を 0〜1151 で表した位置(生の位置と同じ目盛り。1023〜1151 は
//                                             読めない区間の中)。首の位置の見積もりが 中心 ± 半幅 を越えたら折り返す。
//                                             見積もりは、読めた位置が直前の見積もりとつじつまが合えばその値、合わなければ
//                                             (読めない区間)、出力から決めた速さで進めた値。区間を抜けた直後の値で合わせ直す。
//                                             この往復の記録は、速度の欄に見積もり(0〜1151)、負荷の欄に読めた値を使ったか(1 / 0)が入る
//                                             <中心> に -1 を書くと、直前の k で求めた時計回りの中心、-2 で反時計回りの中心を使う
//   k <出力> <ミリ秒>                         脚の位相を調べる: その出力で歩かせ、機体の揺れ(ジャイロ)を首の位置ごとに調べて、
//                                             旋回の往復の中心を求める。結果は本体に保存し、j で出す
//   j                                         k の結果を出す(新しい順ではなく、求めた順)
//   x                                         止める
//   d                                         直前に回したときの記録を出す
//   i                                         起動の理由、起動からの時間、Wi-Fi の IP アドレス、加速度を出す
//   q <p か r の指令>                         予約に足す(16 個まで)。予約は、画面をタッチするか g で順に実行する
//                                             予約は本体に保存するので、再起動しても残る
//   c                                         予約を消す
//   z                                         記録を消す(記録が残っている間は、g では始まらない)
//
// 画面のタッチ:
//   実行中にタッチ                            止める(その回の記録は捨てる。終わった回の記録は残る)
//   予約が最後まで終わっていないときにタッチ  記録を消して、1 個目からやり直す(3 秒後に始まる)
//   最後まで終わったあとに 2 秒長押し         記録を消して、1 個目からやり直す
//   g                                         予約の実行を始める
//
// ケーブルを外して床で試すときは、q で予約してからケーブルを外し、画面をタッチする(3 秒後に始まる)。
// 終わったらケーブルをつなぎ直して d で記録を出す。記録の「電流」の欄には、何番目の予約かが入る。
// 予約で実行した記録は、1 個終わるたびにフラッシュに書くので、再起動しても電源が切れても残る。
// 書く場所は、フラッシュ(16 MB)の最後の 128 KB(0xfe0000 から)。領域配置のどの領域にも入っていない所で、
// 出荷時は空(2026-11-17 に読み出した内容で確認)。動いているアプリ領域(ota_0)の中は、ESP-IDF が書き込みを禁じている。
//
// 歩行用のファームウェアで家の Wi-Fi を設定してあれば、起動時にそこへつなぎ、同じ指令を HTTP でも受ける
// (ケーブルを外して床で試すため)。 /cmd?c=<指令>  /dump
// p と r は、指令を受けてから 1 秒は首を回さずに記録だけする(ジャイロのずれを後で引くため)。
//
// 出力する行(止まっている間は 0.2 秒ごと。回している間は出さずに本体の中に記録し、d で出す):
//   D,<マイクロ秒>,<出力>,<位置>,<速度>,<負荷>,<電流>,<電圧>,<温度>,<読み取りにかかったマイクロ秒>,<ジャイロを積算した角度 x>,<y>,<z>(0.001°)
//   位置などが -1 のときは、サーボから返事が無かった
// 回している間に出さないのは、首を回すとパソコンとの USB 通信が切れたため(2026-11-20 の実機で 2 回)。
//
// 公式ライブラリの M5StackChan.begin() は呼ばない(呼ぶと 20 ms ごとのタスクが同じ通信線を使う)。
#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_flash.h>
#include <cmath>
#include <mutex>
#include <drivers/FTServo_Arduino/src/SCSCL.h>
#include <drivers/PY32IOExpander/PY32IOExpander.hpp>

namespace {

constexpr uint8_t YAW_ID = 1;
constexpr uint32_t RUN_LIMIT_MS = 60000;
constexpr uint32_t PREROLL_MS = 1000;  // 指令を受けてから首を回し始めるまで(止まったままジャイロを記録する時間)

SCSCL bus;
m5::PY32IOExpander_Class expander;
WebServer server(80);

int duty = 0;  // いまの出力(-1000〜1000)
uint32_t runUntilMs = 0;  // 0 以外なら、この時刻に止める
uint32_t startAtMs = 0;  // 0 以外なら、この時刻に首を回し始める
int startDuty = 0;
bool rocking = false;
int rockLow = 0, rockHigh = 0, rockDuty = 0;
// h の往復の状態
enum class Hybrid { off, start, seek, timed, back, lost };
Hybrid hybrid = Hybrid::off;
uint32_t hybridTimedMs = 0;  // 上端で折り返してから、次に折り返すまで
int hybridLow = -1;  // 0 以上なら w の往復(下端も位置で折り返す)。読めない区間の向こう側の位置
// 戻りで、読めない区間を抜けたか。0: まだ。1: 区間の端の値(位置 10 以下)を見た。2: そのあと 10〜150 の値を見た(抜けた)
// 区間の中では 330〜415 前後のでたらめな値が返るので、1 だけでは、区間へ入っていく途中と区別できない
int hybridArmed = 0;
int hybridDelayMs = -1;  // 0 以上なら v の往復
uint32_t hybridSyncMs = 0;  // v の往復で、位置 40 を下向きに通過した時刻(0 はまだ)
constexpr int HYBRID_SYNC_POS = 40;
constexpr int HYBRID_FAR_MIN = 900;  // これ以上の位置は、読めない区間の向こう側とみなす
constexpr uint32_t HYBRID_DOWN_LIMIT_MS = 900;
uint32_t hybridSinceMs = 0;  // いまの状態に入った時刻
int hybridPrevPos = -1;  // 1 つ前に読めた位置
constexpr int HYBRID_STEP = 100;  // 続けて読んだ位置の差がこれより大きければ、読めない区間の値とみなす(全開で 1 回に約 26)
constexpr uint32_t HYBRID_BACK_LIMIT_MS = 1500;
// e の往復: 首の位置の見積もり
constexpr double CIRCLE = 1152.0;  // 1 周(1 = 0.3125°)
constexpr double EST_VMAX = 1400.0;  // 全開で回り続けたときの速さの仮定 [位置/秒]
constexpr double EST_TAU = 0.07;  // 出力を変えてから速さが変わるまでの時定数の仮定 [秒]
constexpr double EST_TOL = 120.0;  // 読めた位置と見積もりの差がこれより小さければ、読めた位置を使う
bool estRock = false;
bool estInit = false, estPlaced = false, estUsed = false;
double est = 0, estVel = 0;  // 見積もり(巻き戻さない連続の値)と速さ
// k: 歩行中の揺れから、脚の位相(旋回の往復の中心)を求める。
// 揺れの速さ [度/秒] を、首の位置 θ(1 周 = 2π)の関数 a0 + a1·cosθ + b1·sinθ + a2·cos2θ + b2·sin2θ に最小二乗で当てはめる。
// 前後の傾き(IMU の x 軸まわり)は 1 周に 2 回の成分が強く、山の位置が速さによらず安定している(半周ずれた 2 つの候補が残る)。
// 鉛直まわり(y 軸)は 1 周に 1 回の成分が強いが、山の位置が速さで動くので、2 つの候補のどちらかを選ぶのにだけ使う。
// 基準は、旋回の中心が分かっている機体(2026-11-20)を出力 60% で歩かせたときの値
constexpr double REF_PITCH2 = 418.0;  // x 軸の 2 回成分の山の位置(0〜575)
constexpr double REF_YAW1 = 246.0;  // y 軸の 1 回成分の山の位置(0〜1151)
constexpr double REF_CW_CENTER = 668.0;  // 上から見て時計回りに回る往復の中心
constexpr uint32_t CALIB_SKIP_MS = 600;  // 歩き始めてから、この時間は使わない
bool calibActive = false;
uint32_t walkStartedMs = 0;  // 首を回し始めた時刻
double calibAtA[5][5], calibAtB[2][5];  // 正規方程式(軸 0 = x、1 = y)
int calibCount = 0;
int64_t calibPrevUs = 0;
int32_t calibPrevAngle[2] = {0, 0};
int calibPrevPos = -1;
int calibDuty = 0;
int calibCw = -1;  // 求めた時計回りの中心(-1 = まだ)
constexpr int CALIB_KEEP = 8;
int calibStored = 0;

void calibReset() {
    memset(calibAtA, 0, sizeof(calibAtA));
    memset(calibAtB, 0, sizeof(calibAtB));
    calibCount = 0;
    calibPrevUs = 0;
    calibPrevPos = -1;
}

// 5 元の連立方程式を解く(部分ピボットつきの消去法)。解けなければ false
bool solve5(double a[5][5], double b[5], double x[5]) {
    for (int col = 0; col < 5; col++) {
        int pivot = col;
        for (int row = col + 1; row < 5; row++) {
            if (fabs(a[row][col]) > fabs(a[pivot][col])) {
                pivot = row;
            }
        }
        if (fabs(a[pivot][col]) < 1e-9) {
            return false;
        }
        for (int k = 0; k < 5; k++) {
            std::swap(a[col][k], a[pivot][k]);
        }
        std::swap(b[col], b[pivot]);
        for (int row = col + 1; row < 5; row++) {
            const double f = a[row][col] / a[col][col];
            for (int k = col; k < 5; k++) {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    for (int row = 4; row >= 0; row--) {
        double sum = b[row];
        for (int k = row + 1; k < 5; k++) {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    return true;
}

double wrapTo(double value, double period) {
    return value - period * floor(value / period);
}

// k の歩行が終わったとき: 当てはめて、旋回の中心を求め、本体に保存する
void calibFinish() {
    char text[160];
    double coef[2][5];
    bool solved = calibCount >= 100;
    for (int axis = 0; axis < 2 && solved; axis++) {
        double a[5][5], b[5];
        memcpy(a, calibAtA, sizeof(a));
        memcpy(b, calibAtB[axis], sizeof(b));
        solved = solve5(a, b, coef[axis]);
    }
    if (!solved) {
        snprintf(text, sizeof(text), "duty=%d n=%d failed", calibDuty, calibCount);
    } else {
        const double pitch2 = wrapTo(atan2(coef[0][4], coef[0][3]) / (2 * M_PI) * 1152.0 / 2, 576.0);
        const double pitch2Amp = hypot(coef[0][3], coef[0][4]);
        const double yaw1 = wrapTo(atan2(coef[1][2], coef[1][1]) / (2 * M_PI) * 1152.0, 1152.0);
        const double yaw1Amp = hypot(coef[1][1], coef[1][2]);
        // 基準からのずれ。半周ずれた 2 つの候補から、鉛直まわりの山の位置が基準に近いほうを選ぶ
        const double shift = wrapTo(pitch2 - REF_PITCH2, 576.0);
        const auto miss = [&](double candidate) { return fabs(wrapTo(yaw1 - (REF_YAW1 + candidate) + 576.0, 1152.0) - 576.0); };
        const double chosen = miss(shift) <= miss(shift + 576.0) ? shift : shift + 576.0;
        calibCw = static_cast<int>(lround(wrapTo(REF_CW_CENTER + chosen, 1152.0))) % 1152;
        snprintf(text, sizeof(text), "duty=%d n=%d pitch2=%.0f amp=%.1f yaw1=%.0f amp=%.1f miss=%.0f/%.0f cw=%d", calibDuty, calibCount,
                 pitch2, pitch2Amp, yaw1, yaw1Amp, miss(shift), miss(shift + 576.0), calibCw);
    }
    Preferences store;
    store.begin("yawprobe", false);
    store.putString(("k" + String(calibStored % CALIB_KEEP)).c_str(), text);
    calibStored++;
    store.putInt("kn", calibStored);
    store.putInt("kcw", calibCw);
    store.end();
}

int estMiss = 0;  // 読める場所にいるはずなのに、読めた位置が見積もりと食い違った回数(続けて)
constexpr int EST_MISS_MAX = 10;  // これだけ続けて食い違ったら、見積もりのほうが違うとみなして、読めた位置に合わせ直す
int64_t estUs = 0;
int estPrevRaw = -1;
int estCenter = 0, estHalf = 0;
uint32_t lastIdleLogMs = 0, lastScreenMs = 0;
size_t sampleCount = 0;  // 記録した数

constexpr size_t QUEUE_MAX = 16;
String queued[QUEUE_MAX];
size_t queueCount = 0, queueIndex = 0;
bool queueActive = false;
uint32_t nextQueueAtMs = 0;
int runIndex = 0;  // 予約の何番目を実行しているか(1 から。予約でない実行は 0)
std::mutex i2cMutex;  // ジャイロ(別タスク)と画面のタッチが、同じ I2C を使う
uint32_t lastTouchMs = 0;
uint32_t holdStartMs = 0;  // 0 以外なら、この時刻から画面に触れ続けている
constexpr uint32_t REDO_HOLD_MS = 2000;

// 予約を本体に保存する(再起動しても残すため)。歩行用のファームウェアとは別の名前空間を使う
void saveQueue() {
    Preferences store;
    store.begin("yawprobe", false);
    store.putUInt("count", queueCount);
    for (size_t i = 0; i < queueCount; i++) {
        store.putString(("q" + String(i)).c_str(), queued[i]);
    }
    store.end();
}

void loadQueue() {
    Preferences store;
    store.begin("yawprobe", true);
    queueCount = min<size_t>(store.getUInt("count", 0), QUEUE_MAX);
    for (size_t i = 0; i < queueCount; i++) {
        queued[i] = store.getString(("q" + String(i)).c_str(), "");
    }
    store.end();
}

struct Sample {
    uint32_t us;
    int16_t duty, pos, speed, load, current;
    int8_t volt, temp;
    uint16_t readUs;
    int32_t angle[3];  // ジャイロを積算した角度 [0.001°]
};
constexpr size_t SAMPLE_MAX = 4096;
Sample samples[SAMPLE_MAX];

// 記録をフラッシュに残す。場所は、フラッシュの最後(どの領域にも入っていない所)
constexpr size_t STORE_SIZE = SAMPLE_MAX * sizeof(Sample);
static_assert(STORE_SIZE % 4096 == 0, "記録の領域はフラッシュの消去の単位(4096)の倍数にする");
constexpr uint32_t FLASH_SIZE = 0x1000000;  // 16 MB
constexpr uint32_t STORE_ADDRESS = FLASH_SIZE - STORE_SIZE;
constexpr uint32_t PARTITIONS_END = 0xe10000;  // partitions_stackchan.csv の最後の領域(coredump)の終わり
static_assert(STORE_ADDRESS >= PARTITIONS_END, "記録の領域が、領域配置の中に入っている");
size_t storedCount = 0;  // フラッシュに書いてある数
String storeError;  // 空でなければ、フラッシュの読み書きに失敗している

bool isErased(const Sample& s) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&s);
    for (size_t i = 0; i < sizeof(Sample); i++) {
        if (bytes[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

void storeFail(const char* what, esp_err_t err) {
    storeError = String(what) + " " + esp_err_to_name(err);
}

// 起動時: フラッシュに残っている記録を読み込む
void storeLoad() {
    uint32_t flashSize = 0;
    esp_err_t err = esp_flash_get_size(nullptr, &flashSize);
    if (err == ESP_OK && flashSize != FLASH_SIZE) {
        storeError = "flash size " + String(flashSize);
        return;
    }
    if (err == ESP_OK) {
        err = esp_flash_read(nullptr, samples, STORE_ADDRESS, STORE_SIZE);
    }
    if (err != ESP_OK) {
        storeFail("read", err);
        return;
    }
    size_t count = 0;
    while (count < SAMPLE_MAX && !isErased(samples[count])) {
        count++;
    }
    sampleCount = storedCount = count;
}

void storeErase() {
    if (!storeError.isEmpty()) {
        return;
    }
    const esp_err_t err = esp_flash_erase_region(nullptr, STORE_ADDRESS, STORE_SIZE);
    if (err != ESP_OK) {
        storeFail("erase", err);
    }
    sampleCount = storedCount = 0;
}

// まだ書いていない記録をフラッシュに足す
void storeAppend() {
    if (sampleCount <= storedCount || !storeError.isEmpty()) {
        return;
    }
    const esp_err_t err = esp_flash_write(nullptr, &samples[storedCount], STORE_ADDRESS + storedCount * sizeof(Sample),
                                          (sampleCount - storedCount) * sizeof(Sample));
    if (err != ESP_OK) {
        storeFail("write", err);
        return;
    }
    storedCount = sampleCount;
}

// 予約が最後まで終わって、その記録が残っているか(記録は 1 個終わるたびに足すので、最後の記録の番号で分かる)
bool queueFinished() {
    return sampleCount > 0 && samples[sampleCount - 1].current >= static_cast<int>(queueCount);
}

void startQueue(uint32_t now) {
    storeErase();
    queueActive = true;
    queueIndex = 0;
    nextQueueAtMs = millis() + 3000;
}

// ジャイロの積算(別タスクで 5 ms ごと)。首のサーボの読み取りは 1 回 17 ms かかるので、同じループでは粗すぎる
portMUX_TYPE gyroMux = portMUX_INITIALIZER_UNLOCKED;
double gyroAngle[3] = {0, 0, 0};  // [°]
float lastAccel[3] = {0, 0, 0};  // [g]

void gyroTask(void*) {
    int64_t last = esp_timer_get_time();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5));
        m5::imu_data_t data;
        {
            std::lock_guard<std::mutex> lock(i2cMutex);
            if (!M5.Imu.update()) {
                continue;
            }
            data = M5.Imu.getImuData();
        }
        const int64_t now = esp_timer_get_time();
        const double dt = (now - last) / 1e6;
        last = now;
        portENTER_CRITICAL(&gyroMux);
        gyroAngle[0] += data.gyro.x * dt;
        gyroAngle[1] += data.gyro.y * dt;
        gyroAngle[2] += data.gyro.z * dt;
        lastAccel[0] = data.accel.x;
        lastAccel[1] = data.accel.y;
        lastAccel[2] = data.accel.z;
        portEXIT_CRITICAL(&gyroMux);
    }
}

int formatSample(char* out, size_t size, const Sample& s) {
    return snprintf(out, size, "D,%u,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d\n", static_cast<unsigned>(s.us), s.duty, s.pos,
                    s.speed, s.load, s.current, s.volt, s.temp, s.readUs, static_cast<int>(s.angle[0]),
                    static_cast<int>(s.angle[1]), static_cast<int>(s.angle[2]));
}

void printSample(const Sample& s) {
    char text[120];
    formatSample(text, sizeof(text), s);
    Serial.print(text);
}

void writeDuty(int value) {
    duty = constrain(value, -1000, 1000);
    bus.WritePWM(YAW_ID, static_cast<int16_t>(duty * 1023 / 1000));
}

void stopAll() {
    rocking = false;
    estRock = false;
    calibActive = false;
    hybrid = Hybrid::off;
    runUntilMs = 0;
    startAtMs = 0;
    writeDuty(0);
}

String infoLine() {
    float accel[3];
    portENTER_CRITICAL(&gyroMux);
    memcpy(accel, lastAccel, sizeof(accel));
    portEXIT_CRITICAL(&gyroMux);
    char text[280];
    snprintf(text, sizeof(text), "# info reset_reason=%d uptime_ms=%u running=%d count=%u stored=%u store_error=%s queue=%u/%u active=%d ip=%s accel=%.3f,%.3f,%.3f",
             static_cast<int>(esp_reset_reason()), static_cast<unsigned>(millis()), runUntilMs ? 1 : 0,
             static_cast<unsigned>(sampleCount), static_cast<unsigned>(storedCount),
             storeError.isEmpty() ? "none" : storeError.c_str(), static_cast<unsigned>(queueIndex), static_cast<unsigned>(queueCount),
             queueActive ? 1 : 0, WiFi.localIP().toString().c_str(), accel[0], accel[1], accel[2]);
    return text;
}

// p / r / x / i を実行して、返事の 1 行を返す。それ以外の指令には空の文字列を返す
String runCommand(const String& line) {
    int a = 0, b = 0, c = 0, d = 0;
    if (sscanf(line.c_str(), "p %d %d", &a, &b) == 2) {
        stopAll();
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = a;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(b, RUN_LIMIT_MS);
        return "# ok";
    }
    if (sscanf(line.c_str(), "r %d %d %d %d", &a, &b, &c, &d) == 4) {
        stopAll();
        rocking = true;
        rockLow = a;
        rockHigh = b;
        rockDuty = abs(c);
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = rockDuty;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(d, RUN_LIMIT_MS);
        return "# ok";
    }
    if (sscanf(line.c_str(), "h %d %d %d %d", &a, &b, &c, &d) == 4) {
        stopAll();
        hybrid = Hybrid::start;
        hybridPrevPos = -1;
        hybridLow = -1;
        hybridDelayMs = -1;
        rockHigh = a;
        rockDuty = abs(b);
        hybridTimedMs = c;
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = 0;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(d, RUN_LIMIT_MS);
        return "# ok";
    }
    if (sscanf(line.c_str(), "w %d %d %d %d", &a, &b, &c, &d) == 4) {
        if (b < HYBRID_FAR_MIN || b > 1022 || a >= HYBRID_FAR_MIN) {
            return "# bad range";
        }
        stopAll();
        hybrid = Hybrid::start;
        hybridPrevPos = -1;
        hybridLow = b;
        hybridDelayMs = -1;
        rockHigh = a;
        rockDuty = abs(c);
        hybridTimedMs = HYBRID_DOWN_LIMIT_MS;
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = 0;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(d, RUN_LIMIT_MS);
        return "# ok";
    }
    if (sscanf(line.c_str(), "k %d %d", &a, &b) == 2) {
        stopAll();
        calibReset();
        calibActive = true;
        calibDuty = a;
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = a;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(b, RUN_LIMIT_MS);
        return "# ok";
    }
    if (line.startsWith("j")) {
        String out = "# calib n=" + String(calibStored) + " cw=" + String(calibCw);
        Preferences store;
        store.begin("yawprobe", true);
        for (int i = max(0, calibStored - CALIB_KEEP); i < calibStored; i++) {
            out += " | " + String(i) + ": " + store.getString(("k" + String(i % CALIB_KEEP)).c_str(), "");
        }
        store.end();
        return out;
    }
    if (sscanf(line.c_str(), "e %d %d %d %d", &a, &b, &c, &d) == 4) {
        if (a == -1 || a == -2) {
            if (calibCw < 0) {
                return "# no calibration";
            }
            a = (calibCw + (a == -2 ? 576 : 0)) % 1152;
        }
        if (a < 0 || a >= static_cast<int>(CIRCLE) || b <= 0 || b > 400) {
            return "# bad range";
        }
        stopAll();
        estRock = true;
        estInit = estPlaced = false;
        estPrevRaw = -1;
        estMiss = 0;
        estCenter = a;
        estHalf = b;
        rockDuty = abs(c);
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = 0;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(d, RUN_LIMIT_MS);
        return "# ok";
    }
    if (sscanf(line.c_str(), "v %d %d %d %d", &a, &b, &c, &d) == 4) {
        if (a <= HYBRID_SYNC_POS || a >= HYBRID_FAR_MIN || b < 0) {
            return "# bad range";
        }
        stopAll();
        hybrid = Hybrid::start;
        hybridPrevPos = -1;
        hybridLow = -1;
        hybridDelayMs = b;
        rockHigh = a;
        rockDuty = abs(c);
        hybridTimedMs = HYBRID_DOWN_LIMIT_MS;
        if (!queueActive) {
            sampleCount = 0;
            runIndex = 0;
        }
        startDuty = 0;
        startAtMs = millis() + PREROLL_MS;
        runUntilMs = startAtMs + min<uint32_t>(d, RUN_LIMIT_MS);
        return "# ok";
    }
    if (line.startsWith("x")) {
        queueActive = false;
        stopAll();
        return "# stop";
    }
    if (line.startsWith("q ")) {
        if (queueCount >= QUEUE_MAX) {
            return "# queue full";
        }
        queued[queueCount++] = line.substring(2);
        saveQueue();
        return "# queued " + String(queueCount);
    }
    if (line.startsWith("c")) {
        queueActive = false;
        queueCount = 0;
        saveQueue();
        return "# cleared";
    }
    if (line.startsWith("g")) {
        if (queueCount == 0) {
            return "# queue empty";
        }
        if (sampleCount > 0) {
            return "# records exist. send z first";
        }
        startQueue(millis());
        return "# go";
    }
    if (line.startsWith("z")) {
        if (runUntilMs || queueActive) {
            return "# busy";
        }
        calibStored = 0;
        calibCw = -1;
        {
            Preferences store;
            store.begin("yawprobe", false);
            store.putInt("kn", 0);
            store.putInt("kcw", -1);
            store.end();
        }
        storeErase();
        return storeError.isEmpty() ? "# erased" : "# erase failed: " + storeError;
    }
    if (line.startsWith("i")) {
        return infoLine();
    }
    return "";
}

void handleLine(const String& line) {
    const String reply = runCommand(line);
    if (!reply.isEmpty()) {
        Serial.println(reply);
    } else if (line.startsWith("d")) {
        Serial.printf("# dump count=%u\n", static_cast<unsigned>(sampleCount));
        for (size_t i = 0; i < sampleCount; i++) {
            printSample(samples[i]);
            if (i % 32 == 31) {
                delay(5);
            }
        }
        Serial.println("# end");
    } else {
        Serial.printf("# unknown: %s\n", line.c_str());
    }
}

void handleHttpCommand() {
    const String reply = runCommand(server.arg("c"));
    server.send(reply.isEmpty() ? 400 : 200, "text/plain", reply.isEmpty() ? "# unknown" : reply);
}

void handleHttpDump() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain", "");
    String chunk;
    chunk.reserve(4200);
    char text[120];
    snprintf(text, sizeof(text), "# dump count=%u\n", static_cast<unsigned>(sampleCount));
    chunk = text;
    for (size_t i = 0; i < sampleCount; i++) {
        formatSample(text, sizeof(text), samples[i]);
        chunk += text;
        if (chunk.length() > 4000) {
            server.sendContent(chunk);
            chunk = "";
        }
    }
    chunk += "# end\n";
    server.sendContent(chunk);
    server.sendContent("");
}

}  // namespace

void setup() {
    M5.begin();
    Serial.begin(115200);
    M5.Display.setBrightness(120);
    M5.Display.fillScreen(TFT_WHITE);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.setFont(&fonts::lgfxJapanGothic_24);
    M5.Display.drawString("首の位置の読み取り試験", 10, 10);

    // サーボの電源を入れる(公式ライブラリの io_expander_init と同じ手順)
    bool expanderReady = false;
    const uint32_t started = millis();
    while (!expanderReady && millis() - started <= 1200) {
        delay(200);
        expanderReady = expander.begin();
    }
    if (expanderReady) {
        expander.setDirection(0, true);
        expander.setPullMode(0, m5::PY32IOExpander_Class::pull_up);
        expander.digitalWrite(0, true);
        delay(200);
    }

    const bool busReady = bus.begin(UART_NUM_1, 1000000, 6, 7);
    const int mode = bus.SwitchMode(YAW_ID, 1);  // 連続回転(PWM)にする
    writeDuty(0);
    Serial.printf("# expander=%d bus=%d switch=%d\n", expanderReady ? 1 : 0, busReady ? 1 : 0, mode);

    // 歩行用のファームウェアが保存した家の Wi-Fi の設定を読む(読むだけで、書き換えない)
    Preferences prefs;
    prefs.begin("strider", true);
    const String ssid = prefs.getString("ssid", "");
    const String pass = prefs.getString("pass", "");
    prefs.end();
    M5.Display.setFont(&fonts::lgfxJapanGothic_16);
    if (ssid.isEmpty()) {
        M5.Display.drawString("家の Wi-Fi の設定なし(USB だけ)", 10, 100);
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        const uint32_t began = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - began < 20000) {
            delay(200);
        }
        if (WiFi.status() == WL_CONNECTED) {
            server.on("/cmd", handleHttpCommand);
            server.on("/dump", handleHttpDump);
            server.begin();
            M5.Display.drawString(("Wi-Fi: " + WiFi.localIP().toString()).c_str(), 10, 100);
        } else {
            M5.Display.drawString("家の Wi-Fi につながらない(USB だけ)", 10, 100);
        }
    }
    M5.Display.setFont(&fonts::lgfxJapanGothic_24);

    loadQueue();
    {
        Preferences store;
        store.begin("yawprobe", true);
        calibStored = store.getInt("kn", 0);
        calibCw = store.getInt("kcw", -1);
        store.end();
    }
    storeLoad();
    xTaskCreate(gyroTask, "gyro", 4096, nullptr, 5, nullptr);
}

void loop() {
    static String line;
    while (Serial.available()) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n') {
            line.trim();
            if (!line.isEmpty()) {
                handleLine(line);
            }
            line = "";
        } else {
            line += ch;
        }
    }
    server.handleClient();

    const uint32_t now = millis();
    if (runUntilMs && now >= runUntilMs) {
        const bool calibrated = calibActive;
        stopAll();
        if (calibrated) {
            calibFinish();
        }
        if (queueActive) {
            storeAppend();
        }
        nextQueueAtMs = millis() + 1500;
    }
    // 予約の実行: 前の実行が終わって 1.5 秒たったら、次を始める
    if (queueActive && !runUntilMs && now >= nextQueueAtMs) {
        if (queueIndex < queueCount) {
            runIndex = static_cast<int>(queueIndex) + 1;
            runCommand(queued[queueIndex++]);
        } else {
            queueActive = false;
        }
    }
    if (startAtMs && now >= startAtMs) {
        startAtMs = 0;
        walkStartedMs = millis();
        writeDuty(startDuty);
    }

    const bool running = runUntilMs != 0;
    // 画面のタッチは 30 ms ごとに見る。触れた瞬間を拾う
    if (now - lastTouchMs >= 30) {
        lastTouchMs = now;
        bool pressed = false, holding = false;
        {
            std::lock_guard<std::mutex> lock(i2cMutex);
            M5.update();
            pressed = M5.Touch.getDetail().wasPressed();
            holding = M5.Touch.getDetail().isPressed();
        }
        if (pressed) {
            holdStartMs = now;
        } else if (!holding) {
            holdStartMs = 0;
        }
        if (queueActive) {
            // 実行中のタッチ: 止める。途中の回の記録は捨てる
            if (pressed) {
                queueActive = false;
                stopAll();
                sampleCount = storedCount;
                holdStartMs = 0;
            }
        } else if (!running && queueCount > 0) {
            // 最後まで終わった記録は、触れただけでは消さない(2 秒の長押しでやり直す)
            const bool redo = queueFinished() ? holdStartMs && now - holdStartMs >= REDO_HOLD_MS : pressed;
            if (redo) {
                holdStartMs = 0;
                startQueue(now);
            }
        }
    }
    if (!running && now - lastIdleLogMs < 200) {
        delay(2);
        return;
    }
    lastIdleLogMs = now;

    const int64_t before = esp_timer_get_time();
    const int got = bus.FeedBack(YAW_ID);
    const int64_t after = esp_timer_get_time();
    int pos = -1, speed = -1, load = -1, volt = -1, temp = -1;
    if (got >= 0) {
        pos = bus.ReadPos(-1);
        speed = bus.ReadSpeed(-1);
        load = bus.ReadLoad(-1);
        volt = bus.ReadVoltage(-1);
        temp = bus.ReadTemper(-1);
    }
    // e の往復: 首の位置の見積もりを進める
    if (estRock && running) {
        const double dt = (after - estUs) / 1e6;
        estUs = after;
        estUsed = false;
        if (!estInit) {
            if (got >= 0) {
                est = pos;
                estVel = 0;
                estInit = true;
                estPrevRaw = pos;
                estUsed = true;
            }
        } else {
            // 出力が正のとき、位置は減る
            const double target = -duty / 1000.0 * EST_VMAX;
            estVel += (target - estVel) * min(1.0, dt / EST_TAU);
            const double predicted = est + estVel * dt;
            if (got >= 0) {
                const bool clamped = pos <= 10 || pos >= 1015;  // 読めない区間の端では 3 か 1022 が返る
                const double candidate = pos + CIRCLE * round((predicted - pos) / CIRCLE);
                const bool consistent = !clamped && fabs(candidate - predicted) < EST_TOL;
                // 読めない区間を抜けた直後(端の値の次に、端のすぐ内側の値が、抜ける向きに動いているときに出た)
                const bool anchor = !clamped && estPrevRaw >= 0 &&
                                    ((estPrevRaw <= 10 && pos < 80 && estVel > 0) || (estPrevRaw >= 1015 && pos > 950 && estVel < 0));
                // 見積もりでは読める場所(読めない区間から離れた所)にいるのに、なめらかに続く読みが食い違い続けたら、
                // 見積もりのほうが違う(始めに読めない区間にいて、でたらめな値を最初の位置にした、など)
                const double wrapped = predicted - CIRCLE * floor(predicted / CIRCLE);
                const bool smooth = !clamped && estPrevRaw >= 0 && abs(pos - estPrevRaw) < 60;
                estMiss = (!consistent && !anchor && smooth && wrapped > 150 && wrapped < 900) ? estMiss + 1 : 0;
                if (consistent || anchor) {
                    estVel = 0.5 * estVel + 0.5 * (candidate - est) / dt;
                    est = candidate;
                    estUsed = true;
                } else if (estMiss >= EST_MISS_MAX) {
                    est = candidate;
                    estMiss = 0;
                    estUsed = true;
                }
                estPrevRaw = pos;
            }
            if (!estUsed) {
                est = predicted;
            }
        }
    }
    Sample sample{static_cast<uint32_t>(after), static_cast<int16_t>(duty),    static_cast<int16_t>(pos),
                  static_cast<int16_t>(speed),  static_cast<int16_t>(load),    static_cast<int16_t>(runIndex),
                  static_cast<int8_t>(volt),    static_cast<int8_t>(temp),     static_cast<uint16_t>(after - before),
                  {0, 0, 0}};
    portENTER_CRITICAL(&gyroMux);
    for (int axis = 0; axis < 3; axis++) {
        sample.angle[axis] = static_cast<int32_t>(gyroAngle[axis] * 1000.0);
    }
    portEXIT_CRITICAL(&gyroMux);
    // k: 揺れの速さを、首の位置ごとに積み上げる
    if (calibActive && running && !startAtMs) {
        const int32_t angle[2] = {sample.angle[0], sample.angle[1]};
        const bool readable = got >= 0 && pos > 10 && pos < 1015 && calibPrevPos >= 0 && abs(pos - calibPrevPos) < 100;
        if (calibPrevUs && readable && millis() - walkStartedMs >= CALIB_SKIP_MS) {
            const double dt = (after - calibPrevUs) / 1e6;
            const double theta = 2 * M_PI * pos / 1152.0;
            const double basis[5] = {1.0, cos(theta), sin(theta), cos(2 * theta), sin(2 * theta)};
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    calibAtA[i][j] += basis[i] * basis[j];
                }
                for (int axis = 0; axis < 2; axis++) {
                    calibAtB[axis][i] += basis[i] * (angle[axis] - calibPrevAngle[axis]) / 1000.0 / dt;
                }
            }
            calibCount++;
        }
        calibPrevUs = after;
        calibPrevAngle[0] = angle[0];
        calibPrevAngle[1] = angle[1];
        calibPrevPos = got >= 0 ? pos : -1;
    }
    if (estRock && running) {
        const double wrapped = est - CIRCLE * floor(est / CIRCLE);
        sample.speed = static_cast<int16_t>(wrapped);
        sample.load = estUsed ? 1 : 0;
    }
    if (!running) {
        printSample(sample);
    } else if (sampleCount < SAMPLE_MAX) {
        samples[sampleCount++] = sample;
    }

    // 往復: 読めた位置が端を越えたら向きを変える。読めなかったときは向きを変えない(記録には -1 が残る)
    // 出力が正のとき位置の値は減る(2026-11-20 の実機で確認)
    if (estRock && running && !startAtMs && estInit) {
        // 中心からの差(−半周〜+半周)で判定する。見積もりを合わせ直して周がずれても、判定は変わらない
        const double offset = est - estCenter - CIRCLE * round((est - estCenter) / CIRCLE);
        if (!estPlaced) {
            estPlaced = true;
            writeDuty(offset < -estHalf ? -rockDuty : rockDuty);  // 近い側から範囲に入る
        } else if (duty > 0 && offset <= -estHalf) {
            writeDuty(-rockDuty);
        } else if (duty < 0 && offset >= estHalf) {
            writeDuty(rockDuty);
        }
    }
    if (hybrid != Hybrid::off && running && !startAtMs) {
        const uint32_t t = millis();
        // 続けて読めた 2 つの位置が近いときだけ、上端を越えたと判定する(読めない区間ではでたらめな値が返るため)
        const bool near = got >= 0 && hybridPrevPos >= 0 && abs(pos - hybridPrevPos) < HYBRID_STEP;
        // w の往復では、読めない区間のでたらめな値で上端を越えたと誤判定しないよう、区間を抜けた印を見たあとだけ判定する
        if (got >= 0 && pos <= 10 && hybridArmed == 0) {
            hybridArmed = 1;
        } else if (got >= 0 && pos > 10 && pos < 150 && hybridArmed == 1) {
            hybridArmed = 2;
        }
        const bool timedOnly = hybridLow < 0 && hybridDelayMs < 0;  // h の往復
        const bool crossedUp = near && hybridPrevPos < rockHigh && pos >= rockHigh && (timedOnly || hybridArmed == 2);
        if (hybrid == Hybrid::timed && !hybridSyncMs && near && hybridPrevPos > HYBRID_SYNC_POS && pos <= HYBRID_SYNC_POS) {
            hybridSyncMs = t;
        }
        const bool delayDone = hybridDelayMs >= 0 && hybridSyncMs && t - hybridSyncMs >= static_cast<uint32_t>(hybridDelayMs);
        const bool reachedLow = hybridLow >= 0 && near && hybridPrevPos >= HYBRID_FAR_MIN && pos >= HYBRID_FAR_MIN && pos <= hybridLow;
        const bool crossedDown = near && hybridPrevPos > rockHigh && pos <= rockHigh;
        auto enter = [&](Hybrid next, int nextDuty) {
            hybrid = next;
            hybridSinceMs = t;
            hybridArmed = 0;
            hybridSyncMs = 0;
            writeDuty(nextDuty);
        };
        if (hybrid == Hybrid::start && got >= 0) {
            if (pos > rockHigh) {
                enter(Hybrid::seek, 300);
            } else {
                enter(Hybrid::back, -rockDuty);
                hybridArmed = 2;  // 上端より下の読める位置から始めるので、読めない区間は通らない
            }
        } else if (hybrid == Hybrid::seek && crossedDown) {
            enter(Hybrid::timed, rockDuty);
        } else if (hybrid == Hybrid::timed && (reachedLow || delayDone || t - hybridSinceMs >= hybridTimedMs)) {
            enter(Hybrid::back, -rockDuty);
        } else if (hybrid == Hybrid::back && crossedUp) {
            enter(Hybrid::timed, rockDuty);
        } else if ((hybrid == Hybrid::back && t - hybridSinceMs >= HYBRID_BACK_LIMIT_MS) ||
                   (hybrid == Hybrid::seek && t - hybridSinceMs >= 4000)) {
            enter(Hybrid::lost, 0);
        }
        if (got >= 0) {
            hybridPrevPos = pos;
        }
    }
    if (rocking && !startAtMs && got >= 0) {
        if (duty > 0 && pos <= rockLow) {
            writeDuty(-rockDuty);
        } else if (duty < 0 && pos >= rockHigh) {
            writeDuty(rockDuty);
        }
    }

    if (!running && now - lastScreenMs >= 250) {
        lastScreenMs = now;
        char text[96];
        snprintf(text, sizeof(text), "位置 %5d  出力 %5d ", pos, duty);
        M5.Display.drawString(text, 10, 60);
        if (!storeError.isEmpty()) {
            snprintf(text, sizeof(text), "保存に失敗 %s  ", storeError.c_str());
        } else if (queueActive) {
            snprintf(text, sizeof(text), "始まります %u / %u      ", static_cast<unsigned>(queueIndex), static_cast<unsigned>(queueCount));
        } else if (queueFinished()) {
            snprintf(text, sizeof(text), "終わり。記録 %u 保存 %u   ", static_cast<unsigned>(sampleCount),
                     static_cast<unsigned>(storedCount));
        } else if (queueCount > 0 && sampleCount > 0) {
            snprintf(text, sizeof(text), "途中 %d / %u。タッチでやり直し", static_cast<int>(samples[sampleCount - 1].current),
                     static_cast<unsigned>(queueCount));
        } else if (queueCount > 0) {
            snprintf(text, sizeof(text), "予約 %u 個。タッチで開始   ", static_cast<unsigned>(queueCount));
        } else {
            snprintf(text, sizeof(text), "予約なし                  ");
        }
        M5.Display.drawString(text, 10, 140);
        M5.Display.drawString(queueFinished() ? "やり直しは 2 秒長押し      " : queueActive ? "タッチで止める            " : "                          ", 10, 180);
    }
}
