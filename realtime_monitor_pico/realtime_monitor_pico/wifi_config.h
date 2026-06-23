//this template file is to be created by the user to store their Wi-Fi credentials
#include <Arduino.h>
#include <pico_ota.h>


const char *ssid = "Kura-Station-Ex"; //change your Wi-Fi name here
const char *password = "Daruk355"; //change your Wi-Fi name here
const char *hostname = "pico-ota"; // optional: set a custom hostname

// ★重要: setup1()/loop1() はここではもう定義しない。
// Arduino-Picoは setup1/loop1 という名前のシンボルがビルドに存在すると
// 自動的にCore1を起動してそれらを呼び出す。
// OTAとWiFi/UDP通信を別コアで同時に行うと、RP2040のWiFi/lwIPスタックが
// コア間排他制御されていないため内部状態が壊れ、クラッシュや再起動ループの原因になる。
// そのためOTAの初期化(otaSetup)・ループ(otaLoop)は、メインの.inoファイル側の
// setup()/loop()（Core0）からまとめて呼び出す。