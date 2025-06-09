#include "MyBLEClient.hpp"

extern "C" void app_main(void) {
    MyBLEClient::init();         // Inicializa BLE
    MyBLEClient::startScan(10);  // Escanea por 10 segundos
}
