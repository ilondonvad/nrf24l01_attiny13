# connect attiny13 to nrf24l01 or Si24R1.
* Ресурсы взяты с сайта https://forum.arduino.cc/t/solved-attiny13a-and-nrf24l01/431914/3
* Часть кода переделана так как никак не получалось запустить и отправить данные
* Модуль отправляет данные по нажатию кнопки, один раз, два, или удержание.
* Код можно настроить на любые входные данные и выходные
* Для передатчика не нужно использовать никаких сторонних библиотек
* Для приемника можно использовать любую NRF24 совместимую библиотеку
* Работает на частоте МК - 9.6MHz - 128KHz.
* Передатчик можно настроить как угодно согласно даташиту
* Передатчик не может принимать данные, изза того что не подключен пин MISO
* Для attiny13 используеться ядро MicroCore, https://github.com/MCUdude/MicroCore
