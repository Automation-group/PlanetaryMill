#include <LiquidCrystal.h>

// Инициализация контактов для подключения LCD диспрелея
// для SafeDuino MEGA 2560
LiquidCrystal lcd(J6, J7, J2, J3, J4, J5);

// Часовой таймер
struct timer {
  unsigned int hour;
  unsigned int minut;
  unsigned int sec;
  unsigned int ticks;
} time;
char timerHMS[9] = "00:00:00"; // часы:минуты:секунды
unsigned int valueClockTimer = 31307; // чиcло прерываний по часовому кварцу для отсчёта одной секунды
int timeSecOld = 0; // счётчик для прерывания каждую секунду

// Энкодер для настройки частоты оборотов
unsigned long int rpm = 10;      // число оборотов в минуту круглого основания от 10 об/мин до 600 об/мин
unsigned long int rpmStep = 10;   // шаг изменения значения скорости об/мин
const int pinFreq_A = A1;
const int pinFreq_B = A0;
unsigned char encoderFreq_A;
unsigned char encoderFreq_B;
unsigned char encoderFreq_A_prev=0;

// Энкодер для настройки времени работы
char timerMilling[6] = "00:01"; // ображение время помола на lcd
char timerPause[6] = "00:00"; // отображение время паузы на lcd
unsigned long int millingTime = 1;  // время помола в минутах от 1 мин до 600 мин
unsigned long int pauseTime = 0;    // время перерыва в минутах от 0 мин до 600 мин
int MillingTimeOn = 1;// параметр задаёт переключение ввода с время помола на время паузы
unsigned int buttonTimerOn = 0; // маркер срабатывания кнопки переключения временем помола и паузой
const int pinTime_A = A4;
const int pinTime_B = A3;
const int pinTime_D = A2;
unsigned char encoderTime_A;
unsigned char encoderTime_B;
unsigned char encoderTime_D;
unsigned char encoderTime_A_prev=0;

// Энкодер для настройки циклов повторения и реверса
int repetitions = 0;   // число повторов от 0 до 99
int reverse = 0;       // ревер: 0 - выключен, 1 - включен
unsigned int buttonReverseOn = 0; // маркер срабатывания кнопки реверса
const int pinRepet_A = A6;
const int pinRepet_B = A5;
const int pinReverse_D = A7;
unsigned char encoderRepet_A;
unsigned char encoderRepet_B;
unsigned char encoderReverse_D;
unsigned char encoderRepet_A_prev=0;

// Кнопки Пуск и Стоп
const int pinButtonStart = A8;
const int pinButtonStop = A9;
unsigned char buttonStart; 
unsigned char buttonStop;
unsigned int buttonStartOn = 0; // маркер о срабатывании кнопки Start
unsigned int millingOn = 0; // влючение помола: 0 - помол выключен, 1 - помол включен 

// Комманды управления Innovert передаваемые по интерфейсу RS-485
const byte forvArray[8]={0x01,0x06,0x20,0x00,0x00,0x0A,0x02,0x0D};// массив Вперед
const byte backArray[8]={0x01,0x06,0x20,0x00,0x00,0x06,0x02,0x08};// массив Назад
const byte stopArray[8]={0x01,0x06,0x20,0x00,0x00,0x01,0x43,0xCA};// массив Стоп
byte freqArray[8]={0x01,0x06,0x20,0x01,0x00,0x00,0x00,0x00};// массив Частота
byte readArray[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};// массив данных чтения
unsigned int freqConverter = (rpm*100)/94; // частота инвертора в Гц умноженная на 10 и делится на коэффициент 9.4
byte blConverter = freqConverter & 0xff; // младший байт частоты 
byte bhConverter = freqConverter >> 8; // старший байт частоты

// Счётчик оборотов на датчике Холла
// Используется только для отслеживания полной остановки планетарной мельницы
int stopOn = 0; // маркер нажатия кнопки СТОП
unsigned long int HallSensorCounter = 0; // счётчик оборотов
unsigned int stopCounter = 0; // счётчик отсчёта одной секунды на базе таймера 4 (250 Гц)

// Переменны используемые во время работы двигателя
int repetitionsNum = repetitions; // текущее число повторов
long int millingTimerSec = 0; // текущее время работы в секундах
long int pauseTimerSec = 0; // текущее время паузы в секундах
int pauseTimeOn = 0; // флаг на включение времени паузы
int reverseOn = 0; // флаг на включение реверса

unsigned int numberCycles = 0; // полное число циклов программы помол/пауза/реверс/пауза/повторы
unsigned int numberCyclesWorked = 0; // отработаннное цисло циклов программы помол/пауза/реверс/пауза/повторы

void setup() {
  // Инициализация порта для отладки программы
  Serial.begin(9600);
  // Иницилизация UART1 для подключения к преобразователю интерфейса RS-485
  Serial1.begin(9600);
  pinMode(10, OUTPUT); // контакт для реализации преобразователя интерйеса UART->RS-485
  digitalWrite(10, HIGH);

  // Настройко энкедера частоты оборотов
  pinMode(pinFreq_A, INPUT);
  pinMode(pinFreq_B, INPUT);

  // Настройко энкедера времени работы
  pinMode(pinTime_A, INPUT);
  pinMode(pinTime_B, INPUT);
  pinMode(pinTime_D, INPUT);

  // Настройко энкедера повторов
  pinMode(pinRepet_A, INPUT);
  pinMode(pinRepet_B, INPUT);
  pinMode(pinReverse_D, INPUT);

  // Настройка кнопок Пуск и Стоп
  pinMode(pinButtonStart, INPUT);
  pinMode(pinButtonStop, INPUT);

  // Настройка сигналов управления
  pinMode(14, OUTPUT); // блокировка/разблокировка крышки
  digitalWrite(14, HIGH); // разблокировка крышки

  pinMode(A14, INPUT); // сигнал о готовности устройства к работе (PASS OK)
  pinMode(A12, INPUT); // аналоговый сигнал с инклинометра Vadc>0.12В - всё в порядке, Vadc<0.12В - мельница наклонена 
  analogReference(INTERNAL1V1); // Vref = 1.1В

  // Инициализация прерываний от цифрового порта 3 по переднему фронту
  attachInterrupt(0, interruptsHallSensor, FALLING);

  // Настройка прерываний по таймер 4 (основной цикл работы программы)
  cli();
  TCCR4A = 0; // reset bit WGM21, WGM20
  TCCR4B = 0;
  //TCCR4B |= (0<<CS42)|(0<<CS41)|(1<<CS40); // clkI/1
  //TCCR4B |= (0<<CS42)|(1<<CS41)|(0<<CS40); // clkI/8
  //TCCR4B |= (0<<CS42)|(1<<CS41)|(1<<CS40); // clkI/64
  TCCR4B |= (1<<CS42)|(0<<CS41)|(0<<CS40); // clkI/256
  //TCCR4B |= (1<<CS42)|(0<<CS41)|(1<<CS40); // clkI/1024
  TCCR4B |= (1<<WGM42); // прерываение по совпадению c OCR4A
  TIMSK4 |= (1<<OCIE4A);
  OCR4A = 250; // 250 Гц

  // Найстройка прерываний по часовому таймеру
  TIMSK2 = 0;
  //ASSR |= (0<<EXCLK)|(0<<AS2); // AS2 -> 0 тактирование от clkI, AS2 -> 1 тактирование от TOSC1
  TCCR2B = (0<<CS22)|(0<<CS21)|(1<<CS20); // clkI/1 
  TIMSK2 = (1<<TOIE2); // включить прерывание по переполнению
  sei();

  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  // Вывод на дисплей частоты оборотов двигателя на экран об/мин
  lcd.setCursor(0, 0);
  lcd.print("Freq:");
  lcd.setCursor(6, 0);
  lcd.print(rpm);
  lcd.setCursor(10, 0);
  lcd.print("rpm");
  
  // Вывод на дисплей времени работы
  lcd.setCursor(0, 1);
  lcd.print("Mill:");
  lcd.setCursor(6, 1);
  lcd.print(timerMilling);
  lcd.print(" hour:min");

  // Вывод на дисплей числа повторов 
  lcd.setCursor(0, 2);
  lcd.print("Rep:");
  lcd.setCursor(6, 2);
  lcd.print(repetitions+1);

  // Вывод на дисплей реверса
  lcd.setCursor(12, 2);
  lcd.print("Rev:");
  lcd.setCursor(17, 2);
  lcd.print("off");

  // Вывод на дисплей времени таймера
  lcd.setCursor(0, 3);
  lcd.print("Timer: 00:00:00 Set");
}

void loop() {
}

void interruptsHallSensor () {
  if(stopOn) HallSensorCounter++;
}

// Прервывания по часовому таймеру
ISR(TIMER2_OVF_vect) {
  time.ticks++;
  if(time.ticks >= valueClockTimer) {
    time.ticks = 0;
    time.sec++;
    if(time.sec==60){
      time.sec=0;
      time.minut++;
    }
      if(time.minut==60){
      time.minut=0;
      time.hour++;
    }
  }
}

ISR(TIMER4_COMPA_vect) {
  // Опрос кнопок Старт и Стоп
  buttonStart = digitalRead(pinButtonStart); 
  buttonStop = digitalRead(pinButtonStop);

  // Опрос энкодеров если планетарная мельница остановлена
  if (!millingOn) encoder();
  
  // Отработка полной остановки двигателя и разблокировка крышки
  if (stopOn) {
    stopCounter++;
    if (stopCounter >= 250) {
      if(!HallSensorCounter) {
        stopOn = 0;
        digitalWrite(14, HIGH); // разблокировка крышки
      }
      stopCounter = 0;
      HallSensorCounter = 0;
    }
  }

  // Обработка кнопок старт и стоп
  if (!buttonStop && buttonStartOn) {
    numberCyclesWorked = 0;
    millingTimerSec = 0;
    pauseTimerSec = 0;
    pauseTimeOn = 0;
    reverseOn = 0;
    stopMill ();
    lcd.setCursor(16, 3); // сигнализация о окончании работы программы
    lcd.print("End ");
  }
  if (!buttonStart && !buttonStartOn) {
    // Обработка сигнала с инклинометра
    if (analogRead(A12) < 112) {
      lcd.setCursor(16, 3);
      lcd.print("ErrI"); // Ошибка! Планетарная мельница имеет наклон больше допустимого
      Serial.println("ErrI");
      return 0;
    }
    // Обработка закрытия крышки
    if (digitalRead(A14) == 0) {
      digitalWrite(14, LOW); // блокировка крышки
    } else {
      lcd.setCursor(16, 3);
      lcd.print("ErrO"); // Ошибка! У планетаной мельницы открыта крышка
      Serial.println("ErrO");
      return 0;
    }
    // Обработка сигнала самодиагностики
    delay(3000); // ожидание срабатывания реле
    if (digitalRead(A14) == 1) {
      lcd.setCursor(16, 3);
      lcd.print("ErrP"); // Ошибка! Самодиагностика не пройдена
      Serial.println("ErrP");
      digitalWrite(14, HIGH);
      return 0;
    }
    timeSecOld = 0; // начальная секунда для отсчёта прерываний
    buttonStartOn = 1;
    millingOn = 1;
    // Расчёт числа циклов работы программы
    if (pauseTime > 0) numberCycles = 2*(reverse+1)*(repetitions+1);
    else numberCycles = (reverse+1)*(repetitions+1);
    repetitionsNum = repetitions; // задание числа повторений программы помола
    converterSetFreq (); // задать частоту оборотов двигателя
    clockTimerReset (); // сброс времени перед началом помола
    TIMSK2 = (1<<TOIE2); // включить часовой таймер
    converterForward ();  // включить двигатель на движение вперёд
    Serial.println("start");
    Serial.println("driver forward");
    lcd.setCursor(16, 3); // сигнализация о начале работы программы
    lcd.print("Proc");
  }

  // Обработка программы помола на основе заданных данных
  if (time.sec == timeSecOld+1 && millingOn) {
    timeSecOld = time.sec;
    if(timeSecOld == 59) timeSecOld = -1; // обработка пересчёта с 59 сек до 0 сек
    
    // Отсчёт времени работы
    //unsigned long int sensCounter = millingTime*freqConverter*6;
    if (millingTime*60 <= millingTimerSec && millingOn) {
      // Обработка события реверса при выключеном времени паузы
      if (pauseTime == 0 && reverse) {
        if (!reverseOn) {
          reverseOn = 1;
          millingTimerSec = 0;
          converterBackward ();  // включить двигатель на движение назад
          Serial.println("driver backward 1");
          numberCyclesWorked++; 
        } else {
          reverseOn = 0;
          if(numberCycles > numberCyclesWorked+1) {
            converterForward ();  // включить двигатель на движение вперёд
            Serial.println("driver forward 1");
          }
          numberCyclesWorked++;
        }
      }
      // Обработка события при переходе на время паузы
      if (pauseTime > 0 && !pauseTimeOn) {
        // Удаление последней паузы в цикле помола
        if(numberCycles > numberCyclesWorked+2) {
          pauseTimeOn = 1;
          converterStop ();
          Serial.println("driver pause");
        } else {
          pauseTimerSec = pauseTime*60;
          reverseOn = 0;
          Serial.println("pause off");
        }
        numberCyclesWorked++;
      }
      // Обаботка события при окончании времени паузы
      if (pauseTime*60 <= pauseTimerSec && pauseTimeOn) {
        pauseTimeOn = 0;
        // Обработка реверса при включенной паузе
        if (reverse) {
          if (!reverseOn) {
            reverseOn = 1;
            millingTimerSec = 0;
            pauseTimerSec = 0;
            converterBackward ();  // включить двигатель на движение назад
            Serial.println("driver backward 2");
            numberCyclesWorked++;
          } else {
            reverseOn = 0;
            if(numberCycles > numberCyclesWorked+1) {
              converterForward ();  // включить двигатель на движение вперёд
              Serial.println("driver forward 2");
            }
            numberCyclesWorked++;
          }
        }
      }
    }

    //Serial.println(HallSensorCounter); // отладка
    //Serial.println(pauseTimerSec); // отладка
    // Обработка числа повторений
    if (repetitions > 0 && millingTimerSec >= millingTime*60 && pauseTimerSec >= pauseTime*60 && !reverseOn && repetitionsNum > 0) {
      repetitionsNum = repetitionsNum - 1;
      millingTimerSec = 0;
      pauseTimerSec = 0;
      pauseTimeOn = 0;
      Serial.print("repetition = ");
      Serial.println(repetitionsNum);
    }
    // Остановка программы после выполнения всех режимов
    if (millingTimerSec >= millingTime*60 && pauseTimerSec >= pauseTime*60 && !reverseOn && !repetitionsNum) {  
      stopMill ();
      numberCyclesWorked = 0;
      millingTimerSec = 0;
      pauseTimerSec = 0;
      pauseTimeOn = 0;
      Serial.println("milling end");
      lcd.setCursor(16, 3); // сигнализация о окончании работы программы
      lcd.print("End ");
    }
    // Обработка ошибок во время работы программы
    // Сигнал с инклинометра
    if (analogRead(A12) < 112) {
      stopMill ();
      numberCyclesWorked = 0;
      millingTimerSec = 0;
      pauseTimerSec = 0;
      pauseTimeOn = 0;
      lcd.setCursor(16, 3);
      lcd.print("ErrI"); // Ошибка! Планетарная мельница имеет наклон более допустимого
      Serial.println("ErrI");
    }
    // Сигнал самодиагностики
    if (digitalRead(A14) == 0) {
      stopMill ();
      numberCyclesWorked = 0;
      millingTimerSec = 0;
      pauseTimerSec = 0;
      pauseTimeOn = 0;
      lcd.setCursor(16, 3);
      lcd.print("ErrP"); // Ошибка! Самодиагностика не пройдена
      Serial.println("ErrP");
    }
    if (!pauseTimeOn) millingTimerSec++; // счётчик времени работы
    if (pauseTimeOn) pauseTimerSec++; // счётчик времени паузы
    lcdTimer(); // вывод значений таймера на дисплей
 }
}

void stopMill () {
  buttonStartOn = 0;
  millingOn = 0;
  converterStop (); // остановить двигатель
  TIMSK2 = (0<<TOIE2); // выключить часовой таймер
  millingTimerSec = 0; // сброс счётчика оборотов
  stopOn = 1; // маркер на отработку полной остановки двигателя
  Serial.println("stop");
}

void converterSetFreq () {
  freqConverter = (rpm*100)/94; // пересчёт из об/мин в частоту в Гц
  // Команда для задания частоты работы инвертора
  blConverter = (byte) (freqConverter & 0xff); // младший байт частоты 
  freqArray[5] = blConverter;
  bhConverter = (byte) (freqConverter >> 8); // старший байт частоты
  freqArray[4] = bhConverter;
  ksArray(freqArray); // рассчёт и задание контрольной суммы для комманды задания частоты
  // Задание частоты работы инвертора
  for(int i=0; i<8; i++) {   
    Serial1.write(freqArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(freqArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(freqArray[i]);
    delayMicroseconds(1000);
  }
}

void converterForward () {
  //delay(1000);
  for(int i=0; i<8; i++) {   
    Serial1.write(forvArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(forvArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(forvArray[i]);
    delayMicroseconds(1000);
  }
} 

void converterBackward () {
  //delay(1000);
  for(int i=0; i<8; i++) {   
    Serial1.write(backArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(backArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(backArray[i]);
    delayMicroseconds(1000);
  }
}

void converterStop () {
  for(int i=0; i<8; i++) {   
    Serial1.write(stopArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(stopArray[i]);
    delayMicroseconds(1000);
  }
  for(int i=0; i<8; i++) {   
    Serial1.write(stopArray[i]);
    delayMicroseconds(1000);
  }
}

void encoder(){
  // Опрос энкодера для настройи частоты оборотов
  encoderFreq_A = digitalRead(pinFreq_A); 
  encoderFreq_B = digitalRead(pinFreq_B);

  // Опрос энкодера для настройки времени работы и паузы
  encoderTime_A = digitalRead(pinTime_A); 
  encoderTime_B = digitalRead(pinTime_B);
  encoderTime_D = digitalRead(pinTime_D);

  // Опрос энкодера для настройки числа повторов и реверса
  encoderRepet_A = digitalRead(pinRepet_A); 
  encoderRepet_B = digitalRead(pinRepet_B);
  encoderReverse_D = digitalRead(pinReverse_D);

  // Обработка данных с энкодера управляющего частотой
  if(!encoderFreq_A && encoderFreq_A_prev){    // если состояние изменилось с положительного к нулю
    if(encoderFreq_B) {
      if(rpm + rpmStep <= 600) rpm += rpmStep;
      lcdFreq();
    }
    else {
      if(rpm - rpmStep >= 10) rpm -= rpmStep;
      lcdFreq();
    }
  }
  encoderFreq_A_prev = encoderFreq_A;     // сохраняем значение А для следующего цикла

  // Обработка данных с энкодера управляющего временем паузы и работы
  if(encoderTime_D) buttonTimerOn = 1;
  else {
    if (buttonTimerOn) {
      buttonTimerOn = 0;
      if(MillingTimeOn){
        MillingTimeOn = 0;
        lcdPauseTime();
      } else {
        MillingTimeOn = 1;
        lcdMillingTime();
      }
    }
  }
  if(!encoderTime_A && encoderTime_A_prev){    // если состояние изменилось с положительного к нулю
    if(encoderTime_B) {
      if (MillingTimeOn) {
        if(millingTime + 1 <= 600) millingTime += 1;
        lcdMillingTime();
      } else {
        if(pauseTime + 1 <= 600) pauseTime += 1;
        lcdPauseTime();
      }
    }
    else {
      if(MillingTimeOn) {
        if(millingTime - 1 >= 1) millingTime -= 1;
        lcdMillingTime();
      } else {
        if(pauseTime - 1 >= 0) pauseTime -= 1;
        lcdPauseTime();
      }
    }
  }
  encoderTime_A_prev = encoderTime_A;     // сохраняем значение А для следующего цикла*/

  // Обработка данных с энкодера управляющего числом повторов и реверсом
  if(encoderReverse_D) buttonReverseOn = 1;
  else {
    if(buttonReverseOn) {
      buttonReverseOn = 0;
      if(reverse){
        reverse = 0;
        lcd.setCursor(17, 2);
        lcd.print("off");
      } else {
        reverse = 1;
        lcd.setCursor(17, 2);
        lcd.print("on ");
      }
      lcd.setCursor(16, 3); // сигнализация о начале задания параметров
      lcd.print("Set ");
    }
  }
  if(!encoderRepet_A && encoderRepet_A_prev){  // если состояние изменилось с положительного к нулю
    if(encoderRepet_B) {
      if(repetitions + 1 <= 99) repetitions += 1;
      lcdRepet();
    }
    else {
      if(repetitions - 1 >= 0) repetitions -= 1;
      lcdRepet();
    }
  }
  encoderRepet_A_prev = encoderRepet_A;     // сохраняем значение А для следующего цикла
}

void lcdFreq() {
  lcd.setCursor(6, 0);
  lcd.print("   ");
  lcd.setCursor(6, 0);
  lcd.print(rpm);
  lcd.setCursor(16, 3); // сигнализация о начале задания параметров
  lcd.print("Set ");
}

void lcdMillingTime() {
  if (millingTime < 60) {
    lcd.setCursor(0, 1);
    lcd.print("Mill:");
    // минуты
    timerMilling[3] = '0' + millingTime/10;
    timerMilling[4] = '0' + millingTime%10;
    // часы
    timerMilling[0] = '0';
    timerMilling[1] = '0';
    // LCD print
    lcd.setCursor(6, 1);
    lcd.print(timerMilling);
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("Mill:");
    // минуты
    timerMilling[3] = '0' + (millingTime-(millingTime/60)*60)/10;
    timerMilling[4] = '0' + (millingTime-(millingTime/60)*60)%10;
    // часы
    timerMilling[0] = '0' + (millingTime/60)/10;
    timerMilling[1] = '0' + (millingTime/60)%10;
    // LCD print
    lcd.setCursor(6, 1);
    lcd.print(timerMilling);
  }
  lcd.setCursor(16, 3); // сигнализация о начале задания параметров
  lcd.print("Set ");
}

void lcdPauseTime() {
  if (pauseTime < 60) {
    lcd.setCursor(0, 1);
    lcd.print("Paus:");
    // минуты
    timerPause[3] = '0' + pauseTime/10;
    timerPause[4] = '0' + pauseTime%10;
    // часы
    timerPause[0] = '0';
    timerPause[1] = '0';
    // LCD print
    lcd.setCursor(6, 1);
    lcd.print(timerPause);
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("Paus:");
    // минуты
    timerPause[3] = '0' + (pauseTime-(pauseTime/60)*60)/10;
    timerPause[4] = '0' + (pauseTime-(pauseTime/60)*60)%10;
    // часы
    timerPause[0] = '0' + (pauseTime/60)/10;
    timerPause[1] = '0' + (pauseTime/60)%10;
    // LCD print
    lcd.setCursor(6, 1);
    lcd.print(timerPause);
  }
  lcd.setCursor(16, 3); // сигнализация о начале задания параметров
  lcd.print("Set ");
}

void lcdRepet() {
  lcd.setCursor(6, 2);
  lcd.print("   ");
  lcd.setCursor(6, 2);
  lcd.print(repetitions+1);
  lcd.setCursor(16, 3); // сигнализация о начале задания параметров
  lcd.print("Set ");
}

void lcdTimer(){
  // секунды
  timerHMS[6] = '0' + time.sec/10;
  timerHMS[7] = '0' + time.sec%10;
  // минуты
  timerHMS[3] = '0' + time.minut/10;
  timerHMS[4] = '0' + time.minut%10;
  // часы
  timerHMS[0] = '0' + time.hour/10;
  timerHMS[1] = '0' + time.hour%10;

  // LCD print
  lcd.setCursor(7, 3);
  lcd.print(timerHMS);
}

// Сброс часового таймера
void clockTimerReset () {
  time.hour = 0;
  time.minut = 0;
  time.sec = 0;
  time.ticks = 0;
}

void ksArray(byte buf[]) {
  int32_t crc = 0xFFFF;
  for (int pos = 0; pos < 6; pos++) {
    crc ^= (int32_t)buf[pos];          // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else                          // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  byte BH = (byte)(crc / 256);
  buf[7] = BH;
  byte BL = (byte)(crc);
  buf[6] = BL;
  //return buf[];
}
