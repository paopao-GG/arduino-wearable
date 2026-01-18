// Viral Science www.viralsciencecreativity.com www.youtube.com/c/viralscience
// Arduino Airsoft Time Bomb

#include <Keypad.h>
#include <Wire.h> 
#include <LCD_I2C.h>

LCD_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change according

int Hours = 0;
int Minutes = 0;
int Seconds = 0;
int trycount = 0;
int keycount = 0;
int i = 0;

int redled = A2;
int yellowled = A1;
int greenled = A3;

int hourstenscode;
int hoursonescode;
int mintenscode;
int minonescode;
int sectenscode;
int seconescode;

long secMillis = 0;
long interval = 1000;

char password[4];
char entered[4];

const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {8, 7, 6, 5};  //7,2,3,5 for Black 4x3 keypad
byte colPins[COLS] = {4, 3, 2};     //6,8,4 for Black 4x3 Keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  pinMode(redled, OUTPUT);
  pinMode(yellowled, OUTPUT);
  pinMode(greenled, OUTPUT);
  digitalWrite(redled,HIGH);
  digitalWrite(yellowled,HIGH);
  digitalWrite(greenled,HIGH);

  lcd.begin();
  lcd.backlight();
  Serial.begin(9600);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bomb activated!");
  lcd.setCursor(0, 1);
  lcd.print("Enter Code:");
  while (keycount < 4)
  {
    lcd.setCursor(keycount + 12, 1);
    lcd.blink();
    char armcode = keypad.getKey();
    armcode == NO_KEY;
    if (armcode != NO_KEY)
    {
      if ((armcode != '*') && (armcode != '#'))
      {
        lcd.print(armcode);
        tone(9, 5000, 100);
        password[keycount] = armcode;
        keycount++;
      }
    }
  }

  if (keycount == 4)
  {
    delay(500);
    lcd.noBlink();
    lcd.clear();
    lcd.home();
    lcd.print("Disarm Code is: ");
    lcd.setCursor(6, 1);
    lcd.print(password[0]);
    lcd.print(password[1]);
    lcd.print(password[2]);
    lcd.print(password[3]);
    delay(1000);
    lcd.clear();
  }
  lcd.setCursor(0, 0);
  lcd.print("Timer: HH:MM:SS");
  lcd.setCursor(0, 1);
  lcd.print("SET:   :  :");
  keycount = 5;

  while (keycount == 5)
  {
    char hourstens = keypad.getKey();
    lcd.setCursor(5, 1);
    lcd.blink();
    if (hourstens >= '0' && hourstens <= '9')
    {
      hourstenscode = hourstens - '0';
      tone(9, 5000, 100);
      lcd.print(hourstens);
      keycount++;
    }
  }

  while (keycount == 6)
  {
    char hoursones = keypad.getKey();
    lcd.setCursor(6, 1);
    lcd.blink();
    if (hoursones >= '0' && hoursones <= '9')
    {
      hoursonescode = hoursones - '0';
      tone(9, 5000, 100);
      lcd.print(hoursones);
      keycount++;
    }
  }

  while (keycount == 7)
  {
    char mintens = keypad.getKey();
    lcd.setCursor(8, 1);
    lcd.blink();
    if (mintens >= '0' && mintens <= '9')
    {
      mintenscode = mintens - '0';
      tone(9, 5000, 100);
      lcd.print(mintens);
      keycount++;
    }
  }

  while (keycount == 8)
  {
    char minones = keypad.getKey();
    lcd.setCursor(9, 1);
    lcd.blink();
    if (minones >= '0' && minones <= '9')
    {
      minonescode = minones - '0';
      tone(9, 5000, 100);
      lcd.print(minones);
      keycount++;
    }
  }

  while (keycount == 9)
  {
    char sectens = keypad.getKey();
    lcd.setCursor(11, 1);
    lcd.blink();
    if (sectens >= '0' && sectens <= '9')
    {
      sectenscode = sectens - '0';
      tone(9, 5000, 100);
      lcd.print(sectens);
      keycount = 10;
    }
  }

  while (keycount == 10)
  {
    char secones = keypad.getKey();
    lcd.setCursor(12, 1);
    lcd.blink();
    if (secones >= '0' && secones <= '9')
    {
      seconescode = secones - '0';
      tone(9, 5000, 100);
      lcd.print(secones);
      keycount = 11;
    }
  }

  if (keycount == 11);
  {
    Hours = (hourstenscode * 10) + hoursonescode;
    Minutes = (mintenscode * 10) + minonescode;
    Seconds = (sectenscode * 10) + seconescode;
    delay(100);
    lcd.noBlink();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Timer set at:");
    if (Hours >= 10)
    {
      lcd.setCursor (7, 1);
      lcd.print (Hours);
    }
    if (Hours < 10)
    {
      lcd.setCursor (7, 1);
      lcd.write ("0");
      lcd.setCursor (8, 1);
      lcd.print (Hours);
    }
    lcd.print (":");

    if (Minutes >= 10)
    {
      lcd.setCursor (10, 1);
      lcd.print (Minutes);
    }
    if (Minutes < 10)
    {
      lcd.setCursor (10, 1);
      lcd.write ("0");
      lcd.setCursor (11, 1);
      lcd.print (Minutes);
    }
    lcd.print (":");

    if (Seconds >= 10)
    {
      lcd.setCursor (13, 1);
      lcd.print (Seconds);
    }

    if (Seconds < 10)
    {
      lcd.setCursor (13, 1);
      lcd.write ("0");
      lcd.setCursor (14, 1);
      lcd.print (Seconds);
    }
    delay(1000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Press # to arm");
    delay (50);
    keycount = 12;
  }

  while (keycount == 12)
  {
    char armkey = keypad.getKey();

    if (armkey == '#')
    {
      tone(9, 5000, 100);
      delay(50);
      tone(9, 0, 100);
      delay(50);
      tone(9, 5000, 100);
      delay(50);
      tone(9, 0, 100);
      delay(50);
      tone(9, 5000, 100);
      delay(50);
      tone(9, 0, 100);
      lcd.clear();
      lcd.print ("Bomb Armed!");
      lcd.setCursor(0, 1);
      lcd.print("Countdown start");
      delay(1000);
      lcd.clear();
      keycount = 0;
    }
  }
}
void loop()
{
  timer();
  char disarmcode = keypad.getKey();

  if (disarmcode == '*')
  {
    tone(9, 5000, 100);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Code: ");

    while (keycount < 4)
    {
      timer();

      char disarmcode = keypad.getKey();
      if (disarmcode == '#')
      {
        tone(9, 5000, 100);
        keycount = 0;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Code: ");
      }
      else if (disarmcode != NO_KEY)
      {
        lcd.setCursor(keycount + 7, 0);
        lcd.blink();
        lcd.print(disarmcode);
        entered[keycount] = disarmcode;
        keycount++;
        tone(9, 5000, 100);
        delay(100);
        lcd.noBlink();
        lcd.setCursor(keycount + 6, 0);
        lcd.print("*");
        lcd.setCursor(keycount + 7, 0);
        lcd.blink();
      }
    }

    if (keycount == 4)
    {
      if (entered[0] == password[0] && entered[1] == password[1] && entered[2] == password[2] && entered[3] == password[3])
      {
        //defusing text
        lcd.noBlink();
        lcd.clear();
        lcd.home();
        
        lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("*");
        delay(1000);
        //lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* *");
        delay(1000);
       // lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* * *");
        delay(1000);
        //lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* * * *");
        delay(1000);
        //lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* * * * *");
        delay(1000);
        //lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* * * * * *");
        delay(1000);
        //lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* * * * * * *");
        delay(1000);
        //lcd.print("Defusing");
        lcd.setCursor(0, 1);
        lcd.print("* * * * * * * *");
        delay(1000);

       // keycount = 0;
        //
        lcd.noBlink();
        lcd.clear();
        lcd.home();
       
        lcd.print("Bomb Defused!");
        lcd.setCursor(0, 1);
        lcd.print("CT WINS!");
        keycount = 0;
        digitalWrite(greenled, LOW);
        delay(15000);
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Reset the Bomb");
        delay(1000000);
      }
      else
      {
        lcd.noBlink();
        lcd.clear();
        lcd.home();
        lcd.print("Wrong Password!");
        trycount++;

        if (Hours > 0)
        {
          Hours = Hours / 2;
        }

        if (Minutes > 0)
        {
          Minutes = Minutes / 2;
        }
        if (Seconds > 0)
        {
          Seconds = Seconds / 2;
        }
        if (trycount == 2)
        {
          interval = interval / 10;
        }
        if (trycount == 3)
        {
          Minutes = Minutes - 59;
          Hours = Hours - 59;
          Seconds = Seconds - 59;
        }
        delay(1000);
        keycount = 0;
      }
    }
  }
}

void timer()
{
  Serial.print(Seconds);
  Serial.println();

  if (Hours <= 0)
  {
    if ( Minutes < 0 )
    {
      lcd.noBlink();
      lcd.clear();
      lcd.home();
      lcd.print("TBomb Exploded");
      lcd.setCursor (0, 1);
      lcd.print("T Wins");

      while (Minutes < 0)
      {
        digitalWrite(redled, LOW);
        tone(9, 5000, 100);
        delay(100);
        digitalWrite(redled, HIGH);
        tone(9, 5000, 100);
        delay(100);
        digitalWrite(yellowled, LOW);
        tone(9, 5000, 100);
        delay(100);
        digitalWrite(yellowled, HIGH);
        tone(9, 5000, 100);
        delay(100);
        digitalWrite(greenled, LOW);
        tone(9, 5000, 100);
        delay(100);
        digitalWrite(greenled, HIGH);
        tone(9, 5000, 100);
        delay(100);
      }
    }
  }
  lcd.setCursor (0, 1);
  lcd.print ("Timer:");

   lcd.setCursor(7, 1);
    
    // Print time in HH:MM:SS format with leading zeros
    if (Hours < 10) lcd.print("0");
    lcd.print(Hours);
    lcd.print(":");
    if (Minutes < 10) lcd.print("0");
    lcd.print(Minutes);
    lcd.print(":");
    if (Seconds < 10) lcd.print("0");
    lcd.print(Seconds);

  if (Hours < 0)
  {
    Hours = 0;
  }

  if (Minutes < 0)
  {
    Hours --;
    Minutes = 59;
  }

  if (Seconds < 1)
  {
    Minutes --;
    Seconds = 59;
  }

  if (Seconds > 0)
  {
    unsigned long currentMillis = millis();

    if (currentMillis - secMillis > interval)
    {
      tone(9, 5000, 50);
      secMillis = currentMillis;
      Seconds --;
      digitalWrite(yellowled, LOW);
      delay(10);
      digitalWrite(yellowled, HIGH);
      delay(10);
    }
  }
}
