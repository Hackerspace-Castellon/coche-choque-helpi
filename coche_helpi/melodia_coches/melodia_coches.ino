// Melodia de coches de choque

const int melody[] = {
  392, 440, 494, 440,
  392, 440, 494, 440,
  392, 440, 494, 440,
  392, 440,

  494, 440, 392, 440,
  494, 440, 392, 440,
  494, 440, 392, 440,

  494, 440, 392, 440,
  494
};

const int durations[] = {
  375, 375, 375, 375,
  375, 375, 375, 375,
  375, 375, 375, 375,
  375, 375,

  375, 375, 375, 375,
  375, 375, 375, 375,
  375, 375, 375, 375,

  375, 375, 375, 375,
  375
};

const int melodyLength = sizeof(melody) / sizeof(melody[0]);

void setup() {
}

void loop() {

  for (int i = 0; i < melodyLength; i++) {

    tone(8, melody[i], durations[i]);

    delay(durations[i] * 1.15);

    noTone(8);
  }

  delay(1000);
}