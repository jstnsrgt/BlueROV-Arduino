

void setup() {
  // put your setup code here, to run once:
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(digitalRead(19));
  Serial.print("\t");
  Serial.println(digitalRead(18));
  delay(1000);
}
