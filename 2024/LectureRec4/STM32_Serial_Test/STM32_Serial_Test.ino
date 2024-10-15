
// set RX and TX pins
HardwareSerial Serial1(PA10, PA9);

// the setup function runs once when you press reset or power the board
void setup()
{
// initialize Serial1:
Serial1.begin(115200);
}

// the loop function runs over and over again forever
void loop()
{
// send "Hello World" message through Serial1
Serial1.println("Hello World");
// wait one second
delay(1000);
}
