void setup()
{
// initialize serial:
Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop()
{
// send "Hello World" message through onboard USB-UART converter (STLink)
Serial.println("Hello World");
// wait one second
delay(1000);
}