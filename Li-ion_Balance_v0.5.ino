#include <LiquidCrystal.h>

float cell1V = 0.00;          //sets variable for the voltage of the first cell
float cell2V = 0.00;          //sets variable for the voltage of the second cell
float cell3V = 0.00;          //sets variable for the voltage of the third cell
float calV = 0.00;            //sets variable for supply voltage compensation
float supplyV = 0.00;         //sets variable for the voltage of the supply to the arduino
float cell1Cal = 50.00 + 1.50; //allows for voltage calibration of the first cell
float cell2Cal = 50.00 + 0.38; //allows for voltage calibration of the second cell
float cell3Cal = 50.00 + 0.85; //allows for voltage calibration of the third cell
float zenerV = 2.48;          // measured voltage of the voltage reference
float cell1AverageVal = 0.00; //sets variable for use in calculating the average voltage of cell 1
float cell2AverageVal = 0.00; //sets variable for use in calculating the average voltage of cell 2
float cell3AverageVal = 0.00; //sets variable for use in calculating the average voltage of cell 3
float cell1AverageV = 0.00;   //sets variable for the current voltage of cell 1
float cell2AverageV = 0.00;   //sets variable for the current voltage of cell 2
float cell3AverageV = 0.00;   //sets variable for the current voltage of cell 3

float averageAmps = 0.00;     //set variable for the average amperage going into the battery pack
float ampsAverageVal = 0.00;  //variable used for calculating the average amperage
float ampSensorMillivolts = 0.00; //used for ACS 712 readings
float currentAmps = 0.00;     //used for the present amp reading
float ampVal = 0.00;          //store the analog value for the ACS 712 without the offset
int ampSensorVal = 512;       //store the analog value for the ACS 712 sensor

float balanceVal = 4.20;      //voltage where the balancing circuits kick in
float protectVal = 4.25;      //voltage where incoming power will be shut off
float ampCutoff = 5.50;       //current where incomming power will be shut off

int zenerVal = 0;             //variable used for analog reading of the voltage reference
int cell1Val = 0;             //variable used for analog reading of cell 1
int cell2Val = 0;             //variable used for analog reading of cell 2
int cell3Val = 0;             //variable used for analog reading of cell 3
int averages = 100;           //sets the number of averages taken during each voltage Measurement

int cell1Bal = 2;             //sets pin 2 as the output to control the balance circuit for cell 1
int cell2Bal = 3;             //sets pin 3 as the output to control the balance circuit for cell 2
int cell3Bal = 4;             //sets pin 4 as the output to control the balance circuit for cell 3
int powerIn = 5;              //sets pin 5 as the output to control the incoming power

int flag = 0;                 //sets up a flag for later use in cell balancing

const int rs = 12, en = 11, d4 = 10, d5 = 9, d6 = 8, d7 = 7;  //pin config of the LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                    //used for LCD setup

byte invertedColon[8] = {
  B11111,
  B10011,
  B10011,
  B11111,
  B10011,
  B10011,
  B11111,
  B11111,
};

void setup() {                // void setup runs once upon power up
  Serial.begin(115200);       //enables serial output
  lcd.begin(16, 2);           //enables the 16x2 LCD
  lcd.write("Initializing");  //write something to the LCD
  lcd.setCursor(0, 1);        //move the cursor the the second line
                                lcd.write("Version 0.5");   //write more to the LCD
  lcd.createChar(0, invertedColon);

  pinMode(cell1Bal, OUTPUT);  //sets the pin used to contol the balance circuitry for cell 1 as an output
  pinMode(cell2Bal, OUTPUT);  //sets the pin used to contol the balance circuitry for cell 2 as an output
  pinMode(cell3Bal, OUTPUT);  //sets the pin used to contol the balance circuitry for cell 3 as an output
  pinMode(powerIn, OUTPUT);   //sets the pin used to contol the input power as an output
  delay(2000);                //2 second delay so you can read the screen
}                             //end of void setup

void loop() {                                       //void loop runs perpetually after the void setup has finished
  for (int i = 0; i < averages; i++) {              //for loop used to get an average voltage of the cells
    zenerVal = analogRead(A3);                      //read the analog value for the voltage reference
    supplyV = (zenerV * 1024.00) / zenerVal;        //calculate the supply voltage of the arduino using the analog value from above
    calV = (supplyV - 5.00) * 10.00;                //find the diffence between the supply voltage and the expected 5 volts

    cell1Val = analogRead(A0);                      //read the analog value for cell 1
    cell2Val = analogRead(A1);                      //read the analog value for cell 1&2
    cell3Val = analogRead(A2);                      //read the analog value for cell 1&2&3

    ampSensorVal = analogRead(A4);                  //read the analog value for the current sensor
    ampVal = ampSensorVal - 509;                 //get rid of the offset the sensor introduces (allows for reading positve and negative current)
    ampSensorMillivolts = (ampVal * 5000.00) / 1024;//gives us the current millivolt reading of the sensor
    currentAmps = (ampSensorMillivolts / 66);       //convert millivolts into amps
    ampsAverageVal = ampsAverageVal + currentAmps;  //add values for averaging

    cell1V = (cell1Val * (cell1Cal + calV)) / 1024; //turn the analog value into a voltage that is calibrated as accurately as possible for cell 1
    cell2V = (cell2Val * (cell2Cal + calV)) / 1024; //turn the analog value into a voltage that is calibrated as accurately as possible for cell 1&2
    cell3V = (cell3Val * (cell3Cal + calV)) / 1024; //turn the analog value into a voltage that is calibrated as accurately as possible for cell 1&2&3

    cell1AverageVal = cell1AverageVal + cell1V;     //add the present voltage to the pervious voltages of cell 1
    cell2AverageVal = cell2AverageVal + cell2V;     //add the present voltage to the pervious voltages of cell 1&2
    cell3AverageVal = cell3AverageVal + cell3V;     //add the present voltage to the pervious voltages of cell 1&2&3
    delay(1);                                       //delay between measurements
  }                                                 //end of the for loop

  cell1AverageV = cell1AverageVal / averages;       //calculate the average of the readings above for cell 1
  cell2AverageV = cell2AverageVal / averages;       //calculate the average of the readings above for cell 1&2
  cell3AverageV = cell3AverageVal / averages;       //calculate the average of the readings above for cell 1&2&3

  averageAmps = ampsAverageVal / averages;          //calculate the average of the reading for the current sensor

  cell2AverageV = cell2AverageV - cell1AverageV;    //calculate the volage of cell 2 only
  cell3AverageV = cell3AverageV - (cell2AverageV + cell1AverageV);  //calculate the volage of cell 3 only

  Serial.print("Cell 1 Voltage = ");  //dump the measured voltages and currents to the serial monitor
  Serial.println(cell1AverageV);
  Serial.print("Cell 2 Voltage = ");
  Serial.println(cell2AverageV);
  Serial.print("Cell 3 Voltage = ");
  Serial.println(cell3AverageV);
  Serial.print("Supply Voltage = ");
  Serial.println(supplyV);
  Serial.print("Amperage = ");
  Serial.println(averageAmps);        //end of serial writing

  lcd.clear();                        //clear the LCD
  lcd.print("1: ");                   //send data to the LCD
  lcd.print(cell1AverageV);
  lcd.print(" 2: ");
  lcd.print(cell2AverageV);
  lcd.setCursor(0, 1);                //move the cursor to the start of the second line
  lcd.print("3: ");
  lcd.print(cell3AverageV);        
  lcd.print(" A: ");
  lcd.print(averageAmps);              //end of lcd control section

  if (cell1AverageV >= balanceVal) {  //if the voltage of cell 1 is at or higher than the balance voltage
    digitalWrite(cell1Bal, HIGH);     //turn the balance circuit on for cell 1
    flag = 1;                         //set flag equal to 1
    lcd.setCursor(1, 0);
    lcd.write(byte(0));
  }

  else {                              //if the voltage of cell 1 is lower than the balance voltage
    digitalWrite(cell1Bal, LOW);      //turn the balance circuit off for cell 1
  }                                   //end of if/else statment

  if (cell2AverageV >= balanceVal) {  //if the voltage of cell 2 is at or higher than the balance voltage
    digitalWrite(cell2Bal, HIGH);     //turn the balance circuit on for cell 2
    flag = 1;                         //set flag equal to 1
    lcd.setCursor(9, 0);
    lcd.write(byte(0));
  }

  else {                              //if the voltage of cell 2 is lower than the balance voltage
    digitalWrite(cell2Bal, LOW);      //turn the balance circuit off for cell 2
  }                                   //end of if/else statment

  if (cell3AverageV >= balanceVal) {  //if the voltage of cell 3 is at or higher than the balance voltage
    digitalWrite(cell3Bal, HIGH);     //turn the balance circuit on for cell 3
    flag = 1;                         //set flag equal to 1
    lcd.setCursor(1, 1);
    lcd.write(byte(0));
  }

  else {                              //if the voltage of cell 3 is lower than the balance voltage
    digitalWrite(cell3Bal, LOW);      //turn the balance circuit off for cell 3
  }                                   //end of if/else statment

  if (flag == 1) {                    //if the flag has been set to 1
    delay(3000);                      //wait 3 seconds
    digitalWrite(cell1Bal, LOW);      //turn off balance circuit for cell 1
    digitalWrite(cell2Bal, LOW);      //turn off balance circuit for cell 2
    digitalWrite(cell3Bal, LOW);      //turn off balance circuit for cell 3
    flag = 0;                         //reset flag to 0
  }                                   //end of if statment

  if (cell1AverageV >= protectVal || cell2AverageV >= protectVal || cell3AverageV >= protectVal) {  //if any of the cells rise above a safe voltage
    digitalWrite(powerIn, LOW);       //turn off the incoming power
  }                                   //end of if statment

  else {                              //if the cell voltages are not above a safe voltage
    digitalWrite(powerIn, HIGH);      //turn the incomming power on
  }                                   //end of else statement

  if (cell1AverageV >= protectVal || cell2AverageV >= protectVal || cell3AverageV >= protectVal || averageAmps > ampCutoff) {  //if any of the cells rise above a safe voltage or if too much current is flowing into the pack
    digitalWrite(powerIn, LOW);       //turn off the incoming power
  }                                   //end of if statment

  else {                              //if the cell voltages are not above a safe voltage
    digitalWrite(powerIn, HIGH);      //turn the incomming power on
  }     

  cell1AverageVal = 0;                //reset the total added voltages for cell 1
  cell2AverageVal = 0;                //reset the total added voltages for cell 2
  cell3AverageVal = 0;                //reset the total added voltages for cell 3

  ampsAverageVal = 0;                 //reset the total added currents from the ACS712 sensor
}                                     //end of void loop
