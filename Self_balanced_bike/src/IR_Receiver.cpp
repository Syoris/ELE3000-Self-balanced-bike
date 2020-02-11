#include <IR_Receiver.h>

IRrecv irrecv(RECV_PIN);
decode_results results;

void IR_Setup(){
  irrecv.enableIRIn(); // Start the receiver
  irrecv.blink13(true);
}

// Get the value sent
unsigned long IR_receive(){    
    if (irrecv.decode(&results)) {
        irrecv.resume(); // Receive the next value
        unsigned long val = results.value;
        if(val != 0xFFFFFFFF){
            return val;
        }
        return 0;
    }
}

// Print the name of the button
void IR_decode(){
    if (irrecv.decode(&results)) {
        unsigned long val = results.value;
        if(val != 0xFFFFFFFF){
            Serial.print("Value: ");
            switch (val)
            {
            case OFF:
                Serial.println("OFF");
                break;
            
            case UP:
                Serial.println("UP");
                break;
            
            default:
                Serial.println(val);
                break;
            }
        }
        delay(50);
        irrecv.resume(); // Receive the next value
    }
}

// Print all the information received
void IR_print_cmd(){
    if (irrecv.decode(&results)) {
        if(results.value != 0xFFFFFFFF){
        if (results.decode_type == NEC) {
            Serial.print("NEC: ");
        } else if (results.decode_type == SONY) {
            Serial.print("SONY: ");
        } else if (results.decode_type == RC5) {
            Serial.print("RC5: ");
        } else if (results.decode_type == RC6) {
            Serial.print("RC6: ");
        } else if (results.decode_type == UNKNOWN) {
            Serial.print("UNKNOWN: ");
        }
        Serial.println(results.value, HEX); 
        delay(50);
        }
        irrecv.resume(); // Receive the next value
    }
}
