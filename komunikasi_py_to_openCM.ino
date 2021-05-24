
#define ID_NUM_21  21
int deg2pos_AX12(float _angle){
  int ret;
  float D2R = 3.14f/180.0f;
  _angle = _angle * D2R;
  ret = _angle * 197;
  ret = ret + 512;
  return ret;
}
int posisi;
#define DXL_BUS_SERIAL1 1
Dynamixel Dxl(DXL_BUS_SERIAL1);
#define GOAL_POSITION 30
void setup() {
  pinMode(BOARD_LED_PIN, OUTPUT);
//  SerialUSB.begin(9600);
Serial3.begin(9600);
Dxl.begin(3);
Dxl.jointMode(ID_NUM_21);
Dxl.writeWord(ID_NUM_21, GOAL_POSITION, deg2pos_AX12(0));
delay(1000);
}

void loop() {
  if(Serial3.available()>0) {
    char data = Serial3.read();
    Serial3.print("Arduino: Received ");
    Serial3.println(data);	
  
    if( data == 'a'){
      posisi+=7;
    }
    if(data == 'b'){
      posisi+=5;
    }
    if(data == 'c'){
      posisi+=3;
    }
    if(data == 'd'){
      posisi+=1;
    }
    if(data == 'e'){
      posisi+=0;
    }
    if(data == 'f'){
      posisi+=0;
    }
    if(data == 'g'){
      posisi-=1;
    }
    if(data == 'h'){
      posisi-=3;
    }
    if(data == 'i'){
      posisi-=5;
    }
    if(data == 'j'){
      posisi-=7;
    }    
  }
  Dxl.writeWord(ID_NUM_21, GOAL_POSITION, deg2pos_AX12(posisi));
}
