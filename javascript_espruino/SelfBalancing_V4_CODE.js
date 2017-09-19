var MR_ENB = B10;
var MR_STP = A7;
var MR_DIR = B1;
var ML_ENB = B5;
var ML_STP = B4;
var ML_DIR = B3;


var KP = 150000;
var KI = 0;
var KD = 10;
var horizontalCoeff = -0.01; //adjust for the fact that the robot is not completely aligned with the sensor



var MPU;
var DMP;
    
    
// empirical max frequency/speed when microstepping (1/32 step) with current limited @ 300mA for each motor ?
var MAX_SPEED = 20000;
var MAX_INTEGRAL_ERR = 5000;


function onInit() {
  console.log("START");
  disable(true);

  I2C1.setup({scl:B6, sda:B7, bitrate:100000});
  // "true" because I have AD0 connected to VCC
  MPU = require("MPU6050").connect(I2C1, true);
  // 200Hz / (1 + 4) = 40Hz
  DMP = require("MPU6050_DMP-trandi").create(MPU, 4);

  // the MPU notifies us everytime there's available data (50Hz as per the       configuration of DMP module)
  setWatch(pidLoop, A8, { repeat:true, edge:'falling' });  // should it be 'falling' ?
  
  console.log("DONE");
}



function forward(orBack) {
  digitalWrite(MR_DIR, orBack);
  digitalWrite(ML_DIR, orBack);
}


function disable(orEnable) {
  digitalWrite(MR_ENB, orEnable);
  digitalWrite(ML_ENB, orEnable);
}

function setSpeed(s) {
  if(s < 0){
    forward(false);
    s = -s;
  }else{
    forward(true);
  }
    
  disable(s < 10);
  
  if(s > MAX_SPEED) s = MAX_SPEED;
  else if(s === 0) s = 1;  // "freq: 0" won't work, whereas 1Hz is pretty much stopped
  
  analogWrite(MR_STP, 0.5, {freq: s});
  analogWrite(ML_STP, 0.5, {freq: s});
}


// frequency at which we'll toggle the step pin, proportional with the resulting RPM
var speed = 0;
var propErr = 0;
var intErr = 0;
var setPoint = 0;



function pidLoop() {
  var data = DMP.getData();
  
  if(data !== undefined) {
    var roll = DMP.getYawPitchRoll(data).roll + horizontalCoeff;
    var gyrox = data.gyrox;
  
    propErr = roll - setPoint;
    
    intErr = KI * intErr;
    if(intErr > MAX_INTEGRAL_ERR) intErr = MAX_INTEGRAL_ERR;
    else if(intErr < -MAX_INTEGRAL_ERR) intErr = -MAX_INTEGRAL_ERR;
    
    if(Math.abs(propErr) > 0.3) {
      speed = 0;
    } else {
      speed = KP * propErr + intErr + KD * gyrox;
      intErr = intErr + propErr;
    }
    
    console.log(roll + " - " + gyrox + " / " + speed);
    setSpeed(speed);
  }
}



