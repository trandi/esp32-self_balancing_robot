var MR_ENB = B10;
var MR_STP = A7;
var MR_DIR = B1;
var ML_ENB = B5;
var ML_STP = B4;
var ML_DIR = B3;


var KP = 2500;
var KI = 0;
var KD = 0;



var MPU;
var gyroOffsetX = 0;
var gyroOffsetY = 0;
var gyroOffsetZ = 0;
var calibCount = 0;
var calibIntId;
    
    
// empirical max frequency/speed when microstepping (1/32 step) with current limited @ 300mA for each motor ?
var MAX_SPEED = 20000;
var MAX_INTEGRAL_ERR = 5000;


function onInit() {
  console.log("START");
  disable(true);

  I2C1.setup({scl:B6, sda:B7, bitrate:100000});
  // "true" because I have AD0 connected to VCC
  MPU = require("MPU6050").connect(I2C1, true);

  MPU.setFullScaleAccelRange(0x02); // +- 8G range
  MPU.setFullScaleGyroRange(0x01); // 500degs / sec
  
  // calibration 
  calibCount = 0;
  calibIntId = setInterval(calibrate, 1);
}

function calibrate() {
  if(calibCount++ === 2000) {
    clearInterval(calibIntId);
    gyroOffsetX = gyroOffsetX / 2000;
    gyroOffsetY = gyroOffsetY / 2000;
    gyroOffsetZ = gyroOffsetZ / 2000;
    
    console.log("GYRO offsets [" + gyroOffsetX + ", " + gyroOffsetY + ", " + gyroOffsetZ + "]");
    
    // now go ahead and run
    setInterval(pidLoop, 20);
  }
  else{
    var rot = MPU.getRotation();
    gyroOffsetX += rot[0];
    gyroOffsetY += rot[1];
    gyroOffsetZ += rot[2];
  }
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

// 0.000305344 = 1 / (1000ms / 20ms interval) / 65.5
var GYRO_RAW_DEGS = 1 / (1000 / 20) / 65.5;
// PI radians / 180 degrees
var DEGS_RAD = 3.14159 / 180;

var rollAccOffset = -1.65;
var pitchAccOffset = -1.7;

// angles in degrees
var roll = 0;
var pitch = 0;
var lastRoll = roll; // to calculate the deriative term
function pidLoop() {
  lastRoll = roll;
  
  var accs = MPU.getAcceleration();
  var accX = accs[0];
  var accY = accs[1];
  var accZ = accs[2];
  
  var accTotalVector = Math.sqrt(accX*accX + accY*accY + accZ*accZ); // this actually should be constant = whatever raw value corresponds to 1G
  var rollAcc = Math.asin(accY / accTotalVector) / DEGS_RAD - rollAccOffset;  // asin() gives radians, result has to be in degrees
  var pitchAcc = Math.asin(accX / accTotalVector) / DEGS_RAD - pitchAccOffset;
  
  
  var gyros = MPU.getRotation();
  var gyroX = gyros[0] - gyroOffsetX;
  var gyroY = gyros[1] - gyroOffsetY;
  var gyroZ = gyros[2] - gyroOffsetZ;
  

  roll += gyroX * GYRO_RAW_DEGS;  
  pitch += gyroY * GYRO_RAW_DEGS;
  // sin() has to be applied on radians
  roll -= pitch * Math.sin(gyroZ * GYRO_RAW_DEGS * DEGS_RAD);
  pitch += roll * Math.sin(gyroZ * GYRO_RAW_DEGS * DEGS_RAD);
  
  
  // integrate accelerometre & gyro. Tiny weightto the noisy accelerometre, just enough to stop the gyro from drifting
  roll = 0.99 * roll + 0.01 * rollAcc;
  pitch = 0.99 * pitch + 0.01 * pitchAcc;

  propErr = roll - setPoint;

  intErr = KI * intErr;
  if(intErr > MAX_INTEGRAL_ERR) intErr = MAX_INTEGRAL_ERR;
  else if(intErr < -MAX_INTEGRAL_ERR) intErr = -MAX_INTEGRAL_ERR;

  if(Math.abs(propErr) > 30) {
    speed = 0;
  } else {
    speed = KP * propErr + intErr + KD * (roll - lastRoll);
    intErr = intErr + propErr;
  }

  
  console.log(roll + " - " + rollAcc + " / " + pitchAcc);
  //setSpeed(speed);
}



