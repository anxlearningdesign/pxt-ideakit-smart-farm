//% color="#af1015" weight=200 block="IDEA KIT: Smart farm"
namespace IdeaKitSmartFarm {
  const PCA9685_ADDRESS = 0x40;
  const MODE1 = 0x00;
  const PRESCALE = 0xfe;
  const LED0_ON_L = 0x06;
  let initializedPCA9685 = false;

  export enum Motors {
    M1A = 0x1,
    M1B = 0x2,
    M2A = 0x3,
    M2B = 0x4,
  }

  export enum Servos {
    S1 = 0x01,
    S2 = 0x02,
    S3 = 0x03,
    S4 = 0x04,
    S5 = 0x05,
    S6 = 0x06,
    S7 = 0x07,
    S8 = 0x08,
  }
    
  let sonaTrack = [0,0,0,0,0,0,0,0,0,0]

  function initPCA9685(): void {
    i2cwrite(PCA9685_ADDRESS, MODE1, 0x00);
    setFreq(50);
    for (let idx = 0; idx < 16; idx++) {
      setPwm(idx, 0, 0);
    }
    initializedPCA9685 = true;
  }
  function setFreq(freq: number): void {
    // Constrain the frequency
    let prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
    let oldmode = i2cread(PCA9685_ADDRESS, MODE1);
    let newmode = (oldmode & 0x7f) | 0x10; // sleep
    i2cwrite(PCA9685_ADDRESS, MODE1, newmode); // go to sleep
    i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale); // set the prescaler
    i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
    control.waitMicros(5000);
    i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xa1);
  }
  function i2cread(addr: number, reg: number) {
    pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
    let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
    return val;
  }

  function i2cwrite(addr: number, reg: number, value: number) {
    let buf = pins.createBuffer(2);
    buf[0] = reg;
    buf[1] = value;
    pins.i2cWriteBuffer(addr, buf);
  }

  function setPwm(channel: number, on: number, off: number): void {
    if (channel < 0 || channel > 15) return;
    let buf = pins.createBuffer(5);
    buf[0] = LED0_ON_L + 4 * channel;
    buf[1] = on & 0xff;
    buf[2] = (on >> 8) & 0xff;
    buf[3] = off & 0xff;
    buf[4] = (off >> 8) & 0xff;
    pins.i2cWriteBuffer(PCA9685_ADDRESS, buf);
  }

  //% block="Motor run|%index|speed %speed"
  //% speed.min=-255 speed.max=255
  export function MotorRun(index: Motors, speed: number): void {
    if (!initializedPCA9685) {
      initPCA9685();
    }
    speed = speed * 16; // map 255 to 4096
    if (speed >= 4096) {
      speed = 4095;
    }
    if (speed <= -4096) {
      speed = -4095;
    }
    if (index > 4 || index <= 0) return;
    let pp = (index - 1) * 2;
    let pn = (index - 1) * 2 + 1;
    if (speed >= 0) {
      setPwm(pp, 0, speed);
      setPwm(pn, 0, 0);
    } else {
      setPwm(pp, 0, 0);
      setPwm(pn, 0, -speed);
    }
  }

  //% block="Ultrasonic distance (cm) trig %trig|echo %echo"
  export function sonarDistance(trig: DigitalPin, echo: DigitalPin): number {
    return sonar.ping(trig, echo, PingUnit.Centimeters);
  }

  //% block="Servo|%index|degree %degree"
  export function Servo(index: Servos, degree: number): void {
    if (!initializedPCA9685) {
      initPCA9685();
    }
    // 50hz: 20,000 us
    let v_us = (degree * 1800) / 180 + 600; // 0.6 ~ 2.4
    let value = (v_us * 4096) / 20000;
    setPwm(index + 7, 0, value);
  }

  //% block="DHT11 %dataPin"
  export function queryData(dataPin: DigitalPin) {
    //initialize
    let _temperature: number;
    let _readSuccessful: boolean = false;
    let _sensorresponding: boolean = false;

    const DHT = "DHT11";
    let startTime: number = 0;
    let endTime: number = 0;
    let checksum: number = 0;
    let checksumTmp: number = 0;
    let dataArray: boolean[] = [];
    let resultArray: number[] = [];

    for (let index = 0; index < 40; index++) dataArray.push(false);
    for (let index = 0; index < 5; index++) resultArray.push(0);

    _temperature = -999.0;
    _readSuccessful = false;
    _sensorresponding = false;
    startTime = input.runningTimeMicros();

    //request data
    pins.digitalWritePin(dataPin, 0); //begin protocol, pull down pin
    basic.pause(18);

    pins.setPull(dataPin, PinPullMode.PullUp);
    pins.digitalReadPin(dataPin); //pull up pin
    control.waitMicros(40);

    _sensorresponding = true;

    while (pins.digitalReadPin(dataPin) == 0); //sensor response
    while (pins.digitalReadPin(dataPin) == 1); //sensor response

    //read data (5 bytes)
    for (let index = 0; index < 40; index++) {
      while (pins.digitalReadPin(dataPin) == 1);
      while (pins.digitalReadPin(dataPin) == 0);
      control.waitMicros(28);
      //if sensor still pull up data pin after 28 us it means 1, otherwise 0
      if (pins.digitalReadPin(dataPin) == 1) dataArray[index] = true;
    }

    endTime = input.runningTimeMicros();

    //convert byte number array to integer
    for (let index = 0; index < 5; index++)
      for (let index2 = 0; index2 < 8; index2++)
        if (dataArray[8 * index + index2])
          resultArray[index] += 2 ** (7 - index2);

    //verify checksum
    checksumTmp =
      resultArray[0] + resultArray[1] + resultArray[2] + resultArray[3];
    checksum = resultArray[4];
    if (checksumTmp >= 512) checksumTmp -= 512;
    if (checksumTmp >= 256) checksumTmp -= 256;
    if (checksum == checksumTmp) _readSuccessful = true;

    //read data if checksum ok
    if (_readSuccessful) {
      if (DHT == "DHT11") {
        //DHT11
        _temperature = resultArray[2] + resultArray[3] / 100;
      } else {
        //DHT22
        let temp_sign: number = 1;
        if (resultArray[2] >= 128) {
          resultArray[2] -= 128;
          temp_sign = -1;
        }
        _temperature =
          ((resultArray[2] * 256 + resultArray[3]) / 10) * temp_sign;
      }
    }
    return _temperature;
  }
    //% block="Ultrasonic track loop trig %trig|echo %echo|distance(cm) %distance"
  export function sonarDistanceLoop(trig: DigitalPin, echo: DigitalPin,distance:number):boolean {
    const sona = sonar.ping(trig, echo, PingUnit.Centimeters);
    sonaTrack.shift()
    if (sona <= distance) {
      sonaTrack.push(1)
    } else {
      sonaTrack.push(0)
    }
    const sum = sonaTrack.reduce((accumulator, current) => {
      return accumulator + current;
    }, 0);    
    if (sum >= 5) {
      return true
    } else {
      return false
    }
  }
}
