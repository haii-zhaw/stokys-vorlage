/**
 * Register
 */
const REG_SERVO_TARGET_POSITION = 0x10;
const REG_SERVO_SETTLING_TIME = 0x18;
const REG_MOTOR_TARGET_POSITION = 0x20;
const REG_MOTOR_TARGET_VELOCITY = 0x24;
const REG_MOTOR_ACTUAL_POSITION = 0x28;
const REG_MOTOR_ACTUAL_VELOCITY = 0x2C;
const REG_MOTOR_MAX_VOLTAGE = 0x30;
const REG_MOTOR_CONTROL_MODE = 0x34;
const REG_SUPPLY_VOLTAGE = 0x60;
const REG_ANALOGIN_VOLTAGE = 0x61;

/**
 * Enums
 */
enum ServoMotors {
    Servo1 = 0,
    Servo2 = 1,
    Servo3 = 2,
    Servo4 = 3,
}

enum DCMotors {
    Motor1 = 0,
    Motor2 = 1,
    Motor3 = 2,
}

enum RobotMotors {
    MotorLinks = 0,
    MotorRechts = 2,
}

enum MotorControlMode {
    Position = 0,
    Velocity = 1,
}

enum AnalogInput {
    AnalogIn1 = 0,
    AnalogIn2 = 1,
}

enum DistanceSensor {
    Sensor1 = 0,
    Sensor2 = 1,
}

/**
 * Parameter
 */
const CONTROLLER_I2C_ADDRESS = 0x42;


/**
* Custom Blocks
*/
//% weight=201 color=#fc0303 icon="\uf085"
//% groups=['Kommunikation', 'Motoren', 'Servos', 'Inputs/Outputs']
namespace stokys {

    //*********************************************************************************************************************************************************************
    /**
     * DistanceSensor
     */
    //% blockId=distanceSensorBlock
    //% weight=39
    //% block="Distanz bei %id"
    //% group="Inputs/Outputs"
    //% inlineInputMode=inline
    export function distanceSensorBlock(id: DistanceSensor): number {
        let voltage = i2cRead(REG_ANALOGIN_VOLTAGE + id, 0x00, NumberFormat.UInt16LE);
        voltage = voltage / 1000.0;

        const sensorMaxVoltage = 3.1;
        const sensorMinVoltage = 0.2;

        let x = voltage;

        // Limit input range
        if (x > sensorMaxVoltage) {
            x = sensorMaxVoltage;
        }
        else if (x < sensorMinVoltage) {
            x = sensorMinVoltage;
        }

        // Evaluate polynomial fitted to the GP2Y0AF15X distance sensor
        let y = 0.0;
        y += x ** 0 * 0.618999;
        y += x ** 1 * -2.30135;
        y += x ** 2 * 4.133445;
        y += x ** 3 * -4.122716;
        y += x ** 4 * 2.404736;
        y += x ** 5 * -0.815516;
        y += x ** 6 * 0.148858;
        y += x ** 7 * -0.011302;

        // Limit output range to 20.. 200 mm
        if (y > 0.2) {
            y = 0.2;
        }
        else if (y < 0.02) {
            y = 0.02;
        }

        return y * 1000.0;
    }


    //*********************************************************************************************************************************************************************
    /**
     * AnalogInputs
     */
    //% blockId=analogInputBlock
    //% weight=40
    //% block="Spannung an %id"
    //% group="Inputs/Outputs"
    //% inlineInputMode=inline
    export function analogInputBlock(id: AnalogInput): number {
        let voltage = i2cRead(REG_ANALOGIN_VOLTAGE + id, 0x00, NumberFormat.UInt16LE);
        voltage = voltage / 1000.0;
        return voltage;
    }

    //*********************************************************************************************************************************************************************
    /**
     * Versorgungsspannung
     */
    //% blockId=voltageSupplyBlock
    //% weight=50
    //% block="Versorgungsspannung"
    //% group="Inputs/Outputs"
    //% inlineInputMode=inline
    export function voltageSupplyBlock(): number {
        let voltage = i2cRead(REG_SUPPLY_VOLTAGE, 0x00, NumberFormat.UInt16LE);
        voltage = voltage / 1000.0;
        return voltage;
    }

    //*********************************************************************************************************************************************************************
    /**
     * Roboter Warte bis Angehalten
     */
    //% blockId=robotAwaitStop
    //% weight=53
    //% block="Warte bis angehalten"
    //% group="Roboter"
    //% inlineInputMode=inline
    export function robotAwaitStop(): void {
        const threshold = 1; // rpm
        while (getRobotMaxSpeed() < threshold);
        while (getRobotMaxSpeed() > threshold);
        return
    }

    function getRobotMaxSpeed(): number {
        let speed_left = motorGetSpeedBlock(DCMotors.Motor1);
        let speed_right = motorGetSpeedBlock(DCMotors.Motor2);
        return Math.max(Math.abs(speed_left), Math.abs(speed_right))
    }

    //*********************************************************************************************************************************************************************
    /**
     * Roboter Positionsdelta Vorgeben
     * @param id Motor Nummer
     * @param delta Positions-Delta
     */
    //% blockId=robotPositionDeltaBlock
    //% weight=54
    //% block="Drehe %id für %delta Umdrehungen"
    //% group="Roboter"
    //% inlineInputMode=inline
    export function robotPositionDeltaBlock(id: RobotMotors, delta: number): void {
        if (id == RobotMotors.MotorLinks) {
            motorPositionDeltaBlock(DCMotors.Motor1, -1.0 * delta)
        } else if (id == RobotMotors.MotorRechts) {
            motorPositionDeltaBlock(DCMotors.Motor3, delta)
        }
    }

    //*********************************************************************************************************************************************************************
    /**
     * Roboter Geschwindigkeit
     * @param speed_left Geschwindigkeit links
     * @param speed_right Geschwindigkeit rechts
     */
    //% blockId=roboterSpeedBlock
    //% weight=55
    //% block="Roboter Geschwindigkeit links %speed_left und rechts %speed_right rpm"
    //% group="Roboter"
    //% inlineInputMode=inline
    export function roboterSpeedBlock(speed_left: number, speed_right: number): void {
        motorSpeedBlock(DCMotors.Motor1, -1.0 * speed_left)
        motorSpeedBlock(DCMotors.Motor3, speed_right)
    }

    //*********************************************************************************************************************************************************************
    /**
     * Motor Geschwindigkeit
     * @param id Motor Nummer
     * @param speed Geschwindigkeit
     */
    //% blockId=motorSpeedBlock
    //% weight=61
    //% block="bewege %id mit Geschwindigkeit %speed rpm"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorSpeedBlock(id: DCMotors, speed: number): void {
        speed = speed / 60.0 * 2.0 * Math.PI * 1000.0; // Convert from rpm to mrad/s
        i2cWrite(REG_MOTOR_TARGET_VELOCITY + id, 0, speed, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Velocity, NumberFormat.UInt8LE);
    }

    //*********************************************************************************************************************************************************************
    /**
     * Motor Geschwindigkeit Messen
     * @param id Motor Nummer
     */
    //% blockId=motorGetSpeedBlock
    //% weight=60
    //% block="Geschwindigkeit von %id"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorGetSpeedBlock(id: DCMotors): number {
        let speed = i2cRead(REG_MOTOR_ACTUAL_VELOCITY + id, 0x00, NumberFormat.Int32LE);
        speed = speed * (60.0 / (2.0 * Math.PI * 1000.0)); // Convert from mrad/s to rpm
        return speed;
    }

    //*********************************************************************************************************************************************************************
    /**
     * Motor Position Messen
     * @param id Motor Nummer
     * @param delta Positions-Delta
     */
    //% blockId=motorGetPositionBlock
    //% weight=57
    //% block="Position von %id"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorGetPositionBlock(id: DCMotors): number {
        let position = i2cRead(REG_MOTOR_ACTUAL_POSITION + id, 0x00, NumberFormat.Int32LE);
        position = position / (2.0 * Math.PI * 1000.0); // Convert mrad to rotations
        return position;
    }

    //*********************************************************************************************************************************************************************
    /**
     * Motor Positionsdelta Vorgeben
     * @param id Motor Nummer
     * @param delta Positions-Delta
     */
    //% blockId=motorPositionDeltaBlock
    //% weight=59
    //% block="%id um %delta Umdrehungen drehen"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorPositionDeltaBlock(id: DCMotors, delta: number): void {
        let position = i2cRead(REG_MOTOR_ACTUAL_POSITION + id, 0x00, NumberFormat.Int32LE);
        position = position + delta * 2.0 * Math.PI * 1000.0 // Convert to mrad
        i2cWrite(REG_MOTOR_TARGET_POSITION + id, 0, position, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Position, NumberFormat.UInt8LE);
    }

    //*********************************************************************************************************************************************************************
    /**
     * Motor Position Vorgeben
     * @param id Motor Nummer
     * @param position Position
     */
    //% blockId=motorPositionBlock
    //% weight=58
    //% block="bewege %id hin zu %position"
    //% group="Motoren"
    //% inlineInputMode=inline
    export function motorPositionBlock(id: DCMotors, position: number): void {
        position = position * 2.0 * Math.PI * 1000.0 // Convert to mrad
        i2cWrite(REG_MOTOR_TARGET_POSITION + id, 0, position, NumberFormat.Int32LE);
        i2cWrite(REG_MOTOR_CONTROL_MODE + id, 0x00, MotorControlMode.Position, NumberFormat.UInt8LE);
    }

    //*********************************************************************************************************************************************************************
    /**
     * Servo Geschwindigkeit
     * @param id Servo Nummer
     * @param time Nachstellzeit
     */
    //% blockId=servoSpeedBlock
    //% weight=79
    //% block="setze Geschwindigkeit von %id auf %time s"
    //% group="Servos"
    //% inlineInputMode=inline
    //% position.min=0 position.max=10
    export function servoSpeedBlock(id: ServoMotors, time: number): void {
        time = time * 1000; // convert to ms
        i2cWrite(REG_SERVO_SETTLING_TIME + id, 0, time, NumberFormat.UInt32LE);
    }

    //*********************************************************************************************************************************************************************
    /**
     * Servo Position
     * @param id Servo Nummer
     * @param position Sollposition
     */
    //% blockId=writeServoBlock
    //% weight=80
    //% block="bewege %id auf Position %position"
    //% group="Servos"
    //% inlineInputMode=inline
    //% position.min=0 position.max=180
    export function writeServoBlock(id: ServoMotors, position: number): void {
        position = position * 1000; // convert to mdeg
        i2cWrite(REG_SERVO_TARGET_POSITION + id, 0, position, NumberFormat.UInt32LE);
    }

    //*********************************************************************************************************************************************************************
    /**
     * Schreibe Nummer
     * @param index Index
     * @param subindex Subindex
     * @param value Wert
     * @param format Datentyp
     */
    //% blockId=i2cWrite
    //% weight=100
    //% block="schreibe Wert %value mit Datentyp %format nach %index:%subindex"
    //% group="Kommunikation"
    //% inlineInputMode=inline
    //% index.min=0 index.max=255
    //% subindex.min=0 index.max=255
    export function i2cWriteBlock(value: number, format: NumberFormat, index: number, subindex: number): void {
        i2cWrite(index, subindex, value, format);
    }

    //*********************************************************************************************************************************************************************
    /**
     * Lese Nummer
     * @param index Index
     * @param subindex Subindex
     * @param format Datentyp
     * @returns Nummer
     */
    //% blockId=i2cRead
    //% weight=99
    //% block="Wert aus %index:%subindex mit Dateityp %format"
    //% group="Kommunikation"
    //% inlineInputMode=inline
    //% index.min=0 index.max=255
    //% subindex.min=0 index.max=255
    export function i2cReadBlock(index: number, subindex: number, format: NumberFormat): number {
        return i2cRead(index, subindex, format)
    }

    //*********************************************************************************************************************************************************************

    function i2cWrite(index: number, subindex: number, value: number, format: NumberFormat): void {
        let payload: number[] = [index, subindex];
        let addressBuffer = Buffer.fromArray(payload);

        switch (format) {
            case NumberFormat.UInt32LE: { break };
            case NumberFormat.Int32LE: { break };
            case NumberFormat.UInt16LE: { break };
            case NumberFormat.Int16LE: { break };
            case NumberFormat.UInt8LE: { break };
            default: {
                control.fail("Unsupported value type");
            }
        }

        pins.i2cWriteBuffer(CONTROLLER_I2C_ADDRESS, addressBuffer, true);
        pins.i2cWriteNumber(CONTROLLER_I2C_ADDRESS, value, format);

        // let readback = i2cRead(index, subindex, format);
        // if (readback != value) {
        //     control.fail("I2C write error");
        // }
    }

    //*********************************************************************************************************************************************************************
    function i2cRead(index: number, subindex: number, format: NumberFormat): number {
        let payload: number[] = [index, subindex];
        let addressBuffer = Buffer.fromArray(payload);

        switch (format) {
            case NumberFormat.UInt32LE: { break };
            case NumberFormat.Int32LE: { break };
            case NumberFormat.UInt16LE: { break };
            case NumberFormat.Int16LE: { break };
            case NumberFormat.UInt8LE: { break };
            default: {
                control.fail("Unsupported value type");
            }
        }

        pins.i2cWriteBuffer(0x42, addressBuffer, true);
        let num = pins.i2cReadNumber(0x42, format);

        return num;
    }
}



